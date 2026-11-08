#include "RagdollInternal.h"
#include <Jolt/RegisterTypes.h>
#include <Jolt/ConfigurationString.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/FPFlushDenormals.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayerInterfaceTable.h>
#include <Jolt/Physics/Collision/BroadPhase/ObjectVsBroadPhaseLayerFilterTable.h>
#include <Jolt/Physics/Collision/ObjectLayerPairFilterTable.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/BodyLock.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <mutex>
#include <unordered_map>
#include <vector>

// Ragdolls on death (humanoid enemies, and the player on generic deaths), simulated with Jolt Physics. See RAGDOLL_PLAN.md.
//
// - Jolt is set up at startup and checked with a smoke test (a sphere dropped onto a box must come to rest on it).
// - The physics system is created with the first ragdoll. Its world is the room's collision (RagdollWorld.cpp), and each
//   ragdoll is the 12-body humanoid of RagdollRig.cpp.
// - Hooks: the partsWorldCalc call in cEm10::move (every Ganado-family enemy) and the second PartsWorldPosCalc call in
//   cPlayer::move. Both run the game's own code first, then, for a ragdolling model, overwrite its parts with the
//   ragdoll's pose. Whatever the game does with the parts afterwards (the dissolve squash, weapons, caps, cloth, hair,
//   shadows) then uses that pose.
// - Physics steps once per game update, at the first hooked call of the update, in fixed 60Hz substeps, so it pauses
//   with the game.
// - The simulated parts take the bodies' transforms (with their scale kept). Every other part keeps the offset it had at
//   the start to the part it hangs off: its parent, or, for parts that duplicate a simulated one (same parent and
//   offset, like the shoulder twins 6/12), that simulated part.
//
// Enemy deaths: the update a Ganado-family enemy's hp goes from above 0 to 0 or below, it turns into a ragdoll right
// away (most kills first play a damage motion for a second or more before the death routine; that's replaced too),
// unless it's in a state that has to keep its animation (grabbing the player, on a vehicle, climbing, scripted, blown
// up, despawning). The killing hit pushes it: a share of the push to every body, the rest as an impulse at the hit.
// While it ragdolls, its root follows the pelvis (x/z, and y on the floor under it), so item drops, ground effects and
// shadows stay with the body. During the dissolve (Die_Lost) the game sinks the root by 6 per update; the ragdoll
// keeps simulating and is drawn that much lower, so it sinks with it. The ragdoll ends when the game hides the enemy,
// brings it back (hp above 0) or frees it.
//
// At most c_maxSimulated ragdolls simulate; past that the oldest freezes in place (no physics, same pose).
//
// Diagnostics (while ragdolls exist):
// - A ragdoll's pose is checked as its update starts: if anything changed it since we wrote it, that's logged (once a
//   second per model), since the game would have drawn that instead.
// - Updates slower than 50 ms are logged with where our time went (once a second). If the game runs below 2 fps for 5
//   updates in a row, every ragdoll stops and enemy death ragdolls are turned off, so a problem can't hang the game.

namespace ragdoll
{
	bool g_enemyDeaths = true;
	float g_deathPushScale = 1.0f;
	bool g_followRoot = true;

	namespace
	{
		// ---- Jolt's reporting ----

		void JoltTrace(const char* fmt, ...)
		{
			char buf[1024];
			va_list args;
			va_start(args, fmt);
			vsnprintf(buf, sizeof(buf), fmt, args);
			va_end(args);
			spd::log()->info("Jolt: {}", buf);
		}

#ifdef JPH_ENABLE_ASSERTS
		// Logs and carries on: returning true would hit a breakpoint, which crashes the game without a debugger attached
		bool JoltAssertFailed(const char* expr, const char* msg, const char* file, JPH::uint line)
		{
			spd::log()->error("Jolt assert: {}:{}: ({}) {}", file, line, expr, msg ? msg : "");
			return false;
		}
#endif

		bool s_joltReady = false;

		// The game runs on one thread and the debug UI on another: everything below that's shared (the ragdoll lists,
		// the world, every Jolt call) is only touched with this held. Recursive since the public calls use each other.
		std::recursive_mutex s_mutex;
		using Lock = std::lock_guard<std::recursive_mutex>;

		// ---- Physics system ----

		namespace BPLayers
		{
			const JPH::BroadPhaseLayer World(0);
			const JPH::BroadPhaseLayer Moving(1);
			constexpr JPH::uint Count = 2;
		}

		// The layer tables must outlive the physics systems using them
		struct LayerSetup
		{
			JPH::ObjectLayerPairFilterTable pairs{ Layers::Count };
			JPH::BroadPhaseLayerInterfaceTable bp{ Layers::Count, BPLayers::Count };
			JPH::ObjectVsBroadPhaseLayerFilterTable objVsBp;

			LayerSetup() : objVsBp(InitBp(), BPLayers::Count, InitPairs(), Layers::Count) {}

			const JPH::BroadPhaseLayerInterface& InitBp()
			{
				bp.MapObjectToBroadPhaseLayer(Layers::World, BPLayers::World);
				bp.MapObjectToBroadPhaseLayer(Layers::Ragdoll, BPLayers::Moving);
				return bp;
			}

			const JPH::ObjectLayerPairFilter& InitPairs()
			{
				pairs.EnableCollision(Layers::World, Layers::Ragdoll);
				pairs.EnableCollision(Layers::Ragdoll, Layers::Ragdoll);
				return pairs;
			}
		};

		LayerSetup* s_layers = nullptr;
		JPH::PhysicsSystem* s_system = nullptr;
		JPH::TempAllocatorImpl* s_temp = nullptr;
		JPH::JobSystemSingleThreaded* s_jobs = nullptr;

		bool EnsureSystem()
		{
			if (s_system)
				return true;
			if (!s_joltReady)
				return false;

			s_layers = new LayerSetup();
			s_temp = new JPH::TempAllocatorImpl(4 * 1024 * 1024);
			s_jobs = new JPH::JobSystemSingleThreaded(JPH::cMaxPhysicsJobs);
			s_system = new JPH::PhysicsSystem();
			s_system->Init(2048, 0, 4096, 4096, s_layers->bp, s_layers->objVsBp, s_layers->pairs);
			spd::log()->info("ragdoll -> physics system created");
			return true;
		}

		// The floor under a point (m), from the room's collision; false if there's none within `depth`
		bool FloorBelow(const JPH::Vec3& from, float depth, float& y)
		{
			if (!s_system)
				return false;
			JPH::RRayCast ray{ JPH::RVec3(from), JPH::Vec3(0.0f, -depth, 0.0f) };
			JPH::RayCastResult hit;
			if (!s_system->GetNarrowPhaseQuery().CastRay(ray, hit, JPH::SpecifiedBroadPhaseLayerFilter(BPLayers::World),
				JPH::SpecifiedObjectLayerFilter(Layers::World)))
				return false;
			y = float(ray.GetPointOnRay(hit.mFraction).GetY());
			return true;
		}

		// ---- Ragdolls ----

		struct StartInfo
		{
			Vec push;            // m/s, every body
			bool death = false;  // started by the enemy dying (its root follows the body)
			int impulsePart = -1;
			JPH::Vec3 impulsePoint = JPH::Vec3::sZero(); // m
			JPH::Vec3 impulse = JPH::Vec3::sZero();      // N s
		};

		struct Instance
		{
			cModel* model = nullptr;
			uint32_t guid = 0;
			float scale = 1.0f;
			bool em10 = false;  // posed through cEm10::move
			bool death = false; // see StartInfo
			uint32_t order = 0; // creation order, for freezing the oldest

			JPH::Ref<JPH::Ragdoll> ragdoll; // null once frozen
			std::vector<Aff> frozen;        // the pose it froze in

			bool inLost = false; // dissolving: drawn lower by how far the game sank the root since it started
			float lostRefY = 0.0f;

			// Diagnostics: where we last put the pelvis, to spot anything else changing the pose before it's drawn
			bool written = false;
			Vec writtenPelvis;
			uint32_t lastOverwriteLog = 0;

			std::vector<cParts*> parts;
			std::vector<int> body;   // per part: its ragdoll body, or -1
			std::vector<int> anchor; // per other part: the part it hangs off, or -1 (left alone)
			std::vector<Aff> offset; // anchor -> part, at the start
			std::vector<std::array<float, 3>> colScale; // per simulated part: its matrix's column lengths

			JPH::Vec3 capA[c_numBodies], capB[c_numBodies];
			float capR[c_numBodies] = {};
		};

		struct Pending
		{
			uint32_t guid;
			Vec push;
			uint32_t requestedFrame;
		};

		std::unordered_map<cModel*, Instance> s_instances;
		std::unordered_map<cModel*, Pending> s_pending;
		JPH::CollisionGroup::GroupID s_nextGroup = 1;
		uint32_t s_nextOrder = 1;
		uint32_t s_lastFrame = 0;
		float s_accum = 0.0f;

		// Diagnostics
		using Clock = std::chrono::steady_clock;
		Clock::time_point s_lastUpdateTime;
		bool s_haveLastUpdateTime = false;
		double s_msSync = 0, s_msStep = 0, s_msWrite = 0; // this update
		int s_stepsThisUpdate = 0;
		int s_slowStreak = 0;
		Clock::time_point s_lastPerfLog;

		double MsSince(Clock::time_point t)
		{
			return std::chrono::duration<double, std::milli>(Clock::now() - t).count();
		}

		// How much of the push each body gets (by c_bodyPart): more on the upper body than the legs, so a hit topples the
		// body over (the feet's friction holds them) instead of sliding it or dropping it on the spot
		constexpr float c_pushWeight[c_numBodies] = {
			0.8f,        // pelvis
			1.0f, 1.25f, // spine, chest
			1.25f,       // head
			1.1f, 1.1f,  // right upper / lower arm
			1.1f, 1.1f,  // left upper / lower arm
			0.55f, 0.3f, // right thigh / shin
			0.55f, 0.3f, // left thigh / shin
		};

		constexpr float c_maxStartSpeed = 6.0f; // m/s, what a part keeps of its animation's movement
		constexpr float c_maxStartUp = 3.0f;    // m/s, of which upwards

		constexpr float c_step = 1.0f / 60.0f;
		constexpr int c_maxSteps = 4;
		constexpr uint32_t c_pendingTimeout = 30; // updates

		void ReleaseBodies(Instance& in)
		{
			if (in.ragdoll)
			{
				in.ragdoll->RemoveFromPhysicsSystem();
				in.ragdoll = nullptr; // destroys its bodies and constraints
			}
		}

		bool ModelAlive(cModel* model, uint32_t guid)
		{
			return (model->be_flag_4 & UNIT_BE_FLAG_ALIVE) && model->guid_F8 == guid;
		}

		cEm10* AsEm10(cModel* model)
		{
			return reinterpret_cast<cEm10*>(model);
		}

		// Still something to ragdoll: alive, the same one, drawn, and (once dead) still dead
		bool InstanceValid(const Instance& in)
		{
			cModel* m = in.model;
			if (!ModelAlive(m, in.guid))
				return false;
			if (in.em10)
			{
				if (AsEm10(m)->Be_flg_4B0 & 0x400000) // hidden at the end of the dissolve
					return false;
				if (!(m->be_flag_4 & UNIT_BE_FLAG_TRANS))
					return false;
			}
			if (in.death && reinterpret_cast<cEm*>(m)->hp_324 > 0) // brought back (room script reuse, parasite)
				return false;
			return true;
		}

		void ComputePart(Instance& in, const JPH::Mat44* mats, std::vector<Aff>& world, std::vector<uint8_t>& state, int i)
		{
			if (state[i] == 2)
				return;
			if (state[i] == 1) // a loop in the anchors: leave it where the game put it
			{
				world[i] = AffFrom(in.parts[i]->mat_C);
				state[i] = 2;
				return;
			}
			state[i] = 1;

			if (in.body[i] >= 0)
				world[i] = JoltToGame(mats[in.body[i]], in.colScale[i].data());
			else if (in.anchor[i] >= 0)
			{
				ComputePart(in, mats, world, state, in.anchor[i]);
				world[i] = AffMul(world[in.anchor[i]], in.offset[i]);
			}
			else
				world[i] = AffFrom(in.parts[i]->mat_C);

			state[i] = 2;
		}

		// Every part's world matrix from the ragdoll (or the frozen pose)
		void ComputePose(Instance& in, std::vector<Aff>& world)
		{
			if (!in.ragdoll)
			{
				world = in.frozen;
				return;
			}

			JPH::RVec3 root;
			JPH::Mat44 mats[c_numBodies];
			in.ragdoll->GetPose(root, mats);
			for (JPH::Mat44& m : mats)
				m.SetTranslation(m.GetTranslation() + JPH::Vec3(root));

			const size_t n = in.parts.size();
			world.resize(n);
			std::vector<uint8_t> state(n, 0);
			for (size_t i = 0; i < n; i++)
				ComputePart(in, mats, world, state, int(i));
		}

		void Freeze(Instance& in)
		{
			if (!in.ragdoll)
				return;
			ComputePose(in, in.frozen);
			ReleaseBodies(in);
			spd::log()->info("ragdoll -> {} frozen (more than {} simulated)", (void*)in.model, c_maxSimulated);
		}

		int CountSimulated()
		{
			int n = 0;
			for (auto& kv : s_instances)
				n += kv.second.ragdoll ? 1 : 0;
			return n;
		}

		void FreezeOldestIfFull()
		{
			while (CountSimulated() >= c_maxSimulated)
			{
				Instance* oldest = nullptr;
				for (auto& kv : s_instances)
					if (kv.second.ragdoll && (!oldest || kv.second.order < oldest->order))
						oldest = &kv.second;
				if (!oldest)
					return;
				Freeze(*oldest);
			}
		}

		bool CreateInstance(cModel* model, const StartInfo& info, bool em10)
		{
			float scale;
			if (!Rig_Matches(model, scale) || !EnsureSystem())
				return false;

			FreezeOldestIfFull();

			const JPH::RagdollSettings* settings = Rig_Settings(scale);

			Instance in;
			in.model = model;
			in.guid = model->guid_F8;
			in.scale = scale;
			in.em10 = em10;
			in.death = info.death;
			in.order = s_nextOrder++;

			std::unordered_map<const cCoord*, int> index;
			for (cParts* p = model->childParts_F4; p; p = p->nextParts_F4)
			{
				index[p] = int(in.parts.size());
				in.parts.push_back(p);
			}

			const size_t n = in.parts.size();
			in.body.assign(n, -1);
			in.anchor.assign(n, -1);
			in.offset.resize(n);
			in.colScale.resize(n);
			for (int b = 0; b < c_numBodies; b++)
				in.body[c_bodyPart[b]] = b;

			std::vector<Aff> start(n);
			for (size_t i = 0; i < n; i++)
				start[i] = AffFrom(in.parts[i]->mat_C);

			for (size_t i = 0; i < n; i++)
			{
				if (in.body[i] >= 0)
				{
					for (int c = 0; c < 3; c++)
						in.colScale[i][c] = AffColumnLength(start[i], c);
					continue;
				}

				const cParts* p = in.parts[i];
				int anchor = -1;

				// A duplicate of a simulated part follows that part
				for (int b = 0; b < c_numBodies && anchor < 0; b++)
				{
					const cParts* s = in.parts[c_bodyPart[b]];
					const Vec d = p->pos_94 - s->pos_94;
					if (s->pParent_6C == p->pParent_6C && fabsf(d.x) + fabsf(d.y) + fabsf(d.z) < 0.01f)
						anchor = c_bodyPart[b];
				}

				if (anchor < 0)
				{
					auto it = index.find(p->pParent_6C);
					if (it != index.end())
						anchor = it->second;
				}

				Aff inv;
				if (anchor >= 0 && AffInverse(start[anchor], inv))
				{
					in.anchor[i] = anchor;
					in.offset[i] = AffMul(inv, start[i]);
				}
			}

			JPH::Mat44 mats[c_numBodies];
			for (int b = 0; b < c_numBodies; b++)
				mats[b] = RigidToJolt(start[c_bodyPart[b]]);

			in.ragdoll = settings->CreateRagdoll(s_nextGroup++, JPH::uint64(uintptr_t(model)), s_system);
			if (!in.ragdoll)
			{
				spd::log()->error("ragdoll -> out of bodies");
				return false;
			}
			in.ragdoll->SetPose(JPH::RVec3::sZero(), mats);
			in.ragdoll->AddToPhysicsSystem(JPH::EActivation::Activate);

			// Keep going the way the parts were moving (world_old / world_old2: where they were at the end of the last two
			// updates). Of this update's movement and the one before, the slower: the update an enemy dies in often snaps
			// to a new animation (a hit reaction or death motion starting from a different pose, a melee launch), which
			// would read as tens of m/s. Then no faster than a falling body moves, and not much upwards.
			const float dt = GlobalPtr()->deltaTime_70 / 30.0f;
			JPH::BodyInterface& bi = s_system->GetBodyInterface();
			const JPH::Vec3 pushV(info.push.x, info.push.y, info.push.z);
			float rawPelvis = 0.0f, rawMax = 0.0f, usedPelvis = 0.0f;
			for (int b = 0; b < c_numBodies; b++)
			{
				const cParts* p = in.parts[c_bodyPart[b]];
				JPH::Vec3 v = JPH::Vec3::sZero();
				if (dt > 0.0f)
				{
					const Vec d1 = p->world_70 - p->world_old_7C;
					const Vec d0 = p->world_old_7C - p->world_old2_88;
					const JPH::Vec3 v1 = JPH::Vec3(d1.x, d1.y, d1.z) * (c_toJolt / dt);
					const JPH::Vec3 v0 = JPH::Vec3(d0.x, d0.y, d0.z) * (c_toJolt / dt);
					v = v0.LengthSq() < v1.LengthSq() ? v0 : v1;
					if (!v.IsNaN() && v.Length() > c_maxStartSpeed)
						v = v.Normalized() * c_maxStartSpeed;
					if (v.IsNaN())
						v = JPH::Vec3::sZero();
					v.SetY(std::min(v.GetY(), c_maxStartUp));

					rawMax = std::max(rawMax, v1.Length());
					if (b == 0)
					{
						rawPelvis = v1.Length();
						usedPelvis = v.Length();
					}
				}
				bi.SetLinearVelocity(in.ragdoll->GetBodyID(b), v + pushV * c_pushWeight[b]);

				Rig_Capsule(b, scale, in.capA[b], in.capB[b], in.capR[b]);
			}

			// The hit itself: on the body of the part that was hit (or the closest simulated part above it)
			if (info.impulsePart >= 0 && info.impulsePart < int(n) && !info.impulse.IsNearZero())
			{
				int part = info.impulsePart;
				for (int guard = 0; part >= 0 && in.body[part] < 0 && guard < 64; guard++)
				{
					auto it = index.find(in.parts[part]->pParent_6C);
					part = it != index.end() ? it->second : -1;
				}
				const int b = part >= 0 ? in.body[part] : 0;
				const JPH::BodyID id = in.ragdoll->GetBodyID(b);

				// Sized for the whole body, but it lands on one piece: no more than 8 m/s on that piece (a forearm hit
				// would otherwise start it at ~40 m/s)
				JPH::Vec3 impulse = info.impulse;
				float mass = 0.0f;
				{
					JPH::BodyLockRead lock(s_system->GetBodyLockInterface(), id);
					if (lock.Succeeded() && lock.GetBody().GetMotionProperties()->GetInverseMass() > 0.0f)
						mass = 1.0f / lock.GetBody().GetMotionProperties()->GetInverseMass();
				}
				const float maxImpulse = mass * 8.0f;
				if (impulse.Length() > maxImpulse)
					impulse = impulse.NormalizedOr(JPH::Vec3::sZero()) * maxImpulse;
				bi.AddImpulse(id, impulse, JPH::RVec3(info.impulsePoint));
			}

			spd::log()->info("ragdoll -> started on {} (id 0x{:02X}, scale {:.2f}, {} parts{}); pelvis moving {:.1f} m/s this update, "
				"{:.1f} used, fastest part {:.1f}; push {:.1f} m/s", (void*)model, int(model->id_100), scale, n, info.death ? ", death" : "",
				rawPelvis, usedPelvis, rawMax, pushV.Length());
			s_instances[model] = std::move(in);
			return true;
		}

		// A dead enemy's root follows its body: x/z of the pelvis, y on the floor under it.
		//
		// Only in the death routines (r_no_0 3), which stay on the ground. The damage routines a kill usually goes through
		// first (Dm_Blow, Dm_KnockOut, Dm_Roof, ...) move the root themselves, with their own speed and gravity, until it
		// lands: putting it back under the pelvis every update means it never lands, the speed keeps growing, and the root
		// ends up kilometres away (the game then sweeps its collision across the whole map, taking seconds per update).
		void FollowRoot(Instance& in, const std::vector<Aff>& world)
		{
			cModel* m = in.model;
			if (m->r_no_0_FC != 3)
				return;

			const Aff& pelvis = world[c_bodyPart[0]];
			Vec pos(pelvis.m[0][3], m->pos_94.y, pelvis.m[2][3]);

			// Never a jump the body couldn't have made (or a broken value)
			const float dx = pos.x - m->pos_94.x, dz = pos.z - m->pos_94.z;
			if (!std::isfinite(pos.x) || !std::isfinite(pos.z) || dx * dx + dz * dz > 20000.0f * 20000.0f)
				return;

			float floorY;
			const JPH::Vec3 from(pelvis.m[0][3] * c_toJolt, pelvis.m[1][3] * c_toJolt + 0.2f, pelvis.m[2][3] * c_toJolt);
			if (FloorBelow(from, 3.0f, floorY))
				pos.y = floorY * c_toGame;

			m->pos_94 = pos;
			m->mat_C[0][3] = pos.x;
			m->mat_C[1][3] = pos.y;
			m->mat_C[2][3] = pos.z;
			m->Motion_1D8.Pos_world_9C = pos; // or PartsWorldPosCalc moves the parts by the change
			m->Fix_parts_13C = 0;             // no foot lock moving the root (and the parts) afterwards
		}

		void WritePose(Instance& in)
		{
			cModel* m = in.model;

			// The dissolve sinks the root; the ragdoll sinks with it
			const bool lost = in.em10 && m->r_no_0_FC == 3 && m->r_no_1_FD == 3;
			if (lost && !in.inLost)
			{
				in.inLost = true;
				in.lostRefY = m->pos_94.y;
			}
			const float sink = in.inLost ? m->pos_94.y - in.lostRefY : 0.0f;

			std::vector<Aff> world;
			ComputePose(in, world);

			if (in.death && in.ragdoll && !in.inLost && g_followRoot)
				FollowRoot(in, world);

			// A dead one stops colliding with characters and objects (the game does the same once a corpse lies still,
			// Die_Cramp). Otherwise, right after this, EmAtCheck pushes the root out of whatever the body slid into (another
			// enemy, a crate's or ladder's box, metres to its edge) and moves every part along with it: that's what gets
			// drawn, flickering between our pose and the pushed one.
			if (in.death && in.em10)
				m->atari_2B4.m_flag_1A &= ~0x200;

			for (size_t i = 0; i < in.parts.size(); i++)
			{
				if (in.ragdoll && in.body[i] < 0 && in.anchor[i] < 0)
					continue;
				cParts* p = in.parts[i];
				for (int r = 0; r < 3; r++)
					for (int c = 0; c < 4; c++)
						p->mat_C[r][c] = world[i].m[r][c];
				p->mat_C[1][3] += sink;
				p->world_70 = Vec(p->mat_C[0][3], p->mat_C[1][3], p->mat_C[2][3]);
			}

			in.written = true;
			in.writtenPelvis = in.parts[c_bodyPart[0]]->world_70;
		}

		// At the start of an update, before a ragdolling model has moved: is its pelvis still where we put it last update?
		// If not, something changed the pose after our write, and that's what was drawn.
		void CheckOverwritten(Instance& in)
		{
			if (!in.written)
				return;
			cModel* model = in.model;
			const cParts* pelvis = in.parts[c_bodyPart[0]];
			const Vec d = pelvis->world_70 - in.writtenPelvis;
			const float dist = sqrtf(d.x * d.x + d.y * d.y + d.z * d.z);
			const uint32_t frame = GlobalPtr()->frameCounter_530C;
			if (dist > 30.0f && frame - in.lastOverwriteLog > 60)
			{
				in.lastOverwriteLog = frame;
				spd::log()->warn("ragdoll -> {}'s pose was changed by {:.0f} units after we wrote it (routine {:02X}-{:02X}-{:02X}-{:02X}, motion attr {:04X})",
					(void*)model, dist, model->r_no_0_FC, model->r_no_1_FD, model->r_no_2_FE, model->r_no_3_FF, model->Motion_1D8.Mot_attr_40);
			}
		}

		void StopEverything(); // below

		// How long the last update took (wall clock), and what of it was ours; bails out if the game crawls
		void Watchdog(const Clock::time_point& now)
		{
			const double frameMs = s_haveLastUpdateTime ? std::chrono::duration<double, std::milli>(now - s_lastUpdateTime).count() : 0.0;
			s_lastUpdateTime = now;
			s_haveLastUpdateTime = true;

			if (s_instances.empty())
			{
				s_slowStreak = 0;
				return;
			}

			if (frameMs > 50.0 && std::chrono::duration<double>(now - s_lastPerfLog).count() > 1.0)
			{
				s_lastPerfLog = now;
				spd::log()->warn("ragdoll -> slow update: {:.1f} ms, ours {:.2f} ms (world sync {:.2f}, physics {:.2f} in {} steps, poses {:.2f}); "
					"{} ragdolls ({} simulated), {} active bodies, {} room pieces ({} rebuilt so far)",
					frameMs, s_msSync + s_msStep + s_msWrite, s_msSync, s_msStep, s_stepsThisUpdate, s_msWrite,
					s_instances.size(), CountSimulated(), s_system ? s_system->GetNumActiveBodies(JPH::EBodyType::RigidBody) : 0,
					World_BodyCount(), World_RebuildCount());
			}

			s_slowStreak = frameMs > 500.0 ? s_slowStreak + 1 : 0;
			if (s_slowStreak >= 5)
			{
				spd::log()->error("ragdoll -> the game has been below 2 fps for 5 updates with ragdolls out: stopping them all and turning "
					"enemy death ragdolls off");
				StopEverything();
				g_enemyDeaths = false;
				s_slowStreak = 0;
			}
		}

		// Once per game update: drop what's gone, then step. `current` is the model being posed (already re-posed by the
		// game this update); every other ragdoll is still as it was drawn.
		void UpdateFrame(cModel* current)
		{
			GLOBAL_WK* pG = GlobalPtr();
			const uint32_t frame = pG->frameCounter_530C;
			if (frame == s_lastFrame)
				return;
			// Frames went by without an update of ours (paused, a menu): not an update that took that long
			if (frame - s_lastFrame != 1)
				s_haveLastUpdateTime = false;
			s_lastFrame = frame;

			Watchdog(Clock::now());
			const double lastStepMs = s_msStep;
			s_msSync = s_msStep = s_msWrite = 0;
			s_stepsThisUpdate = 0;

			for (auto it = s_instances.begin(); it != s_instances.end();)
			{
				if (!InstanceValid(it->second))
				{
					spd::log()->info("ragdoll -> {} ended", (void*)it->first);
					ReleaseBodies(it->second);
					it = s_instances.erase(it);
				}
				else
					++it;
			}

			for (auto& kv : s_instances)
				if (kv.first != current)
					CheckOverwritten(kv.second);

			for (auto it = s_pending.begin(); it != s_pending.end();)
			{
				if (frame - it->second.requestedFrame > c_pendingTimeout)
				{
					spd::log()->warn("ragdoll -> {} was never posed through a hooked call, not started", (void*)it->first);
					it = s_pending.erase(it);
				}
				else
					++it;
			}

			if (s_instances.empty() || !s_system)
				return;

			float dt = pG->deltaTime_70 / 30.0f;
			if (!(dt > 0.0f))
				return;
			dt = std::min(dt, 0.1f);

			Clock::time_point t0 = Clock::now();
			World_Sync(dt);
			s_msSync = MsSince(t0);

			// After a physics update over 10 ms, one step at most (dropping the rest of the time), so a slow update
			// can't make the next one slower still
			const int maxSteps = lastStepMs > 10.0 ? 1 : c_maxSteps;

			s_accum += dt;
			int steps = int(s_accum / c_step);
			if (steps > maxSteps)
			{
				steps = maxSteps;
				s_accum = 0.0f;
			}
			else
				s_accum -= steps * c_step;

			if (steps > 0)
			{
				t0 = Clock::now();
				s_system->Update(steps * c_step, steps, s_temp, s_jobs);
				s_msStep = MsSince(t0);
				s_stepsThisUpdate = steps;
			}
		}

		// ---- Enemy deaths ----

		struct Track
		{
			uint32_t guid = 0;
			int16_t hp = 0;
			uint8_t r0 = 0, r1 = 0;
		};
		std::unordered_map<cModel*, Track> s_track; // every Ganado-family enemy, as of its last update

		// Routines that keep their animation when the enemy dies in them (r_no_1 per r_no_0; see RAGDOLL_PLAN.md)
		bool KeepsAnimation(uint8_t r0, uint8_t r1)
		{
			switch (r0)
			{
			case 1: // Move
				switch (r1)
				{
				case 0x06: // R100Cliff
				case 0x09: // R101Cart
				case 0x0A: case 0x0B: // EvtDash, EvtWalk
				case 0x23: // ShotGatling
				case 0x2E: // ClawCriHit (killing the player)
				case 0x30: case 0x32: // C_SawHit, C_SawCriHit (killing the player)
				case 0x33: case 0x34: case 0x35: case 0x36: case 0x37: case 0x38: case 0x39: // Catch, NeckHang (Leon, Luis, Ashley), Backhold, Bombhold, DashCatch
				case 0x3A: // TakeAway (carrying Ashley)
				case 0x3C: // ClimbOver
				case 0x40: case 0x41: case 0x42: // LadderClimb, VLadderClimb, LadderReset
				case 0x47: // Drive
				case 0x48: case 0x49: // Catapult, RockPush
				case 0x4A: case 0x4B: // R10CParasite, R10CPCancel
				case 0x50: // FixBomber
				case 0x52: case 0x53: case 0x54: // R222Dragon A/B/C
				case 0x55: // R227Barrel
				case 0x57: // R10FGondola
				case 0x5A: case 0x5B: // R11DAppear1/2
				case 0x5C: // R212Drill
				case 0x5E: // R201EventWait
				case 0x5F: // R209Gatling
				case 0x61: case 0x62: // R21BTrolleyJump 1/2
				case 0x63: // R303FireDash
				case 0x65: // UFOCatch
				case 0x66: // R300TakeAshley
				case 0x67: // R30FBullJump
				case 0x68: case 0x69: // R320Gatling, R300Gatling
				case 0x6A: case 0x6C: // R305Bomber, R408Bomber
				case 0x6B: // R321DeadBody
					return true;
				}
				return false;
			case 2: // Damage
				switch (r1)
				{
				case 0x05: // Dm_Fence
				case 0x06: // Dm_Ladder
				case 0x0C: // Dm_TakeAway
				case 0x10: // Dm_FS (suplex)
				case 0x11: // Dm_Gatling
				case 0x15: // Dm_NeckBreak
					return true;
				}
				return false;
			case 3: // Die
				return r1 == 3 || r1 == 5; // Die_Lost (despawn), Die_Bomb (gibbed)
			case 4: // Scenario
				return true;
			}
			return false;
		}

		// How hard the killing hit pushes (m/s), by em10's weapon groups (em10DmSetWep_tbl)
		float WeaponPush(int wep, float distSq)
		{
			switch (wep)
			{
			case 0x07: case 0x08: case 0x21: // shotguns: a near hit blows them away
				return distSq < 36e6f ? 7.5f : 3.5f;
			case 0x05: case 0x06: case 0x09: case 0x0A: case 0x0D: case 0x0F: case 0x12: case 0x13: case 0x1C:
			case 0x29: case 0x2C: case 0x2D: // rifles, magnum, rocket, blasts
				return 6.0f;
			case 0x22: case 0x23: case 0x24: case 0x25: // kicks, suplex
				return 2.5f;
			case 0x00: case 0x14: // hand, knife
				return 1.2f;
			case 0x17: case 0x2A: // flash
				return 0.3f;
			default: // bullets
				return 3.5f;
			}
		}

		// Where the hit box was hit (a simpler EmGetDmPos: the game's hit point when it recorded one, else the middle of
		// the box), and the part it belongs to
		bool HitPoint(cEm* em, Vec& point, int& part)
		{
			YARARE_INFO* y = em->m_DmgInfo_328.m_pDamageYarare_18;
			// Only trust it if it's one of this enemy's own hit boxes
			const uintptr_t lo = uintptr_t(em), hi = lo + sizeof(cEm10);
			if (!y || uintptr_t(y) < lo || uintptr_t(y) + sizeof(YARARE_INFO) > hi)
				return false;

			part = y->parts_no_26 - 1; // 1-based, 0 = the model itself
			if (y->flag_24 & 0x4000) // YAT_FLAG_DMPOS: the exact point
			{
				point = y->cross_C;
				return true;
			}

			const cCoord* c = em;
			if (part >= 0)
			{
				const cParts* p = em->childParts_F4;
				for (int i = 0; p && i < part; i++)
					p = p->nextParts_F4;
				if (!p)
					return false;
				c = p;
			}

			Vec mid = y->offset_0;
			if (y->flag_24 & 2) // YAT_FLAG_X_AXIS
				mid.x += 0.5f * y->height_1C;
			else if (y->flag_24 & 4) // YAT_FLAG_Z_AXIS
				mid.z += 0.5f * y->height_1C;
			else
				mid.y += 0.5f * y->height_1C;

			const Mtx& m = c->mat_C;
			point = Vec(m[0][0] * mid.x + m[0][1] * mid.y + m[0][2] * mid.z + m[0][3],
				m[1][0] * mid.x + m[1][1] * mid.y + m[1][2] * mid.z + m[1][3],
				m[2][0] * mid.x + m[2][1] * mid.y + m[2][2] * mid.z + m[2][3]);
			return true;
		}

		void StartDeath(cModel* model, const Track& prev)
		{
			cEm* em = reinterpret_cast<cEm*>(model);
			const uint8_t r0 = model->r_no_0_FC, r1 = model->r_no_1_FD;

			if (KeepsAnimation(r0, r1) || KeepsAnimation(prev.r0, prev.r1))
			{
				spd::log()->info("ragdoll -> {} died in routine {:02X}-{:02X} (was {:02X}-{:02X}): keeps its animation", (void*)model,
					r0, r1, prev.r0, prev.r1);
				return;
			}
			if (AsEm10(model)->Be_flg_4B0 & 0x30000) // on a ladder / fence
			{
				spd::log()->info("ragdoll -> {} died climbing: keeps its animation", (void*)model);
				return;
			}
			if (!(model->be_flag_4 & UNIT_BE_FLAG_TRANS))
				return;

			float scale;
			if (!Rig_Matches(model, scale))
			{
				spd::log()->info("ragdoll -> {} (id 0x{:02X}) died, but isn't the humanoid skeleton", (void*)model, int(model->id_100));
				return;
			}

			StartInfo info;
			info.death = true;

			// The push: from where the shot came from through where it hit
			Vec point;
			int part = -1;
			const cDmgInfo& dmg = em->m_DmgInfo_328;
			Vec dir;
			bool haveDir = false;
			if (HitPoint(em, point, part))
			{
				dir = point - dmg.m_PosFrom_8;
				dir.y *= 0.3f; // mostly sideways, the game's shots come from a little above
				const float len = sqrtf(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z);
				if (len > 1.0f && len < 50000.0f)
				{
					dir = dir / len;
					haveDir = true;
				}
			}
			if (!haveDir)
			{
				// No usable hit: away from the player
				cModel* pl = (cModel*)PlayerPtr();
				dir = pl ? model->pos_94 - pl->pos_94 : Vec(0, 0, 1);
				dir.y = 0.0f;
				const float len = sqrtf(dir.x * dir.x + dir.z * dir.z);
				dir = len > 1.0f ? dir / len : Vec(0, 0, 1);
				part = -1;
			}

			const float speed = WeaponPush(dmg.m_Wep_6, dmg.m_Dist_mb_14) * g_deathPushScale;
			const Vec up(0.0f, 0.1f * speed, 0.0f);
			spd::log()->info("ragdoll -> {} died in routine {:02X}-{:02X} (was {:02X}-{:02X}), weapon 0x{:02X}, hit part {}{}, push {:.1f} m/s "
				"along ({:.2f}, {:.2f}, {:.2f})", (void*)model, r0, r1, prev.r0, prev.r1, int(dmg.m_Wep_6), part,
				haveDir ? "" : " (no hit point)", speed, dir.x, dir.y, dir.z);

			// 75% as a push on every body (weighted towards the upper body), 25% as an impulse where it was hit (69 kg at
			// scale 1)
			info.push = dir * (0.75f * speed) + up;
			if (part >= 0 || haveDir)
			{
				const float mass = 69.0f * scale * scale * scale;
				info.impulsePart = part >= 0 ? part : 0;
				info.impulsePoint = JPH::Vec3(point.x, point.y, point.z) * c_toJolt;
				info.impulse = JPH::Vec3(dir.x, dir.y, dir.z) * (0.25f * speed * mass);
			}

			if (!CreateInstance(model, info, true))
				spd::log()->warn("ragdoll -> couldn't start on {} as it died", (void*)model);
		}

		// Called for every Ganado-family enemy each update (as it's posed): spots the update its hp runs out
		void TrackEm10(cModel* model)
		{
			cEm* em = reinterpret_cast<cEm*>(model);
			Track& t = s_track[model];
			const Track prev = t;
			const bool same = prev.guid == model->guid_F8;

			t.guid = model->guid_F8;
			t.hp = em->hp_324;
			t.r0 = model->r_no_0_FC;
			t.r1 = model->r_no_1_FD;

			if (same && prev.hp > 0 && t.hp <= 0 && g_enemyDeaths && s_joltReady && !s_instances.count(model) && !s_pending.count(model))
				StartDeath(model, prev);

			// Enemies come and go with rooms
			if (s_track.size() > 256)
			{
				for (auto it = s_track.begin(); it != s_track.end();)
				{
					if (!ModelAlive(it->first, it->second.guid))
						it = s_track.erase(it);
					else
						++it;
				}
			}
		}

		// After the game posed a hooked model
		void AfterPose(cModel* model, bool em10)
		{
			// Denormals flushed to zero while we run (as Jolt's own worker threads do): settling bodies decay their
			// velocities towards zero, and SSE arithmetic on denormals is two orders of magnitude slower. The game's
			// setting comes back when this returns.
			JPH::FPFlushDenormals flushDenormals;

			if (em10)
				TrackEm10(model);

			if (s_instances.empty() && s_pending.empty())
			{
				s_haveLastUpdateTime = false; // the watchdog times updates with ragdolls out, not the gaps between them
				return;
			}

			UpdateFrame(model);

			auto pend = s_pending.find(model);
			if (pend != s_pending.end())
			{
				Pending p = pend->second;
				s_pending.erase(pend);
				StartInfo info;
				info.push = p.push;
				if (model->guid_F8 == p.guid && !CreateInstance(model, info, em10))
					spd::log()->warn("ragdoll -> couldn't start on {}", (void*)model);
				return; // already in the pose it starts from
			}

			auto it = s_instances.find(model);
			if (it != s_instances.end())
			{
				if (InstanceValid(it->second))
				{
					const Clock::time_point t0 = Clock::now();
					WritePose(it->second);
					s_msWrite += MsSince(t0);
				}
				else
				{
					ReleaseBodies(it->second);
					s_instances.erase(it);
				}
			}
		}

		void StopEverything()
		{
			s_pending.clear();
			for (auto& kv : s_instances)
				ReleaseBodies(kv.second);
			s_instances.clear();
		}

		// ---- Hooks ----

		uint8_t* s_em10PartsWorldCalc = nullptr; // cEm10::move: the partsWorldCalc call
		uint8_t* s_plPartsWorldPosCalc = nullptr; // cPlayer::move: the second PartsWorldPosCalc call

		void(__fastcall* cEm10_partsWorldCalc_orig)(cModel* thisptr, void* unused);
		void __fastcall cEm10_partsWorldCalc_hook(cModel* thisptr, void* unused)
		{
			cEm10_partsWorldCalc_orig(thisptr, unused);
			Lock lock(s_mutex);
			AfterPose(thisptr, true);
		}

		void(__cdecl* cPlayer_PartsWorldPosCalc_orig)(cModel* pMod);
		void __cdecl cPlayer_PartsWorldPosCalc_hook(cModel* pMod)
		{
			cPlayer_PartsWorldPosCalc_orig(pMod);
			Lock lock(s_mutex);
			AfterPose(pMod, false);
		}

		void FindHookPoints()
		{
			// cEm10::move: partsWorldCalc, em10ScaleCompress, em10BombNeckMove, partsFixAdjust, PartsWorldPosCalc. The
			// ragdoll pose goes in right after partsWorldCalc (+2).
			auto pattern = hook::pattern("8B CE E8 ? ? ? ? 8B CE E8 ? ? ? ? E8 ? ? ? ? 8B CE E8 ? ? ? ? 56 E8");
			if (pattern.size() == 1)
				s_em10PartsWorldCalc = pattern.get_first<uint8_t>(2);

			// cPlayer::move: the second PartsWorldPosCalc (+1), right before Wep->move and moveCloth
			pattern = hook::pattern("56 E8 ? ? ? ? 8B 8E D8 07 00 00 83 C4 04 E8");
			if (pattern.size() == 1)
				s_plPartsWorldPosCalc = pattern.get_first<uint8_t>(1);

			spd::log()->info("ragdoll -> cEm10::move partsWorldCalc call: {}, cPlayer::move PartsWorldPosCalc call: {}",
				(void*)s_em10PartsWorldCalc, (void*)s_plPartsWorldPosCalc);
			if (!s_em10PartsWorldCalc || !s_plPartsWorldPosCalc)
				spd::log()->error("ragdoll -> hook point not found");
		}

		void InstallHooks()
		{
			if (s_em10PartsWorldCalc)
			{
				ReadCall(s_em10PartsWorldCalc, cEm10_partsWorldCalc_orig);
				InjectHook(s_em10PartsWorldCalc, cEm10_partsWorldCalc_hook, HookType::Call);
			}
			if (s_plPartsWorldPosCalc)
			{
				ReadCall(s_plPartsWorldPosCalc, cPlayer_PartsWorldPosCalc_orig);
				InjectHook(s_plPartsWorldPosCalc, cPlayer_PartsWorldPosCalc_hook, HookType::Call);
			}
		}

		// ---- Smoke test ----

		bool SmokeTest()
		{
			using namespace JPH;

			LayerSetup layers;
			TempAllocatorImpl tempAllocator(1024 * 1024);
			JobSystemSingleThreaded jobSystem(cMaxPhysicsJobs);

			PhysicsSystem system;
			system.Init(64, 0, 64, 64, layers.bp, layers.objVsBp, layers.pairs);

			BodyInterface& bi = system.GetBodyInterface();

			// 10 x 1 x 10 m floor, top at y = 0
			BodyID floor = bi.CreateAndAddBody(BodyCreationSettings(new BoxShape(Vec3(5.0f, 0.5f, 5.0f)), RVec3(0.0f, -0.5f, 0.0f),
				Quat::sIdentity(), EMotionType::Static, Layers::World), EActivation::DontActivate);

			// 0.25 m sphere, dropped from 2 m
			const float radius = 0.25f;
			BodyID ball = bi.CreateAndAddBody(BodyCreationSettings(new SphereShape(radius), RVec3(0.0f, 2.0f, 0.0f),
				Quat::sIdentity(), EMotionType::Dynamic, Layers::Ragdoll), EActivation::Activate);

			// 3 s at 60 Hz
			for (int i = 0; i < 180; i++)
				system.Update(1.0f / 60.0f, 1, &tempAllocator, &jobSystem);

			RVec3 pos = bi.GetCenterOfMassPosition(ball);
			bool asleep = !bi.IsActive(ball);

			bi.RemoveBody(ball);
			bi.DestroyBody(ball);
			bi.RemoveBody(floor);
			bi.DestroyBody(floor);

			// Resting contacts are allowed to sink in by up to the penetration slop (2 cm by default)
			bool ok = fabsf(float(pos.GetY()) - radius) <= system.GetPhysicsSettings().mPenetrationSlop + 0.005f;
			spd::log()->info("ragdoll -> smoke test: sphere settled at y = {:.4f} (expected {:.4f}), {}: {}", float(pos.GetY()), radius,
				asleep ? "asleep" : "still awake", ok ? "OK" : "FAILED");
			return ok;
		}
	}

	// ---- Shared ----

	JPH::PhysicsSystem* System()
	{
		return s_system;
	}

	JPH::Mat44 RigidToJolt(const Aff& a)
	{
		using namespace JPH;
		Vec3 x(a.m[0][0], a.m[1][0], a.m[2][0]);
		Vec3 y(a.m[0][1], a.m[1][1], a.m[2][1]);
		x = x.NormalizedOr(Vec3::sAxisX());
		y = (y - x * x.Dot(y)).NormalizedOr(x.GetNormalizedPerpendicular());
		Vec3 z = x.Cross(y);
		Vec3 t = Vec3(a.m[0][3], a.m[1][3], a.m[2][3]) * c_toJolt;
		return Mat44(Vec4(x, 0.0f), Vec4(y, 0.0f), Vec4(z, 0.0f), Vec4(t, 1.0f));
	}

	Aff JoltToGame(JPH::Mat44Arg m, const float colScale[3])
	{
		Aff o;
		for (int c = 0; c < 3; c++)
		{
			JPH::Vec3 col = m.GetColumn3(c) * colScale[c];
			o.m[0][c] = col.GetX();
			o.m[1][c] = col.GetY();
			o.m[2][c] = col.GetZ();
		}
		JPH::Vec3 t = m.GetTranslation() * c_toGame;
		o.m[0][3] = t.GetX();
		o.m[1][3] = t.GetY();
		o.m[2][3] = t.GetZ();
		return o;
	}

	// ---- Public ----

	bool Supported(cModel* model)
	{
		if (!s_joltReady || !model)
			return false;

		const bool player = model == (cModel*)PlayerPtr();
		if (player ? !s_plPartsWorldPosCalc : (!s_em10PartsWorldCalc || !IsGanado(model->id_100)))
			return false;

		float scale;
		return Rig_Matches(model, scale);
	}

	bool Start(cModel* model, const Vec& push)
	{
		Lock lock(s_mutex);
		if (!Supported(model) || IsActive(model))
			return false;
		s_pending[model] = Pending{ model->guid_F8, push, GlobalPtr()->frameCounter_530C };
		return true;
	}

	void Stop(cModel* model)
	{
		Lock lock(s_mutex);
		s_pending.erase(model);
		auto it = s_instances.find(model);
		if (it != s_instances.end())
		{
			ReleaseBodies(it->second);
			s_instances.erase(it);
		}
	}

	void StopAll()
	{
		Lock lock(s_mutex);
		StopEverything();
	}

	bool IsActive(cModel* model)
	{
		Lock lock(s_mutex);
		return s_instances.count(model) || s_pending.count(model);
	}

	int ActiveCount()
	{
		Lock lock(s_mutex);
		return int(s_instances.size() + s_pending.size());
	}

	int SimulatedCount()
	{
		Lock lock(s_mutex);
		return CountSimulated();
	}

	int WorldBodyCount()
	{
		Lock lock(s_mutex);
		return World_BodyCount();
	}

	void GetDebugCapsules(std::vector<DebugCapsule>& out)
	{
		Lock lock(s_mutex);
		if (!s_system)
			return;
		JPH::BodyInterface& bi = s_system->GetBodyInterface();
		for (auto& kv : s_instances)
		{
			const Instance& in = kv.second;
			if (!in.ragdoll)
				continue;
			for (int b = 0; b < c_numBodies; b++)
			{
				const JPH::BodyID id = in.ragdoll->GetBodyID(b);
				const JPH::RMat44 t = bi.GetWorldTransform(id);
				const JPH::Vec3 a = JPH::Vec3(t * in.capA[b]) * c_toGame;
				const JPH::Vec3 e = JPH::Vec3(t * in.capB[b]) * c_toGame;
				out.push_back({ Vec(a.GetX(), a.GetY(), a.GetZ()), Vec(e.GetX(), e.GetY(), e.GetZ()), in.capR[b] * c_toGame, bi.IsActive(id) });
			}
		}
	}
}

void re4t::init::Ragdoll()
{
	using namespace ragdoll;

	JPH::RegisterDefaultAllocator();
	JPH::Trace = JoltTrace;
	JPH_IF_ENABLE_ASSERTS(JPH::AssertFailed = JoltAssertFailed;)

	// Same JPH_* defines here as in the library (Jolt.props)
	if (!JPH::VerifyJoltVersionID())
	{
		spd::log()->error("ragdoll -> Jolt was built with different settings than re4_tweaks, ragdolls disabled");
		return;
	}

	JPH::Factory::sInstance = new JPH::Factory();
	JPH::RegisterTypes();

	spd::log()->info("ragdoll -> Jolt Physics {}.{}.{} ({})", JPH_VERSION_MAJOR, JPH_VERSION_MINOR, JPH_VERSION_PATCH,
		JPH::GetConfigurationString());

	s_joltReady = SmokeTest();

	FindHookPoints();
	if (s_joltReady)
		InstallHooks();
}
