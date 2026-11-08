#include "RagdollInternal.h"
#include <Jolt/RegisterTypes.h>
#include <Jolt/ConfigurationString.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayerInterfaceTable.h>
#include <Jolt/Physics/Collision/BroadPhase/ObjectVsBroadPhaseLayerFilterTable.h>
#include <Jolt/Physics/Collision/ObjectLayerPairFilterTable.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyInterface.h>

#include <algorithm>
#include <array>
#include <cstdarg>
#include <cstdio>
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
// For now ragdolls are only started from the debug UI (Em Manager, "Parts (ragdoll)"); the death triggers come later.

namespace ragdoll
{
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

		// ---- Ragdolls ----

		struct Instance
		{
			cModel* model = nullptr;
			uint32_t guid = 0;
			float scale = 1.0f;
			JPH::Ref<JPH::Ragdoll> ragdoll;

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
		uint32_t s_lastFrame = 0;
		float s_accum = 0.0f;

		constexpr float c_step = 1.0f / 60.0f;
		constexpr int c_maxSteps = 4;
		constexpr uint32_t c_pendingTimeout = 30; // updates

		void DestroyInstance(Instance& in)
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

		bool CreateInstance(cModel* model, const Vec& push)
		{
			float scale;
			if (!Rig_Matches(model, scale) || !EnsureSystem())
				return false;

			const JPH::RagdollSettings* settings = Rig_Settings(scale);

			Instance in;
			in.model = model;
			in.guid = model->guid_F8;
			in.scale = scale;

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

			// Keep going the way the parts were moving (world_old is where they were at the end of the last update)
			const float dt = GlobalPtr()->deltaTime_70 / 30.0f;
			JPH::BodyInterface& bi = s_system->GetBodyInterface();
			const JPH::Vec3 pushV(push.x, push.y, push.z);
			for (int b = 0; b < c_numBodies; b++)
			{
				const cParts* p = in.parts[c_bodyPart[b]];
				JPH::Vec3 v = JPH::Vec3::sZero();
				if (dt > 0.0f)
				{
					const Vec d = p->world_70 - p->world_old_7C;
					v = JPH::Vec3(d.x, d.y, d.z) * (c_toJolt / dt);
					if (v.Length() > 15.0f)
						v = v.Normalized() * 15.0f;
				}
				bi.SetLinearVelocity(in.ragdoll->GetBodyID(b), v + pushV);

				Rig_Capsule(b, scale, in.capA[b], in.capB[b], in.capR[b]);
			}

			spd::log()->info("ragdoll -> started on {} (id 0x{:02X}, scale {:.2f}, {} parts)", (void*)model, int(model->id_100), scale, n);
			s_instances[model] = std::move(in);
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

		void WritePose(Instance& in)
		{
			JPH::RVec3 root;
			JPH::Mat44 mats[c_numBodies];
			in.ragdoll->GetPose(root, mats);
			for (JPH::Mat44& m : mats)
				m.SetTranslation(m.GetTranslation() + JPH::Vec3(root));

			const size_t n = in.parts.size();
			std::vector<Aff> world(n);
			std::vector<uint8_t> state(n, 0);
			for (size_t i = 0; i < n; i++)
				ComputePart(in, mats, world, state, int(i));

			for (size_t i = 0; i < n; i++)
			{
				if (in.body[i] < 0 && in.anchor[i] < 0)
					continue;
				cParts* p = in.parts[i];
				for (int r = 0; r < 3; r++)
					for (int c = 0; c < 4; c++)
						p->mat_C[r][c] = world[i].m[r][c];
				p->world_70 = Vec(world[i].m[0][3], world[i].m[1][3], world[i].m[2][3]);
			}
		}

		// Once per game update: drop what's gone, then step
		void UpdateFrame()
		{
			GLOBAL_WK* pG = GlobalPtr();
			const uint32_t frame = pG->frameCounter_530C;
			if (frame == s_lastFrame)
				return;
			s_lastFrame = frame;

			for (auto it = s_instances.begin(); it != s_instances.end();)
			{
				if (!ModelAlive(it->first, it->second.guid))
				{
					spd::log()->info("ragdoll -> {} is gone, ragdoll removed", (void*)it->first);
					DestroyInstance(it->second);
					it = s_instances.erase(it);
				}
				else
					++it;
			}

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

			World_Sync(dt);

			s_accum += dt;
			int steps = int(s_accum / c_step);
			if (steps > c_maxSteps)
			{
				steps = c_maxSteps;
				s_accum = 0.0f;
			}
			else
				s_accum -= steps * c_step;

			if (steps > 0)
				s_system->Update(steps * c_step, steps, s_temp, s_jobs);
		}

		// After the game posed a hooked model
		void AfterPose(cModel* model)
		{
			if (s_instances.empty() && s_pending.empty())
				return;

			UpdateFrame();

			auto pend = s_pending.find(model);
			if (pend != s_pending.end())
			{
				Pending p = pend->second;
				s_pending.erase(pend);
				if (model->guid_F8 == p.guid && !CreateInstance(model, p.push))
					spd::log()->warn("ragdoll -> couldn't start on {}", (void*)model);
				return; // already in the pose it starts from
			}

			auto it = s_instances.find(model);
			if (it != s_instances.end())
				WritePose(it->second);
		}

		// ---- Hooks ----

		uint8_t* s_em10PartsWorldCalc = nullptr; // cEm10::move: the partsWorldCalc call
		uint8_t* s_plPartsWorldPosCalc = nullptr; // cPlayer::move: the second PartsWorldPosCalc call

		void(__fastcall* cEm10_partsWorldCalc_orig)(cModel* thisptr, void* unused);
		void __fastcall cEm10_partsWorldCalc_hook(cModel* thisptr, void* unused)
		{
			cEm10_partsWorldCalc_orig(thisptr, unused);
			AfterPose(thisptr);
		}

		void(__cdecl* cPlayer_PartsWorldPosCalc_orig)(cModel* pMod);
		void __cdecl cPlayer_PartsWorldPosCalc_hook(cModel* pMod)
		{
			cPlayer_PartsWorldPosCalc_orig(pMod);
			AfterPose(pMod);
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
		if (!Supported(model) || IsActive(model))
			return false;
		s_pending[model] = Pending{ model->guid_F8, push, GlobalPtr()->frameCounter_530C };
		return true;
	}

	void Stop(cModel* model)
	{
		s_pending.erase(model);
		auto it = s_instances.find(model);
		if (it != s_instances.end())
		{
			DestroyInstance(it->second);
			s_instances.erase(it);
		}
	}

	void StopAll()
	{
		s_pending.clear();
		for (auto& kv : s_instances)
			DestroyInstance(kv.second);
		s_instances.clear();
	}

	bool IsActive(cModel* model)
	{
		return s_instances.count(model) || s_pending.count(model);
	}

	int ActiveCount()
	{
		return int(s_instances.size() + s_pending.size());
	}

	int WorldBodyCount()
	{
		return World_BodyCount();
	}

	void GetDebugCapsules(std::vector<DebugCapsule>& out)
	{
		if (!s_system)
			return;
		JPH::BodyInterface& bi = s_system->GetBodyInterface();
		for (auto& kv : s_instances)
		{
			const Instance& in = kv.second;
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
