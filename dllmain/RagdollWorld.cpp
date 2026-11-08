#include "RagdollInternal.h"
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyInterface.h>

#include <vector>

// The room's collision for the ragdolls: one Jolt mesh per collision piece in SatMgr (the same floors, slopes and walls the
// characters walk on and stop at; EatMgr's enemy-only barriers are left out).
//
// Each piece's triangles are baked into world space as they are when it's built, with the body at the origin. Pieces
// whose matrix changes later (room scripts, trolley, gondola, ...) become kinematic and follow it: the body's transform is
// the piece's current matrix times the inverse of the one it was built with. Pieces are rebuilt when the slot holds
// different data (room change), and taken out of the world while the game has them turned off.
//
// Jolt ignores the back faces of mesh triangles, so each triangle is wound to face along the game's own normal for it.

namespace ragdoll
{
	namespace
	{
		struct Piece
		{
			// What the body was built from
			const Vec* vtx = nullptr;
			const void* poly = nullptr;
			uint16_t numPoly = 0;
			uint16_t numVtx = 0;
			Aff built{};

			JPH::BodyID body;
			bool added = false;
			bool kinematic = false;
			bool failed = false; // no usable triangles: don't try again until the data changes
		};

		std::vector<Piece> s_pieces; // by SatMgr slot

		void DestroyPiece(Piece& p)
		{
			JPH::PhysicsSystem* sys = System();
			if (sys && !p.body.IsInvalid())
			{
				JPH::BodyInterface& bi = sys->GetBodyInterface();
				if (p.added)
					bi.RemoveBody(p.body);
				bi.DestroyBody(p.body);
			}
			p = Piece();
		}

		JPH::Vec3 MulPoint(const Aff& a, const Vec& v)
		{
			return JPH::Vec3(
				a.m[0][0] * v.x + a.m[0][1] * v.y + a.m[0][2] * v.z + a.m[0][3],
				a.m[1][0] * v.x + a.m[1][1] * v.y + a.m[1][2] * v.z + a.m[1][3],
				a.m[2][0] * v.x + a.m[2][1] * v.y + a.m[2][2] * v.z + a.m[2][3]);
		}

		JPH::Vec3 MulDir(const Aff& a, const Vec& v)
		{
			return JPH::Vec3(
				a.m[0][0] * v.x + a.m[0][1] * v.y + a.m[0][2] * v.z,
				a.m[1][0] * v.x + a.m[1][1] * v.y + a.m[1][2] * v.z,
				a.m[2][0] * v.x + a.m[2][1] * v.y + a.m[2][2] * v.z);
		}

		bool BuildPiece(cSat& sat, Piece& p)
		{
			using namespace JPH;

			p.vtx = sat.vert_p_C;
			p.poly = sat.poly_p_18;
			p.numPoly = sat.polygon_num_1E;
			p.numVtx = sat.vertex_num_1C;
			p.built = AffFrom(sat.mat_60);

			// The normals' transform: the matrix's 3x3 inverse transposed (only its direction matters here)
			Aff inv;
			if (!AffInverse(p.built, inv))
			{
				p.failed = true;
				return false;
			}
			Aff normalMat = p.built;
			for (int r = 0; r < 3; r++)
				for (int c = 0; c < 3; c++)
					normalMat.m[r][c] = inv.m[c][r];

			VertexList verts;
			verts.reserve(p.numVtx);
			for (uint16_t i = 0; i < p.numVtx; i++)
			{
				Vec3 w = MulPoint(p.built, sat.vert_p_C[i]) * c_toJolt;
				verts.push_back(Float3(w.GetX(), w.GetY(), w.GetZ()));
			}

			IndexedTriangleList tris;
			tris.reserve(p.numPoly);
			for (uint16_t i = 0; i < p.numPoly; i++)
			{
				const uint16_t* poly = sat.poly_p_18[i];
				uint32 a = poly[0], b = poly[1], c = poly[2];
				if (a >= p.numVtx || b >= p.numVtx || c >= p.numVtx)
					continue;

				Vec3 va(verts[a]), vb(verts[b]), vc(verts[c]);
				Vec3 n = (vb - va).Cross(vc - va);
				if (sat.norm_p_10 && poly[3] < sat.normal_num_28)
				{
					Vec3 gameNormal = MulDir(normalMat, sat.norm_p_10[poly[3]]);
					if (n.Dot(gameNormal) < 0.0f)
						std::swap(b, c);
				}
				tris.push_back(IndexedTriangle(a, b, c, 0));
			}

			MeshShapeSettings meshSettings(std::move(verts), std::move(tris));
			Shape::ShapeResult res = meshSettings.Create();
			if (res.HasError())
			{
				p.failed = true;
				return false;
			}

			BodyCreationSettings bcs(res.Get(), RVec3::sZero(), Quat::sIdentity(), EMotionType::Static, Layers::World);
			bcs.mAllowDynamicOrKinematic = true; // in case the piece moves later
			bcs.mFriction = 0.8f;
			bcs.mRestitution = 0.0f;

			Body* body = System()->GetBodyInterface().CreateBody(bcs);
			if (!body)
			{
				p.failed = true; // out of bodies
				return false;
			}
			p.body = body->GetID();
			return true;
		}

		bool SameMatrix(const Aff& a, const Mtx b)
		{
			for (int r = 0; r < 3; r++)
				for (int c = 0; c < 4; c++)
					if (fabsf(a.m[r][c] - b[r][c]) > (c == 3 ? 0.01f : 1e-5f))
						return false;
			return true;
		}
	}

	void World_Sync(float dt)
	{
		using namespace JPH;

		PhysicsSystem* sys = System();
		if (!sys || !SatMgr || !SatMgr->m_Array_4)
			return;

		BodyInterface& bi = sys->GetBodyInterface();

		const size_t count = SatMgr->count();
		for (size_t i = count; i < s_pieces.size(); i++)
			DestroyPiece(s_pieces[i]);
		s_pieces.resize(count);

		for (size_t i = 0; i < count; i++)
		{
			cSat& sat = *SatMgr->get(i);
			Piece& p = s_pieces[i];

			const bool valid = sat.IsValid() && sat.vert_p_C && sat.poly_p_18 && sat.polygon_num_1E && sat.vertex_num_1C;
			const bool enabled = valid && (sat.m_Flag_2A & cSat::FLAG::FLAG_ENABLE_VAL) == cSat::FLAG::FLAG_ENABLE_VAL;

			// Different data in this slot: a different piece
			const bool same = valid && p.vtx == sat.vert_p_C && p.poly == sat.poly_p_18 && p.numPoly == sat.polygon_num_1E &&
				p.numVtx == sat.vertex_num_1C;
			if (!same && (!p.body.IsInvalid() || p.failed))
				DestroyPiece(p);

			if (!enabled)
			{
				if (p.added)
				{
					bi.RemoveBody(p.body);
					p.added = false;
				}
				continue;
			}

			if (p.body.IsInvalid())
			{
				if (p.failed || !BuildPiece(sat, p))
					continue;
			}

			if (!p.added)
			{
				bi.AddBody(p.body, EActivation::DontActivate);
				p.added = true;
			}

			// Moved since it was built: follow it
			if (!SameMatrix(p.built, sat.mat_60) || p.kinematic)
			{
				Aff inv, delta;
				if (!AffInverse(p.built, inv))
					continue;
				delta = AffMul(AffFrom(sat.mat_60), inv);
				const Mat44 m = RigidToJolt(delta);

				if (!p.kinematic)
				{
					bi.SetMotionType(p.body, EMotionType::Kinematic, EActivation::DontActivate);
					p.kinematic = true;
				}
				if (dt > 0.0f)
					bi.MoveKinematic(p.body, RVec3(m.GetTranslation()), m.GetQuaternion(), dt);
				else
					bi.SetPositionAndRotation(p.body, RVec3(m.GetTranslation()), m.GetQuaternion(), EActivation::DontActivate);
			}
		}
	}

	void World_Clear()
	{
		for (Piece& p : s_pieces)
			DestroyPiece(p);
		s_pieces.clear();
	}

	int World_BodyCount()
	{
		int n = 0;
		for (const Piece& p : s_pieces)
			n += p.added ? 1 : 0;
		return n;
	}
}
