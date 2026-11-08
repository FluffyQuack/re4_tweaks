#pragma once

// Shared between the ragdoll source files. Jolt comes first: it must be the first Jolt header, and before Windows'
// headers define their macros.
#include <Jolt/Jolt.h>
#include <Jolt/Core/Reference.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Ragdoll/Ragdoll.h>

#include <cmath>
#include "dllmain.h"
#include "Game.h"
#include "Ragdoll.h"

namespace ragdoll
{
	// Game units are mm, Jolt's are m
	constexpr float c_toJolt = 0.001f;
	constexpr float c_toGame = 1000.0f;

	namespace Layers
	{
		constexpr JPH::ObjectLayer World = 0;   // the room's collision (SatMgr)
		constexpr JPH::ObjectLayer Ragdoll = 1;
		constexpr JPH::uint Count = 2;
	}

	// Null until the first ragdoll starts (Ragdoll.cpp)
	JPH::PhysicsSystem* System();

	// ---- Affine 3x4 matrices, laid out like the game's Mtx (rows; translation in column 3) ----

	struct Aff
	{
		float m[3][4];
	};

	inline Aff AffFrom(const Mtx src)
	{
		Aff a;
		for (int r = 0; r < 3; r++)
			for (int c = 0; c < 4; c++)
				a.m[r][c] = src[r][c];
		return a;
	}

	inline Aff AffMul(const Aff& a, const Aff& b)
	{
		Aff o;
		for (int r = 0; r < 3; r++)
		{
			for (int c = 0; c < 4; c++)
			{
				float v = a.m[r][0] * b.m[0][c] + a.m[r][1] * b.m[1][c] + a.m[r][2] * b.m[2][c];
				if (c == 3)
					v += a.m[r][3];
				o.m[r][c] = v;
			}
		}
		return o;
	}

	// False when singular
	inline bool AffInverse(const Aff& a, Aff& o)
	{
		const float(*m)[4] = a.m;
		float det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
			- m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
			+ m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
		if (fabsf(det) < 1e-12f)
			return false;
		float id = 1.0f / det;
		o.m[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * id;
		o.m[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * id;
		o.m[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * id;
		o.m[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * id;
		o.m[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * id;
		o.m[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * id;
		o.m[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * id;
		o.m[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * id;
		o.m[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * id;
		for (int r = 0; r < 3; r++)
			o.m[r][3] = -(o.m[r][0] * m[0][3] + o.m[r][1] * m[1][3] + o.m[r][2] * m[2][3]);
		return true;
	}

	inline float AffColumnLength(const Aff& a, int c)
	{
		return sqrtf(a.m[0][c] * a.m[0][c] + a.m[1][c] * a.m[1][c] + a.m[2][c] * a.m[2][c]);
	}

	// The rotation and translation of a game matrix (columns normalized and made orthogonal), in Jolt's units
	JPH::Mat44 RigidToJolt(const Aff& a);

	// A Jolt rigid transform back to a game matrix, with each column scaled by colScale and the translation in game units
	Aff JoltToGame(JPH::Mat44Arg m, const float colScale[3]);

	// ---- World (RagdollWorld.cpp) ----

	// Brings the collision pieces in line with SatMgr: builds new ones, drops stale ones, adds/removes the ones turned on
	// or off, and moves the ones whose matrix changed (kinematic). dt in seconds.
	void World_Sync(float dt);
	void World_Clear();
	int World_BodyCount();

	// ---- Skeleton (RagdollRig.cpp) ----

	constexpr int c_numBodies = 12;

	// The game part each ragdoll body drives
	extern const int c_bodyPart[c_numBodies];

	// Parts 0-25 match the shared humanoid skeleton; scale is the model's (1.0 for Leon, 1.25 for Soldiers)
	bool Rig_Matches(cModel* model, float& scale);

	// Ragdoll settings for this scale (built once per scale, then shared)
	const JPH::RagdollSettings* Rig_Settings(float scale);

	// A body's capsule (end points in the body's local space, m)
	void Rig_Capsule(int body, float scale, JPH::Vec3& a, JPH::Vec3& b, float& radius);
}
