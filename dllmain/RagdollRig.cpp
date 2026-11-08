#include "RagdollInternal.h"
#include <Jolt/Skeleton/Skeleton.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Constraints/SwingTwistConstraint.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>

#include <algorithm>
#include <map>

// The ragdoll for the humanoid skeleton shared by Leon, Ganados, Zealots, Soldiers and Garradors (parts 0-25, see
// RAGDOLL_PLAN.md, "Part mapping").
//
// Everything is defined in the skeleton's bind pose: a T-pose with no rotation on any part, facing +Z, Y up, -X being
// the character's right. Each body's frame is its part's frame (so a body's transform is its part's world transform,
// without scale), and its capsule is placed in that frame. The joint limits are relative to the bind pose.

namespace ragdoll
{
	namespace
	{
		// Bind pose: each part's parent and local offset (game units, scale 1). Part 0 sits at the origin.
		struct BindPart
		{
			int parent;
			float x, y, z;
		};

		const BindPart c_bind[26] =
		{
			{ -1,    0.0f,    0.0f,   0.0f }, //  0 pelvis
			{  0,    0.0f,   10.0f,   0.0f }, //  1 waist
			{  1,    0.0f,  210.0f, -13.4f }, //  2 chest
			{  2,    0.0f,  200.0f, -21.0f }, //  3 neck
			{  3,    0.0f,   90.0f,  11.0f }, //  4 head
			{  2,  -45.0f,  105.0f, -21.0f }, //  5 clavicle R
			{  5, -127.0f,   42.8f,   0.0f }, //  6 shoulder R (twin of 7)
			{  5, -127.0f,   42.8f,   0.0f }, //  7 shoulder R: upper arm
			{  7, -264.0f,    0.0f,   0.0f }, //  8 elbow R: forearm
			{  8, -154.0f,    0.0f,   0.0f }, //  9 forearm twist R
			{  9, -114.0f,    0.0f,   0.0f }, // 10 wrist R: hand
			{  2,   45.0f,  105.0f, -21.0f }, // 11 clavicle L
			{ 11,  127.0f,   42.8f,   0.0f }, // 12 shoulder L (twin of 13)
			{ 11,  127.0f,   42.8f,   0.0f }, // 13 shoulder L: upper arm
			{ 13,  264.0f,    0.0f,   0.0f }, // 14 elbow L: forearm
			{ 14,  154.0f,    0.0f,   0.0f }, // 15 forearm twist L
			{ 15,  114.0f,    0.0f,   0.0f }, // 16 wrist L: hand
			{  0,    0.0f,    0.0f,   0.0f }, // 17 leg root
			{ 17,  -92.0f, -130.0f,   0.0f }, // 18 hip R: thigh
			{ 18,    0.0f, -435.5f,   0.0f }, // 19 knee R: shin
			{ 19,    0.0f, -439.5f,   0.0f }, // 20 ankle R: foot
			{ 20,    0.0f,  -91.0f,  86.8f }, // 21 toe R
			{ 17,   92.0f, -130.0f,   0.0f }, // 22 hip L: thigh
			{ 22,    0.0f, -435.5f,   0.0f }, // 23 knee L: shin
			{ 23,    0.0f, -439.5f,   0.0f }, // 24 ankle L: foot
			{ 24,    0.0f,  -91.0f,  86.8f }, // 25 toe L
		};

		// Parts whose local offset must match the bind pose for a model to count as this skeleton (part 0's is animated,
		// part 21/25's toe differs slightly between models)
		const int c_checkedParts[] = { 1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 13, 14, 15, 16, 18, 19, 20, 22, 23, 24 };

		enum class ShapeKind
		{
			AcrossX, // capsule along X through `center` (torso)
			AlongY,  // capsule along Y through `center` (head)
			Bone,    // capsule from the part to endPart, extended past it by `extend`
		};

		enum class JointKind
		{
			None,       // the root
			SwingTwist, // cone around `rest`, twist around the bone
			Hinge,      // around `axis`, 0 = the bind pose
		};

		struct BodyDef
		{
			const char* name;
			int part;
			int parent; // body index

			int endPart; // where the bone ends (twist axis, and the capsule for Bone shapes)
			ShapeKind shape;
			float cx, cy, cz; // AcrossX / AlongY: capsule centre in the part's frame
			float halfHeight; // AcrossX / AlongY
			float radius;
			float extend;     // Bone

			JointKind joint;
			float ax, ay, az;   // SwingTwist: plane axis; Hinge: hinge axis
			float rx, ry, rz;   // SwingTwist: the cone's centre (rest direction of the bone)
			float normalCone;   // SwingTwist: swing around twist x plane (degrees)
			float planeCone;    // SwingTwist: swing around the plane axis
			float twist;        // SwingTwist: +- twist
			float hingeMin, hingeMax; // Hinge
		};

		// Rest directions tilt each cone towards where the joint usually is, since cones are symmetric: the spine and neck
		// a little forward, the arms down and forward, the hips forward (bend 100 degrees forward, 40 back).
		const float c_s5 = 0.08716f, c_c5 = 0.99619f;   // sin/cos 5
		const float c_s10 = 0.17365f, c_c10 = 0.98481f; // sin/cos 10

		const BodyDef c_bodies[c_numBodies] =
		{
			//  name          part parent end  shape              centre                 hh     r     ext   joint                  axis              rest                        normal plane twist hinge
			{ "Pelvis",       0,   -1,   -1, ShapeKind::AcrossX,  0.0f, -60.0f,  0.0f,   80.0f, 100.0f, 0.0f, JointKind::None },
			{ "Spine",        1,    0,    2, ShapeKind::AcrossX,  0.0f, 105.0f, -5.0f,   70.0f, 105.0f, 0.0f, JointKind::SwingTwist, 0.0f, 0.0f, 1.0f, 0.0f, c_c5, c_s5,          25.0f, 20.0f, 20.0f },
			{ "Chest",        2,    1,    3, ShapeKind::AcrossX,  0.0f, 110.0f, -5.0f,   95.0f, 110.0f, 0.0f, JointKind::SwingTwist, 0.0f, 0.0f, 1.0f, 0.0f, c_c5, c_s5,          25.0f, 20.0f, 25.0f },
			{ "Head",         3,    2,    4, ShapeKind::AlongY,   0.0f, 150.0f, 15.0f,   40.0f,  95.0f, 0.0f, JointKind::SwingTwist, 0.0f, 0.0f, 1.0f, 0.0f, c_c10, c_s10,        40.0f, 30.0f, 50.0f },
			{ "UpperArmR",    7,    2,    8, ShapeKind::Bone,     0.0f,   0.0f,  0.0f,    0.0f,  50.0f, 0.0f, JointKind::SwingTwist, 0.0f, 0.0f, 1.0f, -0.65f, -0.65f, 0.35f,   75.0f, 85.0f, 70.0f },
			{ "LowerArmR",    8,    4,   10, ShapeKind::Bone,     0.0f,   0.0f,  0.0f,    0.0f,  42.0f, 90.0f, JointKind::Hinge,     0.0f, 1.0f, 0.0f, 0, 0, 0, 0, 0, 0,                -3.0f, 140.0f },
			{ "UpperArmL",   13,    2,   14, ShapeKind::Bone,     0.0f,   0.0f,  0.0f,    0.0f,  50.0f, 0.0f, JointKind::SwingTwist, 0.0f, 0.0f, 1.0f, 0.65f, -0.65f, 0.35f,    75.0f, 85.0f, 70.0f },
			{ "LowerArmL",   14,    6,   16, ShapeKind::Bone,     0.0f,   0.0f,  0.0f,    0.0f,  42.0f, 90.0f, JointKind::Hinge,     0.0f, -1.0f, 0.0f, 0, 0, 0, 0, 0, 0,               -3.0f, 140.0f },
			{ "ThighR",      18,    0,   19, ShapeKind::Bone,     0.0f,   0.0f,  0.0f,    0.0f,  70.0f, 0.0f, JointKind::SwingTwist, 0.0f, 0.0f, 1.0f, 0.0f, -0.866f, 0.5f,       70.0f, 35.0f, 25.0f },
			{ "ShinR",       19,    8,   20, ShapeKind::Bone,     0.0f,   0.0f,  0.0f,    0.0f,  55.0f, 40.0f, JointKind::Hinge,     1.0f, 0.0f, 0.0f, 0, 0, 0, 0, 0, 0,                -3.0f, 145.0f },
			{ "ThighL",      22,    0,   23, ShapeKind::Bone,     0.0f,   0.0f,  0.0f,    0.0f,  70.0f, 0.0f, JointKind::SwingTwist, 0.0f, 0.0f, 1.0f, 0.0f, -0.866f, 0.5f,       70.0f, 35.0f, 25.0f },
			{ "ShinL",       23,   10,   24, ShapeKind::Bone,     0.0f,   0.0f,  0.0f,    0.0f,  55.0f, 40.0f, JointKind::Hinge,     1.0f, 0.0f, 0.0f, 0, 0, 0, 0, 0, 0,                -3.0f, 145.0f },
		};
		// Elbows: flexing brings the forearm forward (+Z), which is +Y for the right arm (bone -X) and -Y for the left
		// (bone +X). Knees: flexing takes the shin back (-Z), +X for both.

		JPH::Vec3 BindPos(int part)
		{
			JPH::Vec3 p = JPH::Vec3::sZero();
			for (int i = part; i >= 0; i = c_bind[i].parent)
				p += JPH::Vec3(c_bind[i].x, c_bind[i].y, c_bind[i].z);
			return p;
		}

		float DegToRad(float d)
		{
			return d * (JPH::JPH_PI / 180.0f);
		}

		// The capsule of a body, in its frame, game units at scale 1: the inner segment's centre and direction, and its
		// half height
		void CapsuleGeometry(const BodyDef& d, JPH::Vec3& centre, JPH::Vec3& dir, float& halfHeight)
		{
			switch (d.shape)
			{
			case ShapeKind::AcrossX:
				centre = JPH::Vec3(d.cx, d.cy, d.cz);
				dir = JPH::Vec3::sAxisX();
				halfHeight = d.halfHeight;
				break;
			case ShapeKind::AlongY:
				centre = JPH::Vec3(d.cx, d.cy, d.cz);
				dir = JPH::Vec3::sAxisY();
				halfHeight = d.halfHeight;
				break;
			case ShapeKind::Bone:
			default:
			{
				JPH::Vec3 e = BindPos(d.endPart) - BindPos(d.part);
				float len = e.Length();
				dir = e / len;
				len += d.extend;
				centre = dir * (0.5f * len);
				halfHeight = std::max(0.5f * len - 0.5f * d.radius, 0.01f * len);
				break;
			}
			}
		}

		JPH::Ref<JPH::RagdollSettings> BuildSettings(float scale)
		{
			using namespace JPH;

			const float k = scale * c_toJolt;

			Ref<RagdollSettings> settings = new RagdollSettings();
			settings->mSkeleton = new Skeleton();
			settings->mParts.resize(c_numBodies);

			Mat44 bindMats[c_numBodies];

			for (int b = 0; b < c_numBodies; b++)
			{
				const BodyDef& d = c_bodies[b];
				settings->mSkeleton->AddJoint(d.name, d.parent);

				const Vec3 pos = BindPos(d.part) * k;
				bindMats[b] = Mat44::sTranslation(pos);

				// Capsule (along local Y) moved into the body's frame
				Vec3 centre, dir;
				float halfHeight;
				CapsuleGeometry(d, centre, dir, halfHeight);
				Ref<Shape> capsule = new CapsuleShape(halfHeight * k, d.radius * k);
				Ref<Shape> shape = RotatedTranslatedShapeSettings(centre * k, Quat::sFromTo(Vec3::sAxisY(), dir), capsule).Create().Get();

				RagdollSettings::Part& part = settings->mParts[b];
				part.SetShape(shape);
				part.mPosition = RVec3(pos);
				part.mRotation = Quat::sIdentity();
				part.mMotionType = EMotionType::Dynamic;
				part.mObjectLayer = Layers::Ragdoll;
				part.mMotionQuality = EMotionQuality::LinearCast; // thin walls and fast limbs
				part.mFriction = 0.8f;
				part.mRestitution = 0.0f;
				part.mLinearDamping = 0.1f;
				part.mAngularDamping = 0.4f;

				if (d.parent < 0)
					continue;

				// The joint is at this body's part; the bone runs to endPart
				const Vec3 bone = (BindPos(d.endPart) - BindPos(d.part)).Normalized();

				if (d.joint == JointKind::Hinge)
				{
					Ref<HingeConstraintSettings> c = new HingeConstraintSettings();
					c->mSpace = EConstraintSpace::WorldSpace;
					c->mPoint1 = c->mPoint2 = RVec3(pos);
					c->mHingeAxis1 = c->mHingeAxis2 = Vec3(d.ax, d.ay, d.az);
					c->mNormalAxis1 = c->mNormalAxis2 = bone;
					c->mLimitsMin = DegToRad(d.hingeMin);
					c->mLimitsMax = DegToRad(d.hingeMax);
					c->mMaxFrictionTorque = 0.5f;
					part.mToParent = c;
				}
				else
				{
					// Body 2 (this one) has its twist axis along the bone; body 1 (the parent) has the cone's centre at
					// the rest direction: both frames rotated by the same bind -> rest rotation
					const Vec3 plane = Vec3(d.ax, d.ay, d.az);
					const Vec3 rest = Vec3(d.rx, d.ry, d.rz).Normalized();
					const Quat toRest = Quat::sFromTo(bone, rest);

					Ref<SwingTwistConstraintSettings> c = new SwingTwistConstraintSettings();
					c->mSpace = EConstraintSpace::WorldSpace;
					c->mPosition1 = c->mPosition2 = RVec3(pos);
					c->mTwistAxis2 = bone;
					c->mPlaneAxis2 = plane;
					c->mTwistAxis1 = rest;
					c->mPlaneAxis1 = (toRest * plane).Normalized();
					c->mNormalHalfConeAngle = DegToRad(d.normalCone);
					c->mPlaneHalfConeAngle = DegToRad(d.planeCone);
					c->mTwistMinAngle = -DegToRad(d.twist);
					c->mTwistMaxAngle = DegToRad(d.twist);
					c->mMaxFrictionTorque = 1.0f;
					part.mToParent = c;
				}
			}

			settings->Stabilize();
			settings->DisableParentChildCollisions(bindMats, 0.02f);
			settings->CalculateBodyIndexToConstraintIndex();
			settings->CalculateConstraintIndexToBodyIdxPair();
			return settings;
		}

		std::map<int, JPH::Ref<JPH::RagdollSettings>> s_settings; // by scale * 100

		cParts* PartAt(cModel* model, int idx)
		{
			cParts* p = model->childParts_F4;
			for (; p && idx > 0; idx--)
				p = p->nextParts_F4;
			return p;
		}

		float Dist(const Vec& a, const Vec& b)
		{
			Vec d = a - b;
			return sqrtf(d.x * d.x + d.y * d.y + d.z * d.z);
		}
	}

	const int c_bodyPart[c_numBodies] = { 0, 1, 2, 3, 7, 8, 13, 14, 18, 19, 22, 23 };

	bool Rig_Matches(cModel* model, float& scale)
	{
		if (!model || !model->childParts_F4)
			return false;

		cParts* parts[26];
		for (int i = 0; i < 26; i++)
		{
			parts[i] = PartAt(model, i);
			if (!parts[i])
				return false;
		}

		for (int i = 1; i < 26; i++)
			if (parts[i]->pParent_6C != parts[c_bind[i].parent])
				return false;

		for (int i : c_checkedParts)
		{
			const Vec& p = parts[i]->pos_94;
			if (fabsf(p.x - c_bind[i].x) > 1.0f || fabsf(p.y - c_bind[i].y) > 1.0f || fabsf(p.z - c_bind[i].z) > 1.0f)
				return false;
		}

		// Bone lengths in the world against the bind pose: thighs and upper arms
		float s = (Dist(parts[18]->world_70, parts[19]->world_70) + Dist(parts[22]->world_70, parts[23]->world_70)) / (2.0f * 435.5f);
		s = 0.5f * s + 0.5f * (Dist(parts[7]->world_70, parts[8]->world_70) + Dist(parts[13]->world_70, parts[14]->world_70)) / (2.0f * 264.0f);
		if (!(s > 0.5f && s < 2.0f))
			return false;

		scale = s;
		return true;
	}

	const JPH::RagdollSettings* Rig_Settings(float scale)
	{
		int key = int(lroundf(scale * 100.0f));
		auto it = s_settings.find(key);
		if (it != s_settings.end())
			return it->second;

		JPH::Ref<JPH::RagdollSettings> settings = BuildSettings(key / 100.0f);
		s_settings[key] = settings;
		spd::log()->info("ragdoll -> built ragdoll settings for scale {:.2f}", key / 100.0f);
		return settings;
	}

	void Rig_Capsule(int body, float scale, JPH::Vec3& a, JPH::Vec3& b, float& radius)
	{
		const float k = float(lroundf(scale * 100.0f)) / 100.0f * c_toJolt;
		JPH::Vec3 centre, dir;
		float halfHeight;
		CapsuleGeometry(c_bodies[body], centre, dir, halfHeight);
		a = (centre - dir * halfHeight) * k;
		b = (centre + dir * halfHeight) * k;
		radius = c_bodies[body].radius * k;
	}
}
