#include <iostream>
#include "dllmain.h"
#include "Game.h"
#include "Settings.h"
#include <unordered_map>
#include <cstring>
#include <cmath>

// Ashley's bust: physics strength, jiggle physics and size
//
// The game has two copies of the same bounce, cPlAshley::moveBust (Ashley as the player) and cSubChar::moveBust (Ashley
// following Leon). Both run after her matrices for the frame are built, and build one offset (a sine on Y, pumped while she
// moves faster than 5 units/frame and gone ~11 frames after she stops), which they add to the rest positions of parts 0x1D,
// 0x1E (the bust) and 0x1A.
//
//  - Strength scales that offset, on its way into those three additions.
//  - Jiggle physics replaces that offset with our own: each bust part is a mass on a damped spring that follows where the
//    part would be at rest, in world space, so any movement of her chest (steps, turns, idle sway) makes it lag and
//    overshoot, and keep wobbling for as long as the damping allows. Strength then scales how far it travels.
//  - Size scales the final matrices of parts 0x1D and 0x1E, right after moveBust returns, so everything skinned to them is
//    scaled around their origin. The matrices are rebuilt from scratch every frame (partsWorldCalc), so the scale doesn't
//    compound, but if one wasn't rebuilt since we last scaled it (it still holds what we wrote), it's left alone.

static constexpr int c_bustParts[] = { 0x1D, 0x1E };
static constexpr int c_bustPartsAll[] = { 0x1D, 0x1E, 0x1A };

// cModel::getPartsPtr
static cParts* GetParts(cModel* model, int idx)
{
	cParts* p = model->childParts_F4;
	if (!p)
		return nullptr;
	if (model->be_flag_4 & UNIT_BE_FLAG_CONTIGUOUSPARTS)
		return idx < model->nParts_102 ? p + idx : nullptr;
	for (; p && idx > 0; idx--)
		p = p->nextParts_F4;
	return p;
}

// ---------------------------------------------------------------------------------------------------------------------
// Jiggle physics

struct JiggleState
{
	Vec mass;       // world
	Vec vel;        // world, units/s
	Vec anchorPrev; // world, where the part was at rest last update
	Vec out;        // world offset to apply
	uint32_t lastFrame = 0;
	bool valid = false;
};
static std::unordered_map<cParts*, JiggleState> s_jiggle;

// The model whose moveBust is running, and its parts' offsets (world)
struct JiggleCurrent
{
	bool active = false;
	cParts* parts[3] = {};
	Vec out[3];
};
static JiggleCurrent s_jiggleCur;

static Vec MtxMulPoint(const Mtx m, const Vec& v)
{
	return Vec(
		m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z + m[0][3],
		m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z + m[1][3],
		m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z + m[2][3]);
}

// A world direction in a matrix's local space (columns may be scaled, not skewed)
static Vec MtxToLocalDir(const Mtx m, const Vec& w)
{
	Vec res;
	float* out[3] = { &res.x, &res.y, &res.z };
	for (int c = 0; c < 3; c++)
	{
		float len2 = m[0][c] * m[0][c] + m[1][c] * m[1][c] + m[2][c] * m[2][c];
		*out[c] = len2 > 0.0f ? (m[0][c] * w.x + m[1][c] * w.y + m[2][c] * w.z) / len2 : 0.0f;
	}
	return res;
}

static float VecLen(const Vec& v)
{
	return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

static Vec VecScale(const Vec& v, float s)
{
	return Vec(v.x * s, v.y * s, v.z * s);
}

static void JiggleStep(cParts* p, const Vec& rest, JiggleState& s, float dt, uint32_t frame)
{
	const float maxTravel = re4t::cfg->fAshleyBustJiggleMaxTravel;
	const float omega = 2.0f * 3.14159265f * re4t::cfg->fAshleyBustJiggleSpeed;
	// Wobble duration = time to fade to ~5% (3 time constants)
	const float zeta = fminf(3.0f / (omega * re4t::cfg->fAshleyBustJiggleDuration), 1.0f);
	const float stiffness = omega * omega;
	const float damping = 2.0f * zeta * omega;

	Vec anchor = MtxMulPoint(p->pParent_6C->mat_C, rest);

	// Start over when new, after a long gap, or after a teleport (room change, cutscene cut)
	Vec jump = anchor - s.anchorPrev;
	if (!s.valid || frame - s.lastFrame > 60 || VecLen(jump) > 300.0f)
	{
		s.mass = anchor;
		s.vel = Vec();
		s.anchorPrev = anchor;
		s.out = Vec();
		s.valid = true;
	}
	s.lastFrame = frame;

	if (dt <= 0.0f)
		return;

	// Semi-implicit Euler, substepped: the anchor moves linearly over the update
	const int steps = 8;
	const float h = dt / steps;
	Vec anchorVel = VecScale(anchor - s.anchorPrev, 1.0f / dt);
	for (int i = 1; i <= steps; i++)
	{
		float t = float(i) / steps;
		Vec a = s.anchorPrev + VecScale(anchor - s.anchorPrev, t);
		Vec x = s.mass - a;
		Vec rv = s.vel - anchorVel;
		Vec acc = VecScale(x, -stiffness) - VecScale(rv, damping);
		s.vel = s.vel + VecScale(acc, h);
		s.mass = s.mass + VecScale(s.vel, h);
	}
	s.anchorPrev = anchor;

	// Keep the simulation itself in bounds
	Vec x = s.mass - anchor;
	float len = VecLen(x);
	float limit = maxTravel * 4.0f;
	if (len > limit)
	{
		s.mass = anchor + VecScale(x, limit / len);
		x = s.mass - anchor;
	}

	// Output: scaled by strength, less travel sideways/forwards than vertically (so it doesn't sink into her chest), then
	// softly limited to maxTravel
	Vec d = VecScale(x, re4t::cfg->fAshleyBustPhysicsStrength);
	d.x *= 0.5f;
	d.z *= 0.5f;
	len = VecLen(d);
	if (len > 0.0f)
		d = VecScale(d, maxTravel * tanhf(len / maxTravel) / len);
	s.out = d;
}

// Rest positions of parts 0x1D, 0x1E, 0x1A in cPlAshley / cSubChar
static uint32_t s_plRestOfs[3];
static uint32_t s_subRestOfs[3];

static void JiggleBegin(cModel* model, const uint32_t restOfs[3])
{
	s_jiggleCur = JiggleCurrent();

	if (!re4t::cfg->bAshleyBustJigglePhysics || !model)
	{
		s_jiggle.clear();
		return;
	}

	cParts* parts[3];
	for (int i = 0; i < 3; i++)
	{
		parts[i] = GetParts(model, c_bustPartsAll[i]);
		if (!parts[i] || !parts[i]->pParent_6C)
			return;
	}

	// Parts come and go with models (room changes etc)
	if (s_jiggle.size() > 32)
		s_jiggle.clear();

	const float dt = GlobalPtr()->deltaTime_70 / 30.0f;
	const uint32_t frame = GlobalPtr()->frameCounter_530C;
	for (int i = 0; i < 2; i++)
	{
		const Vec* rest = (const Vec*)((uint8_t*)model + restOfs[i]);
		JiggleState& s = s_jiggle[parts[i]];
		JiggleStep(parts[i], *rest, s, dt, frame);
		s_jiggleCur.parts[i] = parts[i];
		s_jiggleCur.out[i] = s.out;
	}

	// 0x1A moves with the bust in the original, so it gets the average
	s_jiggleCur.parts[2] = parts[2];
	s_jiggleCur.out[2] = VecScale(s_jiggleCur.out[0] + s_jiggleCur.out[1], 0.5f);
	s_jiggleCur.active = true;
}

static void JiggleEnd()
{
	s_jiggleCur.active = false;
}

// ---------------------------------------------------------------------------------------------------------------------
// The additions in moveBust

// PSVECAdd
static void(__cdecl* PSVECAdd_orig)(const Vec* a, const Vec* b, Vec* ab);

// The bounce offset is the second operand, the result goes to the part's pos
static void __cdecl BustVecAdd_hook(const Vec* base, const Vec* ofs, Vec* out)
{
	if (s_jiggleCur.active)
	{
		cParts* part = reinterpret_cast<cParts*>(reinterpret_cast<uint8_t*>(out) - 0x94); // cCoord::pos_94
		for (int i = 0; i < 3; i++)
		{
			if (s_jiggleCur.parts[i] == part)
			{
				// pos is in the parent's space
				Vec local = MtxToLocalDir(part->pParent_6C->mat_C, s_jiggleCur.out[i]);
				PSVECAdd_orig(base, &local, out);
				return;
			}
		}
	}

	float strength = re4t::cfg->fAshleyBustPhysicsStrength;
	if (strength == 1.0f)
	{
		PSVECAdd_orig(base, ofs, out);
		return;
	}

	Vec scaled(ofs->x * strength, ofs->y * strength, ofs->z * strength);
	PSVECAdd_orig(base, &scaled, out);
}

// ---------------------------------------------------------------------------------------------------------------------
// Size

// The matrix we last wrote to each scaled part
struct BustMtx
{
	Mtx m;
};
static std::unordered_map<cParts*, BustMtx> s_scaledMats;

static void ApplyBustSize(cModel* model)
{
	float size = re4t::cfg->fAshleyBustSize;
	if (size == 1.0f || !model)
	{
		s_scaledMats.clear();
		return;
	}

	// Parts come and go with models (room changes etc)
	if (s_scaledMats.size() > 32)
		s_scaledMats.clear();

	for (int idx : c_bustParts)
	{
		cParts* p = GetParts(model, idx);
		if (!p)
			continue;

		auto last = s_scaledMats.find(p);
		if (last != s_scaledMats.end() && std::memcmp(last->second.m, p->mat_C, sizeof(Mtx)) == 0)
			continue;

		// mat * scale: the rotation columns, around the part's own origin
		for (int r = 0; r < 3; r++)
			for (int c = 0; c < 3; c++)
				p->mat_C[r][c] *= size;

		std::memcpy(s_scaledMats[p].m, p->mat_C, sizeof(Mtx));
	}
}

// ---------------------------------------------------------------------------------------------------------------------

// cPlAshley::moveBust
static void(__fastcall* cPlAshley__moveBust_orig)(cModel* thisptr, void* unused);
static void __fastcall cPlAshley__moveBust_hook(cModel* thisptr, void* unused)
{
	JiggleBegin(thisptr, s_plRestOfs);
	cPlAshley__moveBust_orig(thisptr, unused);
	JiggleEnd();
	ApplyBustSize(thisptr);
}

// cSubChar::moveBust
static void(__fastcall* cSubChar__moveBust_orig)(cModel* thisptr, void* unused);
static void __fastcall cSubChar__moveBust_hook(cModel* thisptr, void* unused)
{
	JiggleBegin(thisptr, s_subRestOfs);
	cSubChar__moveBust_orig(thisptr, unused);
	JiggleEnd();
	ApplyBustSize(thisptr);
}

void re4t::init::AshleyBust()
{
	// The PSVECAdd calls in cPlAshley::moveBust / cSubChar::moveBust, after the lea/add of the rest positions of 0x1D,
	// 0x1E, 0x1A (disp32 at +2)
	const char* plPatterns[3] = { "8D 8F 14 08 00 00 51 E8", "8D 87 20 08 00 00 50 E8", "81 C7 2C 08 00 00 57 E8" };
	const char* subPatterns[3] = { "8D 97 38 08 00 00 52 E8", "8D 8F 44 08 00 00 51 E8", "81 C7 50 08 00 00 57 E8" };

	auto findAdds = [](const char* const pats[3], uint8_t* calls[3], uint32_t restOfs[3])
	{
		for (int i = 0; i < 3; i++)
		{
			auto p = hook::pattern(pats[i]);
			uint8_t* at = p.count(1).get(0).get<uint8_t>(0);
			restOfs[i] = *(uint32_t*)(at + 2);
			calls[i] = at + 7;
		}
	};

	uint8_t* plAdds[3];
	uint8_t* subAdds[3];
	findAdds(plPatterns, plAdds, s_plRestOfs);
	findAdds(subPatterns, subAdds, s_subRestOfs);

	ReadCall(plAdds[0], PSVECAdd_orig);
	for (int i = 0; i < 3; i++)
	{
		InjectHook(plAdds[i], BustVecAdd_hook, HookType::Call);
		InjectHook(subAdds[i], BustVecAdd_hook, HookType::Call);
	}

	// cPlAshley::move: cPlayer::move, then tail-jumps to moveBust
	auto pattern = hook::pattern("56 8B F1 E8 ? ? ? ? 8B CE 5E E9");
	uint8_t* plMoveBust = pattern.count(1).get(0).get<uint8_t>(11);
	ReadCall(plMoveBust, cPlAshley__moveBust_orig);
	InjectHook(plMoveBust, cPlAshley__moveBust_hook, HookType::Jump);

	// cSubChar::move's moveBust call
	pattern = hook::pattern("8B 16 8B 42 34 8B CE FF D0 8B CE E8 ? ? ? ? 8B 86 30 08 00 00");
	uint8_t* subMoveBust = pattern.count(1).get(0).get<uint8_t>(11);
	ReadCall(subMoveBust, cSubChar__moveBust_orig);
	InjectHook(subMoveBust, cSubChar__moveBust_hook, HookType::Call);

	spd::log()->info("{} -> Ashley bust hooks applied", __FUNCTION__);
}
