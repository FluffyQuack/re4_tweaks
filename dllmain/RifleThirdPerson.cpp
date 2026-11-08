#include <iostream>
#include <algorithm>
#include <cmath>
#include <cstring>
#include "dllmain.h"
#include "Game.h"
#include "Settings.h"
#include "Patches.h"

// UnscopedRifleThirdPerson: the rifle and semi-auto rifle without a scope attached aim in third person, like the other
// guns, instead of switching to the scope camera.
//
// The rifles have their own weapon routine (PlRifleMove: ready, set, fire, down, reload, next target), built around the
// scope camera: set step 0 starts it, the bolt cycle saves and closes it and opens it again, the reload's end opens it,
// and the shot, the laser marker and the down state's pitch all come from its line (CameraControl::getTrajectory).
// With this on (and no scope attached):
//
//  - The scope camera never starts (those startScope calls are skipped), and the bolt cycle's saveScopeParam /
//    loadScopeParam are skipped too: without a scope camera they'd treat whatever camera is running as one.
//  - getTrajectory returns the line of the rifle itself: from its model's part 0 along its -X, as the game does for
//    every other gun's laser (cObjWep::getMarkerPos). So the shot, the laser dot and the down state's pitch follow
//    the rifle.
//  - In the set state, the common aim control runs (PlWepLockCtrl: stick / mouse pitch and turning, as the handgun's
//    and shotgun's set states do) and the laser sight is drawn.
//  - The rifle's raise carries a camera track (meant to lead into the scope); it's ignored, so the aim camera stays on.
//  - The rifle's aim has no up / down motions (the scope did the pitch), so the upper body bends by the aim pitch:
//    after cPlBody::move (waistMove) sets the waist twist on the spine parts 1 and 2, half the pitch is added to each
//    one's extra rotation (inv_offset.x). Every pose computed after that includes it, the laser's and the rifle's too.

namespace
{
	// How far the aim can pitch (m3r: -1..1 is -90..90 degrees), and how the upper body bends for it
	constexpr float c_maxRate = 0.5f;          // 45 degrees either way
	constexpr float c_bendSign = -1.0f;        // inv_offset.x for a positive m3r (aiming up)
	constexpr float c_halfPi = 1.57079633f;

	// Recoil: the upper body kicks up by this much on a shot and settles back over ~10 frames
	constexpr float c_recoilKick = 0.12f; // radians
	constexpr float c_recoilSettle = 0.25f; // per 30 fps frame

	// The aim line's start in the rifle model's part 0, and its length
	const Vec c_muzzleOfs(228.0f, -24.0f, 29.0f);
	constexpr float c_lineLen = 200000.0f;

	// cPlayer
	constexpr uint32_t c_plStat = 0x468;     // stat flags
	constexpr uint32_t c_plStatScope = 0x10; // F_SCOPE
	constexpr uint32_t c_plWep = 0x7D8;      // cPlWep*
	constexpr uint32_t c_wepObj = 0x34;      // cPlWep::m_pWep (cObjWep*)
	constexpr uint32_t c_wepPitch = 0x28;    // cPlWep::pitch (radians)

	// cPlBody
	constexpr uint32_t c_bodyModel = 0x38;

	// cParts
	constexpr uint32_t c_partsInvOffsetX = 0x128; // extra rotation (with motParts flags 0x40000000)

	// GLOBAL_WK
	constexpr uint32_t c_gWepArc = 0x44;     // the weapon's archive (motions)

	// cDelayF m3r: the aim pitch, as the mot3 blend rate
	struct DelayF
	{
		float val0; // current
		float val1; // target
		float delay;
	};
	DelayF* s_m3r = nullptr;
	void* s_mot3 = nullptr;

	float s_bend = 0.0f; // radians, the whole bend (half on each spine part)
	float s_recoil = 0.0f; // radians, the recoil kick on top
	uint32_t s_bendFrame = 0xFFFFFFFF;

	bool Active()
	{
		if (!re4t::cfg->bUnscopedRifleThirdPerson)
			return false;
		GLOBAL_WK* pG = GlobalPtr();
		if (!pG)
			return false;
		return (pG->weapon_no_4FC0 == 9 || pG->weapon_no_4FC0 == 0xA) && pG->weapon_type_4FC1 == 0;
	}

	bool ScopeCameraOn()
	{
		GLOBAL_WK* pG = GlobalPtr();
		return pG && (pG->flags_STATUS_0_501C[0] & 0x40); // STA_SCOPE_CAMERA
	}

	uint32_t& Stat(cModel* pl)
	{
		return *reinterpret_cast<uint32_t*>(reinterpret_cast<uint8_t*>(pl) + c_plStat);
	}

	cModel* WeaponObject(cModel* pl)
	{
		uint8_t* wep = *reinterpret_cast<uint8_t**>(reinterpret_cast<uint8_t*>(pl) + c_plWep);
		return wep ? *reinterpret_cast<cModel**>(wep + c_wepObj) : nullptr;
	}

	void* WeaponMotion(int idx)
	{
		uint8_t* arc = *reinterpret_cast<uint8_t**>(reinterpret_cast<uint8_t*>(GlobalPtr()) + c_gWepArc);
		return arc ? arc + *reinterpret_cast<uint32_t*>(arc + 4 * idx) : nullptr;
	}

	cParts* GetParts(cModel* model, int idx)
	{
		cParts* p = model->childParts_F4;
		for (; p && idx > 0; idx--)
			p = p->nextParts_F4;
		return p;
	}

	void ClampRate()
	{
		if (!s_m3r)
			return;
		s_m3r->val0 = std::clamp(s_m3r->val0, -c_maxRate, c_maxRate);
		s_m3r->val1 = std::clamp(s_m3r->val1, -c_maxRate, c_maxRate);
	}

	// ---- Game functions ----

	void(__cdecl* PlWepLockCtrl)(cModel* pl, int a);
	void(__fastcall* cPlayer_setLaserSight)(cModel* pl, void* edx, int draw, int noCalc, int c);
	void(__fastcall* cMot3_set)(void* mot3, void* edx, cModel* m, void* m0, void* m1, void* m2, void* seq, uint32_t b, int c,
		uint32_t d, uint32_t e);
	void(__fastcall* cMot3_move)(void* mot3, void* edx, float rate);
	void(__fastcall* cObjWep_setDisp)(void* obj, void* edx, int part, int on);

	// ---- The scope camera ----

	void(__fastcall* CameraControl_startScope)(void* cam, void* edx, Vec* pos, Vec* at);
	void __fastcall CameraControl_startScope_hook(void* cam, void* edx, Vec* pos, Vec* at)
	{
		if (Active())
			return;
		CameraControl_startScope(cam, edx, pos, at);
	}

	void(__fastcall* CameraControl_saveScopeParam)(void* cam, void* edx);
	void __fastcall CameraControl_saveScopeParam_hook(void* cam, void* edx)
	{
		if (!ScopeCameraOn())
			return;
		CameraControl_saveScopeParam(cam, edx);
	}

	void(__fastcall* CameraControl_loadScopeParam)(void* cam, void* edx);
	void __fastcall CameraControl_loadScopeParam_hook(void* cam, void* edx)
	{
		if (!ScopeCameraOn())
			return;
		CameraControl_loadScopeParam(cam, edx);
	}

	// The rifle's raise (ready motion 0x14) carries a camera track, which CameraControl::checkAttachCamera follows
	// (CameraAttachedToMotion) until the scope takes over. In third person there's no scope after it, so the camera
	// dropped back to the aim camera when the raise ended; instead, the track is ignored while raising and the aim
	// camera stays on, as with the other guns.
	void(__fastcall* CameraControl_checkAttachCamera)(void* cam, void* edx);
	void __fastcall CameraControl_checkAttachCamera_hook(void* cam, void* edx)
	{
		if (Active())
		{
			cModel* pl = reinterpret_cast<cModel*>(PlayerPtr());
			// Raising, or the frame it ends on: the set state is entered with the raise still playing (the aim idle only
			// starts in set step 0, next update)
			if (pl && pl->r_no_0_FC == 0 && pl->r_no_1_FD == 6 && (pl->r_no_2_FE == 0 || (pl->r_no_2_FE == 1 && pl->r_no_3_FF == 0)))
				return;
		}
		CameraControl_checkAttachCamera(cam, edx);
	}

	// CameraControl::getTrajectory, replaced: the scope camera's line while it's up (as the original), else the rifle's
	// own line in third-person mode
	void __fastcall CameraControl_getTrajectory_hook(void* cam, void* edx, Vec* p0, Vec* p1)
	{
		if (ScopeCameraOn())
		{
			const uint8_t* scope = *reinterpret_cast<uint8_t**>(reinterpret_cast<uint8_t*>(cam) + 0x710);
			*p0 = *reinterpret_cast<const Vec*>(scope + 0xA8);
			*p1 = *reinterpret_cast<const Vec*>(scope + 0xB4);
			return;
		}
		if (!Active())
			return;

		cModel* pl = reinterpret_cast<cModel*>(PlayerPtr());
		cModel* wep = pl ? WeaponObject(pl) : nullptr;
		cParts* p = wep ? wep->childParts_F4 : nullptr;
		if (!p)
			return;

		const Mtx& m = p->mat_C;
		const Vec& o = c_muzzleOfs;
		*p0 = Vec(m[0][0] * o.x + m[0][1] * o.y + m[0][2] * o.z + m[0][3],
			m[1][0] * o.x + m[1][1] * o.y + m[1][2] * o.z + m[1][3],
			m[2][0] * o.x + m[2][1] * o.y + m[2][2] * o.z + m[2][3]);
		Vec dir(-m[0][0], -m[1][0], -m[2][0]);
		const float len = sqrtf(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z);
		if (len < 1e-6f)
			return;
		const float s = c_lineLen / len;
		*p1 = Vec(p0->x + dir.x * s, p0->y + dir.y * s, p0->z + dir.z * s);
	}

	// ---- The rifle's routine ----

	// Ready step 0 (the draw): the aim starts level, or at the camera's pitch as the other guns do (PlSetLockPitch: a
	// positive pitch is doubled, and m3r is the pitch over 90 degrees)
	void(__cdecl* wep09_r3_ready00)(cModel* pl);
	void __cdecl wep09_r3_ready00_hook(cModel* pl)
	{
		wep09_r3_ready00(pl);
		if (!Active() || !s_m3r)
			return;

		float pitch = 0.0f;
		if (AimStartsFromCamera())
		{
			pitch = CameraAimStartPitch();
			if (pitch > 0.0f)
				pitch += pitch;
			pitch = std::clamp(pitch, -c_maxRate * c_halfPi, c_maxRate * c_halfPi);
		}
		if (uint8_t* wep = *reinterpret_cast<uint8_t**>(reinterpret_cast<uint8_t*>(pl) + c_plWep))
			*reinterpret_cast<float*>(wep + c_wepPitch) = pitch;
		s_m3r->val0 = s_m3r->val1 = pitch / c_halfPi;
	}

	// Set step 0 (the aim idle starts): the three-way blend PlWepLockCtrl moves gets the aim idle on all three, so it
	// doesn't blend in whatever it last held; no scope flag
	void(__cdecl* wep09_r3_set00)(cModel* pl);
	void __cdecl wep09_r3_set00_hook(cModel* pl)
	{
		wep09_r3_set00(pl);
		if (!Active() || !s_mot3)
			return;
		Stat(pl) &= ~c_plStatScope;
		if (void* mot = WeaponMotion(0x15))
		{
			cMot3_set(s_mot3, nullptr, pl, mot, mot, mot, nullptr, 5, 0, 4, 0);
			cMot3_move(s_mot3, nullptr, s_m3r ? s_m3r->val0 : 0.0f);
		}
	}

	// The set state's laser call (setLaserSight(0, 0, 0): computed, not drawn): in third-person mode, the aim control and
	// a drawn laser, as the handgun's set state does
	void __fastcall wep09_r2_set_laser_hook(cModel* pl, void* edx, int draw, int noCalc, int c)
	{
		if (Active() && pl->r_no_2_FE == 1 && pl->r_no_3_FF == 1)
		{
			Stat(pl) &= ~c_plStatScope;
			PlWepLockCtrl(pl, 0);
			ClampRate();
			cPlayer_setLaserSight(pl, edx, 1, 0, 0);
			return;
		}
		cPlayer_setLaserSight(pl, edx, draw, noCalc, c);
	}

	// Fire step 0 (the shot): in the scope the rifle model is hidden for the shot (setDisp(1, 0)); in third person it
	// stays, and the upper body kicks
	void(__cdecl* wep09_r3_fire00)(cModel* pl);
	void __cdecl wep09_r3_fire00_hook(cModel* pl)
	{
		wep09_r3_fire00(pl);
		if (!Active())
			return;
		if (cModel* wep = WeaponObject(pl))
			cObjWep_setDisp(wep, nullptr, 1, 1);
		s_recoil = c_recoilKick;
	}

	// The bolt cycle's start (fire step 2) and the reload reset the aim pitch to level, which the scope camera keeps on
	// its own; in third person the pitch is kept
	void(__cdecl* wep09_r3_fire20)(cModel* pl);
	void __cdecl wep09_r3_fire20_hook(cModel* pl)
	{
		const DelayF keep = *s_m3r;
		wep09_r3_fire20(pl);
		if (Active())
			*s_m3r = keep;
	}

	void(__cdecl* wep09_r2_reload)(cModel* pl);
	void __cdecl wep09_r2_reload_hook(cModel* pl)
	{
		const DelayF keep = *s_m3r;
		wep09_r2_reload(pl);
		if (Active())
			*s_m3r = keep;
	}

	// ---- The bend ----

	// Once per frame: follows the aim pitch while aiming (set, fire, reload, next target), else eases back
	void UpdateBend(cModel* pl)
	{
		const uint32_t frame = GlobalPtr()->frameCounter_530C;
		if (frame == s_bendFrame)
			return;
		s_bendFrame = frame;

		const float dt = GlobalPtr()->deltaTime_70;
		s_recoil -= s_recoil * std::min(1.0f, c_recoilSettle * dt);
		if (fabsf(s_recoil) < 1e-4f)
			s_recoil = 0.0f;

		const uint8_t r2 = pl->r_no_2_FE;
		const bool aiming = Active() && s_m3r && pl->r_no_0_FC == 0 && pl->r_no_1_FD == 6 && (r2 == 1 || r2 == 2 || r2 == 4 || r2 == 5);
		if (aiming)
		{
			s_bend = c_bendSign * (std::clamp(s_m3r->val0, -c_maxRate, c_maxRate) * c_halfPi + s_recoil);
			return;
		}
		s_recoil = 0.0f;
		const float k = std::min(1.0f, 0.3f * dt);
		s_bend -= s_bend * k;
		if (fabsf(s_bend) < 1e-4f)
			s_bend = 0.0f;
	}

	void(__fastcall* cPlBody_move)(void* body, void* edx, int a);
	void __fastcall cPlBody_move_hook(void* body, void* edx, int a)
	{
		cPlBody_move(body, edx, a);

		cModel* m = *reinterpret_cast<cModel**>(reinterpret_cast<uint8_t*>(body) + c_bodyModel);
		if (!m || m != reinterpret_cast<cModel*>(PlayerPtr()))
			return;
		UpdateBend(m);
		if (s_bend == 0.0f)
			return;

		// waistMove has just set these (and their flags)
		for (int idx : { 1, 2 })
		{
			if (cParts* p = GetParts(m, idx))
				*reinterpret_cast<float*>(reinterpret_cast<uint8_t*>(p) + c_partsInvOffsetX) += 0.5f * s_bend;
		}
	}

	template <typename T>
	void HookCall(uint8_t* at, T& orig, T hook)
	{
		ReadCall(at, orig);
		InjectHook(at, hook, HookType::Call);
	}

	template <typename T>
	void HookTableEntry(uint8_t* tblRef, int idx, T& orig, T hook)
	{
		void** tbl = *reinterpret_cast<void***>(tblRef);
		orig = reinterpret_cast<T>(tbl[idx]);
		injector::WriteMemory(&tbl[idx], reinterpret_cast<void*>(hook), true);
	}
}

void re4t::init::RifleThirdPerson()
{
	// Every place first: if one isn't there (a different exe, or another patch got there first), nothing is hooked
	struct Find
	{
		const char* what;
		const char* pattern;
		size_t count;
		uint8_t* at;
	};
	Find finds[] = {
		{ "PlWepLockCtrl", "55 8B EC 83 EC 14 56 8B 75 08 57 BF 01 00 00 00 39 3D", 1 },
		// The rifle's down state: m3r.reset (fst val1, fstp val0), and further on mot3.move(m3r)
		{ "m3r", "D9 15 ? ? ? ? 6A 01 D9 1D ? ? ? ? 6A 01 D9 EE D9 1D", 1 },
		{ "mot3.move", "D9 05 ? ? ? ? 51 B9 ? ? ? ? D9 1C 24 E8 ? ? ? ? C7 86 FC 00 00 00 00 00 02 00", 1 },
		{ "mot3.set", "53 03 C2 50 50 50 56 B9 ? ? ? ? E8", 2 },
		{ "getTrajectory", "55 8B EC A1 ? ? ? ? F6 80 1C 50 00 00 40 74 40 8B 81 10 07 00 00", 1 },
		// Set step 0: startScope (+9)
		{ "set step 0", "6A 00 6A 00 B9 ? ? ? ? E8 ? ? ? ? E8 ? ? ? ? 8B 75 08 83 8E 68 04 00 00 10", 1 },
		// Reload, near its end: startScope (+15)
		{ "reload", "0F 8A ? ? ? ? 6A 00 6A 00 B9 ? ? ? ? E8 ? ? ? ? E8 ? ? ? ? 83 8E 68 04 00 00 10", 1 },
		// Bolt cycle's end: startScope (+9), loadScopeParam (+19)
		{ "bolt cycle end", "33 DB 53 53 B9 ? ? ? ? E8 ? ? ? ? B9 ? ? ? ? E8 ? ? ? ? E8 ? ? ? ? 83 8E 68 04 00 00 10", 1 },
		// Bolt cycle's start: saveScopeParam (+17)
		{ "bolt cycle start", "D9 15 ? ? ? ? B9 ? ? ? ? D9 1D ? ? ? ? E8 ? ? ? ? 8B 75 08 8B CE E8", 1 },
		// The set state: its step table (+10) and its setLaserSight call (+27)
		{ "set state", "0F B6 86 FF 00 00 00 8B 0C 85 ? ? ? ? 56 FF D1 83 C4 04 33 DB 53 53 53 8B CE E8 ? ? ? ? 39 9E 2C 04 00 00", 1 },
		// The ready state's step table (+25)
		{ "ready state", "81 88 24 50 00 00 00 20 00 00 53 56 8B 75 08 0F B6 86 FF 00 00 00 8B 0C 85 ? ? ? ? 56 FF D1 83 C4 04 E8 ? ? ? ? 85 C0 75 ? 80 BE FF 00 00 00 03 74 ? F6 86 68 04 00 00 40", 1 },
		// cPlayer::setLaserSight: cPlBody::move (+14) is its first call
		{ "setLaserSight", "55 8B EC 56 8B F1 8B 8E E4 07 00 00 6A 00 E8 ? ? ? ? 8B CE E8 ? ? ? ? 8B 86 D8 07 00 00 8B 48 34 E8", 1 },
		// Fire step 0: setDisp(1, 0) (+4), hiding the rifle
		{ "fire step 0", "6A 00 6A 01 E8 ? ? ? ? 8D 45 CC 50 8D BE 4C 04 00 00", 1 },
		// CameraControl::Move: checkAttachCamera (+4)
		{ "checkAttachCamera", "56 57 8B CB E8 ? ? ? ? 80 7B 37 11 75", 1 },
		// The mouse aim's per-weapon table (+2): the rifles are off in it, as they only aim through the scope
		{ "mouse aim", "80 B8 ? ? ? ? 00 0F 95 C0 84 C0 75 05 5B", 1 },
	};
	for (Find& f : finds)
	{
		auto pattern = hook::pattern(f.pattern);
		if (pattern.size() != f.count)
		{
			spd::log()->error("{} -> {} not found ({} matches), unscoped rifle third-person aim not applied", __FUNCTION__, f.what,
				pattern.size());
			return;
		}
		f.at = pattern.get(0).get<uint8_t>(0);
	}
	auto at = [&finds](const char* what) { for (Find& f : finds) if (!strcmp(f.what, what)) return f.at; return (uint8_t*)nullptr; };

	// The routine's main table (ready, set, fire, down, reload, next): the weapon routines all dispatch alike, so it's
	// the one whose set state holds the set-state code found above (at +0x15), and the fire state's step table comes
	// from its code (+6: movzx, mov edx, [table + eax*4] at +10)
	void** mainTbl = nullptr;
	{
		auto pattern = hook::pattern("0F B6 86 FE 00 00 00 8B 0C 85 ? ? ? ? 56 FF D1 8B 8E D8 07 00 00 83 C4 04 5E 5D E9");
		for (size_t i = 0; i < pattern.size() && !mainTbl; i++)
		{
			void** tbl = *pattern.get(i).get<void**>(10);
			if (reinterpret_cast<uint8_t*>(tbl[1]) + 0x15 == at("set state"))
				mainTbl = tbl;
		}
	}
	static const uint8_t fireCode[] = { 0x0F, 0xB6, 0x88, 0xFF, 0x00, 0x00, 0x00, 0x8B, 0x14, 0x8D };
	if (!mainTbl || memcmp(reinterpret_cast<uint8_t*>(mainTbl[2]) + 6, fireCode, sizeof(fireCode)) != 0)
	{
		spd::log()->error("{} -> the rifle's routine tables weren't found, unscoped rifle third-person aim not applied", __FUNCTION__);
		return;
	}
	uint8_t* fireTblRef = reinterpret_cast<uint8_t*>(mainTbl[2]) + 6 + 10;

	PlWepLockCtrl = reinterpret_cast<decltype(PlWepLockCtrl)>(at("PlWepLockCtrl"));
	s_m3r = *reinterpret_cast<DelayF**>(at("m3r") + 10);
	s_mot3 = *reinterpret_cast<void**>(at("mot3.move") + 8);
	ReadCall(at("mot3.move") + 15, cMot3_move);
	ReadCall(at("mot3.set") + 12, cMot3_set);

	InjectHook(at("getTrajectory"), CameraControl_getTrajectory_hook, HookType::Jump);

	HookCall(at("set step 0") + 9, CameraControl_startScope, CameraControl_startScope_hook);
	InjectHook(at("reload") + 15, CameraControl_startScope_hook, HookType::Call);
	InjectHook(at("bolt cycle end") + 9, CameraControl_startScope_hook, HookType::Call);
	HookCall(at("bolt cycle end") + 19, CameraControl_loadScopeParam, CameraControl_loadScopeParam_hook);
	HookCall(at("bolt cycle start") + 17, CameraControl_saveScopeParam, CameraControl_saveScopeParam_hook);

	HookTableEntry(at("set state") + 10, 0, wep09_r3_set00, wep09_r3_set00_hook);
	HookCall(at("set state") + 27, cPlayer_setLaserSight, wep09_r2_set_laser_hook);
	HookTableEntry(at("ready state") + 25, 0, wep09_r3_ready00, wep09_r3_ready00_hook);
	HookTableEntry(fireTblRef, 0, wep09_r3_fire00, wep09_r3_fire00_hook);
	HookTableEntry(fireTblRef, 2, wep09_r3_fire20, wep09_r3_fire20_hook);
	wep09_r2_reload = reinterpret_cast<decltype(wep09_r2_reload)>(mainTbl[4]);
	injector::WriteMemory(&mainTbl[4], reinterpret_cast<void*>(wep09_r2_reload_hook), true);
	ReadCall(at("fire step 0") + 4, cObjWep_setDisp);
	HookCall(at("checkAttachCamera") + 4, CameraControl_checkAttachCamera, CameraControl_checkAttachCamera_hook);

	// Mouse aim for the rifles (only the third-person aim gets there for them: PlWepLockCtrl)
	uint8_t* mouseTbl = *reinterpret_cast<uint8_t**>(at("mouse aim") + 2);
	injector::WriteMemory<uint8_t>(mouseTbl + 9, 1, true);
	injector::WriteMemory<uint8_t>(mouseTbl + 0xA, 1, true);

	// cPlBody::move, through its thunk
	uint8_t* bodyMoveThunk = injector::GetBranchDestination(at("setLaserSight") + 14).get<uint8_t>();
	cPlBody_move = reinterpret_cast<decltype(cPlBody_move)>(injector::GetBranchDestination(bodyMoveThunk).get<void>());
	InjectHook(bodyMoveThunk, cPlBody_move_hook, HookType::Jump);

	spd::log()->info("{} -> unscoped rifle third-person aim hooks applied", __FUNCTION__);
}
