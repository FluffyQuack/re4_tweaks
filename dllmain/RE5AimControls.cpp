#include <iostream>
#include <algorithm>
#include <cmath>
#include <cstring>
#include "dllmain.h"
#include "Game.h"
#include "Settings.h"
#include "Patches.h"

// RE5AimControls: aiming with a controller moves the aim the way Resident Evil 5 does.
//
// RE4 (PlWepLockCtrl) moves the aim at a speed proportional to the stick's push on each axis, ramping up to full speed
// over ~0.23 s after the stick leaves the centre (repCtr), and turns the body once the waist reaches the weapon's limit.
//
// RE5 (re5dx9.exe, the cPadEx aim update; see the findings in RE5-aim-controls.md):
//  - The stick: each axis / 32767, then the push is the length of that (at most 1), and nothing at 0.3 or less.
//  - "Turbo": while the stick is pushed at least 0.95, a counter rises to 1 at 3/s (rifles: 2/s), otherwise falls at
//    4/s (rifles: 6/s). The push used is then (0.95 - k) * turbo + k * push / 0.95 (k = 0.5, rifles 0.4), at most 1:
//    anything short of a full push moves the aim at under half speed, and a full push speeds up to full over ~1/3 s.
//  - That push, in the stick's direction, goes through a deadzone of 0.1 on each axis, rescaled to 0..1 at 0.95.
//  - The aim then turns at 57.9 degrees per second times that (both up / down and left / right), times the weapon's
//    speed (machine guns 1.7, others 1.0) and the Aiming Speed option: Slowest 0.8, Slow 0.9, Default 1.0, Fast 1.3,
//    Fastest 1.6.
//
// Here, with a controller: the RE5 movement is applied to RE4's aim (the pitch blend m3r, at 45 degrees per 1.0 as RE4's
// own lock-on aiming uses it, and the waist yaw, turning the body past the weapon's limit), and the original function
// runs with the stick zeroed so its own stick movement adds nothing. Its d-pad aiming, limits, sway and motion blending
// all still run. The sway (PlWepLockRand) is told the aim moved, as the original's stick movement would.

// From ControllerTweaks.cpp: pad 0's left stick before any deadzone (XINPUT_GAMEPAD sThumbLX / LY, up is positive)
extern int RawAnalogLX;
extern int RawAnalogLY;

namespace
{
	// RE5's tuning (sParameter, read from the running game)
	constexpr float c_inDeadzone = 0.3f;     // the stick's push, below which it's centred
	constexpr float c_full = 0.95f;          // a full push
	constexpr float c_axisDeadzone = 0.1f;   // per axis, after the turbo curve
	constexpr float c_degPerSec = 57.9f;     // full speed, both axes
	constexpr float c_speedSetting[5] = { 0.8f, 0.9f, 1.0f, 1.3f, 1.6f }; // Slowest .. Fastest
	constexpr float c_machineGunSpeed = 1.7f;

	struct Turbo
	{
		float up;      // per second, while pushed fully
		float down;    // per second, otherwise
		float partial; // k: the share of the push that isn't turbo
	};
	constexpr Turbo c_turbo{ 3.0f, 4.0f, 0.5f };
	constexpr Turbo c_turboRifle{ 2.0f, 6.0f, 0.4f };

	// RE4: the aim pitch m3r per degree (1.0 is 45 degrees, as PlWepAutoTrack and the lock-on turn aim it at a target)
	constexpr float c_degPerM3r = 45.0f;
	constexpr float c_degToRad = 0.017453292f;

	// cPlayer
	constexpr uint32_t c_plWaistYaw = 0x448; // m_Fwork0: the aim's yaw on the waist
	constexpr uint32_t c_plWep = 0x7D8;      // cPlWep*
	constexpr uint32_t c_wepObj = 0x34;      // cPlWep::m_pWep (cObjWep*)

	struct DelayF
	{
		float val0; // current
		float val1; // target
		float delay;
	};

	void(__cdecl* s_lockCtrl)(cModel* pl, int a2) = nullptr; // the original, through a trampoline
	void(__cdecl* s_lockRand)(cModel* pl, int mflag, float* angX, float* angY) = nullptr;
	int(__cdecl* s_joyKamae)() = nullptr;
	int(__cdecl* s_joyLKamae)() = nullptr;
	int32_t* s_stickX = nullptr; // Joy[0].stickX / Y: -127..127, the left stick after RE4's deadzone
	int32_t* s_stickY = nullptr;
	DelayF* s_m3r = nullptr;

	float s_turbo = 0.0f;
	bool s_moved = false;

	// Diagnostics: the weapon's actual pitch against m3r, to check c_degPerM3r
	uint32_t s_lastLogFrame = 0;
	float s_lastLoggedM3r = 100.0f;

	bool IsRifle(uint8_t wep)
	{
		return wep == 0x09 || wep == 0x0A || wep == 0x28 || wep == 0x2F;
	}

	float WeaponSpeed(uint8_t wep)
	{
		switch (wep)
		{
		case 0x0B: case 0x0C: case 0x1B: case 0x1D: case 0x27: // TMPs, Chicago Typewriter, HUNK's MP5
			return c_machineGunSpeed;
		default:
			return 1.0f;
		}
	}

	float AxisDeadzone(float v)
	{
		const float a = fabsf(v);
		if (a < c_axisDeadzone)
			return 0.0f;
		return copysignf(std::min((a - c_axisDeadzone) / (c_full - c_axisDeadzone), 1.0f), v);
	}

	// The stick, right and up positive, through RE5's curve: each axis -1..1, the turbo updated
	void RE5Stick(float dt, const Turbo& t, float& x, float& y)
	{
		float sx, sy;
		if (LastUsedDevice() == InputDevices::XinputController)
		{
			sx = std::clamp(RawAnalogLX / 32767.0f, -1.0f, 1.0f);
			sy = std::clamp(RawAnalogLY / 32767.0f, -1.0f, 1.0f);
		}
		else
		{
			sx = *s_stickX / 127.0f;
			sy = *s_stickY / 127.0f;
		}

		const float len = sqrtf(sx * sx + sy * sy);
		float push = std::min(len, 1.0f);
		if (push <= c_inDeadzone)
			push = 0.0f;

		s_turbo = std::clamp(s_turbo + (push >= c_full ? t.up : -t.down) * dt, 0.0f, 1.0f);

		x = y = 0.0f;
		if (push == 0.0f)
			return;
		const float p = std::clamp((c_full - t.partial) * s_turbo + t.partial * push / c_full, 0.0f, 1.0f);
		x = AxisDeadzone(sx / len * p);
		y = AxisDeadzone(sy / len * p);
	}

	void LogPitch(cModel* pl)
	{
		GLOBAL_WK* pG = GlobalPtr();
		const uint32_t frame = pG->frameCounter_530C;
		if (frame - s_lastLogFrame < 15 || fabsf(s_m3r->val0 - s_lastLoggedM3r) < 0.1f)
			return;
		uint8_t* wep = *reinterpret_cast<uint8_t**>(reinterpret_cast<uint8_t*>(pl) + c_plWep);
		cModel* obj = wep ? *reinterpret_cast<cModel**>(wep + c_wepObj) : nullptr;
		cParts* p = obj ? obj->childParts_F4 : nullptr;
		if (!p)
			return;
		// The gun's line: its part 0's -X (cObjWep::getMarkerPos)
		const float dx = -p->mat_C[0][0], dy = -p->mat_C[1][0], dz = -p->mat_C[2][0];
		const float elev = atan2f(dy, sqrtf(dx * dx + dz * dz)) / c_degToRad;
		spd::log()->info("RE5AimControls -> weapon 0x{:02X}: m3r {:.2f}, gun pitch {:.1f} degrees", int(pG->weapon_no_4FC0), s_m3r->val0, elev);
		s_lastLogFrame = frame;
		s_lastLoggedM3r = s_m3r->val0;
	}

	void __cdecl PlWepLockCtrl_hook(cModel* pl, int a2)
	{
		if (!re4t::cfg->bRE5AimControls || !isController() || !pl)
		{
			s_lockCtrl(pl, a2);
			return;
		}

		GLOBAL_WK* pG = GlobalPtr();
		const uint8_t wepNo = pG->weapon_no_4FC0;
		const float dt = pG->deltaTime_70 / 30.0f;
		float& waist = *reinterpret_cast<float*>(reinterpret_cast<uint8_t*>(pl) + c_plWaistYaw);

		float x, y;
		RE5Stick(dt, IsRifle(wepNo) ? c_turboRifle : c_turbo, x, y);

		float waistWanted = 0.0f;
		s_moved = false;
		if ((x != 0.0f || y != 0.0f) && (s_joyKamae() || s_joyLKamae()))
		{
			const int setting = std::clamp(re4t::cfg->iRE5AimSpeed, 0, 4);
			const float degPerSec = c_degPerSec * c_speedSetting[setting] * WeaponSpeed(wepNo);

			// Pitch: up raises m3r, unless the game's aim is inverted
			float d = y * degPerSec * dt / c_degPerM3r;
			if (FlagIsSet(SystemSavePtr()->flags_CONFIG_0, uint32_t(Flags_CONFIG::CFG_AIM_REVERSE)))
				d = -d;
			s_m3r->val1 = std::clamp(s_m3r->val1 + d, -1.0f, 1.0f);
			if (s_m3r->delay == 0.0f)
				s_m3r->val0 = s_m3r->val1;

			// Yaw: right lowers the waist angle
			waist -= x * degPerSec * dt * c_degToRad;
			waistWanted = waist;
			s_moved = true;
		}

		// The original, without the stick (d-pad aiming, the waist limit, sway, motion blending)
		const int32_t sx = *s_stickX, sy = *s_stickY;
		*s_stickX = 0;
		*s_stickY = 0;
		s_lockCtrl(pl, a2);
		*s_stickX = sx;
		*s_stickY = sy;

		// What the waist limit cut off turns the body, as the original does with the stick
		if (s_moved)
		{
			pl->ang_A0.y += waistWanted - waist;
			s_moved = false;
			LogPitch(pl);
		}
	}

	// PlWepLockRand: re-centre the sway on the aim when we moved it (else the sway would pull it back)
	void __cdecl PlWepLockRand_hook(cModel* pl, int mflag, float* angX, float* angY)
	{
		if (s_moved)
			mflag |= 1;
		s_lockRand(pl, mflag, angX, angY);
	}
}

void re4t::init::RE5AimControls()
{
	// PlWepLockCtrl
	auto pattern = hook::pattern("55 8B EC 83 EC 14 56 8B 75 08 57 BF 01 00 00 00 39 3D");
	if (pattern.size() != 1)
	{
		spd::log()->error("RE5AimControls -> PlWepLockCtrl not found");
		return;
	}
	uint8_t* fn = pattern.get_first<uint8_t>();

	pattern = hook::pattern("C7 45 EC 00 00 00 00 E8 ? ? ? ? 85 C0 75 0D E8");
	auto pStick = hook::pattern("8D 48 70 DB 05 ? ? ? ? D8 4D F0");
	auto pStickX = hook::pattern("DB 05 ? ? ? ? D8 CB DD 05");
	auto pRand = hook::pattern("8B 45 EC D9 05 ? ? ? ? 8D BE 48 04 00 00 D9 5D F0 57 8D 55 F0 52 50 56 E8");
	if (pattern.size() != 1 || pStick.size() != 1 || pStickX.size() != 1 || pRand.size() != 1)
	{
		spd::log()->error("RE5AimControls -> hook points not found ({} {} {} {})", pattern.size(), pStick.size(), pStickX.size(), pRand.size());
		return;
	}

	uint8_t* kamae = pattern.get_first<uint8_t>(7);
	s_joyKamae = reinterpret_cast<decltype(s_joyKamae)>(injector::GetBranchDestination(kamae).get<void>());
	s_joyLKamae = reinterpret_cast<decltype(s_joyLKamae)>(injector::GetBranchDestination(kamae + 9).get<void>());
	s_stickY = *pStick.get_first<int32_t*>(5);
	s_stickX = *pStickX.get_first<int32_t*>(2);
	s_m3r = *pRand.get_first<DelayF*>(5);

	uint8_t* randCall = pRand.get_first<uint8_t>(25);
	s_lockRand = reinterpret_cast<decltype(s_lockRand)>(injector::GetBranchDestination(randCall).get<void>());
	InjectHook(randCall, PlWepLockRand_hook, HookType::Call);

	// The original runs through a trampoline: its first 6 bytes (push ebp; mov ebp, esp; sub esp, 0x14), then a jump back
	uint8_t* tramp = static_cast<uint8_t*>(VirtualAlloc(nullptr, 16, MEM_COMMIT | MEM_RESERVE, PAGE_EXECUTE_READWRITE));
	if (!tramp)
	{
		spd::log()->error("RE5AimControls -> couldn't allocate the trampoline");
		return;
	}
	memcpy(tramp, fn, 6);
	tramp[6] = 0xE9;
	*reinterpret_cast<int32_t*>(tramp + 7) = int32_t(fn + 6 - (tramp + 11));
	s_lockCtrl = reinterpret_cast<decltype(s_lockCtrl)>(tramp);
	InjectHook(fn, PlWepLockCtrl_hook, HookType::Jump);

	spd::log()->info("RE5AimControls -> hooks applied (PlWepLockCtrl {}, stick {} / {}, m3r {})", (void*)fn, (void*)s_stickX,
		(void*)s_stickY, (void*)s_m3r);
}
