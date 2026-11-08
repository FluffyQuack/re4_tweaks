# Ragdoll plan

Ragdolls on death for humanoid enemies, and for the player on generic deaths only, simulated with Jolt Physics.

References:
- Jolt: `H:\projects\codework\others-github\joltphysics` (v5.6.1, `47398015`)
- GC decomp: `H:\projects\codework\others-github\re4` (line numbers below are from it unless a `dllmain\` path is given)

Decisions so far:
- The ragdoll replaces the animation instantly on death (no blend from the death animation).
- Corpses are not pushed by Leon walking through them (no player collision body).
- Boss melee hits that kill the player through the generic death routine also trigger the ragdoll.

## Scope

**Enemies included:** the Ganado family. Every one of them (villagers, Zealots, Soldiers, Garradors, ids 0x10-0x20 and the Separate Ways ones) runs the same `src/em10/em10.cpp` (`cEm10`). The per-id modules `src/em11..em20/*_set.cpp` only fill motion tables. One code path covers all of them. They are filtered by `id_100`.

**Enemies excluded:** the Merchant, Luis/Ashley NPCs, bosses, Regenerators/Iron Maidens, animals, Novistadors.

**Player:** generic HP deaths only (definition in Phase 5). Leon first. Ada, HUNK and Wesker use Leon's motion table, so they probably share his skeleton. Krauser and Ashley-as-player are later and unverified.

## Phase 0: Jolt in the build

1. Add a submodule `external/JoltPhysics`, pinned to `47398015`.
2. Add a `Jolt.vcxproj` static library to `re4_tweaks.sln`:
   - Win32; sources pulled in with a wildcard (`Jolt\**\*.cpp`).
   - Same runtime library per configuration as `dllmain`. Release is `/MT`; Debug is the default `/MDd`. Jolt's own CMake defaults to the static runtime, which is why we don't build through CMake.
3. Shared `Jolt.props`, imported by both projects, so the library and the mod use identical `JPH_*` defines:
   - SSE2 only (no AVX/AVX2/SSE4/LZCNT/F16C/FMA defines), for old CPUs.
   - No debug renderer, no profiler, no object streams.
   - `JPH_FLOATING_POINT_EXCEPTIONS_ENABLED` off.
   - No GPU compute (DX12/Vulkan/Metal).
4. Smoke test in `re4t::init::Ragdoll()`:
   - `RegisterDefaultAllocator`, `Factory`, `RegisterTypes`.
   - Create a `PhysicsSystem`, step it once, log the result.
   - Confirm the Jolt version/config check passes.

## Phase 1: in-game research tools

1. **Part dump window:** done, as the "Parts (ragdoll)" section of Em Manager (`RagdollDebug.cpp`). For the selected character it:
   - lists each part's index, parent index, length to the parent, local offset and world position;
   - draws the skeleton and indices on screen, and can highlight one part;
   - copies the list to the clipboard or writes it to the log, with the parts' height range.

   **Done:** Leon, a Ganado, a Zealot, a Soldier and a Garrador are mapped (see Phase 3, "Part mapping"). The original task was to map the Ganado, Zealot, Soldier, Garrador and Leon skeletons. Known player parts from the decomp:
   - 0 root; 1 and 2 spine/chest; 3 neck; 4 head;
   - 5-10 and 11-16 arms;
   - 18-21 and 22-25 legs (`pl00_mirror`, player.cpp:71).

   For Ganados the code uses head 5 (1-based, i.e. part 4), torso 2, and the arm/leg hit boxes at em10.cpp:3366-3412.
2. **Units:** confirmed, 1 m = 1000 game units (Leon's parts span 1759).
3. **Frame-rate scaling.** Done for em10 (see PC findings). Still to check: the player death demo timers (`gameDiedemo`).
4. **PC addresses:** done (see PC findings).

### PC findings

From `bio4.exe` (the unpatched `bio4.exe.bak`). The addresses are for reference only; the code finds everything by byte pattern.

| What | Address | How it's found |
|---|---|---|
| `cModel::partsWorldCalc` | `0x65BED0` (jump stub `0x405DA3`) | Already read by MotionFixes |
| `cEm10::move`: `partsWorldCalc` call (enemy hook point) | `0x45B30A` | `8B CE E8 ? ? ? ? 8B CE E8 ? ? ? ? E8 ? ? ? ? 8B CE E8 ? ? ? ? 56 E8`, +2 (unique) |
| `cPlayer::move`: second `PartsWorldPosCalc` call (player hook point) | `0x765174` | `56 E8 ? ? ? ? 8B 8E D8 07 00 00 83 C4 04 E8`, +1 (unique) |
| `cModel::partsFixAdjust` | `0x65C680` (stub `0x4049EE`) | Follows the enemy hook point |
| `PartsWorldPosCalc` | `0x65EDB0` (stub `0x4068BB`) | Follows the enemy hook point |
| `EmGetDmPos` (cdecl: em, pos, ang) | `0x5B2C70` (stub `0x40A952`, 26 callers) | Not patternable: an identical copy at `0x5B31A0` has no callers. Reimplement it from em_sub.cpp:73 (about 60 lines) in Phase 4. |

The enemy sequence on PC is the same as the decomp:
- `test [esi+0x4B0], 0x400000` (work `Be_flg`);
- then neck/waist/slope;
- then `partsWorldCalc`, `0x430510` (ScaleCompress), `0x438240` (BombNeckMove), `partsFixAdjust`, `PartsWorldPosCalc`.

The player sequence is also as in the decomp:
- `partsWorldCalc`, `partsFixAdjust`, `PartsWorldPosCalc`;
- then `EmAtCheck`, `SatMgr.check`, `atari.move`;
- then `PartsWorldPosCalc` (the hook point);
- then `Wep->move` (`[esi+0x7D8]`) and the virtual at `+0x54` (moveCloth).

**Timers on PC.** Die_Lost's `Timer` (`0x4B4`) counts down by `pG->deltaTime` (`[0xC06F3C]+0x70`). The sinking does not: `Compress_y -= 0.028` and `pos.y -= 6.0` run once per update (0x450E7B-0x450EB8), so corpses sink twice as fast at 60fps. That's the game's own behaviour; the frozen ragdoll just rides along with it.

`Ragdoll.cpp` looks up both hook points at startup and logs them. Nothing is patched yet.

## Phase 2: physics world (`dllmain\Ragdoll.cpp`, `RagdollWorld.cpp`)

**Done**, as built:
- The physics system is created with the first ragdoll (not at DLL load).
- Collision pieces:
  - Each `cSat` piece's triangles are baked into world space, with the body at the origin. Each triangle is wound to face along the game's normal for it, because Jolt ignores back faces.
  - A piece whose matrix changes after it was built becomes kinematic and follows the change (current matrix × inverse of the build matrix).
  - Pieces are rebuilt when their slot holds different data, and removed while turned off.
- Stepping:
  - Lazily, at the first hooked pose of each game update (`frameCounter_530C` changed), so it pauses with the game.
  - Fixed 60 Hz substeps, at most 4 per update.
  - Only while ragdolls exist.
- Ragdoll-against-ragdoll collision is always on for now (the setting comes in Phase 6).

The original design notes follow.

- **Units and coordinates:**
  - 1 m = 1000 units, Y-up, gravity 9.81 m/s².
  - Conversion between the game's 3x4 row-major matrices and Jolt's `Mat44`.
  - Scale is stripped before handing a pose to Jolt and restored afterwards (reuse the MotionFixes split/blend helpers).
- **World collision from `SatMgr`:**
  - One static `MeshShape` per `cSat` (`vtx`, `poly_p` indices 0-2), placed with `cSat::mat`.
  - Leave out `EatMgr` (enemy-only barriers).
  - Watch each `cSat` every frame: its transform and its enabled flag. Some move or toggle: room scripts (r31c, r209, r10c, …), `objTrolley`, `objGondola`, `objBull`, `objRobo`. Changed meshes become kinematic bodies; disabled ones are removed from the world.
  - Rebuild everything on room change.
- **Layers:** `WORLD` and `RAGDOLL`.
  - A ragdoll's own limbs never collide with each other (`DisableParentChildCollisions` plus its group filter).
  - Ragdoll-against-ragdoll collision is a setting.
  - No player or living-enemy bodies.
- **Stepping:**
  - Once per game frame, before `EmMgr.move`, in fixed 60 Hz substeps driven by the frame time (`deltaTime_70 / 30`).
  - Paused while the game is stopped (`pG->Stop_flg`, menus).
  - Single-threaded job system.

## Phase 3: skeleton definitions

### Part mapping (measured in game)

Measured with the part dump window on:
- Leon (119 parts);
- a Ganado (id 0x16, 34 parts);
- a Zealot (id 0x11, 37 parts);
- a Soldier (id 0x1D, 37 parts);
- a Garrador (id 0x1C, 38 parts).

**Parts 0-25 are the same skeleton in all five,** with the same local offsets. Only the model scale differs:
- Leon and the Zealot: 1.0;
- the Ganado: about 1.01;
- the Soldier and the Garrador: 1.25 (bone lengths 263 / 330 / 544 instead of 210 / 264 / 435.5).

Lengths and capsule sizes are therefore measured from the live pose at the start, never taken from the table.

- **Units:** Leon's parts span 1759 units, so 1 m = 1000 units.
- **Bind pose:** a T-pose with no rotation on any part. Arm offsets are pure ±X, leg and spine offsets pure ±Y, and the toes point +Z (forward). That makes the bind pose a clean reference for constraint frames and joint limits.
- **Sides:** the character faces +Z with Y up, and -X is the character's right. Confirmed: part 10 is Leon's right hand. So 5-10 and 18-21 are the right side, and 11-16 and 22-25 the left.

| Part | Parent | Offset (bind) | What it is | Ragdoll |
|---|---|---|---|---|
| 0 | model | y ≈ 1114-1132 | hips / pelvis | **Pelvis** body |
| 1 | 0 | +10 Y | waist (waistMove target) | **Lower spine** body (1 → 2) |
| 2 | 1 | +210 Y | chest | **Chest** body (2 → 3) |
| 3 | 2 | +200 Y | neck | **Head** body (3 → head top, about 4 + 170 Y) |
| 4 | 3 | +90 Y | head | Follows 3 (animated local) |
| 5 / 11 | 2 | ∓45, +105 Y | clavicle R / L | Follows the chest (animated local) |
| 7 / 13 | 5 / 11 | ∓127, +42.8 Y | shoulder: upper arm R / L | **Upper arm** bodies (7 → 8, 264 long) |
| 6 / 12 | 5 / 11 | same as 7 / 13 | shoulder twin (deform helper) | Follows 7 / 13 (offset captured at start) |
| 8 / 14 | 7 / 13 | ∓264 X | elbow: forearm R / L | **Lower arm** bodies (8 → 10, 268 long) |
| 9 / 15 | 8 / 14 | ∓154 X | forearm twist | Follows (animated local) |
| 10 / 16 | 9 / 15 | ∓114 X | wrist / hand | Follows (animated local) |
| 17 | 0 | 0 | leg root | Follows the pelvis (animated local) |
| 18 / 22 | 17 | ∓92 X, -130 Y | hip: thigh R / L | **Thigh** bodies (18 → 19, 435.5 long) |
| 19 / 23 | 18 / 22 | -435.5 Y | knee: shin R / L | **Shin** bodies (19 → 20, 439.5 long) |
| 20 / 24 | 19 / 23 | -439.5 Y | ankle: foot | Follows (animated local) |
| 21 / 25 | 20 / 24 | -91 Y, +82-87 Z | toe | Follows (animated local) |

That's 12 bodies: pelvis, lower spine, chest, head, and 2 each of upper arm, lower arm, thigh and shin. It's the same set as the Jolt sample's 12-body human.

**Bone directions in the parts' local frames** (Jolt twist axes):
- right arm: -X;
- left arm: +X;
- legs: -Y;
- spine and neck: +Y.

**Parts beyond 25,** all left to the game (they follow their parents):
- Ganado 26-33: 26 on part 1; 27-33 face/jaw on 4 (30 on 27).
- Zealot 26-36: as the Ganado, plus 34/35 (zero-length, on the knees 19/23; robe helpers) and 36 (on 3; hood).
- Soldier 26-36: as the Ganado, plus 34-36 on the right hand 10 (held weapon parts; 36 reaches 1420 units out).
- Garrador 26-37: as the Ganado, plus 34/35 (claws, on the forearms 8/14) and 36/37 (on the chest).
- Leon:
  - 26-28, 32, 33: face;
  - 29/30: on the chest;
  - 31: on 17 (holster);
  - 34-63: fingers;
  - 64-85: hair bones;
  - 86-109: jacket bones;
  - 110: twin of 3;
  - 111/112, 115/116: on the chest;
  - 113/114: twins of 6/12;
  - 117/118: twins of 8/14.
  
  Hair, jacket and holster are driven by `moveCloth`, which runs after the player hook, so they follow the ragdoll.

**Twin parts** (same parent and offset as a simulated part: 6/12, Leon's 113/114/117/118 and 110) keep their start-of-ragdoll offset to that simulated part, rather than their animated local transform. Otherwise the skin weighted to them would stay behind.

### As built (`RagdollRig.cpp`)

**Done.** All of it is defined in the bind pose (T-pose, no rotations), so a body's frame is its part's frame and a body's transform is its part's world transform without scale.

- **Model check.** A model counts as this skeleton when parts 1-25 have the expected parents and bind offsets. Its scale is measured from thigh and upper-arm lengths in the live pose. One `RagdollSettings` is built per scale (rounded to 0.01) and shared.
- **Capsules** (game units at scale 1):

  | Body | Shape | Radius |
  |---|---|---|
  | Pelvis | across X, half 80 | 100 |
  | Spine | across X, half 70 | 105 |
  | Chest | across X, half 95 | 110 |
  | Head | along Y, half 40 | 95 |
  | Upper arms | elbow to shoulder | 50 |
  | Forearms | elbow to wrist, +90 for the hand | 42 |
  | Thighs | hip to knee | 70 |
  | Shins | knee to ankle, +40 | 55 |

  Mass comes from volume at Jolt's default density, about 69 kg at scale 1.
- **Joints.** Elbows and knees are hinges, 0° in the bind pose:
  - elbows flex forward, -3° to 140°;
  - knees flex backward, -3° to 145°.
  
  The rest are swing-twist, each cone centred on a tilted rest direction because cones are symmetric:

  | Joint | Tilt of the cone | Forward/back | Sideways | Twist |
  |---|---|---|---|---|
  | Spine, chest | 5° forward | 25° | 20° | ±20-25° |
  | Neck | 10° forward | 40° | 30° | ±50° |
  | Shoulders | down and forward | 75° | 85° up/down | ±70° |
  | Hips | 30° forward | 70° (100° forward, 40° back) | 35° | ±25° |

  Friction torque is 1 N·m on swing-twist joints and 0.5 N·m on hinges.
- **Bodies:** linear casting (CCD) for thin walls and fast limbs; friction 0.8, restitution 0, damping 0.1 linear / 0.4 angular.
- **Parts that aren't simulated** keep their start-of-ragdoll offset to the part they hang off (frozen, not animated), so a dead body doesn't keep twitching with its death animation. Twins hang off the simulated part they duplicate.
- **Tested outside the game.** A verbatim copy of `RagdollRig.cpp`, built against Jolt, was put through five drops: T-pose falling back or pushed forward (scale 1.0 and 1.25), tumbling from 2 m, and a hard shotgun-like push. In every case:
  - the body ends up lying flat (pelvis 0.10-0.15 m up), sinking into the floor by at most the 2 cm penetration slop;
  - joints stay within 0.7 mm of each other;
  - it goes to sleep in about 2 s;
  - elbows only bend forward and knees only backward, within their limits.

The original design notes follow.

- One table per skeleton type with 12 simulated bodies: pelvis, spine/chest, head, upper and lower arms, upper and lower legs. Hands and feet stay locked to the forearm and shin. Each entry has a capsule radius, joint type and limits, and a share of the mass.
- Starting values from Jolt's `Samples/Utils/RagdollLoader.cpp`: 12-body human, capsule sizes, swing/twist limits, hinge knees and elbows.
- Capsule lengths and directions are measured from the live pose when the ragdoll starts, so Ganado size variants and `Scale` fit automatically.
- One shared `RagdollSettings` per skeleton type, built once and stabilised (`Stabilize`).
- Parts that aren't simulated (fingers, jaw, cloth bones) keep their animated local offset under their simulated parent.

## Phase 4: enemy ragdolls

### As built (`Ragdoll.cpp`, "Enemy deaths")

Built, not yet tested in game.

- **Trigger.** Every Ganado-family enemy is tracked in the `cEm10::move` hook: its guid, hp and routine as of the last update. The update its hp goes from above 0 to 0 or below, it becomes a ragdoll on the spot.
  - It doesn't if the current or the previous routine is one of these (`r_no_0`-`r_no_1`; for move routines `r_no_1` is the `Em10_R1_move_tbl` index / 2):
    - **1-xx:**
      - Leon-paired: catch / neck hang / backhold / bombhold / dash catch (`0x33`-`0x39`), `UFOCatch` `0x65`, the claw and chainsaw kills on Leon (`0x2E`, `0x30`, `0x32`);
      - carrying Ashley: `TakeAway` `0x3A`, `R300TakeAshley` `0x66`;
      - vehicles and props: cart `0x09`, gatlings `0x23` `0x5F` `0x68` `0x69`, drive `0x47`, catapult / rock push `0x48` `0x49`, bombers `0x50` `0x6A` `0x6C`, dragon `0x52`-`0x54`, barrel `0x55`, gondola `0x57`, drill `0x5C`, trolley `0x61` `0x62`, bull `0x67`;
      - climbing: `0x3C`, `0x40`-`0x42`;
      - scripted: `0x06`, `0x0A`, `0x0B`, `0x4A`, `0x4B`, `0x5A`, `0x5B`, `0x5E`, `0x63`, `0x6B`.
    - **2-xx:** fence `0x05`, ladder `0x06`, take-away `0x0C`, suplex `0x10`, gatling `0x11`, neck break `0x15`.
    - **3-03** (Die_Lost, a despawn), **3-05** (Die_Bomb, gibbed), and all of **4** (scenario).
  - It also doesn't on a ladder or fence (`Be_flg & 0x30000`), when not drawn, or when the model isn't the humanoid skeleton. Every skip is logged with its routine, for tuning.
- **Push.**
  - **Hit point:** a simpler `EmGetDmPos`: the game's recorded point when the hit box has `YAT_FLAG_DMPOS`, otherwise the middle of the hit box. The hit box is only trusted if it lies inside the enemy.
  - **Direction:** from `m_PosFrom` (the shot's origin) to the hit point, with the vertical part scaled by 0.3. Without a usable hit: away from Leon.
  - **Strength by weapon group** (m/s):

    | Weapon | Push |
    |---|---|
    | Shotgun, near (within 6000) | 6 |
    | Shotgun, far | 2.5 |
    | Rifles, magnum, rocket, blasts | 4.5 |
    | Kicks / suplex | 4 |
    | Hand, knife | 1.2 |
    | Flash | 0.3 |
    | Bullets | 2 |

    Everything is multiplied by `g_deathPushScale`.
  - **Split:** 40% as velocity on every body (plus 15% upwards), 60% as an impulse at the hit point on the body of the hit part (or the closest simulated part above it). The impulse is sized for 69 kg × scale³.
- **Root follows the body** while simulating:
  - `pos` x/z is set to the pelvis, and y to the floor under it (a ray against the room collision);
  - `mat` translation and `Motion.Pos_world` are kept in step, so `PartsWorldPosCalc` doesn't move the parts again;
  - `Fix_parts` is cleared, so there's no foot lock.
- **Dissolve.** From the first Die_Lost update, the ragdoll keeps simulating, but every part is drawn lower by how far the game's root has sunk since then, and the root is no longer driven. The game's squash and fade still apply.
- **End.** The ragdoll is removed when the enemy is hidden (`Be_flg & 0x400000`), not drawn, has hp above 0 again, or is freed or reused (guid).
- **Cap.** At most 16 simulate. Past that, the oldest freezes: its last pose is kept and its bodies are released. A frozen corpse still sinks with the dissolve.
- **Debug UI:**
  - an "Enemy death ragdolls" toggle (`g_enemyDeaths`, on);
  - a "Root follows body" toggle (`g_followRoot`, on);
  - a "Death push scale" slider;
  - the simulated / max counts.

**After the first in-game test** (occasional one-frame pose flicker; a grenade into a group dropped the game below 1 fps):
- **Threads.** The game updates on its own thread, and the ImGui debug UI runs on another (the codebase already uses `Game_ScheduleInMainThread` for this). The debug UI was reading the ragdoll list and calling Jolt with no synchronisation. A grenade adding six ragdolls in one update resizes that list, so this is the prime suspect for the hang.

  Fix: one recursive mutex guards all shared ragdoll state and every Jolt call. The hooks take it, and so does every public function.
- **MotionFixes.** Its pre-draw pass re-poses models whose motion has attr `0x4000` (from saved animation poses), and ragdolling models weren't excluded as planned. That's a likely cause of the one-frame flicker. It now skips them.
- **The UI-thread race probably wasn't the trigger after all:** the Em Manager wasn't open during the grenade test, and the build was Release. The lock stays (the race was real).

  Two more defences:
  - **Denormals** are flushed to zero while ragdoll code runs (Jolt only does this on its own worker threads, which we don't use; SSE math on denormals is about 100× slower, and settling bodies decay their velocities towards zero).
  - **Adaptive steps:** after a physics update over 10 ms, the next does one substep at most.

  **Cause found by the diagnostics** (02:08 build):
  - Some updates took 2.9 s and 15.3 s with 0.00 ms of our own time, while a ragdoll's parts jumped 51,241,400 units in Dm_Blow's airborne stage (`02-04-03`).
  - The damage routines move the root themselves (speed, gravity) until it lands. Root-follow put it back under the pelvis every update, so it never landed, the speed kept growing, and the root flew kilometres away. The game's collision sweep across the map is what took seconds.
  
  Fixes:
  - The root only follows in the death routines (`r_no_0` 3), which stay on the ground.
  - It never jumps more than 20 m in one update, or to a non-finite value.
  
  The "pose was changed" check had also run after the enemy's routine, which moves the parts along with the root before they're re-posed, so it reported false alarms. It now runs at the start of the update, on ragdolls whose model hasn't moved yet.
- **Hit impulse** was sized for the whole body (0.6 × push × 69 kg) but applied to one piece, starting a hit forearm at about 40 m/s. It's now capped at 8 m/s on that piece.
- **Measured outside the game:** 8 ragdolls created overlapping around a blast, with those impulses, on a mesh floor with walls. Jolt's worst step was 0.5 ms (average 0.1 ms) and everything slept within 3 s, so Jolt's own cost isn't the problem.
- **Diagnostics:**
  - A ragdoll's pelvis is checked as its next update starts; if it moved more than 30 units since our write, that's logged with the routine and motion attr.
  - Updates slower than 50 ms are logged with our time split up: world sync, physics and steps, poses, active bodies, room pieces and builds.
  - Collision-piece builds over 5 ms are logged.
- **Mercenaries run as Ada (14:35 build):**
  - *"Slow update" lines of seconds with 0.00 ms of ours:* false alarms. The watchdog's clock only ran while ragdolls existed, so the first update after one started measured the whole gap since the last one. It now restarts whenever there are none, or frames went by without an update of ours (pause, menus).
  - *Corpse flickering between positions (next to a ladder), and pose changes of 1000-6000 units logged in Die_Normal (`03-02`) and as it hands over to Die_Cramp (`03-00-00-01`):* after we pose a dead enemy, cEm10::move runs EmAtCheck, which pushes the root out of other characters and objects (a crate's or ladder's box: up to metres, to its edge) and then moves every part by that push (PartsWorldPosCalc). The ragdoll doesn't collide with objects, so the body can lie inside one. A dead ragdoll now has its character collision turned off (atari flag 0x200) when it's posed, as the game itself does in Die_Cramp; EmAtCheck then returns without touching it.
  - *Bodies flying up (Ada's head-stun melee, some rifle kills on a downed enemy):* the start velocity was each part's movement in the update it died in, capped at 15 m/s. That update often snaps to a new animation (a hit reaction or death motion starting from another pose), which reads as tens of m/s. Now it's the slower of that update's and the previous one's movement, capped at 6 m/s, and at most 3 m/s upwards. The start log line has the pelvis speed (raw and used), the fastest part and the push; a "died in" line has the routine, weapon, hit part and push direction.
- **Gun kills dropped bodies on the spot** (after the Mercenaries fixes): a handgun kill pushed at 2 m/s, 40% of it on every body and 60% as an impulse capped on the hit piece, and the feet's friction ate what reached the legs. Now: bullets 3.5 m/s, rifles / magnum / rocket / blasts 6, shotguns 7.5 near (3.5 far), kicks 2.5; 75% goes on every body, weighted towards the upper body (chest and head 1.25, pelvis 0.8, thighs 0.55, shins 0.3) so the body topples, 25% as the hit impulse, and 10% upwards.
  - If the game stays below 2 fps for 5 updates with ragdolls out, they're all stopped and enemy death ragdolls are turned off.

The original design notes follow.

### Trigger

The frame that HP goes from above 0 to 0 or below, checked when `cEm10::move` returns. Not on `r_no_0 == 3`: most kills spend 1+ seconds in a damage routine first (Dm_Blow, Dm_KnockOut, Dm_Roof … then `setRno(3,0,…)`, e.g. em10.cpp:15029-15830).

- Plaga emergence filters itself out: `em10LostHead(em,2,…)` sets hp = 3000 within the same `em10DmCk` (17656).
- The delayed headless death (hp held at 1, `Die_wait = 90`, 1287/1539) triggers later on its own, when hp finally hits 0.

### Exclusions (body keeps its animation)

| Case | How to identify |
|---|---|
| Explosion (body hidden after 3 frames, gore is effects only) | Die_Bomb `r_no_0==3 && r_no_1==5`, or the explosive branches at 1672/1692/1764/2706 |
| Paired with Leon: neck hang 0x34, back hold 0x37, bomb hold 0x38 | the corresponding routines |
| Paired with Leon: suplex Dm_FS 0x10, Dm_NeckBreak 0x15, knee kick / Showtay / heel 0x12-0x14 | the corresponding routines; decide per move, the ones not glued to Leon may be fine |
| Truck driver (Drive 0x47, R0=1) | the routine |
| Gatling 209 (0x5F) | the routine |
| Drill 212 (0x5C) | the routine |
| Gondola rider (goes straight to Die_Lost) | the routine |
| Scripted deaths | R321DeadBody, set 0x3F pre-placed corpses, Goto mode 5, R100Cliff, R303FireDash, `setLost()` |
| Ladder / fence | Dm_Ladder (`Be_flg & 0x10000`), Dm_Fence (`Be_flg & 0x20000`); maybe allowed once the enemy is off the ladder, to test |

### Start

- Pose: the current part matrices.
- Per-body velocity: current vs previous-frame transforms.
- Push:
  - **Point:** `EmGetDmPos` (em_sub.cpp:73) gives the impact point on the hit box.
  - **Direction:** impact point minus `m_DmgInfo.m_PosFrom` (that's the shot origin or blast centre).
  - **Body piece:** `m_pDamageYarare->parts_no` (1-based).
  - **Strength:** a per-weapon table keyed by `m_Wep`, scaled down with distance (`len` is squared distance; the code's near threshold is 36e6).
  - **Fire and environment kills:** don't go through `set()`, so they get no push or a small default.

### Each frame

- Hook the `partsWorldCalc` call inside `cEm10::move` (em10.cpp:2554). Run the original, then overwrite the simulated parts.
  - Write both `mat` and `world` (effects, sounds, `em10CoreBreak`, `em10BombNeckMove`, the camera and blood checks read `world`).
  - Keep `world_old` consistent.
- What runs after the hook keeps working on the ragdoll pose:
  - `em10ScaleCompress`, `em10BombNeckMove` (headless neck stump, parts 0x24/0x1B), `partsFixAdjust`, `PartsWorldPosCalc`;
  - weapons, caps, hoods, cloth, belts and attached effects, which read the parts during their own updates.
- **Root:** the death motions move `em->pos` (`Mot_attr` bit 0). Every frame set `pos` XZ to the pelvis and `pos.y` to the floor under it, and set `Motion.Pos_world = pos`, so `PartsWorldPosCalc` doesn't shift the ragdoll a second time. Items drop at `pos` and ground effects use it, so it must follow the body.
- **MotionFixes:** ragdolling models are skipped by its 60fps blending.

### Dissolve and end

- When Die_Lost starts (`r_no_0==3 && r_no_1==3`):
  - stop simulating and freeze the pose relative to `em->mat`;
  - each frame, write root × frozen pose;
  - the game's own sinking (`pos.y -= 6`, Die_Lost writes `em->mat`) and squash (`em10ScaleCompress`) then apply on top, and the fade (`invisible_factor`) is untouched.
- Corpses kept permanently (stage 1 room 0, some types/sets, 15942-15969) just go to sleep.
- **Stop and remove the ragdoll when any of these happens:**
  - work `Be_flg_4B0 & 0x400000` (hidden; the pose block is skipped);
  - `be_flag & 2` cleared;
  - hp > 0 again (`setReset` reuse, parasite);
  - `be_flag & 1` cleared;
  - `guid_F8` changed;
  - room change.
- em10 never destroys a corpse; it only hides it, and there's no corpse cap. Ours caps active ragdolls (default 16): when the cap is hit, the oldest freezes.
- **Safety fallback:** if the pelvis drops well below the last floor it touched, freeze the ragdoll.

## Phase 5: player generic deaths

### When to decide

At the start of `cPlayer::move`, or in `Pl_R0_Die` with `r_no_1 == 0`. Not inside `PlSetDamage`: a boss can override the routine or decapitate later in the same `EmMgr.move`, which runs before the player's update (game.cpp:600-608).

### Generic test

All of these:
- `pl_life <= 0`;
- `r_no_0 == 2`, or `r_no_0 == 1` with (`r_no_1 == 0 && r_no_3 == 6`) or (`r_no_1 == 1 && m_Work0 == 1`);
- not in an event (`stat & F_EVENT`);
- the head is still attached (`Body->m_pHead != 0`). Scythe/axe/claw decapitation stays in routine 2 but is visually special.

Special deaths fall outside this test on their own:
- routine 4: grabs, chainsaw, El Gigante, boulder, laser, mine cart;
- routine 5: events and traps;
- routine 0 / `r_no_1` 0xF: boat and jet ski;
- Ashley's death: Leon goes to routine 6 with HP above 0.

### Boss melee

Verdugo, U-3, Krauser, Mendez, Saddler and the truck also kill through the generic routine with some attacks. Decided: these trigger the ragdoll too, so there's no need to track the attacker.

### Hook point

Inside `cPlayer::move`, after the second `PartsWorldPosCalc` and before `Wep->move` (player.cpp:379-380). The weapon in hand, cloth (hair, jacket, holster; stateful, never run twice) and attached effects then follow the ragdoll.

- The death collision cylinder follows part 3 (`atari.m_parts_no = 4`). `SatMgr.check` runs before the hook, so drive `pos` from the pelvis as for enemies.

### Game over

- `gameDiedemo` (game.cpp:1074):
  - "YOU ARE DEAD" at 90 frames;
  - the Continue menu at 360 frames, which sets `Stop_flg` and freezes the simulation;
  - the game keeps updating until then.
- **Death camera** (`CameraLookAt`, cam_extra.cpp:1010): it targets `world` of part 1 or 2 from a frozen camera position. That works with the ragdoll as long as `world` is written.
- Check at runtime whether motion 0x4C has attach-camera channels.

## Phase 6: settings, debugging, testing

- **Settings:** a `[RAGDOLL]` section in `settings.ini`, plus `Settings.cpp/.h` and `cfgMenu.cpp` entries:
  - `bEnemyRagdolls`, `bPlayerRagdoll`;
  - `fRagdollImpulseScale`, `iMaxRagdolls`;
  - `bRagdollsCollideWithEachOther`.
- **Debug view:** capsules and the world collision mesh, drawn with the `DebugDisplay.cpp` line helpers.
- **Test cases:**
  - a large village fight;
  - stairs, slopes, ledges and roofs;
  - water;
  - close-range shotgun, handgun at distance, grenades (excluded), knife;
  - kicks and suplexes, headshots and the delayed headless death, Plaga emergence;
  - burning;
  - permanent-corpse rooms, leaving a room with corpses, `setReset` rooms (r104, r214);
  - save/load;
  - 30/60fps and frame drops;
  - ~15 bodies at once;
  - player: generic death, decapitation (excluded), chainsaw (excluded), El Gigante (excluded), boss melee (ragdoll).

## Risks

- PC binary differences from the GC decomp: inlining, float timers, offsets. Every call site needs checking in the PC exe.
- Bodies clipping into props (barrels, crates, tables): they aren't in the SAT. Optional later: kinematic shapes from object collision cylinders.
- `pos` driven from the pelvis versus `SatMgr.check` / `EmAtCheck` pushing it, until `Be_flg 0x400000`.
- A dying player whose last hit leaves HP 0 in routine 0 (rare `setDamage` edge case) gets no ragdoll. That's acceptable.
