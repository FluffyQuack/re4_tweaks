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

1. **Part dump window** in the debug UI, for the selected character:
   - lists each part's index, parent index, local offset and world position;
   - draws the indices on screen.

   Use it to map the Ganado, Zealot, Soldier, Garrador and Leon skeletons. Known player parts from the decomp:
   - 0 root; 1 and 2 spine/chest; 3 neck; 4 head;
   - 5-10 and 11-16 arms;
   - 18-21 and 22-25 legs (`pl00_mirror`, player.cpp:71).

   For Ganados the code uses head 5 (1-based, i.e. part 4), torso 2, and the arm/leg hit boxes at em10.cpp:3366-3412.
2. **Confirm units.** 1 m = 1000 game units is likely: the sight ranges use 3500-15000.
3. **Confirm frame-rate scaling.** Check how the PC port scales the GC frame timers: the em10 timers are floats on PC, and the player death demo timers need checking too.
4. **Find the PC addresses** for the call sites below, using byte patterns as the rest of the codebase does.

## Phase 2: physics world (`dllmain\Ragdoll.cpp`)

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

- One table per skeleton type with 12 simulated bodies: pelvis, spine/chest, head, upper and lower arms, upper and lower legs. Hands and feet stay locked to the forearm and shin. Each entry has a capsule radius, joint type and limits, and a share of the mass.
- Starting values from Jolt's `Samples/Utils/RagdollLoader.cpp`: 12-body human, capsule sizes, swing/twist limits, hinge knees and elbows.
- Capsule lengths and directions are measured from the live pose when the ragdoll starts, so Ganado size variants and `Scale` fit automatically.
- One shared `RagdollSettings` per skeleton type, built once and stabilised (`Stabilize`).
- Parts that aren't simulated (fingers, jaw, cloth bones) keep their animated local offset under their simulated parent.

## Phase 4: enemy ragdolls

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
