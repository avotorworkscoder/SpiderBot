# SpiderBot Development Session Summary
**Author:** Amit Parihar  
**Session Date:** March 25 – April 25, 2026  
**Starting Version:** V2.3 (final tested code)  
**Ending Version:** V3.5 (Phase 3 Continuous Bézier Engine)

---

## Files Produced This Session

| File | Description |
|------|-------------|
| `spiderBotV2_4.ino` | Added 9 named movement functions + PS2 controller integration |
| `spiderBotV2_5.ino` | Full optimisation — unified leg arrays, single-step execution, no delay in gait |
| `spiderBotV2_6_final.ino` | Non-blocking gait engine, command queue, joystick control, plug-and-play gaits |
| `spiderBotV3_2.ino` | Added ArduinoOTA for wireless deployment without physical connection |
| `spiderBotV3_3.ino` | Phase 1: MPU-6050 levelling, IK height bugfixes, and Hexapod Tactile landing |
| `spiderBotV3_4.ino` | Phase 2: True 3D Roll/Pitch Rotational Matrices replacing linear levelling |
| `spiderBotV3_5.ino` | Phase 3: Parametric continuous streaming engine with 4-point Bézier Z-curves |

---

## What We Changed Version by Version

### V2.3 → V2.4
- Added **9 named movement functions**: `walkForward()`, `walkBackward()`, `turnLeft()`, `turnRight()`, `rotateLeft()`, `rotateRight()`, `standStill()`, `increaseHeight()`, `decreaseHeight()`
- Added `tripodTurn(strideL, strideR)` — asymmetric stride for arc walking
- Added `bodyZ` and `heightOffset` for live height control
- Added `RobotState` enum expanded with `STATE_TURN_LEFT`, `STATE_TURN_RIGHT`, `STATE_ROTATE_LEFT`, `STATE_ROTATE_RIGHT`
- Added `strideScale` field to `Leg` struct (compensates for 45° corner coxa mounting angle)
- Recenter step added to both `tripodStep` and `rotateStep` — prevents foot position drift each cycle
- All recenter steps changed to move **one tripod group at a time** (never all 6 legs at once — stability fix)

### V2.4 → V2.5 (Optimisation pass)
- **Merged `legs1[]` and `legs2[]` into one `legs[6]` array** with `walkY` and `rotY` fields per leg
- **Merged `moveLeg()` + `moveFootXYZ()`** into a single function — one call, one stack frame
- **Replaced `tripodStep` / `rotateStep` / `tripodTurn`** (3 copy-pasted functions, ~120 lines) with one **`runGait(swingY, dragY)`** engine that takes two lambdas
- **Removed `heightOffset`** — redundant, `bodyZ` already tracked the same thing
- **Removed `standHome()`** — was an alias for `standStill()`, collapsed into one function
- **Added `btLog()`** helper — prints to both BT and USB Serial in one call
- **Replaced `increaseHeight()` / `decreaseHeight()`** duplicate logic with `adjustHeight(float delta)`
- **Single-step execution model**: `handleCommand()` calls movement functions directly — no state machine dispatching from `loop()`
- **Removed movement `switch` from `loop()`** — loop only reads input + polls battery
- `RobotState` enum replaced with `LastCmd` enum (status display only, not used to drive motion)

### V2.5 → V2.6 (Non-blocking engine + new features)
- **Removed all `delay()` from gait execution** — replaced with millis()-based phase timer
- **Added command queue** — 8-slot circular buffer, no inputs lost during a step
- **Added joystick control** — BT command `j80,30` blends 80% forward + 30% left turn in one gait call
- **Added plug-and-play gait table** — `GaitDef` struct + `GAIT_TABLE[]` array
- **Added 4 gait styles**: Tripod Walk (0), Crab Walk (1), Diagonal Walk (2), Wave Gait (3)
- `runGait()` kept **completely unchanged** — new engine calls `sendPhase()` + `gaitTick()` instead
- Added `parseInput()` — handles single-char and multi-char BT commands (`j...`, `g...`)
- `startGait(continuous)` replaces the old direct function calls

### V2.6 Cleanup (variable audit)
- **Removed `currentGait`** — was always a copy of `activeGait`. Replaced all references with `activeGait` directly
- **Removed `tSwing`, `tPlant`, `tCenter`** — pre-computed from `moveTime` but never changed mid-cycle. Replaced with inline `moveTime`, `moveTime/3`, `moveTime/2`
- **Renamed `gaitContinue` → `looping`** — clearer name, same purpose
- **`startGait(gaitIdx, continuous)` → `startGait(continuous)`** — `gaitIdx` parameter removed since it was always `activeGait`

---

## Bugs Fixed This Session

| Bug | Location | What was wrong | Fix |
|-----|----------|----------------|-----|
| Missing comma | `legs1[]` init, Mid-Left row | `{ 3, 10 -3, ...}` — C++ silently computed `10-3=7`, wrong restX and restY | Added comma: `{ 3, 10, -3, ...}` |
| Wrong `coxaDir` index | `tripodStep` (6 lines) | `legs1[i].coxaDir` used instead of `legs1[A[i]].coxaDir` — `i` goes 0,1,2 but group indices are 0,2,4 | Changed to `legs1[A[i]].coxaDir` and `legs1[B[i]].coxaDir` |
| `rotateStep` was translating not rotating | `rotateStep()` | All legs swung `+yawOffset` in the same direction — body slid sideways | Multiplied by `coxaDir` so left/right sides oppose each other |
| Recenter moved all 6 legs at once | `tripodStep`, `rotateStep` | All 6 legs lifted simultaneously — no stable tripod support | Split into two groups: recenter A first (B planted), then recenter B |
| `rotateStep` used X-axis offset for yaw | `rotateStep()` V2.1/V2.2 | `restX + sign * yawOffset` — X is reach direction, not rotation | Changed to Y-axis opposing swings using `swingDir()` per side |
| `tripodStep` Phase 2 sent wrong stride | `tripodStep()` V2.1 | Phase 2 sent `+strideY` to both groups — they mirrored each other and spun | Phase 2 now correctly has B swing `+strideY` and A drag `-strideY` |
| Version string mismatch | `btCallback` | Displayed `V2.2 Ready!` in a V2.3 file | Updated to match actual version |
| `rotateStep` missing `coxaDir` multiplication | `rotateStep()` V2.3 | `yawOffset` applied to all legs equally — no differential torque | Multiplied by `legs2[idx].coxaDir` so left legs go one way, right the other |

---

## Final Variable Set (V2.6)

### Leg struct fields
| Field | Type | Meaning |
|-------|------|---------|
| `id` | `uint8_t` | LSC-32 base servo ID (coxa=id, femur=id+1, tibia=id+2) |
| `restX` | `float` | Foot reach distance outward (cm) |
| `walkY` | `float` | Y rest position for forward/backward/turn gaits |
| `rotY` | `float` | Y rest position for rotate-in-place (all 0 = symmetric) |
| `restZ` | `float` | Current height — `adjustHeight()` updates this live |
| `coxaDir` | `int` | +1 = left-side leg, -1 = right-side leg |
| `strideScale` | `float` | Mounting angle correction (1.0 = horizontal, 1.41 = 45° corner) |

### Gait engine state
| Variable | Type | Meaning |
|----------|------|---------|
| `activeGait` | `uint8_t` | Which `GAIT_TABLE[]` entry is active (0–3). Change this to switch style. |
| `gaitPhase` | `GaitPhase` | Current phase: IDLE / SEND_A / PLANT_A / SEND_B / PLANT_B / RECENTER |
| `looping` | `bool` | true = step repeats continuously; false = run once then IDLE |
| `phaseStart` | `unsigned long` | millis() timestamp when current phase began |
| `jFwd` | `float` | Joystick forward/back blend (-1.0 to +1.0) |
| `jTurn` | `float` | Joystick turn blend (-1.0 right arc to +1.0 left arc) |

### Runtime parameters
| Variable | Type | Default | Meaning |
|----------|------|---------|---------|
| `moveTime` | `uint16_t` | 500 ms | Duration per gait phase. Lower = faster. |
| `bodyZ` | `float` | -9.0 cm | Current body height. Updated by `adjustHeight()`. |

### Command queue
| Variable | Meaning |
|----------|---------|
| `cmdQueue[8]` | Circular buffer holding up to 8 pending single-char commands |
| `qHead` | Read position (pops from here) |
| `qTail` | Write position (pushes here) |
| `qCount` | Number of commands currently waiting |

---

## Hardware Configuration (unchanged throughout)
- **ESP32** built-in Bluetooth (SPP) — device name: `SpiderBot`
- **LSC-32** servo controller on UART2 — RX=16, TX=17
- **6 legs**, tripod groups: A={0,2,4}, B={1,3,5}
- **Coxa mounting**: middle legs horizontal (strideScale=1.0), corner legs at 45° (strideScale=1.0 currently — theoretical is 1.41, tune if drift persists)
- **BT App**: Serial Bluetooth Terminal — Line ending → Newline `\n`

---

## Coding Standard Decided This Session

Going forward, all code must follow this style:

1. **Well structured** — sections ordered by call dependency (lower-level functions always above what calls them)
2. **Simple and direct** — no redundant variables, no duplicate logic, no dead code kept "for reference"
3. **Understandable at first glance** — section headers explain *what and why*, not just what
4. **Comments explain the why** — not `// increment i` but `// wrap around to start of buffer`
5. **Variable names are self-explanatory** — `looping` not `gaitContinue`, `COXA` not `L1`

---

## Next Steps (Next Session)

### 1. Physical tuning
- [ ] Test `strideScale` on corner legs — theoretical value is **1.41** but currently set to **1.0**. Walk straight and increase from 1.0 toward 1.41 in steps of 0.1 until the bot tracks straight
- [ ] Tune `walkY` values per leg if any leg still drifts during walk
- [ ] Test all 4 gait styles (`g0`–`g3`) on the physical bot and note which work well
- [ ] Verify `rotateLeft` / `rotateRight` actually spins cleanly in place

### 2. PS2 Controller integration (from V2.4 draft)
- [ ] Merge PS2 controller input (`PS2X_lib`) into V2.6 cleanly
- [ ] Map D-pad to movement functions, L1/R1 to rotate, △/× to height, L2/R2 to speed
- [ ] PS2 + BT should work simultaneously (BT as override/debug, PS2 as primary control)

### 3. Joystick app
- [ ] Build a simple custom Android app with a real on-screen joystick that sends `j<fwd>,<turn>` continuously while the stick is held
- [ ] Or: configure an existing app (Joystick BT Commander) to output the same format

### 4. Gait improvements
- [ ] Test Wave Gait (`g3`) — slowest but most stable, good for rough terrain
- [ ] Consider adding a **ripple gait** where each individual leg steps in sequence (6-beat cycle) for maximum stability
- [ ] Consider adding a **bounce gait** — all legs lift slightly at the same time for a stationary shimmy/dance move

### 5. Sensors (future)
- [ ] Add ultrasonic sensor (HC-SR04) on front for obstacle detection
- [ ] Auto-stop or auto-turn when obstacle detected within threshold distance
- [ ] Consider MPU-6050 IMU for body levelling on slopes

---

## March 26, 2026 — V2.7 Development

### Session Start
- **Starting Version:** V2.6 (final clean output)
- **File:** `spiderBotV2.7/spiderBotV2.7.ino`

### V2.6 → V2.7: Crab Function Consolidation

- **Merged 4 crab Y functions into 2:** `crabFwdSwingY`/`crabFwdDragY` + `crabBwdSwingY`/`crabBwdDragY` → `crabSwingY`/`crabDragY`. Direction controlled by `jFwd` sign (+1 = forward, -1 = backward), same pattern as tripod.
- **Merged 4 crab X functions into 2:** `crabLeftSwingX`/`crabLeftDragX` + `crabRightSwingX`/`crabRightDragX` → `crabSwingX`/`crabDragX`. Direction controlled by `jFwd` sign.
- **Gait table reduced from 7 to 5 entries** (then further to 4 in V2.8).
- **Crab movement functions updated:** `crabBackward()` uses gait 1 with `jFwd = -1.0`, `crabRight()` uses gait 4 with `jFwd = -1.0`.

### V2.7: Joystick Crab Mode Integration

Three design options were evaluated:

1. **Option 1 — Dominant Axis Selection** (implemented first): Pick the larger joystick axis to select crab gait. Simple, no engine changes.
2. **Option 2 — Combined 2D Crab Gait** (implemented, replaced Option 1): New `jStrafe` variable for independent X-axis scaling. Unified crab gait with both X and Y strides. True omnidirectional crab movement from joystick.
3. **Option 3 — Reuse jTurn** (rejected): Less clear, dual meaning for `jTurn`.

### V2.7: New `jStrafe` Variable and Independent Scaling

- Added `jStrafe` — independent X-axis scale for crab mode
- `sendPhase()` and `plantGroup()` split scaling: `scaleY = jFwd`, `scaleX = jStrafe` in crab mode
- Joystick parser maps `rawFwd → jFwd`, `-rawTurn → jStrafe` in crab mode
- Status report (`?` command) now shows `jStrafe`

### V2.7: Stride Amplitude Fix

- **Bug:** `j50,-50` showed mostly rightward movement, not diagonal
- **Cause:** Y stride was 1.0 cm but X stride was 3.0 cm — 3:1 mismatch
- **Fix:** Equalized Y stride to 3.0 cm to match X

### V2.7: `indepXY` Flag and Diagonal Walk

- **Added `bool indepXY` to `GaitDef` struct** — tells engine to use independent X/Y scaling vs differential mixing
- **Diagonal Walk (g2) repurposed:** Now uses `tripodSwingY`/`tripodDragY` for Y + new `walkSwingX`/`walkDragX` for X, with `indepXY = true`
- **Old `diagSwingY`/`diagDragY` removed** (had per-leg-pair bias, not useful with independent scaling)
- **Engine checks `g.indepXY`** instead of `crabMode` for scale splitting
- **Joystick respects active gait:** checks `GAIT_TABLE[activeGait].indepXY`, doesn't force `activeGait = 0`
- **Result:** `g2` + diagonal joystick = true straight-line diagonal movement (not arc turn)

### Final Gait Table (V2.7)

| Index | Name | swingX | dragX | swingY | dragY | indepXY |
|-------|------|--------|-------|--------|-------|---------|
| 0 | Tripod Walk | defaultX | defaultX | tripodSwingY | tripodDragY | false |
| 1 | Crab Walk | crabSwingX | crabDragX | crabSwingY | crabDragY | true |
| 2 | Diagonal Walk | walkSwingX | walkDragX | tripodSwingY | tripodDragY | true |
| 3 | Wave Gait | defaultX | defaultX | waveSwingY | waveDragY | false |

### Documentation Created (March 26)

| File | Description |
|------|-------------|
| `tripod_walk_explained.md` | Deep dive: 2 tripod groups, differential mixing, 5-phase engine, timing breakdown |
| `crab_walk_explained.md` | Deep dive: crab stance geometry, unified 2D gait, independent scaling, joystick mapping |
| `diagonal_walk_explained.md` | Deep dive: true diagonal vs arc turn, walkSwingX/DragX, comparison table |
| `wave_gait_explained.md` | Deep dive: 6-phase theory, why V2.7 wave was fake (still tripod engine), implementation plan |
| `crab_joystick_options.md` | Design options for joystick+crab integration (3 approaches compared) |

---

## March 26, 2026 — V2.8 Development

### Session Continuation
- **Created V2.8** by copying V2.7 folder and renaming `.ino` file
- **File:** `spiderBotV2.8/spiderBotV2.8.ino`

### V2.7 → V2.8: True Wave Gait (6-Phase Engine)

- **Added `WAVE_SEQ[6] = { 2, 1, 0, 5, 4, 3 }`** — wave order: rear→front left, then rear→front right
- **Added `WAVE_STEP` and `WAVE_PLANT` to `GaitPhase` enum**
- **Added `wavePhaseIdx`** — tracks which leg (0–5) in the wave sequence
- **Implemented `waveStep()`:** Lifts 1 leg to front of stride, interpolates 5 dragging legs based on phases since each last swung
- **Implemented `wavePlant()`:** Plants the swing leg back down
- **Updated `startGait()`:** Detects `activeGait == 3` → routes to `WAVE_STEP` instead of `SEND_A`
- **Updated `gaitTick()`:** Two new cases for `WAVE_STEP` → `WAVE_PLANT`, cycling through all 6 legs
- **Simplified wave stride functions:** Removed per-leg-pair bias, uniform 2.5 cm stride

### V2.8: Bug Fix — Walk Functions Overriding Gait

- **Bug:** `g3` then `F` still ran tripod gait
- **Cause:** `walkForward()` hard-coded `activeGait = 0`, overriding the user's gait selection
- **Fix:** Removed `activeGait = 0` from all walk/turn/rotate functions. They now only set `jFwd`/`jTurn` and call `startGait()`. The gait is controlled entirely by the `g` command.

### V2.8: Updated Header and Comments

- All version strings updated: header, status report, BT welcome, serial print
- Section 3 comment updated for indepXY and wave engine
- Section 5 comment rewritten: documents both tripod and wave engine paths
- Section 6 comment updated for unified crab and diagonal descriptions

### Final Variable Set Additions (V2.8)

| Variable | Type | Meaning |
|----------|------|---------|
| `jStrafe` | `float` | Independent X-axis scale (-1.0 to +1.0), used when `gait.indepXY = true` |
| `wavePhaseIdx` | `uint8_t` | Current leg index (0–5) in `WAVE_SEQ` during wave gait |
| `GaitDef.indepXY` | `bool` | true = X/Y scaled independently (jFwd, jStrafe); false = differential (jFwd + jTurn×coxaDir) |

---

## March 27, 2026 — Research & Planning

### Rough Terrain Gaits Analysis

Evaluated 6 additional gaits for terrain navigation:

| Gait | Legs Down | Speed | Stability | Implementation Effort |
|------|-----------|-------|-----------|----------------------|
| High-Step Wave | 5 | ★ | ★★★★★ | ~5 min (change LIFT_H) |
| Tetrapod | 4 | ★★ | ★★★ | ~1 hr (3-phase engine) |
| Ripple | 4 | ★★ | ★★★★ | ~2 hr (overlapping timing) |
| Adaptive Tripod | 3 | ★★★ | ★★+ | ~2 hr (per-leg lift) |
| Body-Shift Walk | 3 | ★★ | ★★★★ | ~3 hr (CoG management) |
| Free Gait | 3–5 | — | ★★★★★ | Days (needs sensors) |

### Industrial Upgrade Roadmap

Created comprehensive roadmap covering 8 areas:

1. **Sensors** — IMU (MPU6050), foot contact switches, current sensing, distance sensors
2. **Body Stabilization** — Auto-leveling on slopes using IMU pitch/roll
3. **Terrain-Adaptive Gaits** — Surface classification, reactive stepping
4. **Mechanical** — Rubber foot tips, sealed design, smart servos (LX-16A/Dynamixel)
5. **Power** — Better battery system, power-efficient gait auto-switching
6. **Autonomous Navigation** — Obstacle avoidance, GPS waypoints, SLAM
7. **Communication** — WiFi dashboard, telemetry logging, ROS2 bridge
8. **Software Architecture** — FreeRTOS tasks, robot-level state machine, config system

### Prioritized Implementation Plan

| Phase | Version | Timeframe | Key Deliverables |
|-------|---------|-----------|-----------------|
| 1 | V2.9 | ~2 weeks | MPU6050 IMU, auto-leveling, rubber feet, battery % |
| 2 | V3.0 | ~3 weeks | Foot contact switches, reactive stepping, tetrapod/ripple gaits |
| 3 | V3.5 | ~4 weeks | Distance sensor, obstacle avoidance, WiFi dashboard |
| 4 | V4.0 | ~2 months | Smart servos, FreeRTOS, telemetry, GPS, ROS2 |

### Documentation Created (March 26-27)

| File | Description |
|------|-------------|
| `rough_terrain_gaits.md` | Analysis of 6 terrain gaits with implementation notes |
| `industrial_upgrade_roadmap.md` | Full roadmap: sensors, stabilization, navigation, architecture |

---

## Next Steps

### Immediate (V2.8 testing)
- [ ] Test true wave gait on physical bot — verify 1-leg-at-a-time sequence
- [ ] Test diagonal walk with joystick — verify true diagonal (not arc)
- [ ] Test crab joystick diagonal — verify balanced 45° movement
- [ ] Tune wave stride (2.5 cm) — increase if too slow, decrease if unstable

### V2.9 (Next version)
- [ ] Add MPU6050 IMU (I2C to ESP32)
- [ ] Implement body tilt reading (pitch + roll)
- [ ] Add auto-leveling (adjust leg Z based on tilt)
- [ ] Add high-step wave mode (doubled LIFT_H)
- [ ] Add tetrapod gait (3-phase, 2-legs-at-a-time)
- [ ] Add rubber foot tips

### Future
- [ ] Foot contact switches (6×)
- [ ] Ripple gait (overlapping wave)
- [ ] Front distance sensor (VL53L1X)
- [ ] WiFi web dashboard
- [ ] Smart serial servos upgrade

---

## March 27, 2026 — V3.0 Development

### Session Start
- **Started new version:** `spiderBotV3/spiderBotV3.ino`
- **Core Goal:** Transition primary control to hardware PS2 DualShock controller while keeping BT Serial as a fallback/debug interface.

### V3.0: PS2 Controller Integration (Initial)

- **Library:** Added `PS2X_lib.h` for DualShock 2 wireless receiver over SPI (Pins: DAT=19, CMD=23, SEL=5, CLK=18).
- **Analog Mapping:** 
  - Left stick → Y-axis (`jFwd`), X-axis (`jTurn`, differential)
  - Right stick → X-axis (`jStrafe`, for crab/diagonal)
- **Button Mapping (Updated):**
  - D-pad → Walk / Strafe (Discrete mode)
  - ○/□ → Rotate Left/Right (Held)
  - △/× → Height Up/Down (One-shot)
  - L1/R1 → Sit/Sleep postures (One-shot)
  - START → Stop / Stand Up (`standStill`)
  - SELECT → Cycle gaits (g0→g1→g2→g3)
  - L3 → Quick-toggle Crab mode
  - R3 → **Master ON/OFF Toggle** (Hardware Mode button replacement)

### V3.0 Bug Fixes: PS2 "Phantom Buttons" & Init Failures

1. **Bug:** "Controller connected" message appeared instantly, even when receiver led was still blinking (searching).
   - **Cause:** `V2.8` checking logic. `ps2x.config_gamepad()` only verifies local SPI wiring to the receiver module, not whether a wireless controller is actually paired to it.
   - **Fix:** Switched to a two-step initialization. Step 1 initializes SPI. Step 2 polls `ps2x.read_gamepad()` and actively waits until analog stick readings settle around 128 (center) rather than 0 or 255.

2. **Bug:** "Phantom button presses" — the controller fired random speed/height/state commands without any physical input.
   - **Cause:** `readPS2()` was running every iteration of the non-blocking `loop()`, resulting in ~500 SPI reads per second. The PS2 protocol supports a maximum of ~60Hz. Over-polling caused SPI data corruption.
   - **Fix:** Implemented a software rate-limit (`PS2_POLL_MS = 50`), throttling PS2 reads to a safe 20Hz.
   - **Fix 2:** Added `pinMode(PS2_DAT, INPUT_PULLUP)` to prevent internal floating data line noise between physical polling cycles.
   - **Fix 3:** Handled a "warmup period" (`PS2_WARMUP_FRAMES = 10`) to discard the first few frames of junk data immediately after connection.

3. **Bug:** D-pad button presses caused infinite looping movement that couldn't be stopped.
   - **Cause:** Initial V3 code used `ButtonPressed()` (one-shot trigger) for the D-pad without corresponding release logic.
   - **Fix:** Refactored physical movement buttons to use `Button()` (held). Added a unified `ps2Moving` state tracker that explicitly issues `standStill()` the moment all movement buttons/sticks return to center.

### V3.0 Bug Fixes: Gait Persistence & Logging

1. **Bug:** Selecting a gait (e.g., Diagonal or Wave) worked temporarily, but using `standStill()`, `sittingLegs()`, or `sleepingLegs()` instantly reset the active gait back to Tripod (g0).
   - **Cause:** `standStill()` and posture commands contained hard-coded `activeGait = 0` assignments. Crab walk masked this issue because it forcefully reasserted `activeGait = 1` on every call.
   - **Fix:** Removed all `activeGait = 0` resets from stopping/posture functions. The gait selection now properly persists globally across stops.

2. **Improvement (Logging):** 
   - Created `logMove(const char* direction)` helper to standardize BT logging.
   - Both BT and PS2 commands now log cleanly in the format: `>> [Gait Name]: [Direction]`.

### V3.0: Control Architecture Enhancements

1. **Master ON/OFF Toggle (`R3`):**
   - The PS2 hardware intrinsically handles the "Mode" button, keeping it hidden from software reads. 
   - Replaced desired "Mode" toggle function with the `R3` button (Right Stick Click).
   - Pressing `R3` flips a global `isBotOn` state. 
   - **OFF State:** Immediately triggers `sleepingLegs()` and disregards ALL further stick/button input (except `R3` to wake back up).
   - **ON State:** Immediately triggers `standStill()` (stands the bot up) and re-enables control logic.

2. **Hold vs. One-Shot Logic Adjustments:**
   - Modified Rotation (○/□) bindings to use `.Button()` so they poll continuously while held.
   - Modified Posture (L1/R1) bindings to use `.ButtonPressed()` so they send the command once, preventing serial log spam.

3. **`sleepingLegs()` Pose Polish:**
   - Adjusted coordinates from `restX, walkY + 12` to `restX - 10, walkY + 16` across all 6 legs to create a tighter, more compact sleep form factor.

---

## Next Steps

### V3.0 Testing
- [ ] Field-test PS2 analog stick sensitivity (adjust `PS2_DEADZONE` if drifting occurs).
- [ ] Verify PS2 and BT override systems don't conflict functionally under load.
- [ ] Test sustained wave gait with PS2 D-Pad versus Analog Sticks.

### V3.1 (Hardware Roadmap Start)
- [ ] Add MPU6050 IMU for Auto-Leveling (See Roadmaps).
- [ ] Add silicone/rubber grip foot tips.

---

## April–May 2026 — Hardware & Kinematics Upgrades

### 🕷️ SpiderBot V3.1 (The PS2 Hardware Update)
This version marks the transition from purely Bluetooth-based commands to the physical DualShock controller.
* **PS2 Integration**: `PS2X_lib` handles raw SPI communication. The analog sticks provide omnidirectional mixing (Left for Fwd/Turn, Right for Strafe).
* **BT Fallback**: The old Bluetooth commands (F, B, L, R) were kept intact as an override/debug channel.

#### 🐛 Bugs Solved & Technical Insights

**Height Kinematics Overhaul — Root Cause & Step-by-Step Proof**

**The Bug: Femur Formula Was 90° Off**
Your physical servo mapping:
`FEMUR servo:  500µs = UP ↑    1500µs = HORIZONTAL →    2500µs = DOWN ↓`
`TIBIA servo:  500µs = INSIDE ←  1500µs = STRAIGHT DOWN ↓  2500µs = OUTSIDE →`

Old formula (WRONG): `p2 = 1500 + (fDeg - 90) * 11.11`
- 0° (horizontal): **500µs** (Vertically UP ❌)
- 40° (default): **945µs** (~50° above horiz)
- 90° (vertical up): **1500µs** (Horizontal ❌)
*The old formula reversed the entire range! At max height (fDeg=0°, femur should be horizontal), the servo went to 500µs = straight UP. That's why the legs flew outward!*

New formula (CORRECT): `p2 = 1500 - fDeg * 11.11`
- 0° (horizontal): **1500µs** (Horizontal →) ✅
- 40° (default): **1056µs** (40° above horiz) ✅
- 90° (vertical up): **500µs** (Vertical UP ↑) ✅

**Step-by-Step: Height from Min to Max**
All values use `restX = 12.5`, `walkY = 0`, `perpTibia = true`:
| Height | bodyZ | fDeg | p2 (femur) | p3 (tibia) | Visual |
|---|---|---|---|---|---|
| Crouched | -7.0 | 62° | 811µs | 811µs | Legs folded up tight |
| Low | -8.0 | 50° | 944µs | 944µs | Slightly crouched |
| **Default** | **-9.0** | **40°** | **1056µs** | **1056µs** | **Normal stance** |
| Tall | -10.0 | 30° | 1167µs | 1167µs | Standing tall |
| Taller | -12.0 | 14° | 1344µs | 1344µs | Very tall |
| **Max** | **-14.0** | **~2°** | **~1478µs** | **~1478µs** | **Femur nearly flat, tibia straight down** |

*Notice how p2 and p3 are always equal when `perpTibia = true`! This is because the tibia counter-rotates by exactly the femur's tilt angle. At max height, both are near 1500µs — the rest position of both servos.*

**The perpTibia Override**
`if (perpTibia) tDeg = 90.0f + fDeg;`
This ensures the tibia always points straight down by cancelling the femur's tilt:
- Femur tilted 40° up → tibia rotates 40° to compensate → still vertical ↓
- Femur horizontal (0°) → tibia at 0° compensation → straight down (its natural rest) ↓

**What Changed (Summary)**
| Change | Old | New |
|---|---|---|
| Femur formula | `1500 + (fDeg-90) × 11.11` | `1500 - fDeg × 11.11` |
| perpTibia flag | _(not present)_ | Overrides tibia to stay vertical |
| restX during height | Changed via perpX (foot slides) | **Stays constant** (foot planted) |
| Height direction | Inverted (+delta = shorter) | Correct (+delta = taller) |

### 📶 SpiderBot V3.2 (The Wireless Flash Update)
This version focused on removing the need to physically tether the robot to your laptop with a USB cable.
* **ArduinoOTA Added**: You can now flash firmware wirelessly over your home WiFi network.
* **Offline Tolerance**: The WiFi connection routine is non-blocking with a 5-second timeout, so if you take the bot outside to a park, it won't get stuck in an infinite loop looking for your home router.
* **Master Power Switch**: Re-mapped R3 (right stick click) to act as a Master ON/OFF toggle that instantly forces the bot into sleep mode and ignores all sticks.

#### 🐛 Bugs Solved & Technical Insights
* **Phantom Button Presses**: The PS2 controller fired random speed/height/state commands without any physical input.
  - **Cause**: `readPS2()` was running every iteration of the non-blocking `loop()`, resulting in ~500 SPI reads per second. The PS2 protocol supports a maximum of ~60Hz. Over-polling caused SPI data corruption.
  - **Fix 1**: Implemented a software rate-limit (`PS2_POLL_MS = 50`), throttling PS2 reads to a safe 20Hz.
  - **Fix 2**: Handled a "warmup period" (`PS2_WARMUP_FRAMES = 10`) to discard the first few frames of junk data immediately after connection.
* **D-pad Infinite Looping**: D-pad button presses caused infinite looping movement that couldn't be stopped.
  - **Fix**: Refactored physical movement buttons to use `.Button()` (held) instead of one-shot triggers, and added a unified `ps2Moving` state tracker to issue `standStill()` the moment sticks return to center.
* **Gait Persistence Bug**: Using `standStill()` or posture commands previously reset the active gait back to Tripod (g0). Removed hardcoded `activeGait = 0` assignments so the gait properly persists across stops.

### ⚖️ SpiderBot V3.3 (The Sensory & Levelling Update)
This was a massive upgrade that gave the bot physical awareness of its surroundings.
* **MPU-6050 (IMU)**: Integrated a 50Hz complementary filter to calculate Roll and Pitch.
* **Active Self-Levelling**: The engine actively tweaks the `restZ` height of individual legs to keep the body perfectly flat when walking over sloped terrain. It also includes "fall detection" to auto-sleep if it tips over.
* **Tactile Foot Sensors**: Integrated 6 digital inputs for limit switches on the feet.

#### 🐛 Bugs Solved & Technical Insights
* **IMU Levelling Calibration**: Discovered the initial linear estimation for Z-axis compensation was inaccurate on steep inclines. Re-calibrated the IMU complementary filter and mathematically locked the Z-axis compensation gain to proper geometry (0.24 cm/deg). Boundary bumped up to 5.0cm.
* **Adaptive Planting & Slip Detection**: During the wave gait, the foot now stops moving down the moment the limit switch clicks, rather than blindly pushing into the ground. It also warns you over BT if a foot loses contact mid-stride.

### V3.4: Phase 2 3D Matrix Kinematics
* **Manual Overrides**: Added Bluetooth toggles (`I` for IMU, `T` for Tactile) for the Hexapod terrain adaptation systems.
* **Global Kinematics**: Abolished the old 2D linear `imuZCompensation` approximation. Configured actual physical offsets (`hipX=6.1`, `hipY=12.0`). Rewrote `moveFootXYZ` to intercept targets and multiply them against a full 3D Roll/Pitch Rotational Matrix.
* **Result**: Complete elimination of horizontal foot sliding. The chassis now perfectly rotates around locked feet dynamically.

### V3.5: Phase 3 Organic Bézier Trajectories
* **Parametric Streaming Engine**: Replaced the chunky `SEND_A` / `PLANT_A` discrete sequence. The gait engine now calculates a floating-point progression `t` from 0.0 to 1.0.
* **Micro-Slicing**: The ESP32 streams highly granular 40ms X/Y/Z coordinate slices to the LSC-32 instead of offloading massive linear delays.
* **Cubic Bézier Formula**: Developed `bezierCubic()` to mathematically shape the Z-axis lift profile. Generates an organic 4-point parametric curve that tangentially impacts the floor at exactly 0 velocity.

#### 🐛 Bugs Solved & Technical Insights
* **Mechanical Jitter ("Ghost Steps")**: The original 30ms streaming approach constantly interrupted the LSC-32 controller before it could finish executing a command, causing severe jitter. We transitioned to a time-sliced deterministic engine where `sTime = qTime - 5` to ensure servo commands overlap boundary transitions, completely removing the micro-pauses between keyframes.
* **Hard Landing Noise**: Reduced `LIFT_H` from 3.0f to 2.2f and utilized the cubic Bézier drop-off to drastically reduce floor-impact noise.

### 🌊 SpiderBot V3.6 (The Phase-Continuous Per-Leg Engine)
The final step toward fluid, organic movement. This removes the concept of fixed "states" or "keyframes" entirely.
* **Per-Leg Phase Accumulators**: Each leg has its own `legPhase[i]` float (0.0 to 1.0) that advances smoothly on every tick.
* **Proportional Speed**: The rate at which phase advances (`phaseStep`) is mathematically scaled by the joystick's magnitude. Pushing the stick further natively speeds up the cycle.
* **Push vs Swing Boundaries**: Legs under `PUSH_FRACTION` (e.g. 0.6) drag linearly backward to push the body forward. Legs above `PUSH_FRACTION` execute the 4-point Bézier swing arc forward.
* **Natural Interleaving**: Instead of forcing Group A vs Group B, legs are simply assigned phase offsets (e.g., `{0, 0.5, 0, 0.5, 0, 0.5}` for a tripod). This creates perfectly continuous, ripple-like motion mimicking real biology with no mechanical micro-pauses.

---

## Next Steps (V3.6+)

### 1. Refine the Smooth Gait Physics
- [ ] Experiment with tuning the 4-segment bezier parameters. The 60/40 stance ratio is great but may slip on fast turns.
- [ ] Incorporate the V3.3 MPU-6050 logic smoothly back into the new V3.6 engine to re-enable tilt-compensated leg `restZ` adjustments while mid-curve.

### 2. Physical & Virtual Alignment
- [ ] Test the V3.6 engine on the physical SpiderBot and adjust `SLICE_MS` thresholds if the motors struggle to keep up with the 40ms update frequency.

### 3. Smart Servo / Architecture Overhaul
- [ ] Shift from brute force I2C servo blocks to proper feedback-loop based control (e.g., dynamically altering `SLICE_MS` based on voltage/load measurements).
- [ ] Start abstracting the gait code into distinct classes to free up the `.ino` file and move towards a clean, object-oriented PlatformIO `src/` directory format.

---
*End of session — next session starts from `spiderBotV3/spiderBotV3.6`*

## 🗺️ Project Roadmaps (V3–V5)

### V3: Hardware Refinements & Terrain Awareness
V3 focuses on making the robot robust enough to traverse unknown terrain without falling over. Right now, it moves well on flat surfaces, but it's blind and assumes every step lands perfectly.

**High-Impact Additions for V3:**
1. **MPU6050 IMU (Auto-Leveling):** The most critical upgrade. By reading Pitch and Roll, the robot can dynamically adjust the `restZ` (height) of each leg to keep its body perfectly level when walking up or down ramps. 
2. **Foot Contact Switches (Microswitches):** Mount small limit switches on the tips of the legs. Right now, the leg lowers to `restZ` blindly. With switches, the leg lowers *until* it feels the ground, allowing it to walk over rocks or stairs without stumbling.
3. **RGB Status Indicators (WS2812B):** A simple strip or ring of NeoPixels. Green = Standing, Pulsing = Walking, Blue = PS2 Connected, Red = Low Battery / Sleeping.
4. **Front-Facing Ultrasonic or LiDAR (VL53L1X):** Before stepping forward, it checks the distance. If an object is less than 15cm away, the robot beeps and automatically stops to prevent a crash.
5. **Servo Pan-Tilt Head for MPU/Ultrasonic:** Dedicate 2 spare ports on the LSC-32 to a cheap pan-tilt mechanism to physically look left and right before turning.

---

### V5: The Camera / Vision Module
V5 is the leap from a remote-controlled toy to an autonomous surveillance/exploration drone. Since cameras require high bandwidth, you typically use an **ESP32-CAM** or a **Raspberry Pi Zero W** alongside your main ESP32 (communicating via UART or I2C).

**Autonomous & Vision Features for V5:**
1. **FPV Web Dashboard (First Person View):** Host a Wi-Fi web server on the ESP32-CAM. You can look at the video stream on your phone or laptop while driving the bot with the PS2 controller.
2. **Motion-Activated Sentry Mode:** You put the bot in `sleepingLegs()`. If the camera detects motion (frame differencing), it automatically runs `standStill()`, aims the camera at the movement, and executes `helloLegs()`!
3. **Color / Object Tracking (OpenCV):** If using a Pi or streaming to a PC, the bot locks onto a bright red ball. As the ball moves left, the bot automatically executes `rotateLeft()`. As it moves forward, the bot executes `walkForward()` to chase it. 
4. **Edge / Drop-off Detection:** The camera points slightly downward. If it sees a sudden drop in the floor (e.g., the edge of a table or staircase), it overrides the controller and executes a rapid `walkBackward()`.
5. **ArUco / QR Code Navigation:** Print QR codes and tape them to walls (e.g., "Kitchen", "Living Room"). The robot walks around randomly until it sees a QR code. It then knows exactly where it is in your house.
6. **2-Axis Camera Gimbal (Pan/Tilt):** Mount the camera on a servo gimbal driven by your LSC-32. You can use the Right Stick on your PS2 controller to look around the room without having to turn the robot's body.

