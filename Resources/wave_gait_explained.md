# Wave Gait — Deep Dive

## What Is a Wave Gait?

A wave gait is the **slowest but most stable** hexapod locomotion pattern. Only **one leg lifts at a time** while the other **5 legs stay on the ground**, pushing the body forward.

The "wave" name comes from the **sequential ripple pattern** — legs lift one after another like a wave propagating across the body.

---

## Visual Comparison: Tripod vs Wave

### Tripod Gait (current g0)
```
Phase 1:  ▲ . ▲     ← 3 legs swing (lift)
          . ▲ .     ← 3 legs drag  (ground)

Phase 2:  . ▲ .     ← swap groups
          ▲ . ▲

Cycle = 2 phases → fast, but only 3 legs on ground
```

### Wave Gait (true)
```
                LEFT SIDE          RIGHT SIDE
Leg index:    0(FL)  1(ML)  2(RL) │ 3(FR)  4(MR)  5(RR)
              ─────────────────────┼────────────────────
Phase 0:        .      .      ▲   │   .      .      .     ← Rear-Left lifts
Phase 1:        .      ▲      .   │   .      .      .     ← Mid-Left lifts
Phase 2:        ▲      .      .   │   .      .      .     ← Front-Left lifts
Phase 3:        .      .      .   │   .      .      ▲     ← Rear-Right lifts
Phase 4:        .      .      .   │   .      ▲      .     ← Mid-Right lifts
Phase 5:        .      .      .   │   ▲      .      .     ← Front-Right lifts

▲ = leg in air (swing)    . = leg on ground (drag)
Cycle = 6 phases → slow, but 5 legs always on ground (maximum stability)
```

The wave propagates **rear → front** on each side, first left then right.

---

## The Stride Math

In a wave gait, the body moves **continuously** — every phase pushes the body forward by 1/6 of a full stride. Here's the key insight:

### Each phase has two actions:
1. **Swing leg** — lifts up, moves to the **front** of its stride (target position)
2. **Drag legs (×5)** — stay on ground, slide **backward** by 1/6 of the stride

### Why 1/6? 
The full stride is divided across 6 phases. Each dragging leg pushes the body forward a little bit. After all 6 phases, the body has moved one full stride.

```
Stride = 3.0 cm (example)
Drag per phase = stride / 6 = 0.5 cm

               Phase 0    Phase 1    Phase 2    Phase 3    Phase 4    Phase 5
Leg 2 (RL):    SWING→+3   drag -0.5  drag -1.0  drag -1.5  drag -2.0  drag -2.5
Leg 1 (ML):    drag -0.5  SWING→+3   drag -0.5  drag -1.0  drag -1.5  drag -2.0
Leg 0 (FL):    drag -0.5  drag -1.0  SWING→+3   drag -0.5  drag -1.0  drag -1.5
Leg 5 (RR):    drag -0.5  drag -1.0  drag -1.5  SWING→+3   drag -0.5  drag -1.0
Leg 4 (MR):    drag -0.5  drag -1.0  drag -1.5  drag -2.0  SWING→+3   drag -0.5
Leg 3 (FR):    drag -0.5  drag -1.0  drag -1.5  drag -2.0  drag -2.5  SWING→+3
                                                                           ↑
                                                                    full stride done
```

> [!IMPORTANT]
> The swing position is always the same (front of stride). The drag position increments backward each phase. After a leg swings, it starts at the front and gradually slides to the rear over the next 5 phases.

---

## Why Your Current Wave Gait Isn't Real

Your current V2.7 engine is a **2-phase machine**:

```
SEND_A → PLANT_A → SEND_B → PLANT_B → RECENTER
```

This moves legs in **two groups of 3** — it's a tripod engine. The "wave gait" (g3) just uses different stride functions ([waveSwingY](file:///f:/Arduino/Git/SpiderBot/spiderBotV2/spiderBotV2.7/spiderBotV2.7.ino#216-221)/[waveDragY](file:///f:/Arduino/Git/SpiderBot/spiderBotV2/spiderBotV2.7/spiderBotV2.7.ino#221-225)) but still moves 3 legs simultaneously. **That's a tripod gait with wave-inspired stride values**, not a true wave.

A true wave needs a **6-phase machine** — one phase per leg.

---

## How to Implement a True Wave Gait

### Step 1: Wave Sequence

Define the order legs lift in:

```cpp
// Wave order: rear→front left side, then rear→front right side
const int WAVE_SEQ[6] = { 2, 1, 0, 5, 4, 3 };
//                        RL ML FL RR MR FR
```

### Step 2: Phase State Machine

Extend the gait engine with a wave-specific path:

```cpp
enum GaitPhase {
  IDLE,
  // Tripod phases (existing)
  SEND_A, PLANT_A, SEND_B, PLANT_B, RECENTER,
  // Wave phases (new)
  WAVE_STEP, WAVE_PLANT
};

uint8_t wavePhaseIdx = 0;  // 0–5: which leg in WAVE_SEQ is swinging
```

### Step 3: Wave Step Function

Each wave step does two things simultaneously:
1. **Swing 1 leg** — lift it and move to the front of its stride
2. **Drag 5 legs** — slide them backward by 1/6 stride

```cpp
void waveStep(uint8_t phaseIdx) {
  GaitDef& g    = GAIT_TABLE[activeGait];
  int swingIdx  = WAVE_SEQ[phaseIdx];
  Leg& swingLeg = crabMode ? crabLegs[swingIdx] : legs[swingIdx];

  // Scale (for joystick proportional control)
  float scale = constrain(jFwd + (jTurn * swingLeg.coxaDir), -1.0f, 1.0f);

  // ─── Swing leg: lift + move to front of stride ───
  float swY = swingLeg.walkY + (g.swingY(swingIdx) - swingLeg.walkY) * scale;
  float swX = swingLeg.restX + (g.swingX(swingIdx) - swingLeg.restX) * scale;
  moveFootXYZ(swingLeg.id, swX, swY, swingLeg.restZ - LIFT_H, moveTime);

  // ─── Drag all other 5 legs: slide backward by 1/6 stride ───
  for (int i = 0; i < 6; i++) {
    if (i == swingIdx) continue;

    Leg& leg = crabMode ? crabLegs[i] : legs[i];
    float legScale = constrain(jFwd + (jTurn * leg.coxaDir), -1.0f, 1.0f);

    // How many phases since this leg last swung?
    // This determines how far back it has dragged.
    int phasesSinceSwing = 0;
    for (int p = 1; p <= 5; p++) {
      int checkPhase = (phaseIdx - p + 6) % 6;
      if (WAVE_SEQ[checkPhase] == i) {
        phasesSinceSwing = p;
        break;
      }
    }

    // Interpolate from swing position toward drag position
    float t = phasesSinceSwing / 5.0f;  // 0.0 = just swung, 1.0 = about to swing
    float targetY = swingLeg.walkY + (g.swingY(i) - leg.walkY) * (1.0f - t) * legScale
                  + (g.dragY(i) - leg.walkY) * t * legScale;
    // Simplified: linear interpolation between swing and drag targets
    float frontY = leg.walkY + (g.swingY(i) - leg.walkY) * legScale;
    float rearY  = leg.walkY + (g.dragY(i)  - leg.walkY) * legScale;
    float dragY  = frontY + (rearY - frontY) * t;

    moveFootXYZ(leg.id, leg.restX, dragY, leg.restZ, moveTime);
  }
}
```

### Step 4: Wave Plant Function

After the swing completes, plant the leg down:

```cpp
void wavePlant(uint8_t phaseIdx) {
  int   swingIdx = WAVE_SEQ[phaseIdx];
  Leg&  leg      = crabMode ? crabLegs[swingIdx] : legs[swingIdx];
  GaitDef& g     = GAIT_TABLE[activeGait];

  float scale = constrain(jFwd + (jTurn * leg.coxaDir), -1.0f, 1.0f);
  float plantY = leg.walkY + (g.swingY(swingIdx) - leg.walkY) * scale;
  moveFootXYZ(leg.id, leg.restX, plantY, leg.restZ, moveTime / 4);
}
```

### Step 5: Engine Integration

Add the wave phases to [gaitTick()](file:///f:/Arduino/Git/SpiderBot/spiderBotV2/spiderBotV2.7/spiderBotV2.7.ino#362-418):

```cpp
case WAVE_STEP:
  waveStep(wavePhaseIdx);
  gaitPhase  = WAVE_PLANT;
  phaseStart = millis();
  break;

case WAVE_PLANT:
  if (elapsed >= (unsigned long)(moveTime + 20)) {
    wavePlant(wavePhaseIdx);
    wavePhaseIdx = (wavePhaseIdx + 1) % 6;

    if (wavePhaseIdx == 0 && !looping) {
      gaitPhase  = RECENTER;
      phaseStart = millis();
    } else {
      gaitPhase  = WAVE_STEP;
      phaseStart = millis() + moveTime / 4;  // brief pause after plant
    }
  }
  break;
```

### Step 6: Start Function

```cpp
void startGait(bool continuous) {
  looping = continuous;
  if (activeGait == 3) {  // Wave gait
    wavePhaseIdx = 0;
    gaitPhase    = WAVE_STEP;
  } else {
    gaitPhase    = SEND_A;
  }
  phaseStart = millis();
}
```

---

## Comparison Summary

| Property | Tripod (g0) | Wave (g3) |
|----------|-------------|-----------|
| Legs lifting at once | 3 | **1** |
| Legs on ground | 3 | **5** |
| Phases per cycle | 2 | **6** |
| Speed | Fast | **Slow (3× slower)** |
| Stability | Moderate | **Maximum** |
| Engine complexity | Simple 2-phase | **6-phase state machine** |
| Turning support | Differential jFwd+jTurn | Same (per-leg scale) |

---

## Key Design Decision

> [!TIP]
> The wave gait is 6× more phases per cycle, so it will be ~3× slower than tripod at the same `moveTime`. You may want to **automatically reduce moveTime** when wave gait is selected to keep it from being painfully slow:
> ```cpp
> // In the g3 command handler:
> moveTime = max(SPEED_MIN, moveTime / 2);  // speed up for wave
> ```
