# Tripod Walk — Deep Dive

## What Is a Tripod Gait?

A tripod gait is the **fastest and most common** hexapod locomotion pattern. **3 legs lift simultaneously** while the other **3 legs stay on the ground**, forming a stable triangle.

The name comes from the fact that the robot always rests on a **tripod** — three legs forming a triangle — ensuring static stability at all times.

---

## The Two Tripod Groups

The 6 legs are split into two alternating triangles:

```
          FRONT
     0(FL) ─── 3(FR)
       │         │
     1(ML) ─── 4(MR)
       │         │
     2(RL) ─── 5(RR)
          REAR

  Group A = { 0, 2, 4 }  →  FL, RL, MR  (triangle △)
  Group B = { 1, 3, 5 }  →  ML, FR, RR  (triangle ▽)
```

```
  Group A on ground:          Group B on ground:
     ●───○                       ○───●
     │   │                       │   │
     ○───●                       ●───○
     │   │                       │   │
     ●───○                       ○───●

  ● = on ground    ○ = in air
```

> [!IMPORTANT]
> The groups are chosen so that each forms a **stable triangle** — one front, one middle, one rear, alternating sides. This prevents the robot from tipping.

---

## The Stride Functions

```cpp
float tripodSwingY(int i){
  return legs[i].walkY - (3.0f * legs[i].coxaDir * legs[i].strideScale);
}
float tripodDragY(int i){
  return legs[i].walkY + (3.0f * legs[i].coxaDir * legs[i].strideScale);
}
```

### What these compute:

| | Left leg (coxaDir = +1) | Right leg (coxaDir = -1) |
|---|---|---|
| **swingY** | walkY − 3.0 (moves backward in Y) | walkY + 3.0 (moves forward in Y) |
| **dragY** | walkY + 3.0 (moves forward in Y) | walkY − 3.0 (moves backward in Y) |

The `coxaDir` flip ensures left and right legs push the body in the **same direction** — both contribute to forward motion even though they're mirrored.

### Worked example (Leg 0, Front-Left):
```
walkY = 3.0,  coxaDir = +1,  stride = 3.0

swingY = 3.0 - (3.0 × 1 × 1.0) = 0.0   ← foot reaches forward
dragY  = 3.0 + (3.0 × 1 × 1.0) = 6.0   ← foot pushes backward

Total Y travel = 6.0 cm per cycle
```

---

## The Scale Factor (Differential Mixing)

The raw swing/drag targets are **never used directly**. They're blended through `finalScale`:

```cpp
float finalScale = constrain(jFwd + (jTurn * leg.coxaDir), -1.0f, 1.0f);
float stepY = leg.walkY + (swingY - leg.walkY) * finalScale;
```

This single formula enables **all movement commands**:

| Command | jFwd | jTurn | Left scale (+1) | Right scale (−1) | Result |
|---------|------|-------|------------------|-------------------|--------|
| Forward | +1.0 | 0.0 | 1.0 | 1.0 | Straight forward |
| Backward | −1.0 | 0.0 | −1.0 | −1.0 | Straight backward |
| Turn Left | +0.66 | −0.33 | 0.33 | 0.99 | Right legs stride more → arc left |
| Turn Right | +0.66 | +0.33 | 0.99 | 0.33 | Left legs stride more → arc right |
| Rotate Left | 0.0 | −1.0 | −1.0 | 1.0 | Left legs backward, right forward → spin |
| Rotate Right | 0.0 | +1.0 | 1.0 | −1.0 | Left legs forward, right backward → spin |

> [!TIP]
> When `finalScale` is negative, the stride **reverses** — the swing position becomes the drag position and vice versa. This is how backward walking works without needing separate gait functions.

---

## The 5-Phase State Machine

Each tripod cycle runs through 5 phases:

```
┌────────┐    ┌─────────┐    ┌────────┐    ┌─────────┐    ┌──────────┐
│ SEND_A │───▶│ PLANT_A │───▶│ SEND_B │───▶│ PLANT_B │───▶│ RECENTER │──┐
└────────┘    └─────────┘    └────────┘    └─────────┘    └──────────┘  │
     ▲                                                                   │
     └───────────────────── (if looping) ────────────────────────────────┘
```

### Phase-by-Phase Walkthrough

#### Phase 1: SEND_A
```
Action: Group A swings (lifts + moves forward)
        Group B drags  (stays on ground, slides backward)

  Group A legs:  moveFootXYZ(... , swingY, Z - LIFT_H, moveTime)
                                         ↑ lifted 3cm
  Group B legs:  moveFootXYZ(... , dragY,  Z,          moveTime)
                                          ↑ on ground

        ○ ─── .            FL lifts, FR drags
        │     │
        . ─── ○            ML drags, MR lifts
        │     │
        ○ ─── .            RL lifts, RR drags

  Duration: moveTime (e.g. 500ms)
  Wait:     moveTime + 20ms buffer
```

#### Phase 2: PLANT_A
```
Action: Group A legs lowered back to ground (at their swing position)

  Group A legs:  moveFootXYZ(... , swingY, Z, moveTime/3)
                                          ↑ back on ground

  Duration: moveTime / 3  (fast drop, e.g. 167ms)
```

#### Phase 3: SEND_B
```
Action: Group B swings (lifts + moves forward)
        Group A drags  (now on ground, slides backward)

        . ─── ○            FL drags, FR lifts
        │     │
        ○ ─── .            ML lifts, MR drags
        │     │
        . ─── ○            RL drags, RR lifts

  Duration: moveTime + 20ms buffer
```

#### Phase 4: PLANT_B
```
Action: Group B legs lowered back to ground

  Duration: moveTime / 3
```

#### Phase 5: RECENTER
```
Action: All 6 legs return to their walkY neutral position

  for all legs:  moveFootXYZ(... , walkY, Z, moveTime/2)

  Duration: moveTime / 2
  Then: if looping → restart at SEND_A
        else       → go to IDLE
```

---

## Full Timing Breakdown

For `moveTime = 500ms`:

| Phase | Duration | Cumulative | What happens |
|-------|----------|------------|--------------|
| SEND_A | 520ms | 520ms | A swings, B drags |
| PLANT_A | 177ms | 697ms | A plants down |
| SEND_B | 520ms | 1217ms | B swings, A drags |
| PLANT_B | 177ms | 1394ms | B plants down |
| RECENTER | 250ms | 1644ms | All legs center |
| **Gap** | 250ms | **1894ms** | Wait before next cycle |

**Total cycle ≈ 1.9 seconds** at default speed.

---

## Why Tripod Is The Default

| Property | Value |
|----------|-------|
| Legs on ground | 3 (always a stable triangle) |
| Speed | Fastest of all gaits |
| Phases per cycle | 2 swing phases |
| Supports turning | Yes — differential via jTurn |
| Supports joystick | Yes — proportional jFwd + jTurn |
| Stability | Moderate (good enough for most surfaces) |

> [!NOTE]
> The tripod gait uses `indepXY = false`, meaning the X axis stays at `restX` (no lateral movement). The [defaultX()](file:///f:/Arduino/Git/SpiderBot/spiderBotV2/spiderBotV2.7/spiderBotV2.7.ino#191-193) function always returns `legs[i].restX`. Both axes share the same `finalScale`.
