# Rough Terrain Gaits — Suggestions

## Current Gait Lineup (V2.8)

| Gait | Legs on ground | Speed | Stability | Terrain |
|------|---------------|-------|-----------|---------|
| g0 Tripod | 3 | ★★★ | ★★ | Flat surfaces |
| g1 Crab | 3 | ★★★ | ★★ | Flat, omnidirectional |
| g2 Diagonal | 3 | ★★★ | ★★ | Flat, omnidirectional |
| g3 Wave | 5 | ★ | ★★★★★ | Moderate rough |

The gap: nothing between tripod (fast, moderate stability) and wave (very slow, max stability). Rough terrain needs **more options** in the middle.

---

## Suggested Gaits for Rough Terrain

### 1. Tetrapod Gait ⭐⭐⭐ (Recommended First)

**Concept:** Only **2 legs lift** at a time, **4 legs stay grounded**. A sweet spot between tripod (3 up) and wave (1 up).

```
         Phase 1      Phase 2      Phase 3
Leg 0:     ▲            .            .
Leg 1:     .            .            ▲
Leg 2:     .            ▲            .
Leg 3:     .            .            ▲
Leg 4:     ▲            .            .
Leg 5:     .            ▲            .

▲ = swinging    . = on ground
4 legs always grounded → far more stable than tripod
```

**Pairs (always opposite corners for balance):**
- Phase 1: {0, 4} — Front-Left + Mid-Right
- Phase 2: {2, 5} — Rear-Left + Rear-Right
- Phase 3: {1, 3} — Mid-Left + Front-Right

| Property | Value |
|----------|-------|
| Legs on ground | **4** |
| Speed | ★★☆ (50% slower than tripod) |
| Stability | ★★★☆ |
| Engine complexity | **Low** — 3-phase version of your tripod engine |
| Good for | Uneven ground, gravel, light inclines |

**Implementation:** Very similar to your tripod engine but with 3 groups of 2 instead of 2 groups of 3:
```cpp
const int TET_A[2] = { 0, 4 };
const int TET_B[2] = { 2, 5 };
const int TET_C[2] = { 1, 3 };
// Engine cycles: SEND_A → PLANT_A → SEND_B → PLANT_B → SEND_C → PLANT_C → RECENTER
```

---

### 2. Ripple Gait ⭐⭐⭐ (Recommended)

**Concept:** A **faster wave** — legs lift in the same rear→front sequence, but neighboring legs **overlap** their swing phases. Two legs are always in the air at any time, but they're **never adjacent**.

```
Timeline →    t0   t1   t2   t3   t4   t5   t6
Leg 2 (RL):   ▲▲▲  ...  ...  ...  ...  ▲▲▲  ...
Leg 1 (ML):   .▲▲  ▲..  ...  ...  ...  .▲▲  ▲..
Leg 0 (FL):   ...  ▲▲▲  ...  ...  ...  ...  ▲▲▲
Leg 5 (RR):   ...  ...  ▲▲▲  ...  ...  ...  ...
Leg 4 (MR):   ...  ...  .▲▲  ▲..  ...  ...  ...
Leg 3 (FR):   ...  ...  ...  ▲▲▲  ...  ...  ...

→ 4 legs on ground at all times
→ 2× faster than wave, almost as stable
```

| Property | Value |
|----------|-------|
| Legs on ground | **4** |
| Speed | ★★☆ (2× faster than wave) |
| Stability | ★★★★ |
| Engine complexity | **Medium** — needs phase-offset timing |
| Good for | Rocky terrain, uneven surfaces |

**Implementation:** Use wave sequence but with overlapping timing. Each leg starts its swing before the previous leg finishes planting:
```cpp
// Phase offsets (in units of moveTime)
const float RIPPLE_OFFSET[6] = { 0.0, 0.33, 0.66, 1.0, 1.33, 1.66 };
// Each leg's swing starts at its offset time within the cycle
```

---

### 3. High-Step Wave ⭐⭐ (Obstacle Crossing)

**Concept:** Same as your wave gait but with **exaggerated lift height** and **shorter stride**. Legs lift high to step over small obstacles.

```
Normal Wave:          High-Step Wave:
     ╱╲                    │
    ╱  ╲                   │
───╱    ╲───         ─────╱ ╲─────
  stride: 5cm            stride: 2cm
  lift: 3cm               lift: 6cm
```

| Property | Value |
|----------|-------|
| Legs on ground | **5** |
| Speed | ★ (slower due to higher lift) |
| Stability | ★★★★★ |
| Engine complexity | **Very Low** — just change LIFT_H and stride |
| Good for | Stepping over wires, small rocks, doorway tracks |

**Implementation:** Trivially simple — change parameters when this gait is selected:
```cpp
// In gait selection:
if (activeGait == HIGH_STEP_WAVE) {
    LIFT_H_active = 6.0f;  // double normal lift
    // Use wave stride functions with reduced amplitude (1.5cm)
}
```

> [!TIP]
> This is the **easiest** terrain gait to implement — it's literally your existing wave gait with different LIFT_H and stride values. You could add it as a sub-mode of g3.

---

### 4. Adaptive Tripod ⭐⭐ (Smart Terrain)

**Concept:** Normal tripod gait but with **variable lift height per leg**. If a leg encounters resistance (or you manually set terrain mode), it lifts higher on subsequent swings.

```
Flat terrain:         Obstacle detected:
   ╱╲                      │╲
  ╱  ╲                     │ ╲
─╱    ╲──             ────╱   ╲──
 3cm lift               6cm lift (this leg only)
```

| Property | Value |
|----------|-------|
| Legs on ground | **3** |
| Speed | ★★★ (same as tripod) |
| Stability | ★★ |
| Engine complexity | **Medium** — per-leg lift heights |
| Good for | Mixed terrain (some flat, some rough) |

**Implementation:** Add a `liftH` field to the Leg struct:
```cpp
struct Leg {
    ...
    float liftH;  // per-leg lift height, default = LIFT_H
};

// In sendPhase, replace LIFT_H with leg.liftH:
moveFootXYZ(leg.id, stepX, stepY, leg.restZ - leg.liftH, moveTime);
```

---

### 5. Body-Shift Walk ⭐⭐ (Slope Navigation)

**Concept:** Before lifting a group of legs, **shift the body's center of gravity** toward the grounded legs. This prevents tipping on slopes.

```
Normal tripod:                Body-shift tripod:
    ●     ○                      ●     ○
    │     │   ← body centered    │  ←──│   ← body shifted toward ●
    ○     ●                      ○     ●
    │     │                      │     │
    ●     ○                      ●     ○
                                 Then lift ○ legs safely
```

| Property | Value |
|----------|-------|
| Legs on ground | **3** |
| Speed | ★★ (slower due to body shift phase) |
| Stability | ★★★★ |
| Engine complexity | **Medium-High** — add body shift phase |
| Good for | Inclines, slopes, tilted surfaces |

**Implementation:** Add a body-shift phase before each SEND phase:
```cpp
// New phases: SHIFT_A → SEND_A → PLANT_A → SHIFT_B → SEND_B → ...
void shiftBody(bool towardGroupA) {
    // Move all legs slightly in X/Y to shift CoG toward the support group
    float shiftX = towardGroupA ? -1.0f : 1.0f;
    for (int i = 0; i < 6; i++) {
        Leg& leg = legs[i];
        moveFootXYZ(leg.id, leg.restX + shiftX, leg.walkY, leg.restZ, moveTime/2);
    }
}
```

---

### 6. Free Gait ⭐ (Advanced — Full Manual Control)

**Concept:** No fixed sequence — each leg is controlled independently. A higher-level planner decides which leg to move and where based on terrain mapping.

| Property | Value |
|----------|-------|
| Legs on ground | **Variable (3–5)** |
| Speed | Variable |
| Stability | ★★★★★ (if planner is good) |
| Engine complexity | **Very High** — needs terrain sensing |
| Good for | Highly irregular terrain, research |

> [!WARNING]
> This requires significant additional hardware (IMU, foot pressure sensors, or distance sensors) and a much more complex software architecture. Not practical for BT remote control, but worth knowing about for future development.

---

## Recommendation Priority

For your SpiderBot, I'd recommend implementing in this order:

```
                        Stability
                           ▲
                           │
          Free Gait ─ ─ ─ ─│─ ─ ─ ─ ─ ─ ─ ─ ★
                           │              ╱
           Body-Shift ─ ─ ─│─ ─ ─ ─ ─ ★
                           │          ╱
        High-Step Wave ─ ─ │─ ─ ─ ★    Wave (existing)
                           │      │ ╱
              Ripple ─ ─ ─ │─ ─ ★ ★
                           │    │
            Tetrapod ─ ─ ─ │─ ★
                           │  │
             Tripod ─ ─ ─ ─│★ (existing)
                           │
                           └──────────────────→ Speed
```

### Phase 1 (Quick wins):
1. **High-Step Wave** — 5 minutes of work, just change lift/stride
2. **Tetrapod** — straightforward 3-phase version of your tripod engine

### Phase 2 (Moderate effort):
3. **Ripple Gait** — overlapping wave, needs phase-offset timing
4. **Adaptive Tripod** — per-leg lift heights

### Phase 3 (Advanced):
5. **Body-Shift Walk** — adds CoG management
6. **Free Gait** — needs sensors and path planner
