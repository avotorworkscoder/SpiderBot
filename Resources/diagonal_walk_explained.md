# Diagonal Walk — Deep Dive

## What Is a Diagonal Walk?

The diagonal walk is a **hybrid gait** that combines the best of tripod and crab:
- Uses the **normal spider stance** (`legs[]`) — not crab legs
- Uses the **tripod 2-phase engine** — same speed as g0
- But enables **true diagonal movement** via independent X/Y scaling — like crab mode

When you push the joystick diagonally, the bot walks at an actual angle instead of curving in an arc.

---

## Diagonal Walk vs Arc Turn (Tripod)

This is the key distinction:

### Tripod (g0) — Arc Turn
```
Joystick at 45°:  jFwd = 0.5, jTurn = -0.5

Left legs:   finalScale = 0.5 + (-0.5 × +1) = 0.0  → no movement
Right legs:  finalScale = 0.5 + (-0.5 × -1) = 1.0  → full stride

Result: Right legs stride, left legs idle → body CURVES LEFT
        Path is an ARC, not a straight diagonal line.
```

```
  Arc turn path:        ╭──╮
                       ╱    ╲
                      ╱      │
                    Start    │
```

### Diagonal Walk (g2) — True Diagonal
```
Joystick at 45°:  jFwd = 0.5, jStrafe = 0.5

ALL legs:  scaleY = 0.5   → half stride forward
           scaleX = 0.5   → half stride sideways

Result: Every leg moves in BOTH X and Y → body moves in STRAIGHT diagonal line.
```

```
  Diagonal path:       ╱
                      ╱
                    ╱
                  Start
```

> [!IMPORTANT]
> Arc turn changes the robot's **heading** (it rotates while moving). Diagonal walk keeps the heading **fixed** — the body translates in a straight line at an angle.

---

## How It Works Mechanically

### The Stride Functions

Diagonal Walk reuses existing functions:

```cpp
// Y-axis: same as tripod (forward/backward)
swingY = tripodSwingY  →  legs[i].walkY - (3.0f × coxaDir × strideScale)
dragY  = tripodDragY   →  legs[i].walkY + (3.0f × coxaDir × strideScale)

// X-axis: new walk strides (lateral movement)
swingX = walkSwingX    →  legs[i].restX - (3.0f × coxaDir × strideScale)
dragX  = walkDragX     →  legs[i].restX + (3.0f × coxaDir × strideScale)
```

### Gait Table Entry

```cpp
{ "Diagonal Walk", walkSwingX, walkDragX, tripodSwingY, tripodDragY, true }
//                 ─────────────────────  ─────────────────────────  ────
//                    X stride (new)         Y stride (reused)      indepXY
```

Key properties:
- **`indepXY = true`** — X and Y axes are scaled independently
- Uses **`legs[]`** (not `crabLegs[]`) — normal spider stance
- Uses **tripod engine** — same SEND_A → PLANT_A → SEND_B → PLANT_B → RECENTER

---

## Independent Scaling in the Engine

Because `indepXY = true`, the engine splits the scale computation:

```cpp
// In sendPhase():
float scaleY = g.indepXY ? jFwd                                          // ← jFwd only
                         : constrain(jFwd + (jTurn * leg.coxaDir), ...);  // ← differential
float scaleX = g.indepXY ? jStrafe                                       // ← jStrafe only
                         : scaleY;                                        // ← same as Y

float stepX = leg.restX + (swingX - leg.restX) * scaleX;
float stepY = leg.walkY + (swingY - leg.walkY) * scaleY;
```

This means:
- `jFwd` controls **forward/backward** stride amplitude (Y axis)
- `jStrafe` controls **lateral** stride amplitude (X axis)
- They are **completely independent** — no differential mixing

---

## Joystick Mapping

When g2 is active and a joystick command arrives:

```cpp
if (GAIT_TABLE[activeGait].indepXY) {
    jFwd    = rawFwd;
    jStrafe = -rawTurn;    // sign flip: right stick = right movement
    jTurn   = 0.0f;
}
```

### Examples:

| Command | jFwd | jStrafe | Movement |
|---------|------|---------|----------|
| `j100,0` | 1.0 | 0.0 | Pure forward (identical to tripod) |
| `j-100,0` | -1.0 | 0.0 | Pure backward (identical to tripod) |
| `j0,-100` | 0.0 | 1.0 | Pure left strafe |
| `j0,100` | 0.0 | -1.0 | Pure right strafe |
| `j70,-70` | 0.7 | 0.7 | 45° forward-left diagonal |
| `j50,-87` | 0.5 | 0.87 | ~60° mostly-sideways-left |

---

## Worked Example: `j70,-70` (Forward-Left Diagonal)

```
jFwd = 0.7,  jStrafe = 0.7

Leg 0 (Front-Left):
  walkY = 3.0,  restX = 10.0,  coxaDir = +1

  tripodSwingY(0) = 3.0 - 3.0 = 0.0
  walkSwingX(0)   = 10.0 - 3.0 = 7.0

  Swing phase:
    stepY = 3.0 + (0.0 - 3.0) × 0.7 = 3.0 - 2.1 = 0.9
    stepX = 10.0 + (7.0 - 10.0) × 0.7 = 10.0 - 2.1 = 7.9

  → Foot moves from (10.0, 3.0) to (7.9, 0.9)
    That's -2.1 in X and -2.1 in Y → true 45° diagonal ✓
```

---

## Comparison: All Three Walking Gaits

| Property | Tripod (g0) | Diagonal (g2) | Crab (g1) |
|----------|-------------|---------------|-----------|
| Leg array | `legs[]` | `legs[]` | `crabLegs[]` |
| Stance | Spider (outward) | Spider (outward) | Crab (sideways) |
| Engine | 2-phase tripod | 2-phase tripod | 2-phase tripod |
| `indepXY` | `false` | **`true`** | **`true`** |
| Y control | jFwd + jTurn×coxaDir | jFwd alone | jFwd alone |
| X control | Always restX | **jStrafe** | **jStrafe** |
| Joystick diagonal | Arc curve | **Straight line** | **Straight line** |
| F/B buttons | Walk forward/backward | Walk forward/backward | Crab forward/backward |
| L/R buttons | Arc turn | Arc turn | Strafe |
| Speed | ★★★ | ★★★ | ★★★ |
| Stability | ★★ | ★★ | ★★ |

---

## When to Use Each Gait

```
┌─────────────────────────────────────────────────────────┐
│  Need turning?                                          │
│    YES → g0 (Tripod) — jTurn gives smooth arc turns     │
│    NO  → Do you need lateral (sideways) movement?       │
│           YES → Do you want crab stance?                │
│                   YES → g1 (Crab Walk)                  │
│                   NO  → g2 (Diagonal Walk)              │
│           NO  → g0 (Tripod) — simplest, fastest         │
└─────────────────────────────────────────────────────────┘
```

> [!TIP]
> **Diagonal Walk is ideal for joystick control** when you want the robot to move in any direction while keeping its spider stance and heading fixed. Think of it as "holonomic mode" — the joystick directly maps to movement direction.

> [!NOTE]
> Diagonal Walk currently doesn't support **arc turning** via buttons — L/R still triggers tripod arc turns (because those functions force `activeGait = 0`). Turning is only possible by switching to g0 or using rotate (W/X).
