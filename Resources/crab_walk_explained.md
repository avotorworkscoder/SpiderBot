# Crab Walk — Deep Dive

## What Is a Crab Walk?

A crab walk makes the hexapod move **sideways**, like a real crab. Instead of legs pointing outward like a spider, the legs are **rotated to face sideways** so all feet point toward the same wall.

This enables **omnidirectional movement** — forward, backward, strafe left, strafe right, and any diagonal — all from a single gait.

---

## Crab Stance vs Normal Stance

### Normal stance (`legs[]`) — spider-like
```
     ↗ FL         FR ↖           Legs point outward
       │           │             Good for forward/backward
     → ML         MR ←           Turning via differential stride
       │           │
     ↘ RL         RR ↙
```

### Crab stance (`crabLegs[]`) — sideways-facing
```
     ↑ FL         FR ↑           ALL legs point the same direction
     │             │             This is how a crab stands
     ↑ ML         MR ↑           Great for sideways strafing
     │             │
     ↑ RL         RR ↑
```

The key difference is in the `walkY` values:

```
Normal legs[]:                    crabLegs[]:
  FL:  walkY =  3                   FL:  walkY =  8
  ML:  walkY = -1                   ML:  walkY =  0
  RL:  walkY = -4                   RL:  walkY = -8
  FR:  walkY =  0                   FR:  walkY = -8
  MR:  walkY =  2                   MR:  walkY =  0
  RR:  walkY =  6                   RR:  walkY =  8
```

> [!IMPORTANT]
> In crab stance, left legs have `walkY = {8, 0, -8}` and right legs have `walkY = {-8, 0, 8}`. This rotation makes all legs point "forward" in the same direction, enabling uniform push for forward/backward movement.

---

## The Unified 2D Gait

Crab Walk is the only gait that drives **both X and Y axes simultaneously** with independent scaling. This is what makes omnidirectional movement possible.

### Stride Functions

```cpp
// Y-axis stride (forward / backward)
float crabSwingY(int i){
  return crabLegs[i].walkY - (3.0f * crabLegs[i].coxaDir * crabLegs[i].strideScale);
}
float crabDragY(int i){
  return crabLegs[i].walkY + (3.0f * crabLegs[i].coxaDir * crabLegs[i].strideScale);
}

// X-axis stride (strafe left / right)
float crabSwingX(int i){
  return crabLegs[i].restX - (3.0f * crabLegs[i].coxaDir * crabLegs[i].strideScale);
}
float crabDragX(int i){
  return crabLegs[i].restX + (3.0f * crabLegs[i].coxaDir * crabLegs[i].strideScale);
}
```

Both axes use the same stride amplitude (3.0 cm) so diagonal movement is balanced.

---

## Independent X/Y Scaling (`indepXY = true`)

The crab gait has `indepXY = true` in the gait table. This changes how the engine computes scale factors:

### Normal gait (indepXY = false):
```
finalScale = constrain(jFwd + jTurn × coxaDir, -1, 1)
scaleX = scaleY = finalScale
→ Both axes share one differential scale
```

### Crab gait (indepXY = true):
```
scaleY = jFwd      ← drives forward/backward
scaleX = jStrafe   ← drives strafe left/right
→ Each axis has its own independent scale
```

This split happens in [sendPhase()](file:///f:/Arduino/Git/SpiderBot/spiderBotV2/spiderBotV2.7/spiderBotV2.7.ino#303-330):

```cpp
float scaleY = g.indepXY ? jFwd
                         : constrain(jFwd + (jTurn * leg.coxaDir), -1.0f, 1.0f);
float scaleX = g.indepXY ? jStrafe : scaleY;

float stepX = leg.restX + (swingX - leg.restX) * scaleX;  // X driven by jStrafe
float stepY = leg.walkY + (swingY - leg.walkY) * scaleY;  // Y driven by jFwd
```

---

## Button Commands

Each button sets `jFwd` and/or `jStrafe` to ±1.0:

```cpp
void crabForward()  { activeGait=1; jFwd= 1.0; jStrafe= 0.0; ... }
void crabBackward() { activeGait=1; jFwd=-1.0; jStrafe= 0.0; ... }
void crabLeft()     { activeGait=1; jFwd= 0.0; jStrafe= 1.0; ... }
void crabRight()    { activeGait=1; jFwd= 0.0; jStrafe=-1.0; ... }
```

```
                       jFwd = +1
                     (crabForward)
                          ↑
                          │
  jStrafe = +1  ←─────── ● ────────→  jStrafe = -1
   (crabLeft)             │              (crabRight)
                          ↓
                       jFwd = -1
                    (crabBackward)
```

---

## Joystick in Crab Mode

When crab mode is active (`g1` selected) and a joystick command arrives:

```cpp
if (GAIT_TABLE[activeGait].indepXY) {
    jFwd    = rawFwd;        // stick Y → crab fwd/bwd
    jStrafe = -rawTurn;      // stick X → crab strafe
    jTurn   = 0.0f;
}
```

The sign flip on `rawTurn` ensures **right stick = right strafe** (natural mapping).

### Joystick examples:

| Command | rawFwd | rawTurn | jFwd | jStrafe | Movement |
|---------|--------|---------|------|---------|----------|
| `j100,0` | 1.0 | 0.0 | 1.0 | 0.0 | Pure forward |
| `j-100,0` | -1.0 | 0.0 | -1.0 | 0.0 | Pure backward |
| `j0,-100` | 0.0 | -1.0 | 0.0 | 1.0 | Pure left strafe |
| `j0,100` | 0.0 | 1.0 | 0.0 | -1.0 | Pure right strafe |
| `j50,-50` | 0.5 | -0.5 | 0.5 | 0.5 | **Diagonal: forward-left** |
| `j-70,70` | -0.7 | 0.7 | -0.7 | -0.7 | **Diagonal: backward-right** |

---

## How Diagonal Movement Works

With `j50,-50` (forward-left diagonal):

```
jFwd = 0.5,  jStrafe = 0.5

For each swing leg:
  scaleY = 0.5  →  stepY = walkY + (swingY - walkY) × 0.5   (half Y stride)
  scaleX = 0.5  →  stepX = restX + (swingX - restX) × 0.5   (half X stride)

Result: each foot moves in BOTH X and Y simultaneously
        → the body translates at a 45° diagonal
```

```
  Pure Forward:     Forward-Left:      Pure Left:
       ↑                ↗                 ←
       │              ╱                   │
       │            ╱                     │
    (jFwd=1      (jFwd=0.5            (jStrafe=1
     jStrafe=0)   jStrafe=0.5)         jFwd=0)
```

> [!NOTE]
> Both stride amplitudes are 3.0 cm ([crabSwingY](file:///f:/Arduino/Git/SpiderBot/spiderBotV2/spiderBotV2.7/spiderBotV2.7.ino#203-208) and [crabSwingX](file:///f:/Arduino/Git/SpiderBot/spiderBotV2/spiderBotV2.7/spiderBotV2.7.ino#209-210) both use `3.0f`). This ensures that at 45° joystick input, the X and Y displacements are equal — giving a true diagonal, not a biased curve.

---

## Mode Activation

```
g1  →  crabMode = true, activeGait = 1
        Switches leg array from legs[] to crabLegs[]
        F/B/L/R commands remap to crab functions

g0, g2, g3  →  crabMode = false
               Switches back to legs[]
               F/B/L/R use normal tripod functions
```

The `crabMode` flag controls **which leg array** the engine uses:
```cpp
Leg& leg = crabMode ? crabLegs[idx] : legs[idx];
```

While `indepXY` controls **how the scaling works** (independent vs differential).
