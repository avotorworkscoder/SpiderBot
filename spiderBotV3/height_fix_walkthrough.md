# Height Adjustment Fix — Root Cause & Step-by-Step Proof

## The Bug: Femur Formula Was 90° Off

### Your physical servo mapping:
```
FEMUR servo:  500µs = UP ↑    1500µs = HORIZONTAL →    2500µs = DOWN ↓
TIBIA servo:  500µs = INSIDE ←  1500µs = STRAIGHT DOWN ↓  2500µs = OUTSIDE →
```

### Old formula (WRONG):
```cpp
p2 = 1500 + (fDeg - 90) × 11.11
```

| fDeg (IK) | p2 (old) | Physical servo position | WANTED |
|---|---|---|---|
| 0° (horizontal) | **500µs** | **Vertically UP** ❌ | Horizontal (1500) |
| 40° (default) | **945µs** | ~50° above horiz | 40° above horiz |
| 90° (vertical up) | **1500µs** | Horizontal | Vertical up (500) |

> [!CAUTION]
> The old formula reversed the entire range! At max height (fDeg=0°, femur should be horizontal), the servo went to 500µs = straight UP. That's why the legs flew outward!

### New formula (CORRECT):
```cpp
p2 = 1500 - fDeg × 11.11
```

| fDeg (IK) | p2 (new) | Physical servo position | Match? |
|---|---|---|---|
| 0° (horizontal) | **1500µs** | Horizontal → | ✅ |
| 40° (default) | **1056µs** | 40° above horiz | ✅ |
| 90° (vertical up) | **500µs** | Vertical UP ↑ | ✅ |

## Step-by-Step: Height from Min to Max

All values use `restX = 12.5`, `walkY = 0`, `perpTibia = true`:

| Height | bodyZ | fDeg | p2 (femur) | p3 (tibia) | Visual |
|---|---|---|---|---|---|
| Crouched | -7.0 | 62° | 811µs | 811µs | Legs folded up tight |
| Low | -8.0 | 50° | 944µs | 944µs | Slightly crouched |
| **Default** | **-9.0** | **40°** | **1056µs** | **1056µs** | **Normal stance** |
| Tall | -10.0 | 30° | 1167µs | 1167µs | Standing tall |
| Taller | -12.0 | 14° | 1344µs | 1344µs | Very tall |
| **Max** | **-14.0** | **~2°** | **~1478µs** | **~1478µs** | **Femur nearly flat, tibia straight down** |

> [!NOTE]
> Notice how p2 and p3 are always equal when `perpTibia = true`! This is because the tibia counter-rotates by exactly the femur's tilt angle. At max height, both are near 1500µs — the rest position of both servos.

## The perpTibia Override

```
if (perpTibia) tDeg = 90.0f + fDeg;
```

This ensures the tibia always points straight down by cancelling the femur's tilt:
- Femur tilted 40° up → tibia rotates 40° to compensate → still vertical ↓
- Femur horizontal (0°) → tibia at 0° compensation → straight down (its natural rest) ↓

## Physical Picture at Max Height

```
     body ═══╗
             ║
    coxa ────● femur (1500µs = horizontal →→→→→→) ── knee
             ║                                       │
             ║                           tibia       │ (1500µs = straight down)
             ║                                       │
             ║                                      foot
                        14.0 cm above ground
```

## What Changed (Summary)

| Change | Old | New |
|---|---|---|
| Femur formula | `1500 + (fDeg-90) × 11.11` | `1500 - fDeg × 11.11` |
| perpTibia flag | _(not present)_ | Overrides tibia to stay vertical |
| restX during height change | Changed via perpX (foot slides) | **Stays constant** (foot planted) |
| Height direction | Inverted (+delta = shorter) | Correct (+delta = taller) |
