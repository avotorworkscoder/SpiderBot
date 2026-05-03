# Coding Style Guide

This style guide ensures consistency between human-written code and AI-generated firmware. It prioritizes the specific memory constraints of the ESP32 and the real-time math requirements of Phase-Continuous 6-DOF kinematics.

## 1. Naming Conventions
- **Constants/Macros**: `UPPER_SNAKE_CASE` (e.g., `const float LIFT_H = 3.0f;`).
- **Variables**: `camelCase` (e.g., `float currentXCoord;`).
- **Functions**: `camelCase` with descriptive verbs (e.g., `updateLevelling()`).
- **Files**: Consistent with existing versioning: `spiderBotV3.6.ino`.

## 2. Formatting & Structure
- **Indentation**: 2 Spaces (Standard Arduino format).
- **Braces**: K&R Style (Opening brace on the same line).
- **Vertical Spacing**: Single line between functions; double line between major logical sections (e.g., Hardware Init vs. Kinematics Matrix).

## 3. Firmware Best Practices (ESP32 Specific)
- **No Blocking**: Use of `delay()` is strictly forbidden in movement logic. Use `millis()`-based time slicing (`lastSlice` checks).
- **Math Optimization**: Use `float` for kinematics but minimize `double` to save ESP32 cycles. Use trigonometric approximations if needed, but `math.h` hardware-accelerated functions (`sin`, `cos`) are preferred on ESP32.
- **Data Types**: Be precise. Use `uint8_t` for pins/IDs, `uint16_t` for timings/pulses, and `float` for geometry.

## 4. Documentation Standards
Header Blocks: Every major section or file must start with a summary:
```cpp
/*===============================================================
 * SpiderBot V3.6 — PHASE-CONTINUOUS GAIT ENGINE
 * ── WHAT'S IN THIS FILE ────────────────────────────────────
 *   1. Hardware + Constants
 *   ...
 *===============================================================*/
```

Inline Comments: Use `// Why: [Intent]` or explain the math visually, rather than just reciting the syntax.
- **Good**: `// Roll is inverted to match the IMU's orientation and prevent positive feedback`
- **Bad**: `// Invert roll`

## 5. Math & Physics
- **Coordinates**: Standard Cartesian (X, Y, Z). Origin (0,0,0) is always the geometric center of the chassis.
- **Angles**: Internal matrix math uses Radians (or Degrees converted instantly via `0.0174533f`). Final IK solver output must convert to PWM Pulse (500-2500) via `DEG_TO_US`.
- **Epsilon**: For float comparisons or drift prevention, use a threshold (e.g., `if (fabsf(a - b) < 0.05f)`).

## 6. Lobot Controller Communication
- **Time-Slicing**: Send batched updates to the LSC-32 exactly every `SLICE_MS` (40ms) to ensure continuous overlapping commands and prevent mechanical jitter.