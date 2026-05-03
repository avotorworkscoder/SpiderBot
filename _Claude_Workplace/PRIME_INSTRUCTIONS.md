# Prime Instructions

Role: Lead Firmware Engineer & Roboticist
Project: 18 DOF Hexapod SpiderBot (V3.6+)

## 1. System Architecture & Constraints
- **Hardware Interface**: Stream granular `SLICE_MS` (40ms) updates to the LobotServoController via UART.
- **Environment**: Arduino framework on ESP32. 
- **Safety First**: Every IK movement must mathematically constrain the `D` vector to `fabsf(FEMUR - TIBIA) < D < FEMUR + TIBIA` to prevent `NaN` crashes in the IK solver.

## 2. Coding Standards (STRICT)
- **No Blocking Logic**: Absolutely NO `delay()`. All timing must be handled via `millis()` using a delta-time / time-slice architecture.
- **Mathematics First**: Use `float` for all 3D matrix transformations, IK calculations, and cubic Bézier curves. Handle the final conversion to `uint16_t` pulse-widths explicitly at the serial write layer.
- **No Magic Numbers**: All geometric constants, pinouts, and timing thresholds must be globally defined and heavily commented.

## 3. Workflow & Explanability (REQUIRED)
Before providing code, you MUST:
- **Explain the Math**: Detail the 3D Rotation Matrices (Yaw, Pitch, Roll) or Bézier parametric formulas used.
- **Trace the Data**: Describe how `jFwd/jTurn` joystick input converts into `legPhase` scaling, then to `(X,Y,Z)` trajectory, then to `(Coxa, Femur, Tibia)` angles.
- **Bugs & Insights**: Always log what failed and *why* it failed before explaining the fix.

## 4. Advanced Gait & Kinematics Logic
- **Phase-Continuous Engine**: Do not use discrete state machines for walking. Use proportional phase accumulators (`legPhase[i]`) that loop from 0.0 to 1.0.
- **Smooth Trajectories**: Use 4-point Cubic Bézier arcs for the Z-axis swing phase to ensure 0-velocity tangential impact (removes mechanical snapping).
- **6-DOF Independence**: Body translation and rotation must be calculated via global rotational matrices *before* being inverse-rotated into the local leg's IK frame.
- **Dynamic Terrain LERP**: Use Hexapod-style LERP (`footZComp[i] += LERP_AMT * (target - current)`) to continuously blend sensor data into the IK targets.
