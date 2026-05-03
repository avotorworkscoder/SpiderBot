# SpiderBot Pre-Flight & Integration Checklists

This file tracks operational readiness and safety milestones for SpiderBot V3.6+.

## 1. Hardware & Electrical Pre-Flight
- [ ] **Battery Voltage Check**: Ensure 2S LiPo is > 7.4V to prevent ESP32 brownouts during 18-servo surges.
- [ ] **PS2 Connection**: Verify the receiver LED is solid. Code must discard initial `PS2_WARMUP_FRAMES` to prevent junk data execution.
- [ ] **IMU Calibration**: Ensure the MPU-6050 rests flat on boot. Roll/Pitch offsets should be neutralized.
- [ ] **I2C/SPI Bus Check**: Verify no hardware conflicts between MPU-6050 (Pins 21/22) and PS2X (Pins 5/18/19/23).

## 2. Firmware Integration & Math Validation
- [ ] **Non-Blocking Verification**: Confirm `loop()` contains absolutely ZERO `delay()` calls.
- [ ] **6-DOF Bounds Check**: Ensure manual translation/rotation limits don't mathematically push `acos()` out of [-1.0, 1.0] in the IK solver.
- [ ] **Phase Interleaving**: Verify that phase offsets correctly interleave without hardware collision (e.g. `{0, 0.5, 0, 0.5, 0, 0.5}`).
- [ ] **OTA Safety**: Ensure `ArduinoOTA` uses a timeout mechanism to prevent blocking offline boots.

## 3. Terrain & Sensor Calibration
- [ ] **IMU Levelling**: Physically tilt the bot; verify all 6 legs visually compensate via `updateLevelling()` LERP.
- [ ] **Foot Switch Test**: Verify active-low triggers on Pins 32, 33, 25, 26, 27, 14. 
- [ ] **Equilibrium Retraction**: Test that pressing a foot switch upwards causes the leg to gracefully retract rather than "jacking" the chassis up.

## 4. Software Maintenance
- [ ] **Session Documentation**: Update `SESSION_SUMMARY.md` with detailed Bug Insights after major fixes.
- [ ] **Modularization Check**: As the codebase grows, ensure logic is strictly grouped (Gaits vs. Kinematics vs. Hardware).
