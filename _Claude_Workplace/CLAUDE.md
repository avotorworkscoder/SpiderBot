# Master Project Hub: SpiderBot

This is the high-level roadmap and "Source of Truth" for the AI to understand the project's current status and technical stack.

## 1. Project Overview
- **Project Name**: SpiderBot (V3.6 → V5)
- **Platform**: 18 DOF Hexapod (6 Legs x 3 Joints) + 6-DOF Body Control
- **Controller**: ESP32 (Core Logic) + Lobot 32-CH Servo Controller
- **Languages**: C++ (Firmware), Python (Automation)
- **Input**: PS2 DualShock Controller (Primary), Bluetooth Serial (Debug)

## 2. Technical Stack & Dependencies
- **Framework**: Arduino (ESP32)
- **Core Libraries**:
  - `LobotServoController.h` (UART Servo Bus)
  - `PS2X_lib.h` (Controller SPI)
  - `Wire.h` (I2C for MPU-6050)
  - `ArduinoOTA.h` (Wireless flashing)
- **Version History**:
  - `V2.x` - Legacy discrete FSM, BT only.
  - `V3.3` - IMU Self-Levelling & Tactile Sensors.
  - `V3.6` - Phase-Continuous Bezier Engine & 6-DOF Kinematics.
  - `V5.0` - (Upcoming) ESP32-CAM autonomous tracking.

## 3. Core Logic Architecture
The system operates on a highly optimized, non-blocking time-sliced architecture (`SLICE_MS = 40ms`):
- **Phase-Continuous Engine**: Replaced rigid state machines. Each leg maintains a float phase (`0.0 - 1.0`).
- **6-DOF Kinematics**: Implements full 3D Rotational Matrices (Roll, Pitch, Yaw) and Translation (X, Y, Z).
- **Adaptive LERP Terrain**: Uses foot switch feedback and IMU angles to dynamically adjust leg extensions in real-time.

## 4. Directory Structure Reference
- `_Claude_Workplace/`: The AI's instructional brain.
  - `PRIME_INSTRUCTIONS.md`: Strict coding rules.
  - `KNOWLEDGE_BASE.md`: Physical dimensions and pinouts.
  - `SESSION_SUMMARY.md`: Detailed chronological development log.
  - `CHECKLISTS.md`: Pre-flight and deployment verification.
- `spiderBotV3/`: Iterative source code directories (Currently on V3.6).