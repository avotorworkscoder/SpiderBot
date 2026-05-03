# SpiderBot Knowledge Base

This file serves as the "Source of Truth" for the SpiderBot hardware configuration. The Firmware Engineer (AI) must reference these physical constants for all Inverse Kinematics (IK), 6-DOF Matrix Transforms, and gait calculations.

## 1. Physical Dimensions (Leg Geometry)
All measurements are in centimeters (cm). Each of the 6 legs consists of 3 segments.
- **Coxa (COXA)**: 6.0 cm
- **Femur (FEMUR)**: 8.5 cm
- **Tibia (TIBIA)**: 14.5 cm
- **Hip Offsets (from body center)**: X = ±6.1 cm, Y = ±12.0 cm (Middle legs Y = 0.0)

## 2. Servo Pulse Configuration (Lobot LSC-32)
- The Lobot controller operates on a pulse range of 500 to 2500.
- **Center Position**: 1500 (Equivalent to 0°/90°).
- **Pulse to Degree Ratio**: 1 degree ≈ 11.11 pulse units.

## 3. Coordinate System (Global 6-DOF)
- **Origin (0,0,0)**: Geometric center of the hexapod chassis.
- **X-Axis**: Right (+) and Left (-). (Note: Leg side direction `coxaDir` handles mirroring).
- **Y-Axis**: Forward (+) and Backward (-).
- **Z-Axis**: Up (+) and Down (-).
- **Default Standing Height**: `bodyZ = -9.0` cm. Clearance boundaries: -7.0 cm (crouched) to -14.0 cm (tall).

## 4. Leg Mapping (ID System)
IDs correspond to the ports on the Lobot Servo Controller.
- **Front-Left (FL)**: 0 (Coxa), 1 (Femur), 2 (Tibia)
- **Mid-Left (ML)**: 3, 4, 5
- **Rear-Left (RL)**: 29, 30, 31
- **Front-Right (FR)**: 6, 7, 8
- **Mid-Right (MR)**: 9, 10, 11
- **Rear-Right (RR)**: 26, 27, 28

## 5. Hardware Pinout (ESP32)
- **UART2 (Lobot)**: RX = 16, TX = 17 (Baud: 115200)
- **PS2 Controller (SPI)**: DAT/MISO = 19, CMD/MOSI = 23, SEL/SS = 5, CLK/SCK = 18
- **MPU-6050 (I2C)**: SDA = 21, SCL = 22
- **Foot Contact Sensors (Active-LOW)**: FL=32, FR=33, ML=25, MR=26, RL=27, RR=14