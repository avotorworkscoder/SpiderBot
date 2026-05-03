/*===============================================================
 * SpiderBot V3.6 — PHASE-CONTINUOUS GAIT ENGINE
 * Author  : Amit Parihar
 * Wiring  : UART2  RX=16  TX=17  →  LSC-32 servo controller
 *           PS2 controller SPI:
 *             DAT=19 (MISO)  CMD=23 (MOSI)
 *             ATT=5  (SS)    CLK=18 (SCK)
 *           MPU-6050 I2C:
 *             SDA=21  SCL=22  (ESP32 default I2C)
 *           Foot contact sensors (digital, active-LOW w/ pull-ups):
 *             FL=34  ML=35  RL=32  FR=33  MR=25  RR=26
 *
 * ── SECTIONS IN THIS FILE ────────────────────────────────────
 *   1.  Hardware + Constants + Leg Data
 *   2.  Computational Kinematics (3D Rotational Matrices)
 *   3.  Gait Shapes         — plug-and-play stride definitions
 *   4.  Command Queue       — circular buffer, never drops a cmd
 *   5.  Gait Engine         — Phase-Continuous Per-Leg Engine  ★ NEW
 *   6.  Movement Funcs      — walk, turn, rotate, crab, diagonal
 *   7.  Height Control      — raise / lower body at runtime
 *   8.  BT Input Parser     — BT + Serial command handler
 *   9.  PS2 Controller      — primary physical controller
 *  10.  OTA Setup           — wireless firmware updates
 *  11.  MPU-6050 (IMU)      — roll/pitch stabilisation + fall detect
 *  12.  Foot Contact        — LERP-based Hexapod terrain adaptation
 *  13.  setup() + loop()
 *
 * ── V3.6 — PHASE-CONTINUOUS PER-LEG ENGINE ───────────────────
 *   Inspired by the reference hexapod (hexapod/src/States/WalkingState.cpp).
 *
 *   Key insight: true smoothness requires updating servo targets EVERY loop,
 *   not waiting for keyframes. The state machine is abolished entirely.
 *
 *   How it works:
 *     • Each leg owns a float `legPhase[i]` (0.0 → 1.0, wraps).
 *     • Every gaitTick() call advances ALL legPhase values by `phaseStep`.
 *     • `phaseStep` is proportional to joystick magnitude — the faster
 *       you push the stick, the faster the phase advances.
 *     • Phase < pushFraction  → leg is on ground (push/drag phase):
 *         XY moves linearly from stride-front to stride-rear (body moves fwd).
 *         Z = 0 (foot on floor). Uses LERP from cycleStartPoint.
 *     • Phase >= pushFraction → leg is in swing (lift phase):
 *         XY returns from rear to front using a 4-point Bézier curve.
 *         Z follows a Bézier arc for smooth lift and soft landing.
 *     • Each leg's phase is OFFSET by its gait slot so legs interleave
 *         naturally — no group A / group B distinction.
 *     • Result: completely continuous, ripple-like motion with no visible
 *         keyframe transitions.
 *
 *   SLICE_MS (40ms) governs how often positions are re-sent to the LSC-32.
 *   LSC-32 time field = SLICE_MS so the servo always has a fresh target
 *   arriving exactly when the previous move completes.
 *
 * ── PHASE 2 INTEGRATION (3D BODY KINEMATICS) ─────────────────
 *   + True 3D Rotational Matrix inside moveFootXYZ keeps feet
 *     planted while the body tilts to compensate IMU readings.
 *
 * ── BUTTON MAP ───────────────────────────────────────────────
 *   - Left stick  → jFwd (Y) + jTurn (X) — differential drive
 *   - Right stick → jStrafe (X-axis, crab/diagonal)
 *   - D-pad       → Walk / Strafe (discrete mode)
 *   - ○ / □       → Rotate Left / Right (held)
 *   - △ / ×       → Height Up / Down (pressed)
 *   - L1 / R1     → Sit / Sleep postures (pressed)
 *   - L2 / R2     → Speed down / up (pressed)
 *   - START       → Stop / Stand Up (pressed)
 *   - SELECT      → Cycle gaits  g0→g1→g2→g3→g0 (pressed)
 *   - L3 (click)  → Toggle crab mode
 *   - R3 (click)  → Master ON/OFF toggle
 *
 * ── BT DEBUG COMMANDS ────────────────────────────────────────
 *   F/B/L/R/W/X/A/Z/H/U/D/S/+/-/?   (Standard Movement)
 *   I         Toggle IMU Levelling ON/OFF
 *   T         Toggle Tactile Terrain Sensors ON/OFF
 *   g0…g3     Gait switch
 *   imu       Print current roll/pitch to BT
 *   P         Toggle Serial Plotter
 *===============================================================*/

#include <Arduino.h>
#include <math.h>
#include "BluetoothSerial.h"
#include <LobotServoController.h>
#include <PS2X_lib.h>
#include <Wire.h>           // MPU-6050 I2C
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
  #error "Bluetooth not enabled — Arduino IDE: Tools → ESP32 Bluetooth"
#endif

// CHANGE THESE TO YOUR HOME ROUTER INFO to enable wireless flashing
const char* WIFI_SSID     = "A.'s Galaxy A52";
const char* WIFI_PASSWORD = "amit1234";


struct FootPos { float x, y, z; };

// ───────────────────────────────────────────────────────────────
//  SECTION 1 — HARDWARE, CONSTANTS, LEG DATA
// ───────────────────────────────────────────────────────────────

// ── Serial / servo hardware ────────────────────────────────────
HardwareSerial       LSC(2);
BluetoothSerial      SerialBT;
LobotServoController controller(LSC);

// ── PS2 controller ─────────────────────────────────────────────
PS2X ps2x;

#define PS2_DAT  19   // MISO
#define PS2_CMD  23   // MOSI
#define PS2_SEL   5   // SS
#define PS2_CLK  18   // SCK

const int PS2_DEADZONE = 20;
uint8_t ps2Warm = 0;  // counts warmup frames after controller connects; resets on blocking calls

// ── Leg geometry (cm) ──────────────────────────────────────────
const float COXA   = 6.0f;
const float FEMUR  = 8.5f;
const float TIBIA  = 14.5f;
const float LIFT_H = 3.0f;  // Reduced for softer, quieter landing

// ── Servo pulse constants ───────────────────────────────────────
const int   SERVO_MID = 1500;
const float DEG_TO_US = 11.11f;

// ── Runtime-tunable parameters ─────────────────────────────────
uint16_t moveTime = 500;
float    bodyX    = 0.0f;
float    bodyY    = 0.0f;
float    bodyZ    = -9.0f;
float    bodyYaw  = 0.0f;

const uint16_t SPEED_MIN  = 150;
const uint16_t SPEED_MAX  = 1000;
const uint16_t SPEED_STEP = 50;

const float HEIGHT_STEP = 1.0f;
const float HEIGHT_LOW  = -7.0f;    // shortest stance (femur ~62° up, crouched)
const float HEIGHT_HIGH = -14.0f;   // tallest stance  (femur nearly horizontal)

// ── Status ──────────────────────────────────────────────────────
bool          btConnected        = false;
float         voltageInVolts     = 0.0f;
unsigned long lastVoltageRequest = 0;
const char*   lastCmdName        = "NONE";
bool          crabMode           = false;
bool          isBotOn            = true;

// ── Logging helpers ─────────────────────────────────────────────
void btLog(const char* msg) { SerialBT.print(msg);    Serial.print(msg);    }
void btLog(int          num) { SerialBT.print(num);    Serial.print(num);    }
void btLog(float        num) { SerialBT.print(num, 2); Serial.print(num, 2); }

// ── Leg data structure ──────────────────────────────────────────
struct Leg {
  uint8_t id;
  float   hipX, hipY;                // Offset from centre of body to hip joint (cm)
  float   restX, walkY, rotY, restZ; // Foot position relative to hip
  int     coxaDir;                   // +1 for Left side, -1 for Right side
  float   strideScale;
};

// Based on standard rectangular hexapods (adjust hipX, hipY if SpiderBot differs):
// +Y is Forward. +X is Right side.
//            id    hx    hy      X    walkY  rotY    Z   dir  scale
Leg legs[6] = {
  {  0, -6.1,  12.0,  12.5,   3,    0,   -9,  +1,  1.0f },  // 0  Front-Left
  {  3, -6.1,  0.0,  12.5,  -1,    0,   -9,  +1,  1.0f },  // 1  Mid-Left
  { 29, -6.1, -12.0,  12.5,  -4,    0,   -9,  +1,  1.0f },  // 2  Rear-Left
  {  6,  6.1,  12.0,  12.5,   0,    0,   -9,  -1,  1.0f },  // 3  Front-Right
  {  9,  6.1,  0.0,  12.5,   2,    0,   -9,  -1,  1.0f },  // 4  Mid-Right
  { 26,  6.1, -12.0,  12.5,   6,    0,   -9,  -1,  1.0f },  // 5  Rear-Right
};

Leg crabLegs[6] = {
  {  0, -6.1,  12.0,  12.5,   12,   12,   -12,  +1,  1.0f },  // 0  Front-Left
  {  3, -6.1,  0.0,  12.5,   0,   0,   -12,  +1,  1.0f },  // 1  Mid-Left
  { 29, -6.1, -12.0,  12.5,  -12,  -12,   -12,  +1,  1.0f },  // 2  Rear-Left
  {  6,  6.1,  12.0,  12.5,  -12,  -12,   -12,  -1,  1.0f },  // 3  Front-Right
  {  9,  6.1,  0.0,  12.5,   0,   0,   -12,  -1,  1.0f },  // 4  Mid-Right
  { 26,  6.1, -12.0,  12.5,   12,   12,   -12,  -1,  1.0f },  // 5  Rear-Right
};

const int GRP_A[3]    = { 0, 2, 4 };
const int GRP_B[3]    = { 1, 3, 5 };
const int WAVE_SEQ[6] = { 2, 1, 0, 5, 4, 3 };


// ───────────────────────────────────────────────────────────────
//  SECTION 11 — MPU-6050 (IMU)
//
//  Uses raw I2C (no external library needed beyond Wire.h).
//  Complementary filter: angle = α*(angle+gyro*dt) + (1-α)*accelAngle
//  α = 0.98 is typical; lower = more accel influence (slower, stable).
// ───────────────────────────────────────────────────────────────

#define MPU_ADDR      0x68   // AD0 pin LOW → 0x68  (HIGH → 0x69)
#define MPU_PWR_MGMT  0x6B
#define MPU_ACCEL_OUT 0x3B
#define MPU_GYRO_OUT  0x43

// Tunable IMU parameters
const float IMU_ALPHA      = 0.98f;  // complementary filter weight
const float IMU_COMP_MAX   = 5.0f;   // max Z compensation per leg (cm)
const float IMU_FALL_DEG   = 70.0f;  // tilt beyond this → auto-sleep
const float IMU_LEVEL_DEAD = 1.5f;   // ignore tilt below this (degrees)

float imuRoll  = 0.0f;  // raw complementary filter output
float imuPitch = 0.0f;  
float smoothRoll  = 0.0f; // LERP smoothed for stable levelling
float smoothPitch = 0.0f;
float bodyRoll    = 0.0f; // Integral controller accumulated roll
float bodyPitch   = 0.0f; // Integral controller accumulated pitch
const float IMU_MAX_ANGLE = 20.0f;
bool  imuReady  = false;
bool  imuActive = false;
bool  imuPlotting = false; // Toggle for serial plotter output

// Offsets
int AX_OFFSET = -950;
int AY_OFFSET = 7;
int AZ_OFFSET = 94;
int GX_OFFSET = 118;
int GY_OFFSET = -123;
int GZ_OFFSET = -62;

unsigned long lastImuRead = 0;
const unsigned long IMU_INTERVAL_MS = 20;  // 50 Hz

// ── Continuous standstill levelling state ───────────────────────
// Tracks the last roll/pitch values actually sent to the servos.
// updateLevelling() uses these to decide whether a re-send is needed.
float         lastLevelRoll    = 0.0f;
float         lastLevelPitch   = 0.0f;
unsigned long lastLevelUpdate  = 0;
const unsigned long LEVEL_INTERVAL_MS  = 50;    // fast update rate for smooth tracking
const float         LEVEL_CHANGE_THRESH = 0.1f; // degrees — LERP handles the noise safely

// ───────────────────────────────────────────────────────────────
//  SECTION 12 — FOOT CONTACT SENSORS
// ───────────────────────────────────────────────────────────────
const int FOOT_PINS[6]   = { 32, 33, 25, 26, 27, 14 };
bool footOnGround[6]     = { false };
bool footSensorsActive   = false;
float footZComp[6]       = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }; // Smoothed current position
float targetFootZComp[6] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }; // Desired target
const float FOOT_EXT_RATE = 0.5f;   // Aggressive downward search step
const float FOOT_COMP_MAX = -5.0f;  // Max downward extension (cm)
const float FOOT_LERP_AMT = 0.15f;  // Hexapod-style smoothing factor (0.0 to 1.0)

void updateFootSensors() {
  for (int i = 0; i < 6; i++) {
    footOnGround[i] = (digitalRead(FOOT_PINS[i]) == LOW);
  }
}

bool imuInit() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_PWR_MGMT);
  Wire.write(0x00);  // wake up the MPU
  if (Wire.endTransmission(true) != 0) return false;
  delay(100);
  return true;
}

// Call at IMU_INTERVAL_MS cadence from loop().
// Updates imuRoll and imuPitch in degrees.
void updateIMU() {
  unsigned long now = millis();
  if (now - lastImuRead < IMU_INTERVAL_MS) return;
  float dt = (now - lastImuRead) / 1000.0f;
  lastImuRead = now;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_ACCEL_OUT);
  if (Wire.endTransmission(false) != 0) return;
  Wire.requestFrom(MPU_ADDR, 14, true);
  if (Wire.available() < 14) return;

  int16_t ax = (int16_t)((Wire.read() << 8) | Wire.read()) + AX_OFFSET;
  int16_t ay = (int16_t)((Wire.read() << 8) | Wire.read()) + AY_OFFSET;
  int16_t az = (int16_t)((Wire.read() << 8) | Wire.read()) + AZ_OFFSET;
  Wire.read(); Wire.read(); // skip temp
  int16_t gx = (int16_t)((Wire.read() << 8) | Wire.read()) + GX_OFFSET;
  int16_t gy = (int16_t)((Wire.read() << 8) | Wire.read()) + GY_OFFSET;
  int16_t gz = (int16_t)((Wire.read() << 8) | Wire.read()) + GZ_OFFSET;

  // Accelerometer angles (noisy but absolute)
  float fax = ax / 16384.0f;  // ±2g range
  float fay = ay / 16384.0f;
  float faz = az / 16384.0f;
  float accelRoll  =  atan2(fay, faz) * 57.2957795f;
  float accelPitch = -atan2(fax, sqrt(fay*fay + faz*faz)) * 57.2957795f;

  // Gyroscope rate (degrees/s)
  float fgx = gx / 131.0f;
  float fgy = gy / 131.0f;

  // Complementary filter (absolute, no drift, but has high-frequency jitter)
  imuRoll  = IMU_ALPHA * (imuRoll  + fgx * dt) + (1.0f - IMU_ALPHA) * accelRoll;
  imuPitch = IMU_ALPHA * (imuPitch + fgy * dt) + (1.0f - IMU_ALPHA) * accelPitch;

  // LERP smoothing for jitter-free mechanical compensation (exponential easing)
  smoothRoll  += 0.1f * (imuRoll  - smoothRoll);
  smoothPitch += 0.1f * (imuPitch - smoothPitch);

  // Integral controller to force body completely horizontal
  if (imuActive) {
    if (fabsf(smoothRoll) > IMU_LEVEL_DEAD) {
      bodyRoll += smoothRoll * 0.02f;
      bodyRoll = constrain(bodyRoll, -IMU_MAX_ANGLE, IMU_MAX_ANGLE);
    }
    if (fabsf(smoothPitch) > IMU_LEVEL_DEAD) {
      bodyPitch += smoothPitch * 0.02f;
      bodyPitch = constrain(bodyPitch, -IMU_MAX_ANGLE, IMU_MAX_ANGLE);
    }
  } else {
    bodyRoll += 0.05f * (0.0f - bodyRoll);
    bodyPitch += 0.05f * (0.0f - bodyPitch);
  }
}

// Check for fall (tilt exceeds threshold); auto-sleep if so.
bool checkFall() {
  if (fabsf(imuRoll) > IMU_FALL_DEG || fabsf(imuPitch) > IMU_FALL_DEG) {
    btLog(">> FALL DETECTED — auto-sleep!\n");
    return true;
  }
  return false;
}

// Find leg index from servo ID
int getLegIndex(uint8_t id) {
  for (int i=0; i<6; i++) if (legs[i].id == id) return i;
  return 0;
}


// ───────────────────────────────────────────────────────────────
//  SECTION 2 — COMPUTATIONAL KINEMATICS & IK SOLVER
// ───────────────────────────────────────────────────────────────

// 2a. Raw IK Solver (Local Leg Space)
void solveIK(uint8_t id, float x, float y, float z, uint16_t t, bool perpTibia = false) {

  float coxaAngle = atan2(y, x);

  float R = sqrt(x*x + y*y) - COXA;
  if (R < 0.5f) R = 0.5f;

  float D = sqrt(R*R + z*z);
  D = constrain(D, fabsf(FEMUR - TIBIA) + 0.1f, FEMUR + TIBIA - 0.1f);

  float tibiaAngle = acos(constrain(
    (FEMUR*FEMUR + TIBIA*TIBIA - D*D) / (2*FEMUR*TIBIA), -1.0f, 1.0f));
  float femurAngle = atan2(z, R) + acos(constrain(
    (FEMUR*FEMUR + D*D - TIBIA*TIBIA) / (2*FEMUR*D), -1.0f, 1.0f));

  float cDeg = coxaAngle  * 57.2957795f;
  float fDeg = femurAngle * 57.2957795f;
  float tDeg = 180.0f - (tibiaAngle * 57.2957795f);

  if (perpTibia) tDeg = 90.0f + fDeg;

  uint16_t p1 = constrain((int)(SERVO_MID +  cDeg  * DEG_TO_US), 500, 2500);
  uint16_t p2 = constrain((int)(SERVO_MID -  fDeg  * DEG_TO_US), 500, 2500);
  uint16_t p3 = constrain((int)(SERVO_MID - (tDeg - 90.0f) * DEG_TO_US), 500, 2500);

  byte pkt[16] = {
    0x55, 0x55, 14, 0x03, 3,
    lowByte(t),     highByte(t),
    id,             lowByte(p1), highByte(p1),
    uint8_t(id+1),  lowByte(p2), highByte(p2),
    uint8_t(id+2),  lowByte(p3), highByte(p3)
  };
  LSC.write(pkt, 16);
}

// 2b. Phase 2 Global Body Kinematics (3D Rotational Matrix)
// Replaces simple imuZCompensation. Keeps foot firmly planted at global (x,y,z) while body tilts!
void moveFootXYZ(uint8_t id, float x, float y, float z, uint16_t t, bool perpTibia = false) {
  int idx = getLegIndex(id);
  Leg& leg = crabMode ? crabLegs[idx] : legs[idx];

  // 1. World coordinate of foot relative to body center
  // Note: V3 maps x as outward extension. Left legs (dir=+1) outward is -X in standard Right/Fwd/Up space.
  float footWorldX = leg.hipX + (x * -leg.coxaDir);
  float footWorldY = leg.hipY + y;
  
  // Combine commanded Z height with sensor terrain mapping (if any)
  float footWorldZ = z + footZComp[idx]; 

  // 2. Body tilts (Roll around Y axis, Pitch around X axis, Yaw around Z axis)
  // Roll is inverted to match the IMU's orientation
  float cp = cos(bodyPitch * 0.0174533f);
  float sp = sin(bodyPitch * 0.0174533f);
  float cr = cos(-bodyRoll * 0.0174533f);
  float sr = sin(-bodyRoll * 0.0174533f);
  float cy = cos(bodyYaw * 0.0174533f);
  float sy = sin(bodyYaw * 0.0174533f);

  // 3. New Global position of the Hip attached to the tilted/translated body
  float hx_2 = leg.hipX * cr;
  float hy_2 = leg.hipY * cp + leg.hipX * sp * sr;
  float hz_2 = leg.hipY * sp - leg.hipX * cp * sr;

  float rHipX = hx_2 * cy - hy_2 * sy + bodyX;
  float rHipY = hx_2 * sy + hy_2 * cy + bodyY;
  float rHipZ = hz_2;

  // 4. Vector from the tilted Hip to the locked Global Foot
  float vecX = footWorldX - rHipX;
  float vecY = footWorldY - rHipY;
  float vecZ = footWorldZ - rHipZ;

  // 5. Inverse Rotate vector back into the Leg's Local Tilted Frame
  // Undo Yaw (Z-axis)
  float ix = vecX * cy + vecY * sy;
  float iy = -vecX * sy + vecY * cy;
  float iz = vecZ;

  // Undo Pitch (X-axis)
  float jx = ix;
  float jy = iy * cp + iz * sp;
  float jz = -iy * sp + iz * cp;

  // Undo Roll (Y-axis)
  float localX = jx * cr - jz * sr;
  float localY = jy;
  float localZ = jx * sr + jz * cr;

  // 6. Map back to IK solver array format
  x = localX * -leg.coxaDir;
  y = localY;
  z = localZ;

  solveIK(id, x, y, z, t, perpTibia);
}


// ───────────────────────────────────────────────────────────────
//  SECTION 3 — GAIT SHAPES
// ───────────────────────────────────────────────────────────────

float defaultX    (int i){ return legs[i].restX;     }
float defaultCrabX(int i){ return crabLegs[i].restX; }
float defaultY    (int i){ return legs[i].walkY;     }
float defaultCrabY(int i){ return crabLegs[i].walkY; }

float tripodSwingY(int i){ return legs[i].walkY - (3.0f * legs[i].coxaDir * legs[i].strideScale); }
float tripodDragY (int i){ return legs[i].walkY + (3.0f * legs[i].coxaDir * legs[i].strideScale); }

float crabSwingY(int i){ return crabLegs[i].walkY - (3.0f * crabLegs[i].coxaDir * crabLegs[i].strideScale); }
float crabDragY (int i){ return crabLegs[i].walkY + (3.0f * crabLegs[i].coxaDir * crabLegs[i].strideScale); }
float crabSwingX(int i){ return crabLegs[i].restX - (3.0f * crabLegs[i].coxaDir * crabLegs[i].strideScale); }
float crabDragX (int i){ return crabLegs[i].restX + (3.0f * crabLegs[i].coxaDir * crabLegs[i].strideScale); }

float walkSwingX(int i){ return legs[i].restX - (3.0f * legs[i].coxaDir * legs[i].strideScale); }
float walkDragX (int i){ return legs[i].restX + (3.0f * legs[i].coxaDir * legs[i].strideScale); }

float waveSwingY(int i){ return legs[i].walkY - (2.5f * legs[i].coxaDir * legs[i].strideScale); }
float waveDragY (int i){ return legs[i].walkY + (2.5f * legs[i].coxaDir * legs[i].strideScale); }

struct GaitDef {
  const char* name;
  float(*swingX)(int);  float(*dragX)(int);
  float(*swingY)(int);  float(*dragY)(int);
  bool indepXY;
};

GaitDef GAIT_TABLE[] = {
  { "Tripod Walk",   defaultX,   defaultX,  tripodSwingY, tripodDragY, false },
  { "Crab Walk",     crabSwingX, crabDragX, crabSwingY,   crabDragY,   true  },
  { "Diagonal Walk", walkSwingX, walkDragX, tripodSwingY, tripodDragY, true  },
  { "Wave Gait",     defaultX,   defaultX,  waveSwingY,   waveDragY,   false },
};
const uint8_t GAIT_COUNT = sizeof(GAIT_TABLE) / sizeof(GAIT_TABLE[0]);

uint8_t activeGait = 0;


// ───────────────────────────────────────────────────────────────
//  SECTION 4 — COMMAND QUEUE  (8-slot circular buffer)
// ───────────────────────────────────────────────────────────────
const uint8_t QUEUE_SIZE = 8;
char          cmdQueue[QUEUE_SIZE];
uint8_t       qHead = 0, qTail = 0, qCount = 0;

void queuePush(char c) {
  if (qCount >= QUEUE_SIZE) return;
  cmdQueue[qTail] = c;
  qTail = (qTail + 1) % QUEUE_SIZE;
  qCount++;
}

bool queuePop(char& out) {
  if (qCount == 0) return false;
  out   = cmdQueue[qHead];
  qHead = (qHead + 1) % QUEUE_SIZE;
  qCount--;
  return true;
}

void queueClear() { qHead = qTail = qCount = 0; }


// ───────────────────────────────────────────────────────────────
//  SECTION 5 — PHASE-CONTINUOUS PER-LEG GAIT ENGINE  (V3.6 NEW)
// ───────────────────────────────────────────────────────────────
//
//  Architecture (based on hexapod/src/States/WalkingState.cpp):
//
//  • Each leg has its own legPhase[i] in [0.0, 1.0).
//  • gaitTick() advances legPhase[i] by phaseStep every SLICE_MS.
//  • Phase < pushFraction  → PUSH: foot on ground, drags body forward.
//  • Phase >= pushFraction → SWING: foot in air, arcs back to front.
//  • Legs start at evenly-spaced phase offsets (e.g. tripod: 0,0.5,0,0.5,0,0.5)
//    so they naturally interleave without any group A/B distinction.
//
//  No state machine. No waiting. Positions sent every SLICE_MS.
// ───────────────────────────────────────────────────────────────

float jFwd    = 0.0f;
float jTurn   = 0.0f;
float jStrafe = 0.0f;

// Tripod offsets: legs {0,2,4} start at phase 0.0, legs {1,3,5} at 0.5
// This produces the classic alternating tripod without any explicit grouping.
const float TRIPOD_OFFSETS[6]  = { 0.0f, 0.5f, 0.0f, 0.5f, 0.0f, 0.5f };
const float RIPPLE_OFFSETS[6]  = { 0.0f, 0.67f, 0.33f, 0.5f, 0.17f, 0.83f };

// pushFraction: fraction of the cycle spent on the ground (pushing).
// 0.6 = 60% push, 40% swing. Higher = more stable, slower apparent swing.
const float PUSH_FRACTION = 0.60f;

// How often we recalculate and resend positions to the LSC-32.
const unsigned long SLICE_MS = 40;

// phaseStep per SLICE_MS tick. Scaled by joystick magnitude in gaitTick().
// Base value: at full stick (|j|=1) one full cycle takes moveTime ms.
// phaseStep = SLICE_MS / moveTime  at full speed.
// This is computed dynamically inside gaitTick().

float legPhase[6]       = { 0.0f }; // Current phase for each leg
bool  legInSwing[6]     = { false }; // Track push→swing transitions
float cycleStartX[6]    = { 0.0f }; // Foot X at start of current cycle segment
float cycleStartY[6]    = { 0.0f }; // Foot Y at start of current cycle segment
bool  isWalking         = false;
unsigned long lastSlice  = 0;

void handleCommand(char cmd); // forward declaration

// Bézier helper (cubic)
float bezierCubic(float t, float p0, float p1, float p2, float p3) {
  float u = 1.0f - t;
  return (u*u*u)*p0 + 3.0f*(u*u)*t*p1 + 3.0f*u*(t*t)*p2 + (t*t*t)*p3;
}

// ── getGaitPoint ───────────────────────────────────────────────
// Returns the 3D foot position (x,y,z) for leg `idx` given its current phase.
// Push phase: linear LERP from front-stride to rear-stride (foot on ground).
// Swing phase: 4-point Bézier from rear back to front with lift arc.
// ──────────────────────────────────────────────────────────────

FootPos getGaitPoint(int idx) {
  GaitDef& g  = GAIT_TABLE[activeGait];
  Leg&     leg = crabMode ? crabLegs[idx] : legs[idx];

  float scale  = g.indepXY ? jFwd
                           : constrain(jFwd + jTurn * leg.coxaDir, -1.0f, 1.0f);
  float scaleX = g.indepXY ? jStrafe : scale;

  // Stride endpoints for this leg
  float frontX = leg.restX + (g.swingX(idx) - leg.restX) * scaleX;
  float rearX  = leg.restX + (g.dragX(idx)  - leg.restX) * scaleX;
  float frontY = leg.walkY + (g.swingY(idx) - leg.walkY) * scale;
  float rearY  = leg.walkY + (g.dragY(idx)  - leg.walkY) * scale;

  float phase = legPhase[idx];

  if (phase < PUSH_FRACTION) {
    // ── PUSH PHASE: foot on ground, linearly sweep rear → front (body moves fwd) ──
    // On entry to push, record current foot position as cycle start.
    float tPush = phase / PUSH_FRACTION;  // normalised 0→1 within push

    float x = cycleStartX[idx] + (rearX - cycleStartX[idx]) * tPush;
    float y = cycleStartY[idx] + (rearY - cycleStartY[idx]) * tPush;
    return { x, y, 0.0f };

  } else {
    // ── SWING PHASE: 4-point Bézier arc rear → front with height ──
    float tSwing = (phase - PUSH_FRACTION) / (1.0f - PUSH_FRACTION); // 0→1

    // XY: linear from rear to front
    float x = rearX + (frontX - rearX) * tSwing;
    float y = rearY + (frontY - rearY) * tSwing;

    // Z: Bézier arc  0 → peak → near-land height → 0
    float z = bezierCubic(tSwing,
                          0.0f,
                          LIFT_H * 1.6f,
                          LIFT_H * 1.6f,
                          0.0f);
    return { x, y, z };
  }
}

// ── initLegPhases ─────────────────────────────────────────────
// Sets phase offsets for the active gait and records cycle start positions.
void initLegPhases() {
  for (int i = 0; i < 6; i++) {
    legPhase[i]    = (activeGait == 0) ? TRIPOD_OFFSETS[i] : RIPPLE_OFFSETS[i];
    legInSwing[i]  = (legPhase[i] >= PUSH_FRACTION);
    Leg& leg       = crabMode ? crabLegs[i] : legs[i];
    cycleStartX[i] = leg.restX;
    cycleStartY[i] = leg.walkY;
  }
}

// ── gaitTick ──────────────────────────────────────────────────
// Called every loop(). Advances all leg phases and resends positions
// to the LSC-32 every SLICE_MS.
void gaitTick() {
  if (!isWalking) return;

  unsigned long now = millis();
  if (now - lastSlice < SLICE_MS) return;
  lastSlice = now;

  // phaseStep: fraction of a full cycle to advance each SLICE_MS tick.
  // At full joystick (|j|=1), one cycle = moveTime. Scaled with speed.
  float jMag     = max(fabsf(jFwd), max(fabsf(jTurn), fabsf(jStrafe)));
  float phaseStep = (SLICE_MS / (float)moveTime) * jMag;

  // Stop walking if joystick is released
  if (jMag < 0.05f) {
    isWalking = false;
    for (int i = 0; i < 6; i++) {
      Leg& leg = crabMode ? crabLegs[i] : legs[i];
      moveFootXYZ(leg.id, leg.restX, leg.walkY, leg.restZ, moveTime / 2);
    }
    return;
  }

  for (int i = 0; i < 6; i++) {
    bool wasInSwing = (legPhase[i] >= PUSH_FRACTION);

    // Advance phase
    legPhase[i] += phaseStep;
    if (legPhase[i] >= 1.0f) legPhase[i] -= 1.0f;

    bool nowInSwing = (legPhase[i] >= PUSH_FRACTION);

    // Detect transition from swing → push: record current position as cycle start
    if (wasInSwing && !nowInSwing) {
      FootPos fp = getGaitPoint(i);
      cycleStartX[i] = fp.x;
      cycleStartY[i] = fp.y;
    }
  }

  // Send updated positions to all legs via LSC-32
  for (int i = 0; i < 6; i++) {
    Leg&    leg = crabMode ? crabLegs[i] : legs[i];
    FootPos fp  = getGaitPoint(i);
    moveFootXYZ(leg.id, fp.x, fp.y, leg.restZ + fp.z, (uint16_t)SLICE_MS);
  }
}

void startGait(bool continuous) {
  for (int i = 0; i < 6; i++) {
    footZComp[i] = targetFootZComp[i] = 0.0f;
  }
  initLegPhases();
  isWalking = true;
  lastSlice = millis();
}

void stopGait() {
  isWalking = false;
  for (int i = 0; i < 6; i++) {
    Leg& leg = crabMode ? crabLegs[i] : legs[i];
    moveFootXYZ(leg.id, leg.restX, leg.walkY, leg.restZ, moveTime / 2);
  }
}

void recenterLegs() {
  for (int i = 0; i < 6; i++) {
    Leg& leg = crabMode ? crabLegs[i] : legs[i];
    moveFootXYZ(leg.id, leg.restX, leg.walkY, leg.restZ, moveTime / 2);
  }
}


// ───────────────────────────────────────────────────────────────
//  SECTION 6 — MOVEMENT FUNCTIONS
// ───────────────────────────────────────────────────────────────

void standStill() {
  stopGait();
  queueClear();
  jFwd = jTurn = jStrafe = 0.0f;
  for (int i = 0; i < 6; i++) {
    footZComp[i] = 0.0f;
    targetFootZComp[i] = 0.0f;
  }
  // perpTibia=true → tibia always hangs straight down while standing
  for (int i = 0; i < 6; i++) {
    Leg& leg = crabMode ? crabLegs[i] : legs[i];
    moveFootXYZ(leg.id, leg.restX, leg.walkY, leg.restZ, moveTime, true);
  }
  // Reset levelling tracker so the next tilt is always picked up immediately
  lastLevelRoll  = bodyRoll;
  lastLevelPitch = bodyPitch;
  lastLevelUpdate = millis();
}

// ── updateLevelling() ───────────────────────────────────────────
// Runs from loop().
// Re-sends leg positions whenever the IMU reports a tilt change.
// Now supports dynamic Terrain Adaptation while walking!
void updateLevelling() {
  if (!isBotOn) return;

  unsigned long now = millis();
  if (now - lastLevelUpdate < LEVEL_INTERVAL_MS) return;

  bool postureChanged = false;

  // 1. Check IMU changes
  if (imuReady && imuActive) {
    float dRoll  = fabsf(bodyRoll  - lastLevelRoll);
    float dPitch = fabsf(bodyPitch - lastLevelPitch);
    if (dRoll >= LEVEL_CHANGE_THRESH || dPitch >= LEVEL_CHANGE_THRESH) {
      postureChanged = true;
      lastLevelRoll  = bodyRoll;
      lastLevelPitch = bodyPitch;
    }
  }

  // 2. Check Foot Sensors & apply Hexapod-style LERP smoothing
  if (footSensorsActive) {
    float maxComp = -999.0f;
    for (int i = 0; i < 6; i++) {
      // If walking, ONLY hunt for ground if this leg is in the PUSH phase.
      // If in SWING phase, smoothly retract compensation to 0.0 so the foot lifts cleanly.
      if (isWalking && legPhase[i] >= PUSH_FRACTION) {
        targetFootZComp[i] = 0.0f;
      } else {
        // Target hunting:
        if (!footOnGround[i]) {
          // In air: search downwards aggressively
          targetFootZComp[i] -= FOOT_EXT_RATE;
          if (targetFootZComp[i] < FOOT_COMP_MAX) targetFootZComp[i] = FOOT_COMP_MAX;
        } else {
          // On ground: softly retract upwards to find equilibrium and yield to rising terrain
          targetFootZComp[i] += FOOT_EXT_RATE * 0.4f;
          if (targetFootZComp[i] > 3.0f) targetFootZComp[i] = 3.0f;
        }
      }
      
      if (targetFootZComp[i] > maxComp) {
        maxComp = targetFootZComp[i];
      }
    }

    // Prevent body jacking / ratchet effect: 
    // Normalise all legs so the highest foot stays at exactly 0.0 (body height).
    if (maxComp < 0.0f && !isWalking) {
      for (int i = 0; i < 6; i++) {
        targetFootZComp[i] -= maxComp; // maxComp is negative, subtracting adds positive value
      }
    }

    for (int i = 0; i < 6; i++) {
      // Apply hexapod logic LERP: distanceFromGround = distanceFromGround + smoothingFactor * (target - current)
      if (fabsf(targetFootZComp[i] - footZComp[i]) > 0.02f) {
        footZComp[i] += FOOT_LERP_AMT * (targetFootZComp[i] - footZComp[i]);
        postureChanged = true;
      }
    }
  } else {
    // Softly retract legs back to absolute perfectly-flat posture if turned off.
    for (int i = 0; i < 6; i++) {
      if (fabsf(targetFootZComp[i]) > 0.01f || fabsf(footZComp[i]) > 0.01f) {
        targetFootZComp[i] = 0.0f;
        footZComp[i] += FOOT_LERP_AMT * (targetFootZComp[i] - footZComp[i]);
        postureChanged = true;
      }
    }
  }

  if (postureChanged && !isWalking) {
    for (int i = 0; i < 6; i++) {
      Leg& leg = crabMode ? crabLegs[i] : legs[i];
      moveFootXYZ(leg.id, leg.restX, leg.walkY, leg.restZ, LEVEL_INTERVAL_MS);
    }
    lastLevelUpdate = now;
  }
}

void sittingLegs() {
  stopGait();
  queueClear();
  jFwd = jTurn = jStrafe = 0.0f;
  for (int i = 0; i < 6; i++)
    moveFootXYZ(legs[i].id, legs[i].restX, legs[i].walkY, legs[i].restZ - 3, 2000);
}

void sleepingLegs() {
  stopGait();
  queueClear();
  jFwd = jTurn = jStrafe = 0.0f;
  for (int i = 0; i < 6; i++)
    moveFootXYZ(legs[i].id, legs[i].restX - 10, legs[i].walkY + 16, legs[i].restZ - 3, 2000);
}

void helloLegs() {
  standStill();
  for (int i = 0; i < 3; i++) {
    moveFootXYZ(legs[3].id, legs[3].restX, legs[3].walkY, legs[3].restZ - 9, 500);
    delay(600);
    moveFootXYZ(legs[3].id, legs[3].restX, legs[3].walkY, legs[3].restZ - 3, 500);
    delay(600);
  }
  // Reset PS2 warmup to flush junk frames caused by blocking delay above
  ps2Warm = 0;
}

// ── Log helper ─────────────────────────────────────────────────
void logMove(const char* direction) {
  char buf[64];
  snprintf(buf, sizeof(buf), ">> %s: %s\n", GAIT_TABLE[activeGait].name, direction);
  btLog(buf);
}

void walkForward()  { jFwd =  1.0f;  jTurn =  0.0f;  startGait(true); }
void walkBackward() { jFwd = -1.0f;  jTurn =  0.0f;  startGait(true); }
void turnLeft()     { jFwd =  0.66f; jTurn = -0.33f; startGait(true); }
void turnRight()    { jFwd =  0.66f; jTurn =  0.33f; startGait(true); }
void rotateLeft()   { jFwd =  0.0f;  jTurn = -1.0f;  startGait(true); }
void rotateRight()  { jFwd =  0.0f;  jTurn =  1.0f;  startGait(true); }

void crabForward()  { activeGait = 1; jFwd =  1.0f; jStrafe =  0.0f; jTurn = 0.0f; startGait(true); }
void crabBackward() { activeGait = 1; jFwd = -1.0f; jStrafe =  0.0f; jTurn = 0.0f; startGait(true); }
void crabLeft()     { activeGait = 1; jFwd =  0.0f; jStrafe =  1.0f; jTurn = 0.0f; startGait(true); }
void crabRight()    { activeGait = 1; jFwd =  0.0f; jStrafe = -1.0f; jTurn = 0.0f; startGait(true); }


// ───────────────────────────────────────────────────────────────
//  SECTION 7 — HEIGHT CONTROL
//
//  Height changes only adjust restZ (the body drops/rises).
//  restX stays CONSTANT — the feet don't slide on the floor.
//  The perpTibia flag in moveFootXYZ ensures the tibia always
//  points straight down regardless of the femur angle.
// ───────────────────────────────────────────────────────────────

void adjustHeight(float delta) {
  // delta > 0 means "taller" → bodyZ goes MORE negative (further extension)
  float newZ = bodyZ - delta;
  if (newZ > HEIGHT_LOW || newZ < HEIGHT_HIGH) {
    btLog(delta > 0 ? ">> Max height reached\n" : ">> Min height reached\n");
    return;
  }
  bodyZ = newZ;
  for (int i = 0; i < 6; i++) {
    legs[i].restZ     = bodyZ;
    crabLegs[i].restZ = bodyZ;
  }
  standStill();  // standStill uses perpTibia=true → tibia stays straight down
  char buf[32]; snprintf(buf, sizeof(buf), ">> Height: %.1f cm\n", bodyZ); btLog(buf);
}

void increaseHeight() { adjustHeight(+HEIGHT_STEP); }
void decreaseHeight() { adjustHeight(-HEIGHT_STEP); }


// ───────────────────────────────────────────────────────────────
//  SECTION 8 — BT INPUT PARSER
// ───────────────────────────────────────────────────────────────

String inputBuffer = "";

void handleCommand(char cmd) {
  switch (cmd) {

    case 'F': case 'f':
      lastCmdName = "FORWARD";
      if (crabMode) { logMove("Forward");  crabForward();  }
      else          { logMove("Forward");  walkForward();  }
      break;

    case 'B': case 'b':
      lastCmdName = "BACKWARD";
      if (crabMode) { logMove("Backward"); crabBackward(); }
      else          { logMove("Backward"); walkBackward(); }
      break;

    case 'L': case 'l':
      lastCmdName = "LEFT";
      if (crabMode) { logMove("Strafe Left");  crabLeft();  }
      else          { logMove("Turn Left");    turnLeft();  }
      break;

    case 'R': case 'r':
      lastCmdName = "RIGHT";
      if (crabMode) { logMove("Strafe Right"); crabRight();  }
      else          { logMove("Turn Right");   turnRight();  }
      break;

    case 'W': case 'w':
      lastCmdName = "ROT LEFT";  logMove("Rotate Left");  rotateLeft();  break;

    case 'X': case 'x':
      lastCmdName = "ROT RIGHT"; logMove("Rotate Right"); rotateRight(); break;

    case 'A': case 'a':
      lastCmdName = "SITTING";  btLog(">> Sitting\n");  sittingLegs(); break;

    case 'Z': case 'z':
      lastCmdName = "SLEEPING"; btLog(">> Sleeping\n"); sleepingLegs(); break;

    case 'H': case 'h':
      lastCmdName = "SAY HI";   btLog(">> Hello\n"); helloLegs(); break;

    case 'U': case 'u':
      lastCmdName = "HEIGHT UP"; increaseHeight(); break;

    case 'D': case 'd':
      lastCmdName = "HEIGHT DN"; decreaseHeight(); break;

    case 'S': case 's':
      lastCmdName = "STOP"; btLog(">> standStill\n"); standStill(); break;

    case 'T': case 't':
      lastCmdName = "SENSORS"; 
      footSensorsActive = !footSensorsActive;
      btLog(footSensorsActive ? ">> Foot Sensors ON\n" : ">> Foot Sensors OFF\n");
      break;

    case 'I': case 'i':
      lastCmdName = "IMU TOGGLE"; 
      imuActive = !imuActive;
      btLog(imuActive ? ">> IMU Levelling ON\n" : ">> IMU Levelling OFF\n");
      if (gaitPhase == IDLE) standStill(); // Instantly apply change
      break;

    case 'P': case 'p':
      lastCmdName = "PLOT TOGGLE";
      imuPlotting = !imuPlotting;
      btLog(imuPlotting ? ">> IMU Plotting ON\n" : ">> IMU Plotting OFF\n");
      break;

    case '+':
      if (moveTime > SPEED_MIN) moveTime -= SPEED_STEP;
      { char b[28]; snprintf(b, sizeof(b), ">> Speed: %u ms\n", moveTime); btLog(b); }
      break;

    case '-':
      if (moveTime < SPEED_MAX) moveTime += SPEED_STEP;
      { char b[28]; snprintf(b, sizeof(b), ">> Speed: %u ms\n", moveTime); btLog(b); }
      break;

    case '?':
      btLog("─── SpiderBot V3.6 ───\n");
      btLog("Last cmd : "); btLog(lastCmdName);                  btLog("\n");
      btLog("Gait     : "); btLog(GAIT_TABLE[activeGait].name);  btLog("\n");
      btLog("Speed    : "); btLog(moveTime);     btLog(" ms\n");
      btLog("Height   : "); btLog(bodyZ);        btLog(" cm\n");
      btLog("Battery  : "); btLog(voltageInVolts); btLog(" V\n");
      btLog("Roll     : "); btLog(smoothRoll);      btLog(" deg\n");
      btLog("Pitch    : "); btLog(smoothPitch);     btLog(" deg\n");
      btLog("Levelling: "); btLog((imuReady && imuActive) ? "ON\n" : "OFF\n");
      btLog("Tactile  : "); btLog(footSensorsActive ? "ON\n" : "OFF\n");
      btLog("──────────────────────\n");
      break;

    default: break;
  }
}

void parseInput(char c) {
  if (c == '\n' || c == '\r') {
    if (inputBuffer.length() == 0) return;
    char first = inputBuffer.charAt(0);

    if (first == 'j' || first == 'J') {
      int comma = inputBuffer.indexOf(',');
      if (comma > 1) {
        float rawFwd  = constrain(inputBuffer.substring(1, comma).toFloat() / 100.0f, -1.0f, 1.0f);
        float rawTurn = constrain(inputBuffer.substring(comma + 1).toFloat() / 100.0f, -1.0f, 1.0f);

        float mag = constrain(sqrt(rawFwd*rawFwd + rawTurn*rawTurn), 0.0f, 1.0f);
        moveTime  = (uint16_t)map((long)(mag * 100), 0, 100, SPEED_MAX, SPEED_MIN);
        lastCmdName = "JOYSTICK";

        if (GAIT_TABLE[activeGait].indepXY) {
          jFwd    = rawFwd;
          jStrafe = -rawTurn;
          jTurn   = 0.0f;
        } else {
          activeGait = 0;
          jFwd    = rawFwd;
          jTurn   = rawTurn;
          jStrafe = 0.0f;
        }

        if (fabsf(rawFwd) < 0.05f && fabsf(rawTurn) < 0.05f) standStill();
        else startGait(true);
      }
    }
    else if (first == 'g' || first == 'G') {
      uint8_t idx = (uint8_t)inputBuffer.substring(1).toInt();
      if (idx < GAIT_COUNT) {
        crabMode   = (idx == 1);
        activeGait = idx;
        jFwd = 1.0f; jTurn = 0.0f; jStrafe = 0.0f;
        char buf[56];
        snprintf(buf, sizeof(buf), ">> Gait: %s%s\n",
                 GAIT_TABLE[idx].name,
                 crabMode ? "  (F/B=fwd-back  L/R=strafe)" : "");
        btLog(buf);
      } else {
        btLog(">> Bad gait number. Use g0-g3.\n");
      }
    }
    // New V3.3 BT commands
    else if (inputBuffer.equalsIgnoreCase("imu")) {
      char b[64];
      snprintf(b, sizeof(b), ">> Roll: %.2f  Pitch: %.2f deg\n", imuRoll, imuPitch);
      btLog(b);
    }

    inputBuffer = "";

  } else {
    if (inputBuffer.length() == 0 && c != 'j' && c != 'J' && c != 'g' && c != 'G')
      handleCommand(c);
    else
      inputBuffer += c;
  }
}


// ───────────────────────────────────────────────────────────────
//  SECTION 9 — PS2 CONTROLLER
// ───────────────────────────────────────────────────────────────

// Convert a raw PS2 axis byte (0–255, centre=128) to -1.0…+1.0.
float ps2Axis(byte raw, bool invert) {
  int centred = (int)raw - 128;
  if (abs(centred) < PS2_DEADZONE) return 0.0f;
  float norm = constrain(centred / 127.0f, -1.0f, 1.0f);
  return invert ? -norm : norm;
}

bool ps2Moving = false;
const uint8_t PS2_WARMUP_FRAMES = 10;

const unsigned long PS2_POLL_MS = 50;
unsigned long lastPS2Read = 0;

void readPS2() {
  unsigned long now = millis();
  if (now - lastPS2Read < PS2_POLL_MS) return;
  lastPS2Read = now;

  ps2x.read_gamepad(false, 0);

  // ── Connection check (Non-blocking) ──────────────────────────
  byte rawLY = ps2x.Analog(PSS_LY);
  byte rawRY = ps2x.Analog(PSS_RY);
  if (rawLY == 255 && rawRY == 255) {
    ps2Warm = 0; // Keep warmup at 0 until a real controller connects
    return;
  }

  if (ps2x.ButtonPressed(PSB_R3)) {
    isBotOn = !isBotOn;
    if (isBotOn) {
      btLog(">> BOT IS NOW ON (Awake)\n");
      standStill();
    } else {
      btLog(">> BOT IS NOW OFF (Sleeping)\n");
      sleepingLegs();
    }
  }

  if (!isBotOn) return;

  if (ps2Warm < PS2_WARMUP_FRAMES) { ps2Warm++; return; }

  float ly = ps2Axis(ps2x.Analog(PSS_LY), true);    // up (+Y) = positive jFwd
  float lx = ps2Axis(ps2x.Analog(PSS_LX), false);   // right   = positive jTurn
  float rx = ps2Axis(ps2x.Analog(PSS_RX), false);   // right   = positive jStrafe (crab / diagonal)

  bool stickActive = (fabsf(ly) > 0.0f || fabsf(lx) > 0.0f || fabsf(rx) > 0.0f);

  bool dpadActive = false;

  if (ps2x.Button(PSB_PAD_UP)) {
    dpadActive = true;
    if (!ps2Moving) { logMove("Forward"); lastCmdName = "FORWARD"; }
    jFwd = 1.0f;  jTurn = 0.0f;
    if (crabMode) { activeGait = 1; jStrafe = 0.0f; }
    if (gaitPhase == IDLE) startGait(true);
  }
  else if (ps2x.Button(PSB_PAD_DOWN)) {
    dpadActive = true;
    if (!ps2Moving) { logMove("Backward"); lastCmdName = "BACKWARD"; }
    jFwd = -1.0f; jTurn = 0.0f;
    if (crabMode) { activeGait = 1; jStrafe = 0.0f; }
    if (gaitPhase == IDLE) startGait(true);
  }
  else if (ps2x.Button(PSB_PAD_LEFT)) {
    dpadActive = true;
    if (!ps2Moving) {
      lastCmdName = "LEFT";
      if (crabMode) logMove("Strafe Left");
      else          logMove("Turn Left");
    }
    if (crabMode) { activeGait = 1; jFwd = 0.0f; jStrafe = 1.0f; jTurn = 0.0f; }
    else          { jFwd = 0.66f; jTurn = -0.33f; }
    if (gaitPhase == IDLE) startGait(true);
  }
  else if (ps2x.Button(PSB_PAD_RIGHT)) {
    dpadActive = true;
    if (!ps2Moving) {
      lastCmdName = "RIGHT";
      if (crabMode) logMove("Strafe Right");
      else          logMove("Turn Right");
    }
    if (crabMode) { activeGait = 1; jFwd = 0.0f; jStrafe = -1.0f; jTurn = 0.0f; }
    else          { jFwd = 0.66f; jTurn = 0.33f; }
    if (gaitPhase == IDLE) startGait(true);
  }
  else if (ps2x.Button(PSB_CIRCLE)) {
    dpadActive = true;
    if (!ps2Moving) { logMove("Rotate Left"); lastCmdName = "ROT LEFT"; }
    jFwd = 0.0f; jTurn = -1.0f;
    if (gaitPhase == IDLE) startGait(true);
  }
  else if (ps2x.Button(PSB_SQUARE)) {
    dpadActive = true;
    if (!ps2Moving) { logMove("Rotate Right"); lastCmdName = "ROT RIGHT"; }
    jFwd = 0.0f; jTurn = 1.0f;
    if (gaitPhase == IDLE) startGait(true);
  }

  if (stickActive && !dpadActive) {
    float mag = constrain(sqrt(ly*ly + lx*lx + rx*rx), 0.0f, 1.0f);
    moveTime  = (uint16_t)map((long)(mag * 100), 0, 100, SPEED_MAX, SPEED_MIN);

    if (GAIT_TABLE[activeGait].indepXY) {
      jFwd = ly;  jStrafe = rx;  jTurn = 0.0f;
    } else {
      jFwd = ly;  jTurn = lx;   jStrafe = 0.0f;
    }

    if (gaitPhase == IDLE) startGait(true);
  }

  bool anyMovement = dpadActive || stickActive;
  if (!anyMovement && ps2Moving) {
    btLog(">> PS2: Stop\n");
    standStill();
  }
  ps2Moving = anyMovement;

  // ── Non-movement one-shot buttons ──────────────────────────
  if (ps2x.ButtonPressed(PSB_TRIANGLE)) { btLog(">> PS2: Height Up\n");   increaseHeight(); }
  if (ps2x.ButtonPressed(PSB_CROSS))    { btLog(">> PS2: Height Down\n"); decreaseHeight(); }

  if (ps2x.ButtonPressed(PSB_L1)) {
    btLog(">> PS2: Sitting\n"); lastCmdName = "SITTING"; sittingLegs();
  }
  if (ps2x.ButtonPressed(PSB_R1)) {
    btLog(">> PS2: Sleeping\n"); lastCmdName = "SLEEPING"; sleepingLegs();
  }

  if (ps2x.ButtonPressed(PSB_START)) {
    btLog(">> PS2: Stand Still\n"); lastCmdName = "STOP"; standStill();
  }

  if (ps2x.ButtonPressed(PSB_SELECT)) {
    uint8_t next = (activeGait + 1) % GAIT_COUNT;
    crabMode   = (next == 1);
    activeGait = next;
    jFwd = 0.0f; jTurn = 0.0f; jStrafe = 0.0f;
    char buf[48];
    snprintf(buf, sizeof(buf), ">> PS2: Gait → %s\n", GAIT_TABLE[activeGait].name);
    btLog(buf);
  }

  if (ps2x.ButtonPressed(PSB_L3)) {
    crabMode   = !crabMode;
    activeGait = crabMode ? 1 : 0;
    jFwd = 0.0f; jTurn = 0.0f; jStrafe = 0.0f;
    btLog(crabMode ? ">> PS2: Crab ON\n" : ">> PS2: Crab OFF\n");
  }

  if (ps2x.ButtonPressed(PSB_L2)) {
    if (moveTime < SPEED_MAX) moveTime += SPEED_STEP;
    char b[32]; snprintf(b, sizeof(b), ">> PS2: Speed %u ms\n", moveTime); btLog(b);
  }
  if (ps2x.ButtonPressed(PSB_R2)) {
    if (moveTime > SPEED_MIN) moveTime -= SPEED_STEP;
    char b[32]; snprintf(b, sizeof(b), ">> PS2: Speed %u ms\n", moveTime); btLog(b);
  }
}


// ───────────────────────────────────────────────────────────────
//  BT CALLBACK
// ───────────────────────────────────────────────────────────────
void btCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t* param) {
  if (event == ESP_SPP_SRV_OPEN_EVT) {
    btConnected = true;
    btLog("SpiderBot V3.3 Ready!\n");
    btLog("PS2 controller is primary input.\n");
    btLog("BT debug: F/B/L/R/W/X/A/Z/H/U/D/S/+/-/?/g0-g3/jFWD,TURN/imu\n");
  }
  if (event == ESP_SPP_CLOSE_EVT) {
    btConnected = false;
    standStill();
    Serial.println("BT: Disconnected");
  }
}


// ───────────────────────────────────────────────────────────────
//  OTA SETUP
// ───────────────────────────────────────────────────────────────
void setupOTA() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi for OTA");

  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 5000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWi-Fi Connected! IP: ");
    Serial.println(WiFi.localIP());

    ArduinoOTA.setHostname("spiderbot");

    ArduinoOTA.onStart([]() {
      String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
      Serial.println("\nStart OTA updating " + type);
      standStill();
    });
    ArduinoOTA.onEnd([]()    { Serial.println("\nEnd OTA"); });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if      (error == OTA_AUTH_ERROR)    Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR)   Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR)     Serial.println("End Failed");
    });

    ArduinoOTA.begin();
    Serial.println("OTA Ready.");
  } else {
    Serial.println("\nWi-Fi timeout. Turning off Wi-Fi to save battery.");
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
  }
}


// ───────────────────────────────────────────────────────────────
//  SETUP
// ───────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  LSC.begin(9600, SERIAL_8N1, 16, 17);

  SerialBT.register_callback(btCallback);
  delay(1000);
  SerialBT.begin("SpiderBot");
  Serial.println("SpiderBot V3.3 — Bluetooth: SpiderBot");

  // ── I2C + MPU-6050 ──────────────────────────────────────────
  Wire.begin(21, 22);  // SDA=21, SCL=22 (ESP32 default)
  if (imuInit()) {
    imuReady = true;
    Serial.println("MPU-6050 OK");
  } else {
    Serial.println("MPU-6050 NOT FOUND — levelling disabled");
  }

  // ── Foot Sensors ────────────────────────────────────────────
  for (int i = 0; i < 6; i++) {
    // 34 & 35 are input-only and have no internal pull-ups.
    // ESP32 ignores INPUT_PULLUP for ports 34-39, but we set it anyway 
    // for the others. Physical external pull-ups assumed present.
    pinMode(FOOT_PINS[i], INPUT_PULLUP);
  }

  // ── PS2 ─────────────────────────────────────────────────────
  pinMode(PS2_DAT, INPUT_PULLUP);

  // Configure SPI bus and detect controller. Error codes: 0=OK, 1=no controller,
  // 2=not DualShock, 3=no rumble. Connection is re-checked non-blocking in readPS2().
  int ps2error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, false, false);
  if (ps2error == 0) {
    Serial.println("PS2 SPI OK.");
  } else {
    Serial.printf("PS2 SPI error %d — will retry in readPS2()\n", ps2error);
  }

  delay(500);
  setupOTA();
  standStill();
}


void plotIMU() {
  if (!imuPlotting) return;
  static unsigned long lastPlot = 0;
  if (millis() - lastPlot < 30) return; // ~33Hz update for smooth plotting
  lastPlot = millis();

  // Serial Plotter format: Name:Value (Limited to 8 variables for legacy plotter support)
  Serial.print("roll:");   Serial.print(smoothRoll);  Serial.print("\t");
  Serial.print("pitch:");  Serial.print(smoothPitch); Serial.print("\t");
  
  // Foot sensors (Left: 1,2,3 | Right: -1,-2,-3)
  Serial.print("FL:"); Serial.print(footOnGround[0] ? 1 : 0);  Serial.print("\t");
  Serial.print("ML:"); Serial.print(footOnGround[1] ? 2 : 0);  Serial.print("\t");
  Serial.print("RL:"); Serial.print(footOnGround[2] ? 3 : 0);  Serial.print("\t");
  Serial.print("FR:"); Serial.print(footOnGround[3] ? -1 : 0); Serial.print("\t");
  Serial.print("MR:"); Serial.print(footOnGround[4] ? -2 : 0); Serial.print("\t");
  Serial.print("RR:"); Serial.println(footOnGround[5] ? -3 : 0);
}

// ───────────────────────────────────────────────────────────────
//  LOOP
// ───────────────────────────────────────────────────────────────
void loop() {
  // 0. OTA (must be called every loop to service download chunks)
  ArduinoOTA.handle();

  // 1. Read BT / Serial debug input
  while (SerialBT.available()) parseInput((char)SerialBT.read());
  while (Serial.available())   parseInput((char)Serial.read());

  // 2. Read PS2 (primary physical controller, rate-limited to PS2_POLL_MS)
  readPS2();

  // 2b. Always update foot sensors for plotter/debug (regardless of tactile mode)
  updateFootSensors();

  // 3. Update IMU (non-blocking complementary filter, 50 Hz cadence)
  if (imuReady) {
    updateIMU();

    // Fall detection — auto-sleep if robot tips beyond IMU_FALL_DEG
    if (isBotOn && checkFall()) {
      isBotOn = false;
      sleepingLegs();
    }
  }

  // 4. Advance gait state machine (non-blocking millis() timer)
  gaitTick();

  // 4b. Continuous standstill levelling (only fires when IDLE + IMU ready)
  updateLevelling();

  // 5. Poll battery voltage every 5 s via LSC-32 feedback packet
  controller.receiveHandle();
  if (millis() - lastVoltageRequest > 5000) {
    controller.getBatteryVoltage();
    lastVoltageRequest = millis();
  }
  static uint16_t lastVoltage = 0;
  if (controller.batteryVoltage != lastVoltage) {
    voltageInVolts = controller.batteryVoltage / 1000.0f;
    lastVoltage    = controller.batteryVoltage;
  }
  // 6. Serial Plotter for IMU debugging
  plotIMU();
}
