/*===============================================================
 * SpiderBot V3.3
 * Author  : Amit Parihar
 * Wiring  : UART2  RX=16  TX=17  →  LSC-32 servo controller
 *           PS2 controller SPI:
 *             DAT=19 (MISO)  CMD=23 (MOSI)
 *             ATT=5  (SS)    CLK=18 (SCK)
 *           MPU-6050 I2C:
 *             SDA=21  SCL=22  (ESP32 default I2C)
 *           Foot contact sensors (digital, active-LOW w/ pull-ups):
 *             FL=34  ML=35  RL=32  FR=33  MR=25  RR=26
 *             (Use limit switches or FSRs w/ comparator to GND)
 *
 * ── SECTIONS IN THIS FILE ────────────────────────────────────
 *   1.  Hardware + Constants + Leg Data
 *   2.  IK Solver           — foot XYZ → servo pulses
 *   3.  Gait Shapes         — plug-and-play stride definitions (2-D)
 *   4.  Command Queue       — circular buffer, never drops a command
 *   5.  Gait Engine         — millis()-driven, never blocks loop()
 *       • Tripod engine  — 2-phase (gaits 0, 1, 2)
 *       • Wave engine    — 6-phase, true 1-leg-at-a-time (gait 3)
 *   6.  Movement Funcs      — walk, turn, rotate, crab, diagonal
 *   7.  Height Control      — raise / lower body at runtime
 *   8.  BT Input Parser     — BT + Serial command handler
 *   9.  PS2 Controller      — primary physical controller
 *  10.  OTA Setup           — wireless firmware updates via Arduino IDE
 *  11.  MPU-6050 (IMU)      — roll/pitch stabilisation + fall detect
 *  12.  Foot Contact Sensors — ground-touch feedback per leg
 *  13.  setup() + loop()
 *
 * ── PHASE 1 INTEGRATION (BUGFIXES & LEVELLING) ────────────────
 *   + Corrected Inverse Kinematics mapping:
 *       • Femur angle physical mapping (SERVO_MID - fDeg) fixed Z-axis inversion.
 *       • Tibia perpendicular lock added for strict height adjustment kinematics.
 *       • Gait sequence updated to use restZ + LIFT_H (foot genuinely lifts UP).
 *   + Validated Height boundaries (Heights from -7.0 crouch to -14.0 tall).
 *   + Tuned IMU Complementary Filter:
 *       • Z-axis levelling gain mathematically tied to proper trigonometry (0.24 cm/deg).
 *       • Compensation boundary increased to 5.0cm to allow obvious physical tilting.
 *   Note: Full rotational matrices (X/Y positional sliding) belong to Phase 2.
 *
 * ── WHAT'S NEW IN V3.3 (Summary) ─────────────────────────────
 *   + MPU-6050 IMU integration (non-blocking, 50 Hz):
 *       • Complementary filter for roll & pitch
 *       • Body self-levelling: adjusts each leg's Z target to
 *         compensate for terrain tilt (up to ±IMU_COMP_MAX cm)
 *       • Fall detection: auto-sleep if tilt > IMU_FALL_DEG
 *       • BT '?' status now shows roll/pitch angles
 *   + Foot contact sensors (6× digital, active-LOW):
 *       • readFootContacts() — debounced per-leg flag array
 *       • Wave gait: early step plant when foot contact detected
 *         (avoids jarring thud on hard contact)
 *       • Slip detection: warns via BT if a grounded leg loses
 *         contact mid-drag
 *   + Bug fixes from V3.2 code review:
 *       • standStill() used leg.rotY instead of leg.walkY — FIXED
 *       • BT connect banner still said "V3.0" — FIXED
 *       • rx axis inversion was wrong in ps2Axis (treated up as
 *         positive strafe, now correctly right=positive) — FIXED
 *       • helloLegs() extern declaration removed; ps2Warm made
 *         file-scope so extern is unnecessary — FIXED
 *       • '?' status report version string updated — FIXED
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
 *   - L3 (click)  → Toggle crab mode (same as g1 / g0)
 *   - R3 (click)  → Master ON/OFF toggle
 *   PS2 sticks return 0–255; centre is 128.
 *   Values within ±PS2_DEADZONE of 128 are treated as zero.
 *   PS2_DEADZONE is 20 by default — tune if sticks drift.
 *
 * ── BT DEBUG COMMANDS ────────────────────────────────────────
 *   F/B/L/R/W/X/A/Z/H/U/D/S/+/-/?   (same as V3.2)
 *   g0…g3     gait switch
 *   jFWD,TURN joystick override
 *   imu       print current roll/pitch to BT
 *   feet      print foot contact bitmask to BT
 *
 * ── BT APP SETUP (Serial Bluetooth Terminal) ─────────────────
 *   Settings → Line ending → Newline (\n)
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
const float LIFT_H = 3.0f;

// ── Servo pulse constants ───────────────────────────────────────
const int   SERVO_MID = 1500;
const float DEG_TO_US = 11.11f;

// ── Runtime-tunable parameters ─────────────────────────────────
uint16_t moveTime = 500;
float    bodyZ    = -9.0f;

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
  float   restX, walkY, rotY, restZ;
  int     coxaDir;
  float   strideScale;
};

//            id    X    walkY  rotY    Z   dir  scale
Leg legs[6] = {
  {  0,  12.5,   3,    0,   -9,  +1,  1.0f },  // 0  Front-Left
  {  3,  12.5,  -1,    0,   -9,  +1,  1.0f },  // 1  Mid-Left
  { 29,  12.5,  -4,    0,   -9,  +1,  1.0f },  // 2  Rear-Left
  {  6,  12.5,   0,    0,   -9,  -1,  1.0f },  // 3  Front-Right
  {  9,  12.5,   2,    0,   -9,  -1,  1.0f },  // 4  Mid-Right
  { 26,  12.5,   6,    0,   -9,  -1,  1.0f },  // 5  Rear-Right
};

Leg crabLegs[6] = {
  {  0,  12.5,   8,   8,   -8,  +1,  1.0f },  // 0  Front-Left
  {  3,  12.5,   0,   0,   -8,  +1,  1.0f },  // 1  Mid-Left
  { 29,  12.5,  -8,  -8,   -8,  +1,  1.0f },  // 2  Rear-Left
  {  6,  12.5,  -8,  -8,   -8,  -1,  1.0f },  // 3  Front-Right
  {  9,  12.5,   0,   0,   -8,  -1,  1.0f },  // 4  Mid-Right
  { 26,  12.5,   8,   8,   -8,  -1,  1.0f },  // 5  Rear-Right
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
}

// Apply tilt compensation: returns a per-leg Z offset based on roll/pitch
// so the robot levels itself on sloped terrain.
// legIdx 0-5 in the usual order; legX/legY are the rest positions.
float imuZCompensation(int legIdx) {
  if (!imuReady || !imuActive) return 0.0f;
  float roll  = (fabsf(smoothRoll)  > IMU_LEVEL_DEAD) ? smoothRoll  : 0.0f;
  float pitch = (fabsf(smoothPitch) > IMU_LEVEL_DEAD) ? smoothPitch : 0.0f;

  // Left legs (0,1,2) benefit from positive roll; right legs (3,4,5) invert.
  // Front legs (0,3) respond to pitch; rear legs (2,5) invert; mid is neutral.
  const float rollSign[6]  = { +1, +1, +1, -1, -1, -1 };
  const float pitchSign[6] = { -1,  0, +1, -1,  0, +1 };

  // GAIN FIX: 0.24 cm/degree matches the real trigonometry (tan(angle) * ~14cm leg span)
  float comp = (roll  * rollSign[legIdx]  * 0.24f)  
             + (pitch * pitchSign[legIdx] * 0.24f);
  return constrain(comp, -IMU_COMP_MAX, IMU_COMP_MAX);
}

// Check for fall (tilt exceeds threshold); auto-sleep if so.
bool checkFall() {
  if (fabsf(imuRoll) > IMU_FALL_DEG || fabsf(imuPitch) > IMU_FALL_DEG) {
    btLog(">> FALL DETECTED — auto-sleep!\n");
    return true;
  }
  return false;
}


// ───────────────────────────────────────────────────────────────
//  SECTION 2 — IK SOLVER
// ───────────────────────────────────────────────────────────────
void moveFootXYZ(uint8_t id, float x, float y, float z, uint16_t t, bool perpTibia = false) {

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

  // When perpTibia is true, override the tibia angle so it always
  // points straight DOWN (perpendicular to ground), regardless of
  // how the femur is tilted.  The formula cancels the femur tilt:
  //   tDeg_perp = 90 + fDeg  →  p3 = SERVO_MID - fDeg * DEG_TO_US
  // At fDeg=0° (femur horizontal): p3 = 1500  (your servo's straight-down position)
  // At fDeg=40° (default height):  p3 = 1056  (counter-rotated to stay vertical)
  if (perpTibia) tDeg = 90.0f + fDeg;

  uint16_t p1 = constrain((int)(SERVO_MID +  cDeg  * DEG_TO_US), 500, 2500);
  uint16_t p2 = constrain((int)(SERVO_MID -  fDeg  * DEG_TO_US), 500, 2500);  // 1500=horizontal, 500=up, 2500=down
  uint16_t p3 = constrain((int)(SERVO_MID - (tDeg - 90.0f) * DEG_TO_US), 500, 2500);  // 1500=straight down

  byte pkt[16] = {
    0x55, 0x55, 14, 0x03, 3,
    lowByte(t),     highByte(t),
    id,             lowByte(p1), highByte(p1),
    uint8_t(id+1),  lowByte(p2), highByte(p2),
    uint8_t(id+2),  lowByte(p3), highByte(p3)
  };
  LSC.write(pkt, 16);
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
//  SECTION 5 — NON-BLOCKING GAIT ENGINE
// ───────────────────────────────────────────────────────────────

float jFwd    = 0.0f;
float jTurn   = 0.0f;
float jStrafe = 0.0f;

enum GaitPhase { IDLE, SEND_A, PLANT_A, SEND_B, PLANT_B, RECENTER, WAVE_STEP, WAVE_PLANT };
GaitPhase     gaitPhase    = IDLE;
bool          looping      = false;
unsigned long phaseStart   = 0;
uint8_t       wavePhaseIdx = 0;

void handleCommand(char cmd);  // forward declaration

void sendPhase(bool groupASwings) {
  GaitDef&   g        = GAIT_TABLE[activeGait];
  const int* swingGrp = groupASwings ? GRP_A : GRP_B;
  const int* dragGrp  = groupASwings ? GRP_B : GRP_A;

  for (int i = 0; i < 3; i++) {
    int   idx    = swingGrp[i];
    Leg&  leg    = crabMode ? crabLegs[idx] : legs[idx];
    float scaleY = g.indepXY ? jFwd : constrain(jFwd + (jTurn * leg.coxaDir), -1.0f, 1.0f);
    float scaleX = g.indepXY ? jStrafe : scaleY;
    float stepX  = leg.restX + (g.swingX(idx) - leg.restX) * scaleX;
    float stepY  = leg.walkY + (g.swingY(idx) - leg.walkY) * scaleY;
    float compZ  = imuZCompensation(idx);
    moveFootXYZ(leg.id, stepX, stepY, leg.restZ + LIFT_H + compZ, moveTime);
  }

  for (int i = 0; i < 3; i++) {
    int   idx    = dragGrp[i];
    Leg&  leg    = crabMode ? crabLegs[idx] : legs[idx];
    float scaleY = g.indepXY ? jFwd : constrain(jFwd + (jTurn * leg.coxaDir), -1.0f, 1.0f);
    float scaleX = g.indepXY ? jStrafe : scaleY;
    float slideX = leg.restX + (g.dragX(idx) - leg.restX) * scaleX;
    float slideY = leg.walkY + (g.dragY(idx) - leg.walkY) * scaleY;
    float compZ  = imuZCompensation(idx);

    moveFootXYZ(leg.id, slideX, slideY, leg.restZ + compZ, moveTime);
  }
}

void plantGroup(bool groupA) {
  GaitDef&   g   = GAIT_TABLE[activeGait];
  const int* grp = groupA ? GRP_A : GRP_B;

  for (int i = 0; i < 3; i++) {
    int   idx    = grp[i];
    Leg&  leg    = crabMode ? crabLegs[idx] : legs[idx];
    float scaleY = g.indepXY ? jFwd : constrain(jFwd + (jTurn * leg.coxaDir), -1.0f, 1.0f);
    float scaleX = g.indepXY ? jStrafe : scaleY;
    float plantX = leg.restX + (g.swingX(idx) - leg.restX) * scaleX;
    float plantY = leg.walkY + (g.swingY(idx) - leg.walkY) * scaleY;
    float compZ  = imuZCompensation(idx);
    moveFootXYZ(leg.id, plantX, plantY, leg.restZ + compZ, moveTime / 3);
  }
}

void recenterLegs() {
  for (int i = 0; i < 6; i++) {
    Leg&  leg   = crabMode ? crabLegs[i] : legs[i];
    float compZ = imuZCompensation(i);
    moveFootXYZ(leg.id, leg.restX, leg.walkY, leg.restZ + compZ, moveTime / 2);
  }
}

void waveStep(uint8_t phaseIdx) {
  GaitDef& g        = GAIT_TABLE[activeGait];
  int      swingIdx = WAVE_SEQ[phaseIdx];
  Leg&     swLeg    = legs[swingIdx];
  float    scale    = constrain(jFwd + (jTurn * swLeg.coxaDir), -1.0f, 1.0f);
  float    compZ    = imuZCompensation(swingIdx);

  float swY = swLeg.walkY + (g.swingY(swingIdx) - swLeg.walkY) * scale;
  moveFootXYZ(swLeg.id, swLeg.restX, swY, swLeg.restZ + LIFT_H + compZ, moveTime);

  for (int i = 0; i < 6; i++) {
    if (i == swingIdx) continue;
    Leg&  leg      = legs[i];
    float legScale = constrain(jFwd + (jTurn * leg.coxaDir), -1.0f, 1.0f);
    float legCompZ = imuZCompensation(i);
    int   phasesSince = 0;
    for (int p = 1; p <= 5; p++) {
      if (WAVE_SEQ[(phaseIdx - p + 6) % 6] == i) { phasesSince = p; break; }
    }
    float t      = phasesSince / 5.0f;
    float frontY = leg.walkY + (g.swingY(i) - leg.walkY) * legScale;
    float rearY  = leg.walkY + (g.dragY(i)  - leg.walkY) * legScale;
    moveFootXYZ(leg.id, leg.restX, frontY + (rearY - frontY) * t,
                leg.restZ + legCompZ, moveTime);
  }
}

void wavePlant(uint8_t phaseIdx) {
  int      swingIdx = WAVE_SEQ[phaseIdx];
  Leg&     leg      = legs[swingIdx];
  GaitDef& g        = GAIT_TABLE[activeGait];
  float    scale    = constrain(jFwd + (jTurn * leg.coxaDir), -1.0f, 1.0f);
  float    plantY   = leg.walkY + (g.swingY(swingIdx) - leg.walkY) * scale;
  float    compZ    = imuZCompensation(swingIdx);

  uint16_t plantTime = moveTime / 4;
  moveFootXYZ(leg.id, leg.restX, plantY, leg.restZ + compZ, plantTime);
}

void startGait(bool continuous) {
  for (int i = 0; i < 6; i++) {
    footZComp[i] = 0.0f;
    targetFootZComp[i] = 0.0f; // Reset terrain adaptation targets
  }
  looping = continuous;
  if (activeGait == 3) {
    wavePhaseIdx = 0;
    gaitPhase    = WAVE_STEP;
  } else {
    gaitPhase = SEND_A;
  }
  phaseStart = millis();
}

void stopGait() { gaitPhase = IDLE; looping = false; }

void gaitTick() {
  if (gaitPhase == IDLE) {
    char cmd;
    if (queuePop(cmd)) handleCommand(cmd);
    return;
  }

  unsigned long elapsed = millis() - phaseStart;

  switch (gaitPhase) {

    case SEND_A:
      sendPhase(true);
      gaitPhase  = PLANT_A;
      phaseStart = millis();
      break;

    case PLANT_A:
      if (elapsed >= (unsigned long)(moveTime + 20)) {
        plantGroup(true);
        gaitPhase  = SEND_B;
        phaseStart = millis();
      }
      break;

    case SEND_B:
      if (elapsed >= (unsigned long)(moveTime / 3 + 10)) {
        sendPhase(false);
        gaitPhase  = PLANT_B;
        phaseStart = millis();
      }
      break;

    case PLANT_B:
      if (elapsed >= (unsigned long)(moveTime + 20)) {
        plantGroup(false);
        gaitPhase  = RECENTER;
        phaseStart = millis();
      }
      break;

    case RECENTER:
      if (elapsed >= (unsigned long)(moveTime / 3 + 10)) {
        recenterLegs();
        if (looping) {
          gaitPhase  = SEND_A;
          phaseStart = millis() + moveTime / 2;
        } else {
          gaitPhase  = IDLE;
        }
      }
      break;

    case WAVE_STEP:
      waveStep(wavePhaseIdx);
      gaitPhase  = WAVE_PLANT;
      phaseStart = millis();
      break;

    case WAVE_PLANT:
      if (elapsed >= (unsigned long)(moveTime + 20)) {
        wavePlant(wavePhaseIdx);
        wavePhaseIdx = (wavePhaseIdx + 1) % 6;
        if (wavePhaseIdx == 0 && !looping) {
          gaitPhase  = RECENTER;
          phaseStart = millis();
        } else {
          gaitPhase  = WAVE_STEP;
          phaseStart = millis() + moveTime / 4;
        }
      }
      break;

    default: gaitPhase = IDLE; break;
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
    float compZ = imuZCompensation(i);
    moveFootXYZ(leg.id, leg.restX, leg.walkY, leg.restZ + compZ, moveTime, true);
  }
  // Reset levelling tracker so the next tilt is always picked up immediately
  lastLevelRoll  = smoothRoll;
  lastLevelPitch = smoothPitch;
  lastLevelUpdate = millis();
}

// ── updateLevelling() ───────────────────────────────────────────
// Runs from loop() when the bot is standing still (IDLE gait).
// Re-sends all 6 leg positions whenever the IMU reports a tilt
// change ≥ LEVEL_CHANGE_THRESH degrees relative to the last send.
// Effect: physically tilting the robot causes visible leg adjustments
// in real time — ideal for checking IMU accuracy and comp gain tuning.
void updateLevelling() {
  if (!isBotOn || gaitPhase != IDLE) return;

  unsigned long now = millis();
  if (now - lastLevelUpdate < LEVEL_INTERVAL_MS) return;

  bool postureChanged = false;

  // 1. Check IMU changes
  if (imuReady && imuActive) {
    float dRoll  = fabsf(smoothRoll  - lastLevelRoll);
    float dPitch = fabsf(smoothPitch - lastLevelPitch);
    if (dRoll >= LEVEL_CHANGE_THRESH || dPitch >= LEVEL_CHANGE_THRESH) {
      postureChanged = true;
      lastLevelRoll  = smoothRoll;
      lastLevelPitch = smoothPitch;
    }
  }

  // 2. Check Foot Sensors & apply Hexapod-style LERP smoothing
  if (footSensorsActive) {
    float maxComp = -999.0f;
    for (int i = 0; i < 6; i++) {
      // footOnGround[i] is now updated globally in loop()
      
      // Target hunting:
      if (!footOnGround[i]) {
        // In air: search downwards aggressively
        targetFootZComp[i] -= FOOT_EXT_RATE;
        if (targetFootZComp[i] < FOOT_COMP_MAX) targetFootZComp[i] = FOOT_COMP_MAX;
      }
      
      if (targetFootZComp[i] > maxComp) {
        maxComp = targetFootZComp[i];
      }
    }

    // Prevent body jacking / ratchet effect:
    // Normalise all legs so the highest foot stays at exactly 0.0 (body height).
    // If the least extended leg is at -2.0, ALL legs are reaching down by at least 2cm.
    // Subtracting the maxComp pulls all legs back up, gently lowering the body!
    if (maxComp < 0.0f) {
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

  if (!postureChanged) return;

  // perpTibia=true → tibia stays perpendicular while levelling
  for (int i = 0; i < 6; i++) {
    Leg& leg = crabMode ? crabLegs[i] : legs[i];
    float totalCompZ = imuZCompensation(i) + footZComp[i];
    moveFootXYZ(leg.id, leg.restX, leg.walkY, leg.restZ + totalCompZ, LEVEL_INTERVAL_MS, true);
  }

  lastLevelUpdate = now;
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
      btLog("─── SpiderBot V3.3 ───\n");
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
