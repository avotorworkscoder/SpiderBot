
/*===============================================================
 * SpiderBot V4.1
 * Author  : Amit Parihar
 * Wiring  : UART2  RX=16  TX=17  →  LSC-32 servo controller
 *           PS2 controller SPI:
 *             DAT=19 (MISO)  CMD=23 (MOSI)
 *             ATT=5  (SS)    CLK=18 (SCK)
 *
 * ── WHAT'S IN THIS FILE ──────────────────────────────────────
 *   1. Hardware + Constants + Leg Data
 *   2. IK Solver       — foot XYZ → servo pulses
 *   3. Gait Shapes     — plug-and-play stride definitions (2-D)
 *   4. Command Queue   — circular buffer, never drops a command
 *   5. Gait Engine     — millis()-driven, never blocks loop()
 *       • Tripod engine  — 2-phase (gaits 0, 1, 2)
 *       • Wave engine    — 6-phase, true 1-leg-at-a-time (gait 3)
 *   6. Movement Funcs  — walk, turn, rotate, crab, diagonal
 *   7. Height Control  — raise / lower body at runtime
 *   8. BT Input Parser — BT + Serial command handler (debug)
 *   9. PS2 Controller  — primary physical controller
 *  10. OTA Setup       — wireless firmware updates
 *  11. setup() + loop()
 *
 * ── WHAT'S NEW IN V4.1 ───────────────────────────────────────
 *   - Added CoppeliaSim telemetry bridge via Serial to stream
 *     raw joint angles for the digital twin simulation.
 *   - Added anglesDirty flag to optimize telemetry stream.
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
 * ── BT DEBUG COMMANDS (still work in V3) ─────────────────────
 *   F/B/L/R/W/X/A/Z/U/D/S/+/-/?   (same as V2.8)
 *   g0…g3     gait switch
 *   jFWD,TURN joystick override
 *
 * ── BT APP SETUP (Serial Bluetooth Terminal) ─────────────────
 *   Settings → Line ending → Newline (\n)
 *===============================================================*/

#include <Arduino.h>
#include <math.h>
#include "BluetoothSerial.h"
#include <LobotServoController.h>
#include <PS2X_lib.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
  #error "Bluetooth not enabled — Arduino IDE: Tools → ESP32 Bluetooth"
#endif

// ── OTA Constants ──────────────────────────────────────────────
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

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

// ── PS2 controller ────────────────────────────────────────────
PS2X ps2x;

#define PS2_DAT  19   // MISO
#define PS2_CMD  23   // MOSI
#define PS2_SEL   5   // SS
#define PS2_CLK  18   // SCK

// Stick raw values are 0–255; centre = 128.
// Anything within ±PS2_DEADZONE counts as zero.
const int PS2_DEADZONE = 40;

// ── Leg geometry (cm) ─────────────────────────────────────────
const float COXA   = 6.0f;
const float FEMUR  = 8.5f;
const float TIBIA  = 14.5f;
const float LIFT_H = 3.0f;

// ── Servo pulse constants ──────────────────────────────────────
const int   SERVO_MID = 1500;
const float DEG_TO_US = 11.11f;

// ── Runtime-tunable parameters ────────────────────────────────
uint16_t moveTime = 500;
float    bodyZ    = -9.0f;

const uint16_t SPEED_MIN  = 150;
const uint16_t SPEED_MAX  = 1000;
const uint16_t SPEED_STEP = 50;

const float HEIGHT_STEP = 1.0f;
const float HEIGHT_MIN  = -12.0f;
const float HEIGHT_MAX  = -7.0f;

// ── Status ─────────────────────────────────────────────────────
bool          btConnected        = false;
float         voltageInVolts     = 0.0f;
unsigned long lastVoltageRequest = 0;
const char*   lastCmdName        = "NONE";
bool          crabMode           = false;
bool          isBotOn            = true;   // Main ON/OFF state

// ── Logging helpers ────────────────────────────────────────────
// NOTE: btLog deliberately does NOT write to Serial.
// Serial is reserved exclusively for the CoppeliaSim telemetry stream
// (angle packets). Mixing debug text into that port caused corrupted
// packets in the Python bridge.
void btLog(const char* msg) { SerialBT.print(msg); }
void btLog(int          num) { SerialBT.print(num); }
void btLog(float        num) { SerialBT.print(num, 2); }

// ── Leg data structure ─────────────────────────────────────────
struct Leg {
  uint8_t id;
  float   restX, walkY, rotY, restZ;
  int     coxaDir;
  float   strideScale;
};

// Normal walking stance
//            id    X  walkY  rotY    Z   dir  scale
Leg legs[6] = {
  {  0,  10,   3,    0,   -9,  +1,  1.0f },  // 0  Front-Left
  {  3,  10,  -1,    0,   -9,  +1,  1.0f },  // 1  Mid-Left
  { 29,  10,  -4,    0,   -9,  +1,  1.0f },  // 2  Rear-Left
  {  6,  10,   0,    0,   -9,  -1,  1.0f },  // 3  Front-Right
  {  9,  10,   2,    0,   -9,  -1,  1.0f },  // 4  Mid-Right
  { 26,  10,   6,    0,   -9,  -1,  1.0f },  // 5  Rear-Right
};

// Crab stance — legs rotated to face sideways
Leg crabLegs[6] = {
  {  0,  10,   8,   8,   -8,  +1,  1.0f },  // 0  Front-Left
  {  3,  10,   0,   0,   -8,  +1,  1.0f },  // 1  Mid-Left
  { 29,  10,  -8,  -8,   -8,  +1,  1.0f },  // 2  Rear-Left
  {  6,  10,  -8,  -8,   -8,  -1,  1.0f },  // 3  Front-Right
  {  9,  10,   0,   0,   -8,  -1,  1.0f },  // 4  Mid-Right
  { 26,  10,   8,   8,   -8,  -1,  1.0f },  // 5  Rear-Right
};

const int GRP_A[3]    = { 0, 2, 4 };  // Front-Left, Rear-Left,   Mid-Right
const int GRP_B[3]    = { 1, 3, 5 };  // Mid-Left,   Front-Right, Rear-Right
const int WAVE_SEQ[6] = { 2, 1, 0, 5, 4, 3 };  // RL ML FL RR MR FR

float currentJointAngles[32]; // Stores latest servo deviation angles for CoppeliaSim
bool anglesDirty = false;     // True when angles have been updated by moveFootXYZ


// ───────────────────────────────────────────────────────────────
//  SECTION 2 — IK SOLVER
// ───────────────────────────────────────────────────────────────
void moveFootXYZ(uint8_t id, float x, float y, float z, uint16_t t) {

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

  uint16_t p1 = constrain((int)(SERVO_MID +  cDeg          * DEG_TO_US), 500, 2500);
  uint16_t p2 = constrain((int)(SERVO_MID + (fDeg - 90.0f) * DEG_TO_US), 500, 2500);
  uint16_t p3 = constrain((int)(SERVO_MID - (tDeg - 90.0f) * DEG_TO_US), 500, 2500);

  currentJointAngles[id] = cDeg;
  currentJointAngles[id+1] = fDeg;
  currentJointAngles[id+2] = tDeg;

  anglesDirty = true;

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
//  SECTION 3 — GAIT SHAPES  (plug-and-play, 2-D)
//
//  indepXY = false → differential: finalScale = jFwd + jTurn×coxaDir
//  indepXY = true  → independent:  scaleY = jFwd,  scaleX = jStrafe
// ───────────────────────────────────────────────────────────────

float defaultX   (int i){ return legs[i].restX;     }
float defaultCrabX(int i){ return crabLegs[i].restX; }
float defaultY   (int i){ return legs[i].walkY;     }
float defaultCrabY(int i){ return crabLegs[i].walkY; }

// Gait 0 — Tripod Walk
float tripodSwingY(int i){ return legs[i].walkY - (3.0f * legs[i].coxaDir * legs[i].strideScale); }
float tripodDragY (int i){ return legs[i].walkY + (3.0f * legs[i].coxaDir * legs[i].strideScale); }

// Gait 1 — Crab Walk (unified 2-D)
float crabSwingY(int i){ return crabLegs[i].walkY - (3.0f * crabLegs[i].coxaDir * crabLegs[i].strideScale); }
float crabDragY (int i){ return crabLegs[i].walkY + (3.0f * crabLegs[i].coxaDir * crabLegs[i].strideScale); }
float crabSwingX(int i){ return crabLegs[i].restX - (3.0f * crabLegs[i].coxaDir * crabLegs[i].strideScale); }
float crabDragX (int i){ return crabLegs[i].restX + (3.0f * crabLegs[i].coxaDir * crabLegs[i].strideScale); }

// Gait 2 — Diagonal Walk (normal legs, independent X+Y)
float walkSwingX(int i){ return legs[i].restX - (3.0f * legs[i].coxaDir * legs[i].strideScale); }
float walkDragX (int i){ return legs[i].restX + (3.0f * legs[i].coxaDir * legs[i].strideScale); }

// Gait 3 — Wave Gait
float waveSwingY(int i){ return legs[i].walkY - (2.5f * legs[i].coxaDir * legs[i].strideScale); }
float waveDragY (int i){ return legs[i].walkY + (2.5f * legs[i].coxaDir * legs[i].strideScale); }

struct GaitDef {
  const char* name;
  float(*swingX)(int);  float(*dragX)(int);
  float(*swingY)(int);  float(*dragY)(int);
  bool indepXY;
};

GaitDef GAIT_TABLE[] = {
  { "Tripod Walk",   defaultX,   defaultX,  tripodSwingY, tripodDragY, false },  // 0
  { "Crab Walk",     crabSwingX, crabDragX, crabSwingY,   crabDragY,   true  },  // 1
  { "Diagonal Walk", walkSwingX, walkDragX, tripodSwingY, tripodDragY, true  },  // 2
  { "Wave Gait",     defaultX,   defaultX,  waveSwingY,   waveDragY,   false },  // 3
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
//
//  Two engine paths:
//   A) Tripod (gaits 0/1/2): SEND_A→PLANT_A→SEND_B→PLANT_B→RECENTER
//   B) Wave   (gait 3):      WAVE_STEP→WAVE_PLANT  ×6
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
    moveFootXYZ(leg.id, stepX, stepY, leg.restZ - LIFT_H, moveTime);
  }

  for (int i = 0; i < 3; i++) {
    int   idx    = dragGrp[i];
    Leg&  leg    = crabMode ? crabLegs[idx] : legs[idx];
    float scaleY = g.indepXY ? jFwd : constrain(jFwd + (jTurn * leg.coxaDir), -1.0f, 1.0f);
    float scaleX = g.indepXY ? jStrafe : scaleY;
    float slideX = leg.restX + (g.dragX(idx) - leg.restX) * scaleX;
    float slideY = leg.walkY + (g.dragY(idx) - leg.walkY) * scaleY;
    moveFootXYZ(leg.id, slideX, slideY, leg.restZ, moveTime);
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
    moveFootXYZ(leg.id, plantX, plantY, leg.restZ, moveTime / 3);
  }
}

void recenterLegs() {
  for (int i = 0; i < 6; i++) {
    Leg& leg = crabMode ? crabLegs[i] : legs[i];
    moveFootXYZ(leg.id, leg.restX, leg.walkY, leg.restZ, moveTime / 2);
  }
}

void waveStep(uint8_t phaseIdx) {
  GaitDef& g        = GAIT_TABLE[activeGait];
  int      swingIdx = WAVE_SEQ[phaseIdx];
  Leg&     swLeg    = legs[swingIdx];
  float    scale    = constrain(jFwd + (jTurn * swLeg.coxaDir), -1.0f, 1.0f);

  float swY = swLeg.walkY + (g.swingY(swingIdx) - swLeg.walkY) * scale;
  moveFootXYZ(swLeg.id, swLeg.restX, swY, swLeg.restZ - LIFT_H, moveTime);

  for (int i = 0; i < 6; i++) {
    if (i == swingIdx) continue;
    Leg&  leg      = legs[i];
    float legScale = constrain(jFwd + (jTurn * leg.coxaDir), -1.0f, 1.0f);
    int   phasesSince = 0;
    for (int p = 1; p <= 5; p++) {
      if (WAVE_SEQ[(phaseIdx - p + 6) % 6] == i) { phasesSince = p; break; }
    }
    float t      = phasesSince / 5.0f;
    float frontY = leg.walkY + (g.swingY(i) - leg.walkY) * legScale;
    float rearY  = leg.walkY + (g.dragY(i)  - leg.walkY) * legScale;
    moveFootXYZ(leg.id, leg.restX, frontY + (rearY - frontY) * t, leg.restZ, moveTime);
  }
}

void wavePlant(uint8_t phaseIdx) {
  int      swingIdx = WAVE_SEQ[phaseIdx];
  Leg&     leg      = legs[swingIdx];
  GaitDef& g        = GAIT_TABLE[activeGait];
  float    scale    = constrain(jFwd + (jTurn * leg.coxaDir), -1.0f, 1.0f);
  float    plantY   = leg.walkY + (g.swingY(swingIdx) - leg.walkY) * scale;
  moveFootXYZ(leg.id, leg.restX, plantY, leg.restZ, moveTime / 4);
}

void startGait(bool continuous) {
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
  // NOTE: activeGait is NOT reset — gait persists across stops.
  for (int i = 0; i < 6; i++) {
    Leg& leg = crabMode ? crabLegs[i] : legs[i];
    moveFootXYZ(leg.id, leg.restX, leg.rotY, leg.restZ, moveTime);
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

  for (int i = 0; i < 3; i++){
      moveFootXYZ(legs[3].id, legs[3].restX, legs[3].walkY, legs[3].restZ - 9, 500);
      delay(600);
      moveFootXYZ(legs[3].id, legs[3].restX, legs[3].walkY, legs[3].restZ - 3, 500);
      delay(600);
    }
    
  // The 3.6 seconds of delay() starves the PS2 receiver, causing it 
  // to timeout and send a frame of junk data (which looks like R1 or R3 
  // being pressed, triggering Sleep mode). 
  // We reset ps2Warm to ignore the next 10 frames of junk data!
  extern uint8_t ps2Warm;
  ps2Warm = 0; 
}

// ── Log helper: prints ">> GaitName: Direction" ───────────────
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
// ───────────────────────────────────────────────────────────────

void adjustHeight(float delta) {
  float newZ = bodyZ + delta;
  if (newZ < HEIGHT_MIN || newZ > HEIGHT_MAX) {
    btLog(delta > 0 ? ">> Max height reached\n" : ">> Min height reached\n");
    return;
  }
  bodyZ = newZ;
  for (int i = 0; i < 6; i++) {
    legs[i].restZ     = bodyZ;
    crabLegs[i].restZ = bodyZ;
  }
  standStill();
  char buf[32]; snprintf(buf, sizeof(buf), ">> Height: %.1f cm\n", bodyZ); btLog(buf);
}

void increaseHeight() { adjustHeight(+HEIGHT_STEP); }
void decreaseHeight() { adjustHeight(-HEIGHT_STEP); }


// ───────────────────────────────────────────────────────────────
//  SECTION 8 — BT INPUT PARSER  (debug / override channel)
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
      lastCmdName = "SITTING";   btLog(">> Sitting\n");  sittingLegs(); break;

    case 'Z': case 'z':
      lastCmdName = "SLEEPING";  btLog(">> Sleeping\n"); sleepingLegs(); break;

    case 'H': case 'h':
      lastCmdName = "SAY HI";    btLog(">> Hello (Say Hi)\n"); helloLegs(); break;

    case 'T': case 't':
      // On-demand telemetry snapshot: sends one line of all 18 joint angles
      // to the PC (CoppeliaSim bridge) immediately, without needing a gait cycle.
      // Useful for calibration and testing the Python bridge connection.
      lastCmdName = "TELEMETRY";
      btLog(">> Telemetry snapshot sent\n");
      sendAnglesToPC();
      break;

    case 'U': case 'u':
      lastCmdName = "HEIGHT UP"; increaseHeight(); break;

    case 'D': case 'd':
      lastCmdName = "HEIGHT DN"; decreaseHeight(); break;

    case 'S': case 's':
      lastCmdName = "STOP"; btLog(">> standStill\n"); standStill(); break;

    case '+':
      if (moveTime > SPEED_MIN) moveTime -= SPEED_STEP;
      { char b[28]; snprintf(b, sizeof(b), ">> Speed: %u ms\n", moveTime); btLog(b); }
      break;

    case '-':
      if (moveTime < SPEED_MAX) moveTime += SPEED_STEP;
      { char b[28]; snprintf(b, sizeof(b), ">> Speed: %u ms\n", moveTime); btLog(b); }
      break;

    case '?':
      btLog("─── SpiderBot V4.1 ───\n");
      btLog("Last cmd : "); btLog(lastCmdName);                  btLog("\n");
      btLog("Gait     : "); btLog(GAIT_TABLE[activeGait].name);  btLog("\n");
      btLog("Crab mode: "); btLog(crabMode ? "ON\n" : "OFF\n");
      btLog("Speed    : "); btLog(moveTime);   btLog(" ms\n");
      btLog("Height   : "); btLog(bodyZ);      btLog(" cm\n");
      btLog("Battery  : "); btLog(voltageInVolts); btLog(" V\n");
      btLog("jFwd     : "); btLog(jFwd);
      btLog("  jTurn  : "); btLog(jTurn);
      btLog("  jStrafe: "); btLog(jStrafe);    btLog("\n");
      btLog("Queue    : "); btLog(qCount);     btLog(" waiting\n");
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
        moveTime    = (uint16_t)map((long)(mag * 100), 0, 100, SPEED_MAX, SPEED_MIN);
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
      if (idx <= 3) {
        crabMode   = (idx == 1);
        activeGait = idx;
        jFwd = 1.0f; jTurn = 0.0f; jStrafe = 0.0f;
        char buf[56];
        snprintf(buf, sizeof(buf), ">> Gait: %s%s\n",
                 GAIT_TABLE[idx].name,
                 crabMode ? "  (F/B=fwd-back  L/R=strafe)" : "");
        btLog(buf);
      } else {
        btLog(">> Bad gait number. Use g0, g1, g2 or g3.\n");
      }
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
//
//  readPS2() is called every loop().  It reads the controller
//  state and maps it directly to the same jFwd / jTurn / jStrafe
//  variables and movement functions used by the BT parser.
//
//  ── Button map ───────────────────────────────────────────────
//
//  LEFT STICK   → analog forward/backward + left/right arc
//                 LY → jFwd   (128=centre, 0=full fwd, 255=full back)
//                 LX → jTurn  (128=centre, 0=full left, 255=full right)
//
//  RIGHT STICK  → analog strafe (used in crab/diagonal mode)
//                 RX → jStrafe (128=centre, 0=full left, 255=full right)
//
//  D-PAD UP     → walkForward()  / crabForward()
//  D-PAD DOWN   → walkBackward() / crabBackward()
//  D-PAD LEFT   → turnLeft()     / crabLeft()
//  D-PAD RIGHT  → turnRight()    / crabRight()
//
//  L1 (pressed) → sittingLegs()
//  R1 (pressed) → sleepingLegs()
//  L2 (pressed) → speed down (-)
//  R2 (pressed) → speed up  (+)
//
//  △ (pressed)  → increaseHeight()
//  × (pressed)  → decreaseHeight()
//  ○ (held)     → rotateLeft()
//  □ (held)     → rotateRight()
//
//  START (press)→ standStill() / stop
//  SELECT (press)→ cycle gaits  g0→g1→g2→g3→g0
//  L3 (click)   → toggle crab mode  (g1 ↔ g0)
//  R3 (click)   → master ON/OFF toggle
//
//  ── Stick → float conversion ─────────────────────────────────
//  Raw 0–255, centre 128.  After deadzone, map to -1.0…+1.0.
//  LY is inverted (0 = stick pushed up = forward = positive jFwd).
// ───────────────────────────────────────────────────────────────

// Convert a raw PS2 axis byte (0–255, centre=128) to -1.0…+1.0.
// invert=true flips the sign (used for Y axes where up=forward).
float ps2Axis(byte raw, bool invert) {
  int centred = (int)raw - 128;
  if (abs(centred) < PS2_DEADZONE) return 0.0f;
  float norm = constrain(centred / 127.0f, -1.0f, 1.0f);
  return invert ? -norm : norm;
}



// Track whether any movement button/stick was active last tick
// so we can stop when everything is released.
bool ps2Moving = false;

// Skip first few frames after controller connects to avoid
// junk data triggering accidental commands.
uint8_t ps2Warm = 0;
const uint8_t PS2_WARMUP_FRAMES = 10;

// PS2 protocol needs ~16ms minimum between SPI reads.
// Polling faster causes data corruption → phantom button presses.
const unsigned long PS2_POLL_MS = 50;   // 20 Hz — safe for PS2
unsigned long lastPS2Read = 0;

void readPS2() {
  // ── Rate limit: PS2 protocol needs ≥16ms between reads ─────
  unsigned long now = millis();
  if (now - lastPS2Read < PS2_POLL_MS) return;
  lastPS2Read = now;

  ps2x.read_gamepad(false, 0);

  // ── Connection check (Non-blocking) ──────────────────────────
  // If the receiver is powered but the controller is OFF, SPI reads 
  // return 255 for axes. We must ignore this so the bot doesn't run away.
  byte rawLY = ps2x.Analog(PSS_LY);
  byte rawRY = ps2x.Analog(PSS_RY);
  if (rawLY == 255 && rawRY == 255) {
    ps2Warm = 0; // Keep warmup at 0 until a real controller connects
    return;
  }

  // ── ON/OFF Master Switch (using R3 since MODE is unreadable hardware) ─
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

  // If the bot is turned OFF, ignore all other PS2 inputs
  if (!isBotOn) return;

  // ── Warmup: skip first N frames after boot ─────────────────
  if (ps2Warm < PS2_WARMUP_FRAMES) {
    ps2Warm++;
    return;   // ignore all inputs until sticks stabilise
  }

  // ── Read sticks ────────────────────────────────────────────
  float ly = ps2Axis(ps2x.Analog(PSS_LY), true);   // up = +jFwd
  float lx = ps2Axis(ps2x.Analog(PSS_LX), false);  // right = +jTurn
  float rx = ps2Axis(ps2x.Analog(PSS_RX), true);    // right = +jStrafe

  bool stickActive = (fabsf(ly) > 0.0f || fabsf(lx) > 0.0f || fabsf(rx) > 0.0f);

  // ── Movement: D-pad (held) ─────────────────────────────────
  //    Same pattern as V2.4.1 — only active while button is HELD.
  //    As soon as all movement buttons are released → stop.
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

  // ── Movement: ○ / □ rotate (held) ──────────────────────────
  // FIXED: Changed back to Button() so they hold continuously.
  // ButtonPressed() only triggers once and instantly stops.
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

  // ── Movement: Analog sticks ────────────────────────────────
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

  // ── Stop: all movement released ────────────────────────────
  bool anyMovement = dpadActive || stickActive;
  if (!anyMovement && ps2Moving) {
    btLog(">> PS2: Stop\n");
    standStill();
  }
  ps2Moving = anyMovement;

  // ══════════════════════════════════════════════════════════════
  //  NON-MOVEMENT ACTIONS (one-shot — ButtonPressed)
  // ══════════════════════════════════════════════════════════════

  // ── Height: △ / × (one-shot per press) ─────────────────────
  if (ps2x.ButtonPressed(PSB_TRIANGLE)) {
    btLog(">> PS2: Height Up\n");
    increaseHeight();
  }
  if (ps2x.ButtonPressed(PSB_CROSS)) {
    btLog(">> PS2: Height Down\n");
    decreaseHeight();
  }

  // ── Posture: L1 = sit, R1 = sleep (one-shot per press) ───────
  if (ps2x.ButtonPressed(PSB_L1)) {
    btLog(">> PS2: Sitting\n");
    lastCmdName = "SITTING";
    sittingLegs();
  }
  if (ps2x.ButtonPressed(PSB_R1)) {
    btLog(">> PS2: Sleeping\n");
    lastCmdName = "SLEEPING";
    sleepingLegs();
  }

  // ── START = stop ──────────────────────────────────────────
  if (ps2x.ButtonPressed(PSB_START)) {
    btLog(">> PS2: Stand Still\n");
    lastCmdName = "STOP";
    standStill();
  }

  // ── SELECT = cycle gaits: g0 → g1 → g2 → g3 → g0 ──────────
  if (ps2x.ButtonPressed(PSB_SELECT)) {
    uint8_t next = (activeGait + 1) % GAIT_COUNT;
    crabMode   = (next == 1);
    activeGait = next;
    jFwd = 0.0f; jTurn = 0.0f; jStrafe = 0.0f;
    char buf[48];
    snprintf(buf, sizeof(buf), ">> PS2: Gait → %s\n", GAIT_TABLE[activeGait].name);
    btLog(buf);
  }

  // ── L3 click = toggle crab mode ────────────────────────────
  if (ps2x.ButtonPressed(PSB_L3)) {
    crabMode   = !crabMode;
    activeGait = crabMode ? 1 : 0;
    jFwd = 0.0f; jTurn = 0.0f; jStrafe = 0.0f;
    btLog(crabMode ? ">> PS2: Crab ON\n" : ">> PS2: Crab OFF\n");
  }

  // ── L2/R2 = speed ──────────────────────────────────────────
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
    btLog("SpiderBot V4.1 Ready!\n");
    btLog("PS2 controller is primary input.\n");
    btLog("BT debug: F/B/L/R/W/X/A/Z/U/D/S/+/-/?/g0-g3/jFWD,TURN\n");
  }
  if (event == ESP_SPP_CLOSE_EVT) {
    btConnected = false;
    standStill();
    Serial.println("BT: Disconnected");
  }
}


// ───────────────────────────────────────────────────────────────
//  OTA (Over-The-Air Updates) SETUP
// ───────────────────────────────────────────────────────────────
void setupOTA() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi for OTA");

  // Wait max 5 seconds for connection so we don't block the bot offline
  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 5000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWi-Fi Connected! IP address: ");
    Serial.println(WiFi.localIP());

    ArduinoOTA.setHostname("spiderbot");

    ArduinoOTA.onStart([]() {
      String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
      Serial.println("\nStart OTA updating " + type);
      standStill(); // Stop moving during update
    });
    ArduinoOTA.onEnd([]() { Serial.println("\nEnd OTA"); });
    
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

    ArduinoOTA.begin();
    Serial.println("OTA Ready.");
  } else {
    Serial.println("\nWi-Fi timeout. Turning off Wi-Fi radio to save battery.");
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
  Serial.println("SpiderBot V4.1 — Bluetooth: SpiderBot");

  setupOTA();

  // Enable internal pull-up on PS2 DAT (MISO) line.
  // Without this, the data line floats between reads and
  // ESP32 reads noise as phantom button presses.
  pinMode(PS2_DAT, INPUT_PULLUP);

  // Step 1: Configure SPI bus to the receiver module.
  // config_gamepad() returns 0 when SPI wiring is OK. It does NOT block!
  int ps2error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, false, false);
  if (ps2error == 0) {
    Serial.println("PS2 SPI OK.");
  } else {
    Serial.println("PS2 SPI Failed.");
  }

  // Step 2: Connection check is now handled non-blockingly inside readPS2()
  // so that the main loop() can start instantly, allowing Bluetooth to work
  // even if the PS2 controller is turned off.

  delay(500);
  standStill();

}


// ───────────────────────────────────────────────────────────────
//  LOOP
// ───────────────────────────────────────────────────────────────
void loop() {
  // Process incoming OTA wireless updates if available
  ArduinoOTA.handle();

  // 1. Read BT / Serial (debug override)
  while (SerialBT.available()) parseInput((char)SerialBT.read());
  while (Serial.available())   parseInput((char)Serial.read());

  // 2. Read PS2 controller (primary input)
  readPS2();

  // 3. Advance gait state machine
  gaitTick();

  // 4. Poll battery voltage every 5 s
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

  // 5. Send joint angles to PC (CoppeliaSim) only when they change
  if (anglesDirty) { 
    sendAnglesToPC();
    anglesDirty = false;
  }
}

void sendAnglesToPC() {
  // Prefix 'A:' marks this as an angle packet.
  // The Python bridge filters on startswith('A:') so any stray
  // boot messages or non-telemetry text is safely ignored.
  Serial.print("A:");
  for (int i = 0; i < 6; i++) {
    uint8_t id = legs[i].id;
    Serial.print(currentJointAngles[id],   2); Serial.print(",");
    Serial.print(currentJointAngles[id+1], 2); Serial.print(",");
    Serial.print(currentJointAngles[id+2], 2);
    if (i < 5) Serial.print(",");
  }
  Serial.println();
}
