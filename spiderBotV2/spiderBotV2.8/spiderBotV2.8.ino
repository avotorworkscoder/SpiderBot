/*===============================================================
 * SpiderBot V2.8
 * Author  : Amit Parihar
 * Wiring  : UART2  RX=16  TX=17  →  LSC-32 servo controller
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
 *   8. Input Parser    — BT + Serial command handler
 *   9. setup() + loop()
 *
 * ── WHAT'S NEW IN V2.8 vs V2.7 ──────────────────────────────
 *   - True wave gait: 6-phase engine where only ONE leg lifts
 *     at a time (rear→front left, then rear→front right).
 *     5 legs always on ground = maximum stability.
 *   - Unified crab gait: 4 crab functions merged into 2
 *     (crabSwingY/crabDragY + crabSwingX/crabDragX).
 *     Direction controlled by jFwd (Y) and jStrafe (X).
 *   - indepXY flag in GaitDef: gaits with indepXY=true use
 *     independent jFwd (Y-axis) and jStrafe (X-axis) scaling.
 *     Enables true diagonal movement, not arc turns.
 *   - Diagonal Walk (g2): uses tripod strides + walk X strides
 *     with indepXY=true.  Joystick diagonal = straight line.
 *   - Joystick respects active gait: no longer forces gait 0.
 *     In g1/g2 mode joystick maps to independent X/Y control.
 *
 * ── COMMAND REFERENCE ────────────────────────────────────────
 *   F / B          walk forward / backward  (or crab F/B)
 *   L / R          arc turn left / right    (or crab strafe)
 *   W / X          rotate left / right (spin in place)
 *   A              sitting position
 *   Z              sleeping position
 *   U / D          raise / lower body height
 *   S              stop / stand still
 *   + / -          speed up / slow down
 *   g0             Tripod Walk   (crab mode OFF)
 *   g1             Crab Walk     (crab mode ON  — F/B/L/R remapped)
 *   g2             Diagonal Walk (crab mode OFF — joystick = diagonal)
 *   g3             Wave Gait     (crab mode OFF — slow, very stable)
 *   jFWD,TURN      joystick  e.g. j66,33 or j0,50
 *   ?              status report
 *
 * ── BT APP SETUP (Serial Bluetooth Terminal) ─────────────────
 *   Settings → Line ending → Newline (\n)
 *===============================================================*/

#include <Arduino.h>
#include <math.h>
#include "BluetoothSerial.h"
#include <LobotServoController.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
  #error "Bluetooth not enabled — Arduino IDE: Tools → ESP32 Bluetooth"
#endif


// ───────────────────────────────────────────────────────────────
//  SECTION 1 — HARDWARE, CONSTANTS, LEG DATA
// ───────────────────────────────────────────────────────────────

HardwareSerial       LSC(2);
BluetoothSerial      SerialBT;
LobotServoController controller(LSC);

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

// ── Logging helpers ────────────────────────────────────────────
void btLog(const char* msg) { SerialBT.print(msg);    Serial.print(msg);    }
void btLog(int          num) { SerialBT.print(num);    Serial.print(num);    }
void btLog(float        num) { SerialBT.print(num, 2); Serial.print(num, 2); }

// ── Leg data structure ─────────────────────────────────────────
struct Leg {
  uint8_t id;
  float   restX, walkY, rotY, restZ;
  int     coxaDir;
  float   strideScale;
};

// Normal walking stance — legs point outward like a spider.
//            id    X  walkY  rotY    Z   dir  scale
Leg legs[6] = {
  {  0,  10,   3,    0,   -9,  +1,  1.0f },  // 0  Front-Left
  {  3,  10,  -1,    0,   -9,  +1,  1.0f },  // 1  Mid-Left
  { 29,  10,  -4,    0,   -9,  +1,  1.0f },  // 2  Rear-Left
  {  6,  10,   0,    0,   -9,  -1,  1.0f },  // 3  Front-Right
  {  9,  10,   2,    0,   -9,  -1,  1.0f },  // 4  Mid-Right
  { 26,  10,   6,    0,   -9,  -1,  1.0f },  // 5  Rear-Right
};

// Crab stance — legs rotated to face sideways.
// walkY values are tuned so all legs point toward the same wall.
// rotY mirrors walkY so standStill() homes correctly in crab mode.
Leg crabLegs[6] = {
  {  0,  10,   8,   8,   -8,  +1,  1.0f },  // 0  Front-Left
  {  3,  10,   0,   0,   -8,  +1,  1.0f },  // 1  Mid-Left
  { 29,  10,  -8,  -8,   -8,  +1,  1.0f },  // 2  Rear-Left
  {  6,  10,  -8,  -8,   -8,  -1,  1.0f },  // 3  Front-Right
  {  9,  10,   0,   0,   -8,  -1,  1.0f },  // 4  Mid-Right
  { 26,  10,   8,   8,   -8,  -1,  1.0f },  // 5  Rear-Right
};

// Tripod groups — two stable triangles that alternate every half-cycle.
const int GRP_A[3] = { 0, 2, 4 };  // Front-Left, Rear-Left,   Mid-Right
const int GRP_B[3] = { 1, 3, 5 };  // Mid-Left,   Front-Right, Rear-Right

// Wave sequence — legs lift one at a time, rear→front per side.
const int WAVE_SEQ[6] = { 2, 1, 0, 5, 4, 3 };  // RL ML FL RR MR FR


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
//  Each gait defines swingX/dragX and swingY/dragY functions.
//  The indepXY flag controls scaling mode:
//    false → differential: finalScale = jFwd + jTurn×coxaDir
//    true  → independent:  scaleY = jFwd,  scaleX = jStrafe
//
//  Wave gait (g3) uses swingY/dragY to define the stride range,
//  but its own 6-phase engine handles per-leg interpolation.
// ───────────────────────────────────────────────────────────────

// ── Neutral X helpers (most gaits don't move X) ───────────────
float defaultX   (int i){ return legs[i].restX;     }
float defaultCrabX(int i){ return crabLegs[i].restX; }

// ── Neutral Y helpers ─────────────────────────────────────────
float defaultY   (int i){ return legs[i].walkY;     }
float defaultCrabY(int i){ return crabLegs[i].walkY; }

// ── Gait 0 — Tripod Walk ──────────────────────────────────────
float tripodSwingY(int i){ return legs[i].walkY - (3.0f * legs[i].coxaDir * legs[i].strideScale); }
float tripodDragY (int i){ return legs[i].walkY + (3.0f * legs[i].coxaDir * legs[i].strideScale); }

// ── Gait 1 — Crab Walk  (unified 2-D: Y + X) ────────────────
// Uses crabLegs[].  Y driven by jFwd, X driven by jStrafe.
//   jFwd    = +1/−1  →  forward / backward
//   jStrafe = +1/−1  →  strafe left / right
float crabSwingY(int i){ return crabLegs[i].walkY - (3.0f * crabLegs[i].coxaDir * crabLegs[i].strideScale); }
float crabDragY (int i){ return crabLegs[i].walkY + (3.0f * crabLegs[i].coxaDir * crabLegs[i].strideScale); }
float crabSwingX(int i){ return crabLegs[i].restX - (3.0f * crabLegs[i].coxaDir * crabLegs[i].strideScale); }
float crabDragX (int i){ return crabLegs[i].restX + (3.0f * crabLegs[i].coxaDir * crabLegs[i].strideScale); }

// ── Walk X stride (normal legs, for diagonal walk) ────────────
float walkSwingX(int i){ return legs[i].restX - (3.0f * legs[i].coxaDir * legs[i].strideScale); }
float walkDragX (int i){ return legs[i].restX + (3.0f * legs[i].coxaDir * legs[i].strideScale); }

// ── Gait 3 — Wave Gait (true 6-phase, 1-leg-at-a-time) ───────
// Stride defines the full range; the wave engine interpolates
// each leg's position based on how many phases since it last swung.
float waveSwingY(int i){ return legs[i].walkY - (2.5f * legs[i].coxaDir * legs[i].strideScale); }
float waveDragY (int i){ return legs[i].walkY + (2.5f * legs[i].coxaDir * legs[i].strideScale); }

// ── Gait table ─────────────────────────────────────────────────
//  Each row: name, swingX, dragX, swingY, dragY, indepXY
//  indepXY = true  →  X scaled by jStrafe, Y scaled by jFwd  (2-D)
//  indepXY = false →  both axes use finalScale = jFwd + jTurn×coxaDir
struct GaitDef {
  const char* name;
  float(*swingX)(int);  float(*dragX)(int);
  float(*swingY)(int);  float(*dragY)(int);
  bool indepXY;         // true = independent X/Y scaling
};

GaitDef GAIT_TABLE[] = {
  // idx  name               swingX         dragX          swingY          dragY         indep
  {  "Tripod Walk",    defaultX,      defaultX,      tripodSwingY,   tripodDragY,   false },  // 0
  {  "Crab Walk",      crabSwingX,    crabDragX,     crabSwingY,     crabDragY,     true  },  // 1
  {  "Diagonal Walk",  walkSwingX,    walkDragX,     tripodSwingY,   tripodDragY,   true  },  // 2
  {  "Wave Gait",      defaultX,      defaultX,      waveSwingY,     waveDragY,     false },  // 3
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
//  Two engine paths share the same state machine:
//
//  A) Tripod engine (gaits 0, 1, 2) — 2-phase:
//     SEND_A → PLANT_A → SEND_B → PLANT_B → RECENTER
//     3 legs swing while 3 drag.  Fast, moderate stability.
//
//  B) Wave engine (gait 3) — 6-phase:
//     WAVE_STEP → WAVE_PLANT  (×6, cycling through WAVE_SEQ)
//     1 leg swings while 5 drag.  Slow, maximum stability.
//
//  Scaling modes (controlled by GaitDef.indepXY):
//    false → differential: finalScale = jFwd + jTurn×coxaDir
//    true  → independent:  scaleY = jFwd,  scaleX = jStrafe
// ───────────────────────────────────────────────────────────────

float jFwd    = 0.0f;
float jTurn   = 0.0f;
float jStrafe = 0.0f;   // independent X-axis scale (used when gait has indepXY)

enum GaitPhase { IDLE, SEND_A, PLANT_A, SEND_B, PLANT_B, RECENTER, WAVE_STEP, WAVE_PLANT };
GaitPhase     gaitPhase    = IDLE;
bool          looping      = false;
unsigned long phaseStart   = 0;
uint8_t       wavePhaseIdx = 0;   // 0–5: index into WAVE_SEQ for wave gait

void handleCommand(char cmd);  // forward declaration

// ── Tripod engine helpers (unchanged from V2.7) ───────────────

void sendPhase(bool groupASwings) {
  GaitDef&   g        = GAIT_TABLE[activeGait];
  const int* swingGrp = groupASwings ? GRP_A : GRP_B;
  const int* dragGrp  = groupASwings ? GRP_B : GRP_A;

  for (int i = 0; i < 3; i++) {
    int   idx   = swingGrp[i];
    Leg&  leg   = crabMode ? crabLegs[idx] : legs[idx];
    float scaleY = g.indepXY ? jFwd
                             : constrain(jFwd + (jTurn * leg.coxaDir), -1.0f, 1.0f);
    float scaleX = g.indepXY ? jStrafe : scaleY;
    float stepX  = leg.restX + (g.swingX(idx) - leg.restX) * scaleX;
    float stepY  = leg.walkY + (g.swingY(idx) - leg.walkY) * scaleY;
    moveFootXYZ(leg.id, stepX, stepY, leg.restZ - LIFT_H, moveTime);
  }

  for (int i = 0; i < 3; i++) {
    int   idx   = dragGrp[i];
    Leg&  leg   = crabMode ? crabLegs[idx] : legs[idx];
    float scaleY = g.indepXY ? jFwd
                             : constrain(jFwd + (jTurn * leg.coxaDir), -1.0f, 1.0f);
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
    int   idx   = grp[i];
    Leg&  leg   = crabMode ? crabLegs[idx] : legs[idx];
    float scaleY = g.indepXY ? jFwd
                             : constrain(jFwd + (jTurn * leg.coxaDir), -1.0f, 1.0f);
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

// ── Wave engine helpers (new in V2.8) ─────────────────────────

void waveStep(uint8_t phaseIdx) {
  GaitDef& g       = GAIT_TABLE[activeGait];
  int      swingIdx = WAVE_SEQ[phaseIdx];
  Leg&     swLeg    = legs[swingIdx];

  float scale = constrain(jFwd + (jTurn * swLeg.coxaDir), -1.0f, 1.0f);

  // Swing leg: lift + move to front of stride
  float swY = swLeg.walkY + (g.swingY(swingIdx) - swLeg.walkY) * scale;
  moveFootXYZ(swLeg.id, swLeg.restX, swY, swLeg.restZ - LIFT_H, moveTime);

  // Drag all other 5 legs: interpolate between swing and drag positions
  for (int i = 0; i < 6; i++) {
    if (i == swingIdx) continue;
    Leg& leg = legs[i];
    float legScale = constrain(jFwd + (jTurn * leg.coxaDir), -1.0f, 1.0f);

    // How many phases since this leg last swung?
    int phasesSince = 0;
    for (int p = 1; p <= 5; p++) {
      if (WAVE_SEQ[(phaseIdx - p + 6) % 6] == i) { phasesSince = p; break; }
    }

    // t = 0.0 right after swing (at front), t = 1.0 about to swing (at rear)
    float t     = phasesSince / 5.0f;
    float frontY = leg.walkY + (g.swingY(i) - leg.walkY) * legScale;
    float rearY  = leg.walkY + (g.dragY(i)  - leg.walkY) * legScale;
    float dragY  = frontY + (rearY - frontY) * t;

    moveFootXYZ(leg.id, leg.restX, dragY, leg.restZ, moveTime);
  }
}

void wavePlant(uint8_t phaseIdx) {
  int  swingIdx = WAVE_SEQ[phaseIdx];
  Leg& leg      = legs[swingIdx];
  GaitDef& g    = GAIT_TABLE[activeGait];

  float scale  = constrain(jFwd + (jTurn * leg.coxaDir), -1.0f, 1.0f);
  float plantY = leg.walkY + (g.swingY(swingIdx) - leg.walkY) * scale;
  moveFootXYZ(leg.id, leg.restX, plantY, leg.restZ, moveTime / 4);
}

// ── Gait start / stop / tick ──────────────────────────────────

void startGait(bool continuous) {
  looping = continuous;
  if (activeGait == 3) {    // wave gait → 6-phase engine
    wavePhaseIdx = 0;
    gaitPhase    = WAVE_STEP;
  } else {                  // tripod/crab/diagonal → 2-phase engine
    gaitPhase    = SEND_A;
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

    // ── Tripod engine phases ────────────────────────────────
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

    // ── Wave engine phases ──────────────────────────────────
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
//
//  Normal mode (crabMode=false):
//    F/B = tripod walk,  L/R = arc turn,  W/X = rotate
//
//  Crab mode (crabMode=true, activated by g1):
//    F/B = crab forward/backward  (jFwd = ±1)
//    L/R = crab strafe            (jStrafe = ±1)
//    W/X = rotate (unchanged — always uses tripod differential)
//
//  Wave gait and Diagonal Walk are activated by g3/g2,
//  then F/B/L/R/joystick drive them through the gait engine.
// ───────────────────────────────────────────────────────────────

void standStill() {
  stopGait();
  queueClear();
  jFwd = jTurn = jStrafe = 0.0f;
  activeGait = 0;
  for (int i = 0; i < 6; i++) {
    Leg& leg = crabMode ? crabLegs[i] : legs[i];
    moveFootXYZ(leg.id, leg.restX, leg.rotY, leg.restZ, moveTime);
  }
}

void sittingLegs() {
  stopGait();
  queueClear();
  jFwd = jTurn = jStrafe = 0.0f;
  activeGait = 0;
  for (int i = 0; i < 6; i++)
    moveFootXYZ(legs[i].id, legs[i].restX, legs[i].walkY, legs[i].restZ - 3, 2000);
}

void sleepingLegs() {
  stopGait();
  queueClear();
  jFwd = jTurn = jStrafe = 0.0f;
  activeGait = 0;
  for (int i = 0; i < 1; i++)
    moveFootXYZ(legs[i].id, legs[i].restX, legs[i].walkY + 12, legs[i].restZ - 3, 2000);
}

// Normal movement — respects the currently selected gait (g0/g2/g3).
// Only jFwd / jTurn are set; activeGait is NOT overridden.
void walkForward()  { jFwd =  1.0f;  jTurn =  0.0f;  startGait(true); }
void walkBackward() { jFwd = -1.0f;  jTurn =  0.0f;  startGait(true); }
void turnLeft()     { jFwd =  0.66f; jTurn = -0.33f; startGait(true); }
void turnRight()    { jFwd =  0.66f; jTurn =  0.33f; startGait(true); }
void rotateLeft()   { jFwd =  0.0f;  jTurn = -1.0f;  startGait(true); }
void rotateRight()  { jFwd =  0.0f;  jTurn =  1.0f;  startGait(true); }

// Crab movement — unified gait 1.  jFwd drives Y, jStrafe drives X.
void crabForward()  { activeGait = 1; jFwd =  1.0f; jStrafe =  0.0f; jTurn = 0.0f; startGait(true); }
void crabBackward() { activeGait = 1; jFwd = -1.0f; jStrafe =  0.0f; jTurn = 0.0f; startGait(true); }
void crabLeft()     { activeGait = 1; jFwd =  0.0f; jStrafe =  1.0f; jTurn = 0.0f; startGait(true); }
void crabRight()    { activeGait = 1; jFwd =  0.0f; jStrafe = -1.0f; jTurn = 0.0f; startGait(true); }

// ───────────────────────────────────────────────────────────────
//  SECTION 7 — HEIGHT CONTROL
//  Updates both legs[] and crabLegs[] so height works in any mode.
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
//  SECTION 8 — INPUT PARSER + COMMAND HANDLER
// ───────────────────────────────────────────────────────────────

String inputBuffer = "";

void handleCommand(char cmd) {
  switch (cmd) {

    case 'F': case 'f':
      lastCmdName = "FORWARD";
      if (crabMode) { btLog(">> crabForward\n");  crabForward();  }
      else          { btLog(">> walkForward\n");   walkForward();  }
      break;

    case 'B': case 'b':
      lastCmdName = "BACKWARD";
      if (crabMode) { btLog(">> crabBackward\n"); crabBackward(); }
      else          { btLog(">> walkBackward\n");  walkBackward(); }
      break;

    case 'L': case 'l':
      lastCmdName = "LEFT";
      if (crabMode) { btLog(">> crabLeft\n");     crabLeft();     }
      else          { btLog(">> turnLeft\n");      turnLeft();     }
      break;

    case 'R': case 'r':
      lastCmdName = "RIGHT";
      if (crabMode) { btLog(">> crabRight\n");    crabRight();    }
      else          { btLog(">> turnRight\n");     turnRight();    }
      break;

    case 'W': case 'w':
      lastCmdName = "ROT LEFT";  btLog(">> rotateLeft\n");  rotateLeft();  break;

    case 'X': case 'x':
      lastCmdName = "ROT RIGHT"; btLog(">> rotateRight\n"); rotateRight(); break;

    case 'A': case 'a':
      lastCmdName = "SITTING";   btLog(">> sittingLegs\n"); sittingLegs(); break;

    case 'Z': case 'z':
      lastCmdName = "SLEEPING";  btLog(">> sleepingLegs\n");sleepingLegs();break;

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
      btLog("─── SpiderBot V2.8 ───\n");
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
        moveTime  = (uint16_t)map((long)(mag * 100), 0, 100, SPEED_MAX, SPEED_MIN);
        lastCmdName = "JOYSTICK";

        if (GAIT_TABLE[activeGait].indepXY) {
          // 2-D gait (crab or diagonal): jFwd drives Y, jStrafe drives X
          jFwd    = rawFwd;
          jStrafe = -rawTurn;     // sign flip: right stick = right strafe
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
      // g0…g3 are user-selectable.  g1 activates crab mode.
      uint8_t idx = (uint8_t)inputBuffer.substring(1).toInt();
      if (idx <= 3) {
        crabMode   = (idx == 1);
        activeGait = idx;
        jFwd = 1.0f; jTurn = 0.0f;
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
//  BT CALLBACK
// ───────────────────────────────────────────────────────────────
void btCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t* param) {
  if (event == ESP_SPP_SRV_OPEN_EVT) {
    btConnected = true;
    btLog("SpiderBot V2.8 Ready!\n");
    btLog("F/B=walk  L/R=turn  W/X=rotate  A=sit  Z=sleep\n");
    btLog("U/D=height  S=stop  +/-=speed  ?=status\n");
    btLog("g0=tripod  g1=crab  g2=diagonal  g3=wave\n");
    btLog("In crab mode: F/B=fwd-back  L/R=strafe\n");
    btLog("j66,33=arc left  j0,50=spin  j100,0=full fwd\n");
  }
  if (event == ESP_SPP_CLOSE_EVT) {
    btConnected = false;
    standStill();
    Serial.println("BT: Disconnected");
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
  Serial.println("SpiderBot V2.8 — pair your phone with: SpiderBot");
  delay(2000);
  standStill();
}


// ───────────────────────────────────────────────────────────────
//  LOOP
// ───────────────────────────────────────────────────────────────
void loop() {
  while (SerialBT.available()) parseInput((char)SerialBT.read());
  while (Serial.available())   parseInput((char)Serial.read());

  gaitTick();

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
}
