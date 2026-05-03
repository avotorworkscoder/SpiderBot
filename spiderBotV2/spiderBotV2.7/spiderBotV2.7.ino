/*===============================================================
 * SpiderBot V2.7
 * Author  : Amit Parihar
 * Wiring  : UART2  RX=16  TX=17  →  LSC-32 servo controller
 *
 * ── WHAT'S IN THIS FILE ──────────────────────────────────────
 *   1. Hardware + Constants + Leg Data
 *   2. IK Solver       — foot XYZ → servo pulses
 *   3. Gait Shapes     — plug-and-play stride definitions (2-D)
 *   4. Command Queue   — circular buffer, never drops a command
 *   5. Gait Engine     — millis()-driven, never blocks loop()
 *   6. Movement Funcs  — walk, turn, rotate, crab directions
 *   7. Height Control  — raise / lower body at runtime
 *   8. Input Parser    — BT + Serial command handler
 *   9. setup() + loop()
 *
 * ── WHAT'S NEW IN V2.7 vs V2.6 ──────────────────────────────
 *   - Crab mode with a dedicated leg geometry (crabLegs[]).
 *     The crab stance rotates legs to face sideways, so the
 *     bot walks like a crab instead of a spider.
 *   - 2-D gait engine: every gait now defines swingX/dragX in
 *     addition to swingY/dragY.  Normal walk uses X=restX
 *     (unchanged), crab left/right drives the X axis instead.
 *   - sendPhase, plantGroup, recenterLegs all switch between
 *     legs[] and crabLegs[] depending on crabMode.
 *   - adjustHeight() updates both leg arrays simultaneously so
 *     height changes work in any mode.
 *   - g1 activates crab mode;  g0/g2/g3 deactivate it.
 *     In crab mode:  F/B = forward/backward,  L/R = strafe.
 *   - g2 activates diagonal walk mode; g0/g1/g3 deactivate it.
 *     In diagonal walk mode, the bot walks diagonally instead of straight.
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
 *   g2             Diagonal Walk (crab mode OFF)
 *   g3             Wave Gait     (crab mode OFF)
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
//  V2.7 extends each gait with X-axis functions (swingX / dragX).
//  Normal gaits return restX unchanged — only crab left/right
//  drive the X axis.  swingY / dragY work exactly as in V2.6.
//
//  GaitDef now holds four function pointers:
//    swingX(i), dragX(i)  — X target for swing / drag
//    swingY(i), dragY(i)  — Y target for swing / drag
//
//  The engine resolves the correct leg struct (legs[] or
//  crabLegs[]) at runtime based on crabMode.
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

// ── Gait 3 — Wave Gait ────────────────────────────────────────
float waveSwingY(int i){
  float extra = (i % 3) * 0.5f;
  return legs[i].walkY - (2.0f + extra) * legs[i].coxaDir * legs[i].strideScale;
}
float waveDragY(int i){
  float extra = (i % 3) * 0.5f;
  return legs[i].walkY + (2.0f + extra) * legs[i].coxaDir * legs[i].strideScale;
}

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
//  SECTION 5 — NON-BLOCKING 2-D GAIT ENGINE
//
//  Same five-phase structure as V2.6 (SEND_A → PLANT_A →
//  SEND_B → PLANT_B → RECENTER).
//
//  V2.7 additions:
//  • sendPhase / plantGroup resolve the active leg struct with:
//      Leg& leg = crabMode ? crabLegs[idx] : legs[idx];
//  • Normal mode: single finalScale = jFwd + jTurn×coxaDir
//    drives both X and Y identically (differential walk+turn).
//  • Crab mode: X and Y have independent scales:
//      scaleY = jFwd          (forward / backward)
//      scaleX = jStrafe       (strafe left / right)
//    This enables true omnidirectional crab movement.
//  • recenterLegs() homes to the correct walkY for each mode.
// ───────────────────────────────────────────────────────────────

float jFwd    = 0.0f;
float jTurn   = 0.0f;
float jStrafe = 0.0f;   // independent X-axis scale (used when gait has indepXY)

enum GaitPhase { IDLE, SEND_A, PLANT_A, SEND_B, PLANT_B, RECENTER };
GaitPhase     gaitPhase  = IDLE;
bool          looping    = false;
unsigned long phaseStart = 0;

void handleCommand(char cmd);  // forward declaration

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

void startGait(bool continuous) {
  looping    = continuous;
  gaitPhase  = SEND_A;
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
//    F   = crab forward  (gait 1)
//    B   = crab backward (gait 4)
//    L   = strafe left   (gait 5)
//    R   = strafe right  (gait 6)
//    W/X = rotate (unchanged — always uses tripod differential)
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

// Normal tripod movement
void walkForward()  { activeGait = 0; jFwd =  1.0f;  jTurn =  0.0f;  startGait(true); }
void walkBackward() { activeGait = 0; jFwd = -1.0f;  jTurn =  0.0f;  startGait(true); }
void turnLeft()     { activeGait = 0; jFwd =  0.66f; jTurn = -0.33f; startGait(true); }
void turnRight()    { activeGait = 0; jFwd =  0.66f; jTurn =  0.33f; startGait(true); }
void rotateLeft()   { activeGait = 0; jFwd =  0.0f;  jTurn = -1.0f;  startGait(true); }
void rotateRight()  { activeGait = 0; jFwd =  0.0f;  jTurn =  1.0f;  startGait(true); }

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
      btLog("─── SpiderBot V2.7 ───\n");
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
    btLog("SpiderBot V2.7 Ready!\n");
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
  Serial.println("SpiderBot V2.7 — pair your phone with: SpiderBot");
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
