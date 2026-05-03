
/*===============================================================
 * SpiderBot V3.5
 * Author  : Amit Parihar
 * Wiring  : UART2  RX=16  TX=17  →  LSC-32 servo controller
 *           PS2 controller SPI:
 *             DAT=19 (MISO)  CMD=23 (MOSI)
 *             ATT=5  (SS)    CLK=18 (SCK)
 *
 * ── WHAT'S IN THIS FILE ──────────────────────────────────────
 *   1. Hardware + Constants + Leg Data
 *   2. IK Solver       — foot XYZ → servo pulses
 *   3. Smooth Gait Engine — continuous phase accumulation
 *       (port of originalHexapodScript.lua — replaces V3.4
 *        discrete SEND_A/PLANT_A state machine)
 *   4. Posture Functions — standStill, sit, sleep, hello
 *   5. Height Control
 *   6. BT Input Parser — BT + Serial command handler
 *   7. PS2 Controller  — primary physical controller
 *   8. BT Callback
 *   9. OTA Setup       — wireless firmware updates
 *  10. CoppeliaSim Bridge — streams joint angles when dirty
 *  11. setup() + loop()
 *
 * ── WHAT'S NEW IN V3.5 ───────────────────────────────────────
 *   • SMOOTH GAIT ENGINE — replaces the old discrete 4-phase
 *     state machine (SEND_A→PLANT_A→SEND_B→PLANT_B→RECENTER).
 *     Now every leg is updated every ~20ms using continuous
 *     phase accumulation and a smooth 4-segment foot arc,
 *     a direct port of the CoppeliaSim originalHexapodScript.lua.
 *
 *   Key improvements over V3.4:
 *     - Update rate:  20ms (50Hz)  vs  500ms (2Hz) per phase
 *     - Foot path:    smooth 4-segment arc vs 2-point jump
 *     - Direction:    instant on next tick vs wait-for-phase
 *     - Speed change: ease-in/out via realStrength smoothing
 *     - Leg sequence: natural ripple {FL,RL,MR,ML,FR,RR}
 *       vs strict A/B tripod groups
 *     - Stance:Swing ratio 2:1 (optimal) vs ~1:1
 *
 *   All other V3.4 features kept intact:
 *     OTA, BT debug, PS2 control, CoppeliaSim bridge,
 *     height control, crab mode, sit/sleep/hello postures.
 *
 * ── BUTTON MAP ───────────────────────────────────────────────
 *   Left  stick  → walk + turn (jFwd / jTurn)
 *   Right stick  → strafe / crab (jStrafe)
 *   D-pad        → discrete walk / strafe
 *   ○ / □        → rotate left / right (held)
 *   △ / ×        → height up / down
 *   L1 / R1      → sit / sleep
 *   L2 / R2      → speed down / up
 *   START        → standStill
 *   SELECT       → (reserved — crab toggle in V3.5)
 *   L3           → toggle crab mode
 *   R3           → master ON/OFF
 *===============================================================*/

#include <Arduino.h>
#include <math.h>
#include "BluetoothSerial.h"
#include <LobotServoController.h>
#include <PS2X_lib.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
  #error "Bluetooth not enabled — Arduino IDE: Tools → ESP32 Bluetooth"
#endif

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

const char* WIFI_SSID     = "A.'s Galaxy A52";
const char* WIFI_PASSWORD = "amit1234";


// ───────────────────────────────────────────────────────────────
//  SECTION 1 — HARDWARE, CONSTANTS, LEG DATA
// ───────────────────────────────────────────────────────────────

HardwareSerial       LSC(2);
BluetoothSerial      SerialBT;
LobotServoController controller(LSC);
PS2X                 ps2x;

#define PS2_DAT  19
#define PS2_CMD  23
#define PS2_SEL   5
#define PS2_CLK  18

const int PS2_DEADZONE = 20;

// ── Leg geometry (cm) ─────────────────────────────────────────
const float COXA   = 6.0f;
const float FEMUR  = 8.5f;
const float TIBIA  = 14.5f;

// ── Servo pulse constants ──────────────────────────────────────
const int   SERVO_MID = 1500;
const float DEG_TO_US = 11.11f;

// ── Runtime-tunable parameters ────────────────────────────────
// moveTime is kept for posture transitions (sit, sleep, standStill).
// The smooth gait engine uses its own 30ms servo target instead.
uint16_t moveTime = 500;
float    bodyZ    = -9.0f;

const uint16_t SPEED_MIN  = 150;
const uint16_t SPEED_MAX  = 1000;
const uint16_t SPEED_STEP = 50;

const float HEIGHT_STEP = 1.0f;
const float HEIGHT_LOW  = -7.0f;    // shortest stance (femur ~62° up, crouched)
const float HEIGHT_HIGH = -14.0f;   // tallest stance  (femur nearly horizontal)

// ── Status ─────────────────────────────────────────────────────
bool          btConnected        = false;
float         voltageInVolts     = 0.0f;
unsigned long lastVoltageRequest = 0;
const char*   lastCmdName        = "NONE";
bool          crabMode           = false;
bool          isBotOn            = true;

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

// ── CoppeliaSim bridge buffers ─────────────────────────────────
float currentJointAngles[32];
bool  anglesDirty = false;


// ───────────────────────────────────────────────────────────────
//  SECTION 2 — IK SOLVER
//  Unchanged from V3.4. Receives a 3-D foot position and outputs
//  servo pulse widths to the LSC-32 controller.
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

  // perpTibia: override tibia to always point straight down
  if (perpTibia) tDeg = 90.0f + fDeg;

  uint16_t p1 = constrain((int)(SERVO_MID +  cDeg  * DEG_TO_US), 500, 2500);
  uint16_t p2 = constrain((int)(SERVO_MID -  fDeg  * DEG_TO_US), 500, 2500);  // 1500=horizontal, 500=up, 2500=down
  uint16_t p3 = constrain((int)(SERVO_MID - (tDeg - 90.0f) * DEG_TO_US), 500, 2500);  // 1500=straight down

  currentJointAngles[id]   = cDeg;
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
//  SECTION 3 — SMOOTH GAIT ENGINE
//
//  Port of originalHexapodScript.lua sysCall_actuation().
//
//  Architecture:
//    • stepProgression  — float that advances by dt*gaitVel every
//      loop() call. Drives all 6 leg phases simultaneously.
//    • Each leg has a fixed phase OFFSET so they stagger naturally.
//    • Each leg's foot traces a 4-segment path every full cycle:
//        DRAG 1 (1/3) → SWING UP (1/6) → SWING DOWN (1/6)
//                                       → DRAG 2 (1/3)
//    • realStrength smoothly converges to targetStrength →
//      natural ease-in/ease-out on start and stop.
//    • Joystick values (jFwd, jTurn, jStrafe) are applied every
//      tick → instant direction response.
// ───────────────────────────────────────────────────────────────

// Joystick inputs (shared with BT parser and PS2 handler)
float jFwd    = 0.0f;
float jTurn   = 0.0f;
float jStrafe = 0.0f;

// legMovementIndex from Lua: {1,4,2,6,3,5}
// Phase offset = (index - 1) / 6
//   FL → 0/6=0.000   RL → 1/6=0.167   MR → 2/6=0.333
//   ML → 3/6=0.500   RR → 4/6=0.667   FR → 5/6=0.833
const float LEG_PHASE_OFFSET[6] = {
  0.000f,  // 0  Front-Left    (index 1)
  0.500f,  // 1  Mid-Left      (index 4)
  0.167f,  // 2  Rear-Left     (index 2)
  0.833f,  // 3  Front-Right   (index 6)
  0.333f,  // 4  Mid-Right     (index 3)
  0.667f,  // 5  Rear-Right    (index 5)
};

// Gait state
float         stepProgression  = 0.0f;
float         realStrength     = 0.0f;  // Smoothed — eases toward targetStrength
float         targetStrength   = 0.0f;  // Set by startSmooth() / stopSmooth()
bool          gaitRunning      = false;
unsigned long lastSmoothTick   = 0;

// Gait parameters (tuned per movement command)
float gaitVel        = 0.9f;   // Phase speed  (Lua: movData.vel)
float gaitAmplitude  = 11.0f;  // Stride half-length cm  (Lua: 0.11 m)
float gaitLiftHeight = 2.5f;   // Foot lift height cm    (Lua: 0.02 m, increased for clearance)
float gaitDir        = 0.0f;   // Walk direction radians (Lua: movData.dir)
float gaitRot        = 0.0f;   // Rotation mode: 0=straight ±1=spin (Lua: movData.rot)

// Servo move time for each individual foot command inside the engine.
// Rule: SMOOTH_T must be larger than the actual tick interval so the servo
// controller always has a future target to smoothly track.
// 80ms gives the physical motors enough time to actually follow the arc
// without snapping or aborting a move mid-way.
const uint16_t SMOOTH_T = 80;


// Call with desired movement strength (0=off, 1=full)
void startSmooth(float strength = 1.0f) {
  gaitRunning    = true;
  targetStrength = strength;
  lastSmoothTick = millis();
}

void stopSmooth() {
  targetStrength = 0.0f;
  // gaitRunning stays true while realStrength eases down — tick handles the exit.
}

// Hard-stop: kills gait immediately. Use for postures (standStill/sit/sleep)
// so the tick doesn't override the posture command on the very next loop().
void killGait() {
  targetStrength = 0.0f;
  realStrength   = 0.0f;
  gaitRunning    = false;
}

// Mirror of setStepMode() in Lua
void setStepMode(float vel, float amplitude, float height,
                 float dir_deg, float rot, float strength) {
  gaitVel        = vel;
  gaitAmplitude  = amplitude;
  gaitLiftHeight = height;
  gaitDir        = dir_deg * (float)DEG_TO_RAD;
  gaitRot        = rot;
  targetStrength = strength;
  if (strength > 0.0f) gaitRunning = true;
}

void smoothGaitTick() {
  if (!gaitRunning) return;

  unsigned long now = millis();
  float dt = (now - lastSmoothTick) / 1000.0f;
  if (dt < 0.015f) return;          // Cap at ~50 Hz
  if (dt > 0.10f)  dt = 0.10f;     // Clamp against stalls
  lastSmoothTick = now;

  // ── Ease-in / ease-out on strength ─────────────────────────
  // Same algorithm as Lua sysCall_actuation():
  //   if error is large → proportional step capped at dt*0.5
  //   if error is small → snap the remaining difference
  float dx = targetStrength - realStrength;
  if (fabsf(dx) > dt * 0.1f) {
    dx = fabsf(dx) * dt * 0.5f / dx;
  }
  realStrength += dx;

  // If we've fully stopped, flag gait as idle
  if (fabsf(realStrength) < 0.005f && fabsf(targetStrength) < 0.001f) {
    realStrength = 0.0f;
    gaitRunning  = false;
    return;
  }

  // ── Compute foot position for all 6 legs ───────────────────
  for (int i = 0; i < 6; i++) {
    Leg& leg = crabMode ? crabLegs[i] : legs[i];

    // Phase for this leg (staggered)
    float sp = fmodf(stepProgression + LEG_PHASE_OFFSET[i], 1.0f);

    float offStride = 0.0f;   // Along stride direction
    float offLift   = 0.0f;   // Vertical (we subtract from restZ below)

    // ── 4-segment foot path ───────────────────────────────────
    if (sp < (1.0f / 3.0f)) {
      // DRAG 1: foot on ground, slides from 0 → +amplitude/2
      offStride = sp * 3.0f * gaitAmplitude / 2.0f;

    } else if (sp < (1.0f/3.0f + 1.0f/6.0f)) {
      // SWING RISE: lifts off, arcs toward apex
      float s   = sp - 1.0f/3.0f;
      offStride = gaitAmplitude/2.0f - gaitAmplitude * s * 6.0f / 2.0f;
      offLift   = s * 6.0f * gaitLiftHeight;

    } else if (sp < (2.0f / 3.0f)) {
      // SWING FALL: descends from apex to landing zone
      float s   = sp - 1.0f/3.0f - 1.0f/6.0f;
      offStride = -gaitAmplitude * s * 6.0f / 2.0f;
      offLift   = (1.0f - s * 6.0f) * gaitLiftHeight;

    } else {
      // DRAG 2: foot on ground, slides from -amplitude/2 → 0
      float s   = sp - 2.0f/3.0f;
      offStride = -gaitAmplitude * (1.0f - s * 3.0f) / 2.0f;
    }

    // ── Joystick blending: forward + turn (differential drive) ─
    // coxaDir (+1 left / -1 right) ensures both sides push the
    // body in the same physical direction.
    float scale = constrain(jFwd + jTurn * leg.coxaDir, -1.0f, 1.0f);

    // Crab / strafe blending via X component
    float strafeComp = 0.0f;
    if (fabsf(jStrafe) > 0.01f) {
      strafeComp = offStride * constrain(jStrafe, -1.0f, 1.0f) * realStrength;
    }

    float footX = leg.restX + strafeComp;
    float footY = leg.walkY + offStride * leg.coxaDir * scale    * realStrength;
    // Arduino Z convention (fixed): more positive = closer to body (UP) → add offLift
    float footZ = leg.restZ + offLift                            * realStrength;

    moveFootXYZ(leg.id, footX, footY, footZ, SMOOTH_T);
  }

  // ── Advance global phase ────────────────────────────────────
  stepProgression = fmodf(stepProgression + dt * gaitVel, 1.0f);
}


// ───────────────────────────────────────────────────────────────
//  SECTION 4 — POSTURE FUNCTIONS
//  These call moveFootXYZ directly with a larger moveTime for
//  smooth transitions into special poses. The smooth gait engine
//  is stopped before any posture call.
// ───────────────────────────────────────────────────────────────

void standStill() {
  killGait();  // Hard-stop: prevents tick from overriding the home pose
  jFwd = jTurn = jStrafe = 0.0f;
  // perpTibia=true → tibia always perpendicular to ground while standing
  for (int i = 0; i < 6; i++) {
    Leg& leg = crabMode ? crabLegs[i] : legs[i];
    moveFootXYZ(leg.id, leg.restX, leg.rotY, leg.restZ, moveTime, true);
  }
}

void sittingLegs() {
  killGait();  // Hard-stop to prevent tick override
  jFwd = jTurn = jStrafe = 0.0f;
  for (int i = 0; i < 6; i++)
    moveFootXYZ(legs[i].id, legs[i].restX, legs[i].walkY, legs[i].restZ - 3, 2000);
}

void sleepingLegs() {
  killGait();  // Hard-stop to prevent tick override
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
  extern uint8_t ps2Warm;
  ps2Warm = 0;
}

void logMove(const char* direction) {
  char buf[64];
  snprintf(buf, sizeof(buf), ">> Smooth Gait: %s\n", direction);
  btLog(buf);
}

// ── Shorthand helpers — same names as V3.4 for backward compat ─
void walkForward()  { jFwd =  1.0f;  jTurn =  0.0f; jStrafe =  0.0f; startSmooth(); }
void walkBackward() { jFwd = -1.0f;  jTurn =  0.0f; jStrafe =  0.0f; startSmooth(); }
void turnLeft()     { jFwd =  0.66f; jTurn = -0.33f; jStrafe = 0.0f; startSmooth(); }
void turnRight()    { jFwd =  0.66f; jTurn =  0.33f; jStrafe = 0.0f; startSmooth(); }
void rotateLeft()   { jFwd =  0.0f;  jTurn = -1.0f;  jStrafe = 0.0f; startSmooth(); }
void rotateRight()  { jFwd =  0.0f;  jTurn =  1.0f;  jStrafe = 0.0f; startSmooth(); }
void crabLeft()     { crabMode = true; jFwd = 0.0f; jStrafe =  1.0f; jTurn = 0.0f; startSmooth(); }
void crabRight()    { crabMode = true; jFwd = 0.0f; jStrafe = -1.0f; jTurn = 0.0f; startSmooth(); }
void crabForward()  { crabMode = true; jFwd = 1.0f; jStrafe =  0.0f; jTurn = 0.0f; startSmooth(); }
void crabBackward() { crabMode = true; jFwd =-1.0f; jStrafe =  0.0f; jTurn = 0.0f; startSmooth(); }


// ───────────────────────────────────────────────────────────────
//  SECTION 5 — HEIGHT CONTROL
//
//  Height changes only adjust restZ. restX stays constant.
//  The perpTibia flag in moveFootXYZ keeps the tibia straight down.
// ───────────────────────────────────────────────────────────────

void adjustHeight(float delta) {
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
  standStill();
  char buf[32]; snprintf(buf, sizeof(buf), ">> Height: %.1f cm\n", bodyZ); btLog(buf);
}

void increaseHeight() { adjustHeight(+HEIGHT_STEP); }
void decreaseHeight() { adjustHeight(-HEIGHT_STEP); }


// ───────────────────────────────────────────────────────────────
//  SECTION 6 — BT INPUT PARSER
// ───────────────────────────────────────────────────────────────

String inputBuffer = "";

void handleCommand(char cmd) {
  switch (cmd) {
    case 'F': case 'f': lastCmdName="FORWARD";  logMove("Forward");      walkForward();  break;
    case 'B': case 'b': lastCmdName="BACKWARD"; logMove("Backward");     walkBackward(); break;
    case 'L': case 'l':
      lastCmdName="LEFT";
      if (crabMode) { logMove("Strafe Left");  crabLeft();  }
      else          { logMove("Turn Left");    turnLeft();  }
      break;
    case 'R': case 'r':
      lastCmdName="RIGHT";
      if (crabMode) { logMove("Strafe Right"); crabRight(); }
      else          { logMove("Turn Right");   turnRight(); }
      break;
    case 'W': case 'w': lastCmdName="ROT LEFT";  logMove("Rotate Left");  rotateLeft();  break;
    case 'X': case 'x': lastCmdName="ROT RIGHT"; logMove("Rotate Right"); rotateRight(); break;
    case 'A': case 'a': lastCmdName="SITTING";  btLog(">> Sitting\n");  sittingLegs();  break;
    case 'Z': case 'z': lastCmdName="SLEEPING"; btLog(">> Sleeping\n"); sleepingLegs(); break;
    case 'H': case 'h': lastCmdName="SAY HI";   btLog(">> Hello\n");    helloLegs();    break;
    case 'U': case 'u': lastCmdName="HEIGHT UP"; increaseHeight(); break;
    case 'D': case 'd': lastCmdName="HEIGHT DN"; decreaseHeight(); break;
    case 'S': case 's': lastCmdName="STOP"; btLog(">> standStill\n"); standStill(); break;

    case '+':
      if (moveTime > SPEED_MIN) moveTime -= SPEED_STEP;
      gaitVel = constrain(gaitVel + 0.1f, 0.3f, 2.0f);
      { char b[40]; snprintf(b, sizeof(b), ">> Speed: %u ms  gaitVel: %.1f\n", moveTime, gaitVel); btLog(b); }
      break;

    case '-':
      if (moveTime < SPEED_MAX) moveTime += SPEED_STEP;
      gaitVel = constrain(gaitVel - 0.1f, 0.3f, 2.0f);
      { char b[40]; snprintf(b, sizeof(b), ">> Speed: %u ms  gaitVel: %.1f\n", moveTime, gaitVel); btLog(b); }
      break;

    case '?':
      btLog("─── SpiderBot V3.5 ───\n");
      btLog("Engine   : Smooth Continuous Gait\n");
      btLog("Last cmd : "); btLog(lastCmdName);       btLog("\n");
      btLog("Crab mode: "); btLog(crabMode?"ON\n":"OFF\n");
      btLog("gaitVel  : "); btLog(gaitVel);            btLog("\n");
      btLog("strength : "); btLog(realStrength);        btLog(" → "); btLog(targetStrength); btLog("\n");
      btLog("Height   : "); btLog(bodyZ);               btLog(" cm\n");
      btLog("Battery  : "); btLog(voltageInVolts);      btLog(" V\n");
      btLog("jFwd     : "); btLog(jFwd);
      btLog("  jTurn  : "); btLog(jTurn);
      btLog("  jStrafe: "); btLog(jStrafe);             btLog("\n");
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
        float mag     = constrain(sqrt(rawFwd*rawFwd + rawTurn*rawTurn), 0.0f, 1.0f);
        // Map magnitude to gaitVel (fast analog control)
        gaitVel     = constrain(mag * 1.5f, 0.3f, 1.5f);
        lastCmdName = "JOYSTICK";
        jFwd    = rawFwd;
        jTurn   = rawTurn;
        jStrafe = 0.0f;
        if (fabsf(rawFwd) < 0.05f && fabsf(rawTurn) < 0.05f) standStill();
        else startSmooth();
      }
    }
    inputBuffer = "";
  } else {
    if (inputBuffer.length() == 0) handleCommand(c);
    else                           inputBuffer += c;
  }
}


// ───────────────────────────────────────────────────────────────
//  SECTION 7 — PS2 CONTROLLER
// ───────────────────────────────────────────────────────────────

float ps2Axis(byte raw, bool invert) {
  int centred = (int)raw - 128;
  if (abs(centred) < PS2_DEADZONE) return 0.0f;
  float norm = constrain(centred / 127.0f, -1.0f, 1.0f);
  return invert ? -norm : norm;
}

bool    ps2Moving = false;
uint8_t ps2Warm   = 0;
const uint8_t    PS2_WARMUP_FRAMES = 10;
const  uint16_t  PS2_POLL_MS       = 50;   // 20 Hz rate-limit
unsigned long    lastPS2Read       = 0;

void readPS2() {
  unsigned long now = millis();
  if (now - lastPS2Read < PS2_POLL_MS) return;
  lastPS2Read = now;

  ps2x.read_gamepad(false, 0);

  // Detect disconnected controller (axis=255 on both sticks)
  byte rawLY = ps2x.Analog(PSS_LY);
  byte rawRY = ps2x.Analog(PSS_RY);
  if (rawLY == 255 && rawRY == 255) { ps2Warm = 0; return; }

  // ── R3: Master ON/OFF ──────────────────────────────────────
  if (ps2x.ButtonPressed(PSB_R3)) {
    isBotOn = !isBotOn;
    if (isBotOn) { btLog(">> BOT ON\n");  standStill(); }
    else         { btLog(">> BOT OFF\n"); sleepingLegs(); }
  }
  if (!isBotOn) return;

  // ── Warmup ─────────────────────────────────────────────────
  if (ps2Warm < PS2_WARMUP_FRAMES) { ps2Warm++; return; }

  // ── Read sticks ────────────────────────────────────────────
  float ly = ps2Axis(ps2x.Analog(PSS_LY), true);
  float lx = ps2Axis(ps2x.Analog(PSS_LX), false);
  float rx = ps2Axis(ps2x.Analog(PSS_RX), true);
  bool  stickActive = (fabsf(ly) > 0.0f || fabsf(lx) > 0.0f || fabsf(rx) > 0.0f);

  // ── D-pad + face buttons (movement held) ──────────────────
  bool dpadActive = false;

  if (ps2x.Button(PSB_PAD_UP)) {
    dpadActive = true;
    if (!ps2Moving) { logMove("Forward"); lastCmdName = "FORWARD"; }
    jFwd = 1.0f; jTurn = 0.0f; jStrafe = 0.0f;
    if (!gaitRunning) startSmooth();
  }
  else if (ps2x.Button(PSB_PAD_DOWN)) {
    dpadActive = true;
    if (!ps2Moving) { logMove("Backward"); lastCmdName = "BACKWARD"; }
    jFwd = -1.0f; jTurn = 0.0f; jStrafe = 0.0f;
    if (!gaitRunning) startSmooth();
  }
  else if (ps2x.Button(PSB_PAD_LEFT)) {
    dpadActive = true;
    if (!ps2Moving) { lastCmdName = "LEFT"; logMove(crabMode ? "Strafe L" : "Turn L"); }
    if (crabMode) { jFwd = 0.0f; jStrafe = 1.0f; jTurn = 0.0f; }
    else          { jFwd = 0.66f; jTurn = -0.33f; jStrafe = 0.0f; }
    if (!gaitRunning) startSmooth();
  }
  else if (ps2x.Button(PSB_PAD_RIGHT)) {
    dpadActive = true;
    if (!ps2Moving) { lastCmdName = "RIGHT"; logMove(crabMode ? "Strafe R" : "Turn R"); }
    if (crabMode) { jFwd = 0.0f; jStrafe = -1.0f; jTurn = 0.0f; }
    else          { jFwd = 0.66f; jTurn = 0.33f; jStrafe = 0.0f; }
    if (!gaitRunning) startSmooth();
  }
  else if (ps2x.Button(PSB_CIRCLE)) {
    dpadActive = true;
    if (!ps2Moving) { logMove("Rotate Left"); lastCmdName = "ROT LEFT"; }
    jFwd = 0.0f; jTurn = -1.0f; jStrafe = 0.0f;
    if (!gaitRunning) startSmooth();
  }
  else if (ps2x.Button(PSB_SQUARE)) {
    dpadActive = true;
    if (!ps2Moving) { logMove("Rotate Right"); lastCmdName = "ROT RIGHT"; }
    jFwd = 0.0f; jTurn = 1.0f; jStrafe = 0.0f;
    if (!gaitRunning) startSmooth();
  }

  // ── Analog sticks ──────────────────────────────────────────
  if (stickActive && !dpadActive) {
    float mag = constrain(sqrt(ly*ly + lx*lx + rx*rx), 0.0f, 1.0f);
    gaitVel   = constrain(mag * 1.5f, 0.3f, 1.5f);  // Analog → gait speed

    if (crabMode) { jFwd = ly; jStrafe = rx; jTurn = 0.0f; }
    else          { jFwd = ly; jTurn   = lx; jStrafe = 0.0f; }

    if (!gaitRunning) startSmooth();
  }

  // ── Stop when all movement released ────────────────────────
  bool anyMovement = dpadActive || stickActive;
  if (!anyMovement && ps2Moving) {
    btLog(">> PS2: Stop\n");
    standStill();
  }
  ps2Moving = anyMovement;

  // ── One-shot non-movement buttons ──────────────────────────
  if (ps2x.ButtonPressed(PSB_TRIANGLE)) { btLog(">> PS2: Height Up\n");   increaseHeight(); }
  if (ps2x.ButtonPressed(PSB_CROSS))    { btLog(">> PS2: Height Down\n"); decreaseHeight(); }

  if (ps2x.ButtonPressed(PSB_L1)) { btLog(">> PS2: Sitting\n");   lastCmdName="SITTING";  sittingLegs();  }
  if (ps2x.ButtonPressed(PSB_R1)) { btLog(">> PS2: Sleeping\n");  lastCmdName="SLEEPING"; sleepingLegs(); }
  if (ps2x.ButtonPressed(PSB_START)) { btLog(">> PS2: Stand\n");  lastCmdName="STOP";     standStill();   }

  // ── L3: toggle crab mode ───────────────────────────────────
  if (ps2x.ButtonPressed(PSB_L3)) {
    crabMode = !crabMode;
    jFwd = jTurn = jStrafe = 0.0f;
    btLog(crabMode ? ">> PS2: Crab ON\n" : ">> PS2: Crab OFF\n");
  }

  // ── L2/R2: gait speed ─────────────────────────────────────
  if (ps2x.ButtonPressed(PSB_L2)) {
    gaitVel = constrain(gaitVel - 0.1f, 0.3f, 2.0f);
    char b[36]; snprintf(b, sizeof(b), ">> PS2: Vel %.1f\n", gaitVel); btLog(b);
  }
  if (ps2x.ButtonPressed(PSB_R2)) {
    gaitVel = constrain(gaitVel + 0.1f, 0.3f, 2.0f);
    char b[36]; snprintf(b, sizeof(b), ">> PS2: Vel %.1f\n", gaitVel); btLog(b);
  }
}


// ───────────────────────────────────────────────────────────────
//  SECTION 8 — BT CALLBACK
// ───────────────────────────────────────────────────────────────
void btCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t* param) {
  if (event == ESP_SPP_SRV_OPEN_EVT) {
    btConnected = true;
    btLog("SpiderBot V3.5 — Smooth Gait Engine Ready!\n");
    btLog("F/B/L/R/W/X/A/Z/U/D/S/+/-/?  jFWD,TURN\n");
  }
  if (event == ESP_SPP_CLOSE_EVT) {
    btConnected = false;
    standStill();
    Serial.println("BT: Disconnected");
  }
}


// ───────────────────────────────────────────────────────────────
//  SECTION 9 — OTA SETUP
// ───────────────────────────────────────────────────────────────
void setupOTA() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi for OTA");

  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 5000) {
    delay(500); Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWi-Fi Connected! IP: "); Serial.println(WiFi.localIP());
    ArduinoOTA.setHostname("spiderbot");
    ArduinoOTA.onStart([]() {
      String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
      Serial.println("\nOTA start: " + type);
      standStill();
    });
    ArduinoOTA.onEnd([]()     { Serial.println("\nOTA end"); });
    ArduinoOTA.onProgress([](unsigned int p, unsigned int t) {
      Serial.printf("OTA: %u%%\r", (p / (t / 100)));
    });
    ArduinoOTA.onError([](ota_error_t err) {
      Serial.printf("OTA Error[%u]\n", err);
    });
    ArduinoOTA.begin();
    Serial.println("OTA Ready.");
  } else {
    Serial.println("\nWi-Fi timeout. Offline mode.");
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
  }
}


// ───────────────────────────────────────────────────────────────
//  SECTION 10 — COPPELIASIM BRIDGE
//  Streams joint angles over USB Serial whenever any foot moved.
//  The pythonBridgeScript.py reads and applies them to the
//  virtual robot's joints.
// ───────────────────────────────────────────────────────────────
void sendAnglesToPC() {
  for (int i = 0; i < 6; i++) {
    uint8_t id = legs[i].id;
    Serial.print(currentJointAngles[id],   2); Serial.print(",");
    Serial.print(currentJointAngles[id+1], 2); Serial.print(",");
    Serial.print(currentJointAngles[id+2], 2);
    if (i < 5) Serial.print(",");
  }
  Serial.println();
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
  Serial.println("SpiderBot V3.5 — Smooth Gait Engine");

  setupOTA();

  pinMode(PS2_DAT, INPUT_PULLUP);
  int ps2error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, false, false);
  Serial.println(ps2error == 0 ? "PS2 SPI OK." : "PS2 SPI Failed.");

  delay(500);
  standStill();
  lastSmoothTick = millis();
}


// ───────────────────────────────────────────────────────────────
//  LOOP
// ───────────────────────────────────────────────────────────────
void loop() {
  // 1. OTA wireless updates (non-blocking check)
  ArduinoOTA.handle();

  // 2. BT / Serial debug commands
  while (SerialBT.available()) parseInput((char)SerialBT.read());
  while (Serial.available())   parseInput((char)Serial.read());

  // 3. PS2 primary controller
  readPS2();

  // 4. Smooth gait engine — runs every ~20ms, updates all 6 legs
  smoothGaitTick();

  // 5. Battery voltage polling (every 5 s)
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

  // 6. CoppeliaSim bridge — send angles only when a foot moved
  if (anglesDirty) {
    sendAnglesToPC();
    anglesDirty = false;
  }
}
