/*===============================================================
 * SpiderBot V2.6
 * Author  : Amit Parihar
 * Wiring  : UART2  RX=16  TX=17  →  LSC-32 servo controller
 *
 * HOW THIS FILE IS ORGANISED (read top to bottom):
 *   1. Hardware + Constants + Leg Data
 *   2. IK Solver   — moves one foot to an XYZ position
 *   3. Gait Shapes — define WHERE each foot goes per gait style
 *   4. Command Queue — stores incoming commands so none are missed
 *   5. Gait Engine — runs gait phases using millis(), no delay()
 *   6. Movement Functions — walkForward, turnLeft, etc.
 *   7. Height Control
 *   8. Input Parser + Command Handler
 *   9. setup() + loop()
 *
 * NEW IN V2.6 vs V2.5:
 *   - No delay() during movement. Each step is split into timed
 *     phases checked by millis() every loop.
 *   - Command queue: commands sent mid-step are saved and run next.
 *   - Joystick: send "j80,30" = 80% forward + 30% left turn blend.
 *   - Plug-and-play gaits: add a row to GAIT_TABLE[] to add a gait.
 *     Change activeGait = 0/1/2/3 to switch walking style instantly.
 *
 * BT APP (Serial Bluetooth Terminal):
 *   Settings → Line ending → Newline (\n)
 *   Macros: F=fwd  B=back  L=turnL  R=turnR
 *           W=rotL  X=rotR  U=up  D=down
 *           S=stop  ?=status  +=faster  -=slower
 *           g0=tripod  g1=crab  g2=diagonal  g3=wave
 *
 * COMMANDS:
 *   F / B         walk forward / backward
 *   L / R         arc turn left / right
 *   W / X         rotate left / right (spin in place)
 *   U / D         raise / lower body height
 *   S or H        stand still (stop everything)
 *   + / -         speed up / slow down
 *   g0 .. g3      switch gait style  (send then press Enter)
 *   j80,30        joystick blend: 80% fwd + 30% left turn
 *   ?             status report
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

// Hardware objects
HardwareSerial       LSC(2);           // UART2 → LSC-32 (RX=16, TX=17)
BluetoothSerial      SerialBT;         // ESP32 built-in Bluetooth
LobotServoController controller(LSC); // reads battery voltage from LSC-32

// Leg geometry (cm)
const float COXA   = 6.0f;    // hip segment length
const float FEMUR  = 8.5f;    // upper leg length
const float TIBIA  = 14.5f;   // lower leg length
const float LIFT_H = 3.0f;    // how high feet lift per step

// Servo constants
const int   SERVO_MID  = 1500;   // pulse width at 90 degrees (microseconds)
const float DEG_TO_US  = 11.11f; // microseconds per degree

// Speed and height — tunable at runtime
uint16_t moveTime = 500;    // ms per gait phase (lower = faster steps)
float    bodyZ    = -9.0f;  // current body height (more negative = lower)

const uint16_t SPEED_MIN   = 150;
const uint16_t SPEED_MAX   = 1000;
const uint16_t SPEED_STEP  = 50;

const float    HEIGHT_STEP = 1.0f;
const float    HEIGHT_MIN  = -12.0f;
const float    HEIGHT_MAX  = -7.0f;

// Status variables
bool          btConnected        = false;
float         voltageInVolts     = 0.0f;
unsigned long lastVoltageRequest = 0;
const char*   lastCmdName        = "NONE";

// Utility: print a message to both BT app and USB Serial at once
void btLog(const char* msg) { SerialBT.println(msg); Serial.println(msg); }

// ── Leg configuration ─────────────────────────────────────────
//
//  id          servo base ID on LSC-32
//              (coxa=id, femur=id+1, tibia=id+2)
//  restX       outward reach of the foot (cm)
//  walkY       Y position used during forward/backward/turn steps
//  rotY        Y position used during rotate-in-place (all 0 = symmetric)
//  restZ       current height — adjustHeight() updates this live
//  coxaDir     +1 = left-side leg, -1 = right-side leg
//  strideScale corrects for coxa mounting angle
//              (1.0 = horizontal, 1.41 = 45 degree corner)
//
struct Leg {
  uint8_t id;
  float   restX, walkY, rotY, restZ;
  int     coxaDir;
  float   strideScale;
};

//  id    X  walkY  rotY    Z   dir  scale
Leg legs[6] = {
  {  0,  10,   3,    0,   -9,  +1,  1.0f },  // Front-Left
  {  3,  10,  -1,    0,   -9,  +1,  1.0f },  // Mid-Left
  { 29,  10,  -4,    0,   -9,  +1,  1.0f },  // Rear-Left
  {  6,  10,   0,    0,   -9,  -1,  1.0f },  // Front-Right
  {  9,  10,   2,    0,   -9,  -1,  1.0f },  // Mid-Right
  { 26,  10,   6,    0,   -9,  -1,  1.0f },  // Rear-Right
};

// Tripod groups: two stable triangles that alternate each step.
// While A lifts and steps, B stays planted to support the body, then swap.
const int GRP_A[3] = { 0, 2, 4 }; // Front-Left, Rear-Left,  Mid-Right
const int GRP_B[3] = { 1, 3, 5 }; // Mid-Left,   Front-Right, Rear-Right


// ───────────────────────────────────────────────────────────────
//  SECTION 2 — IK SOLVER
//
//  This is the lowest-level function. Everything else calls this.
//  Give it a 3D foot position (X, Y, Z in cm) and it:
//    1. Solves the joint angles using inverse kinematics
//    2. Converts angles to servo pulse widths (microseconds)
//    3. Sends a 16-byte packet to the LSC-32 to physically move the leg
// ───────────────────────────────────────────────────────────────
void moveFootXYZ(uint8_t id, float x, float y, float z, uint16_t t) {

  // Coxa: what horizontal angle should the hip point at?
  float coxaAngle = atan2(y, x);

  // Femur+Tibia: how far must they reach horizontally (after the hip)?
  float R = sqrt(x*x + y*y) - COXA;
  if (R < 0.5f) R = 0.5f; // clamp to avoid math errors near zero

  // Total straight-line distance from the knee joint to the foot
  float D = sqrt(R*R + z*z);
  D = constrain(D, fabsf(FEMUR-TIBIA)+0.1f, FEMUR+TIBIA-0.1f); // keep inside workspace

  // Law of cosines to find each joint angle
  float tibiaAngle = acos(constrain((FEMUR*FEMUR + TIBIA*TIBIA - D*D) / (2*FEMUR*TIBIA), -1.0f, 1.0f));
  float femurAngle = atan2(z, R) + acos(constrain((FEMUR*FEMUR + D*D - TIBIA*TIBIA) / (2*FEMUR*D), -1.0f, 1.0f));

  // Convert radians to degrees
  float cDeg = coxaAngle  * 180.0f / PI;
  float fDeg = femurAngle * 180.0f / PI;
  float tDeg = 180.0f - (tibiaAngle * 180.0f / PI);

  // Convert degrees to servo pulse widths
  uint16_t p1 = constrain((int)(SERVO_MID +  cDeg          * DEG_TO_US), 500, 2500);
  uint16_t p2 = constrain((int)(SERVO_MID + (fDeg - 90.0f) * DEG_TO_US), 500, 2500);
  uint16_t p3 = constrain((int)(SERVO_MID - (tDeg - 90.0f) * DEG_TO_US), 500, 2500);

  // Build and send the 16-byte LSC-32 packet (moves all 3 servos at once)
  byte pkt[16] = {
    0x55, 0x55, 14, 0x03, 3,           // header: sync, length, command, count
    lowByte(t),    highByte(t),          // move duration
    id,            lowByte(p1), highByte(p1),   // coxa servo
    uint8_t(id+1), lowByte(p2), highByte(p2),   // femur servo
    uint8_t(id+2), lowByte(p3), highByte(p3)    // tibia servo
  };
  LSC.write(pkt, 16);
}


// ───────────────────────────────────────────────────────────────
//  SECTION 3 — GAIT SHAPES (plug-and-play)
//
//  A "gait" = just two small functions that answer:
//    swingY(legIndex) → where does this foot step TO when lifting?
//    dragY(legIndex)  → where does this foot slide TO while on ground?
//
//  The gait engine calls these. It doesn't care what they calculate,
//  so you can add any new gait without changing any engine code.
//
//  TO ADD A NEW GAIT:
//    1. Write a swingY and dragY function below
//    2. Add one row to GAIT_TABLE[]   ← that's it, nothing else changes
// ───────────────────────────────────────────────────────────────

// Gait 0 — Tripod Walk (default forward/backward movement)
// Left and right legs step in opposite Y directions (coxaDir handles this).
float tripodSwing(int i){ return legs[i].walkY - (3.0f * legs[i].coxaDir * legs[i].strideScale); }
float tripodDrag (int i){ return legs[i].walkY + (3.0f * legs[i].coxaDir * legs[i].strideScale) * 2; }

// Gait 1 — Crab Walk (slides sideways)
// All 6 legs step the same +Y direction — body slides sideways.
float crabSwing(int i){ return legs[i].restX + 3.0f; }
float crabDrag (int i){ return legs[i].restX - 6.0f; }

// Gait 2 — Diagonal Walk (travels at an angle)
// Front legs get a +2 cm Y bias, rear legs get -2 cm — creates a diagonal path.
float diagSwing(int i){
  float bias = (i <= 1) ? 2.0f : (i <= 3) ? 0.0f : -2.0f; // front / mid / rear
  return legs[i].walkY - (3.0f * legs[i].coxaDir * legs[i].strideScale) + bias;
}
float diagDrag(int i){
  float bias = (i <= 1) ? 2.0f : (i <= 3) ? 0.0f : -2.0f;
  return legs[i].walkY + (3.0f * legs[i].coxaDir * legs[i].strideScale) * 2 - bias * 2;
}

// Gait 3 — Wave Gait (very stable ripple, 5 legs grounded at all times)
// Each leg steps slightly further than the previous — creates a ripple effect.
float waveSwing(int i){
  float extra = (i % 3) * 0.5f;  // 0.0, 0.5, or 1.0 extra cm per leg
  return legs[i].walkY - (2.0f + extra) * legs[i].coxaDir * legs[i].strideScale;
}
float waveDrag(int i){
  float extra = (i % 3) * 0.5f;
  return legs[i].walkY + (2.0f + extra) * legs[i].coxaDir * legs[i].strideScale * 2;
}

// Gait table — one row per gait style
struct GaitDef { const char* name; float(*swingY)(int); float(*dragY)(int); };

GaitDef GAIT_TABLE[] = {
  { "Tripod Walk",   tripodSwing, tripodDrag },  // 0 — default
  { "Crab Walk",     crabSwing,   crabDrag   },  // 1
  { "Diagonal Walk", diagSwing,   diagDrag   },  // 2
  { "Wave Gait",     waveSwing,   waveDrag   },  // 3
};
const uint8_t GAIT_COUNT = sizeof(GAIT_TABLE) / sizeof(GAIT_TABLE[0]);

uint8_t activeGait = 0;  // ← change this one number to switch walking style


// ───────────────────────────────────────────────────────────────
//  SECTION 4 — COMMAND QUEUE (8-slot circular buffer)
//
//  Why it exists:
//    Each gait step takes ~500 ms. Without a queue, pressing a
//    button while the bot is mid-step would lose that command.
//    Instead, commands go into this queue and the engine picks
//    them up automatically when the current step finishes.
//
//  How it works:
//    qHead = where to read the next item from
//    qTail = where to write the next item to
//    When either index hits the end of the array, it wraps to 0.
// ───────────────────────────────────────────────────────────────
const uint8_t QUEUE_SIZE = 8;
char          cmdQueue[QUEUE_SIZE];
uint8_t       qHead = 0, qTail = 0, qCount = 0;

void queuePush(char c) {
  if (qCount >= QUEUE_SIZE) return;       // full — silently discard
  cmdQueue[qTail] = c;
  qTail = (qTail + 1) % QUEUE_SIZE;      // advance write position, wrap at end
  qCount++;
}

bool queuePop(char &out) {
  if (qCount == 0) return false;          // nothing waiting
  out   = cmdQueue[qHead];
  qHead = (qHead + 1) % QUEUE_SIZE;      // advance read position, wrap at end
  qCount--;
  return true;
}

void queueClear() { qHead = qTail = qCount = 0; }


// ───────────────────────────────────────────────────────────────
//  SECTION 5 — NON-BLOCKING GAIT ENGINE
//
//  Each full step cycle has 5 phases:
//
//   SEND_A  →  tell Group A to lift+step, Group B to drag on ground
//   PLANT_A →  wait moveTime ms, then lower Group A to the ground
//   SEND_B  →  tell Group B to lift+step, Group A to drag on ground
//   PLANT_B →  wait moveTime ms, then lower Group B to the ground
//   RECENTER → all 6 legs return to their rest positions
//   then: repeat (if continuous) or go IDLE (if single-step)
//
//  Instead of delay(), each phase stores its start time (phaseStart).
//  gaitTick() runs every loop(). It checks millis() - phaseStart
//  and advances when enough time has passed.
//  → loop() is NEVER blocked and can always read new BT input.
//
//  JOYSTICK BLEND (jFwd, jTurn):
//    jFwd  scales stride length: +1.0 = full forward, -1.0 = backward
//    jTurn adds a side bias:     +1.0 = arc left,     -1.0 = arc right
//    Both are applied inside sendPhase() to blend in one smooth motion.
// ───────────────────────────────────────────────────────────────

// Joystick values — set by movement functions or the 'j' command
float jFwd  = 0.0f;  // -1.0 (backward)  to  +1.0 (forward)
float jTurn = 0.0f;  // -1.0 (right arc) to  +1.0 (left arc)

// Gait engine state variables
enum GaitPhase { IDLE, SEND_A, PLANT_A, SEND_B, PLANT_B, RECENTER };
GaitPhase     gaitPhase = IDLE;
bool          looping   = false; // true = keep repeating the step; false = run once
unsigned long phaseStart = 0;   // millis() when the current phase started

// Forward declaration — gaitTick() calls handleCommand() when IDLE
void handleCommand(char cmd);

// sendPhase(): send servo commands for one half-cycle
//   groupASwings = true  → Group A lifts+steps, Group B stays and drags
//   groupASwings = false → Group B lifts+steps, Group A stays and drags
void sendPhase(bool groupASwings) {
  GaitDef&   g        = GAIT_TABLE[activeGait];
  const int* swingGrp = groupASwings ? GRP_A : GRP_B;
  const int* dragGrp  = groupASwings ? GRP_B : GRP_A;

  // Swing group — lift up and step to the target Y position
  for (int i = 0; i < 3; i++) {
    int   idx      = swingGrp[i];
    float stepY    = legs[idx].walkY + (g.swingY(idx) - legs[idx].walkY) * (jFwd == 0 ? 1.0f : jFwd);
    float turnBias = jTurn * 1.5f * legs[idx].coxaDir; // left legs go farther, right legs less
    moveFootXYZ(legs[idx].id, legs[idx].restX, stepY + turnBias, legs[idx].restZ - LIFT_H, moveTime);
  }

  // Drag group — stay on the ground, slide to push the body forward
  for (int i = 0; i < 3; i++) {
    int   idx      = dragGrp[i];
    float slideY   = legs[idx].walkY + (g.dragY(idx) - legs[idx].walkY) * (jFwd == 0 ? 1.0f : jFwd);
    float turnBias = jTurn * 1.5f * legs[idx].coxaDir;
    moveFootXYZ(legs[idx].id, legs[idx].restX, slideY + turnBias, legs[idx].restZ, moveTime);
  }
}

// plantGroup(): lower one group straight down to the floor
void plantGroup(bool groupA) {
  GaitDef&   g   = GAIT_TABLE[activeGait];
  const int* grp = groupA ? GRP_A : GRP_B;
  for (int i = 0; i < 3; i++)
    moveFootXYZ(legs[grp[i]].id, legs[grp[i]].restX, g.dragY(grp[i]), legs[grp[i]].restZ, moveTime / 3);
}

// recenterLegs(): move all 6 legs back to their walkY rest positions
void recenterLegs() {
  for (int i = 0; i < 6; i++)
    moveFootXYZ(legs[i].id, legs[i].restX, legs[i].walkY, legs[i].restZ, moveTime / 2);
}

// startGait(): begin a gait cycle using whichever gait activeGait points to.
//   continuous = true  → keeps repeating until standStill() is called
//   continuous = false → runs one complete cycle then stops
void startGait(bool continuous) {
  looping    = continuous;
  gaitPhase  = SEND_A;
  phaseStart = millis();
}

void stopGait() { gaitPhase = IDLE; looping = false; }

// gaitTick(): called every loop(). Advances the phase when its timer expires.
void gaitTick() {

  // When IDLE, pick up the next waiting command (if any)
  if (gaitPhase == IDLE) {
    char cmd;
    if (queuePop(cmd)) handleCommand(cmd);
    return;
  }

  unsigned long elapsed = millis() - phaseStart;

  switch (gaitPhase) {

    case SEND_A:   // Phase 1 — send commands immediately (no wait)
      sendPhase(true);
      gaitPhase  = PLANT_A;
      phaseStart = millis();
      break;

    case PLANT_A:  // Phase 2 — wait for servos to finish, then plant Group A
      if (elapsed >= (unsigned long)(moveTime + 20)) {
        plantGroup(true);
        gaitPhase  = SEND_B;
        phaseStart = millis();
      }
      break;

    case SEND_B:   // Phase 3 — short settle, then send Group B commands
      if (elapsed >= (unsigned long)(moveTime / 3 + 10)) {
        sendPhase(false);
        gaitPhase  = PLANT_B;
        phaseStart = millis();
      }
      break;

    case PLANT_B:  // Phase 4 — wait for servos, then plant Group B
      if (elapsed >= (unsigned long)(moveTime + 20)) {
        plantGroup(false);
        gaitPhase  = RECENTER;
        phaseStart = millis();
      }
      break;

    case RECENTER: // Phase 5 — return all legs to rest, then loop or stop
      if (elapsed >= (unsigned long)(moveTime / 3 + 10)) {
        recenterLegs();
        phaseStart = millis();
        if (looping) {
          gaitPhase  = SEND_A;
          phaseStart = millis() + moveTime / 2; // wait for recenter to finish first
        } else {
          gaitPhase  = IDLE;                    // one-shot: done
        }
      }
      break;

    default: gaitPhase = IDLE; break;
  }
}


// ───────────────────────────────────────────────────────────────
//  SECTION 6 — MOVEMENT FUNCTIONS
//
//  Each function sets jFwd/jTurn (direction + blend) then calls
//  startGait(true) in continuous mode. The engine keeps repeating
//  until standStill() is called.
// ───────────────────────────────────────────────────────────────

void standStill() {
  stopGait(); queueClear(); jFwd = jTurn = 0.0f;
  // Home all legs to the symmetric rest position (rotY = 0 for all)
  for (int i = 0; i < 6; i++)
    moveFootXYZ(legs[i].id, legs[i].restX, legs[i].rotY, legs[i].restZ, moveTime);
}

void walkForward()  { jFwd =  1.0f; jTurn =  0.0f; startGait(true); }
void walkBackward() { jFwd = -1.0f; jTurn =  0.0f; startGait(true); }
void turnLeft()     { jFwd =  0.5f; jTurn =  1.0f; startGait(true); }
void turnRight()    { jFwd =  0.5f; jTurn = -1.0f; startGait(true); }
void rotateLeft()   { jFwd =  0.0f; jTurn =  1.0f; startGait(true); }
void rotateRight()  { jFwd =  0.0f; jTurn = -1.0f; startGait(true); }


// ───────────────────────────────────────────────────────────────
//  SECTION 7 — HEIGHT CONTROL
//
//  adjustHeight(+1.0) raises the body by 1 cm.
//  adjustHeight(-1.0) lowers it.
//  It updates restZ in all 6 legs so every gait picks up the change.
// ───────────────────────────────────────────────────────────────

void adjustHeight(float delta) {
  float newZ = bodyZ + delta;
  if (newZ < HEIGHT_MIN || newZ > HEIGHT_MAX) {
    btLog(delta > 0 ? ">> Max height reached" : ">> Min height reached");
    return;
  }
  bodyZ = newZ;
  for (int i = 0; i < 6; i++) legs[i].restZ = bodyZ;
  standStill(); // re-home legs at new height
  char buf[32]; snprintf(buf, sizeof(buf), ">> Height: %.1f cm", bodyZ); btLog(buf);
}

void increaseHeight() { adjustHeight(+HEIGHT_STEP); }
void decreaseHeight() { adjustHeight(-HEIGHT_STEP); }


// ───────────────────────────────────────────────────────────────
//  SECTION 8 — INPUT PARSER + COMMAND HANDLER
//
//  parseInput(char) handles every character arriving over BT.
//    Single-char commands (F, B, L...) → dispatched immediately.
//    Multi-char commands → buffered until newline arrives:
//      "j80,30"  = joystick: 80% forward, 30% left turn
//      "g2"      = switch to gait index 2
// ───────────────────────────────────────────────────────────────

String inputBuffer = ""; // accumulates characters for multi-char commands

void handleCommand(char cmd) {
  switch (cmd) {
    case 'F': case 'f': lastCmdName="FORWARD";      btLog(">> walkForward");   walkForward();   break;
    case 'B': case 'b': lastCmdName="BACKWARD";     btLog(">> walkBackward");  walkBackward();  break;
    case 'L': case 'l': lastCmdName="TURN LEFT";    btLog(">> turnLeft");      turnLeft();      break;
    case 'R': case 'r': lastCmdName="TURN RIGHT";   btLog(">> turnRight");     turnRight();     break;
    case 'W': case 'w': lastCmdName="ROTATE LEFT";  btLog(">> rotateLeft");    rotateLeft();    break;
    case 'X': case 'x': lastCmdName="ROTATE RIGHT"; btLog(">> rotateRight");   rotateRight();   break;
    case 'U': case 'u': lastCmdName="HEIGHT UP";    increaseHeight();          break;
    case 'D': case 'd': lastCmdName="HEIGHT DOWN";  decreaseHeight();          break;
    case 'S': case 's':
    case 'H': case 'h': lastCmdName="STOP";         btLog(">> standStill");    standStill();    break;
    case '+':
      if (moveTime > SPEED_MIN) moveTime -= SPEED_STEP;
      { char b[28]; snprintf(b, sizeof(b), ">> Speed: %u ms", moveTime); btLog(b); }
      break;
    case '-':
      if (moveTime < SPEED_MAX) moveTime += SPEED_STEP;
      { char b[28]; snprintf(b, sizeof(b), ">> Speed: %u ms", moveTime); btLog(b); }
      break;
    case '?':
      SerialBT.println("─── SpiderBot V2.6 ───");
      SerialBT.print("Last cmd : "); SerialBT.println(lastCmdName);
      SerialBT.print("Gait     : "); SerialBT.println(GAIT_TABLE[activeGait].name);
      SerialBT.print("Speed    : "); SerialBT.print(moveTime);       SerialBT.println(" ms");
      SerialBT.print("Height   : "); SerialBT.print(bodyZ);           SerialBT.println(" cm");
      SerialBT.print("Battery  : "); SerialBT.print(voltageInVolts);  SerialBT.println(" V");
      SerialBT.print("jFwd     : "); SerialBT.print(jFwd);
      SerialBT.print("  jTurn  : "); SerialBT.println(jTurn);
      SerialBT.print("Queue    : "); SerialBT.print(qCount);          SerialBT.println(" waiting");
      SerialBT.println("──────────────────────");
      break;
    default: break; // silently ignore \r \n and unrecognised characters
  }
}

void parseInput(char c) {
  if (c == '\n' || c == '\r') {
    // End of a multi-char command — process the buffered string now
    if (inputBuffer.length() == 0) return;
    char first = inputBuffer.charAt(0);

    if (first == 'j' || first == 'J') {
      // Joystick: "j80,30" → fwd = 0.80, turn = 0.30
      int comma = inputBuffer.indexOf(',');
      if (comma > 1) {
        jFwd  = constrain(inputBuffer.substring(1, comma).toFloat() / 100.0f, -1.0f, 1.0f);
        jTurn = constrain(inputBuffer.substring(comma + 1).toFloat() / 100.0f, -1.0f, 1.0f);
        moveTime    = (uint16_t)map((long)(fabsf(jFwd)*100), 0, 100, SPEED_MAX, SPEED_MIN);
        lastCmdName = "JOYSTICK";
        if (fabsf(jFwd) < 0.05f && fabsf(jTurn) < 0.05f)
          standStill();
        else
          startGait(true);
      }
    }
    else if (first == 'g' || first == 'G') {
      // Gait switch: "g2" → use GAIT_TABLE[2]
      uint8_t idx = (uint8_t)inputBuffer.substring(1).toInt();
      if (idx < GAIT_COUNT) {
        activeGait = idx;
        char buf[40]; snprintf(buf, sizeof(buf), ">> Gait: %s", GAIT_TABLE[idx].name); btLog(buf);
      } else {
        btLog(">> Bad gait number. Use g0, g1, g2 or g3.");
      }
    }
    inputBuffer = ""; // clear buffer ready for next command

  } else {
    // Single-char command: dispatch immediately unless it's a multi-char prefix
    if (inputBuffer.length() == 0 && c != 'j' && c != 'J' && c != 'g' && c != 'G')
      handleCommand(c);
    else
      inputBuffer += c; // accumulate multi-char command
  }
}


// ───────────────────────────────────────────────────────────────
//  BT CALLBACK
//  Fires automatically when a phone connects or disconnects.
// ───────────────────────────────────────────────────────────────
void btCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  if (event == ESP_SPP_SRV_OPEN_EVT) {
    btConnected = true;
    btLog("SpiderBot V2.6 Ready!");
    btLog("F/B=walk  L/R=turn  W/X=rotate  U/D=height  S=stop  ?=status");
    btLog("g0..g3=gait   j80,30=joystick (fwd%, turn%)");
  }
  if (event == ESP_SPP_CLOSE_EVT) {
    btConnected = false;
    standStill(); // always home the bot when phone disconnects
    Serial.println("BT: Disconnected");
  }
}


// ───────────────────────────────────────────────────────────────
//  SETUP — runs once on power-on
// ───────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  LSC.begin(9600, SERIAL_8N1, 16, 17); // connect to LSC-32 on UART2
  SerialBT.register_callback(btCallback);
  SerialBT.begin("SpiderBot");          // bot appears as "SpiderBot" when pairing
  Serial.println("SpiderBot V2.6 — pair your phone with: SpiderBot");
  delay(2000);
  standStill(); // home all legs on boot
}


// ───────────────────────────────────────────────────────────────
//  LOOP — runs forever, never blocked
//
//  Three things happen every cycle:
//   1. Read any new BT / Serial characters and parse them
//   2. Tick the gait engine (advances phase when timer expires)
//   3. Poll battery voltage every 5 seconds
// ───────────────────────────────────────────────────────────────
void loop() {
  // 1. Read and parse all waiting input characters
  while (SerialBT.available()) parseInput((char)SerialBT.read());
  while (Serial.available())   parseInput((char)Serial.read());

  // 2. Advance the gait state machine (non-blocking, timer-driven)
  gaitTick();

  // 3. Battery voltage: request every 5 s, store value when it changes
  controller.receiveHandle(); // must call every loop to catch incoming replies
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
