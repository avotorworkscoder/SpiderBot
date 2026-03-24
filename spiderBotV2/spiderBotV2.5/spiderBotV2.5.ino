/*******************************************************
 * FileName:     spiderBotV2_5.ino
 * Author:       Amit Parihar
 * Version:      2.5
 * Date:         2026
 * Description:  Spider bot — ESP32 built-in BT (SPP) + LSC-32
 * Changes v2.5: Full code optimisation.
 *   - Unified legs1/legs2 into one Leg array (walkY + rotY per leg)
 *   - Single runGait() engine replaces 3 duplicate gait functions
 *   - Single-step execution: commands run once, bot stops automatically
 *   - adjustHeight() replaces duplicate increase/decrease logic
 *   - btLog() helper replaces repeated dual-print pairs
 *   - Removed redundant heightOffset variable
 *   - Collapsed moveLeg+moveFootXYZ into one function
 *   - Collapsed standHome/standStill alias
 *
 * WIRING:       UART2: RX=16, TX=17 → LSC-32
 *
 * SERIAL BLUETOOTH TERMINAL SETUP:
 *   Settings → Line ending → None   ← prevents \r\n junk
 *   Macros: [FWD]=F [BCK]=B [LFT]=L [RGT]=R
 *           [ROTL]=W [ROTR]=X [UP]=U [DWN]=D
 *           [STOP]=S [INFO]=? [SPD+]=+ [SPD-]=-
 *
 * COMMANDS (each runs ONE step then stops):
 *   F/B  - walkForward / walkBackward
 *   L/R  - turnLeft / turnRight   (arc walk)
 *   W/X  - rotateLeft / rotateRight  (spin in place)
 *   U/D  - increaseHeight / decreaseHeight
 *   S/H  - standStill
 *   +/-  - speed up / down
 *   ?    - status report
 *******************************************************/

#include <Arduino.h>
#include <math.h>
#include <functional>
#include "BluetoothSerial.h"
#include <LobotServoController.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
  #error "Bluetooth not enabled. Enable in: Tools → ESP32 Bluetooth"
#endif

// ═══════════════════════════════════════════════════════════════════
//  OBJECTS
// ═══════════════════════════════════════════════════════════════════
HardwareSerial       LSC(2);
BluetoothSerial      SerialBT;
LobotServoController controller(LSC);

// ═══════════════════════════════════════════════════════════════════
//  GEOMETRY CONSTANTS
// ═══════════════════════════════════════════════════════════════════
const float L1           = 6.0f;
const float L2           = 8.5f;
const float L3           = 14.5f;
const int   SERVO_CENTER = 1500;
const float DEG_TO_US    = 11.11f;
const float LIFT_H       = 3.0f;   // foot lift height per step (cm)

// ═══════════════════════════════════════════════════════════════════
//  LEG CONFIGURATION — single unified array
//
//  walkY: Y rest position for walk/turn gaits  (was legs1[].restY)
//  rotY : Y rest position for rotate gaits     (was legs2[].restY, all 0)
//  restZ: shared, modified live by adjustHeight()
// ═══════════════════════════════════════════════════════════════════
struct Leg {
  uint8_t id;
  float   restX;
  float   walkY;       // Y for walking stance
  float   rotY;        // Y for rotation stance (0 for symmetric spin)
  float   restZ;       // current height — updated by adjustHeight()
  int     coxaDir;     // +1 = left side, -1 = right side
  float   strideScale; // 1.0 = horizontal, 1.41 = 45° corner
};

//  id   X   walkY  rotY   Z   dir  scale
Leg legs[6] = {
  {  0,  10,   3,    0,   -9,  +1,  1.0f },  // Front-Left  (45° corner)
  {  3,  10,  -1,    0,   -9,  +1,  1.0f },  // Mid-Left    (horizontal)
  { 29,  10,  -4,    0,   -9,  +1,  1.0f },  // Rear-Left   (45° corner)
  {  6,  10,   0,    0,   -9,  -1,  1.0f },  // Front-Right (45° corner)
  {  9,  10,   2,    0,   -9,  -1,  1.0f },  // Mid-Right   (horizontal)
  { 26,  10,   6,    0,   -9,  -1,  1.0f },  // Rear-Right  (45° corner)
};

// Tripod groups — stable alternating triangles
const int GRP_A[3] = { 0, 2, 4 };  // FL, RL, MR
const int GRP_B[3] = { 1, 3, 5 };  // ML, FR, RR

// ═══════════════════════════════════════════════════════════════════
//  RUNTIME PARAMETERS
// ═══════════════════════════════════════════════════════════════════
uint16_t moveTime = 500;
float    bodyZ    = -9.0f;

const uint16_t SPEED_MIN   = 200;
const uint16_t SPEED_MAX   = 1000;
const uint16_t SPEED_STEP  = 50;
const float    HEIGHT_STEP = 1.0f;
const float    HEIGHT_MIN  = -12.0f;
const float    HEIGHT_MAX  = -7.0f;

bool          btConnected        = false;
float         voltageInVolts     = 0.0f;
unsigned long lastVoltageRequest = 0;
const char*   lastCmdName        = "NONE";

// ═══════════════════════════════════════════════════════════════════
//  UTILITY: print to both BT and USB Serial in one call
// ═══════════════════════════════════════════════════════════════════
void btLog(const char* msg) {
  SerialBT.println(msg);
  Serial.println(msg);
}

// ═══════════════════════════════════════════════════════════════════
//  BT CALLBACK
// ═══════════════════════════════════════════════════════════════════
void btCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  if (event == ESP_SPP_SRV_OPEN_EVT) {
    btConnected = true;
    btLog("SpiderBot V2.5 Ready!");
    btLog("F/B=walk L/R=turn W/X=rotate U/D=height S=stop ?=status");
  }
  if (event == ESP_SPP_CLOSE_EVT) {
    btConnected = false;
    Serial.println("BT: Disconnected — bot will home on next loop");
    // standStill() not safe to call from callback; handled in loop()
  }
}

// ═══════════════════════════════════════════════════════════════════
//  IK SOLVER + PACKET SENDER  (two old functions merged into one)
//
//  Calculates coxa/femur/tibia pulse widths from foot XYZ and
//  writes the 16-byte LSC-32 packet directly — no intermediate call.
// ═══════════════════════════════════════════════════════════════════
void moveFootXYZ(uint8_t id, float x, float y, float z, uint16_t t) {
  float coxaAngle = atan2(y, x);
  float R = sqrt(x*x + y*y) - L1;
  if (R < 0.5f) R = 0.5f;

  float D = sqrt(R*R + z*z);
  D = constrain(D, fabsf(L2 - L3) + 0.1f, L2 + L3 - 0.1f);

  float tibiaRad = acos(constrain((L2*L2 + L3*L3 - D*D) / (2*L2*L3), -1.0f, 1.0f));
  float femurRad = atan2(z, R) + acos(constrain((L2*L2 + D*D - L3*L3) / (2*L2*D), -1.0f, 1.0f));

  float cDeg = coxaAngle * 180.0f / PI;
  float fDeg = femurRad  * 180.0f / PI;
  float tDeg = 180.0f - (tibiaRad * 180.0f / PI);

  uint16_t p1 = (uint16_t)constrain((int)(SERVO_CENTER +  cDeg           * DEG_TO_US), 500, 2500);
  uint16_t p2 = (uint16_t)constrain((int)(SERVO_CENTER + (fDeg - 90.0f)  * DEG_TO_US), 500, 2500);
  uint16_t p3 = (uint16_t)constrain((int)(SERVO_CENTER - (tDeg - 90.0f)  * DEG_TO_US), 500, 2500);

  byte pkt[16] = {
    0x55, 0x55, 14, 0x03, 3,
    lowByte(t),         highByte(t),
    id,                 lowByte(p1), highByte(p1),
    uint8_t(id + 1),    lowByte(p2), highByte(p2),
    uint8_t(id + 2),    lowByte(p3), highByte(p3)
  };
  LSC.write(pkt, 16);
}

// ═══════════════════════════════════════════════════════════════════
//  GAIT ENGINE — one function drives ALL gait types
//
//  Instead of tripodStep / rotateStep / tripodTurn as 3 separate
//  copy-pasted functions, all gaits call runGait() with two lambdas:
//    swingY(legIdx) → Y target when this leg is in the air (swinging)
//    dragY(legIdx)  → Y target when this leg is on the ground (dragging)
//
//  Sequence: Phase1 (A swings, B drags) → Phase2 (B swings, A drags) → Recenter
// ═══════════════════════════════════════════════════════════════════
void runGait(
  std::function<float(int)> swingY,
  std::function<float(int)> dragY
) {
  uint16_t tSwing  = moveTime;
  uint16_t tPlant  = moveTime / 3;
  uint16_t tCenter = moveTime / 2;

  // Phase 1 — A swings, B drags
  for (int i = 0; i < 3; i++)
    moveFootXYZ(legs[GRP_A[i]].id, legs[GRP_A[i]].restX, swingY(GRP_A[i]), legs[GRP_A[i]].restZ - LIFT_H, tSwing);
  for (int i = 0; i < 3; i++)
    moveFootXYZ(legs[GRP_B[i]].id, legs[GRP_B[i]].restX, dragY(GRP_B[i]),  legs[GRP_B[i]].restZ, tSwing);
  delay(tSwing + 20);
  for (int i = 0; i < 3; i++)    // A plants
    moveFootXYZ(legs[GRP_A[i]].id, legs[GRP_A[i]].restX, swingY(GRP_A[i]), legs[GRP_A[i]].restZ, tPlant);
  delay(tPlant + 10);

  // Phase 2 — B swings, A drags
  for (int i = 0; i < 3; i++)
    moveFootXYZ(legs[GRP_B[i]].id, legs[GRP_B[i]].restX, swingY(GRP_B[i]), legs[GRP_B[i]].restZ - LIFT_H, tSwing);
  for (int i = 0; i < 3; i++)
    moveFootXYZ(legs[GRP_A[i]].id, legs[GRP_A[i]].restX, dragY(GRP_A[i]),  legs[GRP_A[i]].restZ, tSwing);
  delay(tSwing + 20);
  for (int i = 0; i < 3; i++)    // B plants
    moveFootXYZ(legs[GRP_B[i]].id, legs[GRP_B[i]].restX, swingY(GRP_B[i]), legs[GRP_B[i]].restZ, tPlant);
  delay(tPlant + 10);

  // Recenter all legs to walk rest position
  for (int i = 0; i < 6; i++)
    moveFootXYZ(legs[i].id, legs[i].restX, legs[i].walkY, legs[i].restZ, tCenter);
  delay(tCenter + 20);
}

// ═══════════════════════════════════════════════════════════════════
//  MOVEMENT FUNCTIONS
//  Each is a one-liner that calls runGait() with appropriate lambdas.
//  Walk stride = 3cm, rotate yaw = 3cm, turn uses asymmetric strides.
// ═══════════════════════════════════════════════════════════════════

void standStill() {
  for (int i = 0; i < 6; i++)
    moveFootXYZ(legs[i].id, legs[i].restX, legs[i].rotY, legs[i].restZ, moveTime);
  delay(moveTime + 50);
}

void walkForward() {
  runGait(
    [](int i){ return legs[i].walkY - (3.0f * legs[i].coxaDir * legs[i].strideScale); },
    [](int i){ return legs[i].walkY + (3.0f * legs[i].coxaDir * legs[i].strideScale) * 2; }
  );
}

void walkBackward() {
  runGait(
    [](int i){ return legs[i].walkY + (3.0f * legs[i].coxaDir * legs[i].strideScale); },
    [](int i){ return legs[i].walkY - (3.0f * legs[i].coxaDir * legs[i].strideScale) * 2; }
  );
}

void turnLeft() {
  // Left legs take short step (1cm), right legs take long step (3cm) → arcs left
  runGait(
    [](int i){ float s = (legs[i].coxaDir > 0) ? 1.0f : 3.0f;
               return legs[i].walkY - (s * legs[i].coxaDir * legs[i].strideScale); },
    [](int i){ float s = (legs[i].coxaDir > 0) ? 1.0f : 3.0f;
               return legs[i].walkY + (s * legs[i].coxaDir * legs[i].strideScale) * 2; }
  );
}

void turnRight() {
  runGait(
    [](int i){ float s = (legs[i].coxaDir > 0) ? 3.0f : 1.0f;
               return legs[i].walkY - (s * legs[i].coxaDir * legs[i].strideScale); },
    [](int i){ float s = (legs[i].coxaDir > 0) ? 3.0f : 1.0f;
               return legs[i].walkY + (s * legs[i].coxaDir * legs[i].strideScale) * 2; }
  );
}

void rotateLeft() {
  // Left legs swing forward (+Y), right legs swing backward (-Y) → pure spin
  runGait(
    [](int i){ return legs[i].rotY + (3.0f ); },
    [](int i){ return legs[i].rotY - (3.0f ) * 2; }
  );
}

void rotateRight() {
  runGait(
    [](int i){ return legs[i].rotY - (3.0f ); },
    [](int i){ return legs[i].rotY + (3.0f ) * 2; }
  );
}

// ═══════════════════════════════════════════════════════════════════
//  HEIGHT CONTROL — single function, delta = ±HEIGHT_STEP
//  Updates bodyZ and all legs[].restZ, then stands to apply.
// ═══════════════════════════════════════════════════════════════════
void adjustHeight(float delta) {
  float newZ = bodyZ + delta;
  if (newZ < HEIGHT_MIN || newZ > HEIGHT_MAX) {
    btLog(delta > 0 ? ">> Max height reached" : ">> Min height reached");
    return;
  }
  bodyZ = newZ;
  for (int i = 0; i < 6; i++) legs[i].restZ = bodyZ;
  standStill();
  char buf[32];
  snprintf(buf, sizeof(buf), ">> Height: %.1f cm", bodyZ);
  btLog(buf);
}

void increaseHeight() { adjustHeight(+HEIGHT_STEP); }
void decreaseHeight() { adjustHeight(-HEIGHT_STEP); }

// ═══════════════════════════════════════════════════════════════════
//  COMMAND HANDLER — direct execution model (V2.5)
//  Movement functions are called here, not dispatched from loop().
//  One command in → one complete movement → function returns → done.
// ═══════════════════════════════════════════════════════════════════
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
      { char buf[24]; snprintf(buf, sizeof(buf), ">> Speed: %u ms", moveTime); btLog(buf); }
      break;

    case '-':
      if (moveTime < SPEED_MAX) moveTime += SPEED_STEP;
      { char buf[24]; snprintf(buf, sizeof(buf), ">> Speed: %u ms", moveTime); btLog(buf); }
      break;

    case '?':
      SerialBT.println("─── SpiderBot V2.5 ───");
      SerialBT.print("Last   : "); SerialBT.println(lastCmdName);
      SerialBT.print("Speed  : "); SerialBT.print(moveTime);       SerialBT.println(" ms");
      SerialBT.print("Height : "); SerialBT.print(bodyZ);           SerialBT.println(" cm");
      SerialBT.print("Battery: "); SerialBT.print(voltageInVolts);  SerialBT.println(" V");
      SerialBT.println("──────────────────────");
      break;

    default: break;  // silently ignore \r \n and unknown chars
  }
}

// ═══════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  LSC.begin(9600, SERIAL_8N1, 16, 17);
  SerialBT.register_callback(btCallback);
  delay(1000);
  SerialBT.begin("SpiderBot");
  Serial.println("SpiderBot V2.5 — BT started. Pair with: SpiderBot");
  delay(2000);
  standStill();
}

// ═══════════════════════════════════════════════════════════════════
//  MAIN LOOP — V2.5
//  Movement is NOT dispatched here. It runs inside handleCommand().
//  Loop handles only: input reading + battery voltage polling.
// ═══════════════════════════════════════════════════════════════════
void loop() {
  // Input
  while (SerialBT.available()) handleCommand((char)SerialBT.read());
  while (Serial.available())   handleCommand((char)Serial.read());

  // Safety: home if BT dropped while a move was in progress
  if (!btConnected) standStill();

  // Battery voltage — poll every 5s, update on change
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
