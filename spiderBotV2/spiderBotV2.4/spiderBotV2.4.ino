/*******************************************************
 * FileName:     spiderBotV2_4.ino
 * Author:       Amit Parihar
 * Version:      2.4
 * Date:         2026
 * Description:  Spider bot with ESP32 built-in Classic Bluetooth (SPP).
 *               No external BT module needed.
 *               UART2 → LSC-32 servo controller (unchanged from V1)
 * Updates: Added 9 Movement function.
 *          Added Battery Voltage check in status values
 *
 * WIRING:       ONLY UART2 needed: RX=16, TX=17 → LSC-32
 *
 * PHONE APP:    "Serial Bluetooth Terminal" (Android) or
 *               "Bluetooth Terminal" (iOS)
 *               Pair name: "SpiderBot"  |  No PIN or PIN: 1234
 *
 * COMMANDS:
 *   F  - walkForward()       B  - walkBackward()
 *   L  - turnLeft()          R  - turnRight()
 *   W  - rotateLeft()        X  - rotateRight()
 *   U  - increaseHeight()    D  - decreaseHeight()
 *   S  - standStill()        H  - standStill() (home)
 *   +  - Speed up            -  - Slow down
 *   ?  - Status report
 *******************************************************/

#include <Arduino.h>
#include <math.h>
#include "BluetoothSerial.h"   // Part of ESP32 Arduino core — no install needed
#include <LobotServoController.h>

// ── Compile-time check ─────────────────────────────────────────
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
  #error "Bluetooth is not enabled. Enable it in Arduino IDE: Tools → ESP32 Bluetooth"
#endif

// ─── Objects ───────────────────────────────────────────────────
HardwareSerial LSC(2);         // UART2 → LSC-32 (pins 16/17)
BluetoothSerial SerialBT;      // ESP32 built-in Classic BT
LobotServoController controller(LSC);  //Create an Lsc object

// ─── Geometry (cm) ─────────────────────────────────────────────
const float L1 = 6.0f;
const float L2 = 8.5f;
const float L3 = 14.5f;
const int   SERVO_CENTER = 1500;
const float DEG_TO_US    = 11.11f;

// ─── Leg Configuration ─────────────────────────────────────────
struct Leg {
  uint8_t id;
  float restX, restY, restZ;
  int   coxaDir;      // +1 left side, -1 right side
  float strideScale;  // Compensates for coxa mounting angle.
                      // Middle legs (horizontal, 90°) = 1.0
                      // Corner legs (45°) = 1.41 (=1/cos45°)
                      // Tune this if robot drifts left or right during walk.
};

Leg legs1[6] = {    // Walking Position
  { 0,  10,  3, -9, +1, 1.0f },   // Front-Left  (45° corner)
  { 3,  10, -1, -9, +1, 1.00f },   // Mid-Left    (horizontal)
  {29,  10, -4, -9, +1, 1.0f },   // Rear-Left   (45° corner)
  { 6,  10,  0, -9, -1, 1.0f },   // Front-Right (45° corner)
  { 9,  10,  2, -9, -1, 1.00f },   // Mid-Right   (horizontal)
  {26,  10,  6, -9, -1, 1.0f },   // Rear-Right  (45° corner)
};//+X → left side, -X → right side, +Y → back side, -Y → front side, Z negative → downward (ground)
// tweaked the y values a little, to conterattack the error.

Leg legs2[6] = {    // Rotating Position
  { 0,  10, 0, -9, +1, 1.0f },   // Front-Left
  { 3,  10, 0, -9, +1, 1.0f },   // Mid-Left
  {29,  10, 0, -9, +1, 1.0f },   // Rear-Left
  { 6,  10, 0, -9, -1, 1.0f },   // Front-Right
  { 9,  10, 0, -9, -1, 1.0f },   // Mid-Right
  {26,  10, 0, -9, -1, 1.0f },   // Rear-Right
};//+X → left side, -X → right side, +Y → back side, -Y → front side, Z negative → downward (ground)


// ─── Speed Control ─────────────────────────────────────────────
uint16_t moveTime = 500;
const uint16_t SPEED_MIN  = 200;
const uint16_t SPEED_MAX  = 1000;
const uint16_t SPEED_STEP = 50;

// ─── Height Control ────────────────────────────────────────────
// heightOffset is added directly to restZ of both leg arrays.
// All gait functions (tripodStep, rotateStep, standHome) use restZ,
// so height change is picked up automatically everywhere.
float heightOffset = 0.0f;
float bodyZ = -9.0f;
const float HEIGHT_STEP = 1.0f;   // cm per press
const float HEIGHT_MIN  = -12.0f;  // max lower  (restZ - 10cm)
const float HEIGHT_MAX  = -7.0f;  // max raise  (restZ -5cm)

// ─── Robot State ───────────────────────────────────────────────
enum RobotState { 
  STATE_STOP,
  STATE_FORWARD, 
  STATE_BACKWARD, 
  STATE_TURN_LEFT,    // arc-walk turn using asymmetric stride
  STATE_TURN_RIGHT,
  STATE_ROTATE_LEFT,  // pure spin in place
  STATE_ROTATE_RIGHT
};
RobotState currentState = STATE_STOP;

// ─── BT Connection Tracking ────────────────────────────────────
bool btConnected = false;

// ─── Battery Voltage Tracking ────────────────────────────────────
float voltageInVolts;
unsigned long lastVoltageRequest = 0; // TO measure the the last request time of getBatteryVoltage

// ─────────────────────────────────────────────────────────────────
//  BT callback — fires when phone connects / disconnects
// ─────────────────────────────────────────────────────────────────
void btCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  if (event == ESP_SPP_SRV_OPEN_EVT) {
    btConnected = true;
    Serial.println("BT: Phone connected");
    SerialBT.println("SpiderBot V2.4 Ready!");
    SerialBT.println("F/B=walk  L/R=turn  W/X=rotate  U/D=height  S/H=stop  +/-=speed  ?=status");
  }
  if (event == ESP_SPP_CLOSE_EVT) {
    btConnected = false;
    currentState = STATE_STOP;   // Safety: stop moving if phone disconnects
    Serial.println("BT: Phone disconnected — stopping");
  }
}

// ─────────────────────────────────────────────────────────────────
//  LOW-LEVEL: 3-servo packet → LSC-32
// ─────────────────────────────────────────────────────────────────
void moveLeg(uint8_t startID, uint16_t p1, uint16_t p2, uint16_t p3, uint16_t time) {
  byte packet[16];
  packet[0] = 0x55; 
  packet[1] = 0x55;
  packet[2] = 14; 
  packet[3] = 0x03; 
  packet[4] = 3;
  packet[5] = lowByte(time);  
  packet[6] = highByte(time);

  packet[7]  = startID;     
  packet[8]  = lowByte(p1); 
  packet[9]  = highByte(p1);
  packet[10] = startID + 1; 
  packet[11] = lowByte(p2); 
  packet[12] = highByte(p2);
  packet[13] = startID + 2; 
  packet[14] = lowByte(p3); 
  packet[15] = highByte(p3);
   
  LSC.write(packet, 16);
}

// ─────────────────────────────────────────────────────────────────
//  IK SOLVER
// ─────────────────────────────────────────────────────────────────
void moveFootXYZ(uint8_t startID, float x, float y, float z, uint16_t time) {
  float coxaAngle = atan2(y, x);
  float R = sqrt(x*x + y*y) - L1;
  if (R < 0.5f) R = 0.5f;

  float D = sqrt(R*R + z*z);
  if (D > (L2 + L3)) D = L2 + L3 - 0.1f;
  if (D < fabsf(L2 - L3)) D = fabsf(L2 - L3) + 0.1f;

  float tibiaRad = acos(constrain((L2*L2 + L3*L3 - D*D) / (2*L2*L3), -1.0f, 1.0f));
  float femurRad = atan2(z, R) + acos(constrain((L2*L2 + D*D - L3*L3) / (2*L2*D), -1.0f, 1.0f));

  float cDeg = coxaAngle * 180.0f / PI;
  float fDeg = femurRad  * 180.0f / PI;
  float tDeg = 180.0f - (tibiaRad * 180.0f / PI);

  int p1 = constrain((int)(SERVO_CENTER + (cDeg * DEG_TO_US)),           500, 2500);
  int p2 = constrain((int)(SERVO_CENTER + ((fDeg - 90.0f) * DEG_TO_US)), 500, 2500);
  int p3 = constrain((int)(SERVO_CENTER - ((tDeg - 90.0f) * DEG_TO_US)), 500, 2500);

  moveLeg(startID, p1, p2, p3, time);
}

// ─────────────────────────────────────────────────────────────────
//  GAIT: Stand / Home
// ─────────────────────────────────────────────────────────────────
void standHome() {
  for (int i = 0; i < 6; i++)
    moveFootXYZ(legs2[i].id, legs2[i].restX, legs2[i].restY, legs2[i].restZ, moveTime);
  delay(moveTime + 50);
}

// ─────────────────────────────────────────────────────────────────
//  GAIT: Tripod walk  (+stride = forward, -stride = backward)
// ─────────────────────────────────────────────────────────────────
void tripodStep(float strideY) {
  int A[3] = {0, 2, 4};
  int B[3] = {1, 3, 5};

  // Phase 1: A lifts & swings, B pushes
  for (int i = 0; i < 3; i++)
    moveFootXYZ(legs1[A[i]].id, legs1[A[i]].restX, legs1[A[i]].restY - (strideY * legs1[A[i]].coxaDir * legs1[A[i]].strideScale), legs1[A[i]].restZ - 3.0f, moveTime);
  
  for (int i = 0; i < 3; i++)
    moveFootXYZ(legs1[B[i]].id, legs1[B[i]].restX, legs1[B[i]].restY + (strideY * legs1[B[i]].coxaDir * legs1[B[i]].strideScale) * 2, legs1[B[i]].restZ, moveTime); 
  delay(moveTime + 20);

  for (int i = 0; i < 3; i++)  // A plants on ground
    moveFootXYZ(legs1[A[i]].id, legs1[A[i]].restX, legs1[A[i]].restY - (strideY * legs1[A[i]].coxaDir * legs1[A[i]].strideScale), legs1[A[i]].restZ, moveTime / 2);
  delay(moveTime / 2 + 20);

  // Phase 2: B lifts & swings, A pushes
  for (int i = 0; i < 3; i++)
    moveFootXYZ(legs1[B[i]].id, legs1[B[i]].restX, legs1[B[i]].restY - (strideY * legs1[B[i]].coxaDir * legs1[B[i]].strideScale), legs1[B[i]].restZ - 3.0f, moveTime);
  
  for (int i = 0; i < 3; i++)
    moveFootXYZ(legs1[A[i]].id, legs1[A[i]].restX, legs1[A[i]].restY + (strideY * legs1[A[i]].coxaDir * legs1[A[i]].strideScale) * 2, legs1[A[i]].restZ, moveTime);
  delay(moveTime + 20);

  for (int i = 0; i < 3; i++)  // B plants on ground
    moveFootXYZ(legs1[B[i]].id, legs1[B[i]].restX, legs1[B[i]].restY - (strideY * legs1[B[i]].coxaDir * legs1[B[i]].strideScale), legs1[B[i]].restZ, moveTime / 2);
  delay(moveTime / 2 + 20);

  // Recenter all legs1 — prevents position drift accumulating each cycle
  for (int i = 0; i < 6; i++)
    moveFootXYZ(legs1[i].id, legs1[i].restX, legs1[i].restY, legs1[i].restZ, moveTime / 2);
  delay(moveTime / 2 + 20);
}

// ─────────────────────────────────────────────────────────────────
//  GAIT: Rotate (+yaw = left, -yaw = right)
//  FIX v2.2: Left/right legs1 swing in opposing Y directions to create
//  torque around vertical axis. Previous X-axis offset had no rotational effect.
// ─────────────────────────────────────────────────────────────────
void rotateStep(float yawOffset) {

  int A[3] = {0, 2, 4};
  int B[3] = {1, 3, 5};

  // Phase 1: A lifts & swings, B pushes
  for (int i = 0; i < 3; i++)
    moveFootXYZ(legs2[A[i]].id, legs2[A[i]].restX, legs2[A[i]].restY + yawOffset, legs2[A[i]].restZ  -3.0f, moveTime);
  for (int i = 0; i < 3; i++)
    moveFootXYZ(legs2[B[i]].id, legs2[B[i]].restX, legs2[B[i]].restY - yawOffset * 2, legs2[B[i]].restZ, moveTime);
  delay(moveTime + 20);

  for (int i = 0; i < 3; i++)  // A plants
    moveFootXYZ(legs2[A[i]].id, legs2[A[i]].restX, legs2[A[i]].restY + yawOffset, legs2[A[i]].restZ , moveTime / 2);
  delay(moveTime / 2 + 20);

  // Phase 2: B lifts & swings, A pushes
  for (int i = 0; i < 3; i++)
    moveFootXYZ(legs2[B[i]].id, legs2[B[i]].restX, legs2[B[i]].restY + yawOffset, legs2[B[i]].restZ - 3.0f, moveTime);
  for (int i = 0; i < 3; i++)
    moveFootXYZ(legs2[A[i]].id, legs2[A[i]].restX, legs2[A[i]].restY - yawOffset * 2, legs2[A[i]].restZ, moveTime);
  delay(moveTime + 20);

  for (int i = 0; i < 3; i++)  // B plants
    moveFootXYZ(legs2[B[i]].id, legs2[B[i]].restX, legs2[B[i]].restY + yawOffset, legs2[B[i]].restZ , moveTime / 2);
  delay(moveTime / 2 + 20);

  // Recenter all legs2 — prevents position drift accumulating each cycle
  for (int i = 0; i < 6; i++)
    moveFootXYZ(legs2[i].id, legs2[i].restX, legs2[i].restY, legs2[i].restZ, moveTime / 2);
  delay(moveTime / 2 + 20);
}

// ─────────────────────────────────────────────────────────────────
//  GAIT: Tripod arc-turn — same structure as tripodStep but each
//  leg uses a side-specific stride length.
//  Left legs (coxaDir=+1) use strideL, right legs (coxaDir=-1) use strideR.
//  turnLeft:  strideL < strideR  → left side steps shorter → body arcs left
//  turnRight: strideR < strideL  → right side steps shorter → body arcs right
// ─────────────────────────────────────────────────────────────────
void tripodTurn(float strideL, float strideR) {
  int A[3] = {0, 2, 4};
  int B[3] = {1, 3, 5};

  // Returns the correct stride for this leg based on which side it is on
  auto S = [&](int idx) -> float {
    return (legs1[idx].coxaDir > 0) ? strideL : strideR;
  };

  // Phase 1: A lifts & swings, B pushes
  for (int i = 0; i < 3; i++)
    moveFootXYZ(legs1[A[i]].id, legs1[A[i]].restX, legs1[A[i]].restY - (S(A[i]) * legs1[A[i]].coxaDir * legs1[A[i]].strideScale), legs1[A[i]].restZ - 3.0f, moveTime);
  for (int i = 0; i < 3; i++)
    moveFootXYZ(legs1[B[i]].id, legs1[B[i]].restX, legs1[B[i]].restY + (S(B[i]) * legs1[B[i]].coxaDir * legs1[B[i]].strideScale) * 2, legs1[B[i]].restZ, moveTime);
  delay(moveTime + 20);

  for (int i = 0; i < 3; i++)  // A plants
    moveFootXYZ(legs1[A[i]].id, legs1[A[i]].restX, legs1[A[i]].restY - (S(A[i]) * legs1[A[i]].coxaDir * legs1[A[i]].strideScale), legs1[A[i]].restZ, moveTime / 2);
  delay(moveTime / 2 + 20);

  // Phase 2: B lifts & swings, A pushes
  for (int i = 0; i < 3; i++)
    moveFootXYZ(legs1[B[i]].id, legs1[B[i]].restX, legs1[B[i]].restY - (S(B[i]) * legs1[B[i]].coxaDir * legs1[B[i]].strideScale), legs1[B[i]].restZ - 3.0f, moveTime);
  for (int i = 0; i < 3; i++)
    moveFootXYZ(legs1[A[i]].id, legs1[A[i]].restX, legs1[A[i]].restY + (S(A[i]) * legs1[A[i]].coxaDir * legs1[A[i]].strideScale) * 2, legs1[A[i]].restZ, moveTime);
  delay(moveTime + 20);

  for (int i = 0; i < 3; i++)  // B plants
    moveFootXYZ(legs1[B[i]].id, legs1[B[i]].restX, legs1[B[i]].restY - (S(B[i]) * legs1[B[i]].coxaDir * legs1[B[i]].strideScale), legs1[B[i]].restZ, moveTime / 2);
  delay(moveTime / 2 + 20);

  // Recenter
  for (int i = 0; i < 6; i++)
    moveFootXYZ(legs1[i].id, legs1[i].restX, legs1[i].restY, legs1[i].restZ, moveTime / 2);
  delay(moveTime / 2 + 20);
}

// ═════════════════════════════════════════════════════════════════
//  NAMED MOVEMENT FUNCTIONS
// ═════════════════════════════════════════════════════════════════

void walkForward()   { tripodStep( 3.0f); }
void walkBackward()  { tripodStep(-3.0f); }

// Arc-walk turns: inside leg takes a 1.0cm step, outside takes 3.0cm.
// The speed difference makes the body curve toward the shorter side.
void turnLeft() {
   tripodTurn(1.0f, 3.0f); }  // left side shorter → arc left

void turnRight() {
   tripodTurn(3.0f, 1.0f); }  // right side shorter → arc right

void rotateLeft() {
   rotateStep( 3.0f); }  // pure spin, uses legs2

void rotateRight() {
   rotateStep(-3.0f); }  // pure spin, uses legs2


void standStill() {
  currentState = STATE_STOP;
  standHome();
}

void increaseHeight() {
  if (bodyZ < HEIGHT_MAX) {
    heightOffset += HEIGHT_STEP;
    bodyZ += HEIGHT_STEP;
    for (int i = 0; i < 6; i++) {
      legs1[i].restZ += HEIGHT_STEP;  // mutate both arrays so all gait
      legs2[i].restZ += HEIGHT_STEP;  // functions pick up the change
    }
    standStill();
    SerialBT.print(">> Height: "); SerialBT.print(bodyZ); SerialBT.println("cm");
    Serial.print("Height: "); Serial.println(bodyZ);
  } else {
    SerialBT.println(">> Max height reached");
  }
}

void decreaseHeight() {
  if (bodyZ > HEIGHT_MIN) {
    heightOffset -= HEIGHT_STEP;
    bodyZ -= HEIGHT_STEP;
    for (int i = 0; i < 6; i++) {
      legs1[i].restZ -= HEIGHT_STEP;
      legs2[i].restZ -= HEIGHT_STEP;
    }
    standStill();
    SerialBT.print(">> Height: "); SerialBT.print(bodyZ); SerialBT.println("cm");
    Serial.print("Height: "); Serial.println(bodyZ);
  } else {
    SerialBT.println(">> Min height reached");
  }
}
// ─────────────────────────────────────────────────────────────────
//  COMMAND HANDLER  (shared by BT and USB Serial)
// ─────────────────────────────────────────────────────────────────
void handleCommand(char cmd) {
  switch (cmd) {
    case 'F': case 'f': currentState = STATE_FORWARD;       SerialBT.println(">> walkForward");    Serial.println("CMD: Forward");       break;
    case 'B': case 'b': currentState = STATE_BACKWARD;      SerialBT.println(">> walkBackward");   Serial.println("CMD: Backward");      break;
    case 'L': case 'l': currentState = STATE_TURN_LEFT;     SerialBT.println(">> turnLeft");       Serial.println("CMD: TurnLeft");      break;
    case 'R': case 'r': currentState = STATE_TURN_RIGHT;    SerialBT.println(">> turnRight");      Serial.println("CMD: TurnRight");     break;
    case 'W': case 'w': currentState = STATE_ROTATE_LEFT;   SerialBT.println(">> rotateLeft");     Serial.println("CMD: RotateLeft");    break;
    case 'X': case 'x': currentState = STATE_ROTATE_RIGHT;  SerialBT.println(">> rotateRight");    Serial.println("CMD: RotateRight");   break;
    case 'U': case 'u': increaseHeight(); break;
    case 'D': case 'd': decreaseHeight(); break;

    case 'S': case 's':
    case 'H': case 'h':
      standStill();
      SerialBT.println(">> standStill"); Serial.println("CMD: Stop");
      break;

    case '+':
      if (moveTime > SPEED_MIN) moveTime -= SPEED_STEP;
      SerialBT.print(">> Speed up → "); SerialBT.print(moveTime); SerialBT.println("ms");
      break;

    case '-':
      if (moveTime < SPEED_MAX) moveTime += SPEED_STEP;
      SerialBT.print(">> Speed down → "); SerialBT.print(moveTime); SerialBT.println("ms");
      break;

    case '?':
      SerialBT.println("─── SpiderBot V2.4 ─────");
      SerialBT.print("State  : ");
      switch (currentState) {
        case STATE_STOP:         SerialBT.println("STOP");         break;
        case STATE_FORWARD:      SerialBT.println("FORWARD");      break;
        case STATE_BACKWARD:     SerialBT.println("BACKWARD");     break;
        case STATE_TURN_LEFT:    SerialBT.println("TURN LEFT");    break;
        case STATE_TURN_RIGHT:   SerialBT.println("TURN RIGHT");   break;
        case STATE_ROTATE_LEFT:  SerialBT.println("ROTATE LEFT");  break;
        case STATE_ROTATE_RIGHT: SerialBT.println("ROTATE RIGHT"); break;
      }
      SerialBT.print("Speed  : "); SerialBT.print(moveTime);    SerialBT.println("ms");
      SerialBT.print("Height : "); SerialBT.print(bodyZ); SerialBT.println("cm");
      SerialBT.print("Current Voltage: "); SerialBT.print(voltageInVolts); SerialBT.println(" V");
      SerialBT.println("──────────────────────");
      break;

    default: break;   // Ignore newlines, \r, etc.
  }
}

// ─────────────────────────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  LSC.begin(9600, SERIAL_8N1, 16, 17);

  // Start Bluetooth with device name "SpiderBot"
  SerialBT.register_callback(btCallback);
  SerialBT.begin("SpiderBot");
  delay(2000);

  Serial.println("SpiderBot V2.4 — Bluetooth started");
  Serial.println("Pair your phone with: SpiderBot");

  delay(1000);
  standStill();
}

// ─────────────────────────────────────────────────────────────────
//  MAIN LOOP
// ─────────────────────────────────────────────────────────────────
void loop() {
  // Read BT commands
  while (SerialBT.available())
    handleCommand((char)SerialBT.read());

  // Also accept USB Serial for bench testing (no phone needed)
  while (Serial.available())
    handleCommand((char)Serial.read());

  // ─────────────────────────────────────────────────────────────────
  // Battery Voltage
  // 1. MUST call this every loop to process incoming serial data
  controller.receiveHandle(); 

  // 2. Request the voltage every 5 seconds (don't spam the board)
  if (millis() - lastVoltageRequest > 5000) {    // millis(): This is a built-in Arduino function that returns the number of milliseconds since the board started up. Think of it as a stopwatch that never stops.
    controller.getBatteryVoltage();           // Sends request to the controller for battery voltage.
    lastVoltageRequest = millis();               // Update the function
    Serial.println("Requesting Voltage...");
  }

  // 3. Extract and print the value whenever it changes
  // Access the public variable 'batteryVoltage' directly
  static uint16_t lastVoltage = 0;
  if (controller.batteryVoltage != lastVoltage) {
    voltageInVolts = controller.batteryVoltage / 1000.0;
    lastVoltage = controller.batteryVoltage;
  }

  // Execute movement via named functions
  switch (currentState) {
    case STATE_FORWARD:      walkForward();   break;
    case STATE_BACKWARD:     walkBackward();  break;
    case STATE_TURN_LEFT:    turnLeft();      break;
    case STATE_TURN_RIGHT:   turnRight();     break;
    case STATE_ROTATE_LEFT:  rotateLeft();    break;
    case STATE_ROTATE_RIGHT: rotateRight();   break;
    case STATE_STOP:         delay(50);       break;
  }
}
