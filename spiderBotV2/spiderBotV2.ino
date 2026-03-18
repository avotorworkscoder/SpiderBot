/*******************************************************
 * FileName:     spiderBotV2.ino
 * Author:       Amit Parihar
 * Version:      2.0
 * Date:         2026
 * Description:  Spider bot with Bluetooth (HC-05/HC-06) control.
 *               Commands received over UART1 (BT) drive the IK engine.
 *               UART2 → LSC-32 servo controller (unchanged from V1)
 *
 * WIRING:
 *   UART2 (LSC-32): RX=16, TX=17   [unchanged]
 *   UART1 (BT):     RX=4,  TX=5    [new — use 1k/2k divider on TX5→HC-05 RXD]
 *
 * BT COMMANDS (send from any "Serial Bluetooth Terminal" app):
 *   F  - Walk Forward
 *   B  - Walk Backward
 *   L  - Turn Left
 *   R  - Turn Right
 *   S  - Stop (stand still)
 *   H  - Home position (all legs neutral)
 *   +  - Increase speed
 *   -  - Decrease speed
 *   ?  - Print status to BT terminal
 *******************************************************/

#include <Arduino.h>
#include <math.h>

// ─── Serial Ports ───────────────────────────────────────────────
HardwareSerial LSC(2);       // UART2 → LSC-32 servo board
HardwareSerial BT(1);        // UART1 → HC-05 Bluetooth module

// ─── Geometry (cm) ──────────────────────────────────────────────
const float L1 = 6.0;        // Coxa length
const float L2 = 8.5;        // Femur length
const float L3 = 14.5;       // Tibia length
const int   SERVO_CENTER = 1500;
const float DEG_TO_US    = 11.11;   // 2000us / 180deg

// ─── Leg Configuration ──────────────────────────────────────────
// Each leg defined by: servo base ID, default stance X/Y/Z
// Leg IDs match your V1 startIDs: 0, 3, 6, 9, 26, 29
struct Leg {
  uint8_t id;       // Base servo ID (Coxa = id, Femur = id+1, Tibia = id+2)
  float   restX;    // Default stance X (cm)
  float   restY;    // Default stance Y (cm, +ve = left side, -ve = right side)
  float   restZ;    // Default stance Z (cm, negative = down)
};

// 6 legs: Front-Left, Mid-Left, Rear-Left, Front-Right, Mid-Right, Rear-Right
Leg legs[6] = {
  { 0,  10,  8, -8 },   // Front-Left
  { 3,  10,  8, -8 },   // Mid-Left
  { 6,  10,  8, -8 },   // Rear-Left
  { 9,  10, -8, -8 },   // Front-Right
  {26,  10, -8, -8 },   // Mid-Right
  {29,  10, -8, -8 },   // Rear-Right
};

// ─── Speed Control ───────────────────────────────────────────────
uint16_t moveTime = 500;          // ms per movement step
const uint16_t SPEED_MIN  = 200;
const uint16_t SPEED_MAX  = 1000;
const uint16_t SPEED_STEP = 50;

// ─── Robot State ─────────────────────────────────────────────────
enum RobotState { STATE_STOP, STATE_FORWARD, STATE_BACKWARD, STATE_LEFT, STATE_RIGHT };
RobotState currentState = STATE_STOP;

// ─────────────────────────────────────────────────────────────────
//  LOW-LEVEL: Send one 3-servo (full leg) packet to LSC-32
// ─────────────────────────────────────────────────────────────────
void moveLeg(uint8_t startID, uint16_t p1, uint16_t p2, uint16_t p3, uint16_t time) {
  byte packet[16];
  packet[0] = 0x55;
  packet[1] = 0x55;
  packet[2] = 14;     // Length: (3 servos * 3) + 5
  packet[3] = 0x03;   // CMD_SERVO_MOVE
  packet[4] = 3;      // Number of servos

  packet[5] = lowByte(time);
  packet[6] = highByte(time);

  packet[7]  = startID;       packet[8]  = lowByte(p1); packet[9]  = highByte(p1);
  packet[10] = startID + 1;   packet[11] = lowByte(p2); packet[12] = highByte(p2);
  packet[13] = startID + 2;   packet[14] = lowByte(p3); packet[15] = highByte(p3);

  LSC.write(packet, 16);
}

// ─────────────────────────────────────────────────────────────────
//  IK SOLVER: World XYZ → servo pulses → moveLeg()
// ─────────────────────────────────────────────────────────────────
void moveFootXYZ(uint8_t startID, float x, float y, float z, uint16_t time) {
  float coxaAngle    = atan2(y, x);
  float horizontalDist = sqrt(x * x + y * y);
  float R = horizontalDist - L1;
  if (R < 0.5f) R = 0.5f;

  float D = sqrt(R * R + z * z);
  if (D > (L2 + L3))         D = L2 + L3 - 0.1f;
  if (D < fabsf(L2 - L3))    D = fabsf(L2 - L3) + 0.1f;

  float cosT    = (L2*L2 + L3*L3 - D*D) / (2*L2*L3);
  float tibiaRad = acos(constrain(cosT, -1.0f, 1.0f));

  float a    = atan2(z, R);
  float cosF = (L2*L2 + D*D - L3*L3) / (2*L2*D);
  float b    = acos(constrain(cosF, -1.0f, 1.0f));
  float femurRad = a + b;

  float cDeg = coxaAngle * 180.0f / PI;
  float fDeg = femurRad  * 180.0f / PI;
  float tDeg = 180.0f - (tibiaRad * 180.0f / PI);

  int p1 = constrain((int)(SERVO_CENTER + (cDeg * DEG_TO_US)),              500, 2500);
  int p2 = constrain((int)(SERVO_CENTER + ((fDeg - 90.0f) * DEG_TO_US)),    500, 2500);
  int p3 = constrain((int)(SERVO_CENTER - ((tDeg - 90.0f) * DEG_TO_US)),    500, 2500);

  moveLeg(startID, p1, p2, p3, time);
}

// ─────────────────────────────────────────────────────────────────
//  GAIT: Move all legs to their rest/home positions
// ─────────────────────────────────────────────────────────────────
void standHome() {
  for (int i = 0; i < 6; i++) {
    moveFootXYZ(legs[i].id, legs[i].restX, legs[i].restY, legs[i].restZ, moveTime);
  }
  delay(moveTime + 50);
}

// ─────────────────────────────────────────────────────────────────
//  GAIT: Tripod walk — legs 0,2,4 move together, then 1,3,5
//  strideY = +ve → forward, -ve → backward
// ─────────────────────────────────────────────────────────────────
void tripodStep(float strideY) {
  // --- Group A: lift and swing forward ---
  int groupA[3] = {0, 2, 4};
  int groupB[3] = {1, 3, 5};

  // Group A: swing (lift)
  for (int i = 0; i < 3; i++) {
    int idx = groupA[i];
    moveFootXYZ(legs[idx].id,
                legs[idx].restX,
                legs[idx].restY + strideY,
                legs[idx].restZ + 3.0f,   // lift 3cm
                moveTime);
  }
  // Group B: push backward (body moves forward)
  for (int i = 0; i < 3; i++) {
    int idx = groupB[i];
    moveFootXYZ(legs[idx].id,
                legs[idx].restX,
                legs[idx].restY - strideY,
                legs[idx].restZ,
                moveTime);
  }
  delay(moveTime + 20);

  // Group A: plant
  for (int i = 0; i < 3; i++) {
    int idx = groupA[i];
    moveFootXYZ(legs[idx].id,
                legs[idx].restX,
                legs[idx].restY + strideY,
                legs[idx].restZ,
                moveTime / 2);
  }
  delay(moveTime / 2 + 20);

  // Group B: swing (lift)
  for (int i = 0; i < 3; i++) {
    int idx = groupB[i];
    moveFootXYZ(legs[idx].id,
                legs[idx].restX,
                legs[idx].restY + strideY,
                legs[idx].restZ + 3.0f,
                moveTime);
  }
  // Group A: push backward
  for (int i = 0; i < 3; i++) {
    int idx = groupA[i];
    moveFootXYZ(legs[idx].id,
                legs[idx].restX,
                legs[idx].restY - strideY,
                legs[idx].restZ,
                moveTime);
  }
  delay(moveTime + 20);

  // Group B: plant
  for (int i = 0; i < 3; i++) {
    int idx = groupB[i];
    moveFootXYZ(legs[idx].id,
                legs[idx].restX,
                legs[idx].restY + strideY,
                legs[idx].restZ,
                moveTime / 2);
  }
  delay(moveTime / 2 + 20);
}

// ─────────────────────────────────────────────────────────────────
//  GAIT: Rotate in place — +ve = turn left, -ve = turn right
//  Uses coxa (rotation) offset to yaw the body
// ─────────────────────────────────────────────────────────────────
void rotateStep(float yawOffset) {
  // Left side legs shift Y positive, right side negative
  // Simple implementation: alternate tripod groups with X offset
  int groupA[3] = {0, 2, 4};
  int groupB[3] = {1, 3, 5};

  for (int i = 0; i < 3; i++) {
    int idx = groupA[i];
    float sign = (legs[idx].restY > 0) ? 1.0f : -1.0f;
    moveFootXYZ(legs[idx].id,
                legs[idx].restX + sign * yawOffset,
                legs[idx].restY,
                legs[idx].restZ + 3.0f, moveTime);
  }
  for (int i = 0; i < 3; i++) {
    int idx = groupB[i];
    float sign = (legs[idx].restY > 0) ? -1.0f : 1.0f;
    moveFootXYZ(legs[idx].id,
                legs[idx].restX + sign * yawOffset,
                legs[idx].restY,
                legs[idx].restZ, moveTime);
  }
  delay(moveTime + 20);
  for (int i = 0; i < 3; i++) {
    int idx = groupA[i];
    moveFootXYZ(legs[idx].id, legs[idx].restX, legs[idx].restY, legs[idx].restZ, moveTime / 2);
  }
  delay(moveTime / 2 + 20);

  for (int i = 0; i < 3; i++) {
    int idx = groupB[i];
    float sign = (legs[idx].restY > 0) ? 1.0f : -1.0f;
    moveFootXYZ(legs[idx].id,
                legs[idx].restX + sign * yawOffset,
                legs[idx].restY,
                legs[idx].restZ + 3.0f, moveTime);
  }
  for (int i = 0; i < 3; i++) {
    int idx = groupA[i];
    float sign = (legs[idx].restY > 0) ? -1.0f : 1.0f;
    moveFootXYZ(legs[idx].id,
                legs[idx].restX + sign * yawOffset,
                legs[idx].restY,
                legs[idx].restZ, moveTime);
  }
  delay(moveTime + 20);
  for (int i = 0; i < 3; i++) {
    int idx = groupB[i];
    moveFootXYZ(legs[idx].id, legs[idx].restX, legs[idx].restY, legs[idx].restZ, moveTime / 2);
  }
  delay(moveTime / 2 + 20);
}

// ─────────────────────────────────────────────────────────────────
//  BLUETOOTH: Parse a single command character
// ─────────────────────────────────────────────────────────────────
void handleCommand(char cmd) {
  switch (cmd) {
    case 'F': case 'f':
      currentState = STATE_FORWARD;
      BT.println(">> Forward");
      Serial.println("CMD: Forward");
      break;

    case 'B': case 'b':
      currentState = STATE_BACKWARD;
      BT.println(">> Backward");
      Serial.println("CMD: Backward");
      break;

    case 'L': case 'l':
      currentState = STATE_LEFT;
      BT.println(">> Turn Left");
      Serial.println("CMD: Left");
      break;

    case 'R': case 'r':
      currentState = STATE_RIGHT;
      BT.println(">> Turn Right");
      Serial.println("CMD: Right");
      break;

    case 'S': case 's':
      currentState = STATE_STOP;
      standHome();
      BT.println(">> Stop");
      Serial.println("CMD: Stop");
      break;

    case 'H': case 'h':
      currentState = STATE_STOP;
      standHome();
      BT.println(">> Home");
      Serial.println("CMD: Home");
      break;

    case '+':
      if (moveTime > SPEED_MIN) moveTime -= SPEED_STEP;
      BT.print(">> Speed up → step time: "); BT.println(moveTime);
      Serial.print("Speed: "); Serial.println(moveTime);
      break;

    case '-':
      if (moveTime < SPEED_MAX) moveTime += SPEED_STEP;
      BT.print(">> Speed down → step time: "); BT.println(moveTime);
      Serial.print("Speed: "); Serial.println(moveTime);
      break;

    case '?':
      BT.println("── SpiderBot V2 Status ──");
      BT.print("State    : ");
      switch(currentState) {
        case STATE_STOP:     BT.println("STOP");     break;
        case STATE_FORWARD:  BT.println("FORWARD");  break;
        case STATE_BACKWARD: BT.println("BACKWARD"); break;
        case STATE_LEFT:     BT.println("LEFT");     break;
        case STATE_RIGHT:    BT.println("RIGHT");    break;
      }
      BT.print("Step time: "); BT.print(moveTime); BT.println(" ms");
      BT.println("─────────────────────────");
      break;

    default:
      // Ignore unknown characters (newlines, etc.)
      break;
  }
}

// ─────────────────────────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  LSC.begin(9600, SERIAL_8N1, 16, 17);   // To LSC-32
  BT.begin(9600, SERIAL_8N1, 4, 5);      // To HC-05 (default baud = 9600)
  delay(1000);

  Serial.println("SpiderBot V2 Ready");
  BT.println("SpiderBot V2 Ready");
  BT.println("Commands: F B L R S H + - ?");

  standHome();  // Move to neutral stance on boot
}

// ─────────────────────────────────────────────────────────────────
//  MAIN LOOP
// ─────────────────────────────────────────────────────────────────
void loop() {
  // ── 1. Read BT commands (non-blocking) ──
  while (BT.available()) {
    char c = (char)BT.read();
    handleCommand(c);
  }

  // ── 2. Also accept commands from USB serial (for bench testing) ──
  while (Serial.available()) {
    char c = (char)Serial.read();
    handleCommand(c);
  }

  // ── 3. Execute current movement state ──
  switch (currentState) {
    case STATE_FORWARD:
      tripodStep(3.0f);      // +3cm stride forward
      break;

    case STATE_BACKWARD:
      tripodStep(-3.0f);     // -3cm stride backward
      break;

    case STATE_LEFT:
      rotateStep(2.5f);      // yaw left
      break;

    case STATE_RIGHT:
      rotateStep(-2.5f);     // yaw right
      break;

    case STATE_STOP:
    default:
      delay(50);             // Idle — just poll BT
      break;
  }
}
