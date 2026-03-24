/*******************************************************
 * FileName:     spiderBotV2_BT.ino
 * Author:       Amit Parihar
 * Version:      2.2
 * Date:         2026
 * Description:  Spider bot with ESP32 built-in Classic Bluetooth (SPP).
 *               No external BT module needed.
 *               UART2 → LSC-32 servo controller (unchanged from V1)
 * Updates: Added 2 legs walking and rotating positions.
 *          Added Improved the walking stability.
 *
 * WIRING:       ONLY UART2 needed: RX=16, TX=17 → LSC-32
 *
 * PHONE APP:    "Serial Bluetooth Terminal" (Android) or
 *               "Bluetooth Terminal" (iOS)
 *               Pair name: "SpiderBot"  |  No PIN or PIN: 1234
 *
 * COMMANDS:
 *   F  - Walk Forward
 *   B  - Walk Backward
 *   L  - Turn Left
 *   R  - Turn Right
 *   S  - Stop / Stand
 *   H  - Home position
 *   +  - Speed up
 *   -  - Slow down
 *   ?  - Status report
 *******************************************************/

#include <Arduino.h>
#include <math.h>
#include "BluetoothSerial.h" // Part of ESP32 Arduino core — no install needed

// ── Compile-time check ─────────────────────────────────────────
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error "Bluetooth is not enabled. Enable it in Arduino IDE: Tools → ESP32 Bluetooth"
#endif

// ─── Objects ───────────────────────────────────────────────────
HardwareSerial LSC(2);    // UART2 → LSC-32 (pins 16/17)
BluetoothSerial SerialBT; // ESP32 built-in Classic BT

// ─── Geometry (cm) ─────────────────────────────────────────────
const float L1 = 6.0f;
const float L2 = 8.5f;
const float L3 = 14.5f;
const int SERVO_CENTER = 1500;
const float DEG_TO_US = 11.11f;

// ─── Leg Configuration ─────────────────────────────────────────
struct Leg
{
  uint8_t id;
  float restX, restY, restZ;
};

Leg legs1[6] = {
    // Walking Position
    {0, 10, 6, -8},  // Front-Left
    {3, 10, -3, -8}, // Mid-Left
    {29, 10, 0, -8}, // Rear-Left
    {6, 10, 0, -8},  // Front-Right
    {9, 10, -3, -8}, // Mid-Right
    {26, 10, 6, -8}, // Rear-Right
}; //+X → left side, -X → right side, +Y → back side, -Y → front side, Z negative → downward (ground)

Leg legs2[6] = {
    // Rotating Position
    {0, 10, 0, -8},  // Front-Left
    {3, 10, 0, -8},  // Mid-Left
    {29, 10, 0, -8}, // Rear-Left
    {6, 10, 0, -8},  // Front-Right
    {9, 10, 0, -8},  // Mid-Right
    {26, 10, 0, -8}, // Rear-Right
}; //+X → left side, -X → right side, +Y → back side, -Y → front side, Z negative → downward (ground)

// ─── Speed Control ─────────────────────────────────────────────
uint16_t moveTime = 500;
const uint16_t SPEED_MIN = 200;
const uint16_t SPEED_MAX = 1000;
const uint16_t SPEED_STEP = 50;

// ─── Robot State ───────────────────────────────────────────────
enum RobotState
{
  STATE_STOP,
  STATE_FORWARD,
  STATE_BACKWARD,
  STATE_LEFT,
  STATE_RIGHT
};
RobotState currentState = STATE_STOP;

// ─── BT Connection Tracking ────────────────────────────────────
bool btConnected = false;

// ─────────────────────────────────────────────────────────────────
//  BT callback — fires when phone connects / disconnects
// ─────────────────────────────────────────────────────────────────
void btCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
  if (event == ESP_SPP_SRV_OPEN_EVT)
  {
    btConnected = true;
    Serial.println("BT: Phone connected");
    SerialBT.println("SpiderBot V2.2 Ready!");
    SerialBT.println("Commands: F B L R S H + - ?");
  }
  if (event == ESP_SPP_CLOSE_EVT)
  {
    btConnected = false;
    currentState = STATE_STOP; // Safety: stop moving if phone disconnects
    Serial.println("BT: Phone disconnected — stopping");
  }
}

// ─────────────────────────────────────────────────────────────────
//  LOW-LEVEL: 3-servo packet → LSC-32
// ─────────────────────────────────────────────────────────────────
void moveLeg(uint8_t startID, uint16_t p1, uint16_t p2, uint16_t p3, uint16_t time)
{
  byte packet[16];
  packet[0] = 0x55;
  packet[1] = 0x55;
  packet[2] = 14;
  packet[3] = 0x03;
  packet[4] = 3;
  packet[5] = lowByte(time);
  packet[6] = highByte(time);

  packet[7] = startID;
  packet[8] = lowByte(p1);
  packet[9] = highByte(p1);
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
void moveFootXYZ(uint8_t startID, float x, float y, float z, uint16_t time)
{
  float coxaAngle = atan2(y, x);
  float R = sqrt(x * x + y * y) - L1;
  if (R < 0.5f)
    R = 0.5f;

  float D = sqrt(R * R + z * z);
  if (D > (L2 + L3))
    D = L2 + L3 - 0.1f;
  if (D < fabsf(L2 - L3))
    D = fabsf(L2 - L3) + 0.1f;

  float tibiaRad = acos(constrain((L2 * L2 + L3 * L3 - D * D) / (2 * L2 * L3), -1.0f, 1.0f));
  float femurRad = atan2(z, R) + acos(constrain((L2 * L2 + D * D - L3 * L3) / (2 * L2 * D), -1.0f, 1.0f));

  float cDeg = coxaAngle * 180.0f / PI;
  float fDeg = femurRad * 180.0f / PI;
  float tDeg = 180.0f - (tibiaRad * 180.0f / PI);

  int p1 = constrain((int)(SERVO_CENTER + (cDeg * DEG_TO_US)), 500, 2500);
  int p2 = constrain((int)(SERVO_CENTER + ((fDeg - 90.0f) * DEG_TO_US)), 500, 2500);
  int p3 = constrain((int)(SERVO_CENTER - ((tDeg - 90.0f) * DEG_TO_US)), 500, 2500);

  moveLeg(startID, p1, p2, p3, time);
}

// ─────────────────────────────────────────────────────────────────
//  GAIT: Stand / Home
// ─────────────────────────────────────────────────────────────────
void standHome()
{
  for (int i = 0; i < 6; i++)
    moveFootXYZ(legs2[i].id, legs2[i].restX, legs2[i].restY, legs2[i].restZ, moveTime);
  delay(moveTime + 50);
}

// ─────────────────────────────────────────────────────────────────
//  GAIT: Tripod walk  (+stride = forward, -stride = backward)
// ─────────────────────────────────────────────────────────────────
void tripodStep(float strideY)
{
  int A[3] = {0, 2, 4};
  int B[3] = {1, 3, 5};

  // Phase 1: A lifts & swings, B pushes
  for (int i = 0; i < 2; i++)
    moveFootXYZ(legs1[A[i]].id, legs1[A[i]].restX, legs1[A[i]].restY - strideY, legs1[A[i]].restZ - 3.0f, moveTime);
  moveFootXYZ(legs1[A[2]].id, legs1[A[2]].restX, legs1[A[2]].restY + strideY, legs1[A[2]].restZ - 3.0f, moveTime); // Opposite stride, cas the leg3 of A set rotates in opposite direction.

  for (int i = 1; i < 3; i++)
    moveFootXYZ(legs1[B[i]].id, legs1[B[i]].restX, legs1[B[i]].restY - strideY * 2, legs1[B[i]].restZ, moveTime); // Opposite stride, cas the leg1 of B set rotates in opposite direction.
  moveFootXYZ(legs1[B[0]].id, legs1[B[0]].restX, legs1[B[0]].restY + strideY * 2, legs1[B[0]].restZ, moveTime);
  delay(moveTime + 20);

  for (int i = 0; i < 2; i++) // A plants on ground
    moveFootXYZ(legs1[A[i]].id, legs1[A[i]].restX, legs1[A[i]].restY - strideY, legs1[A[i]].restZ, moveTime / 2);
  moveFootXYZ(legs1[A[2]].id, legs1[A[2]].restX, legs1[A[2]].restY + strideY, legs1[A[2]].restZ, moveTime / 2); // Opposite stride, cas the leg3 of A set rotates in opposite direction.
  delay(moveTime / 2 + 20);

  // Phase 2: B lifts & swings, A pushes
  for (int i = 1; i < 3; i++)
    moveFootXYZ(legs1[B[i]].id, legs1[B[i]].restX, legs1[B[i]].restY + strideY, legs1[B[i]].restZ - 3.0f, moveTime);
  moveFootXYZ(legs1[B[0]].id, legs1[B[0]].restX, legs1[B[0]].restY - strideY, legs1[B[0]].restZ - 3.0f, moveTime); // Opposite stride, cas the leg1 of B set rotates in opposite direction.

  for (int i = 0; i < 2; i++)
    moveFootXYZ(legs1[A[i]].id, legs1[A[i]].restX, legs1[A[i]].restY + strideY * 2, legs1[A[i]].restZ, moveTime); // Opposite stride, cas the leg3 of A set rotates in opposite direction.
  moveFootXYZ(legs1[A[2]].id, legs1[A[2]].restX, legs1[A[2]].restY - strideY * 2, legs1[A[2]].restZ, moveTime);
  delay(moveTime + 20);

  for (int i = 1; i < 3; i++) // B plants on ground
    moveFootXYZ(legs1[B[i]].id, legs1[B[i]].restX, legs1[B[i]].restY + strideY, legs1[B[i]].restZ, moveTime / 2);
  moveFootXYZ(legs1[B[0]].id, legs1[B[0]].restX, legs1[B[0]].restY - strideY, legs1[B[0]].restZ, moveTime / 2); // Opposite stride, cas the leg1 of B set rotates in opposite direction.
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
void rotateStep(float yawOffset)
{

  int A[3] = {0, 2, 4};
  int B[3] = {1, 3, 5};

  // Phase 1: A lifts & swings, B pushes
  for (int i = 0; i < 3; i++)
    moveFootXYZ(legs2[A[i]].id, legs2[A[i]].restX, legs2[A[i]].restY + yawOffset, legs2[A[i]].restZ - 3.0f, moveTime);
  for (int i = 0; i < 3; i++)
    moveFootXYZ(legs2[B[i]].id, legs2[B[i]].restX, legs2[B[i]].restY - yawOffset * 2, legs2[B[i]].restZ, moveTime);
  delay(moveTime + 20);

  for (int i = 0; i < 3; i++) // A plants
    moveFootXYZ(legs2[A[i]].id, legs2[A[i]].restX, legs2[A[i]].restY + yawOffset, legs2[A[i]].restZ, moveTime / 2);
  delay(moveTime / 2 + 20);

  // Phase 2: B lifts & swings, A pushes
  for (int i = 0; i < 3; i++)
    moveFootXYZ(legs2[B[i]].id, legs2[B[i]].restX, legs2[B[i]].restY + yawOffset, legs2[B[i]].restZ - 3.0f, moveTime);
  for (int i = 0; i < 3; i++)
    moveFootXYZ(legs2[A[i]].id, legs2[A[i]].restX, legs2[A[i]].restY - yawOffset * 2, legs2[A[i]].restZ, moveTime);
  delay(moveTime + 20);

  for (int i = 0; i < 3; i++) // B plants
    moveFootXYZ(legs2[B[i]].id, legs2[B[i]].restX, legs2[B[i]].restY + yawOffset, legs2[B[i]].restZ, moveTime / 2);
  delay(moveTime / 2 + 20);

  // Recenter all legs2 — prevents position drift accumulating each cycle
  for (int i = 0; i < 6; i++)
    moveFootXYZ(legs2[i].id, legs2[i].restX, legs2[i].restY, legs2[i].restZ, moveTime / 2);
  delay(moveTime / 2 + 20);
}

// ─────────────────────────────────────────────────────────────────
//  COMMAND HANDLER  (shared by BT and USB Serial)
// ─────────────────────────────────────────────────────────────────
void handleCommand(char cmd)
{
  switch (cmd)
  {
  case 'F':
  case 'f':
    currentState = STATE_FORWARD;
    SerialBT.println(">> Forward");
    Serial.println("CMD: Forward");
    break;
  case 'B':
  case 'b':
    currentState = STATE_BACKWARD;
    SerialBT.println(">> Backward");
    Serial.println("CMD: Backward");
    break;
  case 'L':
  case 'l':
    currentState = STATE_LEFT;
    SerialBT.println(">> Left");
    Serial.println("CMD: Left");
    break;
  case 'R':
  case 'r':
    currentState = STATE_RIGHT;
    SerialBT.println(">> Right");
    Serial.println("CMD: Right");
    break;

  case 'S':
  case 's':
  case 'H':
  case 'h':
    currentState = STATE_STOP;
    standHome();
    SerialBT.println(">> Stopped");
    Serial.println("CMD: Stop");
    break;

  case '+':
    if (moveTime > SPEED_MIN)
      moveTime -= SPEED_STEP;
    SerialBT.print(">> Speed up → ");
    SerialBT.print(moveTime);
    SerialBT.println("ms");
    break;

  case '-':
    if (moveTime < SPEED_MAX)
      moveTime += SPEED_STEP;
    SerialBT.print(">> Speed down → ");
    SerialBT.print(moveTime);
    SerialBT.println("ms");
    break;

  case '?':
    SerialBT.println("─── SpiderBot V2.2 ───");
    SerialBT.print("State : ");
    switch (currentState)
    {
    case STATE_STOP:
      SerialBT.println("STOP");
      break;
    case STATE_FORWARD:
      SerialBT.println("FORWARD");
      break;
    case STATE_BACKWARD:
      SerialBT.println("BACKWARD");
      break;
    case STATE_LEFT:
      SerialBT.println("LEFT");
      break;
    case STATE_RIGHT:
      SerialBT.println("RIGHT");
      break;
    }
    SerialBT.print("Step  : ");
    SerialBT.print(moveTime);
    SerialBT.println("ms");
    SerialBT.println("──────────────────────");
    break;

  default:
    break; // Ignore newlines, \r, etc.
  }
}

// ─────────────────────────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────────────────────────
void setup()
{
  Serial.begin(115200);
  LSC.begin(9600, SERIAL_8N1, 16, 17);

  // Start Bluetooth with device name "SpiderBot"
  SerialBT.register_callback(btCallback);
  SerialBT.begin("SpiderBot");
  delay(2000);

  Serial.println("SpiderBot V2.2 — Bluetooth started");
  Serial.println("Pair your phone with: SpiderBot");

  delay(1000);
  standHome();
}

// ─────────────────────────────────────────────────────────────────
//  MAIN LOOP
// ─────────────────────────────────────────────────────────────────
void loop()
{
  // Read BT commands
  while (SerialBT.available())
    handleCommand((char)SerialBT.read());

  // Also accept USB Serial for bench testing (no phone needed)
  while (Serial.available())

    handleCommand((char)Serial.read());

  // Execute movement
  switch (currentState)
  {
  case STATE_FORWARD:
    tripodStep(3.0f);
    break;
  case STATE_BACKWARD:
    tripodStep(-3.0f);
    break;
  case STATE_LEFT:
    rotateStep(3.0f);
    break;
  case STATE_RIGHT:
    rotateStep(-3.0f);
    break;
  case STATE_STOP:
    delay(50);
    break;
  }
}
