#include <Arduino.h>
#include <math.h>

HardwareSerial LSC(2);

// GEOMETRY (cm)
const float L1 = 6.0;  // coxa
const float L2 = 8.5;  // femur
const float L3 = 14.5; // tibia
const int SERVO_CENTER = 1500;
const float DEG_TO_US = 11.11; // 2000us / 180deg = 11.11

void setup()
{
  Serial.begin(115200);
  LSC.begin(9600, SERIAL_8N1, 16, 17);
  delay(1000);
  Serial.println("LSC-32 Hex Control Ready");
}

// NEW: Function to move 3 servos (one full leg) in one packet
void moveLeg(uint8_t startID, uint16_t p1, uint16_t p2, uint16_t p3, uint16_t time)
{
  byte packet[16];
  packet[0] = 0x55;
  packet[1] = 0x55;
  packet[2] = 14;   // Length: (3 servos * 3) + 5 = 14
  packet[3] = 0x03; // CMD_SERVO_MOVE
  packet[4] = 3;    // Number of servos

  packet[5] = lowByte(time);
  packet[6] = highByte(time);

  // Servo 1 (Coxa)
  packet[7] = startID;
  packet[8] = lowByte(p1);
  packet[9] = highByte(p1);

  // Servo 2 (Femur)
  packet[10] = startID + 1;
  packet[11] = lowByte(p2);
  packet[12] = highByte(p2);

  // Servo 3 (Tibia)
  packet[13] = startID + 2;
  packet[14] = lowByte(p3);
  packet[15] = highByte(p3);

  LSC.write(packet, 16);
}

void moveFootXYZ(uint8_t startID, float x, float y, float z, uint16_t time)
{
  float coxaAngle = atan2(y, x);
  float horizontalDist = sqrt(x * x + y * y);
  float R = horizontalDist - L1;
  if (R < 0.5)
    R = 0.5; // avoid singularity
  float D = sqrt(R * R + z * z);

  Serial.print("R: ");
  Serial.print(R);
  Serial.print(", D: ");
  Serial.print(D);

  // 1. Workspace Safety Check
  if (D > (L2 + L3))
    D = L2 + L3 - 0.1;
  if (D < abs(L2 - L3))
    D = abs(L2 - L3) + 0.1; // Prevent "self-collision"

  // 2. Law of Cosines
  float cosT = (L2 * L2 + L3 * L3 - D * D) / (2 * L2 * L3);
  float tibiaRad = acos(constrain(cosT, -1.0, 1.0)); // Constrain to valid range

  float a = atan2(z, R);
  float cosF = (L2 * L2 + D * D - L3 * L3) / (2 * L2 * D);
  float b = acos(constrain(cosF, -1.0, 1.0));
  float femurRad = a + b;

  // 3. Convert to Degrees
  float cDeg = coxaAngle * 180.0 / PI;
  float fDeg = (femurRad * 180.0 / PI);
  float tDeg = 180 - (tibiaRad * 180.0 / PI);

  Serial.print(", cDeg: ");
  Serial.print(cDeg);
  Serial.print(", fDeg: ");
  Serial.print(fDeg);
  Serial.print(", tDeg: ");
  Serial.print(tDeg);

  // 4. Calculate Pulses
  float femurOffset = -90.0; // adjust after testing
  float tibiaOffset = -90.0; // adjust after testing

  int p1 = SERVO_CENTER + (cDeg * DEG_TO_US);
  int p2 = SERVO_CENTER + ((fDeg + femurOffset) * DEG_TO_US);
  int p3 = SERVO_CENTER - ((tDeg + tibiaOffset) * DEG_TO_US);

  // 5. Constrain Pulses (Critical for LSC-32)
  p1 = constrain(p1, 500, 2500);
  p2 = constrain(p2, 500, 2500);
  p3 = constrain(p3, 500, 2500);

  // DEBUG: Check if p2 (Femur) is hitting the limit
  Serial.print("Target Pulses -> Coxa: ");
  Serial.print(p1);
  Serial.print(" | Femur: ");
  Serial.print(p2);
  Serial.print(" | Tibia: ");
  Serial.println(p3);

  moveLeg(startID, p1, p2, p3, time);
}

void loop()
{
  // Try reaching backward and slightly down
  // X=10 (reach), Y=5 (backward), Z=-8 (on the floor)
  moveFootXYZ(0, 10, 8, -8, 500);
  moveFootXYZ(9, 10, -8, -8, 500);
  moveFootXYZ(29, 10, 8, -8, 500);
  delay(1000);

  // Lift the leg up
  moveFootXYZ(0, 10, 0, -12, 500);
  moveFootXYZ(9, 10, 0, -12, 500);
  moveFootXYZ(29, 10, 0, -12, 500);
  delay(1000);

  // X=10, Y=0(Straight), Z=-8 (on the floor )
  moveFootXYZ(0, 10, 0, -8, 500);
  moveFootXYZ(9, 10, 0, -8, 500);
  moveFootXYZ(29, 10, 0, -8, 500);
  delay(1000);

  // X=10 (reach), Y=5 (backward), Z=-8 (on the floor)
  moveFootXYZ(3, 10, 8, -8, 500);
  moveFootXYZ(6, 10, -8, -8, 500);
  moveFootXYZ(26, 10, -8, -8, 500);
  delay(1000);

  // Lift the leg up
  moveFootXYZ(3, 10, 0, -12, 500);
  moveFootXYZ(6, 10, 0, -12, 500);
  moveFootXYZ(26, 10, 0, -12, 500);
  delay(1000);

  // X=10, Y=0(Straight), Z=-8 (on the floor )
  moveFootXYZ(3, 10, 0, -8, 500);
  moveFootXYZ(6, 10, 0, -8, 500);
  moveFootXYZ(26, 10, 0, -8, 500);
  delay(1000);
}
