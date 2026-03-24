#include <Arduino.h>

// UART2 on ESP32: RX=16, TX=17
HardwareSerial LSC(2); 

void setup() {

  Serial.begin(115200); // Debug to PC
  // LSC board requires exactly 9600 baud
  LSC.begin(9600, SERIAL_8N1, 16, 17);  // 8N1: 8bit/ No Parity bit/ 1 stop bit
  
  delay(1000);
  Serial.println("LSC-32 Hex Control Ready");
}


void loop() {
  

  // 1. Move Servo 1 to Position 2000 in 1000ms
  //moveServoHex(0, 2500, 1000); 
  //delay(2000);

  moveServoHex(26, 1500, 1000); 

  moveServoHex(27, 500, 1000); 

  moveServoHex(28, 1500, 1000); 

  // 2. Move Servo 1 to Position 1000 in 1000ms
  //moveServoHex(0, 500, 1000); 
  //delay(2000);

}

/**
 * Sends a CMD_SERVO_MOVE command (Value: 3)
 * Format: 0x55 0x55 | Length | 0x03 | NumServos | TimeL | TimeH | ID | PosL | PosH
 */
void moveServoHex(uint8_t id, uint16_t position, uint16_t time) {
  byte packet[10];

  packet[0] = 0x55;              // Frame Header
  packet[1] = 0x55;              // Frame Header
  packet[2] = 0x08;              // Data Length: (1 Servo * 3) + 5 = 8
  packet[3] = 0x03;              // Command: CMD_SERVO_MOVE
  packet[4] = 0x01;              // Parameter: Number of servos to control
  
  // Time (Low byte, then High byte)
  packet[5] = lowByte(time);     
  packet[6] = highByte(time);    
  
  packet[7] = id;                // Servo ID
  
  // Angle Position (Low byte, then High byte)
  packet[8] = lowByte(position); 
  packet[9] = highByte(position);

  LSC.write(packet, 10);         // Send the 10-byte packet to the board
  Serial.println("Moving motor " + String(id) + " to position " + String(position) + "...");
}