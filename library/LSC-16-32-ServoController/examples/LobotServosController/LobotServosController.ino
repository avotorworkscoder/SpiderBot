/**
 * FileName:     FlexibleLobotServocontrollerExample
 * Author:       Amit Parihar
 * Github:       https://github.com/avotorworkscoder
 * Date:         2026/02/04

 * Description: Read the attached servo library <LobotServoController.cpp> and <LobotServoController.h> to understand the logic behind each function and method, to use it more
  flexibily in all your projects requiring complex tasks and logics.
**/

#include <LobotServoController.h>

LobotServoController myse;  // No serial initialised, Default serial communication: 0RX, 0TX

void setup() {
  pinMode(13,OUTPUT);
  Serial.begin(9600);
  while(!Serial);     // Wait for serial monitor to open
  digitalWrite(13,HIGH);    // Blink the Inbuilt LED

}

void loop() {
  myse.runActionGroup(100,0);  // Continuously run action group 100
  delay(1000);
  myse.stopActionGroup(); // Stop action group execution
  delay(1000);
  myse.setActionGroupSpeed(100,200); // Set action group 100 speed to 200%
  delay(1000);
  myse.runActionGroup(100,5);  // Run action group 100 five times
  delay(1000);
  myse.stopActionGroup(); // Stop action group execution
  delay(1000);
  myse.moveServo(0,1500,1000); // Move servo 0 to position 1500 in 1000ms
  delay(1000);
  myse.moveServo(2,800,1000); // Move servo 2 to position 800 in 1000ms
  delay(1000);
  myse.moveServos(5,1000,0,1300,2,700,4,600,6,900,8,790);  //
    // Control 5 servos, move time 1000ms, servo 0 to position 1300, servo 2 to position 700, servo 4 to position 600,
    // servo 6 to position 900, servo 8 to position 790
  delay(1000);

LobotServo servos[2];   // Servo ID and position structure array
  servos[0].ID = 2;       // Servo 2
  servos[0].Position = 1400;  // Position 1400
  servos[1].ID = 4;       // Servo 4
  servos[1].Position = 700;   //Position 700
  
  myse.moveServos(servos, 2, 1000); // Control two servos, movement time 1000ms, IDs and positions are specified by servos
}