/******************************************************
 * FileName:      LobotServoController.h
 * Author:        Amit Parihar
 * Github:        https://github.com/avotorworkscoder
 * Date:          04/02/2026
 * Description:   Macro definitions and classes for the Hiwonder/ Lobot servo controller development library
 *****************************************************/

#ifndef LOBOTSERVOCONTROLLER_H // Stands for: If not defined -> This command tells the compiler to check if a specific identifier (in this case, LOBOTSERVOCONTROLLER_H) has already been defined earlier in the compilation process.
#define LOBOTSERVOCONTROLLER_H // Prevents Double Inclusion: If a header file is included multiple times (directly or indirectly) in a single project, it can cause "redefinition" errors where the compiler thinks you are trying to define the same class or variable twice.

#include <Arduino.h>

// Sending commands
#define FRAME_HEADER 0x55            // Frame header
#define CMD_SERVO_MOVE 0x03          // Servo movement command
#define CMD_ACTION_GROUP_RUN 0x06    // Run action group command
#define CMD_ACTION_GROUP_STOP 0x07   // Stop action group command
#define CMD_ACTION_GROUP_SPEED 0x0B  // Set action group speed command
#define CMD_GET_BATTERY_VOLTAGE 0x0F // Get battery voltage command

// Receiving commands
#define BATTERY_VOLTAGE 0x0F         // Battery voltage
#define ACTION_GROUP_RUNNING 0x06    // Action group is running
#define ACTION_GROUP_STOPPED 0x07    // Action group is stopped
#define ACTION_GROUP_COMPLETE 0x08   // Action group completed
#define CMD_MULT_SERVO_POS_READ 0x15 // Read multiple Servo Position

struct LobotServo    // Creates a custom Data type. Instead of just using a standard int, you can now create a variable of type LobotServo that holds two distinct values.
{                    // Servo ID and position structure
  uint8_t ID;        // Servo ID (0-255 value)
  uint16_t Position; // Servo data (0-65,535 value)
};
/**
 * HOW TO USE IT:
  LobotServo myMotor;
  myMotor.ID = 5;          // Assign ID 5 to this motor
  myMotor.Position = 1500; // Set its position to the midpoint
 */

class LobotServoController
{
public:
  LobotServoController();                  // A Default Constructor that initializes the object without specific settings.
  LobotServoController(HardwareSerial &A); // A Parameterized Constructor that tells the controller which hardware serial port (e.g., Serial1 or Serial2) to use for communication.
  ~LobotServoController();                 // A Destructor that cleans up the object when it is no longer needed.

  // METHODS
  void moveServo(uint8_t servoID, uint16_t Position, uint16_t Time); // Moves one specific servo to a target position over a set duration.
  void moveServos(LobotServo servos[], uint8_t Num, uint16_t Time);  // Moves multiple servos at once using an array of the LobotServo structs we discussed earlier.
  void moveServos(uint8_t Num, uint16_t Time, ...);                  // A Variadic Function that lets you list out multiple servo IDs and positions directly in the function call without an array.

  // ACTION GROUPS
  void runActionGroup(uint8_t NumOfAction, uint16_t Times);      // Triggers a pre-saved sequence of movements (like "wave" or "walk") and repeats it a specific number of times.
  void stopActionGroup(void);                                    // Immediately halts any running sequence.
  void setActionGroupSpeed(uint8_t NumOfAction, uint16_t Speed); // Adjusts how fast a specific sequence (or all sequences) should play back.
  void setAllActionGroupSpeed(uint16_t Speed);                   // Adjusts how fast all sequences should play back.

  // STATUS & UTILITY
  void getBatteryVoltage(void);      // Requests the current power level from the control board.
  void receiveHandle(void);          // A crucial "background" function that processes incoming data from the serial port to update the controller's status.
  void readServos(uint8_t Num, ...); // Request positions for N servos

public:
  uint8_t numOfActinGroupRunning; // Sequence number of the action group currently running
  uint16_t actionGroupRunTimes;   // Number of times the currently running action group has run
  bool isRunning;                 // Is an action group running?
  uint16_t batteryVoltage;        // Control board battery voltage
  uint8_t id;
  uint16_t pos;
  int demo;

  HardwareSerial *SerialX; // A <Pointer> to the serial port being used to send commands.
};
#endif