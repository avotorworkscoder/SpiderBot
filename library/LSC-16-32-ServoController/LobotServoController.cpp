/******************************************************
 * FileName:     LobotServoController.cpp
 * Author:       Amit Parihar
 * Github:       https://github.com/avotorworkscoder
 * Date:         04/02/2026
 * Description:  Lobot servo controller secondary development library, this file contains
the specific implementation of this library
*****************************************************/

#include "LobotServoController.h"
#include <Arduino.h>

#define GET_LOW_BYTE(A) (uint8_t)((A))                           // Macro function to get the low eight bits of A
#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8)                     // Macro function to get the high eight bits of A
#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B)) // Macro function to combine A as the high eight bits and B as the low eight bits into a 16-bit integer
#define TO_HEX_BYTE(A) ((uint8_t)(A))

LobotServoController::LobotServoController() // Default Constructor: Its job is to "set the stage" by initializing all variables to a safe starting state as soon as you create an instance of the controller.
{
    // Initialize the running action group number to 0xFF, the number of runs to 0, the running flag to false, and the battery voltage to 0
    numOfActinGroupRunning = 0xFF; // 0xFF: 'null' value. This means "no action group is currently playing.
    actionGroupRunTimes = 0;       // This means "no action group is currently playing."
    isRunning = false;             // Explicitly states that the robot is currently idle.
    batteryVoltage = 0;            // Sets the voltage variable to zero until the first actual reading is taken.

#if defined(__AVR_ATmega32U4__) // For  Arduino Leonardo, Arduino Micro, Arduino Esplora, and various "Pro Micro" compatible boards
    SerialX = &Serial1;
#else
    SerialX = &Serial;
#endif
}

LobotServoController::LobotServoController(HardwareSerial &A) // This is a parameterized constructor that first runs the default setup and then assigns a specific serial port (like Serial2) as passed to communicate with the servos.
{
    LobotServoController();
    SerialX = &A;
}
LobotServoController::~LobotServoController() // This is a destructor used to "destroy" the object; it is currently empty because there is no complex memory to clean up manually.

{
}

/*********************************************************************************
* Function:  moveServo
* Description： Controls a single servo to rotate
* Parameters:   servoID: Servo ID, Position: Target position, Time: Rotation time
Servo ID range: 0 <= servoID <= 31, Time range: Time > 0
* Return:       No return value
* Others:
**********************************************************************************/
void LobotServoController::moveServo(uint8_t servoID, uint16_t Position, uint16_t Time) // Sends a command in the form of data packet
{
    uint8_t buf[11];
    if (servoID > 31 || !(Time > 0))
    { // Servo ID cannot be greater than 31, can be modified according to the corresponding control board
        return;
    }
    buf[0] = FRAME_HEADER; // Fill frame header
    buf[1] = FRAME_HEADER;
    buf[2] = 8;                       // Data length = length + command + servoCount + 2 time state + number of servos to control * 3 , here = 5 + 1 * 3
    buf[3] = CMD_SERVO_MOVE;          // Fill servo movement command
    buf[4] = 1;                       // Number of servos to control
    buf[5] = GET_LOW_BYTE(Time);      // Fill the low eight bits of the time
    buf[6] = GET_HIGH_BYTE(Time);     // Fill the high eight bits of the time
    buf[7] = servoID;                 // Servo ID
    buf[8] = GET_LOW_BYTE(Position);  // Fill the low eight bits of the target position
    buf[9] = GET_HIGH_BYTE(Position); // Fill the high eight bits of the target position

    SerialX->write(buf, 10); // Sends a 10 byte packet
}

/*********************************************************************************
* Function:  moveServos
* Description： Controls multiple servos to rotate
* Parameters:   servos[]: servo structure array, Num: number of servos, Time: rotation time
0 < Num <= 32, Time > 0
* Return:       No return value
* Others:
**********************************************************************************/
void LobotServoController::moveServos(LobotServo servos[], uint8_t Num, uint16_t Time)
{
    uint8_t buf[103]; // Create buffer
    if (Num < 1 || Num > 32 || !(Time > 0))
    {
        return; // The number of servos cannot be zero or greater than 32, and the time cannot be zero
    }
    buf[0] = FRAME_HEADER; // Fill frame header
    buf[1] = FRAME_HEADER;
    buf[2] = Num * 3 + 5;         // Data length = number of servos to control * 3 + 5
    buf[3] = CMD_SERVO_MOVE;      // Fill in the servo movement command
    buf[4] = Num;                 // Number of servos to control
    buf[5] = GET_LOW_BYTE(Time);  // Get the low byte of the time
    buf[6] = GET_HIGH_BYTE(Time); // Get the high byte of the time
    uint8_t index = 7;
    for (uint8_t i = 0; i < Num; i++)
    {                                                     // Loop to fill in servo ID and corresponding target position
        buf[index++] = servos[i].ID;                      // Fill in the servo ID
        buf[index++] = GET_LOW_BYTE(servos[i].Position);  // Fill in the low byte of the target position
        buf[index++] = GET_HIGH_BYTE(servos[i].Position); // Fill in the high byte of the target position
    }
    SerialX->write(buf, buf[2] + 2); // Send the frame, and length is data length(at 2nd index) + two bytes of frame header
}

/*********************************************************************************
 * Function:  moveServos
 * Description： Controls multiple servos to rotate
 * Parameters:   Num: Number of servos, Time: Rotation time, ...: Servo ID, rotation angle, Servo ID, rotation angle, and so on
 * Return:       No return value
 * Others:
 **********************************************************************************/
void LobotServoController::moveServos(uint8_t Num, uint16_t Time, ...)
{
    uint8_t buf[128];
    va_list arg_ptr;
    va_start(arg_ptr, Time); // Get the address of the first variable parameter
    if (Num < 1 || Num > 32 || (!(Time > 0)))
    {
        return; // The number of servos cannot be zero or greater than 32, the time cannot be zero, and the variable parameters cannot be empty
    }
    buf[0] = FRAME_HEADER; // Fill in the frame header
    buf[1] = FRAME_HEADER;
    buf[2] = Num * 3 + 5;         // Data length = number of servos to control * 3 + 5
    buf[3] = CMD_SERVO_MOVE;      // Servo movement command
    buf[4] = Num;                 // Number of servos to control
    buf[5] = GET_LOW_BYTE(Time);  // Get the low byte of the time
    buf[6] = GET_HIGH_BYTE(Time); // Get the high byte of the time
    uint8_t index = 7;
    for (uint8_t i = 0; i < Num; i++)
    {                                             // Get and fill the servo ID and corresponding target position from the variable arguments
        uint16_t tmp = va_arg(arg_ptr, uint16_t); // Get the servo ID from the variable arguments
        buf[index++] = GET_LOW_BYTE(tmp);         // It seems that integer variable arguments in avrgcc are 16-bit
        // Then get its low byte
        uint16_t pos = va_arg(arg_ptr, uint16_t); // Get the corresponding target position from the variable arguments
        buf[index++] = GET_LOW_BYTE(pos);         // Fill the low byte of the target position
        buf[index++] = GET_HIGH_BYTE(pos);        // Fill the high byte of the target position
    }
    va_end(arg_ptr);                 // Set arg_ptr to null
    SerialX->write(buf, buf[2] + 2); // Send the frame
}
/*********************************************************************************
 * Function:  runActionGroup
 * Description: Runs the specified action group
 * Parameters:  NumOfAction: Action group number, Times: Number of executions
 * Return:      None
 * Others:      Times = 0 for infinite loop
 **********************************************************************************/
void LobotServoController::runActionGroup(uint8_t numOfAction, uint16_t Times)
{
    uint8_t buf[7];
    buf[0] = FRAME_HEADER; // Fill frame header
    buf[1] = FRAME_HEADER;
    buf[2] = 5;                    // Data length, number of data bytes in the data frame excluding the frame header, this command is fixed at 5
    buf[3] = CMD_ACTION_GROUP_RUN; // Fill in the run action group command
    buf[4] = numOfAction;          // Fill in the action group number to run
    buf[5] = GET_LOW_BYTE(Times);  // Get the low eight bits of the number of times to run
    buf[6] = GET_HIGH_BYTE(Times); // Get the high eight bits of the number of times to run
    SerialX->write(buf, 7);        // Send data frame
}

/*********************************************************************************
 * Function:  stopActionGroup
 * Description: Stops the action group from running
 * Parameters:  None
 * Return:      None
 * Others:
 **********************************************************************************/
void LobotServoController::stopActionGroup(void)
{
    uint8_t buf[4];
    buf[0] = FRAME_HEADER; // Fill frame header
    buf[1] = FRAME_HEADER;
    buf[2] = 2;                     // Data length, number of data bytes in the data frame excluding the frame header, this command is fixed at 2
    buf[3] = CMD_ACTION_GROUP_STOP; // Fill in the stop running action group command

    SerialX->write(buf, 4); // Send data frame
}

/*********************************************************************************
 * Function:  setActionGroupSpeed
 * Description: Sets the running speed of the specified action group
 * Parameters:  NumOfAction: Action group number, Speed: Target speed
 * Return:      None
 * Others:
 **********************************************************************************/

void LobotServoController::setActionGroupSpeed(uint8_t numOfAction, uint16_t Speed)
{
    uint8_t buf[7];
    buf[0] = FRAME_HEADER; // Fill in the frame header
    buf[1] = FRAME_HEADER;
    buf[2] = 5;                      // Data length, the number of data bytes in the data frame excluding the frame header, this command is fixed at 5
    buf[3] = CMD_ACTION_GROUP_SPEED; // Fill in the command to set the action group speed
    buf[4] = numOfAction;            // Fill in the action group number to be set
    buf[5] = GET_LOW_BYTE(Speed);    // Get the low eight bits of the target speed
    buf[6] = GET_HIGH_BYTE(Speed);   // Get the high eight bits of the target speed

    SerialX->write(buf, 7); // Send the data frame
}

/*********************************************************************************
 * Function:  setAllActionGroupSpeed
 * Description： Sets the running speed of all action groups
 * Parameters:   Speed: Target speed
 * Return:       No return value
 * Others:
 **********************************************************************************/

void LobotServoController::setAllActionGroupSpeed(uint16_t Speed)
{
    setActionGroupSpeed(0xFF, Speed); // Call the action group speed setting, when the group number is 0xFF, set the speed of all groups
}

/*********************************************************************************
 * Function:  getBatteryVoltage
 * Description： Sends the command to get the battery voltage
 * Parameters:   No input parameters
 * Return:       No return value
 * Others:
 **********************************************************************************/

void LobotServoController::getBatteryVoltage()
{
    uint8_t buf[4];
    buf[0] = FRAME_HEADER; // Fill in the frame header
    buf[1] = FRAME_HEADER;
    buf[2] = 2;                       // Data length, the number of data bytes in the data frame excluding the frame header, this command is fixed at 2
    buf[3] = CMD_GET_BATTERY_VOLTAGE; // Fill in the battery voltage command

    SerialX->write(buf, 4); // Send the data frame
}

/*********************************************************************************
 * Function:  readServos
 * Description： Sends request for Servo position
 * Parameters:   Num: Servo ID parameters
 * Return: No return value
 * Others:
 **********************************************************************************/
void LobotServoController::readServos(uint8_t Num, ...)
{
    uint8_t buf[128];
    va_list arg_ptr;
    va_start(arg_ptr, Num);

    if (Num < 1 || Num > 32)
    {
        return;
    }
    buf[0] = FRAME_HEADER;
    buf[1] = FRAME_HEADER;
    buf[2] = Num + 3; // Length N = quantity + 3 [cite: 94]
    buf[3] = CMD_MULT_SERVO_POS_READ;
    buf[4] = Num; // Parameter 1: quantity [cite: 95]
    uint8_t index = 5;
    for (uint8_t i = 0; i < Num; i++)
    {
        int tp = va_arg(arg_ptr, int); // Get the servo ID from the variable arguments
        buf[index++] = tp;             // Fill IDs [cite: 96, 102]
    }

    va_end(arg_ptr);
    SerialX->write(buf, buf[2] + 2); // Send full packet
}

/*********************************************************************************
 * Function:  receiveHandle
 * Description： Handles serial port received data
 * Parameters:   No input parameters
 * Return: No return value
 * Others:
 **********************************************************************************/
void LobotServoController::receiveHandle()
{
    uint8_t buf[16];
    static uint8_t len = 0;
    static uint8_t getHeader = 0;
    if (!SerialX->available())
        return;
    // Return if no data is available
    do
    {
        switch (getHeader)
        {
        case 0:
            if (SerialX->read() == FRAME_HEADER) // Is the first byte 0x55?
                getHeader = 1;                   // Yes? Advance to the next step.
            break;                               // Exit the switch and wait for the next byte.

        case 1:
            if (SerialX->read() == FRAME_HEADER) // Is the second byte also 0x55?
                getHeader = 2;                   // Yes? We found a real message!
            else
                getHeader = 0; // No? It was a fake alarm. Reset to start.
            break;             // Exit the switch.

        case 2:
            len = SerialX->read(); // Read how many bytes are coming next.
            getHeader = 3;         // Advance.
            break;                 // Exit the switch.

        case 3:
            if (SerialX->readBytes(buf, len - 1) > 0) // Try to grab all the data at once.
                getHeader = 4;                        // Success! Move to Case 4.
            else
            {
                len = 0;
                getHeader = 0; // Failed to read? Reset everything.
                break;         // Emergency Exit.
            }
            // NOTICE: There is no 'break' here!
            // This is called "Fall-through." Since we have the data,
            // we go straight into Case 4 without waiting.

        case 4:
            switch (buf[0])
            {
            case BATTERY_VOLTAGE:                            // Battery voltage command
                batteryVoltage = BYTE_TO_HW(buf[2], buf[1]); // Combine high and low bytes to form battery voltage
                break;
            case ACTION_GROUP_RUNNING:                            // Action group is running
                numOfActinGroupRunning = buf[1];                  // Get the running action group number
                actionGroupRunTimes = BYTE_TO_HW(buf[3], buf[2]); // Combine high and low bytes to form the number of runs
                isRunning = true;                                 // Set the running flag to true
                break;
            case ACTION_GROUP_STOPPED:         // Action group stopped
            case ACTION_GROUP_COMPLETE:        // Action group completed
                isRunning = false;             // Set the running flag to false
                numOfActinGroupRunning = 0xFF; // Set the running action group number to 0xFF
                actionGroupRunTimes = 0;       // Set the number of runs to 0
                break;
                // Inside LobotServoController::receiveHandle() -> switch (buf[0])
            case CMD_MULT_SERVO_POS_READ:
            {
                demo = 10;
                uint8_t numServos = buf[1]; // Parameter 1: quantity
                for (uint8_t i = 0; i < numServos; i++)
                {
                    id = buf[2 + (i * 3)];                                // Servo ID
                    pos = BYTE_TO_HW(buf[4 + (i * 3)], buf[3 + (i * 3)]); // Position

                    // Debug output to see the read values
                    Serial.print("Servo ID: ");
                    Serial.print(id);
                    Serial.print(" Position: ");
                    Serial.println(pos);
                }

                break;
            }
            default:
                break;
            }

        default:
            len = 0;       // Clear the length.
            getHeader = 0; // Reset the progress marker to 0 for the next message.
            break;         // Final exit.
        }
    } while (SerialX->available());
}