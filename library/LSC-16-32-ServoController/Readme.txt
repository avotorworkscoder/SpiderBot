LobotServoController Arduino Library
A specialized library for interfacing with Hiwonder/LewanSoul serial bus servo controllers (such as the LSC-16 and LSC-32) using an Arduino.


📁 Folder Contents

LobotServoController.h: Header file containing class and function declarations.

LobotServoController.cpp: Source file containing the logic and protocol implementations.

keywords.txt: Syntax highlighting rules for the Arduino IDE.

examples/: Pre-made sketches to help you get started quickly.

changelog: A record of updates and version changes.


🚀 Installation

1. ZIP Installation (Recommended)
Compress this entire LobotServoController folder into a .zip file.
Open the Arduino IDE.
Go to Sketch > Include Library > Add .ZIP Library....
Select your created zip file.

2. Manual Installation
Copy the LobotServoController folder.
Paste it into your Arduino libraries directory (usually Documents/Arduino/libraries).
Restart the Arduino IDE.


🛠️ Customization
To customize this library for your specific project:

Modify Logic: Edit the .cpp file to change how serial commands are formatted or sent.

Add Functions: Declare new methods in the .h file and define them in the .cpp file.

Update Keywords: If you add new functions, add them to keywords.txt as KEYWORD2 to enable syntax highlighting in the IDE.



📖 Usage Example

#include <LobotServoController.h>

// Initialize the controller
LobotServoController myController(Serial); 

void setup() {
  Serial.begin(115200);
}

void loop() {
  // Example: Move Servo ID 1 to position 500 at speed 1000
  myController.moveServo(1, 500, 1000);
  delay(2000);
}
