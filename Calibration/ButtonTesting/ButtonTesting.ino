//*             Foot contact sensors (digital, active-LOW w/ pull-ups):
//*             FL=34  ML=35  RL=32  FR=33  MR=25  RR=26
//*             (Use limit switches or FSRs w/ comparator to GND)
const int buttonPin = 35; // GPIO pins
const int ledPin = 2;    // Built-in LED

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  // Use INPUT_PULLUP to avoid needing an external resistor
  pinMode(buttonPin, INPUT_PULLUP); 
}

void loop() {
  int buttonState = digitalRead(buttonPin);
  
  // Button is pressed when it's LOW
  if (buttonState == LOW) {
    digitalWrite(ledPin, HIGH); // Turn LED on
    Serial.println("Button Pressed");
  } else {
    digitalWrite(ledPin, LOW);  // Turn LED off
  }
  delay(100);
}
