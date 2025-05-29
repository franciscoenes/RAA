#include <Servo.h>

// Pin definition for base servo
const int BASE_SERVO_PIN = 9;  // Change this to match your wiring

// Create servo object
Servo baseServo;

// Variables for serial communication
String inputString = "";      // String to hold incoming data
bool stringComplete = false;  // Whether the string is complete
int lastAngle = 0;           // Store last angle, start at center position (0 = center)

void setup() {
  // Initialize serial communication
  Serial.begin(57600);  // Match the baud rate with Python code
  
  // Reserve 200 bytes for the inputString
  inputString.reserve(200);
  
  // Attach servo
  baseServo.attach(BASE_SERVO_PIN);
  
  // Set initial position (0 degrees = center position)
  baseServo.write(0);
  delay(1000);  // Give servo time to reach position
  
  // Debug message
  Serial.println("Base servo initialized at center position (0°)");
}

void loop() {
  // When a newline arrives, process the angle
  if (stringComplete) {
    // Convert string to integer
    int angle = inputString.toInt();
    
    // Ensure angle is between 0 (center) and 90 (left)
    angle = constrain(angle, 0, 90);
    
    // Only move if angle has changed
    if (angle != lastAngle) {
      // Debug print
      Serial.print("Moving to angle: ");
      Serial.print(angle);
      Serial.println("°");
      
      // Move servo to position
      baseServo.write(angle);
      lastAngle = angle;
    }
    
    // Clear the string for next input
    inputString = "";
    stringComplete = false;
  }
}

// SerialEvent occurs whenever new data comes in the hardware serial RX
void serialEvent() {
  while (Serial.available()) {
    // Get the new byte
    char inChar = (char)Serial.read();
    
    // Add it to the inputString if it's not a newline
    if (inChar != '\n') {
      inputString += inChar;
    }
    // If the incoming character is a newline, set a flag so the main loop can
    // process the string
    else {
      stringComplete = true;
    }
  }
} 