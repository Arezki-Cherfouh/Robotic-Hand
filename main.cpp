/*
 * Arduino Robotic Hand Controller with Servo Motors
 * Receives finger states via USB serial and controls servo motors
 * to mimic real hand movements in a physical cartoon/robotic hand
 * 
 * Hardware Requirements:
 * - Arduino board (Uno, Mega, Nano, etc.)
 * - 5x Servo motors (SG90, MG90S, or similar)
 * - External 5V power supply for servos (recommended for 5 servos)
 * - Robotic hand mechanism (3D printed or mechanical)
 * 
 * Wiring:
 * Servo 1 (Thumb):  Pin 3
 * Servo 2 (Index):  Pin 5
 * Servo 3 (Middle): Pin 6
 * Servo 4 (Ring):   Pin 9
 * Servo 5 (Pinky):  Pin 10
 * 
 * Data Format: "T,I,M,R,P\n"
 * T = Thumb (0=closed, 1=open)
 * I = Index (0=closed, 1=open)
 * M = Middle (0=closed, 1=open)
 * R = Ring (0=closed, 1=open)
 * P = Pinky (0=closed, 1=open)
 */

#include <Arduino.h>
#include <Servo.h>

// Servo objects for each finger
Servo thumbServo;
Servo indexServo;
Servo middleServo;
Servo ringServo;
Servo pinkyServo;

// Array to hold all servos for easy iteration
Servo fingerServos[5];

// Servo pin assignments
const int SERVO_PINS[5] = {3, 5, 6, 9, 10};  // Thumb, Index, Middle, Ring, Pinky

// Servo angle configurations
// Adjust these values based on your mechanical hand design
// CLOSED_ANGLE: angle when finger is fully closed (fist)
// OPEN_ANGLE: angle when finger is fully extended (open hand)
const int CLOSED_ANGLE[5] = {0, 0, 0, 0, 0};       // Adjust per finger if needed
const int OPEN_ANGLE[5] = {180, 180, 180, 180, 180}; // Adjust per finger if needed

// Current target angles for smooth movement
int targetAngles[5] = {90, 90, 90, 90, 90};  // Start at middle position
int currentAngles[5] = {90, 90, 90, 90, 90};

// Movement speed (degrees per update cycle)
const int MOVE_SPEED = 5;  // Lower = smoother but slower, Higher = faster but jerky

// Finger state variables
bool fingerStates[5] = {false, false, false, false, false};
String fingerNames[5] = {"Thumb", "Index", "Middle", "Ring", "Pinky"};

// Buffer for serial data
String inputString = "";
bool stringComplete = false;

// Timing for smooth servo movement
unsigned long lastMoveTime = 0;
const int MOVE_INTERVAL = 15;  // milliseconds between servo updates

// Function declarations
void parseFingerData(String data);
void updateServoPositions();
void moveServosSmooth();
void calibrateServos();

void setup() {
  // Initialize serial communication at 9600 baud
  Serial.begin(9600);
  
  // Reserve space for input string
  inputString.reserve(20);
  
  // Attach servos to pins
  thumbServo.attach(SERVO_PINS[0]);
  indexServo.attach(SERVO_PINS[1]);
  middleServo.attach(SERVO_PINS[2]);
  ringServo.attach(SERVO_PINS[3]);
  pinkyServo.attach(SERVO_PINS[4]);
  
  // Store servos in array for easy access
  fingerServos[0] = thumbServo;
  fingerServos[1] = indexServo;
  fingerServos[2] = middleServo;
  fingerServos[3] = ringServo;
  fingerServos[4] = pinkyServo;
  
  // Initialize all servos to middle position
  for (int i = 0; i < 5; i++) {
    fingerServos[i].write(90);
    delay(100);
  }
  
  Serial.println("========================================");
  Serial.println("  Robotic Hand Controller - ACTIVE");
  Serial.println("========================================");
  Serial.println("Servos initialized on pins:");
  Serial.println("  Thumb:  Pin 3");
  Serial.println("  Index:  Pin 5");
  Serial.println("  Middle: Pin 6");
  Serial.println("  Ring:   Pin 9");
  Serial.println("  Pinky:  Pin 10");
  Serial.println("========================================");
  Serial.println("Waiting for hand tracking data...");
  Serial.println();
  
  delay(1000);
  
  // Optional: Run calibration sequence
  // calibrateServos();
}

void loop() {
  // Check if new data is available
  if (stringComplete) {
    parseFingerData(inputString);
    updateServoPositions();
    
    // Clear the string
    inputString = "";
    stringComplete = false;
  }
  
  // Smoothly move servos toward target positions
  moveServosSmooth();
}

/*
 * Serial Event Handler - called when data is available
 */
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}

/*
 * Parse incoming finger data
 * Expected format: "1,0,1,0,1" (thumb, index, middle, ring, pinky)
 */
void parseFingerData(String data) {
  int fingerIndex = 0;
  int startPos = 0;
  
  // Parse comma-separated values
  for (int i = 0; i <= data.length(); i++) {
    if (i == data.length() || data.charAt(i) == ',') {
      if (fingerIndex < 5) {
        String value = data.substring(startPos, i);
        value.trim();
        fingerStates[fingerIndex] = (value == "1");
        fingerIndex++;
      }
      startPos = i + 1;
    }
  }
}

/*
 * Update target servo positions based on finger states
 */
void updateServoPositions() {
  for (int i = 0; i < 5; i++) {
    if (fingerStates[i]) {
      // Finger is open - set to open angle
      targetAngles[i] = OPEN_ANGLE[i];
    } else {
      // Finger is closed - set to closed angle
      targetAngles[i] = CLOSED_ANGLE[i];
    }
  }
  
  // Debug output
  Serial.print("Hand State: ");
  for (int i = 0; i < 5; i++) {
    Serial.print(fingerNames[i]);
    Serial.print("=");
    Serial.print(fingerStates[i] ? "OPEN" : "CLOSED");
    if (i < 4) Serial.print(", ");
  }
  Serial.println();
}

/*
 * Smoothly move servos toward target positions
 * This prevents jerky movements and reduces stress on servos
 */
void moveServosSmooth() {
  unsigned long currentTime = millis();
  
  // Only update servos at specified interval for smooth movement
  if (currentTime - lastMoveTime >= MOVE_INTERVAL) {
    lastMoveTime = currentTime;
    
    bool isMoving = false;
    
    for (int i = 0; i < 5; i++) {
      // Calculate difference between current and target
      int diff = targetAngles[i] - currentAngles[i];
      
      if (abs(diff) > 0) {
        isMoving = true;
        
        // Move toward target by MOVE_SPEED degrees
        if (diff > 0) {
          currentAngles[i] += min(MOVE_SPEED, diff);
        } else {
          currentAngles[i] += max(-MOVE_SPEED, diff);
        }
        
        // Write new angle to servo
        fingerServos[i].write(currentAngles[i]);
      }
    }
  }
}

/*
 * Calibration sequence - test each finger individually
 * Uncomment the call in setup() to run on startup
 */
void calibrateServos() {
  Serial.println("Starting calibration sequence...");
  delay(1000);
  
  for (int i = 0; i < 5; i++) {
    Serial.print("Testing ");
    Serial.print(fingerNames[i]);
    Serial.println("...");
    
    // Close finger
    Serial.println("  Closing...");
    fingerServos[i].write(CLOSED_ANGLE[i]);
    delay(1000);
    
    // Open finger
    Serial.println("  Opening...");
    fingerServos[i].write(OPEN_ANGLE[i]);
    delay(1000);
    
    // Return to middle
    Serial.println("  Neutral...");
    fingerServos[i].write(90);
    delay(500);
  }
  
  Serial.println("Calibration complete!");
  Serial.println();
}