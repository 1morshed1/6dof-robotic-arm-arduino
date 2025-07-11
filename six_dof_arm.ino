#include <Servo.h>

// Create servo objects for 6 DOF
Servo baseServo;        // Base rotation
Servo shoulderServo;    // Shoulder joint
Servo elbowServo;       // Elbow joint
Servo wristPitchServo;  // Wrist pitch
Servo wristRollServo;   // Wrist roll
Servo gripperServo;     // Gripper

// Flex sensor pins (analog inputs)
const int flexPin1 = A0;  // Base control
const int flexPin2 = A1;  // Shoulder control
const int flexPin3 = A2;  // Elbow control
const int flexPin4 = A3;  // Wrist pitch control
const int flexPin5 = A4;  // Wrist roll control
const int flexPin6 = A5;  // Gripper control

// Servo pins (digital outputs)
const int basePin = 3;
const int shoulderPin = 5;
const int elbowPin = 6;
const int wristPitchPin = 9;
const int wristRollPin = 10;
const int gripperPin = 11;

// Calibration values for flex sensors (you'll need to adjust these)
// These are the raw analog values when the sensor is straight and fully bent
int flexStraight[6] = {200, 200, 200, 200, 200, 200};
int flexBent[6] = {800, 800, 800, 800, 800, 800};

// Servo angle limits (adjust based on your specific servos and arm design)
int servoMin[6] = {0, 20, 0, 0, 0, 0};      // Minimum angles
int servoMax[6] = {180, 160, 180, 180, 180, 180}; // Maximum angles

// Smoothing variables
int lastServoPos[6] = {90, 90, 90, 90, 90, 90}; // Last servo positions
const int smoothingFactor = 5; // Higher = more smoothing

// Movement speed control
unsigned long lastUpdate = 0;
const int updateInterval = 20; // Update every 20ms for smooth movement

void setup() {
  Serial.begin(9600);
  
  // Attach servos to their pins
  baseServo.attach(basePin);
  shoulderServo.attach(shoulderPin);
  elbowServo.attach(elbowPin);
  wristPitchServo.attach(wristPitchPin);
  wristRollServo.attach(wristRollPin);
  gripperServo.attach(gripperPin);
  
  // Initialize servos to center positions
  baseServo.write(90);
  shoulderServo.write(90);
  elbowServo.write(90);
  wristPitchServo.write(90);
  wristRollServo.write(90);
  gripperServo.write(90);
  
  Serial.println("6-DOF Robotic Arm with Flex Sensors Initialized");
  Serial.println("Starting calibration sequence...");
  
  // Calibrate flex sensors
  calibrateFlexSensors();
  
  delay(2000); // Wait for servos to reach initial positions
}

void loop() {
  // Check if it's time to update
  if (millis() - lastUpdate >= updateInterval) {
    
    // Read all flex sensors
    int flexValues[6];
    flexValues[0] = analogRead(flexPin1);
    flexValues[1] = analogRead(flexPin2);
    flexValues[2] = analogRead(flexPin3);
    flexValues[3] = analogRead(flexPin4);
    flexValues[4] = analogRead(flexPin5);
    flexValues[5] = analogRead(flexPin6);
    
    // Convert flex sensor values to servo angles
    int targetAngles[6];
    for (int i = 0; i < 6; i++) {
      targetAngles[i] = mapFlexToServo(flexValues[i], i);
    }
    
    // Apply smoothing and update servos
    updateServos(targetAngles);
    
    // Print debug information
    printDebugInfo(flexValues, targetAngles);
    
    lastUpdate = millis();
  }
}

// Map flex sensor reading to servo angle
int mapFlexToServo(int flexValue, int sensorIndex) {
  // Constrain the flex value to calibrated range
  flexValue = constrain(flexValue, flexStraight[sensorIndex], flexBent[sensorIndex]);
  
  // Map to servo angle range
  int angle = map(flexValue, flexStraight[sensorIndex], flexBent[sensorIndex], 
                  servoMin[sensorIndex], servoMax[sensorIndex]);
  
  return constrain(angle, servoMin[sensorIndex], servoMax[sensorIndex]);
}

// Update servos with smoothing
void updateServos(int targetAngles[]) {
  // Array of servo objects for easier iteration
  Servo* servos[] = {&baseServo, &shoulderServo, &elbowServo, 
                     &wristPitchServo, &wristRollServo, &gripperServo};
  
  for (int i = 0; i < 6; i++) {
    // Apply smoothing
    int smoothedAngle = (lastServoPos[i] * (smoothingFactor - 1) + targetAngles[i]) / smoothingFactor;
    
    // Update servo position
    servos[i]->write(smoothedAngle);
    lastServoPos[i] = smoothedAngle;
  }
}

// Calibrate flex sensors
void calibrateFlexSensors() {
  Serial.println("Calibrating flex sensors...");
  Serial.println("Keep all sensors straight for 3 seconds");
  
  delay(3000);
  
  // Read straight values
  for (int i = 0; i < 6; i++) {
    int sum = 0;
    for (int j = 0; j < 10; j++) {
      sum += analogRead(A0 + i);
      delay(50);
    }
    flexStraight[i] = sum / 10;
  }
  
  Serial.println("Now bend all sensors fully for 3 seconds");
  delay(3000);
  
  // Read bent values
  for (int i = 0; i < 6; i++) {
    int sum = 0;
    for (int j = 0; j < 10; j++) {
      sum += analogRead(A0 + i);
      delay(50);
    }
    flexBent[i] = sum / 10;
  }
  
  Serial.println("Calibration complete!");
  Serial.println("Straight values: ");
  for (int i = 0; i < 6; i++) {
    Serial.print("Sensor ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(flexStraight[i]);
  }
  
  Serial.println("Bent values: ");
  for (int i = 0; i < 6; i++) {
    Serial.print("Sensor ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(flexBent[i]);
  }
}

// Print debug information
void printDebugInfo(int flexValues[], int targetAngles[]) {
  static unsigned long lastPrint = 0;
  
  // Print debug info every 500ms
  if (millis() - lastPrint >= 500) {
    Serial.println("--- Status ---");
    String joints[] = {"Base", "Shoulder", "Elbow", "WristP", "WristR", "Gripper"};
    
    for (int i = 0; i < 6; i++) {
      Serial.print(joints[i]);
      Serial.print(" - Flex: ");
      Serial.print(flexValues[i]);
      Serial.print(" | Angle: ");
      Serial.print(targetAngles[i]);
      Serial.print(" | Actual: ");
      Serial.println(lastServoPos[i]);
    }
    Serial.println();
    
    lastPrint = millis();
  }
}

// Emergency stop function (can be called from serial input)
void emergencyStop() {
  Serial.println("EMERGENCY STOP!");
  
  // Stop all servos at current position
  for (int i = 0; i < 6; i++) {
    // Servos automatically hold position when no new commands are sent
  }
  
  // You could add additional safety measures here
  // like detaching servos or moving to a safe position
}

// Optional: Serial command interface
void serialEvent() {
  if (Serial.available()) {
    String command = Serial.readString();
    command.trim();
    
    if (command == "STOP") {
      emergencyStop();
    } else if (command == "CALIBRATE") {
      calibrateFlexSensors();
    } else if (command == "RESET") {
      // Reset to center positions
      for (int i = 0; i < 6; i++) {
        lastServoPos[i] = 90;
      }
    }
  }
}

// Optional: Function to set custom servo limits
void setServoLimits(int joint, int minAngle, int maxAngle) {
  if (joint >= 0 && joint < 6) {
    servoMin[joint] = constrain(minAngle, 0, 180);
    servoMax[joint] = constrain(maxAngle, 0, 180);
    
    Serial.print("Joint ");
    Serial.print(joint);
    Serial.print(" limits set to: ");
    Serial.print(servoMin[joint]);
    Serial.print(" - ");
    Serial.println(servoMax[joint]);
  }
}
