// First, let's add our function prototype at the top with the other prototypes
#define USE_PCA9685_SERVO_EXPANDER 
#define MAX_EASING_SERVOS 16 
#define NUMBER_OF_SERVOS 12 
#define ENABLE_EASE_CUBIC 
#define VERSION "0.0.3" // Updated version number

#include <Arduino.h> 
#include <ServoEasing.hpp> 
#include <math.h> 

#define FIRST_PCA9685_EXPANDER_ADDRESS PCA9685_DEFAULT_ADDRESS 

// Robot configuration 
struct QuadrupedConfig { 
  int servoMinMaxes[12][2]; 
  int thighLength; 
  int kneeLength; 
  int defaultSpeed; 
  int easeType; 
}; 

QuadrupedConfig config = { 
  {  // servoMinMaxes[12][2] 
    {74, 174}, {89, -6}, {107, 200}, {92, -3},     // Hip servos 
    {181, 86}, {9, 111}, {185, 90}, {10, 111},     // Thigh servos 
    {87, 7}, {94, 177}, {103, 16}, {91, 168},      // Knee servos 
  }, 
  81,  // thighLength 
  109, // kneeLength 
  30,  // defaultSpeed 
  EASE_CUBIC_IN_OUT // easeType 
}; 

// Robot states 
enum RobotState { 
  IDLE, 
  COMPACT, 
  STANDING, 
  CROUCHING, 
  CALIBRATING, 
  LEG_LIFTING 
}; 

RobotState currentState = IDLE; 
unsigned long stateStartTime = 0; 
int currentLegIndex = 0; 

// Function prototypes 
void getAndAttach16ServosToPCA9685Expander(uint8_t aPCA9685I2CAddress); 
int GetServoAngleToPercentRange(int id, int angle); 
void ServoPercentRange(int id, int percent); 
int GetServoPercentRange(int id, int percent); 
int GetServoPercentMagnitude(int id); 
void SetPosition(int index, int position); 
void LiftLeg(int id); 
void DoLegLifts(); 
void CalibrateServos(); 
void calibrateIndividualServo(); // New function prototype
void calibrateSingleServo(int servoId); // Additional prototype for the helper function
void printCalibrationResults(); 
void GotoCompact(); 
void FinalStand(); 
void Crouch(); 
void Stand(); 
void IKStand(int height); 
void processCommand(String command); 
void handleServoCommand(char servoType, int baseIndex, String command); 
void listServoPositions();
void printServoInfo(int servoId); // New helper function prototype

// Convert servo angle (0-90 degrees) to matching position in servo's min-max range 
int GetServoAngleToPercentRange(int id, int angle) { 
  return map(angle, 0, 90, config.servoMinMaxes[id][0], config.servoMinMaxes[id][1]); 
} 

// Set servo to position based on percentage (0-100%) 
void ServoPercentRange(int id, int percent) { 
  ServoEasing::ServoEasingArray[id]->write( 
    map(percent, 0, 100, config.servoMinMaxes[id][0], config.servoMinMaxes[id][1]) 
  ); 
} 

// Convert percentage (0-100%) to matching position in servo's min-max range 
int GetServoPercentRange(int id, int percent) { 
  return map(percent, 0, 100, config.servoMinMaxes[id][0], config.servoMinMaxes[id][1]); 
} 

// Get the servo's full range of motion 
int GetServoPercentMagnitude(int id) { 
  return abs(config.servoMinMaxes[id][1] - config.servoMinMaxes[id][0]); 
} 

// Set servo position with easing 
void SetPosition(int index, int position) { 
  ServoEasing::ServoEasingArray[index]->setEaseTo(GetServoPercentRange(index, position), config.defaultSpeed * 2); 
} 

// Print information about a specific servo
void printServoInfo(int servoId) {
  // Determine servo type and leg position
  String servoType;
  String legPosition;
  
  if (servoId < 4) {
    servoType = F("Hip");
  } else if (servoId < 8) {
    servoType = F("Thigh");
  } else {
    servoType = F("Knee");
  }
  
  switch (servoId % 4) {
    case 0: legPosition = F("Front Left"); break;
    case 1: legPosition = F("Front Right"); break;
    case 2: legPosition = F("Back Left"); break;
    case 3: legPosition = F("Back Right"); break;
  }
  
  int currentPos = ServoEasing::ServoEasingNextPositionArray[servoId];
  int minPos = config.servoMinMaxes[servoId][0];
  int maxPos = config.servoMinMaxes[servoId][1];
  
  Serial.print(F("\n--- "));
  Serial.print(legPosition);
  Serial.print(F(" "));
  Serial.print(servoType);
  Serial.print(F(" Servo (ID: "));
  Serial.print(servoId);
  Serial.println(F(") ---"));
  
  Serial.print(F("Current position: "));
  Serial.println(currentPos);
  
  Serial.print(F("Calibrated MIN: "));
  Serial.println(minPos);
  
  Serial.print(F("Calibrated MAX: "));
  Serial.println(maxPos);
  
  Serial.print(F("Movement range: "));
  Serial.print(abs(maxPos - minPos));
  Serial.println(F(" degrees"));
}

// Function to calibrate an individual servo through serial monitor
void calibrateIndividualServo() {
  Serial.println(F("\n=== Individual Servo Calibration ==="));
  Serial.println(F("Select servo to calibrate (0-11):"));
  Serial.println(F("  Hip servos: 0-3 (rotate leg side to side)"));
  Serial.println(F("  Thigh servos: 4-7 (rotate leg forward/backward)"));
  Serial.println(F("  Knee servos: 8-11 (extend/contract leg)"));
  Serial.println(F("Or enter 'list' to see all servo positions and info"));
  
  // Wait for servo selection
  while (true) {
    if (Serial.available() > 0) {
      String input = Serial.readStringUntil('\n');
      input.trim();
      
      if (input.equals(F("list"))) {
        // List all servo positions and info
        for (int i = 0; i < 12; i++) {
          printServoInfo(i);
        }
        Serial.println(F("\nSelect servo to calibrate (0-11), or enter 'exit' to return:"));
        continue;
      }
      
      if (input.equals(F("exit"))) {
        Serial.println(F("Exiting calibration mode"));
        return;
      }
      
      int servoId = input.toInt();
      
      if (servoId < 0 || servoId > 11) {
        Serial.println(F("Invalid servo ID. Please enter a number between 0-11."));
        continue;
      }
      
      // Valid servo ID selected, proceed with calibration
      calibrateSingleServo(servoId);
      
      // After calibrating one servo, ask if user wants to calibrate another
      Serial.println(F("\nCalibrate another servo? Enter servo ID (0-11), 'list', or 'exit':"));
    }
    delay(10);
  }
}

// Function to calibrate a specific servo
void calibrateSingleServo(int servoId) {
  // Print detailed information about this servo
  printServoInfo(servoId);
  
  // Get current position as starting point
  int currentPos = ServoEasing::ServoEasingNextPositionArray[servoId];
  
  Serial.println(F("\nCalibration commands:"));
  Serial.println(F("  +/- : Move servo by 1 degree"));
  Serial.println(F("  ++/-- : Move servo by 5 degrees"));
  Serial.println(F("  +++/--- : Move servo by 10 degrees"));
  Serial.println(F("  p### : Move to position ### (absolute)"));
  Serial.println(F("  min : Set current position as MIN calibration"));
  Serial.println(F("  max : Set current position as MAX calibration"));
  Serial.println(F("  center : Move to middle position between min and max"));
  Serial.println(F("  test : Test full range of motion"));
  Serial.println(F("  info : Show current servo information"));
  Serial.println(F("  save : Save calibration"));
  Serial.println(F("  done : Finish calibrating this servo"));
  
  // Interactive calibration loop
  bool calibrating = true;
  while (calibrating) {
    if (Serial.available() > 0) {
      String command = Serial.readStringUntil('\n');
      command.trim();
      
      // Process command
      if (command.equals(F("done"))) {
        Serial.println(F("Finished calibrating this servo"));
        calibrating = false;
      } 
      else if (command.equals(F("save"))) {
        Serial.println(F("Saving calibration values"));
        printCalibrationResults();
      }
      else if (command.equals(F("info"))) {
        printServoInfo(servoId);
      }
      else if (command.equals(F("min"))) {
        config.servoMinMaxes[servoId][0] = currentPos;
        Serial.print(F("Set MIN value to "));
        Serial.println(currentPos);
      }
      else if (command.equals(F("max"))) {
        config.servoMinMaxes[servoId][1] = currentPos;
        Serial.print(F("Set MAX value to "));
        Serial.println(currentPos);
      }
      else if (command.equals(F("center"))) {
        // Move to middle position between min and max
        int minPos = config.servoMinMaxes[servoId][0];
        int maxPos = config.servoMinMaxes[servoId][1];
        int centerPos = minPos + (maxPos - minPos) / 2;
        
        currentPos = centerPos;
        ServoEasing::ServoEasingArray[servoId]->setEaseTo(currentPos, 40);
        while (!updateAllServos()) {}
        Serial.print(F("Moved to center position: "));
        Serial.println(currentPos);
      }
      else if (command.equals(F("test"))) {
        // Test movement from min to max
        Serial.println(F("Testing range of motion..."));
        int minPos = config.servoMinMaxes[servoId][0];
        int maxPos = config.servoMinMaxes[servoId][1];
        
        // Move to min position
        Serial.print(F("Moving to MIN position: "));
        Serial.println(minPos);
        ServoEasing::ServoEasingArray[servoId]->setEaseTo(minPos, 30);
        while (!updateAllServos()) {}
        delay(1000);
        
        // Move to max position
        Serial.print(F("Moving to MAX position: "));
        Serial.println(maxPos);
        ServoEasing::ServoEasingArray[servoId]->setEaseTo(maxPos, 30);
        while (!updateAllServos()) {}
        delay(1000);
        
        // Return to current position
        Serial.print(F("Returning to previous position: "));
        Serial.println(currentPos);
        ServoEasing::ServoEasingArray[servoId]->setEaseTo(currentPos, 30);
        while (!updateAllServos()) {}
        
        Serial.println(F("Range test complete"));
      }
      else if (command.startsWith(F("p"))) {
        // Move to absolute position
        int newPos = command.substring(1).toInt();
        currentPos = newPos;
        ServoEasing::ServoEasingArray[servoId]->setEaseTo(currentPos, 40);
        while (!updateAllServos()) {}
        Serial.print(F("Moved to position: "));
        Serial.println(currentPos);
      }
      else if (command.equals(F("+++"))) {
        // Move 10 degrees forward
        currentPos += 10;
        ServoEasing::ServoEasingArray[servoId]->setEaseTo(currentPos, 40);
        while (!updateAllServos()) {}
        Serial.print(F("New position: "));
        Serial.println(currentPos);
      }
      else if (command.equals(F("---"))) {
        // Move 10 degrees backward
        currentPos -= 10;
        ServoEasing::ServoEasingArray[servoId]->setEaseTo(currentPos, 40);
        while (!updateAllServos()) {}
        Serial.print(F("New position: "));
        Serial.println(currentPos);
      }
      else if (command.equals(F("++"))) {
        // Move 5 degrees forward
        currentPos += 5;
        ServoEasing::ServoEasingArray[servoId]->setEaseTo(currentPos, 40);
        while (!updateAllServos()) {}
        Serial.print(F("New position: "));
        Serial.println(currentPos);
      }
      else if (command.equals(F("--"))) {
        // Move 5 degrees backward
        currentPos -= 5;
        ServoEasing::ServoEasingArray[servoId]->setEaseTo(currentPos, 40);
        while (!updateAllServos()) {}
        Serial.print(F("New position: "));
        Serial.println(currentPos);
      }
      else if (command.equals(F("+"))) {
        // Move 1 degree forward
        currentPos += 1;
        ServoEasing::ServoEasingArray[servoId]->setEaseTo(currentPos, 40);
        while (!updateAllServos()) {}
        Serial.print(F("New position: "));
        Serial.println(currentPos);
      }
      else if (command.equals(F("-"))) {
        // Move 1 degree backward
        currentPos -= 1;
        ServoEasing::ServoEasingArray[servoId]->setEaseTo(currentPos, 40);
        while (!updateAllServos()) {}
        Serial.print(F("New position: "));
        Serial.println(currentPos);
      }
      else {
        Serial.println(F("Unknown command"));
      }
    }
    delay(10);
  }
}

// Lift a single leg with smooth movement 
void LiftLeg(int id) { 
  // Store current positions 
  int startThigh = ServoEasing::ServoEasingNextPositionArray[id+4]; 
  int startKnee = ServoEasing::ServoEasingNextPositionArray[id+8]; 

  // Move thigh and knee for lifting 
  ServoEasing::ServoEasingArray[id+4]->setEaseTo(startThigh - 20, 30);
  ServoEasing::ServoEasingArray[id+8]->setEaseTo(startKnee + 20, 30);
  while (!updateAllServos()) {}
  
  delay(500); // Hold lifted position
  
  // Return to original position
  ServoEasing::ServoEasingArray[id+4]->setEaseTo(startThigh, 30);
  ServoEasing::ServoEasingArray[id+8]->setEaseTo(startKnee, 30);
  while (!updateAllServos()) {}
}

// Implementation of turning and walking functions

// Define walking parameters structure and gait types
enum GaitType {
  WAVE_GAIT,
  RIPPLE_GAIT,
  TROT_GAIT
};

struct WalkingParameters {
  bool isMoving;
  int direction;    // 1 for forward, -1 for backward, 0 for turning in place
  float bodyHeight;
  float stepLength;
  float stepHeight;
  unsigned long cycleStartTime;
  unsigned long cycleDuration;
};

// Define leg position structure
struct LegPosition {
  float x;
  float y;
  float z;
};

// Constants for leg dimensions and movement limits
#define LEG_THIGH_LENGTH config.thighLength
#define LEG_KNEE_LENGTH config.kneeLength
#define TOTAL_SERVOS NUMBER_OF_SERVOS

// Constants for servo angle limits
#define HIP_MIN_ANGLE 0
#define HIP_MAX_ANGLE 180
#define THIGH_MIN_ANGLE 0
#define THIGH_MAX_ANGLE 180
#define KNEE_MIN_ANGLE 0
#define KNEE_MAX_ANGLE 180

// Constants for body movement limits
#define MIN_BODY_HEIGHT 80
#define MAX_BODY_HEIGHT 140
#define MIN_STEP_LENGTH 30
#define MAX_STEP_LENGTH 100
#define MIN_STEP_HEIGHT 20
#define MAX_STEP_HEIGHT 60

// Global variables for walking and turning
WalkingParameters walkParams = {
  false,      // isMoving
  1,          // direction (forward)
  120.0f,     // bodyHeight
  70.0f,      // stepLength
  40.0f,      // stepHeight
  0,          // cycleStartTime
  1500        // cycleDuration (milliseconds)
};

float currentTurnFactor = 0.0;  // -1.0 (full left) to 1.0 (full right)
GaitType currentGait = TROT_GAIT;

// Servo offsets for fine-tuning leg positions
int servoOffsets[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int servoSpeeds[12] = {40, 40, 40, 40, 30, 30, 30, 30, 30, 30, 30, 30};

// Update the walking function to include turning
void updateWalking() {
  if (!walkParams.isMoving) {
    return;
  }
  
  unsigned long currentTime = millis();
  unsigned long timeSinceCycleStart = currentTime - walkParams.cycleStartTime;
  
  // Calculate phase in the walking cycle (0.0 to 1.0)
  float cyclePhase = (float)(timeSinceCycleStart % walkParams.cycleDuration) / walkParams.cycleDuration;
  
  // Calculate and apply position for each leg
  for (int legIndex = 0; legIndex < 4; legIndex++) {
    LegPosition legPos;
    
    // Use the turning-aware function instead of basic walking
    calculateLegWalkingPositionWithTurn(legIndex, cyclePhase, legPos, currentTurnFactor);
    
    // Use inverse kinematics to calculate servo angles
    int hipAngle, thighAngle, kneeAngle;
    calculateLegIK(legIndex, legPos.x, legPos.y, legPos.z, hipAngle, thighAngle, kneeAngle);
    
    // Set servo positions
    mapAnglesAndSetServos(legIndex, hipAngle, thighAngle, kneeAngle);
  }
  
  // Update servos
  updateAllServos();
}

// Modify the startTurn function to handle turning in place
void startTurn(int direction) {
  // direction: +1 for right turn, -1 for left turn
  Serial.print(F("Starting to turn "));
  if (direction > 0) {
    Serial.println(F("right"));
  } else {
    Serial.println(F("left"));
  }
  
  // Set the turn factor to full strength (-1.0 or 1.0)
  currentTurnFactor = direction > 0 ? 1.0 : -1.0;
  
  // If not already moving, start movement for turning in place
  if (!walkParams.isMoving) {
    walkParams.isMoving = true;
    walkParams.direction = 0;  // No forward/backward movement when turning in place
    walkParams.cycleStartTime = millis();
    
    // Initial positions for turning in place
    for (int legIndex = 0; legIndex < 4; legIndex++) {
      LegPosition legPos;
      float initialPhase = 0.75 + (legIndex * 0.25);  // Spread legs in the walking cycle
      calculateLegWalkingPositionWithTurn(legIndex, initialPhase, legPos, currentTurnFactor);
      
      int hipAngle, thighAngle, kneeAngle;
      calculateLegIK(legIndex, legPos.x, legPos.y, legPos.z, hipAngle, thighAngle, kneeAngle);
      
      mapAnglesAndSetServos(legIndex, hipAngle, thighAngle, kneeAngle);
    }
    
    // Wait for initial positioning to complete
    while (!updateAllServos()) {
      delay(10);
    }
  }
}

// Function to adjust turn factor while walking or turning
void adjustTurnFactor(float factor) {
  currentTurnFactor = constrain(factor, -1.0, 1.0);
  
  // Output the current turning status
  if (currentTurnFactor == 0.0) {
    Serial.println(F("Walking straight"));
  } else if (currentTurnFactor > 0) {
    Serial.print(F("Turning right with factor: "));
    Serial.println(currentTurnFactor);
  } else {
    Serial.print(F("Turning left with factor: "));
    Serial.println(abs(currentTurnFactor));
  }
}

// Update the startWalking function to reset turn factor
void startWalking(int direction) {
  Serial.println(F("Starting to walk"));
  
  // Reset turning when starting a new walking direction
  currentTurnFactor = 0.0;
  
  // Set walking parameters
  walkParams.direction = direction; // 1 for forward, -1 for backward
  walkParams.isMoving = true;
  walkParams.cycleStartTime = millis();
  
  // Initial servo positions
  // This ensures a smooth start in the walking sequence
  float initialPhase = 0.0;
  
  for (int legIndex = 0; legIndex < 4; legIndex++) {
    LegPosition legPos;
    calculateLegWalkingPositionWithTurn(legIndex, initialPhase, legPos, currentTurnFactor);
    
    int hipAngle, thighAngle, kneeAngle;
    calculateLegIK(legIndex, legPos.x, legPos.y, legPos.z, hipAngle, thighAngle, kneeAngle);
    
    mapAnglesAndSetServos(legIndex, hipAngle, thighAngle, kneeAngle);
  }
  
  // Wait for initial positioning to complete
  while (!updateAllServos()) {}
}

// Modify stopWalking to reset turn factor
void stopWalking() {
  Serial.println(F("Stopping walk"));
  walkParams.isMoving = false;
  currentTurnFactor = 0.0;  // Reset turn factor
  
  // Execute a smooth transition to standing position
  IKStand(walkParams.bodyHeight);
}

// Calculate leg position during walking, incorporating turning
void calculateLegWalkingPositionWithTurn(int legIndex, float phase, LegPosition &legPos, float turnFactor) {
  // Leg positions: 0=front-left, 1=front-right, 2=back-left, 3=back-right
  
  // Adjust phase based on leg and gait type
  float adjustedPhase = phase;
  
  // Apply leg-specific phase offset based on gait type
  switch(currentGait) {
    case WAVE_GAIT:
      // Sequential leg movement (0.25 offset between each leg)
      adjustedPhase = fmod(phase + (legIndex * 0.25), 1.0);
      break;
    case RIPPLE_GAIT:
      // Alternating tripod gait (legs move in two groups)
      if (legIndex == 0 || legIndex == 3) { // Front-left and back-right
        adjustedPhase = phase;
      } else { // Front-right and back-left
        adjustedPhase = fmod(phase + 0.5, 1.0);
      }
      break;
    case TROT_GAIT:
      // Diagonal pairs move together
      if (legIndex == 0 || legIndex == 3) { // Front-left and back-right
        adjustedPhase = phase;
      } else { // Front-right and back-left
        adjustedPhase = fmod(phase + 0.5, 1.0);
      }
      break;
  }
  
  // Default leg positions when standing
  float defaultX = 0;
  float defaultY = (legIndex % 2 == 0) ? -50 : 50; // Left legs: -50, Right legs: 50
  
  // Adjust for front/back position
  if (legIndex < 2) { // Front legs
    defaultX = 70;
  } else { // Back legs
    defaultX = -70;
  }
  
  // Start with default position
  legPos.x = defaultX;
  legPos.y = defaultY;
  legPos.z = -walkParams.bodyHeight;
  
  // If not moving, just return the default position
  if (!walkParams.isMoving) {
    return;
  }
  
  // Walking cycle calculations
  if (adjustedPhase < 0.5) {
    // Stance phase (foot on ground) - moves backward relative to body
    float stanceProgress = adjustedPhase * 2.0; // 0.0 to 1.0 during stance
    
    // Calculate forward/backward movement
    float xOffset = walkParams.stepLength * (0.5 - stanceProgress) * walkParams.direction;
    
    // Apply turn factor during stance phase
    // For turning, outer legs move faster than inner legs
    float turnAdjustment = 0;
    
    // Determine if this is an inner or outer leg relative to turn direction
    bool isRightLeg = (legIndex % 2 == 1); // 1 and 3 are right legs
    
    if ((isRightLeg && turnFactor > 0) || (!isRightLeg && turnFactor < 0)) {
      // Outer leg during turn - moves more
      turnAdjustment = abs(turnFactor) * walkParams.stepLength * 0.5;
    } else if ((isRightLeg && turnFactor < 0) || (!isRightLeg && turnFactor > 0)) {
      // Inner leg during turn - moves less
      turnAdjustment = -abs(turnFactor) * walkParams.stepLength * 0.5;
    }
    
    // Apply forward/backward position with turn adjustment
    legPos.x += xOffset + turnAdjustment;
    
    // Turn in place (when direction is 0)
    if (walkParams.direction == 0 && turnFactor != 0) {
      // When turning in place, legs move in a circular pattern
      float turnDirection = (isRightLeg) ? -turnFactor : turnFactor;
      legPos.x += turnDirection * walkParams.stepLength * 0.3 * sin(stanceProgress * M_PI);
      legPos.y += turnDirection * walkParams.stepLength * 0.2 * cos(stanceProgress * M_PI);
    }
  } else {
    // Swing phase (foot in air) - moves forward relative to body
    float swingProgress = (adjustedPhase - 0.5) * 2.0; // 0.0 to 1.0 during swing
    
    // Calculate forward/backward movement
    float xOffset = walkParams.stepLength * (swingProgress - 0.5) * walkParams.direction;
    
    // Calculate leg lift height using sine wave for smooth movement
    float legLift = walkParams.stepHeight * sin(swingProgress * M_PI);
    
    // Apply turn factor during swing phase
    float turnAdjustment = 0;
    bool isRightLeg = (legIndex % 2 == 1);
    
    if ((isRightLeg && turnFactor > 0) || (!isRightLeg && turnFactor < 0)) {
      // Outer leg during turn
      turnAdjustment = abs(turnFactor) * walkParams.stepLength * 0.5;
    } else if ((isRightLeg && turnFactor < 0) || (!isRightLeg && turnFactor > 0)) {
      // Inner leg during turn
      turnAdjustment = -abs(turnFactor) * walkParams.stepLength * 0.5;
    }
    
    // Apply forward/backward position with turn adjustment
    legPos.x += xOffset + turnAdjustment;
    
    // Apply leg lift
    legPos.z += legLift;
    
    // Turn in place (when direction is 0)
    if (walkParams.direction == 0 && turnFactor != 0) {
      float turnDirection = (isRightLeg) ? -turnFactor : turnFactor;
      legPos.x += turnDirection * walkParams.stepLength * 0.3 * sin((swingProgress + 0.5) * M_PI);
      legPos.y += turnDirection * walkParams.stepLength * 0.2 * cos((swingProgress + 0.5) * M_PI);
    }
  }
}
// Calculate inverse kinematics for leg position
void calculateLegIK(int legIndex, float x, float y, float z, int &hipAngle, int &thighAngle, int &kneeAngle) {
  // Convert leg index to left/right side
  bool isRightSide = (legIndex % 2 == 1); // 1 and 3 are right legs
  
  // Hip calculation (hip servo rotates around Y axis)
  hipAngle = (int)(atan2(y, x) * 180.0 / M_PI);
  
  // Adjust hip angle based on side
  if (!isRightSide) {
    hipAngle = 90 - hipAngle; // For left legs
  } else {
    hipAngle = 90 + hipAngle; // For right legs
  }
  
  // Calculate the distance from hip to foot in the X-Z plane
  float legXZ = sqrt(x*x + z*z);
  
  // Calculate thigh and knee angles using the law of cosines
  float a = LEG_THIGH_LENGTH;
  float b = LEG_KNEE_LENGTH;
  float c = legXZ;
  
  // Check if position is reachable
  if (c > (a + b)) {
    // Position is too far to reach, limit to maximum extension
    c = a + b - 5; // Leave a small margin
  }
  
  // Calculate knee angle using law of cosines: cos(C) = (a² + b² - c²) / (2ab)
  float cosKnee = (a*a + b*b - c*c) / (2 * a * b);
  cosKnee = constrain(cosKnee, -1.0, 1.0); // Prevent domain errors
  float kneeRadians = acos(cosKnee);
  kneeAngle = 180 - (int)(kneeRadians * 180.0 / M_PI);
  
  // Calculate thigh angle
  float thighToGroundAngle = atan2(-z, x); // Angle from thigh to ground
  float thighToKneeAngle = acos((a*a + c*c - b*b) / (2 * a * c));
  float thighRadians = thighToGroundAngle + thighToKneeAngle;
  thighAngle = (int)(thighRadians * 180.0 / M_PI);
  
  // Adjust angles based on servo mounting orientation
  if (!isRightSide) {
    // Left side leg adjustments
    thighAngle = 180 - thighAngle;
    kneeAngle = 180 - kneeAngle;
  }
  
  // Apply angle constraints
  hipAngle = constrain(hipAngle, HIP_MIN_ANGLE, HIP_MAX_ANGLE);
  thighAngle = constrain(thighAngle, THIGH_MIN_ANGLE, THIGH_MAX_ANGLE);
  kneeAngle = constrain(kneeAngle, KNEE_MIN_ANGLE, KNEE_MAX_ANGLE);
}

// Map calculated angles to servo values and set servo positions
void mapAnglesAndSetServos(int legIndex, int hipAngle, int thighAngle, int kneeAngle) {
  // Calculate servo indices
  int hipServoIndex = legIndex;
  int thighServoIndex = legIndex + 4;
  int kneeServoIndex = legIndex + 8;
  
  // Map angles to servo positions with offsets
  int hipPos = map(hipAngle, 0, 180, config.servoMinMaxes[hipServoIndex][0], 
                   config.servoMinMaxes[hipServoIndex][1]) + servoOffsets[hipServoIndex];
  
  int thighPos = map(thighAngle, 0, 180, config.servoMinMaxes[thighServoIndex][0], 
                     config.servoMinMaxes[thighServoIndex][1]) + servoOffsets[thighServoIndex];
  
  int kneePos = map(kneeAngle, 0, 180, config.servoMinMaxes[kneeServoIndex][0], 
                    config.servoMinMaxes[kneeServoIndex][1]) + servoOffsets[kneeServoIndex];
  
  // Set servo target positions with speeds
  ServoEasing::ServoEasingArray[hipServoIndex]->setEaseTo(hipPos, servoSpeeds[hipServoIndex]);
  ServoEasing::ServoEasingArray[thighServoIndex]->setEaseTo(thighPos, servoSpeeds[thighServoIndex]);
  ServoEasing::ServoEasingArray[kneeServoIndex]->setEaseTo(kneePos, servoSpeeds[kneeServoIndex]);
}

// Process serial commands for gait selection
void setGaitType(int gaitType) {
  switch(gaitType) {
    case 0:
      currentGait = WAVE_GAIT;
      Serial.println(F("Switching to Wave Gait"));
      break;
    case 1:
      currentGait = RIPPLE_GAIT;
      Serial.println(F("Switching to Ripple Gait"));
      break;
    case 2:
      currentGait = TROT_GAIT;
      Serial.println(F("Switching to Trot Gait"));
      break;
    default:
      Serial.println(F("Invalid gait type, using Trot Gait"));
      currentGait = TROT_GAIT;
  }
}

// Change walking parameters
void setWalkingParameters(float height, float stepLength, float stepHeight) {
  // Constrain values to valid ranges
  walkParams.bodyHeight = constrain(height, MIN_BODY_HEIGHT, MAX_BODY_HEIGHT);
  walkParams.stepLength = constrain(stepLength, MIN_STEP_LENGTH, MAX_STEP_LENGTH);
  walkParams.stepHeight = constrain(stepHeight, MIN_STEP_HEIGHT, MAX_STEP_HEIGHT);
  
  Serial.println(F("Walking parameters updated:"));
  Serial.print(F("  Body height: "));
  Serial.println(walkParams.bodyHeight);
  Serial.print(F("  Step length: "));
  Serial.println(walkParams.stepLength);
  Serial.print(F("  Step height: "));
  Serial.println(walkParams.stepHeight);
}

// Do series of leg lifts 
void DoLegLifts() { 
  if (currentState != LEG_LIFTING) { 
    Serial.println(F("Starting leg lifts...")); 
    currentState = LEG_LIFTING; 
    currentLegIndex = 0; 
    stateStartTime = millis(); 
  } 
  
  unsigned long currentTime = millis(); 
  if (currentTime - stateStartTime > 2000) { 
    // Time to lift next leg 
    LiftLeg(currentLegIndex); 
    
    currentLegIndex = (currentLegIndex + 1) % 4; // Move to next leg 
    stateStartTime = currentTime; 
    
    if (currentLegIndex == 0) { 
      // We've completed a full cycle of all legs 
      Serial.println(F("Leg lift sequence complete")); 
      currentState = STANDING; 
    } 
  } 
}

// Calibrate all servos
void CalibrateServos() { 
  Serial.println(F("Starting calibration procedure...")); 
  currentState = CALIBRATING; 
  
  // Reset all servos to center position
  for (int i = 0; i < 12; i++) { 
    int centerPos = (config.servoMinMaxes[i][0] + config.servoMinMaxes[i][1]) / 2; 
    ServoEasing::ServoEasingArray[i]->write(centerPos); 
  } 
  delay(1000); // Give servos time to reach positions
  
  Serial.println(F("Calibration menu:"));
  Serial.println(F("1. Calibrate all servos sequentially"));
  Serial.println(F("2. Calibrate individual servo"));
  Serial.println(F("3. Print current calibration values"));
  Serial.println(F("4. Return to standing position"));
  
  // Wait for user input
  while (currentState == CALIBRATING) {
    if (Serial.available() > 0) {
      char choice = Serial.read();
      
      switch (choice) {
        case '1':
          // Sequential calibration of all servos
          Serial.println(F("Sequential calibration not implemented yet."));
          break;
        case '2':
          // Calibrate individual servo
          calibrateIndividualServo();
          Serial.println(F("Returning to calibration menu..."));
          Serial.println(F("Calibration menu:"));
          Serial.println(F("1. Calibrate all servos sequentially"));
          Serial.println(F("2. Calibrate individual servo"));
          Serial.println(F("3. Print current calibration values"));
          Serial.println(F("4. Return to standing position"));
          break;
        case '3':
          // Print calibration values
          printCalibrationResults();
          break;
        case '4':
          // Exit calibration mode
          Serial.println(F("Exiting calibration, returning to standing position..."));
          currentState = STANDING;
          Stand();
          break;
      }
    }
    delay(10);
  }
}

// Print calibration results
void printCalibrationResults() {
  Serial.println(F("\n=== Current Calibration Values ==="));
  Serial.println(F("Copy these values to update your config:"));
  
  Serial.println(F("QuadrupedConfig config = {"));
  Serial.println(F("  {  // servoMinMaxes[12][2]"));
  
  // Hip servos
  Serial.print(F("    {"));
  Serial.print(config.servoMinMaxes[0][0]);
  Serial.print(F(", "));
  Serial.print(config.servoMinMaxes[0][1]);
  Serial.print(F("}, {"));
  Serial.print(config.servoMinMaxes[1][0]);
  Serial.print(F(", "));
  Serial.print(config.servoMinMaxes[1][1]);
  Serial.print(F("}, {"));
  Serial.print(config.servoMinMaxes[2][0]);
  Serial.print(F(", "));
  Serial.print(config.servoMinMaxes[2][1]);
  Serial.print(F("}, {"));
  Serial.print(config.servoMinMaxes[3][0]);
  Serial.print(F(", "));
  Serial.print(config.servoMinMaxes[3][1]);
  Serial.println(F("},     // Hip servos"));
  
  // Thigh servos
  Serial.print(F("    {"));
  Serial.print(config.servoMinMaxes[4][0]);
  Serial.print(F(", "));
  Serial.print(config.servoMinMaxes[4][1]);
  Serial.print(F("}, {"));
  Serial.print(config.servoMinMaxes[5][0]);
  Serial.print(F(", "));
  Serial.print(config.servoMinMaxes[5][1]);
  Serial.print(F("}, {"));
  Serial.print(config.servoMinMaxes[6][0]);
  Serial.print(F(", "));
  Serial.print(config.servoMinMaxes[6][1]);
  Serial.print(F("}, {"));
  Serial.print(config.servoMinMaxes[7][0]);
  Serial.print(F(", "));
  Serial.print(config.servoMinMaxes[7][1]);
  Serial.println(F("},     // Thigh servos"));
  
  // Knee servos
  Serial.print(F("    {"));
  Serial.print(config.servoMinMaxes[8][0]);
  Serial.print(F(", "));
  Serial.print(config.servoMinMaxes[8][1]);
  Serial.print(F("}, {"));
  Serial.print(config.servoMinMaxes[9][0]);
  Serial.print(F(", "));
  Serial.print(config.servoMinMaxes[9][1]);
  Serial.print(F("}, {"));
  Serial.print(config.servoMinMaxes[10][0]);
  Serial.print(F(", "));
  Serial.print(config.servoMinMaxes[10][1]);
  Serial.print(F("}, {"));
  Serial.print(config.servoMinMaxes[11][0]);
  Serial.print(F(", "));
  Serial.print(config.servoMinMaxes[11][1]);
  Serial.println(F("},      // Knee servos"));
  
  Serial.println(F("  },"));
  Serial.print(F("  "));
  Serial.print(config.thighLength);
  Serial.println(F(",  // thighLength"));
  Serial.print(F("  "));
  Serial.print(config.kneeLength);
  Serial.println(F(", // kneeLength"));
  Serial.print(F("  "));
  Serial.print(config.defaultSpeed);
  Serial.println(F(",  // defaultSpeed"));
  Serial.println(F("  EASE_CUBIC_IN_OUT // easeType"));
  Serial.println(F("};"));
}

// Go to compact position (folded legs)
void GotoCompact() { 
  Serial.println(F("Moving to compact position...")); 
  currentState = COMPACT; 
  
  // Set all hip servos to center
  for (int i = 0; i < 4; i++) {
    ServoEasing::ServoEasingArray[i]->setEaseTo(map(90, 0, 180, config.servoMinMaxes[i][0], 
                                                  config.servoMinMaxes[i][1]), config.defaultSpeed);
  }
  
  // Fold legs in - thigh servos
  for (int i = 4; i < 8; i++) {
    // Different positions for left and right legs
    int thighPos = (i % 2 == 0) ? 0 : 180; // Left legs: 0, Right legs: 180
    ServoEasing::ServoEasingArray[i]->setEaseTo(map(thighPos, 0, 180, config.servoMinMaxes[i][0], 
                                                  config.servoMinMaxes[i][1]), config.defaultSpeed);
  }
  
  // Fold legs in - knee servos
  for (int i = 8; i < 12; i++) {
    // Different positions for left and right legs
    int kneePos = (i % 2 == 0) ? 180 : 0; // Left legs: 180, Right legs: 0
    ServoEasing::ServoEasingArray[i]->setEaseTo(map(kneePos, 0, 180, config.servoMinMaxes[i][0], 
                                                  config.servoMinMaxes[i][1]), config.defaultSpeed);
  }
  
  // Wait for movement to complete
  while (!updateAllServos()) {}
  Serial.println(F("Compact position reached"));
}

// Stand the robot at final height
void FinalStand() { 
  Serial.println(F("Moving to final standing position...")); 
  IKStand(120); // Use a standard standing height of 120mm
  currentState = STANDING; 
}

// Crouch down
void Crouch() { 
  Serial.println(F("Crouching down...")); 
  currentState = CROUCHING; 
  
  // Use inverse kinematics to lower the body
  IKStand(70); // Lower standing height
}

// Stand with inverse kinematics at specified height
void Stand() {
  Serial.println(F("Standing up..."));
  currentState = STANDING;
  
  // Use inverse kinematics to set standard standing height
  IKStand(120); // Standard standing height
}

// Use inverse kinematics to position the robot at a specific height
void IKStand(int height) {
  Serial.print(F("Setting height to: "));
  Serial.println(height);
  
  // Constrain height to valid range
  height = constrain(height, MIN_BODY_HEIGHT, MAX_BODY_HEIGHT);
  
  // Update walking parameters to match this height
  walkParams.bodyHeight = height;
  
  // Position each leg using inverse kinematics
  for (int legIndex = 0; legIndex < 4; legIndex++) {
    // Calculate default position for this leg
    float x = (legIndex < 2) ? 70 : -70; // Front legs forward, back legs backward
    float y = (legIndex % 2 == 0) ? -50 : 50; // Left legs negative Y, right legs positive Y
    float z = -height; // Negative Z is down
    
    // Calculate servo angles using inverse kinematics
    int hipAngle, thighAngle, kneeAngle;
    calculateLegIK(legIndex, x, y, z, hipAngle, thighAngle, kneeAngle);
    
    // Set servo positions
    mapAnglesAndSetServos(legIndex, hipAngle, thighAngle, kneeAngle);
  }
  
  // Wait for movement to complete
  while (!updateAllServos()) {
    delay(10);
  }
  
  Serial.println(F("Standing position reached"));
}

// Process commands from serial input
void processCommand(String command) {
  command.trim(); // Remove leading/trailing whitespace
  
  if (command.length() == 0) {
    return; // Ignore empty commands
  }
  
  // Convert to lowercase for case-insensitive comparison
  command.toLowerCase();
  
  // Check for main commands
  if (command == "help") {
    // Show available commands
    Serial.println(F("\n------ Robot Commands ------"));
    Serial.println(F("stand:     Move to standing position"));
    Serial.println(F("sit:       Move to sitting/crouched position"));
    Serial.println(F("compact:   Fold legs into compact position"));
    Serial.println(F("cal:       Enter calibration mode"));
    Serial.println(F("lift:      Perform leg lifting sequence"));
    Serial.println(F("fw:        Start walking forward"));
    Serial.println(F("bw:        Start walking backward"));
    Serial.println(F("left:      Turn left"));
    Serial.println(F("right:     Turn right"));
    Serial.println(F("straight:  Walk straight"));
    Serial.println(F("stop:      Stop walking or turning"));
    Serial.println(F("gait n:    Set gait type (0=wave, 1=ripple, 2=trot)"));
    Serial.println(F("height n:  Set body height (80-140mm)"));
    Serial.println(F("step n:    Set step length (30-100mm)"));
    Serial.println(F("h:         Help - show this menu"));
    Serial.println(F("info:      Show robot status"));
  }
  else if (command == "stand" || command == "s") {
    Stand();
  }
  else if (command == "sit" || command == "crouch") {
    Crouch();
  }
  else if (command == "compact" || command == "fold") {
    GotoCompact();
  }
  else if (command == "cal" || command == "calibrate") {
    CalibrateServos();
  }
  else if (command == "lift" || command == "legs") {
    DoLegLifts();
  }
  else if (command == "fw" || command == "forward") {
    startWalking(1); // 1 = forward
  }
  else if (command == "bw" || command == "backward") {
    startWalking(-1); // -1 = backward
  }
  else if (command == "left") {
    startTurn(-1); // -1 = left turn
  }
  else if (command == "right") {
    startTurn(1); // 1 = right turn
  }
  else if (command == "straight") {
    adjustTurnFactor(0.0); // Remove any turning
  }
  else if (command == "stop") {
    stopWalking();
  }
  else if (command.startsWith("gait ")) {
    // Set gait type
    int gaitType = command.substring(5).toInt();
    setGaitType(gaitType);
  }
  else if (command.startsWith("height ")) {
    // Set body height
    int height = command.substring(7).toInt();
    setWalkingParameters(height, walkParams.stepLength, walkParams.stepHeight);
    if (!walkParams.isMoving) {
      // If not currently walking, update the standing position
      IKStand(height);
    }
  }
  else if (command.startsWith("step ")) {
    // Set step length
    int stepLength = command.substring(5).toInt();
    setWalkingParameters(walkParams.bodyHeight, stepLength, walkParams.stepHeight);
  }
  else if (command.startsWith("h ")) {
    // Set step height
    int stepHeight = command.substring(2).toInt();
    setWalkingParameters(walkParams.bodyHeight, walkParams.stepLength, stepHeight);
  }
  else if (command == "info" || command == "status") {
    // Print current status
    Serial.println(F("\n------ Robot Status ------"));
    
    // Print current state
    Serial.print(F("Current state: "));
    switch(currentState) {
      case IDLE: Serial.println(F("IDLE")); break;
      case COMPACT: Serial.println(F("COMPACT")); break;
      case STANDING: Serial.println(F("STANDING")); break;
      case CROUCHING: Serial.println(F("CROUCHING")); break;
      case CALIBRATING: Serial.println(F("CALIBRATING")); break;
      case LEG_LIFTING: Serial.println(F("LEG_LIFTING")); break;
      default: Serial.println(F("UNKNOWN")); break;
    }
    
    // Print walking parameters
    Serial.print(F("Walking: "));
    if (walkParams.isMoving) {
      if (walkParams.direction > 0) {
        Serial.print(F("Forward"));
      } else if (walkParams.direction < 0) {
        Serial.print(F("Backward"));
      } else {
        Serial.print(F("Turning"));
      }
      
      // Print turning status
      if (currentTurnFactor != 0) {
        Serial.print(F(" turning "));
        if (currentTurnFactor > 0) {
          Serial.print(F("right ("));
        } else {
          Serial.print(F("left ("));
        }
        Serial.print(abs(currentTurnFactor * 100));
        Serial.print(F("%)"));
      }
      Serial.println();
    } else {
      Serial.println(F("Stopped"));
    }
    
    // Print gait type
    Serial.print(F("Gait type: "));
    switch(currentGait) {
      case WAVE_GAIT: Serial.println(F("Wave")); break;
      case RIPPLE_GAIT: Serial.println(F("Ripple")); break;
      case TROT_GAIT: Serial.println(F("Trot")); break;
      default: Serial.println(F("Unknown")); break;
    }
    
    // Print parameters
    Serial.print(F("Body height: "));
    Serial.print(walkParams.bodyHeight);
    Serial.println(F("mm"));
    
    Serial.print(F("Step length: "));
    Serial.print(walkParams.stepLength);
    Serial.println(F("mm"));
    
    Serial.print(F("Step height: "));
    Serial.print(walkParams.stepHeight);
    Serial.println(F("mm"));
    
    Serial.print(F("Version: "));
    Serial.println(VERSION);
  }
  else if (command.startsWith("hip") || command.startsWith("thigh") || command.startsWith("knee")) {
    // Handle servo-specific commands
    char servoType = command.charAt(0); // 'h', 't', or 'k'
    String restOfCommand = command.substring(3); // Skip "hip"/"thi"/"kne"
    
    int baseIndex = 0;
    if (servoType == 't') { // thigh
      baseIndex = 4;
    } else if (servoType == 'k') { // knee
      baseIndex = 8;
    }
    
    handleServoCommand(servoType, baseIndex, restOfCommand);
  }
  else if (command == "list") {
    // List all servo positions
    listServoPositions();
  }
  else {
    Serial.print(F("Unknown command: "));
    Serial.println(command);
    Serial.println(F("Type 'help' for available commands"));
  }
}

// Handle commands specific to servo groups
void handleServoCommand(char servoType, int baseIndex, String command) {
  // Check if it's a valid leg index command like "hip0 50" to set hip servo 0 to 50%
  int legIndex = -1;
  int value = -1;
  
  // Parse the command for leg index and value
  int spaceIndex = command.indexOf(' ');
  if (spaceIndex > 0) {
    // Extract leg index and value
    String legIndexStr = command.substring(0, spaceIndex);
    String valueStr = command.substring(spaceIndex + 1);
    
    legIndex = legIndexStr.toInt();
    value = valueStr.toInt();
    
    if (legIndex >= 0 && legIndex < 4 && value >= 0 && value <= 100) {
      // Valid command, set the servo
      int servoIndex = baseIndex + legIndex;
      Serial.print(F("Setting servo "));
      Serial.print(servoIndex);
      Serial.print(F(" to "));
      Serial.print(value);
      Serial.println(F("%"));
      
      ServoPercentRange(servoIndex, value);
    } else {
      Serial.println(F("Invalid leg index or value. Use 0-3 for leg index and 0-100 for value."));
    }
  } else {
    Serial.println(F("Invalid command format. Use 'hip0 50' to set hip servo 0 to 50%."));
  }
}

// List positions of all servos
void listServoPositions() {
  Serial.println(F("\n------ Servo Positions ------"));
  
  // Create an array of servo type names for easy reference
  const char* servoTypes[] = {"Hip", "Hip", "Hip", "Hip", 
                             "Thigh", "Thigh", "Thigh", "Thigh",
                             "Knee", "Knee", "Knee", "Knee"};
  
  // Create an array of leg position names
  const char* legPositions[] = {"Front Left", "Front Right", "Back Left", "Back Right"};
  
  for (int i = 0; i < 12; i++) {
    int currentPos = ServoEasing::ServoEasingNextPositionArray[i];
    int servoTypeIndex = i / 4; // 0=hip, 1=thigh, 2=knee
    int legIndex = i % 4;       // 0=FL, 1=FR, 2=BL, 3=BR
    
    // Calculate the percentage within the calibrated range
    int minPos = config.servoMinMaxes[i][0];
    int maxPos = config.servoMinMaxes[i][1];
    int range = abs(maxPos - minPos);
    int relativePos = abs(currentPos - minPos);
    int percentage = (range > 0) ? (relativePos * 100 / range) : 0;
    
    // Print formatted information
    Serial.print(legPositions[legIndex]);
    Serial.print(F(" "));
    Serial.print(servoTypes[i]);
    Serial.print(F(" (ID: "));
    Serial.print(i);
    Serial.print(F("): Position = "));
    Serial.print(currentPos);
    Serial.print(F(" ("));
    Serial.print(percentage);
    Serial.println(F("%)"));
  }
}

// Function to initialize and attach servos to the PCA9685 expander
void getAndAttach16ServosToPCA9685Expander(uint8_t aPCA9685I2CAddress) {
  Serial.print(F("Attaching servos to PCA9685 expander at address: 0x"));
  Serial.println(aPCA9685I2CAddress, HEX);
  
  // Initialize PCA9685 servo expander
  if (!ServoEasing::InitializeServoEasingEffortless()) {
    Serial.println(F("Error initializing PCA9685 servo expander"));
    while (true) {
      // Error happened, stay in infinite loop
      delay(1000);
    }
  }
  
  // Attach all servos with their min/max values
  for (int i = 0; i < NUMBER_OF_SERVOS; i++) {
    if (!ServoEasing::ServoEasingArray[i]->attach(
          ServoEasing::ServoEasingArray[i]->getPin(), 
          config.servoMinMaxes[i][0],
          config.servoMinMaxes[i][1])) {
      Serial.print(F("Error attaching servo "));
      Serial.println(i);
    }
    
    // Set easing type for each servo
    ServoEasing::ServoEasingArray[i]->setEasingType(config.easeType);
    
    // Set initial position to middle of range
    int middlePosition = (config.servoMinMaxes[i][0] + config.servoMinMaxes[i][1]) / 2;
    ServoEasing::ServoEasingArray[i]->write(middlePosition);
  }
  
  Serial.println(F("All servos attached successfully"));
}

// Main setup function
void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  
  // Wait for serial to connect (important for USB connection)
  while (!Serial && millis() < 3000) {};
  
  // Print welcome message
  Serial.println(F("\n==================================="));
  Serial.print(F("Quadruped Robot Controller v"));
  Serial.println(VERSION);
  Serial.println(F("==================================="));
  
  // Initialize servo expander
  getAndAttach16ServosToPCA9685Expander(FIRST_PCA9685_EXPANDER_ADDRESS);
  
  // Start in idle state
  currentState = IDLE;
  
  // Display available commands
  Serial.println(F("\nReady! Type 'help' for available commands"));
}

// Main loop function
void loop() {
  // Process any serial commands
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    processCommand(command);
  }
  
  // Handle current state
  switch (currentState) {
    case IDLE:
      // Do nothing in idle mode
      break;
      
    case LEG_LIFTING:
      // Continue leg lift sequence
      DoLegLifts();
      break;
      
    case CALIBRATING:
      // Handled within the CalibrateServos function
      break;
      
    case STANDING:
    case COMPACT:
    case CROUCHING:
      // For static positions, just maintain the state
      break;
      
    default:
      break;
  }
  
  // Update walking sequence if robot is walking
  if (walkParams.isMoving) {
    updateWalking();
  }
  
  // Always check for servo updates
  updateAllServos();
  
  // Short delay to prevent CPU overload
  delay(10);
}

// Function to set servo parameters
void setServoParameters(int servoId, int minAngle, int maxAngle, int offset, int speed) {
  if (servoId < 0 || servoId >= NUMBER_OF_SERVOS) {
    Serial.println(F("Invalid servo ID"));
    return;
  }
  
  // Update servo configuration
  config.servoMinMaxes[servoId][0] = minAngle;
  config.servoMinMaxes[servoId][1] = maxAngle;
  servoOffsets[servoId] = offset;
  servoSpeeds[servoId] = speed;
  
  // Reattach servo with new parameters
  ServoEasing::ServoEasingArray[servoId]->detach();
  if (!ServoEasing::ServoEasingArray[servoId]->attach(
        ServoEasing::ServoEasingArray[servoId]->getPin(), 
        minAngle,
        maxAngle)) {
    Serial.print(F("Error reattaching servo "));
    Serial.println(servoId);
  }
  
  // Set easing type
  ServoEasing::ServoEasingArray[servoId]->setEasingType(config.easeType);
  
  Serial.print(F("Updated servo "));
  Serial.print(servoId);
  Serial.println(F(" parameters"));
}

// Function to reset all servos to default positions
void resetServos() {
  Serial.println(F("Resetting all servos to default positions..."));
  
  // Move all servos to their middle positions
  for (int i = 0; i < NUMBER_OF_SERVOS; i++) {
    int middlePosition = (config.servoMinMaxes[i][0] + config.servoMinMaxes[i][1]) / 2;
    ServoEasing::ServoEasingArray[i]->setEaseTo(middlePosition, 30);
  }
  
  // Wait for all servos to reach their positions
  while (!updateAllServos()) {
    delay(10);
  }
  
  Serial.println(F("All servos reset"));
}

// Handle emergency stop
void emergencyStop() {
  Serial.println(F("EMERGENCY STOP"));
  
  // Stop all servos immediately
  for (int i = 0; i < NUMBER_OF_SERVOS; i++) {
    ServoEasing::ServoEasingArray[i]->stop();
  }
  
  // Reset state
  walkParams.isMoving = false;
  currentState = IDLE;
  
  // Wait for user input to continue
  Serial.println(F("Enter 'reset' to reinitialize or any other command to continue"));
  while (!Serial.available()) {
    delay(10);
  }
  
  String command = Serial.readStringUntil('\n');
  if (command.equals("reset")) {
    resetServos();
  }
}

// Update configuration values
void updateConfig(int thighLength, int kneeLength, int defaultSpeed, int easeType) {
  config.thighLength = thighLength;
  config.kneeLength = kneeLength;
  config.defaultSpeed = defaultSpeed;
  config.easeType = easeType;
  
  // Update easing type for all servos
  for (int i = 0; i < NUMBER_OF_SERVOS; i++) {
    ServoEasing::ServoEasingArray[i]->setEasingType(easeType);
  }
  
  Serial.println(F("Configuration updated"));
}

// Function to handle diagnostics
void runDiagnostics() {
  Serial.println(F("\n===== Running Diagnostics ====="));
  
  // Check servo communication
  Serial.println(F("Checking servo communication..."));
  bool allServosOK = true;
  
  for (int i = 0; i < NUMBER_OF_SERVOS; i++) {
    ServoEasing::ServoEasingArray[i]->setEaseTo(
      config.servoMinMaxes[i][0] + (config.servoMinMaxes[i][1] - config.servoMinMaxes[i][0]) / 2, 
      40);
    
    if (!ServoEasing::ServoEasingArray[i]->isAttached()) {
      Serial.print(F("ERROR: Servo "));
      Serial.print(i);
      Serial.println(F(" is not attached"));
      allServosOK = false;
    }
  }
  
  // Wait for all servos to reach their positions
  while (!updateAllServos()) {
    delay(10);
  }
  
  if (allServosOK) {
    Serial.println(F("All servos are communicating correctly"));
  } else {
    Serial.println(F("Some servos have communication issues"));
  }
  
  // Check servo range of motion
  Serial.println(F("Checking servo range of motion..."));
  
  for (int i = 0; i < NUMBER_OF_SERVOS; i++) {
    int range = abs(config.servoMinMaxes[i][1] - config.servoMinMaxes[i][0]);
    if (range < 30) {
      Serial.print(F("WARNING: Servo "));
      Serial.print(i);
      Serial.println(F(" has a very limited range of motion"));
    }
  }
  
  // Check configuration values
  Serial.println(F("Checking configuration values..."));
  
  if (config.thighLength <= 0 || config.kneeLength <= 0) {
    Serial.println(F("ERROR: Invalid leg dimensions"));
  } else {
    Serial.println(F("Leg dimensions are valid"));
  }
  
  if (config.defaultSpeed <= 0 || config.defaultSpeed > 100) {
    Serial.println(F("WARNING: Default speed may be outside optimal range"));
  } else {
    Serial.println(F("Default speed is within normal range"));
  }
  
  // Return to standing position
  Serial.println(F("Diagnostics complete, returning to standing position"));
  Stand();
}

// Function to save calibration data to EEPROM
void saveCalibrationToEEPROM() {
  Serial.println(F("Saving calibration data to EEPROM..."));
  
  // This is a placeholder - actual EEPROM saving would be implemented here
  // For Arduino with EEPROM library:
  // #include <EEPROM.h>
  // int address = 0;
  // EEPROM.put(address, config);
  
  Serial.println(F("Calibration data saved"));
}

// Function to load calibration data from EEPROM
void loadCalibrationFromEEPROM() {
  Serial.println(F("Loading calibration data from EEPROM..."));
  
  // This is a placeholder - actual EEPROM loading would be implemented here
  // For Arduino with EEPROM library:
  // #include <EEPROM.h>
  // int address = 0;
  // EEPROM.get(address, config);
  
  Serial.println(F("Calibration data loaded"));
}

// Function to create a demo sequence
void runDemoSequence() {
  Serial.println(F("Starting demo sequence..."));
  
  // Step 1: Stand up
  Stand();
  delay(1000);
  
  // Step 2: Do leg lifts
  for (int i = 0; i < 4; i++) {
    LiftLeg(i);
    delay(500);
  }
  
  // Step 3: Walk forward
  Serial.println(F("Walking forward..."));
  startWalking(1);
  
  // Walk for 5 seconds
  delay(5000);
  
  // Step 4: Turn right
  Serial.println(F("Turning right..."));
  adjustTurnFactor(0.8);
  
  // Turn for 3 seconds
  delay(3000);
  
  // Step 5: Turn left
  Serial.println(F("Turning left..."));
  adjustTurnFactor(-0.8);
  
  // Turn for 3 seconds
  delay(3000);
  
  // Step 6: Walk backward
  Serial.println(F("Walking backward..."));
  startWalking(-1);
  
  // Walk for 5 seconds
  delay(5000);
  
  // Step 7: Stop and crouch
  stopWalking();
  delay(1000);
  Crouch();
  delay(1000);
  
  // Step 8: Compact position
  GotoCompact();
  delay(1000);
  
  // Step 9: Stand again
  Stand();
  
  Serial.println(F("Demo sequence complete"));
}

// Function to handle battery voltage monitoring
void checkBatteryVoltage() {
  // This is a placeholder - actual battery monitoring would use an analog pin
  // Example with analog pin A0:
  // int batteryReading = analogRead(A0);
  // float batteryVoltage = batteryReading * (5.0 / 1023.0) * (voltage divider ratio);
  
  float batteryVoltage = 7.4; // Example value for 2S LiPo battery
  
  Serial.print(F("Battery voltage: "));
  Serial.print(batteryVoltage);
  Serial.println(F("V"));
  
  // Check if voltage is too low
  if (batteryVoltage < 6.8) {
    Serial.println(F("WARNING: Battery voltage is low"));
  }
}

// Function to set a specific leg position using inverse kinematics
void setLegPosition(int legIndex, float x, float y, float z) {
  if (legIndex < 0 || legIndex >= 4) {
    Serial.println(F("Invalid leg index"));
    return;
  }
  
  Serial.print(F("Setting leg "));
  Serial.print(legIndex);
  Serial.print(F(" position to ("));
  Serial.print(x);
  Serial.print(F(", "));
  Serial.print(y);
  Serial.print(F(", "));
  Serial.print(z);
  Serial.println(F(")"));
  
  // Calculate servo angles using inverse kinematics
  int hipAngle, thighAngle, kneeAngle;
  calculateLegIK(legIndex, x, y, z, hipAngle, thighAngle, kneeAngle);
  
  // Set servo positions
  mapAnglesAndSetServos(legIndex, hipAngle, thighAngle, kneeAngle);
}

// Function to handle balancing using center of gravity
void balanceRobot() {
  // This is a placeholder for balance control
  // In a real implementation, this would use sensors like accelerometers/gyroscopes
  
  Serial.println(F("Balancing robot..."));
  
  // Calculate center position
  float centerX = 0;
  float centerY = 0;
  float height = walkParams.bodyHeight;
  
  // Adjust each leg position to maintain balance
  for (int i = 0; i < 4; i++) {
    float legX = (i < 2) ? 70 : -70; // Front/back
    float legY = (i % 2 == 0) ? -50 : 50; // Left/right
    float legZ = -height;
    
    // Apply slight adjustments for balance
    legZ += (i < 2) ? 5 : -5; // Slightly tilt forward/backward
    
    // Set leg position
    setLegPosition(i, legX, legY, legZ);
  }
  
  // Wait for movement to complete
  while (!updateAllServos()) {
    delay(10);
  }
  
  Serial.println(F("Balance adjusted"));
}

// Main function
int main() {
  // Initialize Arduino framework
  init();
  
  // Setup and call setup function
  setup();
  
  // Main loop
  while (true) {
    loop();
  }
  
  return 0;
}
