/* =============================================================================
 * WRO FUTURE ENGINEERS — OBSTACLE CHALLENGE FIRMWARE (OPTIMIZED)
 * (BLUETOOTH SERIAL + GYRO 75-DEG PARKING EXIT + POST-TURN STABILIZATION + TUNED RED)
 * ESP32 | BNO055 | HuskyLens | PCA9685-style servo steering | 3x HC-SR04
 * ============================================================================= */

#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "HUSKYLENS.h"
#include "BluetoothSerial.h"

// ==============================
// Bluetooth Initialization
// ==============================
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
BluetoothSerial SerialBT;

// ==============================
// Hardware Pins
// ==============================
#define IN1 13
#define IN2 12
#define ENA 25
#define SERVO_PIN 4

#define TRIG_FRONT 5
#define ECHO_FRONT 18
#define TRIG_RIGHT 17
#define ECHO_RIGHT 16
#define TRIG_LEFT 26
#define ECHO_LEFT 27

#define START_BTN_PIN 0 

// ==============================
// Turn Stop Settings
// ==============================
int TURN = 0;
#define STOP_AFTER_TIME 4500
#define STOP_BOT_AFTER_TURNS 12
unsigned long turnStartTime = 0;

// ==============================
// Servo Constraints
// ==============================
#define SERVO_CENTER_DEG 90
#define SERVO_MIN_DEG 28
#define SERVO_MAX_DEG 142
#define SERVO_SHIFT_LEFT 40
#define SERVO_SHIFT_RIGHT 142  

// ==============================
// Sensors & Connectivity
// ==============================
Servo steeringServo;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
HUSKYLENS huskylens;

bool isGreenBlock(int id) { return (id == 1 || id == 2 || id == 3); }
bool isRedBlock(int id) { return (id == 4 || id == 5 || id == 6); }

#define ID_MAGENTA 7

// ==============================
// Vision thresholds
// ==============================
#define HUSKY_SOFT_TRIGGER 300   
#define HUSKY_AREA_TRIGGER 700  

// ==============================
// Drive params 
// ==============================
#define BASE_SPEED 95             
#define WALL_STOP_DIST 60
#define TURN_EARLY_DONE_TOL 13.0f
#define TURN_SLOW_ZONE_DEG 32.0f
#define TURN_SLOW_SPEED 135       
#define TURN_FAST_SPEED 200       

#define POST_TURN_VISION_HOLD_MS 1200 

// ==============================
// Maneuver tuning 
// ==============================
#define MANEUVER_REVERSE_SPEED 150   
#define MANEUVER_SHIFT_SPEED 135    
#define MANEUVER_SHIFT_OUT_MS 850   
#define MANEUVER_CLEAR_MS 700
#define MANEUVER_SHIFT_BACK_MS 650
#define MANEUVER_MIN_CLEARANCE 18.0f 
#define BLOCK_TOO_CLOSE_CM 20.0f
#define BLOCK_SAFE_DISTANCE_CM 42.0f
#define BLOCK_REVERSE_MAX_MS 900

#define RED_AREA_TRIGGER 600
#define RED_SHIFT_OUT_EXTRA_MS 150   
#define RED_CLEAR_EXTRA_MS 220       
#define RED_MIN_CLEARANCE 24.0f
#define RED_SHIFT_BACK_MS 500
#define RED_SIDE_ESCAPE_MS 450

// ==============================
// Pillar cooldown / lockout
// ==============================
#define PILLAR_COOLDOWN_MS 1800   
#define PILLAR_COOLDOWN_MAX_MS 3500   
#define EST_CM_PER_S_AT_BASE 35.0f  
#define PILLAR_COOLDOWN_CM 45.0f  

#define PILLAR_VIEW_LATCH_MS 300  
#define CREEP_SPEED 115   
#define WALL_HOLD_DIST 18.0f
#define TURN_COMMIT_DEG 45.0f  

// ==============================
// PID & State Variables
// ==============================
float Kp = 1.4, Ki = 0.05, Kd = 1.5;
float headingError, lastError = 0, integral = 0;
float targetHeading = 0;
unsigned long lastPIDTime = 0; // Added for time-dependent PID

enum State { STRAIGHT, TURNING };
State state = STRAIGHT;
float turnTarget = 0;
bool isClockwiseRun = true;

unsigned long lastSerialTime = 0;
int currentSteerDeg = SERVO_CENTER_DEG;
int currentMotorSpeed = 0;
float frontDist = 999, leftDist = 999, rightDist = 999;
bool isBlockInSight = false;

unsigned long lastPillarSeenMs = 0;     
bool pillarCooldownArmed = false;
unsigned long pillarCooldownStartMs = 0;
float cooldownTravelCm = 0;
unsigned long lastTravelUpdateMs = 0;
float turnStartHeading = 0;    
unsigned long postTurnVisionHoldUntilMs = 0;

// ==============================
// Prototypes
// ==============================
void setSteeringDeg(int angleDeg);
void motorForward(int speed);
void motorBackward(int speed);
void motorRampForward(int targetSpeed);
void motorRampBackward(int targetSpeed);
void motorStop();
float readDistanceCM(int trigPin, int echoPin);
float getHeading();
float headingDiff(float target, float current);
void calibrateIMU();
void driveStraightPID();
void decideTurn();
void executeTurn();
void executeTurnSetup(bool isRightTurn);
void printTelemetry();
void executeAvoidanceManeuver(bool isGreen);
void executeTinyNudge(bool isGreen);
void executeGreenManeuver();
void executeRedManeuver();
void reverseIfBlockTooClose();
bool handleVision(bool turnCommitted);
bool scanForPillar(int &id, int &x, int &area);
bool pillarInView();
bool pillarVisibleNow();
bool pillarCooldownActive();
void armPillarCooldown();
void updateTravelEstimate();
bool turnAllowed();
void executeDeadlockRecovery();
bool pillarClearedFromSide(bool wasGreen);
void exitParkingSequence(float startH);

// ==============================
// Setup
// ==============================
void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  if (!SerialBT.begin("WRO_ESP32_Bot")) {
    Serial.println("Bluetooth init failed!");
  }
  
  delay(1000);

  pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT); pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  steeringServo.attach(SERVO_PIN);
  setSteeringDeg(SERVO_CENTER_DEG);
  delay(500);

  bool bnoOK = false;
  for (int i = 0; i < 5 && !bnoOK; i++) {
    bnoOK = bno.begin();
    if (!bnoOK) { Serial.println("BNO055 retry..."); delay(500); }
  }
  if (!bnoOK) {
    Serial.println("BNO055 not detected after retries — check wiring/power.");
    while (1) delay(10);
  }
  bno.setExtCrystalUse(true);
  delay(800);

  while (!huskylens.begin(Wire)) {
    Serial.println("HuskyLens failed! Check wiring.");
    delay(500);
  }
  huskylens.writeAlgorithm(ALGORITHM_COLOR_RECOGNITION);

  calibrateIMU();

  Serial.println("Letting IMU internal filters stabilize...");
  delay(3000); 

  float initialLeftDist = readDistanceCM(TRIG_LEFT, ECHO_LEFT);
  delay(20); // Ultrasonic spacing
  float initialRightDist = readDistanceCM(TRIG_RIGHT, ECHO_RIGHT);
  
  if (initialLeftDist > 0 && initialLeftDist < 60.0f) {
    isClockwiseRun = true;
    Serial.println("Wall on LEFT -> CLOCKWISE RUN");
  } else {
    isClockwiseRun = false;
    Serial.println("Wall on RIGHT -> ANTI-CLOCKWISE RUN");
  }
  
  pinMode(START_BTN_PIN, INPUT_PULLUP);
  Serial.println("Waiting for START button...");
  
  while (digitalRead(START_BTN_PIN) == HIGH) delay(10);

  float startH = getHeading();
  
  exitParkingSequence(startH);

  targetHeading = startH;
  lastTravelUpdateMs = millis();
  lastPIDTime = millis();

  motorForward(0);
  Serial.printf("--- ROBOT READY FOR TRACK --- initial targetHeading %.1f\n", targetHeading);
  delay(1000);
}

// ==============================
// GYRO-BASED PARKING EXIT
// ==============================
void exitParkingSequence(float startH) {
  float currentHeading = startH;
  float accumulatedAngle = 0.0;
  float targetEscapeAngle = 75.0; 
  int TURN_SPEED = 120; 

  int steerOut = isClockwiseRun ? SERVO_MAX_DEG : SERVO_MIN_DEG; 
  int steerAlign = isClockwiseRun ? SERVO_MIN_DEG : SERVO_MAX_DEG;

  while (accumulatedAngle < targetEscapeAngle) {
    setSteeringDeg(steerOut); 
    delay(300); 
    motorForward(TURN_SPEED);
    delay(300); 
    motorStop(); delay(150);

    setSteeringDeg(SERVO_CENTER_DEG); 
    delay(300); 
    motorBackward(TURN_SPEED);
    delay(200); 
    motorStop(); delay(150);

    currentHeading = getHeading();
    accumulatedAngle = fabs(headingDiff(startH, currentHeading));
  }
  
  setSteeringDeg(steerAlign); 
  delay(300); 
  motorForward(TURN_SPEED);

  unsigned long exitStartTime = millis();
  while (true) {
    float diffFromStraight = fabs(headingDiff(startH, getHeading()));
    if (diffFromStraight < 4.0) break;
    if (millis() - exitStartTime > 3000) break;
    delay(20);
  }

  motorStop();
  setSteeringDeg(SERVO_CENTER_DEG);
  delay(200);

  motorBackward(200); delay(40); 
  motorBackward(130);
  
  unsigned long revStart = millis();
  while(millis() - revStart < 1200) { 
      float err = headingDiff(startH, getHeading());
      setSteeringDeg(SERVO_CENTER_DEG - constrain((int)(Kp * err), -25, 25));
      delay(20);
  }
  motorStop(); delay(200);
  
  postTurnVisionHoldUntilMs = millis() + POST_TURN_VISION_HOLD_MS;
}

// ==============================
// Main Loop
// ==============================
void loop() {
  if (Serial.available() > 0 || SerialBT.available() > 0) {
    String input = Serial.available() > 0 ? Serial.readStringUntil('\n') : SerialBT.readStringUntil('\n');
    input.trim();
    if (input.equalsIgnoreCase("green")) executeGreenManeuver();
    if (input.equalsIgnoreCase("red")) executeRedManeuver();
  }

  updateTravelEstimate();   

  if (TURN >= STOP_BOT_AFTER_TURNS && state == STRAIGHT && millis() - turnStartTime >= STOP_AFTER_TIME) {
    motorStop();
    setSteeringDeg(SERVO_CENTER_DEG);
    while (1) delay(10);
  }

  bool turnCommitted = false;
  if (state == TURNING) {
    float progressed = fabs(headingDiff(getHeading(), turnStartHeading));
    turnCommitted = (progressed >= TURN_COMMIT_DEG);
  }

  if (state == STRAIGHT) {
    bool cameraInControl = handleVision(turnCommitted);
    if (cameraInControl) {
      printTelemetry();
      return; 
    }

    frontDist = readDistanceCM(TRIG_FRONT, ECHO_FRONT);
    driveStraightPID();   

    bool wallAhead = (frontDist > 5 && frontDist < WALL_STOP_DIST);
    if (!wallAhead) {
        printTelemetry();
        return;
    }

    if (!turnAllowed()) {
      if (frontDist <= WALL_HOLD_DIST) executeDeadlockRecovery();
      else motorForward(CREEP_SPEED);
      printTelemetry();
      return;
    }

    delay(30);
    float confirmDist = readDistanceCM(TRIG_FRONT, ECHO_FRONT);
    if (confirmDist > 5 && confirmDist < WALL_STOP_DIST) {
      motorStop();
      delay(200);

      if (pillarVisibleNow() || !turnAllowed()) {
        printTelemetry();
        return;
      }
      decideTurn();
    }
  } 
  else if (state == TURNING) {
    executeTurn();
  }
  printTelemetry();
}

// =====================================================================
// Deadlock Recovery Failsafe
// =====================================================================
void executeDeadlockRecovery() {
  setSteeringDeg(SERVO_CENTER_DEG);
  delay(250); 
  motorRampBackward(BASE_SPEED); 

  unsigned long reverseStart = millis();
  float currentDist = readDistanceCM(TRIG_FRONT, ECHO_FRONT);

  while ((currentDist < 66.0f || currentDist > 900.0f) && (millis() - reverseStart < 3000)) {
    currentDist = readDistanceCM(TRIG_FRONT, ECHO_FRONT);
    delay(20); 
  }

  motorStop();
  delay(200); 

  pillarCooldownArmed = false; 

  int id = 0, x = 0, area = 0;
  if (scanForPillar(id, x, area) && area > HUSKY_AREA_TRIGGER && (isRedBlock(id) || isGreenBlock(id))) {
    if (isGreenBlock(id)) executeAvoidanceManeuver(true);
    else executeAvoidanceManeuver(false);
    return;
  }

  float chk = readDistanceCM(TRIG_FRONT, ECHO_FRONT);
  if (chk > 5 && chk < WALL_STOP_DIST) decideTurn();
  else state = STRAIGHT;
}

// =====================================================================
// Pillar cooldown / lockout 
// =====================================================================
void armPillarCooldown() {
  pillarCooldownArmed = true;
  pillarCooldownStartMs = millis();
  cooldownTravelCm = 0;          
  lastTravelUpdateMs = millis();   
}

void updateTravelEstimate() {
  unsigned long now = millis();
  unsigned long dtMs = now - lastTravelUpdateMs;
  lastTravelUpdateMs = now;

  if (dtMs == 0 || dtMs > 1000) return;   
  if (currentMotorSpeed <= 0) return;

  float dt = dtMs / 1000.0f;
  cooldownTravelCm += ((float)currentMotorSpeed / (float)BASE_SPEED) * EST_CM_PER_S_AT_BASE * dt;
}

bool pillarCooldownActive() {
  if (!pillarCooldownArmed) return false;
  unsigned long elapsed = millis() - pillarCooldownStartMs;
  if (elapsed >= PILLAR_COOLDOWN_MAX_MS) { pillarCooldownArmed = false; return false; }
  if (elapsed < PILLAR_COOLDOWN_MS) return true;
  if (cooldownTravelCm < PILLAR_COOLDOWN_CM) return true;

  pillarCooldownArmed = false;
  return false;
}

bool pillarInView() { return (lastPillarSeenMs != 0) && ((millis() - lastPillarSeenMs) < PILLAR_VIEW_LATCH_MS); }
bool turnAllowed() { return (!pillarInView()) && (!pillarCooldownActive()); }

bool pillarVisibleNow() {
  int id = 0, x = 160, area = 0;
  if (!scanForPillar(id, x, area)) return false;
  if (area <= HUSKY_SOFT_TRIGGER) return false;
  if (!isRedBlock(id) && !isGreenBlock(id)) return false;   

  lastPillarSeenMs = millis();
  armPillarCooldown();
  return true;
}

bool scanForPillar(int &id, int &x, int &area) {
  huskylens.request();
  if (!huskylens.available()) return false;

  int largestArea = 0;
  int detectedID = 0;
  int detectedX = 160; 

  while (huskylens.available()) {
    HUSKYLENSResult result = huskylens.read();
    int a = result.width * result.height;
    int tempID = result.ID;
    
    if (tempID == ID_MAGENTA) {
        bool isTall = (result.height * 10 >= result.width * 7); 
        if (!isTall) continue; 
        tempID = 4;            
    }

    if (a > largestArea) {
      largestArea = a;
      detectedID = tempID;
      detectedX = result.xCenter;
    }
  }

  id = detectedID; x = detectedX; area = largestArea;
  return true;
}

// =====================================================================
// 3-ZONE GRID AVOIDANCE LOGIC
// =====================================================================
bool handleVision(bool turnCommitted) {
  if (millis() < postTurnVisionHoldUntilMs) return false;

  int detectedID = 0, detectedX = 160, largestArea = 0;
  if (!scanForPillar(detectedID, detectedX, largestArea)) {
    isBlockInSight = false;
    return false;
  }

  isBlockInSight = (largestArea > HUSKY_SOFT_TRIGGER);
  if (!isBlockInSight) return false;
  if (detectedID == ID_MAGENTA) return false; 
  if (!isRedBlock(detectedID) && !isGreenBlock(detectedID)) return false; 

  if (pillarCooldownActive()) {
      if (largestArea < HUSKY_AREA_TRIGGER * 1.5) return false; 
  }

  lastPillarSeenMs = millis();
  armPillarCooldown();

  if (turnCommitted) return false;   

  int hardTrigger = isRedBlock(detectedID) ? RED_AREA_TRIGGER : HUSKY_AREA_TRIGGER;

  if (largestArea > hardTrigger) {
    if (isGreenBlock(detectedID)) {
      if (detectedX > 213) executeTinyNudge(true); 
      else executeAvoidanceManeuver(true); 
    } else {
      if (detectedX < 106) executeTinyNudge(false); 
      else executeAvoidanceManeuver(false); 
    }
    return true;
  }

  int intensity = constrain(map(largestArea, HUSKY_SOFT_TRIGGER, HUSKY_AREA_TRIGGER, 30, 65), 30, 65);
  int steerAdj = 0;

  if (isRedBlock(detectedID)) {
    if (detectedX > 213) steerAdj = intensity; 
    else if (detectedX > 106) steerAdj = (int)(intensity * 1.5); 
    else steerAdj = (int)(intensity * 0.5); 
  } else {
    if (detectedX < 106) steerAdj = -intensity; 
    else if (detectedX < 213) steerAdj = (int)(-intensity * 1.5); 
    else steerAdj = (int)(-intensity * 0.5); 
  }
  
  setSteeringDeg(SERVO_CENTER_DEG + constrain(steerAdj, -60, 60));
  motorForward(BASE_SPEED);
  return true;
}

bool pillarClearedFromSide(bool wasGreen) {
  int id = 0, x = 160, area = 0;
  if (!scanForPillar(id, x, area)) return true;      
  if (area <= HUSKY_SOFT_TRIGGER) return true;        
  return wasGreen ? !isGreenBlock(id) : !isRedBlock(id);
}

void reverseIfBlockTooClose() {
  float dist = readDistanceCM(TRIG_FRONT, ECHO_FRONT);
  if (dist <= BLOCK_TOO_CLOSE_CM || dist > 900.0f) {
    setSteeringDeg(SERVO_CENTER_DEG);
    motorBackward(200); delay(40);
    motorBackward(MANEUVER_REVERSE_SPEED);

    unsigned long reverseStart = millis();
    while (millis() - reverseStart < BLOCK_REVERSE_MAX_MS) {
      dist = readDistanceCM(TRIG_FRONT, ECHO_FRONT);
      frontDist = dist;
      if (dist >= BLOCK_SAFE_DISTANCE_CM && dist < 900.0f) break;
      delay(20);
    }
    motorStop(); delay(120);
  }
}

// ---------------- Maneuver Sequences ----------------
void executeTinyNudge(bool isGreen) {
  int shiftOutDeg = isGreen ? SERVO_SHIFT_LEFT : SERVO_SHIFT_RIGHT;
  motorStop(); delay(80);
  reverseIfBlockTooClose();
  
  setSteeringDeg(shiftOutDeg);
  motorForward(300); delay(40);
  motorForward(BASE_SPEED);
  delay(250); 
  
  setSteeringDeg(SERVO_CENTER_DEG); 
  integral = 0; lastError = 0;
  state = STRAIGHT;
  armPillarCooldown();
  lastPillarSeenMs = millis();
  lastPIDTime = millis();
}

void executeAvoidanceManeuver(bool isGreen) {
  int shiftOutDeg = isGreen ? SERVO_SHIFT_LEFT : SERVO_SHIFT_RIGHT;
  int shiftBackDeg = isGreen ? SERVO_SHIFT_RIGHT : SERVO_SHIFT_LEFT;
  int checkTrig = isGreen ? TRIG_RIGHT : TRIG_LEFT; 
  int checkEcho = isGreen ? ECHO_RIGHT : ECHO_LEFT;
  int shiftOutMs = isGreen ? MANEUVER_SHIFT_OUT_MS : MANEUVER_SHIFT_OUT_MS + RED_SHIFT_OUT_EXTRA_MS;
  int clearMaxMs = isGreen ? MANEUVER_CLEAR_MS + 400 : MANEUVER_CLEAR_MS + 400 + RED_CLEAR_EXTRA_MS;
  int shiftBackMs = isGreen ? MANEUVER_SHIFT_BACK_MS : RED_SHIFT_BACK_MS;
  float minSideClearance = isGreen ? MANEUVER_MIN_CLEARANCE : RED_MIN_CLEARANCE;

  motorStop(); delay(100);
  setSteeringDeg(SERVO_CENTER_DEG);

  reverseIfBlockTooClose();

  setSteeringDeg(shiftOutDeg); delay(150);
  
  motorForward(200); delay(40);
  motorForward(MANEUVER_SHIFT_SPEED);
  delay(shiftOutMs);
  motorStop(); delay(100);

  setSteeringDeg(SERVO_CENTER_DEG); delay(100);
  
  motorForward(200); delay(40);
  motorForward(MANEUVER_SHIFT_SPEED);
  
  unsigned long clearStart = millis();
  bool cleared = false;
  while (millis() - clearStart < clearMaxMs) {
    if ((millis() - clearStart) >= MANEUVER_CLEAR_MS && pillarClearedFromSide(isGreen)) {
      cleared = true; break;
    }
    delay(20);
  }
  motorStop(); delay(100);

  if (!cleared) {
    motorForward(200); delay(40);
    motorForward(MANEUVER_SHIFT_SPEED);
    unsigned long extraStart = millis();
    while (millis() - extraStart < 400 && !pillarClearedFromSide(isGreen)) delay(20);
    motorStop(); delay(100);
  }

  float sideDist = readDistanceCM(checkTrig, checkEcho);
  if (sideDist > 0 && sideDist < minSideClearance) {
    if (!isGreen) setSteeringDeg(SERVO_SHIFT_RIGHT);
    motorForward(200); delay(40);
    motorForward(MANEUVER_SHIFT_SPEED);
    delay(400); 
    motorStop(); delay(100);
    if (!isGreen) setSteeringDeg(SERVO_CENTER_DEG);
  }

  if (!isGreen) {
    float redSideDist = readDistanceCM(TRIG_LEFT, ECHO_LEFT);
    if (redSideDist > 0 && redSideDist < RED_MIN_CLEARANCE) {
      setSteeringDeg(SERVO_SHIFT_RIGHT);
      motorForward(200); delay(40);
      motorForward(MANEUVER_SHIFT_SPEED);
      unsigned long escapeStart = millis();
      while (millis() - escapeStart < RED_SIDE_ESCAPE_MS) {
        redSideDist = readDistanceCM(TRIG_LEFT, ECHO_LEFT);
        if (redSideDist > RED_MIN_CLEARANCE || redSideDist > 900) break;
        delay(20);
      }
      motorStop(); delay(100);
      setSteeringDeg(SERVO_CENTER_DEG);
      delay(80);
    }
  }

  setSteeringDeg(shiftBackDeg); delay(100);
  motorForward(200); delay(40);
  motorForward(MANEUVER_SHIFT_SPEED);
  delay(shiftBackMs);

  setSteeringDeg(SERVO_CENTER_DEG);
  motorStop(); delay(150);

  unsigned long alignStart = millis();
  while(millis() - alignStart < 500) {
    float currHeading = getHeading();
    float err = headingDiff(targetHeading, currHeading);

    int vid = 0, vx = 160, varea = 0;
    int camBias = 0;
    
    if (scanForPillar(vid, vx, varea) && (isRedBlock(vid) || isGreenBlock(vid))) {
      if (varea > HUSKY_AREA_TRIGGER) break; 
      if (varea > HUSKY_SOFT_TRIGGER) camBias = constrain(-(vx - 160) / 5, -30, 30);   
    }

    int steerCmd = constrain(SERVO_CENTER_DEG + (int)(Kp * err) + camBias, SERVO_MIN_DEG, SERVO_MAX_DEG);
    setSteeringDeg(steerCmd);
    motorForward(BASE_SPEED);
    delay(20);
  }

  setSteeringDeg(SERVO_CENTER_DEG);
  motorStop(); delay(150);

  integral = 0; lastError = 0;
  state = STRAIGHT;
  armPillarCooldown();
  lastPillarSeenMs = millis();
  lastPIDTime = millis();
}

void executeGreenManeuver() { executeAvoidanceManeuver(true); }
void executeRedManeuver() { executeAvoidanceManeuver(false); }

// ---------------- Serial Monitoring ----------------
void printTelemetry() {
  if (millis() - lastSerialTime < 150) return;
  lastSerialTime = millis();

  float currHeading = getHeading();
  String prefix = (state == STRAIGHT) ? "[STR] " : "[TRN] ";
  
  char buffer[128];
  snprintf(buffer, sizeof(buffer), "F:%4.1f L:%4.1f R:%4.1f | H:%5.1f T:%5.1f E:%5.1f", 
           frontDist, leftDist, rightDist, currHeading, targetHeading, headingError);

  Serial.print(prefix); Serial.println(buffer);
  if(SerialBT.hasClient()) { SerialBT.print(prefix); SerialBT.println(buffer); }
}

// ---------------- Ultrasonic ----------------
float readDistanceCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 15000);
  if (duration == 0) return 999;
  return duration * 0.0343f / 2.0f;
}

// ---------------- IMU Calibration & Data ----------------
void calibrateIMU() {
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  while (gyro < 3) {
    bno.getCalibration(&system, &gyro, &accel, &mag);
    delay(200);
  }
}

float getHeading() {
  imu::Vector<3> e = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  return e.x();
}

float headingDiff(float target, float current) {
  float d = target - current;
  while (d > 180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

// ---------------- Steering ----------------
void setSteeringDeg(int angleDeg) {
  currentSteerDeg = constrain(angleDeg, SERVO_MIN_DEG, SERVO_MAX_DEG);
  steeringServo.write(currentSteerDeg);
}

// ---------------- Soft Start Motors ----------------
void motorRampForward(int targetSpeed) {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  for (int s = 100; s <= targetSpeed; s += 15) { analogWrite(ENA, s); delay(2); }
  currentMotorSpeed = targetSpeed;
  analogWrite(ENA, targetSpeed);
}

void motorRampBackward(int targetSpeed) {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  for (int s = 100; s <= targetSpeed; s += 15) { analogWrite(ENA, s); delay(2); }
  currentMotorSpeed = -targetSpeed;
  analogWrite(ENA, targetSpeed);
}

void motorForward(int speed) {
  currentMotorSpeed = speed;
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  analogWrite(ENA, speed);
}

void motorBackward(int speed) {
  currentMotorSpeed = -speed;
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  analogWrite(ENA, speed);
}

void motorStop() {
  currentMotorSpeed = 0;
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 255); // Active brake instead of coasting
}

// =====================================================================
// STRAIGHT DRIVE: PID WITH TIME DELTA & CROSS-TALK DELAY
// =====================================================================
void driveStraightPID() {
  float current = getHeading();
  headingError = headingDiff(targetHeading, current);

  float centerBias = 0.0;
  
  // Ultrasonic Cross-talk prevention:
  leftDist = readDistanceCM(TRIG_LEFT, ECHO_LEFT);
  delay(15); 
  rightDist = readDistanceCM(TRIG_RIGHT, ECHO_RIGHT);

  if (TURN > 0 && fabs(headingError) < 25.0f) {
      if (leftDist > 0 && leftDist < 100.0f && rightDist > 0 && rightDist < 100.0f) {
          centerBias = (rightDist - leftDist) * 1.0f;
      } else if (leftDist > 0 && leftDist < 100.0f) {
          centerBias = (40.0f - leftDist) * 2.2f; 
      } else if (rightDist > 0 && rightDist < 100.0f) {
          centerBias = (rightDist - 40.0f) * 2.2f; 
      }
  }

  float activeError = headingError + constrain(centerBias, -35.0f, 35.0f);

  // Time-dependent PID integration
  unsigned long now = millis();
  float dt = (now - lastPIDTime) / 1000.0f;
  if (dt <= 0.0f) dt = 0.01f; 
  lastPIDTime = now;

  integral += activeError * dt;
  integral = constrain(integral, -30, 30);
  float derivative = (activeError - lastError) / dt;
  lastError = activeError;

  float correction = Kp * activeError + Ki * integral + Kd * derivative;
  setSteeringDeg(SERVO_CENTER_DEG + constrain((int)correction, -40, 40));
  motorForward(BASE_SPEED);
}

// =====================================================================
// NEW: CLASSIC 3-POINT REVERSE SWING TURN (QUICK DECISION)
// =====================================================================
void decideTurn() {
  unsigned long creepStart = millis();
  while (millis() - creepStart < 2000) {
    float dist = readDistanceCM(TRIG_FRONT, ECHO_FRONT);
    if (dist > 0 && (dist <= 30.0f || dist > 900.0f)) break;
    
    setSteeringDeg(SERVO_CENTER_DEG + constrain((int)(Kp * headingDiff(targetHeading, getHeading())), -20, 20));
    motorForward(115); delay(20);
  }
  motorStop(); delay(100);

  leftDist = readDistanceCM(TRIG_LEFT, ECHO_LEFT);
  delay(20); // Ultrasonic cross-talk fix
  rightDist = readDistanceCM(TRIG_RIGHT, ECHO_RIGHT);

  if (leftDist < 0) leftDist = 999;
  if (rightDist < 0) rightDist = 999;

  TURN++;
  turnStartTime = millis();
  turnStartHeading = getHeading();   

  bool isRightTurn = (leftDist <= rightDist);
  turnTarget = targetHeading + (isRightTurn ? 90.0f : -90.0f);
  
  // Prevent wrapping drift
  if (turnTarget < 0) turnTarget += 360.0f;
  if (turnTarget >= 360.0f) turnTarget -= 360.0f;

  executeTurnSetup(isRightTurn);
  turnStartTime = millis(); 
  state = TURNING;
}

void executeTurnSetup(bool isRightTurn) {
  setSteeringDeg(isRightTurn ? SERVO_MIN_DEG : SERVO_MAX_DEG);
  delay(150);

  motorBackward(200); delay(50);
  motorBackward(150); 

  unsigned long startRev = millis();
  float startH = getHeading();
  delay(150); 

  while (millis() - startRev < 1200) { 
    if (fabs(headingDiff(startH, getHeading())) >= 45.0f) break;
    delay(20);
  }
  motorStop(); delay(150);
}

void executeTurn() {
  float diff = headingDiff(turnTarget, getHeading());
  headingError = diff;

  if (fabs(diff) <= TURN_EARLY_DONE_TOL) {
    motorStop(); setSteeringDeg(SERVO_CENTER_DEG); delay(150); 

    motorBackward(255); delay(40); 
    motorBackward(140); 
    unsigned long revStart = millis();
    
    while(millis() - revStart < 700) { 
        float revErr = headingDiff(turnTarget, getHeading());
        setSteeringDeg(SERVO_CENTER_DEG - constrain((int)(Kp * revErr), -30, 30)); 
        if (fabs(revErr) <= 1.5f && (millis() - revStart > 150)) break;
        delay(20);
    }
    
    motorStop(); setSteeringDeg(SERVO_CENTER_DEG); delay(200);

    targetHeading = turnTarget;
    integral = 0; lastError = 0; lastPIDTime = millis();
    lastPillarSeenMs = 0; lastTravelUpdateMs = millis();
    
    postTurnVisionHoldUntilMs = millis() + POST_TURN_VISION_HOLD_MS; 
    state = STRAIGHT;
    return;
  }

  setSteeringDeg((diff > 0) ? SERVO_MAX_DEG : SERVO_MIN_DEG);
  motorForward(fabs(diff) > TURN_SLOW_ZONE_DEG ? TURN_FAST_SPEED : TURN_SLOW_SPEED); 

  if (millis() - turnStartTime > 3500) {
      motorStop(); delay(200);
      setSteeringDeg(SERVO_CENTER_DEG); delay(300); 
      motorBackward(200); delay(50);
      motorBackward(140); delay(600); 
      motorStop(); delay(200);
      turnStartTime = millis(); 
  }
}
