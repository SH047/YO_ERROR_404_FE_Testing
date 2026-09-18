#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_NeoPixel.h>

// ==============================
// ESP32-C3 Hardware Pins
// ==============================
#define IN1_PIN 0      
#define IN2_PIN 1      
#define SERVO_PIN 6    
#define NEOPIXEL_PIN 7 
#define START_BTN_PIN 10 

// ==============================
// I2C Multiplexer Routing
// ==============================
#define TCAADDR 0x70
#define TOF_FRONT_CH 0
#define TOF_RIGHT_CH 1
#define TOF_LEFT_CH 2
#define BNO_CH 4

// ==============================
// Objects & State
// ==============================
Adafruit_VL53L0X tof[3]; 
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
Servo steeringServo;
Adafruit_NeoPixel strip = Adafruit_NeoPixel(16, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Turn Stop Settings
int TURN = 0;
#define STOP_AFTER_TIME 6500
#define STOP_BOT_AFTER_TURNS 12
#define FINISH_LINE_TIME_MS 3000 // Push forward for 3 seconds at the end
unsigned long turnStartTime = 0;
unsigned long lastTurnEndTime = 0; 

// ==============================
// 1. TURN SMOOTHNESS PARAMETERS
// ==============================
#define SERVO_CENTER_DEG 90
#define SERVO_MIN_DEG 28
#define SERVO_MAX_DEG 142       
// Symmetric sharp turning arcs (45 degrees off-center in both directions)
#define SERVO_SHIFT_LEFT 45     
#define SERVO_SHIFT_RIGHT 135   

// ==============================
// 2. TURN DECISION PARAMETERS
// ==============================
#define TURN_THRESHOLD_CM 60.0   // Trigger the turn much earlier
#define SIDE_GAP_THRESHOLD 50.0  // Confirm a real corner and ignore noise
#define TURN_COOLDOWN_MS 1000    // Mandatory straight drive time after a turn

#define BASE_SPEED 190         
#define TURN_SPEED 170

float target_heading = 0.0;
bool isTurning = false;
bool isReversing = false;        
int turnDirection = 0; 
int lockedTurnDirection = 0; 
int currentServoAngle = SERVO_CENTER_DEG;

unsigned long lastSerialTime = 0;

void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void setNeoPixels(uint8_t r, uint8_t g, uint8_t b) {
  for(int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(r, g, b));
  }
  strip.show();
}

float getDistance(uint8_t muxChannel) {
  tcaselect(muxChannel);
  VL53L0X_RangingMeasurementData_t measure;
  tof[muxChannel].rangingTest(&measure, false); 
  
  if (measure.RangeStatus == 4 || measure.RangeMilliMeter > 2000) return 999.0; 
  return measure.RangeMilliMeter / 10.0;      
}

float getHeadingError(float target, float current) {
  float diff = target - current;
  while (diff > 180) diff -= 360;
  while (diff < -180) diff += 360;
  return diff;
}

void driveMotor(int speed, bool forward) {
  if (speed == 0) {
    analogWrite(IN1_PIN, 0);       
    digitalWrite(IN2_PIN, LOW);    
  } else {
    digitalWrite(IN2_PIN, forward ? HIGH : LOW); 
    analogWrite(IN1_PIN, speed);                 
  }
}

void centerWheels() {
  Serial.println("Running wheel centering sweep...");
  steeringServo.write(SERVO_MIN_DEG);
  delay(400);
  steeringServo.write(SERVO_MAX_DEG);
  delay(400);
  steeringServo.write(SERVO_CENTER_DEG);
  delay(500); 
  Serial.println("Wheels centered.");
}

void setup() {
  Serial.begin(115200);
  
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(START_BTN_PIN, INPUT_PULLUP);
  
  strip.begin();
  setNeoPixels(255, 0, 0); 

  Wire.begin(8, 9); 

  steeringServo.setPeriodHertz(50);
  steeringServo.attach(SERVO_PIN, 500, 2400); 
  centerWheels();
  driveMotor(0, true);

  for (int i = 0; i < 3; i++) {
    tcaselect(i);
    if (!tof[i].begin()) {
      Serial.print("ERROR: ToF Channel "); Serial.print(i); Serial.println(" missing!");
      while (1) { setNeoPixels(255, 255, 0); delay(100); setNeoPixels(0,0,0); delay(100); }
    }
  }

  tcaselect(BNO_CH);
  if (!bno.begin()) {
    Serial.println("ERROR: No BNO055 detected!");
    while (1) { setNeoPixels(255, 255, 0); delay(100); setNeoPixels(0,0,0); delay(100); }
  }
  delay(100);
  bno.setExtCrystalUse(true);

  setNeoPixels(0, 255, 0); 
  Serial.println("Hardware Ready. Waiting for START button...");

  while (digitalRead(START_BTN_PIN) == HIGH) { delay(50); }
  delay(50); 
  while (digitalRead(START_BTN_PIN) == LOW) { delay(10); }

  tcaselect(BNO_CH);
  sensors_event_t event;
  bno.getEvent(&event);
  target_heading = event.orientation.x;
  
  setNeoPixels(0, 255, 255); 
  Serial.print("Run Started! Target Heading locked at: ");
  Serial.println(target_heading);
}

void loop() {
  if (TURN >= STOP_BOT_AFTER_TURNS) {
    Serial.print("12 Turns Complete! Actively centering for ");
    Serial.print(FINISH_LINE_TIME_MS / 1000);
    Serial.println(" seconds to cross the section...");
    setNeoPixels(255, 0, 255); 
    
    unsigned long finishStartTime = millis();
    
    // Run the centering loop actively for FINISH_LINE_TIME_MS
    while (millis() - finishStartTime < FINISH_LINE_TIME_MS) {
      float distR = getDistance(TOF_RIGHT_CH);
      float distL = getDistance(TOF_LEFT_CH);
      
      tcaselect(BNO_CH);
      sensors_event_t event;
      bno.getEvent(&event);
      float headingError = getHeadingError(target_heading, event.orientation.x);

      float wallCentering = 0;
      float Kp_Wall = 1.2; 
      float targetWallDist = 42.0; 

      if (distL < 80.0 && distR < 80.0) {
        wallCentering = (distR - distL) * (Kp_Wall * 0.5);
      } else if (lockedTurnDirection == 1) {
        if (distR < 80.0) wallCentering -= (targetWallDist - distR) * Kp_Wall;
        else if (distL < 80.0) wallCentering += (targetWallDist - distL) * Kp_Wall;
      } else if (lockedTurnDirection == -1) {
        if (distL < 80.0) wallCentering += (targetWallDist - distL) * Kp_Wall;
        else if (distR < 80.0) wallCentering -= (targetWallDist - distR) * Kp_Wall;
      } else {
        if (distR < 80.0) wallCentering -= (targetWallDist - distR) * Kp_Wall;
        else if (distL < 80.0) wallCentering += (targetWallDist - distL) * Kp_Wall;
      }

      float correction = (headingError * 1.2) + wallCentering; 
      int finishAngle = SERVO_CENTER_DEG + correction;
      finishAngle = constrain(finishAngle, SERVO_MIN_DEG, SERVO_MAX_DEG);
      
      steeringServo.write(finishAngle);
      driveMotor(BASE_SPEED, true);
      
      delay(15); 
    }
    
    driveMotor(0, true);
    setNeoPixels(255, 0, 0); 
    Serial.println("RACE COMPLETE - System Halted.");
    while(1) { delay(1000); } 
  }

  float distF = getDistance(TOF_FRONT_CH);
  float distR = getDistance(TOF_RIGHT_CH);
  float distL = getDistance(TOF_LEFT_CH);

  tcaselect(BNO_CH);
  sensors_event_t event;
  bno.getEvent(&event);
  float current_heading = event.orientation.x;
  float headingError = getHeadingError(target_heading, current_heading);

  if (!isTurning) {
    bool shouldTurn = false;

    // Trigger turn ONLY when distance is <= 60cm
    if (distF <= TURN_THRESHOLD_CM && distF > 2.0 && (millis() - lastTurnEndTime > TURN_COOLDOWN_MS)) {
      
      bool rightOpen = (distR > SIDE_GAP_THRESHOLD); 
      bool leftOpen = (distL > SIDE_GAP_THRESHOLD);

      // Lap 1 Direction Lock
      if (lockedTurnDirection == 0) {
        if (rightOpen && !leftOpen) {
          lockedTurnDirection = 1; 
          shouldTurn = true;
          Serial.println("LOCKED LAP DIRECTION: CLOCKWISE");
        } else if (leftOpen && !rightOpen) {
          lockedTurnDirection = -1; 
          shouldTurn = true;
          Serial.println("LOCKED LAP DIRECTION: ANTI-CLOCKWISE");
        } else if (distF <= 40.0) { 
          // Emergency Override increased to 40 to prevent getting too close to the wall on Lap 1
          lockedTurnDirection = (distR > distL) ? 1 : -1;
          shouldTurn = true;
          Serial.print("EMERGENCY BLIND LOCK: ");
          Serial.println(lockedTurnDirection == 1 ? "CW" : "CCW");
        }
      } 
      // Subsequent laps logic
      else {
        // Restored safety checks so the bot doesn't turn on false ToF readings
        if (lockedTurnDirection == 1 && (rightOpen || distF <= 40.0)) shouldTurn = true;
        if (lockedTurnDirection == -1 && (leftOpen || distF <= 40.0)) shouldTurn = true;
      }
    }

    if (shouldTurn) {
      isTurning = true;
      turnStartTime = millis();
      setNeoPixels(255, 165, 0); 
      
      if (lockedTurnDirection == 1) {
        turnDirection = 1;
        target_heading += 90.0;
        currentServoAngle = SERVO_SHIFT_RIGHT;
      } else {
        turnDirection = -1;
        target_heading -= 90.0;
        currentServoAngle = SERVO_SHIFT_LEFT;
      }
      
      if (target_heading >= 360.0) target_heading -= 360.0;
      if (target_heading < 0.0) target_heading += 360.0;
      
    } else {
      // Straight-Line Wall Centering
      float wallCentering = 0;
      float Kp_Wall = 1.2; 
      float targetWallDist = 42.0; 

      if (distL < 80.0 && distR < 80.0) {
        wallCentering = (distR - distL) * (Kp_Wall * 0.5);
      } else if (lockedTurnDirection == 1) {
        if (distR < 80.0) wallCentering -= (targetWallDist - distR) * Kp_Wall;
        else if (distL < 80.0) wallCentering += (targetWallDist - distL) * Kp_Wall;
      } else if (lockedTurnDirection == -1) {
        if (distL < 80.0) wallCentering += (targetWallDist - distL) * Kp_Wall;
        else if (distR < 80.0) wallCentering -= (targetWallDist - distR) * Kp_Wall;
      } else {
        if (distR < 80.0) wallCentering -= (targetWallDist - distR) * Kp_Wall;
        else if (distL < 80.0) wallCentering += (targetWallDist - distL) * Kp_Wall;
      }

      float correction = (headingError * 1.2) + wallCentering; 
      currentServoAngle = SERVO_CENTER_DEG + correction;
      currentServoAngle = constrain(currentServoAngle, SERVO_MIN_DEG, SERVO_MAX_DEG);
      
      steeringServo.write(currentServoAngle);
      driveMotor(BASE_SPEED, true);
    }
  } else {
    // ==========================================
    // AUTONOMOUS 3-POINT TURN CRASH RECOVERY
    // ==========================================
    
    // Hysteresis: Start reversing if too close, stop reversing only when safe
    if (distF < 12.0) {
      isReversing = true;
    } else if (distF > 20.0) {
      isReversing = false; 
    }

    if (isReversing) {
      // Wedged against the wall: INVERT steering and REVERSE hard
      int reverseAngle = (turnDirection == 1) ? SERVO_SHIFT_LEFT : SERVO_SHIFT_RIGHT;
      steeringServo.write(reverseAngle);
      currentServoAngle = reverseAngle;
      
      driveMotor(255, false); 
    } else {
      // Normal Forward Turn
      steeringServo.write(currentServoAngle);
      driveMotor(TURN_SPEED, true);
    }
    
    if (abs(headingError) < 8.0 || (millis() - turnStartTime > STOP_AFTER_TIME)) {
      isTurning = false;
      isReversing = false; 
      TURN++;
      lastTurnEndTime = millis(); 
      
      currentServoAngle = SERVO_CENTER_DEG;
      steeringServo.write(currentServoAngle);
      setNeoPixels(0, 255, 255); 
    }
  }

  // Update serial print to reflect the new state variable
  if (millis() - lastSerialTime > 250) {
    lastSerialTime = millis();
    Serial.print("T:"); Serial.print(TURN);
    Serial.print(" | F:"); Serial.print(distF, 0);
    Serial.print(" L:"); Serial.print(distL, 0);
    Serial.print(" R:"); Serial.print(distR, 0);
    Serial.print(" | H_Err:"); Serial.print(headingError, 1);
    Serial.print(" | Ang:"); Serial.print(currentServoAngle);
    
    if (isTurning) {
      if (isReversing) Serial.println(" | State: CRASH RECOVERY (REVERSE)");
      else {
        Serial.print(" | State: TURNING ");
        Serial.println(turnDirection == 1 ? "CW" : "CCW");
      }
    } else {
      Serial.println(" | State: STRAIGHT");
    }
  }
}
