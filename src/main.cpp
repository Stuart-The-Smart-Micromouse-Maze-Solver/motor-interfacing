#include <Arduino.h>

// ============================================================================
// Pins
// ============================================================================
// Motor 1 (Left)
const int AIN1 = 37;
const int AIN2 = 36;
const int ENC_A = 21;
const int ENC_B = 47;

// Motor 2 (Right)
const int BIN1 = 40;
const int BIN2 = 39;
const int ENC_C = 1;
const int ENC_D = 2;

const int STBY = 38;

// PWM Configuration
const int PWM_FREQ = 20000;
const int PWM_RESOLUTION = 8;
const int PWM_CH_LEFT = 0;
const int PWM_CH_RIGHT = 1;

// ============================================================================
// Robot Constants
// ============================================================================
const int ENCODER_PPR = 7;
const float GEAR_RATIO = 20.0;
const float WHEEL_DIAMETER_M = 0.032;
const float WHEEL_BASE_M = 0.103;
const float QUAD_FACTOR = 4.0;

const float COUNTS_PER_REV = GEAR_RATIO * ENCODER_PPR * QUAD_FACTOR; // 560
const float METERS_PER_COUNT = WHEEL_DIAMETER_M * PI / COUNTS_PER_REV;

// ============================================================================
// Encoder
// ============================================================================
struct Encoder
{
  volatile int64_t counts;
  int64_t prevCounts;
  uint32_t prevTime;
};

Encoder leftEncoder = {0, 0, 0};
Encoder rightEncoder = {0, 0, 0};

void IRAM_ATTR leftEncoderISR_A()
{
  int a = digitalRead(ENC_A);
  int b = digitalRead(ENC_B);
  leftEncoder.counts += (a == b) ? -1 : 1;
}

void IRAM_ATTR leftEncoderISR_B()
{
  int a = digitalRead(ENC_A);
  int b = digitalRead(ENC_B);
  leftEncoder.counts += (a != b) ? -1 : 1;
}

void IRAM_ATTR rightEncoderISR_C()
{
  int c = digitalRead(ENC_C);
  int d = digitalRead(ENC_D);
  rightEncoder.counts += (c == d) ? 1 : -1;
}

void IRAM_ATTR rightEncoderISR_D()
{
  int c = digitalRead(ENC_C);
  int d = digitalRead(ENC_D);
  rightEncoder.counts += (c != d) ? 1 : -1;
}

// ============================================================================
// Motor Control
// ============================================================================
struct Motor
{
  int pwmChannel;
  int pin1;
  int pin2;
  int command; // -255 to 255
  float rpm;
};

Motor leftMotor = {PWM_CH_LEFT, AIN1, AIN2, 0, 0};
Motor rightMotor = {PWM_CH_RIGHT, BIN1, BIN2, 0, 0};

void setMotorCommand(Motor *motor, int cmd)
{
  cmd = constrain(cmd, -255, 255);
  motor->command = cmd;

  if (cmd > 0)
  {
    digitalWrite(motor->pin2, LOW);
    digitalWrite(motor->pin1, HIGH); // Set direction first
    ledcWrite(motor->pwmChannel, cmd);
  }
  else if (cmd < 0)
  {
    digitalWrite(motor->pin1, LOW);
    digitalWrite(motor->pin2, HIGH); // Set direction first
    ledcWrite(motor->pwmChannel, -cmd);
  }
  else
  {
    ledcWrite(motor->pwmChannel, 0);
    digitalWrite(motor->pin1, LOW);
    digitalWrite(motor->pin2, LOW);
  }
}

void stopMotors()
{
  setMotorCommand(&leftMotor, 0);
  setMotorCommand(&rightMotor, 0);
}

// ============================================================================
// PID Controller
// ============================================================================
struct PID
{
  float kp;
  float ki;
  float kd;
  float integral;
  float prevError;
  float maxIntegral; // Anti-windup
  float output;
};

// PID gains - tune these values
const float SPEED_KP = 2.0;
const float SPEED_KI = 5.0;
const float SPEED_KD = 0.05;
const float MAX_INTEGRAL = 100.0;

PID leftPID = {SPEED_KP, SPEED_KI, SPEED_KD, 0, 0, MAX_INTEGRAL, 0};
PID rightPID = {SPEED_KP, SPEED_KI, SPEED_KD, 0, 0, MAX_INTEGRAL, 0};

void resetPID(PID *pid)
{
  pid->integral = 0;
  pid->prevError = 0;
  pid->output = 0;
}

float computePID(PID *pid, float error, float dt)
{
  if (dt <= 0)
    dt = 0.001;

  // Proportional
  float p = pid->kp * error;

  // Integral with anti-windup
  pid->integral += error * dt;
  pid->integral = constrain(pid->integral, -pid->maxIntegral, pid->maxIntegral);
  float i = pid->ki * pid->integral;

  // Derivative
  float d = pid->kd * (error - pid->prevError) / dt;
  pid->prevError = error;

  pid->output = p + i + d;
  return pid->output;
}

// ============================================================================
// Speed Measurement
// ============================================================================
float countsToRPM(int64_t counts, float dt)
{
  if (dt <= 0)
    return 0;
  float revolutions = counts / COUNTS_PER_REV;
  return (revolutions / dt) * 60.0;
}

void updateMotorSpeeds(float dt)
{
  int64_t leftDelta = leftEncoder.counts - leftEncoder.prevCounts;
  int64_t rightDelta = rightEncoder.counts - rightEncoder.prevCounts;

  leftMotor.rpm = countsToRPM(leftDelta, dt);
  rightMotor.rpm = countsToRPM(rightDelta, dt);

  leftEncoder.prevCounts = leftEncoder.counts;
  rightEncoder.prevCounts = rightEncoder.counts;
}

// ============================================================================
// Odometry
// ============================================================================
struct Pose
{
  float x;
  float y;
  float theta;
};

Pose robotPose = {0, 0, 0};

void updateOdometry(float dt)
{
  float leftDist = (leftEncoder.counts - leftEncoder.prevCounts) * METERS_PER_COUNT;
  float rightDist = (rightEncoder.counts - rightEncoder.prevCounts) * METERS_PER_COUNT;

  float dist = (leftDist + rightDist) / 2.0;
  float dTheta = (rightDist - leftDist) / WHEEL_BASE_M;

  robotPose.theta += dTheta;

  // Normalize to [-PI, PI]
  while (robotPose.theta > PI)
    robotPose.theta -= 2 * PI;
  while (robotPose.theta < -PI)
    robotPose.theta += 2 * PI;

  robotPose.x += dist * cos(robotPose.theta);
  robotPose.y += dist * sin(robotPose.theta);
}

// ============================================================================
// Speed Control (PID-based)
// ============================================================================
void setMotorSpeedRPM(Motor *motor, Encoder *encoder, PID *pid, float targetRPM)
{
  // Calculate error: target - current (positive error = need to speed up)
  float error = targetRPM - motor->rpm;

  // Get time delta
  uint32_t now = micros();
  float dt = (now - encoder->prevTime) / 1000000.0;
  encoder->prevTime = now;

  if (dt > 0.1)
    dt = 0.01; // Cap dt to prevent huge jumps

  // Compute PID output
  float correction = computePID(pid, error, dt);

  // Apply correction to current command
  int newCmd = motor->command + (int)correction;
  newCmd = constrain(newCmd, -255, 255);

  // Set motor command
  setMotorCommand(motor, newCmd);
}

// ============================================================================
// High-Level Movement Functions
// ============================================================================
void moveForwardCm(float distanceCm, float speedRPM)
{
  // Store initial encoder counts
  int64_t startLeft = leftEncoder.counts;
  int64_t startRight = rightEncoder.counts;

  // Reset PIDs
  resetPID(&leftPID);
  resetPID(&rightPID);

  // Initialize timing
  leftEncoder.prevTime = micros();
  rightEncoder.prevTime = micros();
  leftEncoder.prevCounts = leftEncoder.counts;
  rightEncoder.prevCounts = rightEncoder.counts;
  uint32_t lastUpdate = millis();
  uint32_t lastPrint = millis();
  uint32_t startTime = millis();
  
  // Timeout: 10 seconds max
  const uint32_t TIMEOUT_MS = 10000;
  
  // Drift correction gain - how aggressively to correct drift
  const float DRIFT_GAIN = 30.0;  // RPM adjustment per cm of drift

  // Set initial motor command to get things moving
  int initialCmd = map(speedRPM, 0, 300, 0, 255);
  initialCmd = constrain(initialCmd, 50, 200);
  setMotorCommand(&leftMotor, initialCmd);
  setMotorCommand(&rightMotor, initialCmd);

  Serial.printf("Moving %.1f cm at %.0f RPM (initial cmd: %d)\n", distanceCm, speedRPM, initialCmd);

  while (true)
  {
    // Timeout check
    if (millis() - startTime > TIMEOUT_MS)
    {
      Serial.println("ERROR: Movement timeout!");
      break;
    }
    
    // Calculate distance traveled by each wheel
    float leftDistCm = (leftEncoder.counts - startLeft) * METERS_PER_COUNT * 100.0;
    float rightDistCm = (rightEncoder.counts - startRight) * METERS_PER_COUNT * 100.0;
    
    // Use minimum distance to ensure BOTH wheels reach target
    float minDistCm = min(leftDistCm, rightDistCm);

    // Stop when the slower wheel reaches target
    if (minDistCm >= distanceCm)
    {
      break;
    }
    
    // Update every 10ms
    if (millis() - lastUpdate >= 10)
    {
      float dt = (millis() - lastUpdate) / 1000.0;
      lastUpdate = millis();

      // Update speed measurements
      updateMotorSpeeds(dt);

      // Calculate drift: positive = left is ahead, negative = right is ahead
      float driftCm = leftDistCm - rightDistCm;
      
      // Calculate RPM correction based on drift
      // If left is ahead (positive drift), slow left down / speed right up
      float driftCorrection = driftCm * DRIFT_GAIN;
      
      // Slow down as we approach target
      float remaining = distanceCm - minDistCm;
      float baseRPM = speedRPM;
      if (remaining < 3.0) baseRPM = speedRPM * 0.6;
      if (remaining < 1.0) baseRPM = speedRPM * 0.3;
      
      // Apply drift correction to target RPMs
      float leftTargetRPM = baseRPM - driftCorrection;
      float rightTargetRPM = baseRPM + driftCorrection;
      
      // Clamp to reasonable range
      leftTargetRPM = constrain(leftTargetRPM, 10.0, speedRPM * 1.5);
      rightTargetRPM = constrain(rightTargetRPM, 10.0, speedRPM * 1.5);
      
      // Apply PID speed control
      setMotorSpeedRPM(&leftMotor, &leftEncoder, &leftPID, leftTargetRPM);
      setMotorSpeedRPM(&rightMotor, &rightEncoder, &rightPID, rightTargetRPM);
    }

    // Print status every 200ms
    if (millis() - lastPrint >= 200)
    {
      float driftCm = leftDistCm - rightDistCm;
      Serial.printf("L: %.1fcm R: %.1fcm | drift: %+.1fcm | cmd L=%d R=%d\n",
                    leftDistCm, rightDistCm, driftCm,
                    leftMotor.command, rightMotor.command);
      lastPrint = millis();
    }

    delay(1);
  }

  stopMotors();

  // Print final result
  float finalLeft = (leftEncoder.counts - startLeft) * METERS_PER_COUNT * 100;
  float finalRight = (rightEncoder.counts - startRight) * METERS_PER_COUNT * 100;
  float finalDrift = finalLeft - finalRight;
  Serial.printf("Done! L: %.1fcm, R: %.1fcm (final drift: %+.1fcm)\n", 
                finalLeft, finalRight, finalDrift);
}

void turnDegrees(float degrees, float speedRPM)
{
  // Calculate arc length each wheel needs to travel
  float arcLength = (degrees / 360.0) * PI * WHEEL_BASE_M;

  int64_t startLeft = leftEncoder.counts;
  int64_t startRight = rightEncoder.counts;

  resetPID(&leftPID);
  resetPID(&rightPID);

  leftEncoder.prevTime = micros();
  rightEncoder.prevTime = micros();
  uint32_t lastUpdate = millis();

  int initialCmd = map(speedRPM, 0, 300, 0, 200);
  initialCmd = constrain(initialCmd, 40, 150);

  // For turning: one wheel forward, one backward
  int leftCmd = (degrees > 0) ? -initialCmd : initialCmd;
  int rightCmd = (degrees > 0) ? initialCmd : -initialCmd;

  setMotorCommand(&leftMotor, leftCmd);
  setMotorCommand(&rightMotor, rightCmd);

  Serial.printf("Turning %.1f degrees at %.0f RPM\n", degrees, speedRPM);

  while (true)
  {
    float leftDistM = abs((leftEncoder.counts - startLeft) * METERS_PER_COUNT);
    float rightDistM = abs((rightEncoder.counts - startRight) * METERS_PER_COUNT);
    float avgDistM = (leftDistM + rightDistM) / 2.0;

    if (avgDistM >= abs(arcLength))
    {
      break;
    }

    if (millis() - lastUpdate >= 10)
    {
      float dt = (millis() - lastUpdate) / 1000.0;
      lastUpdate = millis();
      updateMotorSpeeds(dt);
    }

    delay(1);
  }

  stopMotors();
  Serial.printf("Turn complete!\n");
}

// ============================================================================
// Test Functions
// ============================================================================
void testMotors()
{
  Serial.println("\n=== Motor Test ===");
  Serial.println("Watch encoder counts - should go UP for forward, DOWN for backward\n");

  int64_t startCount;

  // Left motor forward
  Serial.println("Left motor FORWARD (expect count to increase)...");
  startCount = leftEncoder.counts;
  setMotorCommand(&leftMotor, 150);
  for (int i = 0; i < 10; i++)
  {
    delay(100);
    Serial.printf("  Left encoder: %lld (delta: %+lld)\n", leftEncoder.counts, leftEncoder.counts - startCount);
  }
  stopMotors();
  Serial.printf("  Final delta: %+lld %s\n\n", leftEncoder.counts - startCount,
                (leftEncoder.counts > startCount) ? "OK" : "WRONG DIRECTION!");
  delay(500);

  // Left motor backward
  Serial.println("Left motor BACKWARD (expect count to decrease)...");
  startCount = leftEncoder.counts;
  setMotorCommand(&leftMotor, -150);
  for (int i = 0; i < 10; i++)
  {
    delay(100);
    Serial.printf("  Left encoder: %lld (delta: %+lld)\n", leftEncoder.counts, leftEncoder.counts - startCount);
  }
  stopMotors();
  Serial.printf("  Final delta: %+lld %s\n\n", leftEncoder.counts - startCount,
                (leftEncoder.counts < startCount) ? "OK" : "WRONG DIRECTION!");
  delay(500);

  // Right motor forward
  Serial.println("Right motor FORWARD (expect count to increase)...");
  startCount = rightEncoder.counts;
  setMotorCommand(&rightMotor, 150);
  for (int i = 0; i < 10; i++)
  {
    delay(100);
    Serial.printf("  Right encoder: %lld (delta: %+lld)\n", rightEncoder.counts, rightEncoder.counts - startCount);
  }
  stopMotors();
  Serial.printf("  Final delta: %+lld %s\n\n", rightEncoder.counts - startCount,
                (rightEncoder.counts > startCount) ? "OK" : "WRONG DIRECTION!");
  delay(500);

  // Right motor backward
  Serial.println("Right motor BACKWARD (expect count to decrease)...");
  startCount = rightEncoder.counts;
  setMotorCommand(&rightMotor, -150);
  for (int i = 0; i < 10; i++)
  {
    delay(100);
    Serial.printf("  Right encoder: %lld (delta: %+lld)\n", rightEncoder.counts, rightEncoder.counts - startCount);
  }
  stopMotors();
  Serial.printf("  Final delta: %+lld %s\n\n", rightEncoder.counts - startCount,
                (rightEncoder.counts < startCount) ? "OK" : "WRONG DIRECTION!");

  Serial.println("Motor test complete!");
}

void testEncoders()
{
  Serial.println("\n=== Encoder Test ===");
  Serial.println("Spin wheels manually and watch counts...");
  Serial.println("start test:");
  leftEncoder.counts = 0;
  rightEncoder.counts = 0;

  for (int i = 0; i < 500; i++)
  {
    Serial.printf("Left: %lld, Right: %lld\n", leftEncoder.counts, rightEncoder.counts);
    delay(200);
  }
}

void testPID()
{
  Serial.println("\n=== PID Speed Control Test ===");

  resetPID(&leftPID);
  resetPID(&rightPID);

  leftEncoder.prevCounts = leftEncoder.counts;
  rightEncoder.prevCounts = rightEncoder.counts;
  leftEncoder.prevTime = micros();
  rightEncoder.prevTime = micros();

  float targetRPM = 100.0;

  // Initial command
  setMotorCommand(&leftMotor, 100);
  setMotorCommand(&rightMotor, 100);

  uint32_t startTime = millis();
  uint32_t lastUpdate = millis();
  uint32_t lastPrint = millis();

  Serial.printf("Target RPM: %.0f\n", targetRPM);

  while (millis() - startTime < 5000)
  {
    if (millis() - lastUpdate >= 10)
    {
      float dt = (millis() - lastUpdate) / 1000.0;
      lastUpdate = millis();

      updateMotorSpeeds(dt);

      setMotorSpeedRPM(&leftMotor, &leftEncoder, &leftPID, targetRPM);
      setMotorSpeedRPM(&rightMotor, &rightEncoder, &rightPID, targetRPM);
    }

    if (millis() - lastPrint >= 100)
    {
      Serial.printf("L: cmd=%3d rpm=%6.1f err=%.1f | R: cmd=%3d rpm=%6.1f err=%.1f\n",
                    leftMotor.command, leftMotor.rpm, targetRPM - leftMotor.rpm,
                    rightMotor.command, rightMotor.rpm, targetRPM - rightMotor.rpm);
      lastPrint = millis();
    }

    delay(1);
  }

  stopMotors();
  Serial.println("PID test complete!");
}

// Demo V1.0 
#define AUTO_DEMO 1

void autoDemoLoop(){
  static bool inRest = false; 
  static uint32_t restStart = 0;

  if (!inRest){
    moveForwardCm(100.0, 250.0); // Move 100cm at 250 RPM
    stopMotors();
    inRest = true;
    restStart = millis();
  } else {
    if (millis() - restStart >= 10000){
      inRest = false;
    }
  }
}

// ============================================================================
// Setup
// ============================================================================
void setup()
{
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n=== Micromouse Motor Control ===");

  // Motor pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);

  // Standby pin
    pinMode(STBY, OUTPUT);
    digitalWrite(STBY, HIGH);

  // PWM setup
  ledcSetup(PWM_CH_LEFT, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CH_RIGHT, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(AIN1, PWM_CH_LEFT);
  ledcAttachPin(BIN1, PWM_CH_RIGHT);

  // Encoder pins
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC_C, INPUT_PULLUP);
  pinMode(ENC_D, INPUT_PULLUP);

  // Encoder interrupts
  attachInterrupt(digitalPinToInterrupt(ENC_A), leftEncoderISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), leftEncoderISR_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_C), rightEncoderISR_C, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_D), rightEncoderISR_D, CHANGE);

  // Initialize timing
  leftEncoder.prevTime = micros();
  rightEncoder.prevTime = micros();

  Serial.println("Initialization complete!");
  Serial.printf("Counts per rev: %.0f\n", COUNTS_PER_REV);
  Serial.printf("Meters per count: %.6f\n", METERS_PER_COUNT);
  Serial.printf("PID: Kp=%.2f Ki=%.2f Kd=%.3f\n", SPEED_KP, SPEED_KI, SPEED_KD);
  Serial.println("\nCommands: 'm' = motor test, 'e' = encoder test, 'p' = PID test, 'g' = go 10cm");
}

// ============================================================================
// Main Loop
// ============================================================================
void loop()
{
  #if AUTO_DEMO
    autoDemoLoop();
  #endif
  
  // Check for serial commands
  if (Serial.available())
  {
    char cmd = Serial.read();

    switch (cmd)
    {
    case 'm':
      testMotors();
      break;
    case 'e':
      testEncoders();
      break;
    case 'p':
      testPID();
      break;
    case 'g':
      moveForwardCm(25.0, 1000.0); // Move 25cm at 1000 RPM
      break;
    case 't':
      turnDegrees(90.0, 60.0); // Turn 90 degrees
      break;
    case 's':
      stopMotors();
      Serial.println("Motors stopped");
      break;
    }
  }

  // Update odometry in background
  static uint32_t lastOdom = millis();
  if (millis() - lastOdom >= 50)
  {
    float dt = (millis() - lastOdom) / 1000.0;
    updateOdometry(dt);
    lastOdom = millis();
  }

  delay(1);
}
