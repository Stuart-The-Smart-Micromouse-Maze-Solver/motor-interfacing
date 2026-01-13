#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_FXOS8700.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_NeoPixel.h>

// ============================================================================
// Pins
// ============================================================================
// I2C for IMU
const int SDA_PIN = 17;
const int SCL_PIN = 16;

// NeoPixel LED
const int NEOPIXEL_PIN = 38;
const int NUM_PIXELS = 1;
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

// ============================================================================
// NeoPixel LED
// ============================================================================
Adafruit_NeoPixel pixel(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// LED State Colors
const uint32_t COLOR_STARTUP = pixel.Color(255, 255, 0); // Yellow - Starting up
const uint32_t COLOR_REST = pixel.Color(0, 255, 0);      // Green - Resting/Idle
const uint32_t COLOR_TURNING = pixel.Color(255, 0, 255); // Magenta - Turning
const uint32_t COLOR_MOVING = pixel.Color(0, 0, 255);    // Blue - Moving forward
const uint32_t COLOR_ERROR = pixel.Color(255, 0, 0);     // Red - Error

void setLEDColor(uint32_t color)
{
  pixel.setPixelColor(0, color);
  pixel.show();
}

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
const float QUAD_FACTOR = 1.0;

const float COUNTS_PER_REV = GEAR_RATIO * ENCODER_PPR * QUAD_FACTOR; // 560
const float METERS_PER_COUNT = WHEEL_DIAMETER_M * PI / COUNTS_PER_REV;

// ============================================================================
// IMU (FXOS8700 + FXAS21002C)
// ============================================================================
Adafruit_FXOS8700 accelMag = Adafruit_FXOS8700(0x8700A, 0x8700B);
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);

struct IMUData
{
  // Acceleration (m/s^2)
  float accelX, accelY, accelZ;

  // Angular velocity (rad/s)
  float gyroX, gyroY, gyroZ;

  // Integrated heading from gyro Z-axis (degrees, 0-360)
  float heading;

  bool accelValid;
  bool gyroValid;
};

IMUData imuData = {0, 0, 0, 0, 0, 0, 0, false, false};

// Gyro bias calibration (offset when stationary)
float gyroBiasX = 0.0;
float gyroBiasY = 0.0;
float gyroBiasZ = 0.0;

// Timing for gyro integration
uint32_t lastIMUUpdate = 0;

void initIMU()
{
  Serial.println("Initializing IMU (FXOS8700 + FXAS21002C)...");

  // Initialize I2C
  Wire.begin(SDA_PIN, SCL_PIN);

  // Initialize accelerometer
  if (!accelMag.begin())
  {
    Serial.println("ERROR: Could not find FXOS8700 sensor!");
    Serial.println("Check wiring: SDA=17, SCL=16");
    imuData.accelValid = false;
  }
  else
  {
    Serial.println("✓ FXOS8700 (Accelerometer) initialized");
    imuData.accelValid = true;
  }

  // Initialize gyroscope
  if (!gyro.begin())
  {
    Serial.println("ERROR: Could not find FXAS21002C gyroscope!");
    Serial.println("Check wiring: SDA=17, SCL=16");
    imuData.gyroValid = false;
  }
  else
  {
    Serial.println("✓ FXAS21002C (Gyroscope) initialized");
    imuData.gyroValid = true;

    // Set gyro range (250, 500, 1000, or 2000 dps)
    gyro.setRange(GYRO_RANGE_250DPS);
  }

  lastIMUUpdate = micros();

  if (imuData.accelValid || imuData.gyroValid)
  {
    Serial.println("IMU initialization complete!");
  }
  else
  {
    Serial.println("WARNING: No IMU sensors detected!");
  }
}

void updateIMU()
{
  if (!imuData.accelValid && !imuData.gyroValid)
    return;

  // Calculate time delta for gyro integration
  uint32_t now = micros();
  float dt = (now - lastIMUUpdate) / 1000000.0;
  lastIMUUpdate = now;

  // Read accelerometer
  if (imuData.accelValid)
  {
    sensors_event_t accel, mag;
    if (accelMag.getEvent(&accel, &mag))
    {
      imuData.accelX = accel.acceleration.x;
      imuData.accelY = accel.acceleration.y;
      imuData.accelZ = accel.acceleration.z;
    }
  }

  // Read gyroscope
  if (imuData.gyroValid)
  {
    sensors_event_t gyroEvent;
    if (gyro.getEvent(&gyroEvent))
    {
      // Store angular velocity (rad/s) with bias correction
      imuData.gyroX = gyroEvent.gyro.x - gyroBiasX;
      imuData.gyroY = gyroEvent.gyro.y - gyroBiasY;
      imuData.gyroZ = gyroEvent.gyro.z - gyroBiasZ;

      // Integrate gyro Z-axis for heading
      imuData.heading += (imuData.gyroZ * 180.0 / PI) * dt;

      // Normalize to 0-360
      while (imuData.heading >= 360.0)
        imuData.heading -= 360.0;
      while (imuData.heading < 0.0)
        imuData.heading += 360.0;
    }
  }
}

void printIMUData()
{
  if (!imuData.accelValid && !imuData.gyroValid)
  {
    Serial.println("IMU: No sensors available");
    return;
  }

  Serial.println("\n=== IMU Data ===");

  if (imuData.gyroValid)
  {
    Serial.printf("Heading: %6.1f° (integrated from gyro)\n", imuData.heading);
    Serial.printf("Rotation Rate (°/s): X:%+7.2f Y:%+7.2f Z:%+7.2f\n",
                  imuData.gyroX * 180.0 / PI,
                  imuData.gyroY * 180.0 / PI,
                  imuData.gyroZ * 180.0 / PI);
  }

  if (imuData.accelValid)
  {
    Serial.printf("Acceleration (m/s²): X:%+6.2f Y:%+6.2f Z:%+6.2f\n",
                  imuData.accelX, imuData.accelY, imuData.accelZ);

    // Calculate magnitude
    float accelMag = sqrt(imuData.accelX * imuData.accelX +
                          imuData.accelY * imuData.accelY +
                          imuData.accelZ * imuData.accelZ);
    Serial.printf("Acceleration Magnitude: %.2f m/s²\n", accelMag);
  }

  Serial.println();
}

void calibrateGyro()
{
  if (!imuData.gyroValid)
  {
    Serial.println("ERROR: Gyroscope not available");
    return;
  }

  Serial.println("\n=== Gyroscope Calibration ===");
  Serial.println("Keep the robot STATIONARY for 5 seconds...");

  float sumX = 0, sumY = 0, sumZ = 0;
  int samples = 0;

  uint32_t startTime = millis();

  while (millis() - startTime < 5000)
  {
    sensors_event_t gyroEvent;
    if (gyro.getEvent(&gyroEvent))
    {
      sumX += gyroEvent.gyro.x;
      sumY += gyroEvent.gyro.y;
      sumZ += gyroEvent.gyro.z;
      samples++;
    }
    delay(10);
  }

  if (samples > 0)
  {
    gyroBiasX = sumX / samples;
    gyroBiasY = sumY / samples;
    gyroBiasZ = sumZ / samples;

    Serial.println("Gyro calibration complete!");
    Serial.printf("Bias - X: %.4f, Y: %.4f, Z: %.4f rad/s\n",
                  gyroBiasX, gyroBiasY, gyroBiasZ);
    Serial.println("Add these values to your code for persistent calibration.");

    // Reset gyro heading
    imuData.heading = 0;
  }
  else
  {
    Serial.println("ERROR: No gyro data collected");
  }
}

void resetIMUHeading()
{
  imuData.heading = 0;
  Serial.println("IMU heading reset to 0°");
}

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
  const float DRIFT_GAIN = 30.0; // RPM adjustment per cm of drift

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
      if (remaining < 3.0)
        baseRPM = speedRPM * 0.6;
      if (remaining < 1.0)
        baseRPM = speedRPM * 0.3;

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
  if (!imuData.gyroValid)
  {
    Serial.println("ERROR: Gyro not available, cannot turn accurately!");
    return;
  }

  // Update IMU a few times to get stable reading
  for (int i = 0; i < 5; i++)
  {
    updateIMU();
    delay(10);
  }

  float targetAngle = abs(degrees);
  bool turningRight = degrees > 0;
  float startHeading = imuData.heading;
  float lastHeading = startHeading;
  float totalTurned = 0.0;

  // Set motor command based on turn direction
  int initialCmd = map(speedRPM, 0, 300, 0, 255);
  initialCmd = constrain(initialCmd, 100, 200); // Maximum power for fast turning

  // For turning: one wheel forward, one backward
  // Positive degrees = turn right (CW): left forward, right backward
  // Negative degrees = turn left (CCW): left backward, right forward
  int leftCmd = turningRight ? initialCmd : -initialCmd;
  int rightCmd = turningRight ? -initialCmd : initialCmd;

  setMotorCommand(&leftMotor, leftCmd);
  setMotorCommand(&rightMotor, rightCmd);

  Serial.printf("Turning %.1f° %s (gyro-based) | Start: %.1f°\n",
                targetAngle, turningRight ? "RIGHT" : "LEFT", startHeading);

  uint32_t startTime = millis();
  uint32_t lastPrint = millis();
  const uint32_t TIMEOUT_MS = 10000; // 10 second timeout
  const float ANGLE_TOLERANCE = 3.0; // Stop when within 3 degrees

  while (true)
  {
    // Update IMU to get current heading
    updateIMU();
    float currentHeading = imuData.heading;

    // Calculate angular change since last reading
    float delta = currentHeading - lastHeading;

    // Handle wraparound (0 <-> 360 transition)
    if (delta > 180.0)
    {
      delta -= 360.0; // Crossed 0 going backward (CCW)
    }
    else if (delta < -180.0)
    {
      delta += 360.0; // Crossed 0 going forward (CW)
    }

    // Accumulate total rotation (always positive)
    totalTurned += abs(delta);
    lastHeading = currentHeading;

    // Check if we've reached target
    if (totalTurned >= targetAngle - ANGLE_TOLERANCE)
    {
      break;
    }

    // Timeout check
    if (millis() - startTime > TIMEOUT_MS)
    {
      Serial.println("ERROR: Turn timeout!");
      break;
    }

    // Slow down as we approach target
    float remaining = targetAngle - totalTurned;
    if (remaining < 10.0 && remaining > 0)
    {
      int slowCmd = initialCmd * 0.85; // Only reduce to 85% (minimal slowdown)
      leftCmd = turningRight ? slowCmd : -slowCmd;
      rightCmd = turningRight ? -slowCmd : slowCmd;
      setMotorCommand(&leftMotor, leftCmd);
      setMotorCommand(&rightMotor, rightCmd);
    }

    // Print status every 150ms
    if (millis() - lastPrint >= 150)
    {
      Serial.printf("  Turned: %.1f° | Heading: %.1f° | Remaining: %.1f°\n",
                    totalTurned, currentHeading, targetAngle - totalTurned);
      lastPrint = millis();
    }

    delay(10); // Small delay for loop timing
  }

  stopMotors();
  delay(100);  // Let it settle
  updateIMU(); // Final update

  float finalHeading = imuData.heading;

  Serial.printf("Turn complete! Target: %.1f° | Total Turned: %.1f° | Final Heading: %.1f° | Error: %.1f°\n",
                targetAngle, totalTurned, finalHeading, abs(targetAngle - totalTurned));
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

// Demo V1.0 - Movement Demo
#define AUTO_DEMO 0

void autoDemoLoop()
{
  static const float seq_cm[] = {18.0f, 36.0f, 54.0f, 72.0f};
  static const size_t N = sizeof(seq_cm) / sizeof(seq_cm[0]);
  static size_t idx = 0;

  static bool inRest = false;
  static uint32_t restStart = 0;

  const float SPEED_RPM_HINT = 200.0f;
  const uint32_t REST_MS = 10000UL;

  if (!inRest)
  {
    moveForwardCm(seq_cm[idx], SPEED_RPM_HINT);
    stopMotors();

    inRest = true;
    restStart = millis();

    idx = (idx + 1) % N;
  }
  else
  {
    if (millis() - restStart >= REST_MS)
    {
      inRest = false;
    }
  }
}

// Demo V2.0 - Turning Demo
#define TURN_DEMO 1

void turnDemoLoop()
{
  static const float seq_degrees[] = {90.0f, 180.0f, 270.0f, 360.0f};
  static const size_t N = sizeof(seq_degrees) / sizeof(seq_degrees[0]);
  static size_t idx = 0;

  static bool inRest = false;
  static uint32_t restStart = 0;

  const float TURN_SPEED_RPM = 200.0f; // Same speed as movement demo
  const uint32_t REST_MS = 7500UL;     // 10 seconds rest between turns (same as movement demo)

  if (!inRest)
  {
    // Set LED to magenta (turning)
    setLEDColor(COLOR_TURNING);

    Serial.printf("\n>>> Demo Turn %d/%d: %.0f degrees <<<\n",
                  idx + 1, N, seq_degrees[idx]);

    turnDegrees(seq_degrees[idx], TURN_SPEED_RPM);
    stopMotors();

    // Set LED to green (rest)
    setLEDColor(COLOR_REST);

    inRest = true;
    restStart = millis();

    idx = (idx + 1) % N;
  }
  else
  {
    if (millis() - restStart >= REST_MS)
    {
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

  // Initialize NeoPixel
  pixel.begin();
  pixel.setBrightness(50);    // Set brightness to 50/255 (not too bright)
  setLEDColor(COLOR_STARTUP); // Yellow during startup

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

  // Initialize IMU
  initIMU();

  Serial.println("Initialization complete!");

  // Set LED to green (rest/idle state)
  setLEDColor(COLOR_REST);
  Serial.printf("Counts per rev: %.0f\n", COUNTS_PER_REV);
  Serial.printf("Meters per count: %.6f\n", METERS_PER_COUNT);
  Serial.printf("PID: Kp=%.2f Ki=%.2f Kd=%.3f\n", SPEED_KP, SPEED_KI, SPEED_KD);
  Serial.println("\nCommands:");
  Serial.println("  'm' = motor test, 'e' = encoder test, 'p' = PID test");
  Serial.println("  'g' = go 25cm, 't' = turn 90°, 's' = stop");
  Serial.println("  'i' = print IMU data, 'y' = calibrate gyro, 'r' = reset heading");
}

// ============================================================================
// Main Loop
// ============================================================================
void loop()
{
#if AUTO_DEMO
  autoDemoLoop();
#endif

#if TURN_DEMO
  turnDemoLoop();
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
    case 'i':
      updateIMU();
      printIMUData();
      break;
    case 'y':
      calibrateGyro();
      break;
    case 'r':
      resetIMUHeading();
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

  // Update IMU in background
  static uint32_t lastIMU = millis();
  if (millis() - lastIMU >= 100) // Update at 10Hz
  {
    updateIMU();
    lastIMU = millis();
  }

  delay(1);
}
