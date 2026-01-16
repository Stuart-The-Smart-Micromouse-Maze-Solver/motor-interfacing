#include "config.h"
#include "motion.h"
#include "motors.h"
#include "encoders.h"
#include "gyro_heading.h"
#include "control.h"

//PID constants for motion control
static PID leftPID  = {1.00f, 0.00f, 0.0015f, 0, 0, 40.0f, 0};
static PID rightPID = {1.00f, 0.00f, 0.0015f, 0, 0, 40.0f, 0};

// PID constants for turning control
static PID leftTurnPID  = {0.80f, 0.2f, 0.000f, 0, 0, 20.0f, 0};
static PID rightTurnPID = {0.80f, 0.2f, 0.000f, 0, 0, 20.0f, 0};

void moveForwardCmClean(float distanceCm, float speedRPM)
{
  const uint32_t TIMEOUT_MS = 8000;
  const uint32_t LOOP_MS = 10;

  const float RAMP_UP_CM = 4.0f;
  const float RAMP_DOWN_CM = 10.0f;
  const float MIN_RPM = 60.0f;
  const float STOP_TOL_CM = 0.5f;

  // smoother steering
  const float HEADING_KP = 1.2f; //lower = reduces how hard it reacts
  const float HEADING_DEADBAND_DEG = 1.0f; // smaller = more sensitive (0.4-1.0)
  const float HEADING_CORR_CLAMP = 30.0f; // steering authority -> too high = sharp corrections (25-60)
  const float CORR_ALPHA = 0.05f; // big = faster response, but more jitter

  const float DRIFT_GAIN_RPM_PER_CM = 18.0f;

  int64_t startLeft  = readEncoderCounts(leftEncoder);
  int64_t startRight = readEncoderCounts(rightEncoder);

  resetPID(&leftPID);
  resetPID(&rightPID);

  leftEncoder.prevTime  = micros();
  rightEncoder.prevTime = micros();
  leftEncoder.prevCounts  = startLeft;
  rightEncoder.prevCounts = startRight;

  uint32_t startTime  = millis();
  uint32_t lastUpdate = millis();

  // stabilize heading
  for (int i = 0; i < 5; i++) { gyroUpdate(); delay(5); }
  float targetHeading = gyroHeadingDeg();

  float corrRPM_f = 0.0f;

  // kick
  setMotorCommand(&leftMotor, 140);
  setMotorCommand(&rightMotor, 140);
  delay(70);

  while (true)
  {
    if (millis() - startTime > TIMEOUT_MS) break;

    if (millis() - lastUpdate < LOOP_MS) { delay(1); continue; }
    float dt = (millis() - lastUpdate) / 1000.0f;
    lastUpdate = millis();

    gyroUpdate();

    int64_t lc = readEncoderCounts(leftEncoder);
    int64_t rc = readEncoderCounts(rightEncoder);

    float leftDistCm  = (lc - startLeft)  * metersPerCountCal() * 100.0f;
    float rightDistCm = (rc - startRight) * metersPerCountCal() * 100.0f;

    float avgDistCm = 0.5f * (leftDistCm + rightDistCm);
    float remaining = distanceCm - avgDistCm;

    if (remaining <= STOP_TOL_CM) break;

    float baseRPM = speedRPM;

    // baseRPM -> how fast we want to go before corrections

    if (avgDistCm < RAMP_UP_CM) {
      float s = avgDistCm / max(0.001f, RAMP_UP_CM);
      baseRPM = MIN_RPM + s * (speedRPM - MIN_RPM);
    }
    if (remaining < RAMP_DOWN_CM) {
      float s = remaining / max(0.001f, RAMP_DOWN_CM);
      float downRPM = MIN_RPM + s * (speedRPM - MIN_RPM);
      baseRPM = min(baseRPM, downRPM);
    }
    baseRPM = constrain(baseRPM, MIN_RPM, speedRPM);

    float corrRaw = 0.0f;

    if (gyroIsValid()) {
      float hErr = angleDiffDeg(targetHeading, gyroHeadingDeg());
      if (fabs(hErr) < HEADING_DEADBAND_DEG) hErr = 0.0f;

      corrRaw = HEADING_KP * hErr;
      corrRaw = constrain(corrRaw, -HEADING_CORR_CLAMP, HEADING_CORR_CLAMP);
    } else {
      float driftCm = leftDistCm - rightDistCm;
      corrRaw = constrain(driftCm * DRIFT_GAIN_RPM_PER_CM,
                          -HEADING_CORR_CLAMP, HEADING_CORR_CLAMP);
    }

    corrRPM_f += CORR_ALPHA * (corrRaw - corrRPM_f);

    float leftTargetRPM  = baseRPM - corrRPM_f;
    float rightTargetRPM = baseRPM + corrRPM_f;

    leftTargetRPM  = constrain(leftTargetRPM,  0.0f, speedRPM * 1.4f);
    rightTargetRPM = constrain(rightTargetRPM, 0.0f, speedRPM * 1.4f);

    updateMotorSpeeds(dt);
    setMotorSpeedRPM(&leftMotor, &leftEncoder, &leftPID, leftTargetRPM);
    setMotorSpeedRPM(&rightMotor, &rightEncoder, &rightPID, rightTargetRPM);
  }

  brakeStop(140);
  delay(120);
}

void moveBackwardCmClean(float distanceCm, float speedRPM)
{
  const uint32_t TIMEOUT_MS = 6000;
  const uint32_t LOOP_MS    = 10;

  const float RAMP_UP_CM   = 3.0f;
  const float RAMP_DOWN_CM = 4.0f;
  const float MIN_RPM      = 60.0f;   
  const float STOP_TOL_CM  = 0.25f;   

  const float HEADING_KP            = 1.2f;
  const float HEADING_DEADBAND_DEG  = 1.0f;
  const float HEADING_CORR_CLAMP    = 30.0f;
  const float CORR_ALPHA            = 0.05f;

  // Reset speed loops
  resetPID(&leftPID);
  resetPID(&rightPID);

  // Stabilize gyro and lock current heading as target
  for (int i = 0; i < 6; i++) { gyroUpdate(); delay(5); }
  float targetHeading = gyroHeadingDeg();

  int64_t startL = readEncoderCounts(leftEncoder);
  int64_t startR = readEncoderCounts(rightEncoder);

  leftEncoder.prevTime  = micros();
  rightEncoder.prevTime = micros();
  leftEncoder.prevCounts  = startL;
  rightEncoder.prevCounts = startR;

  float corrFilt = 0.0f;

  uint32_t t0 = millis();
  uint32_t last = millis();

  while (true)
  {
    if (millis() - t0 > TIMEOUT_MS) break;

    if (millis() - last < LOOP_MS) { delay(1); continue; }
    float dt = (millis() - last) / 1000.0f;
    last = millis();

    gyroUpdate();

    int64_t lc = readEncoderCounts(leftEncoder);
    int64_t rc = readEncoderCounts(rightEncoder);

    float leftDistCm  = fabs((lc - startL) * metersPerCountCal() * 100.0f);
    float rightDistCm = fabs((rc - startR) * metersPerCountCal() * 100.0f);
    float avgDistCm   = 0.5f * (leftDistCm + rightDistCm);

    float remaining = distanceCm - avgDistCm;
    if (remaining <= STOP_TOL_CM) break;

    // Base speed profile (positive magnitude)
    float baseRPMmag = speedRPM;

    if (avgDistCm < RAMP_UP_CM) {
      float s = avgDistCm / max(0.001f, RAMP_UP_CM);
      baseRPMmag = MIN_RPM + s * (speedRPM - MIN_RPM);
    }

    if (remaining < RAMP_DOWN_CM) {
      float s = remaining / max(0.001f, RAMP_DOWN_CM);
      float downRPM = MIN_RPM + s * (speedRPM - MIN_RPM);
      baseRPMmag = min(baseRPMmag, downRPM);
    }

    baseRPMmag = constrain(baseRPMmag, MIN_RPM, speedRPM);

    // Heading correction
    float hErr = angleDiffDeg(targetHeading, gyroHeadingDeg());
    if (fabs(hErr) < HEADING_DEADBAND_DEG) hErr = 0.0f;

    float corrRaw = HEADING_KP * hErr;
    corrRaw = constrain(corrRaw, -HEADING_CORR_CLAMP, HEADING_CORR_CLAMP);

    // Smooth steering
    corrFilt += CORR_ALPHA * (corrRaw - corrFilt);

    // BACKWARD = negative base RPM
    float baseRPM = -baseRPMmag;

    float leftTargetRPM  = baseRPM - corrFilt;
    float rightTargetRPM = baseRPM + corrFilt;

    // Update measured RPM and apply speed PID
    updateMotorSpeeds(dt);
    setMotorSpeedRPM(&leftMotor,  &leftEncoder,  &leftPID,  leftTargetRPM);
    setMotorSpeedRPM(&rightMotor, &rightEncoder, &rightPID, rightTargetRPM);
  }

  brakeStop(120);
  delay(80);
}


void autoDemoLoop()
{
  static const float seq_cm[] = {18.0f, 36.0f, 54.0f, 72.0f};
  static const size_t N = sizeof(seq_cm) / sizeof(seq_cm[0]);
  static size_t idx = 0;

  static bool inRest = false;
  static uint32_t restStart = 0;

  const float SPEED_RPM = 220.0f;
  const uint32_t REST_MS = 10000UL;

  if (!inRest) {
    moveForwardCmClean(seq_cm[idx], SPEED_RPM);
    stopMotors();
    inRest = true;
    restStart = millis();
    idx = (idx + 1) % N;
  } else {
    if (millis() - restStart >= REST_MS) inRest = false;
  }
}

void turnDegreesGyro(float angleDeg, float turnSpeedRPM)
{
  if (!gyroIsValid()) {
    Serial.println("Gyro not valid -> cannot do gyro turn");
    return;
  }

  // ---- knobs ----
  const uint32_t TIMEOUT_MS       = 5000;
  const uint32_t LOOP_MS          = 10;

  const float STOP_EARLY_DEG      = 2.0f;   // Stops a bit early to avoid overshoot 
  const float SLOW1_DEG           = 20.0f;  // at this point starts slowing down
  const float SLOW2_DEG           = 6.0f;   // on the last segment does min speed
  const float TURN_KP_RPM_PER_DEG = 3.0f;   // maps "remaining deg" -> RPM command

  const float MIN_TURN_RPM        = 25.0f;  // must overcome stiction
  const float MIN_ACTIVE_DEG      = 8.0f;   // below this, allow RPM to drop (prevents hunting)

  const float TURN_BALANCE = 0.03f; // bias between left/right to keep it centered

  bool turnRight = (angleDeg > 0);
  float targetDeg = fabs(angleDeg);

  // Reset TURN speed PIDs (separate from forward)
  resetPID(&leftTurnPID);
  resetPID(&rightTurnPID);

  // Init speed measurement state
  leftEncoder.prevTime    = micros();
  rightEncoder.prevTime   = micros();
  leftEncoder.prevCounts  = readEncoderCounts(leftEncoder);
  rightEncoder.prevCounts = readEncoderCounts(rightEncoder);

  // Stabilize gyro a bit
  for (int i = 0; i < 10; i++) { gyroUpdate(); delay(5); }

  float prev = gyroHeadingDeg();
  float turned = 0.0f;

  uint32_t startTime = millis();
  uint32_t lastUpdate = millis();

  while (true)
  {
    if (millis() - startTime > TIMEOUT_MS) {
      Serial.println("Turn timeout");
      break;
    }

    if (millis() - lastUpdate < LOOP_MS) { delay(1); continue; }
    float dt = (millis() - lastUpdate) / 1000.0f;
    lastUpdate = millis();

    gyroUpdate();
    float cur = gyroHeadingDeg();

    // accumulate delta with wrap handling
    float d = cur - prev;
    if (d > 180.0f)  d -= 360.0f;
    if (d < -180.0f) d += 360.0f;

    turned += fabs(d);
    prev = cur;

    float remaining = targetDeg - turned;
    if (remaining <= STOP_EARLY_DEG) break;

    // Build RPM command (fast far, slow near)
    float rpmCmd;

    if (remaining > SLOW1_DEG) {
      rpmCmd = turnSpeedRPM;
    } else if (remaining > SLOW2_DEG) {
      // proportional-ish ramp down
      rpmCmd = MIN_TURN_RPM + (remaining / SLOW1_DEG) * (turnSpeedRPM - MIN_TURN_RPM);
    } else {
      // creep zone
      rpmCmd = MIN_TURN_RPM;
    }

    // optional proportional on remaining
    float p = TURN_KP_RPM_PER_DEG * remaining;
    rpmCmd = min(rpmCmd, p);
    rpmCmd = min(rpmCmd, turnSpeedRPM);

    if (remaining > MIN_ACTIVE_DEG) rpmCmd = max(rpmCmd, MIN_TURN_RPM);
    else                            rpmCmd = max(rpmCmd, 0.0f);

    float leftTargetRPM  = (turnRight ? +rpmCmd : -rpmCmd);
    float rightTargetRPM = -leftTargetRPM;

    // Apply trim (keeps spin centered)
    leftTargetRPM  *= (1.0f + TURN_BALANCE);
    rightTargetRPM *= (1.0f - TURN_BALANCE);

    updateMotorSpeeds(dt);

    setMotorSpeedRPM(&leftMotor,  &leftEncoder,  &leftTurnPID,  leftTargetRPM);
    setMotorSpeedRPM(&rightMotor, &rightEncoder, &rightTurnPID, rightTargetRPM);
  }

  brakeStop(140);
  stopMotors();
  delay(120);

  Serial.printf("Turn done. wanted=%.1f turned=%.1f\n", targetDeg, turned);
}
