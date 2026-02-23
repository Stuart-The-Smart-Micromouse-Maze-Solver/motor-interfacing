#include <Arduino.h>
#include "motion.h"
#include "encoders.h"
#include "motors.h"
#include "config.h"
#include "gyro_heading.h"

// -------------------- motion state machine --------------------
enum MotionState { MOTION_IDLE, MOTION_FORWARD, MOTION_TURN, MOTION_WAIT};
static MotionState state = MOTION_IDLE;

static long startL = 0;
static long startR = 0;
static long targetCounts = 0;
static uint32_t moveStartMs = 0;

static const int STOP_TOL_COUNTS = 1;
static const float FRONT_STOP_CM = 6.0f;

//---added rn
static const int PWM_KICK = 200;          // confirmed 200 works
static const int PWM_MAX  = 180;          // tune
static const int PWM_MIN_RUN = 160;       // above stall (tune)
static const float KP_POS = 1.50f;        // speed proportional to distance remaining
static const uint32_t KICK_MS = 120; 

static float turnTargetHeading = 0.0f;
static long  turnStartL = 0;
static long  turnStartR = 0;
static uint32_t turnStartMs = 0;

static const float TURN_TOL_DEG = 1.5f;        // 
static const uint32_t TURN_MIN_MS = 120;       // avoid instant-stop from noise
static const uint32_t TURN_TIMEOUT_MS = 6000;  // idk if this is too much

static uint32_t waitUntilMs = 0;


static inline float wrap360(float h)
{
  while (h >= 360.0f) h -= 360.0f;
  while (h < 0.0f)    h += 360.0f;
  return h;
}

// -------------------- command queue --------------------
enum CmdType { CMD_FWD_CELLS, CMD_FWD_CM, CMD_TURN_DEG, CMD_WAIT_MS };

struct MotionCmd {
  CmdType type;
  int cells;
  float cm;
  float deg;
  uint32_t waitMs;
};

static const int CMD_Q_LEN = 8;
static MotionCmd cmdQ[CMD_Q_LEN];
static int qHead = 0, qTail = 0, qCount = 0;

static bool enqueueCmd(const MotionCmd &c)
{
  if (qCount >= CMD_Q_LEN) return false;
  cmdQ[qTail] = c;
  qTail = (qTail + 1) % CMD_Q_LEN;
  qCount++;
  return true;
}

static bool dequeueCmd(MotionCmd &out)
{
  if (qCount <= 0) return false;
  out = cmdQ[qHead];
  qHead = (qHead + 1) % CMD_Q_LEN;
  qCount--;
  return true;
}


// -------------------- PID controller --------------------
class SystemPID {
public:
  SystemPID()
  : distanceKp(0.35f), distanceKi(0.0f), distanceKd(0.0f),
    encoderKp(0.35f), encoderKi(0.0f), encoderKd(0.0f),
    distancePrevError(0.0f), encoderPrevError(0.0f),
    distanceIntegral(0.0f), encoderIntegral(0.0f),
    basePWM_forward(130.0f), 
    turnKp(0.0f), turnKi(0.0f), turnKd(0.01f)
  
    {}

  void resetTurn()
  {
    turnPrevError = 0.0f;
    turnIntegral = 0.0f;
  }

  void reset()
  {
    distancePrevError = 0;
    encoderPrevError  = 0;
    distanceIntegral  = 0;
    encoderIntegral   = 0;
  }

  void update(float dt, float leftDistance, float frontDistance, float rightDistance, long remainingCounts)
  {
    if (dt <= 0.0f) dt = 1e-3f;

    // If sensors missing, disable wall correction
    if (leftDistance == -1 && rightDistance == -1) {
      leftDistance = 0;
      rightDistance = 0;
    } else {
      if (leftDistance == -1)  leftDistance  = 75.0f;
      if (rightDistance == -1) rightDistance = 79.0f;
    }

    // --- Side (wall) correction ---
    float currentPosition = constrain(leftDistance, 0, 80) - constrain(rightDistance, 0, 80);
    float distanceError = 0.0f - currentPosition;
    distanceIntegral += distanceError * dt;
    float distanceDerivative = (distanceError - distancePrevError) / dt;
    distancePrevError = distanceError;

    float sideCorrection = (distanceKp * distanceError) + (distanceKi * distanceIntegral) + (distanceKd * distanceDerivative);

    // If walls far / not reliable, ignore side correction
    if (((int)leftDistance >= 40 || (int)leftDistance == 0) &&
        ((int)rightDistance >= 40 || (int)rightDistance == 0)) {
      sideCorrection = 0.0f;
    }

    // --- Encoder straightness correction ---
    long leftCounts  = (long)readEncoderCounts(leftEncoder);
    long rightCounts = (long)readEncoderCounts(rightEncoder);

    float currentEncoderDiff = (float)((leftCounts - startL) - (rightCounts - startR));

    float encoderError = 0.0f - currentEncoderDiff;
    encoderIntegral += encoderError * dt;
    float encoderDerivative = (encoderError - encoderPrevError) / dt;
    encoderPrevError = encoderError;

    float encoderCorrection = (encoderKp * encoderError) + (encoderKi * encoderIntegral) + (encoderKd * encoderDerivative);

    //float base = basePWM_forward;

    // dynamic speed profile (scaled by distance)
    /*    
    float base = 0.0f;
    float progress = targetCounts - remainingCounts;

    float cells = (float)targetCounts / (float)COUNTS_PER_CELL;

    // Short moves -> lower max; long moves -> higher min
    float minRun = (cells >= 4.0f) ? 160.0f : (cells >= 3.0f ? 160.0f : 155.0f);
    float maxpwm = (cells <= 2.0f) ? 160.0f : (cells <= 3.0f ? 165.0f : 165.0f);
    if (maxpwm < minRun + 5.0f) maxpwm = minRun + 5.0f;

    float accelCounts = constrain(targetCounts * 0.25f, 15.0f, 120.0f);
    float decelCounts = constrain(targetCounts * 0.70f, 70.0f, 260.0f);

    if (progress < accelCounts) {
      base = minRun + (maxpwm - minRun) * (progress / accelCounts);
    } else if (remainingCounts < decelCounts) {
      float decelRatio = remainingCounts / decelCounts;
      decelRatio = constrain(decelRatio, 0.0f, 1.0f);
      base = minRun + (maxpwm - minRun) * (decelRatio * decelRatio);
    } else {
      base = maxpwm;
    }

    // Softer kick for long runs
    float kickPwm = (cells >= 4.0f) ? 190.0f : 190.0f;
    uint32_t kickMs = (cells >= 4.0f) ? 70 : 120;
    const float kickBiasRight = 8.0f; // compensate stronger left motor during kick
    bool kickActive = false;

    if (millis() - moveStartMs < kickMs) {
      base = kickPwm;
      kickActive = true;
    }
    

    

    float leftPWM  = base + sideCorrection + encoderCorrection;
    float rightPWM = base - sideCorrection - encoderCorrection;

    if (kickActive) {
      leftPWM  -= kickBiasRight;
      rightPWM += kickBiasRight;
    }


    leftPWM  = constrain(leftPWM,  -255.0f, 255.0f);
    rightPWM = constrain(rightPWM, -255.0f, 255.0f);

    setMotorCommand(&leftMotor,  (int)(leftPWM));
    setMotorCommand(&rightMotor, (int)(rightPWM));
    */

    // motors::rightMotorPID.s
  }
 
  void turnUpdate(float dt, float targetHeadingDeg, long turnStartL, long turnStartR)
{
  if (dt <= 0.0f) dt = 1e-3f;

  float current = gyroHeadingDeg();
  float err = angleDiffDeg(targetHeadingDeg, current); 

  // PID on heading error
  turnIntegral += err * dt;
  turnIntegral = constrain(turnIntegral, -30.0f, 30.0f); // anti-windup
  
  float deriv = (err - turnPrevError) / dt;
  turnPrevError = err;

  float u = turnKp * err + turnKi * turnIntegral + turnKd * deriv;

  int dir = (err > 0) ? +1 : -1;

  // Base + correction magnitude
  const int PWM_TURN_BASE = 170;   // This works to break friction
  const int PWM_TURN_MIN  = 160;
  const int PWM_TURN_MAX  = 175;  // not sure if too high

  float mag = PWM_TURN_BASE + fabsf(u);
  mag = constrain(mag, (float)PWM_TURN_MIN, (float)PWM_TURN_MAX);

  // Encoder balance to keep pivot symmetric
  long dL = (long)readEncoderCounts(leftEncoder)  - turnStartL;
  long dR = (long)readEncoderCounts(rightEncoder) - turnStartR;
  // For a perfect pivot, dL + dR ~= 0 (left negative, right positive).
  float balanceErr = (float)(dL + dR);
  float balanceKp  = 0.20f;              // tune: increase if one wheel overshoots
  float balanceCorr = balanceKp * balanceErr;

  // Apply correction to reduce the wheel that moved more
  int leftCmd  = (int)constrain((-dir * mag) + balanceCorr, -255.0f, 255.0f);
  int rightCmd = (int)constrain(( dir * mag) - balanceCorr, -255.0f, 255.0f);

  setMotorCommand(&leftMotor, leftCmd);
  setMotorCommand(&rightMotor, rightCmd);
}
  
  float distanceKp, distanceKi, distanceKd;
  float encoderKp, encoderKi, encoderKd;
  float turnKp, turnKi, turnKd;

  float distancePrevError, encoderPrevError;
  float distanceIntegral, encoderIntegral;

  float turnPrevError = 0.0f;
  float turnIntegral = 0.0f;

  float basePWM_forward;
};

static SystemPID PID;

// -------------------- helpers --------------------
static inline long avgProgressCounts()
{
  long currL = (long)readEncoderCounts(leftEncoder);
  long currR = (long)readEncoderCounts(rightEncoder);
  long dL = currL - startL;
  long dR = currR - startR;
  return (labs(dL) + labs(dR)) / 2;
}

// -------------------- public API --------------------
void motionInit()
{
  state = MOTION_IDLE;
}

bool motionIsBusy()
{
  return state != MOTION_IDLE;
}

void motionStop()
{
  stopMotors();
  state = MOTION_IDLE;
}

bool motionMoveForwardCells(int cells)
{
  if (cells <= 0) return false;
  if (motionIsBusy()) return false;

  startL = (long)readEncoderCounts(leftEncoder);
  startR = (long)readEncoderCounts(rightEncoder);
  targetCounts = (long)cells * (long)COUNTS_PER_CELL;

  PID.reset();
  moveStartMs = millis();
  state = MOTION_FORWARD;
  return true;
}

bool motionMoveForwardCm(float cm)
{
  if (cm <= 0) return false;
  if (motionIsBusy()) return false;

  long counts = (long)lround(cm / CM_PER_COUNT);

  startL = (long)readEncoderCounts(leftEncoder);
  startR = (long)readEncoderCounts(rightEncoder);
  targetCounts = counts;

  PID.reset();
  moveStartMs = millis();
  state = MOTION_FORWARD;
  return true;
}

bool motionTurnDeg(float deg)
{

  if (motionIsBusy()) return false;
  if (!gyroIsValid()) return false;

  float start = gyroHeadingDeg();

  float target = wrap360(start - deg);

  // 0..360
  while (target >= 360.0f) target -= 360.0f;
  while (target < 0.0f)    target += 360.0f;

  turnTargetHeading = target;
  turnStartL = (long)readEncoderCounts(leftEncoder);
  turnStartR = (long)readEncoderCounts(rightEncoder);
  turnStartMs = millis();

  PID.resetTurn();
  state = MOTION_TURN;
  return true;
}

static void tryStartNextCmd()
{
  if (state != MOTION_IDLE) return;
  if (qCount == 0) return;

  MotionCmd c;
  if (!dequeueCmd(c)) return;

  bool ok = false;
  switch (c.type) {
    case CMD_FWD_CELLS: ok = motionMoveForwardCells(c.cells); break;
    case CMD_FWD_CM:    ok = motionMoveForwardCm(c.cm);       break;
    case CMD_TURN_DEG:  ok = motionTurnDeg(c.deg);            break;

    case CMD_WAIT_MS:
      stopMotors();
      waitUntilMs = millis() + (uint16_t)c.waitMs;
      state = MOTION_WAIT;
      ok = true;
      break;
  }

  if (!ok) {
    stopMotors();
    state = MOTION_IDLE;
  }
}

void motionUpdate(float dt, float leftDist, float frontDist, float rightDist)
{
  gyroUpdate();

  if (state == MOTION_IDLE) {
    tryStartNextCmd();
    if (state == MOTION_IDLE) return; // still nothing to do
  }

  // when tof front is implemented
  if (frontDist >= 0 && frontDist < FRONT_STOP_CM) {
    brakeMotors(30);
    motionStop();
    return;
  }

    if (state == MOTION_WAIT) {
      stopMotors();
      if ((int32_t)(millis() - waitUntilMs) >= 0) {
      state = MOTION_IDLE; 
      }
    return;
    }


  if (state == MOTION_FORWARD) {
    long prog = avgProgressCounts();
    long remaining = targetCounts - prog;

    if (prog >= (targetCounts - STOP_TOL_COUNTS)) {
      brakeMotors(50); //stops the wheels quickly 
      motionStop();
      return;
    }
    // Drive using PID straightening
    PID.update(dt, leftDist, frontDist, rightDist, remaining);
    return; 
  }

  if (state == MOTION_TURN) {
    float err = angleDiffDeg(turnTargetHeading, gyroHeadingDeg());

    PID.turnUpdate(dt, turnTargetHeading, turnStartL, turnStartR);

    if ((fabsf(err) < TURN_TOL_DEG) && (millis() - turnStartMs > TURN_MIN_MS)) {
      brakeMotors(20);
      motionStop();
      return;
    }

    if (millis() - turnStartMs > TURN_TIMEOUT_MS) {
      brakeMotors(30);
      motionStop();
      return;
    }
    return;
  }
}

bool MoveForwardCells(int cells)
{
  bool ok = enqueueCmd({CMD_FWD_CELLS, cells, 0.0f, 0.0f, 0});
  tryStartNextCmd(); // start immediately if idle
  return ok;
}

bool MoveForwardCm(float cm)
{
  MotionCmd c;
  c.type = CMD_FWD_CM;
  c.cells = 0;
  c.cm = cm;
  c.deg = 0.0f;

  bool ok = enqueueCmd(c);
  tryStartNextCmd();
  return ok;
}

bool TurnRight()
{
  bool ok = enqueueCmd({CMD_TURN_DEG, 0, 0.0f, 90.0f,0});
  tryStartNextCmd();
  return ok;
}

bool TurnLeft()
{
  bool ok = enqueueCmd({CMD_TURN_DEG, 0, 0.0f, -90.0f,0});
  tryStartNextCmd();
  return ok;
}

bool Turn180()
{
  bool ok = enqueueCmd({CMD_TURN_DEG, 0, 0.0f, 180.0f,0});
  tryStartNextCmd();
  return ok;
}

bool WaitMs(uint16_t ms)
{
  bool ok = enqueueCmd({CMD_WAIT_MS, 0, 0.0f, 0.0f, ms});
  tryStartNextCmd();
  return ok;
}



