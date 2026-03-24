#include "config.h"
#include "motors.h"
#include "Arduino.h"
#include "PID.h"
#include "encoders.h"
#include "gyro_heading.h"
#include "distance.h"

Motor leftMotor  = { AIN1, AIN2, PWM_CH_L1, PWM_CH_L2, 0, 0.0f };
Motor rightMotor = { BIN1, BIN2, PWM_CH_R1, PWM_CH_R2, 0, 0.0f };


void setMotorCommand(Motor *m, int cmd)
{
    cmd = constrain(cmd, -MOTOR_ACTIVE_PWM_RANGE, MOTOR_ACTIVE_PWM_RANGE);
    if (cmd == 0) {
        m->command = 0;
        ledcWrite(m->chFwd, 0);
        ledcWrite(m->chRev, 0);
        return;
    }
    int pwm = constrain(abs(cmd) + MOTOR_PWM_MIN, MOTOR_PWM_MIN, MOTOR_PWM_MAX);
    m->command = (cmd > 0) ? pwm : -pwm;
    if (cmd > 0) { ledcWrite(m->chFwd, pwm); ledcWrite(m->chRev, 0); }
    else         { ledcWrite(m->chFwd, 0);   ledcWrite(m->chRev, pwm); }
}

void stopMotors()
{
    setMotorCommand(&leftMotor, 0);
    setMotorCommand(&rightMotor, 0);
}


namespace motors
{

// ═══════════════════════════════════════════════════════════════════
//  Timing — PID tick rates
//  CRITICAL FIX: We do NOT call registerTimeFunction(micros).
//  The PID library in tick-based mode accumulates integral per-tick
//  and computes derivative per-tick, which is correct and stable
//  for fixed-rate loops. With micros(), deltaTime=5000 would make
//  integral grow 5000x too fast and derivative 5000x too weak.
// ═══════════════════════════════════════════════════════════════════
const int   POSITION_PID_DELAY_US = 5000;   // 200 Hz
const int   VELOCITY_PID_DELAY_US = 2500;   // 400 Hz

const float COMPLETE_POSITION_ERR = 1.0f;   // encoder counts (~0.07 cm)
const float COMPLETE_ROTATION_ERR = 2.0f;   // degrees
const uint32_t SETTLE_TIME_MS     = 100;    // hold within tolerance for this long

// Wall correction alpha — applied at 200 Hz during straight moves only
const float WALL_CORRECTION_ALPHA = 0.03f;  // reduced from 0.05 to avoid jitter

bool isInAction     = false;
bool performingTurn = false;

uint32_t nowUs;
uint32_t last_pos_pid_tick = 0;
uint32_t last_vel_pid_tick = 0;

static int32_t cachedLeftCounts  = 0;
static int32_t cachedRightCounts = 0;

float lastRightVel     = 0.0f;
float lastLeftVel      = 0.0f;
float angularVelOffset = 0.0f;

float tof_correction_angle = 0.0f;

// For settle-time detection
static uint32_t settleStartMs = 0;
static bool     inSettleZone  = false;


// ═══════════════════════════════════════════════════════════════════
//  readTurn() — heading sensor for rotation PID
//  Wall correction is applied via correctGyroDriftFromWalls() which
//  nudges readDeg() at the source. No additive offset here.
// ═══════════════════════════════════════════════════════════════════
float readTurn()
{
    if (!performingTurn) {
        float distRight = (float)getDistanceRight();
        float distLeft  = (float)getDistanceLeft();
        correctGyroDriftFromWalls(distLeft, distRight, WALL_CORRECTION_ALPHA);
        tof_correction_angle = 0.3f * (distRight - distLeft); // logging only
    } else {
        tof_correction_angle = 0.0f;
    }
    return -readDeg();   // sign convention for your robot
}

// ═══════════════════════════════════════════════════════════════════
//  updateTurn() — rotation PID output handler
//  FIX: Removed the 0.3/0.7 EMA smoothing filter that was causing
//  3+ tick phase lag. The rotation PID's D term already provides
//  damping — the external filter was fighting it, causing windup
//  in the I term while the smoothed output lagged behind.
// ═══════════════════════════════════════════════════════════════════
void updateTurn(float output)
{
    angularVelOffset = output;   // direct pass-through, no smoothing
}


// ═══════════════════════════════════════════════════════════════════
//  Position PID source/output
// ═══════════════════════════════════════════════════════════════════
float readPosition()
{
    return (cachedRightCounts + cachedLeftCounts) / 2.0f;
}

void updatePosition(float targetVel)
{
    // Accel limiter
    float rightTarget = constrain(targetVel + angularVelOffset,
                                  lastRightVel - MAX_ACCEL, lastRightVel + MAX_ACCEL);
    float leftTarget  = constrain(targetVel - angularVelOffset,
                                  lastLeftVel  - MAX_ACCEL, lastLeftVel  + MAX_ACCEL);

    // Hard speed clamp — nothing gets past this
    rightTarget = constrain(rightTarget, -MAX_VELOCITY_RPM, MAX_VELOCITY_RPM);
    leftTarget  = constrain(leftTarget,  -MAX_VELOCITY_RPM, MAX_VELOCITY_RPM);

    rightVelocityPID.setTarget(rightTarget);
    leftVelocityPID.setTarget(leftTarget);

    lastRightVel = rightTarget;
    lastLeftVel  = leftTarget;
}


// ═══════════════════════════════════════════════════════════════════
//  Velocity PID source/output
//  FIX: Must call readRPM() — that's what computes and updates
//  filteredRPM from encoder counts. Without it, filteredRPM stays
//  at its initial value of 0 and the velocity PID is blind.
// ═══════════════════════════════════════════════════════════════════
float readRightVelocity()          { return readRPM(rightEncoder); }
void  updateRightVelocity(float p) { setCommand(&rightMotor, (int)p); }
float readLeftVelocity()           { return readRPM(leftEncoder); }
void  updateLeftVelocity(float p)  { setCommand(&leftMotor,  (int)p); }


// ═══════════════════════════════════════════════════════════════════
//  PID gains — tuned for TICK-BASED operation (no time function)
//
//  Position PID ticks at 200 Hz. Each tick, the integral grows by
//  exactly `error * I_gain` (not error * 5000 * I_gain like before).
//  This means I gain can be a sane value.
//
//  Rule of thumb for tick-based tuning:
//    P: same concept as time-based — proportional response to error
//    I: each tick adds error*I to the accumulator. At 200 Hz with
//       a steady error of 100 counts, I=0.01 accumulates 1.0/tick
//       = 200/sec. This is fast. Start small.
//    D: (error - lastError)*D each tick. A spike of 10 counts/tick
//       with D=1.0 gives 10 of damping. Reasonable.
// ═══════════════════════════════════════════════════════════════════

// Position: controls forward/backward motion in encoder counts
float motor_pos_P  = 0.12f;
float motor_pos_I  = 0.002f;   // tick-based: accumulates error*I per tick
float motor_pos_D  = 0.8f;     // tick-based: damping on count-rate-of-change

// Rotation: controls heading in degrees
float motor_turn_P = 2.5f;     // stronger P for crisp turns
float motor_turn_I = 0.005f;   // slow integral to eliminate steady-state
float motor_turn_D = 1.5f;     // damping to prevent overshoot

// Velocity: inner loop, ticks at 1 kHz
float motor_vel_P  = 0.6f;
float motor_vel_I  = 0.005f;   // was 0.001 with micros (= effectively 5.0)
float motor_vel_D  = 0.08f;    // was 0.1 with micros (= effectively 0.00002)

PIDController<float> rotationPID    (motor_turn_P, motor_turn_I, motor_turn_D, readTurn,          updateTurn);
PIDController<float> positionPID    (motor_pos_P,  motor_pos_I,  motor_pos_D,  readPosition,      updatePosition);
PIDController<float> rightVelocityPID(motor_vel_P, motor_vel_I,  motor_vel_D,  readRightVelocity, updateRightVelocity);
PIDController<float> leftVelocityPID (motor_vel_P, motor_vel_I,  motor_vel_D,  readLeftVelocity,  updateLeftVelocity);


// ═══════════════════════════════════════════════════════════════════
//  init()
// ═══════════════════════════════════════════════════════════════════
void init()
{
    pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
    pinMode(STBY, OUTPUT);
    digitalWrite(STBY, HIGH);

    ledcSetup(PWM_CH_L1, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CH_L2, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CH_R1, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CH_R2, PWM_FREQ, PWM_RESOLUTION);

    ledcAttachPin(AIN1, PWM_CH_L1);
    ledcAttachPin(AIN2, PWM_CH_L2);
    ledcAttachPin(BIN1, PWM_CH_R1);
    ledcAttachPin(BIN2, PWM_CH_R2);

    stopMotors();

    // ═══════════════════════════════════════════════════════════════
    //  FIX: DO NOT call registerTimeFunction(micros).
    //  The PID library uses deltaTime in its integral and derivative:
    //    integral  += error * deltaTime
    //    derivative = (error - lastError) / deltaTime
    //  With micros(), deltaTime ≈ 5000, so:
    //    integral accumulates error * 5000 per tick (EXPLOSION)
    //    derivative = error_change / 5000 (DEAD)
    //  Without registerTimeFunction, the PID operates in tick-based
    //  mode where integral += error and derivative = (error-lastError)
    //  per tick, which is correct for fixed-rate loops.
    // ═══════════════════════════════════════════════════════════════
    // positionPID.registerTimeFunction(micros);     // REMOVED
    // rotationPID.registerTimeFunction(micros);     // REMOVED
    // rightVelocityPID.registerTimeFunction(micros);// REMOVED
    // leftVelocityPID.registerTimeFunction(micros); // REMOVED

    rightVelocityPID.setOutputBounds(-MOTOR_ACTIVE_PWM_RANGE,   MOTOR_ACTIVE_PWM_RANGE);
    leftVelocityPID.setOutputBounds (-MOTOR_ACTIVE_PWM_RANGE,   MOTOR_ACTIVE_PWM_RANGE);
    rotationPID.setOutputBounds     (-(float)MAX_TURN_RATE,      (float)MAX_TURN_RATE);

    // Position PID output = velocity target in RPM. Cap to MAX_VELOCITY_RPM.
    positionPID.setOutputBounds(-MAX_VELOCITY_RPM, MAX_VELOCITY_RPM);

    // Rotation PID integral anti-windup
    rotationPID.setMaxIntegralCumulation(200.0f);
    positionPID.setMaxIntegralCumulation(500.0f);
    rightVelocityPID.setMaxIntegralCumulation(300.0f);
    leftVelocityPID.setMaxIntegralCumulation(300.0f);
}


// ═══════════════════════════════════════════════════════════════════
//  setCommand() — low-level motor PWM with dead-zone handling
// ═══════════════════════════════════════════════════════════════════
void setCommand(Motor *m, int cmd)
{
    cmd = constrain(cmd, -MOTOR_ACTIVE_PWM_RANGE, MOTOR_ACTIVE_PWM_RANGE);
    if (cmd == 0) {
        m->command = 0;
        ledcWrite(m->chFwd, 0);
        ledcWrite(m->chRev, 0);
        return;
    }
    int pwm = constrain(abs(cmd) + MOTOR_PWM_MIN, MOTOR_PWM_MIN, MOTOR_PWM_MAX);
    m->command = (cmd > 0) ? pwm : -pwm;
    if (cmd > 0) { ledcWrite(m->chFwd, pwm); ledcWrite(m->chRev, 0); }
    else         { ledcWrite(m->chFwd, 0);   ledcWrite(m->chRev, pwm); }
}


// ═══════════════════════════════════════════════════════════════════
//  tick() — main control loop, called from loop()
// ═══════════════════════════════════════════════════════════════════
void tick()
{
    nowUs = micros();

    // ── Outer loop: position + rotation at 200 Hz ────────────────
    if (nowUs - last_pos_pid_tick >= (uint32_t)POSITION_PID_DELAY_US) {
        last_pos_pid_tick = nowUs;

        readBothEncoders(cachedLeftCounts, cachedRightCounts);

        rotationPID.tick();
        positionPID.tick();

        // ── Completion detection with settle time ────────────────
        if (isInAction) {
            bool posOk  = fabsf(positionPID.getError()) / COUNTS_PER_CM < 0.5f;
            bool turnOk = fabsf(rotationPID.getError()) < COMPLETE_ROTATION_ERR;

            bool inZone;
            if (performingTurn) {
                inZone = turnOk;
            } else {
                inZone = posOk && turnOk;
            }

            if (inZone) {
                if (!inSettleZone) {
                    inSettleZone = true;
                    settleStartMs = millis();
                }
                if (millis() - settleStartMs >= SETTLE_TIME_MS) {
                    isInAction = false;
                    inSettleZone = false;
                    angularVelOffset = 0.0f;
                    lastRightVel = 0.0f;
                    lastLeftVel  = 0.0f;
                    // Hard stop first, then disable velocity PIDs.
                    // setEnabled(false) zeroes integralCumulation + output,
                    // which prevents P-term oscillation when target returns to 0.
                    stop();
                    rightVelocityPID.setTarget(0.0f);
                    leftVelocityPID.setTarget(0.0f);
                    rightVelocityPID.setEnabled(false);
                    leftVelocityPID.setEnabled(false);
                }
            } else {
                inSettleZone = false;
            }
        }
    }

    // ── Inner loop: velocity at 1 kHz ────────────────────────────
    if (nowUs - last_vel_pid_tick >= (uint32_t)VELOCITY_PID_DELAY_US) {
        last_vel_pid_tick = nowUs;
        rightVelocityPID.tick();
        leftVelocityPID.tick();
    }
}


void stop()
{
    setCommand(&leftMotor, 0);
    setCommand(&rightMotor, 0);
}

void brake(uint32_t ms)
{
    ledcWrite(leftMotor.chFwd,  255); ledcWrite(leftMotor.chRev,  255);
    ledcWrite(rightMotor.chFwd, 255); ledcWrite(rightMotor.chRev, 255);
    delay(ms);
    stopMotors();
}


// ═══════════════════════════════════════════════════════════════════
//  setTargetPosition() — move forward by `cm` centimeters
// ═══════════════════════════════════════════════════════════════════
void setTargetPosition(float cm)
{
    if (isInAction) return;
    resetDeg();
    rotationPID.setTarget(0.0f);
    tof_correction_angle = 0.0f;
    performingTurn = false;
    inSettleZone   = false;
    // Re-enable velocity PIDs for this move (may have been disabled on prior completion).
    rightVelocityPID.setEnabled(true);
    leftVelocityPID.setEnabled(true);
    isInAction     = true;
    positionPID.setTarget(readAvgPosition() + cm * COUNTS_PER_CM);
}


// ═══════════════════════════════════════════════════════════════════
//  setTargetRotation() — turn by `deg` degrees (+ = right, - = left)
// ═══════════════════════════════════════════════════════════════════
void setTargetRotation(float deg)
{
    if (isInAction) return;
    lastRightVel     = 0.0f;
    lastLeftVel      = 0.0f;
    angularVelOffset = 0.0f;
    tof_correction_angle = 0.0f;
    resetDeg();
    rotationPID.setTarget(deg);
    performingTurn = true;
    inSettleZone   = false;
    // Re-enable velocity PIDs for this move (may have been disabled on prior completion).
    rightVelocityPID.setTarget(0.0f);
    leftVelocityPID.setTarget(0.0f);
    rightVelocityPID.setEnabled(true);
    leftVelocityPID.setEnabled(true);
    isInAction     = true;
}


// ═══════════════════════════════════════════════════════════════════
//  TestTuneInnerControlLoop() — velocity PID tuning (unchanged)
// ═══════════════════════════════════════════════════════════════════
void TestTuneInnerControlLoop()
{
    enum Waveform { SQUARE, TRAPEZOID, TRIANGLE, SINE };

    const Waveform WAVE = SQUARE;
    const float    AMP        = MAX_VELOCITY_RPM * 0.8f; // 80% of speed limit
    const float    MAX_VEL    = MAX_VELOCITY_RPM;
    const uint32_t PERIOD_MS  = 2000;
    const bool RUN_BOTH = true;

    positionPID.setEnabled(false);
    rotationPID.setEnabled(false);
    angularVelOffset = 0.0f;

    const uint32_t loopStart = millis();

    for (;;) {
        const uint32_t nowMs  = millis();
        const uint32_t t      = (nowMs - loopStart) % PERIOD_MS;
        const float    phase  = (float)t / (float)PERIOD_MS;
        float          target = 0.0f;

        switch (WAVE) {
            case SQUARE:
                target = (phase < 0.5f) ? AMP : 0.0f;
                break;
            case TRAPEZOID:
                if      (phase < 0.25f) target = AMP *  (phase / 0.25f);
                else if (phase < 0.50f) target = AMP;
                else if (phase < 0.75f) target = AMP * (1.0f - (phase - 0.5f) / 0.25f);
                else                    target = 0.0f;
                break;
            case TRIANGLE:
                target = (phase < 0.5f)
                    ? AMP *  (phase / 0.5f)
                    : AMP * (1.0f - (phase - 0.5f) / 0.5f);
                break;
            case SINE:
                target = AMP * sinf(2.0f * (float)M_PI * phase);
                break;
        }

        target = constrain(target, -MAX_VEL, MAX_VEL);

        rightVelocityPID.setTarget(target);
        leftVelocityPID.setTarget(RUN_BOTH ? target : 0.0f);

        nowUs = micros();
        if (nowUs - last_vel_pid_tick >= (uint32_t)VELOCITY_PID_DELAY_US) {
            last_vel_pid_tick = nowUs;
            rightVelocityPID.tick();
            leftVelocityPID.tick();
        }

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

} // namespace motors
