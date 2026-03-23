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

const int   POSITION_PID_DELAY_US = 5000;
const int   VELOCITY_PID_DELAY_US = 1000;
const float COMPLETE_POSITION_ERR = 0.5f;   // cm
const float COMPLETE_ROTATION_ERR = 3.0f;   // degrees

// ── Wall correction alpha ─────────────────────────────────────────────────────
// How aggressively to nudge the gyro toward wall-implied heading each tick.
// This is applied at 200 Hz (every position PID tick where readTurn is called).
// 0.10 = 10% correction per tick toward truth — fast enough to catch drift
//        within one cell at normal speeds.
// Decrease if corrections feel jerky; increase if drift isn't caught in time.
// The effective rate is also scaled by distance weight inside correctGyroDriftFromWalls,
// so single-wall and far-wall corrections are automatically gentler.
const float WALL_CORRECTION_ALPHA = 0.05f;

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

float tof_correction_angle = 0.0f;  // logged via server, not used in control


// ─────────────────────────────────────────────────────────────────────────────
//  readTurn()
//
//  Returns gyro heading. Wall correction is applied by calling
//  correctGyroDriftFromWalls() which nudges readDeg() at the source —
//  no additive offset here. This prevents double-application.
//
//  Supports: both walls, one wall, multi-cell distances.
//  See heading.cpp correctGyroDriftFromWalls() for geometry details.
// ─────────────────────────────────────────────────────────────────────────────
float readTurn()
{
    if (!performingTurn) {
        float distRight = (float)getDistanceRight();
        float distLeft  = (float)getDistanceLeft();

        // correctGyroDriftFromWalls handles all validity checking internally.
        // It classifies the readings (both/one/none), applies appropriate weights,
        // and nudges zeroOffsetDeg. readDeg() below already reflects the correction.
        correctGyroDriftFromWalls(distLeft, distRight, WALL_CORRECTION_ALPHA);

        // For server logging only — doesn't feed into control
        tof_correction_angle = 0.3f * (distRight - distLeft);
    } else {
        tof_correction_angle = 0.0f;
    }

    return -readDeg();   // negative: sign convention for your robot
}

void updateTurn(float output)
{
    static float smoothed = 0.0f;
    smoothed = 0.3f * output + 0.7f * smoothed;
    angularVelOffset = smoothed;
}


// ─────────────────────────────────────────────────────────────────────────────
//  readPosition() / updatePosition()
// ─────────────────────────────────────────────────────────────────────────────
float readPosition()
{
    return (cachedRightCounts + cachedLeftCounts) / 2.0f;

    // Front ToF fusion — enable once straight-line is solid
    /*
    int frontMM = getDistanceFront();
    if (!performingTurn && frontMM > 10 && frontMM < (int)(FRONT_TOF_TO_WALL_CM * 10.0f + 60.0f)) {
        float errorMM   = (float)frontMM - FRONT_TOF_TO_WALL_CM * 10.0f;
        float tofCounts = (errorMM / 10.0f) * COUNTS_PER_CM;
        return (cachedRightCounts + cachedLeftCounts) / 2.0f + tofCounts * 0.3f;
    }
    return (cachedRightCounts + cachedLeftCounts) / 2.0f;
    */
}

void updatePosition(float targetVel)
{
    float rightTarget = constrain(targetVel + angularVelOffset,
                                  lastRightVel - MAX_ACCEL, lastRightVel + MAX_ACCEL);
    float leftTarget  = constrain(targetVel - angularVelOffset,
                                  lastLeftVel  - MAX_ACCEL, lastLeftVel  + MAX_ACCEL);

    rightVelocityPID.setTarget(rightTarget);
    leftVelocityPID.setTarget(leftTarget);

    lastRightVel = rightTarget;
    lastLeftVel  = leftTarget;
}

float readRightVelocity()          { return readRPM(rightEncoder); }
// float readRightVelocity()          { return rightEncoder.filteredRPM; }
void  updateRightVelocity(float p) { setCommand(&rightMotor, (int)p); }
float readLeftVelocity()           { return readRPM(leftEncoder); }
// float readLeftVelocity()           { return leftEncoder.filteredRPM; }
void  updateLeftVelocity(float p)  { setCommand(&leftMotor,  (int)p); }


float motor_pos_P  = 0.2f;
float motor_pos_I  = 0.0016f;
float motor_pos_D  = 0.0f;

float motor_turn_P = 1.0f;
float motor_turn_I = 0.0004f;
float motor_turn_D = 0.0f;

float motor_vel_P  = 0.6f;
float motor_vel_I  = 0.001f;
float motor_vel_D  = 0.1f;

PIDController<float> rotationPID    (motor_turn_P, motor_turn_I, motor_turn_D, readTurn,          updateTurn);
PIDController<float> positionPID    (motor_pos_P,  motor_pos_I,  motor_pos_D,  readPosition,      updatePosition);
PIDController<float> rightVelocityPID(motor_vel_P, motor_vel_I,  motor_vel_D,  readRightVelocity, updateRightVelocity);
PIDController<float> leftVelocityPID (motor_vel_P, motor_vel_I,  motor_vel_D,  readLeftVelocity,  updateLeftVelocity);


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

    positionPID.registerTimeFunction(micros);
    rotationPID.registerTimeFunction(micros);
    rightVelocityPID.registerTimeFunction(micros);
    leftVelocityPID.registerTimeFunction(micros);

    rightVelocityPID.setOutputBounds(-MOTOR_ACTIVE_PWM_RANGE,   MOTOR_ACTIVE_PWM_RANGE);
    leftVelocityPID.setOutputBounds (-MOTOR_ACTIVE_PWM_RANGE,   MOTOR_ACTIVE_PWM_RANGE);
    rotationPID.setOutputBounds     (-MOTOR_ACTIVE_PWM_RANGE*2, MOTOR_ACTIVE_PWM_RANGE*2);
}

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

void tick()
{
    nowUs = micros();

    if (nowUs - last_pos_pid_tick >= (uint32_t)POSITION_PID_DELAY_US) {
        last_pos_pid_tick = nowUs;

        readBothEncoders(cachedLeftCounts, cachedRightCounts);

        rotationPID.tick();
        positionPID.tick();

        if (fabsf(rotationPID.getError()) < COMPLETE_ROTATION_ERR &&
            fabsf(positionPID.getError()) / COUNTS_PER_CM < COMPLETE_POSITION_ERR) {
            isInAction = false;
        }
    }

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

void setTargetPosition(float cm)
{
    if (isInAction) return;
    resetDeg();
    rotationPID.setTarget(0.0f);
    tof_correction_angle = 0.0f;
    performingTurn = false;
    isInAction     = true;
    positionPID.setTarget(readAvgPosition() + cm * COUNTS_PER_CM);
}

void setTargetRotation(float deg)
{
    if (isInAction) return;
    rightVelocityPID.setTarget(0.0f);
    leftVelocityPID.setTarget(0.0f);
    lastRightVel     = 0.0f;
    lastLeftVel      = 0.0f;
    angularVelOffset = 0.0f;
    tof_correction_angle = 0.0f;
    resetDeg();
    rotationPID.setTarget(deg);
    performingTurn = true;
    isInAction     = true;
}


// ─────────────────────────────────────────────────────────────────────────────
//  TestTuneInnerControlLoop()
//
//  Blocking eternal loop for tuning the velocity PIDs in isolation.
//  Bypasses positionPID and rotationPID entirely — sets velocity targets directly.
//
//  Waveform guide:
//    SQUARE    — best starting point for P and D tuning. The step response
//                reveals overshoot, settling time, and ringing clearly.
//    TRAPEZOID — best for I tuning. Ramp + hold shows whether the integrator
//                eliminates steady-state error without winding up on the ramp.
//    TRIANGLE  — constant acceleration demand. Good for checking whether I
//                accumulates (winds up) inappropriately during a ramp.
//    SINE      — validation after P/I/D are roughly dialled in. If the motor
//                tracks a sine cleanly, the loop has sufficient bandwidth.
//
//  Adjust the CONFIG block below. Nothing else needs changing.
// ─────────────────────────────────────────────────────────────────────────────
void TestTuneInnerControlLoop()
{
    // ── CONFIG ────────────────────────────────────────────────────────────────
    enum Waveform { SQUARE, TRAPEZOID, TRIANGLE, SINE };
 
    /*  Uncomment whichever waveform you want to run:  */
    const Waveform WAVE = SQUARE;
    // const Waveform WAVE = TRAPEZOID;
    // const Waveform WAVE = TRIANGLE;
    // const Waveform WAVE = SINE;
 
    const float    AMP        = 400.0f;  // peak velocity target (same units as readRPM)
    const float    MAX_VEL    = 500.0f;  // hard clamp — never exceeded regardless of waveform
    const uint32_t PERIOD_MS  = 2000;     // full cycle length in ms:
    //   SQUARE:    high AMP for PERIOD_MS/2, then 0 for PERIOD_MS/2
    //   TRAPEZOID: ramp-up PERIOD_MS/4 | hold PERIOD_MS/4 | ramp-down PERIOD_MS/4 | hold-0 PERIOD_MS/4
    //   TRIANGLE:  ramp up PERIOD_MS/2, ramp back down PERIOD_MS/2
    //   SINE:      one full sinusoidal cycle (signed — tests both directions)
 
    const bool RUN_BOTH = true;  // true  = both motors get the same target (normal)
                                 // false = right motor only, left holds 0 (isolation)
    // ── END CONFIG ────────────────────────────────────────────────────────────
 
    // Disable position and rotation PIDs so they don't fight the velocity loop
    positionPID.setEnabled(false);
    rotationPID.setEnabled(false);
    angularVelOffset = 0.0f;  // zero cross-coupling from rotation PID
 
    const uint32_t loopStart = millis();
 
    for (;;) {
        const uint32_t nowMs  = millis();
        const uint32_t t      = (nowMs - loopStart) % PERIOD_MS;
        const float    phase  = (float)t / (float)PERIOD_MS;  // 0.0 → <1.0
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
                // Signed sine — tests positive and negative velocity tracking
                target = AMP * sinf(2.0f * (float)M_PI * phase);
                // Swap to offset sine (always positive) if preferred:
                // target = AMP * 0.5f * (1.0f - cosf(2.0f * (float)M_PI * phase));
                break;
        }
 
        // Hard clamp — never exceed MAX_VEL regardless of waveform math
        target = constrain(target, -MAX_VEL, MAX_VEL);
 
        // Push target directly to velocity PIDs, bypassing position/rotation layer
        rightVelocityPID.setTarget(target);
        leftVelocityPID.setTarget(RUN_BOTH ? target : 0.0f);
 
        // Tick only the velocity loop
        nowUs = micros();
        if (nowUs - last_vel_pid_tick >= (uint32_t)VELOCITY_PID_DELAY_US) {
            last_vel_pid_tick = nowUs;
            rightVelocityPID.tick();
            leftVelocityPID.tick();
        }
 
        // 1ms yield — keeps the FreeRTOS watchdog happy and lets Core 0 breathe
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}
 
} // namespace motors