#include "autotune.h"
#include "motors.h"
#include "encoders.h"
#include "gyro_heading.h"
#include "config.h"

// ═══════════════════════════════════════════════════════════════════
//  Step response sampling buffer
//  1500 samples at 1kHz = 1.5 seconds of data per test
// ═══════════════════════════════════════════════════════════════════
static const int MAX_SAMPLES = 1500;
static float samplesL[MAX_SAMPLES];
static float samplesR[MAX_SAMPLES];

// Forward declarations
static void disableAllPIDs();
static void enableAllPIDs();

// Helper: tick only velocity PIDs for a duration (ms), keeping outer disabled
static void tickVelocityOnly(uint32_t durationMs) {
    uint32_t start = millis();
    uint32_t lastTick = micros();
    while (millis() - start < durationMs) {
        uint32_t now = micros();
        if (now - lastTick >= 1000) {  // 1kHz
            lastTick = now;
            motors::rightVelocityPID.tick();
            motors::leftVelocityPID.tick();
        }
        yield();
    }
}


// ═══════════════════════════════════════════════════════════════════
//  Phase 1: Motor step response characterization
//
//  Drives BOTH motors forward at the same command so the robot
//  goes straight (not spinning). Measures RPM response for each
//  motor simultaneously.
//
//  Returns: plant gain K (RPM per command unit) and time constant
//           τ (milliseconds) for each motor.
// ═══════════════════════════════════════════════════════════════════
struct MotorModel {
    float K;        // steady-state RPM per command unit
    float tau_ms;   // time constant in ms
    float ss_rpm;   // steady-state RPM
    bool  valid;
};

static MotorModel characterizeMotors(
    int stepCmd,
    std::function<void(const String&)> logFn)
{
    MotorModel result = {0, 0, 0, false};

    // Zero motors, wait for things to settle
    motors::setCommand(&leftMotor, 0);
    motors::setCommand(&rightMotor, 0);
    delay(500);

    // Reset encoder RPM tracking
    readRPM(leftEncoder);
    readRPM(rightEncoder);
    delay(50);

    // Apply step to both motors simultaneously
    motors::setCommand(&leftMotor, stepCmd);
    motors::setCommand(&rightMotor, stepCmd);

    // Sample RPM at 1kHz for MAX_SAMPLES ticks
    int n = 0;
    uint32_t lastSampleUs = micros();
    while (n < MAX_SAMPLES) {
        uint32_t now = micros();
        if (now - lastSampleUs >= 1000) {  // 1kHz
            lastSampleUs = now;
            samplesL[n] = readRPM(leftEncoder);
            samplesR[n] = readRPM(rightEncoder);
            n++;
        }
        yield();
    }

    // Stop motors
    motors::setCommand(&leftMotor, 0);
    motors::setCommand(&rightMotor, 0);
    delay(200);

    // ── Compute steady-state (average of last 500 samples = 500ms) ──
    float ssL = 0, ssR = 0;
    int ssStart = MAX_SAMPLES - 500;
    for (int i = ssStart; i < MAX_SAMPLES; i++) {
        ssL += samplesL[i];
        ssR += samplesR[i];
    }
    ssL /= 500.0f;
    ssR /= 500.0f;

    if (ssL < 5.0f && ssR < 5.0f) {
        logFn("  ERROR: Neither motor responded! Check wiring/driver.");
        return result;
    }
    if (ssL < 5.0f) {
        logFn("  WARNING: Left motor weak (SS=" + String(ssL,1) + " RPM). Using right only.");
        ssL = ssR;  // use right as estimate
    }
    if (ssR < 5.0f) {
        logFn("  WARNING: Right motor weak (SS=" + String(ssR,1) + " RPM). Using left only.");
        ssR = ssL;  // use left as estimate
    }

    float avgSS = (ssL + ssR) / 2.0f;
    float K = avgSS / (float)stepCmd;

    // ── Compute time constant τ (time to reach 63.2% of steady-state) ──
    float threshold632_L = ssL * 0.632f;
    float threshold632_R = ssR * 0.632f;
    int tauTicksL = MAX_SAMPLES, tauTicksR = MAX_SAMPLES;

    for (int i = 0; i < MAX_SAMPLES; i++) {
        if (samplesL[i] >= threshold632_L && tauTicksL == MAX_SAMPLES) tauTicksL = i;
        if (samplesR[i] >= threshold632_R && tauTicksR == MAX_SAMPLES) tauTicksR = i;
    }

    float tauL_ms = (float)tauTicksL;   // at 1kHz, 1 tick = 1ms
    float tauR_ms = (float)tauTicksR;
    float avgTau = (tauL_ms + tauR_ms) / 2.0f;

    // Sanity checks
    if (avgTau < 2.0f)   avgTau = 10.0f;   // minimum 10ms (filter artifact)
    if (avgTau > 800.0f) avgTau = 200.0f;   // cap at 200ms

    logFn("  Left:  SS=" + String(ssL,1) + " RPM, tau=" + String(tauL_ms,0) + " ms");
    logFn("  Right: SS=" + String(ssR,1) + " RPM, tau=" + String(tauR_ms,0) + " ms");
    logFn("  Plant K=" + String(K,4) + " RPM/cmd, tau=" + String(avgTau,1) + " ms");

    result.K = K;
    result.tau_ms = avgTau;
    result.ss_rpm = avgSS;
    result.valid = true;
    return result;
}


// ═══════════════════════════════════════════════════════════════════
//  Phase 2: Compute velocity PID gains from plant model
//
//  Uses IMC (Internal Model Control) tuning for first-order plant.
//  The PID is tick-based at 1kHz:
//    output = Kp*e + Ki*sum(e) + Kd*(e - e_prev)
//
//  lambda = desired closed-loop time constant (in ticks).
//  Larger lambda = slower but more stable.
// ═══════════════════════════════════════════════════════════════════
struct PIDGains { float P, I, D; };

static PIDGains computeVelocityGains(float K, float tau_ms, float lambdaFactor)
{
    PIDGains g;

    // Convert τ from ms to ticks (at 1kHz, 1 tick = 1ms)
    float tau_ticks = tau_ms;

    // λ = desired closed-loop time constant (ticks)
    // lambdaFactor=1.0 is aggressive, 2.0 is conservative
    float lambda_ticks = tau_ticks * lambdaFactor;
    if (lambda_ticks < 5.0f) lambda_ticks = 5.0f;   // minimum 5ms

    // IMC tuning for first-order plant:
    //   Kp = τ / (K * λ)
    //   Ki = 1 / (K * λ)     [per tick]
    //   Kd = 0                [velocity loops rarely need D]
    g.P = tau_ticks / (K * lambda_ticks);
    g.I = 1.0f / (K * lambda_ticks);
    g.D = 0.0f;

    // Sanity bounds
    // FIX: Minimum I raised from 0.0001 to 0.002. With the motor dead zone
    // (MOTOR_PWM_MIN=100), the velocity PID must build enough integral to
    // output ~40 cmd units before the motor starts spinning. At I=0.0005
    // this takes over 1 second. At I=0.002 with error=60 RPM the integral
    // reaches 40 in ~333 ticks (333ms) — much more responsive.
    g.P = constrain(g.P, 0.05f, 5.0f);
    g.I = constrain(g.I, 0.002f, 0.1f);

    return g;
}


// ═══════════════════════════════════════════════════════════════════
//  Phase 3: Verify velocity PID with closed-loop step test
//
//  Applies a step target to velocity PIDs, measures overshoot
//  and settling time. Returns overshoot percentage.
// ═══════════════════════════════════════════════════════════════════
static float velocityStepTest(
    float targetRPM,
    PIDGains gains,
    std::function<void(const String&)> logFn)
{
    // Apply gains
    motors::rightVelocityPID.setPID(gains.P, gains.I, gains.D);
    motors::leftVelocityPID.setPID(gains.P, gains.I, gains.D);

    // Enable velocity PIDs, disable outer loops
    motors::positionPID.setEnabled(false);
    motors::rotationPID.setEnabled(false);
    motors::rightVelocityPID.setEnabled(true);
    motors::leftVelocityPID.setEnabled(true);

    // Zero targets, wait
    motors::rightVelocityPID.setTarget(0.0f);
    motors::leftVelocityPID.setTarget(0.0f);
    tickVelocityOnly(300);

    // Step to target
    motors::rightVelocityPID.setTarget(targetRPM);
    motors::leftVelocityPID.setTarget(targetRPM);

    // Sample for 1.5s
    float peakRPM = 0;
    int n = 0;
    uint32_t lastSampleUs = micros();
    while (n < MAX_SAMPLES) {
        uint32_t now = micros();
        if (now - lastSampleUs >= 1000) {
            lastSampleUs = now;
            motors::rightVelocityPID.tick();
            motors::leftVelocityPID.tick();
            float avgRPM = (readRPM(rightEncoder) + readRPM(leftEncoder)) / 2.0f;
            if (avgRPM > peakRPM) peakRPM = avgRPM;
            samplesL[n] = avgRPM;
            n++;
        }
        yield();
    }

    // Stop
    motors::rightVelocityPID.setTarget(0.0f);
    motors::leftVelocityPID.setTarget(0.0f);
    tickVelocityOnly(300);
    motors::stop();

    // Compute overshoot
    float overshoot = ((peakRPM - targetRPM) / targetRPM) * 100.0f;
    if (overshoot < 0) overshoot = 0;

    // FIX: Also check that the controller actually REACHED the target.
    // Previously only overshoot was checked — a controller that topped out at
    // 30 RPM targeting 65 RPM would report 0% overshoot and pass.
    // Compute steady-state from last 200 samples.
    float ssAvg = 0;
    int ssCount = min(MAX_SAMPLES, 200);
    for (int i = MAX_SAMPLES - ssCount; i < MAX_SAMPLES; i++) ssAvg += samplesL[i];
    ssAvg /= (float)ssCount;

    bool reachedTarget = (ssAvg >= targetRPM * 0.60f);  // must reach at least 60% of target

    // Compute settling time (within 5% of target)
    float band = targetRPM * 0.05f;
    int settledTick = MAX_SAMPLES;
    for (int i = MAX_SAMPLES - 1; i >= 0; i--) {
        if (fabsf(samplesL[i] - targetRPM) > band) {
            settledTick = i + 1;
            break;
        }
    }
    float settleMs = (float)settledTick;

    logFn("  Step test: overshoot=" + String(overshoot,1) + "%, settle=" +
          String(settleMs,0) + "ms, peak=" + String(peakRPM,1) +
          ", ss=" + String(ssAvg,1) + (reachedTarget ? " OK" : " UNDER-TARGET"));

    // FIX: If the controller never reached the target, force a retry
    // by reporting high overshoot. Phase 3 retry makes lambda MORE
    // conservative (lower gains), which is wrong here — we actually need
    // MORE aggressive gains. So we return a special large value that the
    // caller interprets as "needs more gain, not less".
    if (!reachedTarget) return -1.0f;  // negative = under-target, needs more gain

    return overshoot;
}


// ═══════════════════════════════════════════════════════════════════
//  Phase 4: Tune rotation PID
//
//  With velocity loop running, applies small angle steps and
//  iteratively adjusts rotation gains.
//  The robot pivots in place — caster ball friction is accounted for
//  by the integral term.
// ═══════════════════════════════════════════════════════════════════
static PIDGains tuneRotation(std::function<void(const String&)> logFn)
{
    PIDGains g;

    // Starting point: moderate P, no I, good D for damping
    // These are tick-based at 200Hz
    g.P = 2.0f;
    g.I = 0.0f;
    g.D = 1.0f;

    motors::rotationPID.setPID(g.P, g.I, g.D);
    motors::rotationPID.setEnabled(true);
    motors::rotationPID.setOutputBounds(-CALIBRATION_RPM, CALIBRATION_RPM);

    // FIX: Position PID MUST be enabled, set to hold current position.
    // The cascade path is: rotationPID → updateTurn() → angularVelOffset
    //   → used inside updatePosition() → velocity PID targets.
    // When positionPID was disabled, updatePosition() never ran, so
    // angularVelOffset had NO path to the motors — the robot couldn't turn.
    // With positionPID holding position, updatePosition outputs:
    //   rightVelTarget = ~0 + angularVelOffset  (pivot right)
    //   leftVelTarget  = ~0 - angularVelOffset  (pivot left)
    motors::positionPID.setEnabled(true);
    motors::positionPID.setTarget(readAvgPosition());
    motors::performingTurn = true;   // suppress wall drift correction during turn tests

    float testAngle = 30.0f;  // degrees

    for (int attempt = 0; attempt < 5; attempt++) {
        motors::rotationPID.setPID(g.P, g.I, g.D);
        logFn("  Turn test #" + String(attempt+1) + ": P=" + String(g.P,3) +
              " I=" + String(g.I,4) + " D=" + String(g.D,3));

        // Reset heading and set target
        resetDeg();
        motors::rotationPID.setTarget(testAngle);
        // Re-anchor position hold to current position each attempt
        motors::positionPID.setTarget(readAvgPosition());

        // Run for 2 seconds, tracking overshoot
        float peakAngle = 0;
        float finalAngle = 0;
        uint32_t start = millis();

        while (millis() - start < 2000) {
            motors::tick();   // handles all PID timing + cached encoder update
            float angle = -readDeg();
            if (angle > peakAngle) peakAngle = angle;
            finalAngle = angle;
            yield();
        }

        // Stop and measure
        motors::rotationPID.setTarget(0.0f);  // doesn't matter, we're analyzing
        motors::stop();
        delay(200);

        float overshoot = peakAngle - testAngle;
        float steadyErr = fabsf(finalAngle - testAngle);

        logFn("  Result: peak=" + String(peakAngle,1) + " final=" +
              String(finalAngle,1) + " overshoot=" + String(overshoot,1) +
              " err=" + String(steadyErr,1));

        // ── Adjust gains ──
        bool needsRetry = false;

        if (overshoot > 8.0f) {
            // Too much overshoot — more damping, less P
            g.D *= 1.5f;
            g.P *= 0.8f;
            needsRetry = true;
            logFn("  -> Reducing P, increasing D (overshoot)");
        } else if (overshoot > 3.0f) {
            // Moderate overshoot — just add damping
            g.D *= 1.2f;
            needsRetry = true;
            logFn("  -> Increasing D slightly");
        }

        if (steadyErr > 3.0f && overshoot < 5.0f) {
            // Steady-state error from friction (caster ball) — add I
            g.I = g.P * 0.003f;
            needsRetry = true;
            logFn("  -> Adding I for steady-state error");
        }

        if (finalAngle < testAngle * 0.7f) {
            // Barely moved — need more P (caster friction)
            g.P *= 1.5f;
            needsRetry = true;
            logFn("  -> Increasing P (insufficient motion)");
        }

        if (!needsRetry) {
            // If there's any steady-state error, add a small I
            if (steadyErr > 1.0f && g.I == 0.0f) {
                g.I = g.P * 0.002f;
            }
            break;
        }

        // Return to starting heading before next attempt
        resetDeg();
        delay(300);
    }

    // Sanity bounds
    g.P = constrain(g.P, 0.5f, 10.0f);
    g.I = constrain(g.I, 0.0f, 0.05f);
    g.D = constrain(g.D, 0.0f, 8.0f);

    return g;
}


// ═══════════════════════════════════════════════════════════════════
//  Phase 5: Tune position PID
//
//  With velocity + rotation loops running, applies a short
//  forward step and iteratively adjusts position gains.
// ═══════════════════════════════════════════════════════════════════
static PIDGains tunePosition(std::function<void(const String&)> logFn)
{
    PIDGains g;

    // Starting point for tick-based position PID at 200Hz
    // Error is in encoder counts, output is velocity target (RPM)
    // FIX: Raised P from 0.10 to 0.20. At P=0.10, the position PID
    // outputs only ~13.5 RPM as a velocity target for 10cm of error.
    // The velocity PID then needs ~1.3s of integral accumulation to
    // build enough command to overcome the motor dead zone at such a
    // low target. At P=0.20, the velocity target is ~27 RPM, halving
    // the time to overcome the dead zone.
    g.P = 0.20f;
    g.I = 0.0f;
    g.D = 1.0f;

    float testDistCm = 10.0f;
    float testDistCounts = testDistCm * COUNTS_PER_CM;

    motors::rotationPID.setEnabled(true);
    motors::positionPID.setEnabled(true);

    // Cap speed during calibration
    motors::positionPID.setOutputBounds(-CALIBRATION_RPM, CALIBRATION_RPM);

    for (int attempt = 0; attempt < 5; attempt++) {
        motors::positionPID.setPID(g.P, g.I, g.D);
        logFn("  Pos test #" + String(attempt+1) + ": P=" + String(g.P,4) +
              " I=" + String(g.I,5) + " D=" + String(g.D,3));

        // Set targets: forward 10cm, heading 0°
        resetDeg();
        float startPos = readAvgPosition();
        motors::rotationPID.setTarget(0.0f);
        motors::positionPID.setTarget(startPos + testDistCounts);
        // Re-enable velocity PIDs — prior attempt completion may have disabled them.
        motors::rightVelocityPID.setEnabled(true);
        motors::leftVelocityPID.setEnabled(true);
        motors::isInAction = true;
        motors::performingTurn = false;

        // Run for 2.5 seconds
        float peakPos = 0;
        float finalPos = 0;
        uint32_t start = millis();

        while (millis() - start < 2500) {
            motors::tick();   // handles all PID timing + cached encoder update
            float pos = readAvgPosition() - startPos;
            if (pos > peakPos) peakPos = pos;
            finalPos = pos;
            yield();
        }

        motors::stop();
        motors::isInAction = false;
        delay(300);

        float targetCounts = testDistCounts;
        float overshootCounts = peakPos - targetCounts;
        float overshootCm = overshootCounts / COUNTS_PER_CM;
        float steadyErrCm = fabsf(finalPos - targetCounts) / COUNTS_PER_CM;

        logFn("  Result: peak=" + String(peakPos/COUNTS_PER_CM,1) + "cm final=" +
              String(finalPos/COUNTS_PER_CM,1) + "cm overshoot=" +
              String(overshootCm,1) + "cm err=" + String(steadyErrCm,1) + "cm");

        bool needsRetry = false;

        if (overshootCm > 3.0f) {
            g.D *= 1.4f;
            g.P *= 0.8f;
            needsRetry = true;
            logFn("  -> Reducing P, increasing D");
        } else if (overshootCm > 1.0f) {
            g.D *= 1.2f;
            needsRetry = true;
            logFn("  -> Increasing D slightly");
        }

        if (steadyErrCm > 1.0f && overshootCm < 2.0f) {
            g.I = g.P * 0.02f;
            needsRetry = true;
            logFn("  -> Adding I for position error");
        }

        if (finalPos < targetCounts * 0.7f) {
            g.P *= 1.5f;
            needsRetry = true;
            logFn("  -> Increasing P (barely moved)");
        }

        if (!needsRetry) {
            if (steadyErrCm > 0.5f && g.I == 0.0f) {
                g.I = g.P * 0.01f;
            }
            break;
        }
    }

    g.P = constrain(g.P, 0.01f, 1.0f);
    g.I = constrain(g.I, 0.0f, 0.02f);
    g.D = constrain(g.D, 0.0f, 5.0f);

    return g;
}


// ═══════════════════════════════════════════════════════════════════
//  Helpers
// ═══════════════════════════════════════════════════════════════════
static void disableAllPIDs() {
    motors::rotationPID.setEnabled(false);
    motors::positionPID.setEnabled(false);
    motors::rightVelocityPID.setEnabled(false);
    motors::leftVelocityPID.setEnabled(false);
    motors::stop();
}

static void enableAllPIDs() {
    motors::rotationPID.setEnabled(true);
    motors::positionPID.setEnabled(true);
    motors::rightVelocityPID.setEnabled(true);
    motors::leftVelocityPID.setEnabled(true);
}


// ═══════════════════════════════════════════════════════════════════
//  Main calibration entry point
// ═══════════════════════════════════════════════════════════════════
CalibrationResult runCalibration(std::function<void(const String&)> logFn)
{
    CalibrationResult result = {};
    result.success = false;

    logFn("═══ CALIBRATION START ═══");
    logFn("Place robot on ground, 30cm clear ahead.");
    delay(1000);

    // ── Phase 1: Motor characterization ──────────────────────────
    logFn("Phase 1: Motor step response...");
    disableAllPIDs();

    // FIX: Start with a moderate command. On the ground, friction/load
    // means PWM 120 (stepCmd=20) may not produce enough RPM for the
    // EMA-filtered readRPM() to converge within the sampling window.
    // Try escalating step commands until we get a valid response.
    MotorModel model = {0, 0, 0, false};
    int stepCmds[] = {40, 60, 80};  // PWM becomes 140, 160, 180 after dead-zone offset
    for (int i = 0; i < 3; i++) {
        logFn("  Trying stepCmd=" + String(stepCmds[i]) +
              " (PWM " + String(stepCmds[i] + MOTOR_PWM_MIN) + ")...");
        model = characterizeMotors(stepCmds[i], logFn);
        if (model.valid) break;
        logFn("  Motors too slow at this level, increasing...");
        delay(300);
    }

    if (!model.valid) {
        logFn("CALIBRATION FAILED: Motors unresponsive at all step levels.");
        logFn("Check: wiring, STBY pin, motor driver, wheels on ground?");
        return result;
    }

    result.plantK_left  = model.K;  // approximate (we measured average)
    result.plantK_right = model.K;
    result.plantTau_left  = model.tau_ms;
    result.plantTau_right = model.tau_ms;

    // ── Phase 2: Compute velocity gains ──────────────────────────
    logFn("Phase 2: Computing velocity PID...");

    float lambdaFactor = 1.0f;  // aggressive: lambda = tau (tightest stable IMC)
    PIDGains velGains = computeVelocityGains(model.K, model.tau_ms, lambdaFactor);

    logFn("  Initial: P=" + String(velGains.P,4) + " I=" + String(velGains.I,5) + " D=" + String(velGains.D,4));

    // ── Phase 3: Verify velocity with step test ──────────────────
    logFn("Phase 3: Velocity verification...");

    float testRPM = model.ss_rpm * 0.5f;  // target at 50% of measured max
    if (testRPM < 40.0f) testRPM = 40.0f;
    if (testRPM > CALIBRATION_RPM) testRPM = CALIBRATION_RPM;

    for (int retry = 0; retry < 4; retry++) {
        float result = velocityStepTest(testRPM, velGains, logFn);

        if (result >= 0.0f && result < 25.0f) {
            logFn("  Velocity PID: OK");
            break;
        }

        if (result < 0.0f) {
            // Under-target: controller too weak to reach setpoint.
            // Need MORE aggressive gains (smaller lambda), not less.
            lambdaFactor *= 0.55f;
            if (lambdaFactor < 0.3f) lambdaFactor = 0.3f;
            velGains = computeVelocityGains(model.K, model.tau_ms, lambdaFactor);
            // Also bump P directly to help overcome dead zone faster
            velGains.P *= 1.5f;
            velGains.P = constrain(velGains.P, 0.05f, 5.0f);
            logFn("  Under-target — more aggressive: lambda=" + String(lambdaFactor,1) +
                  " P=" + String(velGains.P,4) + " I=" + String(velGains.I,5));
        } else {
            // Too much overshoot — make more conservative
            lambdaFactor *= 1.5f;
            velGains = computeVelocityGains(model.K, model.tau_ms, lambdaFactor);
            logFn("  Overshoot — more conservative: lambda=" + String(lambdaFactor,1) +
                  " P=" + String(velGains.P,4) + " I=" + String(velGains.I,5));
        }
    }

    // Apply final velocity gains
    motors::rightVelocityPID.setPID(velGains.P, velGains.I, velGains.D);
    motors::leftVelocityPID.setPID(velGains.P, velGains.I, velGains.D);
    motors::rightVelocityPID.setEnabled(true);
    motors::leftVelocityPID.setEnabled(true);

    result.vel_P = velGains.P;
    result.vel_I = velGains.I;
    result.vel_D = velGains.D;

    delay(500);

    // ── Phase 4: Rotation PID tuning ─────────────────────────────
    logFn("Phase 4: Rotation PID tuning...");
    PIDGains rotGains = tuneRotation(logFn);
    motors::rotationPID.setPID(rotGains.P, rotGains.I, rotGains.D);

    logFn("  Final rotation: P=" + String(rotGains.P,3) + " I=" +
          String(rotGains.I,4) + " D=" + String(rotGains.D,3));

    result.rot_P = rotGains.P;
    result.rot_I = rotGains.I;
    result.rot_D = rotGains.D;

    delay(500);

    // ── Phase 5: Position PID tuning ─────────────────────────────
    logFn("Phase 5: Position PID tuning...");
    PIDGains posGains = tunePosition(logFn);
    motors::positionPID.setPID(posGains.P, posGains.I, posGains.D);

    logFn("  Final position: P=" + String(posGains.P,4) + " I=" +
          String(posGains.I,5) + " D=" + String(posGains.D,3));

    result.pos_P = posGains.P;
    result.pos_I = posGains.I;
    result.pos_D = posGains.D;

    delay(500);

    // ── Phase 6: Final verification ──────────────────────────────
    logFn("Phase 6: Verification move (18cm + 90° turn)...");
    enableAllPIDs();

    // Restore normal speed limits
    motors::positionPID.setOutputBounds(-MAX_VELOCITY_RPM, MAX_VELOCITY_RPM);
    motors::rotationPID.setOutputBounds(-MAX_TURN_RATE, MAX_TURN_RATE);

    // Forward one cell
    motors::setTargetPosition(CELL_SIZE_CM);
    uint32_t timeout = millis();
    while (motors::isInAction && millis() - timeout < 4000) {
        motors::tick();
        yield();
    }

    delay(300);

    // Turn 90°
    motors::setTargetRotation(90.0f);
    timeout = millis();
    while (motors::isInAction && millis() - timeout < 4000) {
        motors::tick();
        yield();
    }

    motors::stop();

    // ── Report ───────────────────────────────────────────────────
    logFn("═══ CALIBRATION COMPLETE ═══");
    logFn("Copy these into motors.cpp:");
    logFn("  float motor_vel_P  = " + String(result.vel_P, 6) + "f;");
    logFn("  float motor_vel_I  = " + String(result.vel_I, 6) + "f;");
    logFn("  float motor_vel_D  = " + String(result.vel_D, 6) + "f;");
    logFn("  float motor_turn_P = " + String(result.rot_P, 6) + "f;");
    logFn("  float motor_turn_I = " + String(result.rot_I, 6) + "f;");
    logFn("  float motor_turn_D = " + String(result.rot_D, 6) + "f;");
    logFn("  float motor_pos_P  = " + String(result.pos_P, 6) + "f;");
    logFn("  float motor_pos_I  = " + String(result.pos_I, 6) + "f;");
    logFn("  float motor_pos_D  = " + String(result.pos_D, 6) + "f;");
    logFn("Plant: K=" + String(model.K,4) + " RPM/cmd, tau=" + String(model.tau_ms,1) + "ms");

    result.success = true;
    return result;
}
