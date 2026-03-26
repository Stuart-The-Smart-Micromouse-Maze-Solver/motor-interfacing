#include "motion.h"
#include "motors.h"
#include "config.h"
#include "encoders.h"
#include "gyro_heading.h"
#include "distance.h"

// ═══════════════════════════════════════════════════════════════════
//  Primitive command types
// ═══════════════════════════════════════════════════════════════════
enum PrimType { PRIM_FORWARD, PRIM_TURN, PRIM_WAIT };

struct Primitive {
    PrimType type;
    float    value;   // cm for forward, degrees for turn, ms for wait
};

// ═══════════════════════════════════════════════════════════════════
//  Command queue — holds expanded primitives
// ═══════════════════════════════════════════════════════════════════
static const int MAX_PRIMS = 64;
static Primitive primQueue[MAX_PRIMS];
static int primHead  = 0;
static int primTail  = 0;
static int primCount = 0;

static bool executing = false;
static bool primActive = false;
static uint32_t waitUntilMs = 0;
static bool currentPrimIsForward = false;   // tracks if active prim is a forward move
static float activeForwardBaseTarget = 0.0f;     // original target for current forward primitive

static void clearQueue() {
    primHead = primTail = primCount = 0;
    executing = false;
    primActive = false;
    waitUntilMs = 0;
    currentPrimIsForward = false;
    activeForwardBaseTarget = readAvgPosition();
}

static bool enqueue(PrimType type, float value) {
    if (primCount >= MAX_PRIMS) return false;
    primQueue[primTail] = { type, value };
    primTail = (primTail + 1) % MAX_PRIMS;
    primCount++;
    return true;
}

static bool dequeue(Primitive& out) {
    if (primCount <= 0) return false;
    out = primQueue[primHead];
    primHead = (primHead + 1) % MAX_PRIMS;
    primCount--;
    return true;
}

// ═══════════════════════════════════════════════════════════════════
//  Parser: "F3,R,F2,L,W500" → primitive queue
// ═══════════════════════════════════════════════════════════════════
static bool parseInstructions(const String& input) {
    clearQueue();

    int start = 0;
    int len = input.length();

    while (start < len) {
        // Skip separators
        while (start < len && (input[start] == ',' || input[start] == ' '))
            start++;
        if (start >= len) break;

        char cmd = toupper(input[start]);
        start++;

        // Parse optional numeric argument
        String numStr = "";
        while (start < len && isDigit(input[start])) {
            numStr += input[start];
            start++;
        }

        float arg = numStr.length() > 0 ? numStr.toFloat() : 0;

        switch (cmd) {
            case 'F': {
                // F or F3 — move forward N cells as one continuous move.
                // Previously queued as N individual 18cm moves with 50ms waits
                // between each; that caused 6 stop/start cycles on F7 which added
                // jerk and lateral drift. Continuous lateral centering handles
                // wall guidance throughout without needing intermediate stops.
                int cells = (arg > 0) ? (int)arg : 1;
                if (!enqueue(PRIM_FORWARD, cells * CELL_SIZE_CM)) return false;
                break;
            }
            case 'R': {
                // R or R90 — turn right (default 90°)
                float deg = (arg > 0) ? arg : 90.0f;
                if (!enqueue(PRIM_TURN, deg)) return false;
                break;
            }
            case 'L': {
                // L or L90 — turn left (default 90°)
                float deg = (arg > 0) ? arg : 90.0f;
                if (!enqueue(PRIM_TURN, -deg)) return false;
                break;
            }
            case 'U':
            case 'B': {
                // U-turn / turnaround
                if (!enqueue(PRIM_TURN, 180.0f)) return false;
                break;
            }
            case 'W': {
                // Wait in ms (default 200)
                float ms = (arg > 0) ? arg : 200.0f;
                if (!enqueue(PRIM_WAIT, ms)) return false;
                break;
            }
            case 'O': {
                // Observe — just a short pause for sensor reading
                if (!enqueue(PRIM_WAIT, 100)) return false;
                break;
            }
            default:
                // Unknown command — skip
                Serial.printf("[Motion] Unknown command: %c\n", cmd);
                break;
        }
    }

    return primCount > 0;
}


// ═══════════════════════════════════════════════════════════════════
//  Front ToF wall correction
//
//  During a forward move, reads the front sensor and nudges the
//  position PID target so the mouse stops on the 18 cm cell grid
//  relative to the detected front wall.
//
//  Valid stop distances from wall:
//    FRONT_TOF_TO_WALL_CM + N * CELL_SIZE_CM   (N = 0, 1, 2 …)
//
//  Closer readings get full weight; further ones are down-weighted
//  because VL53L4CX accuracy degrades with distance.
// ═══════════════════════════════════════════════════════════════════
static void applyFrontWallCorrection()
{
    // Use min(raw, filtered) to defeat EMA lag during fast approach.
    // At cruise speed (200 RPM = 35 cm/s) the EMA filter (α=0.2, 30 Hz) lags the
    // true front distance by ~4.6 cm.  Using the filtered reading alone makes the
    // correction compute an idealRemainingCm that is ~4.6 cm too large, creating a
    // moving target the robot can never catch at 40 RPM.  The robot ends up chasing
    // the target for the entire approach and stopping at 16–20 mm instead of 35 mm.
    // Raw is updated every sensor cycle (~33 ms) and has no lag; it is the right
    // signal for the correction.  Taking min(raw, filtered) means:
    //   • Approaching fast (raw < filtered due to lag) → use raw (accurate, conservative)
    //   • Receding / stable (raw ≥ filtered) → use filtered (noise-reduced)
    int frontFiltered = getDistanceFront();
    int frontRaw      = getDistanceFrontRaw();
    int frontMM = (frontRaw > 0 && frontRaw < frontFiltered) ? frontRaw : frontFiltered;
    if (frontMM <= 0) {
        // No valid reading — don't restore full-speed bounds, a mid-approach glitch
        // would send the robot charging into the wall at 200 RPM.  Keep last cap.
        return;
    }

    float frontCm = frontMM / 10.0f;
    if (frontCm > FRONT_CORR_MAX_RANGE_CM) {
        motors::positionPID.setOutputBounds(-MAX_VELOCITY_RPM, MAX_VELOCITY_RPM);
        return;  // too far to trust
    }

    float currentPosCounts = readAvgPosition();
    float remainingCm = (activeForwardBaseTarget - currentPosCounts) / COUNTS_PER_CM;

    // Activate within the final 2 cells of planned travel.
    // Gate 1 (FRONT_CORR_MAX_RANGE_CM ≈ 25 cm) is the real limiter — it prevents
    // corrections from firing while the front wall is still far away. Gate 2 just
    // needs to be wide enough to not block valid corrections for offset starts.
    // With a 2-cell window (36 cm), robots that started up to ~12 cm from the back
    // of their starting cell (e.g. center-placed, 9 cm offset → 12.5 cm overshoot)
    // have their correction fire as soon as the front wall enters sensor range,
    // rather than being rejected because the encoder still shows > 27 cm remaining.
    if (remainingCm < -(CELL_SIZE_CM * 0.6f) || remainingCm > (CELL_SIZE_CM * 2.0f)) {
        motors::positionPID.setOutputBounds(-MAX_VELOCITY_RPM, MAX_VELOCITY_RPM);
        return;
    }

    // Ideal remaining travel: exactly enough for front sensor to read FRONT_TOF_TO_WALL_CM.
    // This directly accounts for any start-position offset without needing nearestN logic.
    float idealRemainingCm = frontCm - FRONT_TOF_TO_WALL_CM;
    if (idealRemainingCm < 0.0f) idealRemainingCm = 0.0f;  // don't reverse

    // Deadband: within 3 mm of the ideal stop, stop nudging the target.
    // Without this, sensor noise keeps pushing the PID target by tiny amounts that
    // hold position error right at the 0.5 cm settle threshold, resetting the 100 ms
    // settle timer on every correction tick and preventing isInAction from clearing.
    if (idealRemainingCm < FRONT_CORR_DEADBAND_CM) {
        motors::positionPID.setOutputBounds(-MAX_VELOCITY_RPM, FRONT_MIN_APPROACH_RPM);
        return;
    }

    float newTarget = currentPosCounts + idealRemainingCm * COUNTS_PER_CM;

    // Pullback cap: prevents a spurious short reading from stopping the robot
    // more than 3/4 of a cell early.  13.5 cm covers a centre-of-cell start
    // (9 cm offset → 12.5 cm overshoot including sensor offset).
    //
    // Extension cap: 9 cm (= 0.5 cells) covers starting from cell centre where
    // the encoder target undershoots the front wall by up to 5.5 cm, and handles
    // accumulated drift up to 90 mm across a long Dijkstra path.  Without this,
    // the old 1.5 cm cap permanently blocked the correction whenever drift exceeded
    // 15 mm, causing the robot to settle far from the wall and "try to converge."
    // The collision abort at 20 mm is the hard safety backstop against overshoot.
    const float MAX_PULLBACK_CM   = CELL_SIZE_CM * 0.75f;  // 13.5 cm
    const float MAX_EXTENSION_CM  = CELL_SIZE_CM * 0.5f;   //  9.0 cm
    newTarget = constrain(newTarget,
                          activeForwardBaseTarget - MAX_PULLBACK_CM  * COUNTS_PER_CM,
                          activeForwardBaseTarget + MAX_EXTENSION_CM * COUNTS_PER_CM);

    float currentTarget = motors::positionPID.getTarget();
    motors::positionPID.setTarget(currentTarget + FRONT_CORR_ALPHA * (newTarget - currentTarget));

    // Progressive approach-speed cap. Target pullback alone is not enough when the
    // robot is still cruising near 200 RPM — it coasts ~1–2 cm past the corrected
    // target before the position loop can bleed the momentum off, which is exactly
    // the 18–19 mm wall-hit behaviour seen in the telemetry.
    //
    // Use the front-wall clearance itself as the slowdown signal:
    //   clearance = frontCm - FRONT_TOF_TO_WALL_CM
    // Far from wall  -> full speed.
    // Near 35 mm stop -> cap toward a low, non-stalling RPM.
    float clearanceCm = frontCm - FRONT_TOF_TO_WALL_CM;
    float t = clearanceCm / FRONT_SLOW_ZONE_CM;
    t = constrain(t, 0.0f, 1.0f);
    float approachCapRpm = FRONT_MIN_APPROACH_RPM +
                           t * (MAX_VELOCITY_RPM - FRONT_MIN_APPROACH_RPM);
    motors::positionPID.setOutputBounds(-MAX_VELOCITY_RPM, approachCapRpm);
}


// ═══════════════════════════════════════════════════════════════════
//  Start the next primitive from the queue
// ═══════════════════════════════════════════════════════════════════
static void startNextPrimitive() {
    Primitive p;
    if (!dequeue(p)) {
        // Queue exhausted — done
        executing = false;
        primActive = false;
        currentPrimIsForward = false;
        motors::positionPID.setOutputBounds(-MAX_VELOCITY_RPM, MAX_VELOCITY_RPM);
        motors::stop();
        Serial.println("[Motion] Sequence complete.");
        return;
    }

    primActive = true;
    currentPrimIsForward = false;

    switch (p.type) {
        case PRIM_FORWARD:
            Serial.printf("[Motion] Forward %.1f cm\n", p.value);
            currentPrimIsForward = true;
            motors::setTargetPosition(p.value);
            activeForwardBaseTarget = motors::positionPID.getTarget();
            break;

        case PRIM_TURN:
            Serial.printf("[Motion] Turn %.1f deg\n", p.value);
            activeForwardBaseTarget = readAvgPosition();
            motors::positionPID.setOutputBounds(-MAX_VELOCITY_RPM, MAX_VELOCITY_RPM);
            motors::setTargetRotationCentered(p.value);
            break;

        case PRIM_WAIT:
            Serial.printf("[Motion] Wait %.0f ms\n", p.value);
            activeForwardBaseTarget = readAvgPosition();
            motors::positionPID.setOutputBounds(-MAX_VELOCITY_RPM, MAX_VELOCITY_RPM);
            waitUntilMs = millis() + (uint32_t)p.value;
            break;
    }
}


// ═══════════════════════════════════════════════════════════════════
//  Public API
// ═══════════════════════════════════════════════════════════════════

void motionInit() {
    clearQueue();
}

bool motionExecute(const String& instructions) {
    if (executing) {
        Serial.println("[Motion] Already executing — aborting previous.");
        motionAbort();
    }

    if (!parseInstructions(instructions)) {
        Serial.println("[Motion] Parse failed or empty.");
        return false;
    }

    Serial.printf("[Motion] Parsed %d primitives. Starting...\n", primCount);
    executing = true;
    primActive = false;
    startNextPrimitive();
    return true;
}

void motionUpdate() {
    if (!executing) return;

    if (!primActive) {
        // Previous primitive finished, start next
        startNextPrimitive();
        return;
    }

    // Check if current primitive is done
    // For WAIT primitives, check timer
    if (waitUntilMs > 0) {
        if (millis() >= waitUntilMs) {
            waitUntilMs = 0;
            primActive = false;  // triggers next primitive on next call
        }
        return;
    }

    // For FORWARD and TURN, check motors::isInAction
    if (!motors::isInAction) {
        // After a turn, wait a moment before starting the next primitive.
        // This gives the chassis and side ToFs one more beat to settle.
        if (!currentPrimIsForward && waitUntilMs == 0) {
            waitUntilMs = millis() + POST_TURN_WAIT_MS;
            return;
        }

        currentPrimIsForward = false;
        primActive = false;
        
    } else if (currentPrimIsForward) {
        // Emergency collision abort — use raw (unfiltered) front reading to
        // bypass the EMA lag (~150ms) that delays detection at speed.
        // Threshold must be well below FRONT_TOF_TO_WALL_CM (35mm) to avoid
        // spurious aborts during normal planned stops at the cell boundary.
        int frontMM = getDistanceFrontRaw();
        if (frontMM > 0 && frontMM <= 20) {
            Serial.println("[Motion] Collision abort: front ToF <= 20mm");
            motors::positionPID.setOutputBounds(-MAX_VELOCITY_RPM, MAX_VELOCITY_RPM);
            motors::brake(20);   // active braking before abort
            motionAbort();
            // Snap position PID target to wherever the robot actually stopped.
            // Without this, any front-correction overshoot remains in the target
            // and the robot shows a non-zero position error when placed back in
            // the correct cell (and fights to push itself into the wall).
            motors::positionPID.setTarget(readAvgPosition());
            return;
        }
        // Rate-limit front wall correction to ~30 Hz (ToF update rate).
        // applyFrontWallCorrection() must NOT run at loop speed — it would
        // accumulate a huge target offset even though alpha looks small.
        static uint32_t lastCorrUs = 0;
        uint32_t nowUs = micros();
        if (nowUs - lastCorrUs >= 33333) {
            lastCorrUs = nowUs;
            applyFrontWallCorrection();
        }
    }
}

bool motionIsBusy() {
    return executing;
}

void motionAbort() {
    clearQueue();
    activeForwardBaseTarget = readAvgPosition();
    currentPrimIsForward = false;
    motors::cancelCenteredRotation();
    motors::positionPID.setOutputBounds(-MAX_VELOCITY_RPM, MAX_VELOCITY_RPM);
    motors::isInAction = false;
    motors::performingTurn = false;
    motors::positionPID.setTarget(readAvgPosition());
    motors::rotationPID.setTarget(0.0f);
    // Clear PID integrals so a crash doesn't corrupt the next run.
    // Without this, a large integral from fighting a wall (e.g. position PID
    // pushing forward against the abort threshold) persists into the next
    // motionExecute() call, causing the "needs restart after crash" symptom.
    motors::positionPID.setEnabled(false);
    motors::positionPID.setEnabled(true);
    motors::rotationPID.setEnabled(false);
    motors::rotationPID.setEnabled(true);
    // Disable velocity PIDs so they don't re-engage on the next tick() call.
    // motors::stop() alone only zeroes PWM; without disabling the PIDs they
    // immediately fight back to their last targets.
    motors::rightVelocityPID.setTarget(0.0f);
    motors::leftVelocityPID.setTarget(0.0f);
    motors::rightVelocityPID.setEnabled(false);
    motors::leftVelocityPID.setEnabled(false);
    motors::stop();
}

int motionQueueRemaining() {
    return executing ? (primCount + (primActive ? 1 : 0)) : 0;
}
