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
    int frontMM = getDistanceFront();
    if (frontMM <= 0) return;                       // no valid reading

    float frontCm = frontMM / 10.0f;
    if (frontCm > FRONT_CORR_MAX_RANGE_CM) return;  // too far to trust

    float currentPosCounts = readAvgPosition();
    float remainingCm = (activeForwardBaseTarget - currentPosCounts) / COUNTS_PER_CM;

    // Only allow front-wall correction inside roughly the FINAL cell of travel.
    // The previous version could start pulling the target backward too early,
    // which is why F7 was consistently stopping near cell 6 instead of cell 7.
    if (remainingCm < -1.0f || remainingCm > (CELL_SIZE_CM + 2.0f)) return;

    // Find the nearest valid grid stop position.
    float distFromBase = frontCm - FRONT_TOF_TO_WALL_CM;
    if (distFromBase < -CELL_SIZE_CM * 0.25f) return;

    int nearestN = max(0, (int)roundf(distFromBase / CELL_SIZE_CM));
    float idealCm = FRONT_TOF_TO_WALL_CM + nearestN * CELL_SIZE_CM;
    float errorCm = frontCm - idealCm;

    if (fabsf(errorCm) < FRONT_CORR_DEADBAND_CM) return;
    if (fabsf(errorCm) > FRONT_CORR_SNAP_TOL_CM) return;

    float confidence;
    if (nearestN == 0)      confidence = 1.0f;
    else if (nearestN == 1) confidence = 0.5f;
    else                    confidence = 0.25f;

    float desiredTarget = currentPosCounts + errorCm * confidence * COUNTS_PER_CM;

    // Never let the front sensor rewrite the move by an entire half-cell.
    // Encoder distance remains the primary source of truth; the ToF is only a
    // small final trim to remove residual count/calibration error.
    const float MAX_ADJUST_COUNTS = 2.0f * COUNTS_PER_CM;  // ±2 cm trim only
    desiredTarget = constrain(desiredTarget,
                              activeForwardBaseTarget - MAX_ADJUST_COUNTS,
                              activeForwardBaseTarget + MAX_ADJUST_COUNTS);

    float currentTarget = motors::positionPID.getTarget();
    motors::positionPID.setTarget(currentTarget + FRONT_CORR_ALPHA * (desiredTarget - currentTarget));
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
            motors::setTargetRotationCentered(p.value);
            break;

        case PRIM_WAIT:
            Serial.printf("[Motion] Wait %.0f ms\n", p.value);
            activeForwardBaseTarget = readAvgPosition();
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
        // Current move/turn completed
        currentPrimIsForward = false;
        primActive = false;  // triggers next primitive on next call
    } else if (currentPrimIsForward) {
        // Emergency collision abort — use raw (unfiltered) front reading to
        // bypass the EMA lag (~150ms) that delays detection at speed.
        // Threshold must be well below FRONT_TOF_TO_WALL_CM (35mm) to avoid
        // spurious aborts during normal planned stops at the cell boundary.
        int frontMM = getDistanceFrontRaw();
        if (frontMM > 0 && frontMM <= 20) {
            Serial.println("[Motion] Collision abort: front ToF <= 20mm");
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
    motors::isInAction = false;
    motors::performingTurn = false;
    motors::positionPID.setTarget(readAvgPosition());
    motors::rotationPID.setTarget(0.0f);
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
