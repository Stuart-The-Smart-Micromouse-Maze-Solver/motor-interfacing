#include <Wire.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_Sensor.h>
#include "config.h"
#include "gyro_heading.h"

static Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);

static bool     gyroValid     = false;
static float    integratedDeg = 0.0f;
static float    zeroOffsetDeg = 0.0f;
static float    gyroBiasZ     = 0.0f;
static uint32_t lastCacheUs   = 0;

static volatile float cachedGz   = 0.0f;
static volatile bool  cacheReady = false;
static volatile bool pendingReset = false;


extern TwoWire I2CBus1;


bool gyroInit()
{
    if (!gyro.begin(33U, &I2CBus1)) {
        gyroValid = false;
        return false;
    }
    gyro.setRange(GYRO_RANGE_250DPS);
    gyroValid     = true;
    integratedDeg = 0.0f;
    zeroOffsetDeg = 0.0f;
    gyroBiasZ     = 0.0f;
    cacheReady    = false;
    lastCacheUs   = micros();
    return true;
}

void gyroQuickBiasCal(uint16_t samples)
{
    if (!gyroValid) return;
    float sum = 0.0f;
    for (uint16_t i = 0; i < samples; i++) {
        sensors_event_t g;
        if (gyro.getEvent(&g)) sum += g.gyro.z;
        delay(2);
    }
    gyroBiasZ     = sum / (float)samples;
    integratedDeg = 0.0f;
    zeroOffsetDeg = 0.0f;
}

// ─────────────────────────────────────────────────────────────────────────────
//  gyroCache() — Core 0, ~100 Hz
//  Only place integration happens. dt is always fresh here.
// ─────────────────────────────────────────────────────────────────────────────
void gyroCache()
{
    if (!gyroValid) return;

    if (pendingReset) {
        integratedDeg = 0.0f;
        zeroOffsetDeg = 0.0f;
        pendingReset = false;
    }

    sensors_event_t g;
    if (!gyro.getEvent(&g)) return;

    float gz = g.gyro.z - gyroBiasZ;
    cachedGz = gz;

    uint32_t now = micros();
    if (!cacheReady) {
        lastCacheUs = now;
        cacheReady  = true;
        return;
    }

    float dt = (float)(now - lastCacheUs) * 1e-6f;
    lastCacheUs = now;

    if (dt <= 0.0f || dt > 0.05f) return;
    if (fabsf(gz) < 0.005f) return;   // ~0.29 deg/s noise floor

    integratedDeg += gz * 57.2957795f * dt;
}

void gyroUpdate() {}  // stub — integration moved to gyroCache()

float readDeg()
{
    return integratedDeg - zeroOffsetDeg;
}

void resetDeg()
{
    // Re-anchor to exactly 0 each time, not just shift the offset.
    // Prevents integratedDeg from growing into the thousands over many turns,
    // which would erode float32 precision (7 decimal digits).
    pendingReset = true;
}


// ─────────────────────────────────────────────────────────────────────────────
//  correctGyroDriftFromWalls()
//
//  Nudges zeroOffsetDeg so that readDeg() drifts toward the wall-implied heading.
//  Call from motors.cpp readTurn() during straight moves only.
//  readTurn() must NOT add any separate offset on top — this function is the
//  sole place wall data influences heading. readDeg() reflects it automatically.
//
//  Supports three cases:
//
//  CASE A — Both walls visible (most reliable):
//    The difference (d_R - d_L) estimates heading error independent of lateral
//    position. weight = 1.0 (full confidence). This is the symmetric case.
//
//  CASE B — One wall visible (single-wall):
//    We know the expected distance to that wall (SIDE_TOF_TO_WALL_MM + n*180mm).
//    We find the nearest multiple and compute the error to it.
//    The error is primarily lateral offset, not heading directly, so the heading
//    contribution is weaker — weight is reduced by half.
//    We still apply it because even a weak signal beats nothing.
//
//  CASE C — Neither wall visible:
//    No correction applied.
//
//  Distance-weighting for multi-cell walls:
//    A sensor reading at distance d has angular sensitivity ∝ d (further = more
//    mm per degree), but VL53L4CX ranging noise grows roughly linearly with d.
//    The two effects roughly cancel, but we apply a mild 1/d weight anyway to
//    favour close walls slightly over far ones. Normalised to 1.0 at the nominal
//    single-cell distance (SIDE_TOF_TO_WALL_MM).
//
//  alpha: overall correction rate (0.0–1.0).
//    0.10 = default during a straight move (tune upward for faster correction)
//    1.0  = hard reset while stationary
// ─────────────────────────────────────────────────────────────────────────────
void correctGyroDriftFromWalls(float distLeftMM, float distRightMM, float alpha)
{
    if (!gyroValid) return;

    const float CELL_MM      = 180.0f;
    const float NOMINAL_MM   = SIDE_TOF_TO_WALL_CM * 10.0f;  // expected distance, 1st cell
    const float MAX_VALID_MM = NOMINAL_MM + 2.0f * CELL_MM + 30.0f; // up to 2 cells away + margin
    const float MIN_VALID_MM = 5.0f;                          // below this = surface reflection

    // ── Classify each sensor ─────────────────────────────────────────────────
    // "Valid" means the reading falls within a plausible multiple of CELL_MM from
    // the nominal distance. We find the nearest expected distance and check the
    // residual is small enough to trust.
    const float SNAP_TOLERANCE_MM = 25.0f;  // ±25mm from a cell multiple is "valid"

    auto nearestCellDist = [&](float d) -> float {
        // Returns the nearest expected distance (NOMINAL + n*CELL for n=0,1,2,...)
        // or -1 if none is within tolerance.
        if (d < MIN_VALID_MM || d > MAX_VALID_MM) return -1.0f;
        float offset = d - NOMINAL_MM;
        float n      = roundf(offset / CELL_MM);
        if (n < 0.0f) n = 0.0f;
        float expected = NOMINAL_MM + n * CELL_MM;
        float residual = fabsf(d - expected);
        return (residual < SNAP_TOLERANCE_MM) ? expected : -1.0f;
    };

    float expectedLeft  = nearestCellDist(distLeftMM);
    float expectedRight = nearestCellDist(distRightMM);

    bool leftValid  = (expectedLeft  > 0.0f);
    bool rightValid = (expectedRight > 0.0f);

    if (!leftValid && !rightValid) return;  // Case C — nothing to do

    float wallAngleErr = 0.0f;
    float weight       = 0.0f;

    if (leftValid && rightValid) {
        // ── Case A: both walls ────────────────────────────────────────────────
        // Heading angle θ ≈ atan((d_R_err - d_L_err) / corridor_width).
        // For small angles: θ_deg ≈ (d_R_err - d_L_err) / corridor_width * (180/π).
        // d_R_err = distRight - expectedRight  (positive = nose pointing left)
        // d_L_err = distLeft  - expectedLeft   (positive = nose pointing right)
        // Net: (d_R_err - d_L_err) / corridor gives signed heading error.
        float errRight = distRightMM - expectedRight;
        float errLeft  = distLeftMM  - expectedLeft;

        // corridor_width ≈ 2 * NOMINAL_MM (sensor to wall each side).
        // Using the full measured sum for robustness.
        float corridorMM = distLeftMM + distRightMM;  // total gap between walls
        if (corridorMM < 10.0f) return;               // degenerate

        // small-angle: θ (deg) = (errRight - errLeft) / corridorMM * 57.296
        wallAngleErr = (errRight - errLeft) / corridorMM * 57.2957795f;

        // Distance weight: average of the two walls, normalised to nominal.
        // Mild 1/d falloff — close walls trusted slightly more.
        float avgDist = (distLeftMM + distRightMM) * 0.5f;
        weight = NOMINAL_MM / avgDist;   // 1.0 at nominal, <1.0 further away

    } else {
        // ── Case B: single wall ───────────────────────────────────────────────
        // We know expected distance but not the other wall. The error from expected
        // is mostly lateral offset — heading contribution is indirect and weaker.
        // We apply at half weight compared to the two-wall case.
        float dist     = leftValid  ? distLeftMM  : distRightMM;
        float expected = leftValid  ? expectedLeft : expectedRight;
        float sign     = leftValid  ? -1.0f : 1.0f;  // left err > 0 → nose right → negative correction

        float lateralErrMM = dist - expected;

        // Approximate heading from lateral error: assume the error is pure lateral
        // and project back through the corridor half-width.
        float halfCorridor = expected;   // expected ≈ sensor-to-wall distance
        if (halfCorridor < 10.0f) return;

        wallAngleErr = sign * (lateralErrMM / halfCorridor) * 57.2957795f * 0.5f;
            //                                                               ^^^
            //                             half weight: lateral offset ≠ pure heading

        // Distance weight: same 1/d normalisation, but also halved for single-wall
        weight = (NOMINAL_MM / dist) * 0.5f;
    }

    /*
    // ── Sanity gate: reject implausible corrections ───────────────────────────
    // Max plausible heading error inside a corridor is about 15°.
    if (fabsf(wallAngleErr) > 15.0f) return;

    // ── Apply weighted correction to zeroOffsetDeg ───────────────────────────
    // We want readDeg() → wallAngleErr.
    // readDeg() = integratedDeg - zeroOffsetDeg
    // target zeroOffsetDeg = integratedDeg - wallAngleErr
    // lerp: zeroOffsetDeg += (alpha * weight) * (currentAngle - wallAngleErr)
    float effectiveAlpha = alpha * weight;
    float currentAngle   = integratedDeg - zeroOffsetDeg;
    zeroOffsetDeg += effectiveAlpha * (currentAngle - wallAngleErr);
    */
    // ── Sanity gate ───────────────────────────────────────────────────────────
    if (fabsf(wallAngleErr) > 15.0f) return;

    // ── Deadband: ignore tiny corrections — they're just sensor noise ─────────
    // Below this threshold, don't touch the gyro at all.
    // This prevents the correction from constantly hunting when already aligned.
    const float DEADBAND_DEG = 0.5f;   // tune: ~0.3–1.0° is a good range
    if (fabsf(wallAngleErr) < DEADBAND_DEG) return;

    // ── Proportional alpha: correction strength scales with how wrong we are ──
    // Small errors → gentle nudge. Large errors → stronger pull.
    // MAX_CORRECTION_DEG is the error at which alpha reaches its full value.
    // Below that it scales linearly down toward zero at the deadband edge.
    const float MAX_CORRECTION_DEG = 5.0f;   // tune: error magnitude for full alpha
    float errorScale = fabsf(wallAngleErr) / MAX_CORRECTION_DEG;
    errorScale = constrain(errorScale, 0.0f, 1.0f);   // clamp at 1.0 for large errors

    float effectiveAlpha = alpha * weight * errorScale;

    float currentAngle = integratedDeg - zeroOffsetDeg;
    zeroOffsetDeg += effectiveAlpha * (currentAngle - wallAngleErr);
}