#pragma once
#include <Arduino.h>
#include <functional>

// ═══════════════════════════════════════════════════════════════════
//  Auto-tune calibration
//
//  Runs a ~15-second sequence that:
//   1. Measures motor step response (plant gain K, time constant τ)
//   2. Computes velocity PID gains from the measured model
//   3. Verifies with a closed-loop velocity step test
//   4. Tunes rotation PID with a small-angle step test
//   5. Tunes position PID with a short-distance step test
//   6. Runs a final verification move
//
//  Call from Core 1 (main loop). The function blocks until done.
//  Put the robot on the ground with 30+ cm of clear space ahead.
//
//  logFn: called with status messages (wire to robotServer.log)
// ═══════════════════════════════════════════════════════════════════

struct CalibrationResult {
    // Velocity PID
    float vel_P, vel_I, vel_D;
    // Rotation PID
    float rot_P, rot_I, rot_D;
    // Position PID
    float pos_P, pos_I, pos_D;
    // Plant model
    float plantK_left;    // RPM per PWM-command unit (left motor)
    float plantK_right;   // RPM per PWM-command unit (right motor)
    float plantTau_left;  // time constant in ms (left motor)
    float plantTau_right; // time constant in ms (right motor)
    bool  success;
};

CalibrationResult runCalibration(std::function<void(const String&)> logFn);
