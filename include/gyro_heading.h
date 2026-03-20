


#pragma once
#include <stdint.h>


// delete
bool gyroIsValid();
float angleDiffDeg(float target, float current);
float gyroHeadingDeg();
// delete




bool  gyroInit();
void  gyroQuickBiasCal(uint16_t samples);

// Core 0 — I2C read + integration with fresh dt. Call at ~100 Hz.
void  gyroCache();

// Stub — kept for call-site compatibility. Does nothing.
void  gyroUpdate();

// Heading in degrees relative to last resetDeg().
float readDeg();

// Zeroes heading and re-anchors integratedDeg to 0.
// Prevents float precision loss across many turns.
void  resetDeg();

// ─────────────────────────────────────────────────────────────────────────────
//  correctGyroDriftFromWalls()
//
//  Nudges readDeg() toward the heading implied by the side ToF sensors.
//  Call from readTurn() during straight moves only.
//
//  Handles three cases automatically:
//    Both walls valid  → full-weight heading correction from (d_R - d_L)
//    One wall valid    → half-weight heading estimate from lateral error
//    Neither valid     → no correction
//
//  Multi-cell distances are supported: any reading within ±25mm of
//  (SIDE_TOF_TO_WALL_CM + n*18cm) for n=0,1,2 is accepted, with a mild
//  1/distance weight so closer walls are trusted slightly more.
//
//  Requires SIDE_TOF_TO_WALL_CM to be defined in config.h.
//
//  alpha: correction rate per call.
//    0.10 = default (tune up for faster correction, down if jerky)
//    1.0  = hard reset while stationary
// ─────────────────────────────────────────────────────────────────────────────
void  correctGyroDriftFromWalls(float distLeftMM, float distRightMM, float alpha);