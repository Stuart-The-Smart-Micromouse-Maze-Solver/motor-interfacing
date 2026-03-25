#pragma once
#include <Arduino.h>

// ================= Pins =================
constexpr int SDA2_PIN = 40;
constexpr int SCL2_PIN = 39;

// Motor driver 
constexpr int AIN1 = 11;
constexpr int AIN2 = 10;
constexpr int BIN1 = 13;
constexpr int BIN2 = 14;
constexpr int STBY = 12;

// Encoder 
constexpr int ENC_A = 3;
constexpr int ENC_B = 46;
constexpr int ENC_C = 18;
constexpr int ENC_D = 8;

// PWM Configuration
constexpr int PWM_FREQ = 20000;
constexpr int PWM_RESOLUTION = 8;

const int PWM_CH_L1 = 0; // AIN1
const int PWM_CH_L2 = 1; // AIN2
const int PWM_CH_R1 = 2; // BIN1
const int PWM_CH_R2 = 3; // BIN2

constexpr int RGB_LED_PIN = 38;

// Wifi Hotspot 
#define WIFI_SSID "superman"
#define WIFI_PWD "tooth123"

// ═══════════════════════════════════════════════════════════════════
//  Encoder / distance constants
// ═══════════════════════════════════════════════════════════════════
const float COUNTS_PER_CM = 13.5f; 
// FIX: CM_PER_COUNT must equal 1/COUNTS_PER_CM. Was 0.302 (4x too large!)
const float CM_PER_COUNT = 1.0f / COUNTS_PER_CM;  // ≈ 0.0741
const int COUNTS_PER_CELL = 243;
const int COUNTS_PER_REV = 140;

const float WHEEL_DIAMETER_CM = 3.34f;
const float WHEEL_CIRCUMFERENCE_CM = WHEEL_DIAMETER_CM * 3.14159f;
const float WHEEL_BASE_DISTANCE = 6.7f;

const float SIDE_TOF_TO_WALL_CM = 4.25f;
const float FRONT_TOF_TO_WALL_CM = 3.5f;

const int MOTOR_PWM_MIN = 100;
const int MOTOR_PWM_MAX = 255;
const int MOTOR_ACTIVE_PWM_RANGE = MOTOR_PWM_MAX - MOTOR_PWM_MIN;

// ═══════════════════════════════════════════════════════════════════
//  Speed limits — THE place to cap how fast the robot goes.
//  Adjust these first if the robot is too fast or too slow.
// ═══════════════════════════════════════════════════════════════════
const float MAX_VELOCITY_RPM = 200.0f;   // max RPM the position PID can request
                                          // 200 RPM ≈ 35 cm/s with your wheels
const float MAX_TURN_RATE    = 150.0f;   // max angular correction from rotation PID
const int   MAX_ACCEL        = 20;       // RPM change per 200Hz tick = 4000 RPM/sec ramp
const float CALIBRATION_RPM  = 200.0f;   // max RPM during auto-calibration
const int MOUSE_OFF_GROUND_ANGLE = 30;

// ═══════════════════════════════════════════════════════════════════
//  Maze constants
// ═══════════════════════════════════════════════════════════════════
const float CELL_SIZE_CM = 18.0f;
const int WALL_FRONT_THRESHOLD_MM = 120;
const int WALL_SIDE_THRESHOLD_MM  = 100;

// ═══════════════════════════════════════════════════════════════════
//  Front ToF wall correction
//
//  During forward moves the front sensor is used to snap the stop
//  position onto the 18 cm cell grid relative to the front wall.
//
//  Valid stop distances from wall: FRONT_TOF_TO_WALL_CM + N * 18cm
//  (N = 0, 1, 2 …). Corrections are blended in gently and weighted
//  by distance — close readings are trusted more than far ones.
// ═══════════════════════════════════════════════════════════════════
const float FRONT_CORR_MAX_RANGE_CM   = FRONT_TOF_TO_WALL_CM + CELL_SIZE_CM * 1.2f;  // ~25 cm — only last 1.2 cells of approach
const float FRONT_CORR_SNAP_TOL_CM    = 3.0f;   // max error from grid line to trust
const float FRONT_CORR_DEADBAND_CM    = 0.3f;    // ignore errors smaller than this
const float FRONT_CORR_ALPHA          = 0.50f;   // blend rate per tick (~87% convergence in 3 ticks)



const float WHEEL_CENTER_TO_REAL_CENTER_CM = 2.0f;  // measure physically