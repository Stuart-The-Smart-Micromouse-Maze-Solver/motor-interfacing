#pragma once
#include <Arduino.h>

// ================= Pins =================
constexpr int SDA_PIN = 17;
constexpr int SCL_PIN = 16;

// Motor driver 
constexpr int AIN1 = 37;
constexpr int AIN2 = 36;
constexpr int BIN1 = 40;
constexpr int BIN2 = 39;
constexpr int STBY = 38;

// Encoder 
constexpr int ENC_A = 21;
constexpr int ENC_B = 47;
constexpr int ENC_C = 1;
constexpr int ENC_D = 2;

// PWM Configuration
constexpr int PWM_FREQ = 20000;
constexpr int PWM_RESOLUTION = 8;
constexpr int PWM_CH_LEFT = 0;
constexpr int PWM_CH_RIGHT = 1;

// ================= Robot constants =================
constexpr float PI_F = 3.14159265358979323846f;

constexpr int   ENCODER_PPR = 7;
constexpr float GEAR_RATIO = 20.0f;
constexpr float WHEEL_DIAMETER_M = 0.032f;
constexpr float WHEEL_BASE_M = 0.103f;
constexpr float QUAD_FACTOR = 1.0f;

constexpr float COUNTS_PER_REV = GEAR_RATIO * ENCODER_PPR * QUAD_FACTOR;
constexpr float METERS_PER_COUNT = (WHEEL_DIAMETER_M * PI_F) / COUNTS_PER_REV;

// Distance calibration knob (tune once)
constexpr float DIST_SCALE = 0.9f; // Calculated by theoretical / measured
inline float metersPerCountCal() { return METERS_PER_COUNT * DIST_SCALE; }
