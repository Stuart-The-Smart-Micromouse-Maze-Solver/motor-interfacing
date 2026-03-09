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


// Empirically determined constants: 

// const float COUNTS_PER_CM = 33.3f; 
const float COUNTS_PER_CM = 13.5f; 
const float CM_PER_COUNT = 0.302f; //this was 0.33056 but changed to 0.2667
const int COUNTS_PER_CELL = 243; //
const int SPR = 140; // counts per wheel revolution

const float WHEEL_DIAMETER_CM = 3.34f;  // measured
const float WHEEL_CIRCUMFERENCE_CM = WHEEL_DIAMETER_CM * 3.14159f;


const float WHEEL_BASE_DISTANCE = 6.7;  // slightly measured? but adjusted?? actually measured 8cm
const float COUNTS_OFFSET_PER_DEG = (1 / COUNTS_PER_CM) * (2 / WHEEL_BASE_DISTANCE) * (180 / 3.1416f);  // I think this is right??? idk

const float SIDE_TOF_TO_WALL_CM = 0.0f;
const float FRONT_TOF_TO_WALL_CM = 0.0f;



const int MOTOR_PWM_MIN = 100;
const int MOTOR_PWM_MAX = 255;
const int MOTOR_ACTIVE_PWM_RANGE = MOTOR_PWM_MAX - MOTOR_PWM_MIN;