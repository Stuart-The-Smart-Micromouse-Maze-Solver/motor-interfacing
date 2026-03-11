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

constexpr int RGB_LED_PIN = 38; // for state viewing


// Wifi Hotspot 
#define WIFI_SSID "STUART"
#define WIFI_PWD "micromouse"


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

const float SIDE_TOF_TO_WALL_CM = 0.0f; // idk yet
const float FRONT_TOF_TO_WALL_CM = 2.0f;    // estimated for center



const int MOTOR_PWM_MIN = 100;
const int MOTOR_PWM_MAX = 255;
const int MOTOR_ACTIVE_PWM_RANGE = MOTOR_PWM_MAX - MOTOR_PWM_MIN;

const int LEFT_MOTOR_PWM_MIN  = 100;
const int RIGHT_MOTOR_PWM_MIN = 108;  // tune this experimentally
const int MOTOR_CMD_DEADBAND  = 6;

// Maze / navigation
constexpr int MAZE_SIZE = 8;
constexpr float CELL_SIZE_CM = 18.0f;
constexpr int START_ROW = 0;
constexpr int START_COL = 0;
constexpr int START_HEADING = 0; // 0=N,1=E,2=S,3=W
constexpr int WALL_PRESENT_THRESHOLD_MM = 120;
constexpr int WALL_OPEN_THRESHOLD_MM = 170;
constexpr int FRONT_BLOCK_THRESHOLD_MM = 75;
constexpr int FRONT_OBSERVE_MAX_MM = 220;
constexpr int SIDE_OBSERVE_MAX_MM = 220;
constexpr int NAV_QUEUE_LEN = 64;
constexpr int ACTION_TIMEOUT_MS = 5000;
constexpr float STALL_RPM_THRESHOLD = 5.0f;
constexpr float STALL_TARGET_RPM_MIN = 35.0f;
constexpr int STALL_KICK_BOOST = 22;
constexpr int STALL_KICK_WINDOW_MS = 180;
constexpr int STALL_ABORT_MS = 1200;
