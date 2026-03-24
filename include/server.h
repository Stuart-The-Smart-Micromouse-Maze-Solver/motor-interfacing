#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "PID.h"
#include <freertos/semphr.h>

// Push rate for SSE telemetry frames. 10 Hz matches your previous poll rate.
// Lower = less CPU; raise to 20 if you want smoother graphs.
static constexpr int TELEMETRY_HZ  = 10;

// Depth of the on-device circular log ring (number of lines).
static constexpr int LOG_RING_SIZE = 64;


class RobotServer {
public:
    RobotServer(PIDController<float>* turn, PIDController<float>* pos,
                PIDController<float>* rVel, PIDController<float>* lVel);

    // Call from your existing xTaskCreatePinnedToCore lambda, same as before.
    // Your distanceUpdateAll() / gyroCache() loop comes right after — unchanged.
    void begin(const char* ssid, const char* password,
               void (*startFunc)(),
               void (*stopFunc)(),
               void (*restartFunc)(),
               void (*posFunc)(int),
               void (*turnFunc)(float),
               bool (*execFunc)(const String&) = nullptr,
               void (*calibrateFunc)() = nullptr);

    // Thread-safe: callable from Core 1 / motors task / any task.
    void log(const String& msg);

    // Called by the internal SSE_Push sub-task. Public so the lambda can reach it.
    void pushTelemetry();

private:
    AsyncWebServer   _server;
    AsyncEventSource _events;   // mounted at /events

    PIDController<float>* _turn;
    PIDController<float>* _pos;
    PIDController<float>* _rVel;
    PIDController<float>* _lVel;

    void (*_startCallback)()     = nullptr;
    void (*_stopCallback)()      = nullptr;
    void (*_restartCallback)()   = nullptr;
    void (*_posCallback)(int)    = nullptr;
    void (*_turnCallback)(float) = nullptr;
    bool (*_execCallback)(const String&) = nullptr;
    void (*_calibrateCallback)() = nullptr;

    SemaphoreHandle_t _logMutex  = nullptr;
    String  _logBuffer[LOG_RING_SIZE];
    uint8_t _logHead             = 0;
    uint8_t _logCount            = 0;
};