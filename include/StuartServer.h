#pragma once
// ═══════════════════════════════════════════════════════════════════════════
//  StuartServer.h  —  Stuart Robot HTTP Server
//  Framework:  Arduino
//              mathieucarbou/AsyncTCP          @ ^3.2.4
//              mathieucarbou/ESPAsyncWebServer @ ^3.3.17
//  No ArduinoJson dependency — all JSON is built manually.
//
//  Usage:
//      StuartServer server;              // global
//      server.init(80);                  // in setup()
//      server.onStart(myStartFn);        // assign handlers
//      server.onData([]() -> TelemetrySnapshot { ... });
//      // WiFi.begin(...) / wait for WL_CONNECTED  somewhere before:
//      server.begin();
//      // Anywhere after begin():
//      server.log("hello browser");
// ═══════════════════════════════════════════════════════════════════════════

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <map>
#include <vector>
#include <utility>
#include <functional>

// ── PID snapshot ─────────────────────────────────────────────────────────────
struct PidSnapshot {
    float actual = 0.0f;   // getFeedback()
    float target = 0.0f;   // getTarget()
    float error  = 0.0f;   // getError()
    float kp     = 0.0f;   // getP()
    float ki     = 0.0f;   // getI()
    float kd     = 0.0f;   // getD()
};

// ── Full telemetry snapshot ───────────────────────────────────────────────────
struct TelemetrySnapshot {
    float     heading       = 0.0f;   // degrees
    float     position      = 0.0f;   // encoder counts (absolute)
    int32_t   encoder_left  = 0;
    int32_t   encoder_right = 0;
    int32_t   tof_l         = 0;      // mm
    int32_t   tof_r         = 0;      // mm
    int32_t   tof_f         = 0;      // mm
    PidSnapshot pos_pid;
    PidSnapshot rot_pid;
    PidSnapshot vel_r_pid;
    PidSnapshot vel_l_pid;
};

// ── Maze graph type ───────────────────────────────────────────────────────────
//  Adjacency list: node → list of neighbours (each passage stored in both dirs)
using MazeGraph = std::map<std::pair<int,int>, std::vector<std::pair<int,int>>>;

// ── Callback types ────────────────────────────────────────────────────────────
using VoidHandler    = std::function<void()>;
using IntHandler     = std::function<void(int)>;
using PidHandler     = std::function<void(const String&, float, float, float)>;
using InstrHandler   = std::function<void(const String&)>;
using DataProvider   = std::function<TelemetrySnapshot()>;
using MazeProvider   = std::function<MazeGraph()>;


// ─────────────────────────────────────────────────────────────────────────────
class StuartServer {
public:
    // ── Lifecycle ─────────────────────────────────────────────────────────────
    void init(uint16_t port = 80);
    void begin();

    // ── Handler assignment (after init(), before begin()) ─────────────────────
    void onData        (DataProvider   fn);   // GET  /data
    void onMaze        (MazeProvider   fn);   // GET  /maze
    void onStart       (VoidHandler    fn);   // POST /start
    void onStop        (VoidHandler    fn);   // POST /stop
    void onRestart     (VoidHandler    fn);   // POST /restart
    void onZero        (VoidHandler    fn);   // POST /zero
    void onPosition    (IntHandler     fn);   // POST /position  (val=N)
    void onRotation    (IntHandler     fn);   // POST /rotation  (val=N)
    void onPid         (PidHandler     fn);   // POST /pid       (pid,p,i,d)
    void onInstructions(InstrHandler   fn);   // POST /instructions (seq)

    // ── Static JSON builders (exposed for testing / reuse) ────────────────────
    static String telemetryToJson(const TelemetrySnapshot& snap);
    static String mazeToJson(const MazeGraph& graph, int width = 8, int height = 8);

    // ── Log streaming (SSE → browser log panel) ───────────────────────────────
    //  Call from any task after begin(). Thread-safe — ESPAsyncWebServer
    //  buffers the send internally.
    void log(const String& msg);

private:
    AsyncWebServer*  _server    = nullptr;
    AsyncEventSource _logEvents{"/log"};   // SSE endpoint

    DataProvider   _dataFn;
    MazeProvider   _mazeFn;
    VoidHandler    _startFn;
    VoidHandler    _stopFn;
    VoidHandler    _restartFn;
    VoidHandler    _zeroFn;
    IntHandler     _positionFn;
    IntHandler     _rotationFn;
    PidHandler     _pidFn;
    InstrHandler   _instructionsFn;

    // Helpers
    static void    _cors(AsyncWebServerResponse* r);
    static String  _param(AsyncWebServerRequest* req, const char* key);
    void           _send200(AsyncWebServerRequest* req,
                            const String& body,
                            const char* ct = "text/plain");
    void           _registerRoutes();
};