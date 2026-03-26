// ═══════════════════════════════════════════════════════════════════════════
//  StuartServer.cpp  —  Stuart Robot HTTP Server
//  Framework:  mathieucarbou/ESPAsyncWebServer @ ^3.3.17
//  JSON:       manual string building, no ArduinoJson
// ═══════════════════════════════════════════════════════════════════════════

#include "StuartServer.h"

// ─────────────────────────────────────────────────────────────────────────────
//  Internal helpers — JSON primitives
// ─────────────────────────────────────────────────────────────────────────────

// Append a float with 4 decimal places, stripping trailing zeros.
static String fmtFloat(float v) {
    // dtostrf gives us a fixed-width string; String(v,4) is fine on ESP32.
    char buf[24];
    dtostrf(v, 0, 4, buf);
    // Strip trailing zeros after decimal point
    char* dot = strchr(buf, '.');
    if (dot) {
        char* end = buf + strlen(buf) - 1;
        while (end > dot + 1 && *end == '0') *end-- = '\0';
    }
    return String(buf);
}

// Escape a String for use inside a JSON string value (handles " and \).
static String jsonStr(const String& s) {
    String out;
    out.reserve(s.length() + 2);
    out += '"';
    for (char c : s) {
        if      (c == '"')  out += "\\\"";
        else if (c == '\\') out += "\\\\";
        else if (c == '\n') out += "\\n";
        else if (c == '\r') out += "\\r";
        else                out += c;
    }
    out += '"';
    return out;
}

// Build a JSON object for one PID block including live gains.
// {"actual":1.2345,"target":0.0,"error":-0.1,"kp":1.0,"ki":0.0,"kd":0.05}
static String pidJson(const PidSnapshot& p) {
    String s = "{\"actual\":";
    s += fmtFloat(p.actual);
    s += ",\"target\":";
    s += fmtFloat(p.target);
    s += ",\"error\":";
    s += fmtFloat(p.error);
    s += ",\"kp\":";
    s += fmtFloat(p.kp);
    s += ",\"ki\":";
    s += fmtFloat(p.ki);
    s += ",\"kd\":";
    s += fmtFloat(p.kd);
    s += '}';
    return s;
}


// ─────────────────────────────────────────────────────────────────────────────
//  Static helpers
// ─────────────────────────────────────────────────────────────────────────────

void StuartServer::_cors(AsyncWebServerResponse* r) {
    r->addHeader("Access-Control-Allow-Origin",  "*");
    r->addHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    r->addHeader("Access-Control-Allow-Headers", "Content-Type");
}

// Look up a param in POST body first, then query string.
String StuartServer::_param(AsyncWebServerRequest* req, const char* key) {
    if (req->hasParam(key, /*isPost=*/true))  return req->getParam(key, true)->value();
    if (req->hasParam(key))                   return req->getParam(key)->value();
    return "";
}

void StuartServer::_send200(AsyncWebServerRequest* req,
                             const String& body,
                             const char* ct)
{
    AsyncWebServerResponse* r = req->beginResponse(200, ct, body);
    _cors(r);
    req->send(r);
}


// ─────────────────────────────────────────────────────────────────────────────
//  Lifecycle
// ─────────────────────────────────────────────────────────────────────────────

void StuartServer::init(uint16_t port) {
    _server = new AsyncWebServer(port);
}

void StuartServer::begin() {
    if (!_server) {
        Serial.println("[StuartServer] ERROR: call init() before begin()");
        return;
    }
    _server->addHandler(&_logEvents);
    _registerRoutes();
    _server->begin();
    Serial.println("[StuartServer] HTTP server started");
}


// ─────────────────────────────────────────────────────────────────────────────
//  log() — push a message to all connected browser log panels via SSE
// ─────────────────────────────────────────────────────────────────────────────

void StuartServer::log(const String& msg) {
    Serial.println(msg);  // always echo to Serial
    if (_logEvents.count() > 0) {
        // Escape the message as a JSON string so special chars survive SSE.
        _logEvents.send(jsonStr(msg).c_str(), "log", millis());
    }
}


// ─────────────────────────────────────────────────────────────────────────────
//  Handler assignment
// ─────────────────────────────────────────────────────────────────────────────

void StuartServer::onData        (DataProvider  fn) { _dataFn         = fn; }
void StuartServer::onMaze        (MazeProvider  fn) { _mazeFn         = fn; }
void StuartServer::onStart       (VoidHandler   fn) { _startFn        = fn; }
void StuartServer::onStop        (VoidHandler   fn) { _stopFn         = fn; }
void StuartServer::onRestart     (VoidHandler   fn) { _restartFn      = fn; }
void StuartServer::onZero        (VoidHandler   fn) { _zeroFn         = fn; }
void StuartServer::onPosition    (IntHandler    fn) { _positionFn     = fn; }
void StuartServer::onRotation    (IntHandler    fn) { _rotationFn     = fn; }
void StuartServer::onPid         (PidHandler    fn) { _pidFn          = fn; }
void StuartServer::onInstructions(InstrHandler  fn) { _instructionsFn = fn; }


// ─────────────────────────────────────────────────────────────────────────────
//  Route registration
// ─────────────────────────────────────────────────────────────────────────────

void StuartServer::_registerRoutes() {

    // ── GET /log  (SSE — browser log panel) ──────────────────────────────────
    _logEvents.onConnect([](AsyncEventSourceClient* client) {
        client->send("connected", "log", millis());
    });

    // ── OPTIONS preflight + 404 ───────────────────────────────────────────────
    _server->onNotFound([this](AsyncWebServerRequest* req) {
        if (req->method() == HTTP_OPTIONS) {
            AsyncWebServerResponse* r = req->beginResponse(204);
            _cors(r);
            req->send(r);
        } else {
            req->send(404, "text/plain", "Not found");
        }
    });

    // ── GET /data ─────────────────────────────────────────────────────────────
    _server->on("/data", HTTP_GET, [this](AsyncWebServerRequest* req) {
        TelemetrySnapshot snap = _dataFn ? _dataFn() : TelemetrySnapshot{};
        _send200(req, telemetryToJson(snap), "application/json");
    });

    // ── GET /maze ─────────────────────────────────────────────────────────────
    _server->on("/maze", HTTP_GET, [this](AsyncWebServerRequest* req) {
        if (_mazeFn) {
            _send200(req, mazeToJson(_mazeFn()), "application/json");
        } else {
            _send200(req, "{\"width\":8,\"height\":8,\"edges\":[]}", "application/json");
        }
    });

    // ── POST /start ───────────────────────────────────────────────────────────
    _server->on("/start", HTTP_POST, [this](AsyncWebServerRequest* req) {
        if (_startFn) _startFn();
        _send200(req, "ok");
    });

    // ── POST /stop ────────────────────────────────────────────────────────────
    _server->on("/stop", HTTP_POST, [this](AsyncWebServerRequest* req) {
        if (_stopFn) _stopFn();
        _send200(req, "ok");
    });

    // ── POST /restart ─────────────────────────────────────────────────────────
    _server->on("/restart", HTTP_POST, [this](AsyncWebServerRequest* req) {
        if (_restartFn) _restartFn();
        _send200(req, "ok");
    });

    // ── POST /zero ────────────────────────────────────────────────────────────
    _server->on("/zero", HTTP_POST, [this](AsyncWebServerRequest* req) {
        if (_zeroFn) _zeroFn();
        _send200(req, "ok");
    });

    // ── POST /position  (val=N) ───────────────────────────────────────────────
    _server->on("/position", HTTP_POST, [this](AsyncWebServerRequest* req) {
        const int val = _param(req, "val").toInt();
        if (_positionFn) _positionFn(val);
        _send200(req, "ok");
    });

    // ── POST /rotation  (val=N) ───────────────────────────────────────────────
    _server->on("/rotation", HTTP_POST, [this](AsyncWebServerRequest* req) {
        const int val = _param(req, "val").toInt();
        if (_rotationFn) _rotationFn(val);
        _send200(req, "ok");
    });

    // ── POST /pid  (pid=name&p=float&i=float&d=float) ─────────────────────────
    _server->on("/pid", HTTP_POST, [this](AsyncWebServerRequest* req) {
        const String name = _param(req, "pid");
        const float  p    = _param(req, "p").toFloat();
        const float  i    = _param(req, "i").toFloat();
        const float  d    = _param(req, "d").toFloat();
        if (_pidFn && name.length()) _pidFn(name, p, i, d);
        _send200(req, "ok");
    });

    // ── POST /instructions  (seq=F1,R,F2,L,…) ────────────────────────────────
    _server->on("/instructions", HTTP_POST, [this](AsyncWebServerRequest* req) {
        const String seq = _param(req, "seq");
        if (_instructionsFn && seq.length()) _instructionsFn(seq);
        _send200(req, "ok");
    });
}


// ─────────────────────────────────────────────────────────────────────────────
//  Telemetry → JSON
// ─────────────────────────────────────────────────────────────────────────────
//
//  Output format:
//  {
//    "heading":1.2345,"position":243.0,
//    "encoder_left":243,"encoder_right":245,
//    "tof_l":85,"tof_r":90,"tof_f":120,
//    "position-pid":  {"actual":0.0,"target":0.0,"error":0.0},
//    "rotation-pid":  {"actual":0.0,"target":0.0,"error":0.0},
//    "velocity-r-pid":{"actual":0.0,"target":0.0,"error":0.0},
//    "velocity-l-pid":{"actual":0.0,"target":0.0,"error":0.0}
//  }
//
// ─────────────────────────────────────────────────────────────────────────────

String StuartServer::telemetryToJson(const TelemetrySnapshot& s) {
    // Each PID block now has 6 floats (actual/target/error/kp/ki/kd).
    // 4 blocks × ~70 chars + top-level fields ~100 chars ≈ 400 bytes.
    String j;
    j.reserve(450);

    j  = "{\"heading\":";        j += fmtFloat(s.heading);
    j += ",\"position\":";       j += fmtFloat(s.position);
    j += ",\"encoder_left\":";   j += s.encoder_left;
    j += ",\"encoder_right\":";  j += s.encoder_right;
    j += ",\"tof_l\":";          j += s.tof_l;
    j += ",\"tof_r\":";          j += s.tof_r;
    j += ",\"tof_f\":";          j += s.tof_f;
    j += ",\"position-pid\":";   j += pidJson(s.pos_pid);
    j += ",\"rotation-pid\":";   j += pidJson(s.rot_pid);
    j += ",\"velocity-r-pid\":"; j += pidJson(s.vel_r_pid);
    j += ",\"velocity-l-pid\":"; j += pidJson(s.vel_l_pid);
    j += '}';

    return j;
}


// ─────────────────────────────────────────────────────────────────────────────
//  Maze graph → JSON
// ─────────────────────────────────────────────────────────────────────────────
//
//  Output format:
//  {"width":8,"height":8,"edges":[[[0,0],[1,0]],[[0,0],[0,1]],...]}
//
//  The adjacency list stores each passage twice (A→B and B→A).
//  We deduplicate by only emitting edges where the first node is
//  lexicographically smaller: (x1 < x2) OR (x1 == x2 AND y1 < y2).
//
// ─────────────────────────────────────────────────────────────────────────────

String StuartServer::mazeToJson(const MazeGraph& graph, int width, int height) {
    // Pre-size: 8×8 maze has at most 112 internal edges.
    // Each edge serialises as "[[x1,y1],[x2,y2]]," = ~18 chars worst case.
    // 112 × 18 + overhead ≈ 2200 bytes.
    String j;
    j.reserve(2400);

    j  = "{\"width\":";  j += width;
    j += ",\"height\":"; j += height;
    j += ",\"edges\":[";

    bool first = true;
    for (const auto& [node, neighbours] : graph) {
        const int x1 = node.first;
        const int y1 = node.second;
        for (const auto& nb : neighbours) {
            const int x2 = nb.first;
            const int y2 = nb.second;
            // Emit each passage only once
            if (x1 > x2 || (x1 == x2 && y1 > y2)) continue;

            if (!first) j += ',';
            first = false;

            j += "[[";
            j += x1; j += ','; j += y1;
            j += "],[";
            j += x2; j += ','; j += y2;
            j += "]]";
        }
    }

    j += "]}";
    return j;
}