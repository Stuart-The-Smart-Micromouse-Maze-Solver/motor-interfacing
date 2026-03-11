#include "server.h"
#include "maze_nav.h"
#include "WiFi.h"

IPAddress local_ip(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

RobotServer::RobotServer(PIDController<float>* turn,
                         PIDController<float>* pos,
                         PIDController<float>* rVel,
                         PIDController<float>* lVel)
    : _server(80), _turn(turn), _pos(pos), _rVel(rVel), _lVel(lVel) {}

void RobotServer::begin(const char* ssid, const char* password,
                        void (*startFunc)(),
                        void (*stopFunc)(),
                        void (*restartFunc)(),
                        void (*posFunc)(int),
                        void (*turnFunc)(float)) {
    _startCallback = startFunc;
    _stopCallback = stopFunc;
    _restartCallback = restartFunc;
    _posCallback = posFunc;
    _turnCallback = turnFunc;

    // ---------------------------------
    // WiFi AP mode with static IP
    // ---------------------------------
    WiFi.mode(WIFI_AP);
    WiFi.setSleep(false);

    bool cfgOk = WiFi.softAPConfig(local_ip, gateway, subnet);
    bool apOk  = WiFi.softAP(ssid, password);

    Serial.println();
    Serial.println("=== Stuart AP Mode ===");
    Serial.print("AP config: ");
    Serial.println(cfgOk ? "OK" : "FAILED");
    Serial.print("AP start:  ");
    Serial.println(apOk ? "OK" : "FAILED");

    if (apOk) {
        Serial.print("SSID: ");
        Serial.println(ssid);
        Serial.print("IP: ");
        Serial.println(WiFi.softAPIP());
    } else {
        Serial.println("Failed to start AP");
    }

    // ---------------------------------
    // Main UI
    // ---------------------------------
    _server.on("/", HTTP_GET, [this](AsyncWebServerRequest *request) {
        String html;
        html.reserve(16000);

        html += "<html><head><title>Stuart Control</title>";
        html += "<meta name='viewport' content='width=device-width, initial-scale=1'/>";
        // html += "<script src='https://cdn.jsdelivr.net/npm/chart.js'></script>";
        html += "<style>";
        html += "body{font-family:sans-serif;background:#121212;color:#eee;display:flex;margin:0;}";
        html += ".sidebar{width:340px;padding:20px;background:#1e1e1e;height:100vh;overflow-y:auto;border-right:1px solid #333;}";
        html += ".main{flex-grow:1;padding:20px;display:flex;flex-direction:column;gap:20px;}";
        html += ".card{background:#252525;padding:15px;border-radius:10px;border:1px solid #333;}";
        html += ".chart-container{height:200px;position:relative;}";
        html += "input{width:70px;background:#333;color:white;border:1px solid #555;padding:4px;}";
        html += "button{background:#007bff;color:white;border:none;padding:8px;border-radius:4px;cursor:pointer;margin:2px;font-size:12px;}";
        html += "button.cmd{background:#28a745;width:100%;margin-top:5px;}";
        html += "button.warn{background:#dc3545;width:100%;font-weight:bold;}";
        html += "button.nav{background:#6f42c1;width:100%;}";
        html += "button.info{background:#17a2b8;width:100%;}";
        html += ".mono{font-family:monospace;white-space:pre-wrap;}";
        html += "hr{border-color:#333;}";
        html += "</style></head><body>";

        // ---------------- Sidebar ----------------
        html += "<div class='sidebar'>";

        html += "<h3>System</h3>";
        html += "<button class='info' onclick=\"fetch('/observe')\">OBSERVE NOW</button>";
        html += "<button class='warn' onclick=\"fetch('/stop')\">STOP</button>";
        html += "<button class='warn' onclick=\"fetch('/reset_pose')\">RESET POSE</button>";

        html += "<h3>Primitive Motion</h3>";
        html += "<button class='cmd' onclick=\"fetch('/pos?val=1')\">Move 1 Cell</button>";
        html += "<div style='display:flex;gap:5px;'>";
        html += "<input id='customPos' value='1'>";
        html += "<button class='cmd' onclick=\"fetch('/pos?val='+document.getElementById('customPos').value)\">Queue Cells</button>";
        html += "</div>";

        html += "<hr>";

        html += "<button class='cmd' onclick=\"fetch('/turn?val=-90')\">Turn Left (-90)</button>";
        html += "<button class='cmd' onclick=\"fetch('/turn?val=90')\">Turn Right (90)</button>";
        html += "<div style='display:flex;gap:5px;'>";
        html += "<input id='customTurn' value='180'>";
        html += "<button class='cmd' onclick=\"fetch('/turn?val='+document.getElementById('customTurn').value)\">Queue Turn</button>";
        html += "</div>";

        html += "<h3>Navigator</h3>";
        html += "<div style='display:flex;gap:5px;'>";
        html += "<input id='cmdSeq' style='width:160px' value='F,R,F'>";
        html += "<button class='nav' onclick=\"fetch('/exec?seq='+encodeURIComponent(document.getElementById('cmdSeq').value))\">Run Seq</button>";
        html += "</div>";
        html += "<button class='nav' onclick=\"fetch('/plan_center')\">Plan To Center</button>";

        html += "<div class='card mono'>";
        html += "<div><b>Pose</b></div><div id='poseBox'>-</div>";
        html += "<div><b>Nav</b></div><div id='navBox'>-</div>";
        html += "</div>";

        html += "<h3>PID Tuning</h3>";
        html += getParamHTML("Position",   _pos,  "pos");
        html += getParamHTML("Turn",       _turn, "turn");
        html += getParamHTML("Velocity R", _rVel, "rvel");
        html += getParamHTML("Velocity L", _lVel, "lvel");

        html += "<div class='card'>";
        html += "<h4>System Log</h4>";
        html += "<textarea id='logBox' readonly "
                "style='width:100%;height:180px;background:#111;color:#0f0;border:1px solid #333;"
                "font-family:monospace;resize:none;padding:10px;'></textarea>";
        html += "</div>";

        html += "</div>";

        // ---------------- Main area ----------------
        html += "<div class='main'>";
        html += "<div class='card'><h4>Position Error</h4><div class='chart-container'><canvas id='posChart'></canvas></div></div>";
        html += "<div class='card'><h4>Velocity Error (L/R)</h4><div class='chart-container'><canvas id='velChart'></canvas></div></div>";
        html += "<div class='card'><h4>Turn Error</h4><div class='chart-container'><canvas id='turnChart'></canvas></div></div>";
        html += "</div>";

        // ---------------- JS ----------------
        html += "<script>";
        html += "function safeText(id,val){const el=document.getElementById(id); if(el) el.innerText=val;}";
        html += "function setLog(t){const box=document.getElementById('logBox'); if(box){box.value=t; box.scrollTop=box.scrollHeight;}}";

        html += "setInterval(()=>{";
        html += "fetch('/data').then(r=>r.json()).then(d=>{";
        html += "['pos','turn','rvel','lvel'].forEach(id=>{";
        html += "safeText(id+'_t', d[id].t);";
        html += "safeText(id+'_o', d[id].o);";
        html += "});";
        html += "}).catch(err=>console.log('data fetch failed', err));";
        html += "},200);";

        html += "setInterval(()=>{";
        html += "fetch('/state').then(r=>r.json()).then(s=>{";
        html += "safeText('poseBox', `row=${s.pose.row} col=${s.pose.col} heading=${s.pose.heading}`);";
        html += "safeText('navBox', `busy=${s.busy} queue=${s.queue} motors=${s.motors_busy}`);";
        html += "}).catch(err=>console.log('state fetch failed', err));";
        html += "},400);";

        html += "setInterval(()=>{";
        html += "fetch('/log').then(r=>r.text()).then(t=>{";
        html += "setLog(t);";
        html += "}).catch(err=>console.log('log fetch failed', err));";
        html += "},300);";
        html += "</script>";

        request->send(200, "text/html", html);
    });

    // ---------------------------------
    // Motion endpoints
    // ---------------------------------
    _server.on("/pos", HTTP_GET, [this](AsyncWebServerRequest *request) {
        if (request->hasParam("val") && _posCallback) {
            _posCallback(request->getParam("val")->value().toInt());
        }
        request->send(200, "text/plain", "OK");
    });

    _server.on("/turn", HTTP_GET, [this](AsyncWebServerRequest *request) {
        if (request->hasParam("val") && _turnCallback) {
            _turnCallback(request->getParam("val")->value().toFloat());
        }
        request->send(200, "text/plain", "OK");
    });

    // ---------------------------------
    // PID / telemetry
    // ---------------------------------
    _server.on("/data", HTTP_GET, [this](AsyncWebServerRequest *request) {
        auto fmt = [](String id, PIDController<float>* p) {
            return "\"" + id + "\":{\"t\":" + String(p->getTarget()) +
                   ",\"o\":" + String(p->getOutput()) +
                   ",\"e\":" + String(p->getError()) + "}";
        };
        request->send(200, "application/json",
            "{" + fmt("turn", _turn) + "," + fmt("pos", _pos) + "," +
            fmt("rvel", _rVel) + "," + fmt("lvel", _lVel) + "}");
    });

    _server.on("/update", HTTP_GET, [this](AsyncWebServerRequest *request) {
        if (!request->hasParam("id") || !request->hasParam("p") ||
            !request->hasParam("i") || !request->hasParam("d")) {
            request->send(400, "text/plain", "Missing PID params");
            return;
        }

        String id = request->getParam("id")->value();
        PIDController<float>* p =
            (id == "turn") ? _turn :
            (id == "pos")  ? _pos  :
            (id == "rvel") ? _rVel : _lVel;

        p->setPID(request->getParam("p")->value().toDouble(),
                  request->getParam("i")->value().toDouble(),
                  request->getParam("d")->value().toDouble());

        request->send(200, "text/plain", "OK");
    });

    // ---------------------------------
    // Navigation / pose / map
    // ---------------------------------
    _server.on("/state", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "application/json", maze_nav::stateJson());
    });

    _server.on("/pose", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "application/json", maze_nav::poseJson());
    });

    _server.on("/maze", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "application/json", maze_nav::mazeJson());
    });

    _server.on("/exec", HTTP_GET, [](AsyncWebServerRequest *request) {
        bool ok = false;
        if (request->hasParam("seq")) {
            ok = maze_nav::enqueueSequence(request->getParam("seq")->value());
        }
        request->send(200, "application/json",
                      String("{\"ok\":") + (ok ? "true" : "false") + "}");
    });

    _server.on("/plan_center", HTTP_GET, [](AsyncWebServerRequest *request) {
        bool ok = maze_nav::planToCenter();
        request->send(200, "application/json",
                      String("{\"ok\":") + (ok ? "true" : "false") + "}");
    });

    _server.on("/observe", HTTP_GET, [](AsyncWebServerRequest *request) {
        maze_nav::observeCurrentCell();
        request->send(200, "text/plain", "OK");
    });

    _server.on("/reset_pose", HTTP_GET, [](AsyncWebServerRequest *request) {
        maze_nav::resetToStart();
        request->send(200, "text/plain", "OK");
    });

    // ---------------------------------
    // Legacy callbacks
    // ---------------------------------
    _server.on("/start", HTTP_GET, [this](AsyncWebServerRequest *request) {
        if (_startCallback) _startCallback();
        request->send(200, "text/plain", "OK");
    });

    _server.on("/stop", HTTP_GET, [this](AsyncWebServerRequest *request) {
        if (_stopCallback) _stopCallback();
        request->send(200, "text/plain", "OK");
    });

    _server.on("/restart", HTTP_GET, [this](AsyncWebServerRequest *request) {
        if (_restartCallback) _restartCallback();
        request->send(200, "text/plain", "OK");
    });

    // ---------------------------------
    // Log endpoint
    // ---------------------------------
    _server.on("/log", HTTP_GET, [this](AsyncWebServerRequest *request) {
        String out;
        for (const auto& line : _logBuffer) {
            out += line + "\n";
        }
        request->send(200, "text/plain", out);
    });

    _server.begin();
}

void RobotServer::log(const String& msg) {
    String line = "[" + String(millis()) + "] " + msg;
    _logBuffer.push_back(line);
    if (_logBuffer.size() > _maxLogLines) {
        _logBuffer.pop_front();
    }
    Serial.println(line);
}

String RobotServer::getParamHTML(String name, PIDController<float>* pid, String id) {
    String h = "<div style='font-size:0.85em; margin-bottom:10px;'><strong>" + name + "</strong><br>";
    h += "P<input id='" + id + "_p' value='" + String(pid->getP(), 3) + "'> ";
    h += "I<input id='" + id + "_i' value='" + String(pid->getI(), 3) + "'> ";
    h += "D<input id='" + id + "_d' value='" + String(pid->getD(), 4) + "'><br>";
    h += "<button onclick=\"fetch('/update?id=" + id +
         "&p='+document.getElementById('" + id + "_p').value+"
         "'&i='+document.getElementById('" + id + "_i').value+"
         "'&d='+document.getElementById('" + id + "_d').value)\">Set</button> ";
    h += "T:<span id='" + id + "_t'>-</span> ";
    h += "O:<span id='" + id + "_o'>-</span></div>";
    return h;
}