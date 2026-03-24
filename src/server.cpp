// server.cpp
// Changes from the original you sent:
//   1. HTML in PROGMEM (flash), served via F() macro — no beginResponse_P (removed in mathieucarbou fork ≥3.0.2)
//   2. SSE replaces /data + /log polling — one persistent /events connection, pushed at TELEMETRY_HZ
//   3. Telemetry sub-task spawned inside begin() on Core 0 — your distanceUpdateAll() loop is unaffected
//   4. _logBuffer is a fixed ring-array protected by a FreeRTOS mutex (was unprotected std::deque)
//   5. Sidebar is resizable by dragging the gutter; graph area absorbs all resize pressure (overflow:hidden)
//   6. PID inputs pre-filled to 6 decimal places — 0.0005 no longer vanishes
//   7. Motion panel: arrow-key pad (▲ ◄ ►) + custom "Move X cells / Turn Y deg" rows (defaults 2 / 180)

#include "server.h"
#include "distance.h"

// ─────────────────────────────────────────────────────────────────────────────
//  PAGE_HTML stored in flash.
//  Use F() when passing to send() — mathieucarbou fork accepts FlashStringHelper.
//  DO NOT use beginResponse_P: it was removed for ESP32 in this fork.
// ─────────────────────────────────────────────────────────────────────────────
static const char PAGE_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>Stuart — Control</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@400;700&family=Syne:wght@700;800&display=swap');
  :root {
    --bg:      #0a0a0f;
    --surface: #12121a;
    --card:    #1a1a26;
    --border:  #2a2a40;
    --accent:  #7c6dfa;
    --green:   #39d98a;
    --red:     #f05e6b;
    --amber:   #ffb547;
    --cyan:    #4bc0c0;
    --purple:  #9966ff;
    --text:    #e2e2f0;
    --muted:   #6a6a8a;
    --mono:    'JetBrains Mono', monospace;
    --display: 'Syne', sans-serif;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    font-family: var(--mono);
    background: var(--bg);
    color: var(--text);
    display: flex;
    height: 100vh;
    overflow: hidden;
    font-size: 13px;
    user-select: none;
  }

  /* ── Sidebar — fixed width, independent scroll ── */
  #sidebar {
    width: 300px;        /* initial; JS drag changes this */
    min-width: 220px;
    max-width: 520px;
    background: var(--surface);
    border-right: 1px solid var(--border);
    display: flex;
    flex-direction: column;
    overflow-y: auto;
    overflow-x: hidden;
    padding: 14px;
    gap: 12px;
    flex-shrink: 0;     /* sidebar NEVER yields space to graphs */
  }
  #sidebar h1 {
    font-family: var(--display);
    font-size: 20px;
    font-weight: 800;
    letter-spacing: -0.5px;
    color: var(--accent);
    border-bottom: 1px solid var(--border);
    padding-bottom: 8px;
    flex-shrink: 0;
  }

  /* Drag handle */
  #resizer {
    width: 5px;
    background: var(--border);
    cursor: col-resize;
    flex-shrink: 0;
    transition: background 0.15s;
  }
  #resizer:hover, #resizer.dragging { background: var(--accent); }

  .section-label {
    font-size: 10px;
    font-weight: 700;
    letter-spacing: 2px;
    color: var(--muted);
    text-transform: uppercase;
    margin-bottom: -4px;
    display: flex;
    align-items: center;
    gap: 6px;
    flex-shrink: 0;
  }

  /* ── Buttons ── */
  .btn {
    display: block;
    width: 100%;
    padding: 8px 10px;
    border: 1px solid var(--border);
    border-radius: 6px;
    background: var(--card);
    color: var(--text);
    font-family: var(--mono);
    font-size: 12px;
    font-weight: 700;
    cursor: pointer;
    letter-spacing: 1px;
    transition: background 0.12s, border-color 0.12s, transform 0.08s;
    text-align: center;
  }
  .btn:active { transform: scale(0.96); }
  .btn.start  { border-color: var(--green); color: var(--green); }
  .btn.start:hover  { background: rgba(57,217,138,0.12); }
  .btn.stop   { border-color: var(--red);   color: var(--red); }
  .btn.stop:hover   { background: rgba(240,94,107,0.12); }
  .btn.restart{ border-color: var(--amber); color: var(--amber); }
  .btn.restart:hover{ background: rgba(255,181,71,0.12); }
  .btn.cmd    { border-color: var(--accent); color: var(--accent); }
  .btn.cmd:hover    { background: rgba(124,109,250,0.12); }
  .btn.sm     { padding: 5px 8px; font-size: 11px; width: auto; flex: none; }

  .row { display: flex; gap: 6px; align-items: center; }
  .row input {
    flex: 1;
    background: var(--bg);
    border: 1px solid var(--border);
    color: var(--text);
    font-family: var(--mono);
    font-size: 12px;
    padding: 6px 8px;
    border-radius: 6px;
    outline: none;
    min-width: 0;
    transition: border-color 0.15s;
  }
  .row input:focus { border-color: var(--accent); }
  .row .btn { flex: 2; }

  /* Arrow pad — 3-col grid, row 1: [empty, up, empty]  row 2: [left, empty, right] */
  .arrow-pad {
    display: grid;
    grid-template-columns: repeat(3, 1fr);
    gap: 5px;
  }
  .arrow-pad .btn { padding: 10px 4px; font-size: 17px; width: 100%; }

  /* PID */
  .pid-block {
    background: var(--card);
    border: 1px solid var(--border);
    border-radius: 8px;
    padding: 9px 11px;
  }
  .pid-block strong { font-size: 10px; letter-spacing: 1px; }
  .pid-row { display: flex; gap: 4px; margin: 5px 0 3px; align-items: center; }
  .pid-row label { font-size: 10px; color: var(--muted); white-space: nowrap; }
  .pid-row input {
    flex: 1;
    background: var(--bg);
    border: 1px solid var(--border);
    color: var(--text);
    font-family: var(--mono);
    font-size: 11px;
    padding: 4px 5px;
    border-radius: 5px;
    outline: none;
    min-width: 0;
  }
  .pid-row input:focus { border-color: var(--accent); }
  .pid-meta { display: flex; gap: 10px; font-size: 10px; color: var(--muted); margin-top: 2px; }
  .pid-meta span { color: var(--text); }

  /* Log */
  #logBox {
    background: var(--bg);
    border: 1px solid var(--border);
    border-radius: 6px;
    color: var(--green);
    font-family: var(--mono);
    font-size: 11px;
    padding: 8px;
    resize: none;
    height: 140px;
    outline: none;
    line-height: 1.5;
    flex-shrink: 0;
  }
  #sse-dot {
    display: inline-block;
    width: 7px; height: 7px;
    border-radius: 50%;
    background: var(--red);
    transition: background 0.3s;
    flex-shrink: 0;
  }
  #sse-dot.live { background: var(--green); box-shadow: 0 0 5px var(--green); }

  /* ── Main — graphs absorb ALL resize pressure ── */
  #main {
    flex: 1;
    display: flex;
    flex-direction: column;
    gap: 10px;
    padding: 14px;
    overflow: hidden;
    min-width: 0;
  }
  .card {
    background: var(--card);
    border: 1px solid var(--border);
    border-radius: 10px;
    padding: 10px 14px;
    flex: 1;
    display: flex;
    flex-direction: column;
    min-height: 0;
    overflow: hidden;
  }
  .card-header {
    display: flex;
    justify-content: space-between;
    align-items: baseline;
    margin-bottom: 4px;
    flex-shrink: 0;
  }
  .card-title { font-size: 10px; font-weight: 700; letter-spacing: 1.5px; color: var(--muted); text-transform: uppercase; }
  .card-val   { font-size: 16px; font-weight: 700; font-family: var(--display); color: var(--text); }
  .chart-wrap { position: relative; flex: 1; min-height: 0; overflow: hidden; }
  canvas { position: absolute; inset: 0; }
</style>
</head>
<body>

<!-- ══ SIDEBAR ═══════════════════════════════════════════════════ -->
<div id="sidebar">
  <h1>STUART</h1>

  <div class="section-label">Control</div>
  <div class="row">
    <button class="btn start"   onclick="cmd('/start')">▶ START</button>
    <button class="btn stop"    onclick="cmd('/stop')">■ STOP</button>
  </div>
  <button class="btn restart" onclick="cmd('/restart')">↺ RESTART</button>
  <button class="btn cmd" onclick="if(confirm('Robot will move! 30cm clear space needed.'))cmd('/calibrate')" style="border-color:#39d98a;color:#39d98a">⚡ CALIBRATE</button>

  <!-- Arrow pad -->
  <div class="section-label">Quick Move</div>
  <div class="arrow-pad">
    <div></div>
    <button class="btn cmd" onclick="cmd('/pos?val=1')"   title="Forward 1 cell">▲</button>
    <div></div>
    <button class="btn cmd" onclick="cmd('/turn?val=-90')" title="Turn Left 90°">◄</button>
    <div></div>
    <button class="btn cmd" onclick="cmd('/turn?val=90')"  title="Turn Right 90°">►</button>
  </div>

  <!-- Custom move -->
  <div class="section-label">Custom Move</div>
  <div class="row">
    <input id="customPos" value="2" type="number" min="1">
    <button class="btn cmd" onclick="cmd('/pos?val='+document.getElementById('customPos').value)">Move X cells</button>
  </div>
  <div class="row">
    <input id="customTurn" value="180" type="number">
    <button class="btn cmd" onclick="cmd('/turn?val='+document.getElementById('customTurn').value)">Turn Y deg</button>
  </div>

  <!-- Maze sequence executor -->
  <div class="section-label">Maze Sequence</div>
  <div class="row">
    <input id="seqInput" value="F,R,F,R,F,R,F" style="flex:2">
    <button class="btn cmd" onclick="cmd('/exec?seq='+encodeURIComponent(document.getElementById('seqInput').value))">Run</button>
  </div>
  <div style="font-size:10px;color:var(--muted);margin-top:-6px">F R L U W &mdash; e.g. F3,R,F2,L,F</div>

  <!-- PID -->
  <div class="section-label">PID Tuning</div>
  <div id="pid-pos"  class="pid-block"></div>
  <div id="pid-turn" class="pid-block"></div>
  <div id="pid-rvel" class="pid-block"></div>
  <div id="pid-lvel" class="pid-block"></div>

  <!-- Log -->
  <div class="section-label"><span id="sse-dot"></span>Live Log</div>
  <textarea id="logBox" readonly></textarea>
</div>

<!-- Drag handle -->
<div id="resizer"></div>

<!-- ══ CHARTS ════════════════════════════════════════════════════ -->
<div id="main">
  <div class="card">
    <div class="card-header">
      <span class="card-title">Position Error</span>
      <span class="card-val" id="val-pos">—</span>
    </div>
    <div class="chart-wrap"><canvas id="posChart"></canvas></div>
  </div>
  <div class="card">
    <div class="card-header">
      <span class="card-title">Velocity Error  L / R</span>
      <span class="card-val" id="val-vel">—</span>
    </div>
    <div class="chart-wrap"><canvas id="velChart"></canvas></div>
  </div>
  <div class="card">
    <div class="card-header">
      <span class="card-title">Turn Error</span>
      <span class="card-val" id="val-turn">—</span>
    </div>
    <div class="chart-wrap"><canvas id="turnChart"></canvas></div>
  </div>
  <div class="card">
    <div class="card-header">
      <span class="card-title">Velocity  Target (dashed) vs Actual (solid)</span>
      <span class="card-val" id="val-rpm">—</span>
    </div>
    <div class="chart-wrap"><canvas id="rpmChart"></canvas></div>
  </div>
  <div class="card">
    <div class="card-header">
      <span class="card-title">ToF Distance  L / F / R  (mm)</span>
      <span class="card-val" id="val-tof">—</span>
    </div>
    <div class="chart-wrap"><canvas id="tofChart"></canvas></div>
  </div>
</div>

<script src="https://cdn.jsdelivr.net/npm/chart.js@4/dist/chart.umd.min.js"></script>
<script>
// ── Sidebar resize ────────────────────────────────────────────────
(function () {
  const sidebar = document.getElementById('sidebar');
  const resizer = document.getElementById('resizer');
  let dragging = false, startX = 0, startW = 0;
  resizer.addEventListener('mousedown', e => {
    dragging = true; startX = e.clientX; startW = sidebar.offsetWidth;
    resizer.classList.add('dragging');
    document.body.style.cursor = 'col-resize';
    e.preventDefault();
  });
  document.addEventListener('mousemove', e => {
    if (!dragging) return;
    const w = Math.min(Math.max(startW + (e.clientX - startX), 220), 520);
    sidebar.style.width = w + 'px';
  });
  document.addEventListener('mouseup', () => {
    dragging = false;
    resizer.classList.remove('dragging');
    document.body.style.cursor = '';
  });
})();

// ── Helpers ───────────────────────────────────────────────────────
function cmd(url) { fetch(url).catch(() => {}); }

// ── Charts ────────────────────────────────────────────────────────
const MAX_PTS = 200;

function makeChart(id, datasets) {
  return new Chart(document.getElementById(id), {
    type: 'line',
    data: { labels: [], datasets },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      animation: false,
      plugins: {
        legend: {
          display: datasets.length > 1,
          labels: { color: '#6a6a8a', font: { family: "'JetBrains Mono'", size: 10 } }
        }
      },
      scales: {
        x: { display: false },
        y: {
          grid: {
            color: ctx => ctx.tick.value === 0 ? '#3a3a5a' : '#1e1e2e',
            lineWidth: ctx => ctx.tick.value === 0 ? 2 : 1
          },
          ticks: { color: '#6a6a8a', font: { family: "'JetBrains Mono'", size: 10 } }
        }
      }
    }
  });
}

function pushChart(chart, ...values) {
  const d = chart.data;
  if (d.labels.length >= MAX_PTS) {
    d.labels.shift();
    d.datasets.forEach(ds => ds.data.shift());
  }
  d.labels.push('');
  values.forEach((v, i) => d.datasets[i].data.push(v));
  chart.update('none');
}

const posChart  = makeChart('posChart',  [{ label:'Pos Err',  borderColor:'#4bc0c0', data:[], borderWidth:1.5, pointRadius:0, fill:false }]);
const velChart  = makeChart('velChart',  [
  { label:'L-Vel', borderColor:'#9966ff', data:[], borderWidth:1.5, pointRadius:0, fill:false },
  { label:'R-Vel', borderColor:'#ffb547', data:[], borderWidth:1.5, pointRadius:0, fill:false }
]);
const turnChart = makeChart('turnChart', [{ label:'Turn Err', borderColor:'#f05e6b', data:[], borderWidth:1.5, pointRadius:0, fill:false }]);

// Velocity tracking chart: dashed = target, solid = actual (feedback)
// L = purple (#9966ff), R = amber (#ffb547)
const rpmChart = makeChart('rpmChart', [
  { label:'L Target', borderColor:'#9966ff', borderDash:[6,3], data:[], borderWidth:1.5, pointRadius:0, fill:false },
  { label:'L Actual', borderColor:'#9966ff', borderDash:[],    data:[], borderWidth:2,   pointRadius:0, fill:false },
  { label:'R Target', borderColor:'#ffb547', borderDash:[6,3], data:[], borderWidth:1.5, pointRadius:0, fill:false },
  { label:'R Actual', borderColor:'#ffb547', borderDash:[],    data:[], borderWidth:2,   pointRadius:0, fill:false },
]);

const tofChart = makeChart('tofChart', [
  { label:'Left',  borderColor:'#9966ff', data:[], borderWidth:1.5, pointRadius:0, fill:false },
  { label:'Front', borderColor:'#39d98a', data:[], borderWidth:2,   pointRadius:0, fill:false },
  { label:'Right', borderColor:'#ffb547', data:[], borderWidth:1.5, pointRadius:0, fill:false },
]);

// ── PID blocks ────────────────────────────────────────────────────
const PID_DEFS = [
  { id:'pos',  label:'Position',   color:'#4bc0c0' },
  { id:'turn', label:'Rotation',   color:'#f05e6b' },
  { id:'rvel', label:'Velocity R', color:'#ffb547' },
  { id:'lvel', label:'Velocity L', color:'#9966ff' },
];

PID_DEFS.forEach(({ id, label, color }) => {
  document.getElementById('pid-' + id).innerHTML = `
    <strong style="color:${color}">${label}</strong>
    <div class="pid-row">
      <label>P</label><input id="${id}_p" value="—">
      <label>I</label><input id="${id}_i" value="—">
      <label>D</label><input id="${id}_d" value="—">
      <button class="btn cmd sm" onclick="setPID('${id}')">Set</button>
    </div>
    <div class="pid-meta">
      T:<span id="${id}_t">—</span>&nbsp;&nbsp;O:<span id="${id}_o">—</span>
    </div>`;
});

function setPID(id) {
  const p = document.getElementById(id + '_p').value;
  const i = document.getElementById(id + '_i').value;
  const d = document.getElementById(id + '_d').value;
  cmd('/update?id=' + id + '&p=' + p + '&i=' + i + '&d=' + d);
}

// ── SSE ───────────────────────────────────────────────────────────
const dot    = document.getElementById('sse-dot');
const logBox = document.getElementById('logBox');
const pidPrefilled = {};

function connectSSE() {
  const es = new EventSource('/events');

  es.addEventListener('telemetry', e => {
    const d = JSON.parse(e.data);

    pushChart(posChart,  d.pos.e);
    pushChart(velChart,  d.lvel.e, d.rvel.e);
    pushChart(turnChart, d.turn.e);
    pushChart(rpmChart,  d.lvel.t, d.lvel.f, d.rvel.t, d.rvel.f);
    pushChart(tofChart,  d.dL, d.dF, d.dR);

    document.getElementById('val-pos').textContent  = d.pos.e.toFixed(3);
    document.getElementById('val-vel').textContent  = d.lvel.e.toFixed(2) + ' / ' + d.rvel.e.toFixed(2);
    document.getElementById('val-turn').textContent = d.turn.e.toFixed(3) + '\xb0';
    document.getElementById('val-rpm').textContent  =
      'L ' + d.lvel.f.toFixed(1) + ' / R ' + d.rvel.f.toFixed(1);
    document.getElementById('val-tof').textContent  =
      d.dL + ' / ' + d.dF + ' / ' + d.dR;

    PID_DEFS.forEach(({ id }) => {
      if (!d[id]) return;
      document.getElementById(id + '_t').textContent = d[id].t.toFixed(6);
      document.getElementById(id + '_o').textContent = d[id].o.toFixed(6);
      if (!pidPrefilled[id] && d[id].p !== undefined) {
        document.getElementById(id + '_p').value = d[id].p.toFixed(6);
        document.getElementById(id + '_i').value = d[id].iv.toFixed(6);
        document.getElementById(id + '_d').value = d[id].dv.toFixed(6);
        pidPrefilled[id] = true;
      }
    });
  });

  es.addEventListener('log', e => {
    logBox.value += e.data + '\n';
    const lines = logBox.value.split('\n');
    if (lines.length > 300) logBox.value = lines.slice(-200).join('\n');
    logBox.scrollTop = logBox.scrollHeight;
  });

  es.onopen  = () => dot.classList.add('live');
  es.onerror = () => {
    dot.classList.remove('live');
    es.close();
    setTimeout(connectSSE, 2000);
  };
}
connectSSE();
</script>
</body>
</html>
)rawhtml";


// ─────────────────────────────────────────────────────────────────────────────
//  Constructor
// ─────────────────────────────────────────────────────────────────────────────
RobotServer::RobotServer(PIDController<float>* turn, PIDController<float>* pos,
                         PIDController<float>* rVel, PIDController<float>* lVel)
    : _server(80), _events("/events"),
      _turn(turn), _pos(pos), _rVel(rVel), _lVel(lVel)
{
    _logMutex = xSemaphoreCreateMutex();
}


// ─────────────────────────────────────────────────────────────────────────────
//  begin()
//
//  Call from your existing xTaskCreatePinnedToCore lambda exactly as before:
//
//      robotServer.begin(WIFI_SSID, WIFI_PWD, startCb, stopCb, restartCb, posCb, turnCb);
//      // then your own for(;;) loop with distanceUpdateAll() / gyroCache() — unchanged
//
//  The SSE telemetry sub-task is spawned here (Core 0, 4 KB stack).
//  Your distanceUpdateAll() stays in YOUR loop — this function does not touch it.
// ─────────────────────────────────────────────────────────────────────────────
void RobotServer::begin(const char* ssid, const char* password,
                        void (*startFunc)(),
                        void (*stopFunc)(),
                        void (*restartFunc)(),
                        void (*posFunc)(int),
                        void (*turnFunc)(float),
                        bool (*execFunc)(const String&),
                        void (*calibrateFunc)())
{
    _startCallback   = startFunc;
    _stopCallback    = stopFunc;
    _restartCallback = restartFunc;
    _posCallback     = posFunc;
    _turnCallback    = turnFunc;
    _execCallback    = execFunc;
    _calibrateCallback = calibrateFunc;

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
    Serial.println("\nWiFi: " + WiFi.localIP().toString());

    // ── Serve HTML from flash via F() ─────────────────────────────────────
    _server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
        AsyncWebServerResponse* r = req->beginResponse(200, "text/html", F(PAGE_HTML));
        r->addHeader("Cache-Control", "max-age=3600");
        req->send(r);
    });

    // ── SSE endpoint ──────────────────────────────────────────────────────
    _events.onConnect([](AsyncEventSourceClient* client) {
        client->send("connected", "log", millis());
    });
    _server.addHandler(&_events);

    // ── Movement ──────────────────────────────────────────────────────────
    _server.on("/pos",  HTTP_GET, [this](AsyncWebServerRequest* req) {
        if (req->hasParam("val") && _posCallback)
            _posCallback(req->getParam("val")->value().toInt());
        req->send(200, "text/plain", "OK");
    });
    _server.on("/turn", HTTP_GET, [this](AsyncWebServerRequest* req) {
        if (req->hasParam("val") && _turnCallback)
            _turnCallback(req->getParam("val")->value().toFloat());
        req->send(200, "text/plain", "OK");
    });

    // ── Maze instruction execution ───────────────────────────────
    // Usage: /exec?seq=F3,R,F2,L,F
    _server.on("/exec", HTTP_GET, [this](AsyncWebServerRequest* req) {
        if (req->hasParam("seq") && _execCallback) {
            String seq = req->getParam("seq")->value();
            bool ok = _execCallback(seq);
            req->send(200, "text/plain", ok ? "Executing: " + seq : "FAILED");
        } else {
            req->send(400, "text/plain", "Usage: /exec?seq=F,R,F,L");
        }
    });

    // ── PID update ────────────────────────────────────────────────────────
    _server.on("/update", HTTP_GET, [this](AsyncWebServerRequest* req) {
        if (!req->hasParam("id")) { req->send(400); return; }
        const String& id = req->getParam("id")->value();
        PIDController<float>* p = (id == "turn") ? _turn :
                                  (id == "pos")  ? _pos  :
                                  (id == "rvel") ? _rVel : _lVel;
        if (req->hasParam("p") && req->hasParam("i") && req->hasParam("d"))
            p->setPID(req->getParam("p")->value().toDouble(),
                      req->getParam("i")->value().toDouble(),
                      req->getParam("d")->value().toDouble());
        req->send(200, "text/plain", "OK");
    });

    // ── Control ───────────────────────────────────────────────────────────
    _server.on("/start",   HTTP_GET, [this](AsyncWebServerRequest* req) { if (_startCallback)   _startCallback();   req->send(200, "text/plain", "OK"); });
    _server.on("/stop",    HTTP_GET, [this](AsyncWebServerRequest* req) { if (_stopCallback)    _stopCallback();    req->send(200, "text/plain", "OK"); });
    _server.on("/restart", HTTP_GET, [this](AsyncWebServerRequest* req) { if (_restartCallback) _restartCallback(); req->send(200, "text/plain", "OK"); });
    _server.on("/calibrate", HTTP_GET, [this](AsyncWebServerRequest* req) { if (_calibrateCallback) _calibrateCallback(); req->send(200, "text/plain", "Calibrating..."); });

    _server.begin();

    // ── SSE telemetry push sub-task ───────────────────────────────────────
    // Pinned to Core 0 alongside AsyncWebServer. 4 KB stack is sufficient.
    // Skips push entirely when _events.count() == 0 (no browser connected).
    xTaskCreatePinnedToCore([](void* pv) {
        RobotServer* self = static_cast<RobotServer*>(pv);
        TickType_t lastWake = xTaskGetTickCount();
        for (;;) {
            self->pushTelemetry();
            vTaskDelayUntil(&lastWake, (1000 / TELEMETRY_HZ) / portTICK_PERIOD_MS);
        }
    }, "SSE_Push", 4096, this, 1, nullptr, 0);
}


// ─────────────────────────────────────────────────────────────────────────────
//  pushTelemetry() — called by SSE_Push at TELEMETRY_HZ
// ─────────────────────────────────────────────────────────────────────────────
void RobotServer::pushTelemetry()
{
    if (_events.count() == 0) return;

    auto fmt = [](const char* id, PIDController<float>* p) -> String {
        return String("\"") + id + "\":{"
            "\"t\":"   + String(p->getTarget(),   6) +
            ",\"o\":"  + String(p->getOutput(),   6) +
            ",\"e\":"  + String(p->getError(),    6) +
            ",\"f\":"  + String(p->getFeedback(), 6) +   // actual measured value (RPM for vel PIDs)
            ",\"p\":"  + String(p->getP(),        6) +
            ",\"iv\":" + String(p->getI(),        6) +
            ",\"dv\":" + String(p->getD(),        6) +
            "}";
    };

    String json = "{" +
        fmt("turn", _turn) + "," +
        fmt("pos",  _pos)  + "," +
        fmt("rvel", _rVel) + "," +
        fmt("lvel", _lVel) + "," +
        "\"dF\":" + String(getDistanceFront()) + "," +
        "\"dL\":" + String(getDistanceLeft())  + "," +
        "\"dR\":" + String(getDistanceRight()) +
    "}";

    _events.send(json.c_str(), "telemetry", millis());
}


// ─────────────────────────────────────────────────────────────────────────────
//  log() — thread-safe, callable from any core / task
// ─────────────────────────────────────────────────────────────────────────────
void RobotServer::log(const String& msg)
{
    String line = "[" + String(millis()) + "] " + msg;
    Serial.println(line);

    _events.send(line.c_str(), "log", millis());

    if (xSemaphoreTake(_logMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        _logBuffer[_logHead] = line;
        _logHead = (_logHead + 1) % LOG_RING_SIZE;
        if (_logCount < LOG_RING_SIZE) _logCount++;
        xSemaphoreGive(_logMutex);
    }
}