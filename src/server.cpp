#include "server.h"

RobotServer::RobotServer(PIDController<float>* turn, PIDController<float>* pos, PIDController<float>* rVel, PIDController<float>* lVel) 
    : _server(80), _turn(turn), _pos(pos), _rVel(rVel), _lVel(lVel) {}

void RobotServer::begin(const char* ssid, const char* password, void (*restartFunc)(), void (*posFunc)(int), void (*turnFunc)(float)) {
    _restartCallback = restartFunc;
    _posCallback = posFunc;
    _turnCallback = turnFunc;

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
    Serial.println("\nWiFi Connected: " + WiFi.localIP().toString());

    _server.on("/", HTTP_GET, [this](AsyncWebServerRequest *request){
        String html = "<html><head><title>PID Multi-Graph</title>";
        html += "<script src='https://cdn.jsdelivr.net/npm/chart.js'></script>";
        html += "<style>body{font-family:sans-serif; background:#121212; color:#eee; display:flex; margin:0;}";
        html += ".sidebar{width:300px; padding:20px; background:#1e1e1e; height:100vh; overflow-y:auto; border-right:1px solid #333;}";
        html += ".main{flex-grow:1; padding:20px; display:flex; flex-direction:column; gap:20px;}";
        html += ".card{background:#252525; padding:15px; border-radius:10px; border:1px solid #333;}";
        html += ".chart-container{height:200px; position:relative;}";
        html += "input{width:50px; background:#333; color:white; border:1px solid #555;}";
        html += "button{background:#007bff; color:white; border:none; padding:8px; border-radius:4px; cursor:pointer; margin:2px; font-size:12px;}";
        html += "button.cmd{background:#28a745; width:100%; margin-top:5px;} .restart{background:#dc3545; width:100%; font-weight:bold;}";
        html += "</style></head><body>";
        
        // SIDEBAR
        html += "<div class='sidebar'><button class='restart' onclick=\"fetch('/restart')\">RESET SYSTEM</button>";
        
        html += "<h3>Movement</h3>";
        html += "<button class='cmd' onclick=\"fetch('/pos?val=1')\">Move 1 Cell</button>";
        html += "<div style='display:flex; gap:5px;'> <input id='customPos' value='2'> <button class='cmd' onclick=\"fetch('/pos?val='+document.getElementById('customPos').value)\">Move X Cells</button></div>";
        html += "<hr style='border-color:#333'>";
        html += "<button class='cmd' onclick=\"fetch('/turn?val=-90')\">Turn Left (-90)</button>";
        html += "<button class='cmd' onclick=\"fetch('/turn?val=90')\">Turn Right (90)</button>";
        html += "<div style='display:flex; gap:5px;'> <input id='customTurn' value='180'> <button class='cmd' onclick=\"fetch('/turn?val='+document.getElementById('customTurn').value)\">Turn X Deg</button></div>";
        
        html += "<h3>PID Tuning</h3>";
        html += getParamHTML("Position", _pos, "pos");
        html += getParamHTML("Turn", _turn, "turn");
        html += getParamHTML("Velocity R", _rVel, "rvel");
        html += getParamHTML("Velocity L", _lVel, "lvel");
        html += "</div>";

        // MAIN CONTENT (3 GRAPHS)
        html += "<div class='main'>";
        html += "<div class='card'><h4>Position Error</h4><div class='chart-container'><canvas id='posChart'></canvas></div></div>";
        html += "<div class='card'><h4>Velocity Error (L/R)</h4><div class='chart-container'><canvas id='velChart'></canvas></div></div>";
        html += "<div class='card'><h4>Turn Error</h4><div class='chart-container'><canvas id='turnChart'></canvas></div></div>";
        html += "</div>";

        // JS LOGIC
        html += "<script>";
        html += "const commonOptions = { responsive:true, maintainAspectRatio:false, animation:false, scales:{y:{grid:{color:'#444',lineWidth:(ctx)=>ctx.value===0?2:1}}, x:{display:false}} };";
        
        html += "const posChart = new Chart(document.getElementById('posChart'), {type:'line', data:{labels:[], datasets:[{label:'Pos Err', borderColor:'#4bc0c0', data:[], borderWidth:2, pointRadius:0}]}, options:commonOptions});";
        html += "const velChart = new Chart(document.getElementById('velChart'), {type:'line', data:{labels:[], datasets:[{label:'L-Vel Err', borderColor:'#9966ff', data:[], borderWidth:2, pointRadius:0},{label:'R-Vel Err', borderColor:'#ffcd56', data:[], borderWidth:2, pointRadius:0}]}, options:commonOptions});";
        html += "const turnChart = new Chart(document.getElementById('turnChart'), {type:'line', data:{labels:[], datasets:[{label:'Turn Err', borderColor:'#ff6384', data:[], borderWidth:2, pointRadius:0}]}, options:commonOptions});";

        html += "setInterval(()=>{ fetch('/data').then(r=>r.json()).then(d=>{";
        html += "  const update = (chart, values) => { if(chart.data.labels.length>100){chart.data.labels.shift(); chart.data.datasets.forEach(ds=>ds.data.shift());} chart.data.labels.push(''); values.forEach((v,i)=>chart.data.datasets[i].data.push(v)); chart.update(); };";
        html += "  update(posChart, [d.pos.e]);";
        html += "  update(velChart, [d.lvel.e, d.rvel.e]);";
        html += "  update(turnChart, [d.turn.e]);";
        html += "  ['pos','turn','rvel','lvel'].forEach(id => { document.getElementById(id+'_t').innerText = d[id].t; document.getElementById(id+'_o').innerText = d[id].o; });";
        html += "});}, 100);</script></body></html>";
        
        request->send(200, "text/html", html);
    });

    // Endpoints for movements
    _server.on("/pos", HTTP_GET, [this](AsyncWebServerRequest *request){
        if(request->hasParam("val") && _posCallback) _posCallback(request->getParam("val")->value().toInt());
        request->send(200, "text/plain", "OK");
    });

    _server.on("/turn", HTTP_GET, [this](AsyncWebServerRequest *request){
        if(request->hasParam("val") && _turnCallback) _turnCallback(request->getParam("val")->value().toFloat());
        request->send(200, "text/plain", "OK");
    });

    // Existing endpoints (Data/Update/Restart)...
    _server.on("/data", HTTP_GET, [this](AsyncWebServerRequest *request){
        auto fmt = [](String id, PIDController<float>* p){ return "\""+id+"\":{\"t\":"+String(p->getTarget())+",\"o\":"+String(p->getOutput())+",\"e\":"+String(p->getError())+"}"; };
        request->send(200, "application/json", "{" + fmt("turn", _turn) + "," + fmt("pos", _pos) + "," + fmt("rvel", _rVel) + "," + fmt("lvel", _lVel) + "}");
    });
    
    _server.on("/update", HTTP_GET, [this](AsyncWebServerRequest *request){
        String id = request->getParam("id")->value();
        PIDController<float>* p = (id=="turn")?_turn:(id=="pos")?_pos:(id=="rvel")?_rVel:_lVel;
        p->setPID(request->getParam("p")->value().toDouble(), request->getParam("i")->value().toDouble(), request->getParam("d")->value().toDouble());
        request->send(200, "text/plain", "OK");
    });

    _server.on("/restart", HTTP_GET, [this](AsyncWebServerRequest *request){ if(_restartCallback) _restartCallback(); request->send(200, "text/plain", "OK"); });

    _server.begin();
}

String RobotServer::getParamHTML(String name, PIDController<float>* pid, String id) {
    String h = "<div style='font-size:0.85em; margin-bottom:10px;'><strong>"+name+"</strong><br>";
    h += "P<input id='"+id+"_p' value='"+String(pid->getP(),3)+"'> I<input id='"+id+"_i' value='"+String(pid->getI(),3)+"'> D<input id='"+id+"_d' value='"+String(pid->getD(),4)+"'><br>";
    h += "<button onclick=\"fetch('/update?id="+id+"&p='+document.getElementById('"+id+"_p').value+'&i='+document.getElementById('"+id+"_i').value+'&d='+document.getElementById('"+id+"_d').value)\">Set</button> ";
    h += "T:<span id='"+id+"_t'>-</span> O:<span id='"+id+"_o'>-</span></div>";
    return h;
}