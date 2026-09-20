#include "local_api.h"

namespace ac::api {

LocalAPIServer::LocalAPIServer(control::ACController& ac,
                               DHTDriver& dht,
                               sensors::PresenceSensorDriver& presence,
                               safety::SafetyManager& safety,
                               network::NetworkManager& network,
                               const String& deviceId)
    : _server(80),
      _ac(ac),
      _dht(dht),
      _presence(presence),
      _safety(safety),
      _network(network),
      _deviceId(deviceId) {}

void LocalAPIServer::begin(uint16_t port) {
    setupRoutes();
    _server.begin();
    Serial.printf("[HTTP] Local REST API and Web Dashboard started on port %u\n", port);
}

void LocalAPIServer::update() {
    _server.handleClient();
}

void LocalAPIServer::sendCORS() {
    _server.sendHeader("Access-Control-Allow-Origin", "*");
    _server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    _server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

void LocalAPIServer::setupRoutes() {
    // CORS Preflight
    _server.onNotFound([this]() {
        if (_server.method() == HTTP_OPTIONS) {
            sendCORS();
            _server.send(204);
        } else {
            _server.send(404, "text/plain", "Not Found");
        }
    });

    _server.on("/", HTTP_GET, [this]() { handleRoot(); });
    _server.on("/api/status", HTTP_GET, [this]() { handleStatus(); });
    _server.on("/api/telemetry", HTTP_GET, [this]() { handleTelemetry(); });
    _server.on("/api/ac/state", HTTP_GET, [this]() { handleGetState(); });
    _server.on("/api/ac/state", HTTP_POST, [this]() { handleSetState(); });
    _server.on("/api/ac/power", HTTP_POST, [this]() { handleSetPower(); });
    _server.on("/api/ac/temp", HTTP_POST, [this]() { handleSetTemperature(); });
    _server.on("/api/ac/fan", HTTP_POST, [this]() { handleSetFan(); });
}

void LocalAPIServer::handleRoot() {
    // Embedded HTML dashboard for zero-dependency browser access
    String html = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Azure Essence Smart AC</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif; }
    body { background: #0f172a; color: #f8fafc; padding: 20px; display: flex; justify-content: center; }
    .card { background: #1e293b; border-radius: 16px; padding: 24px; max-width: 440px; width: 100%; box-shadow: 0 10px 25px rgba(0,0,0,0.5); }
    h1 { font-size: 1.3rem; margin-bottom: 4px; color: #38bdf8; text-align: center; }
    .sub { text-align: center; color: #94a3b8; font-size: 0.85rem; margin-bottom: 20px; }
    .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; margin-bottom: 20px; }
    .metric { background: #0f172a; padding: 14px; border-radius: 12px; text-align: center; }
    .metric-val { font-size: 1.8rem; font-weight: bold; color: #f1f5f9; }
    .metric-lbl { font-size: 0.75rem; color: #64748b; margin-top: 4px; text-transform: uppercase; letter-spacing: 0.05em; }
    .temp-dial { background: #0f172a; border-radius: 16px; padding: 20px; text-align: center; margin-bottom: 20px; }
    .dial-title { font-size: 0.85rem; color: #94a3b8; margin-bottom: 8px; }
    .dial-val { font-size: 3.2rem; font-weight: 800; color: #38bdf8; }
    .dial-btns { display: flex; justify-content: center; gap: 16px; margin-top: 10px; }
    .btn-circle { width: 44px; height: 44px; border-radius: 50%; border: none; background: #334155; color: white; font-size: 1.4rem; cursor: pointer; }
    .btn-circle:active { background: #38bdf8; color: #0f172a; }
    .power-btn { width: 100%; padding: 14px; border-radius: 12px; border: none; font-size: 1.1rem; font-weight: bold; cursor: pointer; margin-bottom: 16px; transition: 0.2s; }
    .power-on { background: #10b981; color: white; }
    .power-off { background: #ef4444; color: white; }
    .sec-title { font-size: 0.8rem; color: #64748b; margin-bottom: 8px; text-transform: uppercase; }
    .btn-group { display: flex; gap: 8px; margin-bottom: 16px; }
    .btn-group button { flex: 1; padding: 10px; border-radius: 8px; border: 1px solid #334155; background: #1e293b; color: #cbd5e1; font-size: 0.85rem; cursor: pointer; }
    .btn-group button.active { background: #0284c7; color: white; border-color: #38bdf8; font-weight: bold; }
    .status-bar { font-size: 0.8rem; color: #64748b; text-align: center; border-top: 1px solid #334155; padding-top: 12px; }
  </style>
</head>
<body>
  <div class="card">
    <h1>Azure Essence AE-18</h1>
    <div class="sub">Smart Controller Retrofit</div>
    <div class="grid">
      <div class="metric"><div class="metric-val" id="roomTemp">--.-&deg;C</div><div class="metric-lbl">Room Temp (DHT22)</div></div>
      <div class="metric"><div class="metric-val" id="roomHum">--%</div><div class="metric-lbl">Humidity (DHT22)</div></div>
    </div>
    <button id="pwrBtn" class="power-btn power-off" onclick="togglePower()">Turn AC ON</button>
    <div class="temp-dial">
      <div class="dial-title">Target Temperature</div>
      <div class="dial-val" id="targetTemp">25&deg;C</div>
      <div class="dial-btns">
        <button class="btn-circle" onclick="adjTemp(-1)">&minus;</button>
        <button class="btn-circle" onclick="adjTemp(1)">&plus;</button>
      </div>
    </div>
    <div class="sec-title">Fan Speed</div>
    <div class="btn-group" id="fanGroup">
      <button onclick="setFan('auto')" id="fan-auto" class="active">Auto</button>
      <button onclick="setFan('med')" id="fan-med">Med</button>
      <button onclick="setFan('high')" id="fan-high">High</button>
    </div>
    <div class="status-bar" id="statusLine">Connecting to ESP32...</div>
  </div>
  <script>
    let curState = { power: false, temperature: 25, fan_speed: 'auto' };
    async function fetchStatus() {
      try {
        const res = await fetch('/api/status');
        const d = await res.json();
        if (d.climate && d.climate.valid) {
          document.getElementById('roomTemp').innerHTML = d.climate.temperature_c.toFixed(1) + '&deg;C';
          document.getElementById('roomHum').innerText = Math.round(d.climate.humidity_percent) + '%';
        }
        if (d.ac) {
          curState = d.ac;
          renderState();
        }
        document.getElementById('statusLine').innerText = 'Online | IP: ' + d.system.ip + ' | ' + (d.system.uptime_s) + 's uptime';
      } catch (e) {
        document.getElementById('statusLine').innerText = 'Offline / Waiting for signal...';
      }
    }
    function renderState() {
      const btn = document.getElementById('pwrBtn');
      if (curState.power) {
        btn.innerText = 'Turn AC OFF';
        btn.className = 'power-btn power-on';
      } else {
        btn.innerText = 'Turn AC ON';
        btn.className = 'power-btn power-off';
      }
      document.getElementById('targetTemp').innerHTML = curState.temperature + '&deg;C';
      ['auto','med','high'].forEach(f => {
        const el = document.getElementById('fan-' + f);
        if (el) el.className = (curState.fan_speed.toLowerCase() === f) ? 'active' : '';
      });
    }
    async function togglePower() {
      await fetch('/api/ac/power', { method: 'POST', headers: {'Content-Type':'application/json'}, body: JSON.stringify({ power: !curState.power }) });
      fetchStatus();
    }
    async function adjTemp(delta) {
      let t = curState.temperature + delta;
      if (t < 16) t = 16; if (t > 31) t = 31;
      await fetch('/api/ac/temp', { method: 'POST', headers: {'Content-Type':'application/json'}, body: JSON.stringify({ temp: t }) });
      fetchStatus();
    }
    async function setFan(spd) {
      await fetch('/api/ac/fan', { method: 'POST', headers: {'Content-Type':'application/json'}, body: JSON.stringify({ fan: spd }) });
      fetchStatus();
    }
    setInterval(fetchStatus, 2000);
    fetchStatus();
  </script>
</body>
</html>
)rawliteral";
    _server.send(200, "text/html", html);
}

void LocalAPIServer::handleStatus() {
    sendCORS();
    JsonDocument doc;

    doc["device_id"] = _deviceId;

    // Climate (DHT22)
    const DHTReading& dht = _dht.getLatestReading();
    JsonObject climate = doc["climate"].to<JsonObject>();
    climate["valid"] = dht.valid;
    climate["temperature_c"] = dht.valid ? dht.temperature_c : 0.0f;
    climate["humidity_percent"] = dht.valid ? dht.humidity_percent : 0.0f;
    climate["status"] = dht.status;

    // AC State
    const control::ACState& state = _ac.getState();
    JsonObject acObj = doc["ac"].to<JsonObject>();
    state.serializeTo(acObj);

    // System
    JsonObject sys = doc["system"].to<JsonObject>();
    sys["ip"] = _network.getIPAddress();
    sys["is_ap"] = _network.isAPMode();
    sys["rssi"] = _network.getRSSI();
    sys["uptime_s"] = millis() / 1000;
    sys["free_heap"] = ESP.getFreeHeap();

    String out;
    serializeJson(doc, out);
    _server.send(200, "application/json", out);
}

void LocalAPIServer::handleTelemetry() {
    sendCORS();
    const DHTReading& dht = _dht.getLatestReading();
    JsonDocument doc;
    doc["temperature_c"] = dht.valid ? dht.temperature_c : 0.0f;
    doc["humidity_percent"] = dht.valid ? dht.humidity_percent : 0.0f;
    doc["valid"] = dht.valid;
    doc["status"] = dht.status;
    doc["timestamp_ms"] = dht.timestamp_ms;

    String out;
    serializeJson(doc, out);
    _server.send(200, "application/json", out);
}

void LocalAPIServer::handleGetState() {
    sendCORS();
    _server.send(200, "application/json", _ac.getState().toJSONString());
}

void LocalAPIServer::handleSetState() {
    sendCORS();
    if (!_server.hasArg("plain")) {
        _server.send(400, "application/json", "{\"error\":\"Missing JSON body\"}");
        return;
    }
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, _server.arg("plain"));
    if (err) {
        _server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
        return;
    }

    if (!doc["power"].isNull()) _ac.setPower(doc["power"].as<bool>(), "rest_api");
    if (!doc["temperature"].isNull()) _ac.setTemperature(doc["temperature"].as<uint8_t>(), "rest_api");
    if (!doc["fan_speed"].isNull()) _ac.setFanSpeed(doc["fan_speed"].as<String>(), "rest_api");
    if (!doc["mode"].isNull()) _ac.setMode(doc["mode"].as<String>(), "rest_api");

    _server.send(200, "application/json", _ac.getState().toJSONString());
}

void LocalAPIServer::handleSetPower() {
    sendCORS();
    if (!_server.hasArg("plain")) {
        _server.send(400, "application/json", "{\"error\":\"Missing body\"}");
        return;
    }
    JsonDocument doc;
    deserializeJson(doc, _server.arg("plain"));
    bool pwr = doc["power"] | false;

    const char* reason = nullptr;
    bool allowed = pwr ? _safety.canTurnOn(millis(), reason) : _safety.canTurnOff(millis(), reason);
    if (!allowed) {
        _server.send(429, "application/json", String("{\"error\":\"") + (reason ? reason : "Safety violation") + "\"}");
        return;
    }

    _ac.setPower(pwr, "rest_api");
    _safety.recordPowerTransition(pwr, millis());
    _server.send(200, "application/json", _ac.getState().toJSONString());
}

void LocalAPIServer::handleSetTemperature() {
    sendCORS();
    if (!_server.hasArg("plain")) {
        _server.send(400, "application/json", "{\"error\":\"Missing body\"}");
        return;
    }
    JsonDocument doc;
    deserializeJson(doc, _server.arg("plain"));
    uint8_t temp = doc["temp"] | doc["temperature"] | 25;

    bool ok = _ac.setTemperature(temp, "rest_api");
    _safety.recordCommandSent(millis());
    if (ok) {
        _server.send(200, "application/json", _ac.getState().toJSONString());
    } else {
        _server.send(400, "application/json", "{\"error\":\"Temperature out of bounds (16-31C)\"}");
    }
}

void LocalAPIServer::handleSetFan() {
    sendCORS();
    if (!_server.hasArg("plain")) {
        _server.send(400, "application/json", "{\"error\":\"Missing body\"}");
        return;
    }
    JsonDocument doc;
    deserializeJson(doc, _server.arg("plain"));
    const char* spd = doc["fan"] | doc["fan_speed"] | "auto";

    bool ok = _ac.setFanSpeed(String(spd), "rest_api");
    _safety.recordCommandSent(millis());
    if (ok) {
        _server.send(200, "application/json", _ac.getState().toJSONString());
    } else {
        _server.send(400, "application/json", "{\"error\":\"Invalid fan speed\"}");
    }
}

} // namespace ac::api
