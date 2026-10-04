#include "local_api.h"

namespace ac::api {

LocalAPIServer::LocalAPIServer(control::ACController& ac,
                               DHTDriver& dht,
                               sensors::PresenceSensorDriver& presence,
                               safety::SafetyManager& safety,
                               network::NetworkManager& network,
                               automation::AutomationEngine& automation,
                               const String& deviceId)
    : _server(80),
      _ac(ac),
      _dht(dht),
      _presence(presence),
      _safety(safety),
      _network(network),
      _automation(automation),
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
    // Captive Portal Detection URLs (Android, iOS, Windows, ChromeOS)
    auto captiveHandler = [this]() {
        if (_network.isAPMode()) {
            _server.sendHeader("Location", "http://192.168.4.1/", true);
            _server.send(302, "text/plain", "");
        } else {
            _server.send(204);
        }
    };
    _server.on("/generate_204", HTTP_GET, captiveHandler);
    _server.on("/gen_204", HTTP_GET, captiveHandler);
    _server.on("/hotspot-detect.html", HTTP_GET, captiveHandler);
    _server.on("/canonical.html", HTTP_GET, captiveHandler);
    _server.on("/ncsi.txt", HTTP_GET, captiveHandler);
    _server.on("/connecttest.txt", HTTP_GET, captiveHandler);
    _server.on("/library/test/success.html", HTTP_GET, captiveHandler);
    _server.on("/success.txt", HTTP_GET, captiveHandler);

    // CORS Preflight & 404 / Captive Fallback
    _server.onNotFound([this]() {
        if (_server.method() == HTTP_OPTIONS) {
            sendCORS();
            _server.send(204);
            return;
        }
        if (_network.isAPMode()) {
            _server.sendHeader("Location", "http://192.168.4.1/", true);
            _server.send(302, "text/plain", "");
            return;
        }
        _server.send(404, "text/plain", "Not Found");
    });

    _server.on("/", HTTP_GET, [this]() { handleRoot(); });
    _server.on("/api/status", HTTP_GET, [this]() { handleStatus(); });
    _server.on("/api/telemetry", HTTP_GET, [this]() { handleTelemetry(); });
    _server.on("/api/ac/state", HTTP_GET, [this]() { handleGetState(); });
    _server.on("/api/ac/state", HTTP_POST, [this]() { handleSetState(); });
    _server.on("/api/ac/power", HTTP_POST, [this]() { handleSetPower(); });
    _server.on("/api/ac/temp", HTTP_POST, [this]() { handleSetTemperature(); });
    _server.on("/api/ac/fan", HTTP_POST, [this]() { handleSetFan(); });
    _server.on("/api/ac/mode", HTTP_POST, [this]() { handleSetMode(); });

    // Wi-Fi Onboarding & Configuration Routes
    _server.on("/api/wifi/scan", HTTP_GET, [this]() { handleWifiScan(); });
    _server.on("/api/wifi/configure", HTTP_POST, [this]() { handleWifiConfigure(); });
    _server.on("/api/wifi/status", HTTP_GET, [this]() { handleWifiStatus(); });

    // Automation & Night Sleep Cycle Routes
    _server.on("/api/automation/night-cycle", HTTP_GET, [this]() { handleNightCycle(); });
    _server.on("/api/automation/night-cycle", HTTP_POST, [this]() { handleNightCycle(); });
}

void LocalAPIServer::handleRoot() {
    // Embedded HTML dashboard for zero-dependency browser access and captive portal onboarding
    String html = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Smart Controller</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif; }
    body { background: #0f172a; color: #f8fafc; padding: 16px; display: flex; justify-content: center; }
    .card { background: #1e293b; border-radius: 16px; padding: 20px; max-width: 440px; width: 100%; box-shadow: 0 10px 25px rgba(0,0,0,0.5); margin-bottom: 16px; }
    h1 { font-size: 1.3rem; margin-bottom: 4px; color: #38bdf8; text-align: center; }
    .sub { text-align: center; color: #94a3b8; font-size: 0.85rem; margin-bottom: 16px; }
    .badge { display: inline-block; padding: 4px 10px; border-radius: 999px; font-size: 0.72rem; font-weight: 700; margin-bottom: 12px; }
    .badge-ap { background: rgba(245, 158, 11, 0.2); color: #fbbf24; border: 1px solid rgba(245, 158, 11, 0.4); }
    .badge-sta { background: rgba(16, 185, 129, 0.2); color: #34d399; border: 1px solid rgba(16, 185, 129, 0.4); }
    .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-bottom: 16px; }
    .metric { background: #0f172a; padding: 12px; border-radius: 10px; text-align: center; }
    .metric-val { font-size: 1.6rem; font-weight: bold; color: #f1f5f9; }
    .metric-lbl { font-size: 0.72rem; color: #64748b; margin-top: 4px; text-transform: uppercase; }
    .temp-dial { background: #0f172a; border-radius: 14px; padding: 16px; text-align: center; margin-bottom: 16px; }
    .dial-title { font-size: 0.8rem; color: #94a3b8; margin-bottom: 6px; }
    .dial-val { font-size: 2.8rem; font-weight: 800; color: #38bdf8; }
    .dial-btns { display: flex; justify-content: center; gap: 14px; margin-top: 8px; }
    .btn-circle { width: 42px; height: 42px; border-radius: 50%; border: none; background: #334155; color: white; font-size: 1.3rem; cursor: pointer; }
    .power-btn { width: 100%; padding: 12px; border-radius: 10px; border: none; font-size: 1rem; font-weight: bold; cursor: pointer; margin-bottom: 14px; }
    .power-on { background: #10b981; color: white; }
    .power-off { background: #ef4444; color: white; }
    .sec-title { font-size: 0.78rem; color: #64748b; margin-bottom: 6px; text-transform: uppercase; font-weight: 700; }
    .btn-group { display: flex; gap: 6px; margin-bottom: 14px; }
    .btn-group button { flex: 1; padding: 8px; border-radius: 8px; border: 1px solid #334155; background: #1e293b; color: #cbd5e1; font-size: 0.8rem; cursor: pointer; }
    .btn-group button.active { background: #0284c7; color: white; border-color: #38bdf8; font-weight: bold; }
    .status-bar { font-size: 0.75rem; color: #64748b; text-align: center; border-top: 1px solid #334155; padding-top: 10px; margin-top: 10px; }
    
    /* Wi-Fi Setup Box */
    .wifi-section { background: #0f172a; border-radius: 14px; padding: 16px; margin-top: 16px; border: 1px solid #334155; }
    .wifi-title { font-size: 0.95rem; font-weight: 700; color: #38bdf8; margin-bottom: 10px; display: flex; justify-content: space-between; align-items: center; }
    .input-group { margin-bottom: 10px; }
    .input-lbl { font-size: 0.72rem; color: #94a3b8; margin-bottom: 4px; display: block; }
    .input-box, select.input-box { width: 100%; padding: 10px; border-radius: 8px; border: 1px solid #334155; background: #1e293b; color: #f8fafc; font-size: 0.85rem; outline: none; }
    .btn-action { width: 100%; padding: 10px; border-radius: 8px; border: none; background: #0284c7; color: white; font-weight: 700; font-size: 0.85rem; cursor: pointer; margin-top: 6px; }
    .btn-scan { background: #334155; color: #38bdf8; padding: 6px 12px; font-size: 0.75rem; border-radius: 6px; border: 1px solid #475569; cursor: pointer; }
    .msg-box { font-size: 0.75rem; padding: 8px; border-radius: 6px; margin-top: 8px; display: none; line-height: 1.4; }
    .msg-ok { background: rgba(16, 185, 129, 0.15); color: #34d399; border: 1px solid rgba(16, 185, 129, 0.3); }
    .msg-err { background: rgba(239, 68, 68, 0.15); color: #f87171; border: 1px solid rgba(239, 68, 68, 0.3); }
    .msg-info { background: rgba(56, 189, 248, 0.15); color: #38bdf8; border: 1px solid rgba(56, 189, 248, 0.3); }
  </style>
</head>
<body>
  <div style="max-width: 440px; width: 100%;">
    <div class="card">
      <div style="text-align: center;">
        <span id="wifiBadge" class="badge badge-ap">AP Mode: SmartController-Setup</span>
      </div>
      <h1>Smart Controller</h1>
      <div class="sub">Universal Smart AC Controller</div>

      <!-- Live Climate Metrics -->
      <div class="grid">
        <div class="metric"><div class="metric-val" id="roomTemp">--.-&deg;C</div><div class="metric-lbl">Room Temp</div></div>
        <div class="metric"><div class="metric-val" id="roomHum">--%</div><div class="metric-lbl">Humidity</div></div>
      </div>

      <!-- AC Controls -->
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

      <!-- Wi-Fi Network Setup Card -->
      <div class="wifi-section">
        <div class="wifi-title">
          <span>Wi-Fi Network Setup</span>
          <button class="btn-scan" id="btnScan" onclick="scanWifi()">Scan Networks</button>
        </div>
        
        <div class="input-group">
          <label class="input-lbl" for="wifiSelect">Discovered Networks</label>
          <select id="wifiSelect" class="input-box" onchange="onSelectNetwork()">
            <option value="">-- Tap 'Scan Networks' to search --</option>
          </select>
        </div>

        <div class="input-group">
          <label class="input-lbl" for="ssidInput">Network SSID</label>
          <input type="text" id="ssidInput" class="input-box" placeholder="e.g. MyHome-WiFi">
        </div>

        <div class="input-group">
          <label class="input-lbl" for="passInput">Wi-Fi Password</label>
          <div style="display: flex; gap: 6px;">
            <input type="password" id="passInput" class="input-box" placeholder="Enter Wi-Fi password">
            <button type="button" class="btn-scan" style="padding: 0 10px;" onclick="togglePass()">Show</button>
          </div>
        </div>

        <button class="btn-action" id="btnConnect" onclick="saveAndConnectWifi()">Save &amp; Connect to Wi-Fi</button>
        <div id="wifiMsg" class="msg-box"></div>
      </div>

      <div class="status-bar" id="statusLine">Connecting to ESP32...</div>
    </div>
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
        const b = document.getElementById('wifiBadge');
        if (d.system) {
          if (d.system.is_ap) {
            b.className = 'badge badge-ap';
            b.innerText = 'AP Mode: SmartController-Setup (192.168.4.1)';
          } else {
            b.className = 'badge badge-sta';
            b.innerText = 'Connected: ' + (d.system.ip || 'Wi-Fi') + ' (' + d.system.rssi + ' dBm)';
          }
          document.getElementById('statusLine').innerText = 'IP: ' + d.system.ip + ' | Heap: ' + Math.round(d.system.free_heap/1024) + 'KB | ' + d.system.uptime_s + 's up';
        }
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

    function togglePass() {
      const p = document.getElementById('passInput');
      p.type = (p.type === 'password') ? 'text' : 'password';
    }

    async function scanWifi() {
      const b = document.getElementById('btnScan');
      b.innerText = 'Scanning...';
      b.disabled = true;
      try {
        const res = await fetch('/api/wifi/scan');
        const list = await res.json();
        const sel = document.getElementById('wifiSelect');
        sel.innerHTML = '<option value="">-- Choose a network (' + list.length + ' found) --</option>';
        list.forEach(n => {
          const opt = document.createElement('option');
          opt.value = n.ssid;
          opt.innerText = n.ssid + ' (' + n.rssi + ' dBm, ' + (n.secure ? 'Secured' : 'Open') + ')';
          sel.appendChild(opt);
        });
      } catch (err) {
        showMsg('Scan failed: ' + err.message, 'err');
      } finally {
        b.innerText = 'Scan Networks';
        b.disabled = false;
      }
    }

    function onSelectNetwork() {
      const sel = document.getElementById('wifiSelect');
      if (sel.value) {
        document.getElementById('ssidInput').value = sel.value;
        document.getElementById('passInput').focus();
      }
    }

    async function saveAndConnectWifi() {
      const ssid = document.getElementById('ssidInput').value.trim();
      const pass = document.getElementById('passInput').value;
      if (!ssid) {
        showMsg('Please select or type a Wi-Fi SSID', 'err');
        return;
      }
      showMsg('Saving credentials and connecting to "' + ssid + '"... The controller will reboot/reconnect. If in AP mode, connect to your router and visit http://smart-ac.local', 'info');
      const b = document.getElementById('btnConnect');
      b.disabled = true;
      b.innerText = 'Connecting...';
      try {
        const res = await fetch('/api/wifi/configure', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ ssid: ssid, password: pass })
        });
        const d = await res.json();
        if (d.success) {
          showMsg('Success! Connected/saved credentials for ' + ssid + '. You can now access http://smart-ac.local on your Wi-Fi.', 'ok');
        } else {
          showMsg('Error: ' + (d.error || 'Failed to save'), 'err');
        }
      } catch (err) {
        showMsg('Request dispatched. Controller may have switched networks. Reconnect your device to ' + ssid + ' and open http://smart-ac.local', 'ok');
      } finally {
        b.disabled = false;
        b.innerText = 'Save & Connect to Wi-Fi';
        setTimeout(fetchStatus, 3000);
      }
    }

    function showMsg(text, type) {
      const el = document.getElementById('wifiMsg');
      el.style.display = 'block';
      el.className = 'msg-box msg-' + type;
      el.innerText = text;
    }

    setInterval(fetchStatus, 2500);
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

    // Automation State & Night Cycle
    JsonDocument autoDoc;
    _automation.toJSON(autoDoc);
    doc["automation"] = autoDoc;

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

    bool pwr = !doc["power"].isNull() ? doc["power"].as<bool>() : _ac.getState().power;
    uint8_t temp = !doc["temperature"].isNull() ? doc["temperature"].as<uint8_t>() : _ac.getState().temperature;
    String mode = !doc["mode"].isNull() ? doc["mode"].as<String>() : _ac.getState().mode;
    String fan = !doc["fan_speed"].isNull() ? doc["fan_speed"].as<String>() : _ac.getState().fanSpeed;

    _ac.setState(pwr, temp, mode, fan, "rest_api");
    if (!doc["power"].isNull()) {
        _safety.recordPowerTransition(pwr, millis());
    }
    _safety.recordCommandSent(millis());

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

void LocalAPIServer::handleSetMode() {
    sendCORS();
    if (!_server.hasArg("plain")) {
        _server.send(400, "application/json", "{\"error\":\"Missing body\"}");
        return;
    }
    JsonDocument doc;
    deserializeJson(doc, _server.arg("plain"));
    const char* m = doc["mode"] | "cool";

    bool ok = _ac.setMode(String(m), "rest_api");
    _safety.recordCommandSent(millis());
    if (ok) {
        _server.send(200, "application/json", _ac.getState().toJSONString());
    } else {
        _server.send(400, "application/json", "{\"error\":\"Invalid mode (cool, dry, fan, auto)\"}");
    }
}

void LocalAPIServer::handleWifiScan() {
    sendCORS();
    std::vector<network::ScannedNetwork> networks = _network.scanNetworks();
    JsonDocument doc;
    JsonArray arr = doc.to<JsonArray>();
    for (const auto& net : networks) {
        JsonObject obj = arr.add<JsonObject>();
        obj["ssid"] = net.ssid;
        obj["rssi"] = net.rssi;
        obj["secure"] = net.isSecure;
        obj["auth"] = net.authMode;
    }
    String out;
    serializeJson(doc, out);
    _server.send(200, "application/json", out);
}

void LocalAPIServer::handleWifiConfigure() {
    sendCORS();
    if (!_server.hasArg("plain")) {
        _server.send(400, "application/json", "{\"error\":\"Missing body\"}");
        return;
    }
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, _server.arg("plain"));
    if (err) {
        _server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
        return;
    }

    String ssid = doc["ssid"] | "";
    String pass = doc["password"] | "";
    ssid.trim();

    if (ssid.length() == 0) {
        _server.send(400, "application/json", "{\"error\":\"SSID cannot be empty\"}");
        return;
    }

    bool ok = _network.configureWifi(ssid, pass);
    if (ok) {
        _server.send(200, "application/json", "{\"success\":true,\"message\":\"Credentials saved. Connecting to network...\"}");
    } else {
        _server.send(500, "application/json", "{\"error\":\"Failed to save credentials\"}");
    }
}

void LocalAPIServer::handleWifiStatus() {
    sendCORS();
    JsonDocument doc;
    doc["connected"] = (WiFi.status() == WL_CONNECTED);
    doc["is_ap"] = _network.isAPMode();
    doc["ssid"] = _network.getSSID();
    doc["ip"] = _network.getIPAddress();
    doc["rssi"] = _network.getRSSI();
    doc["hostname"] = _network.getHostname();
    doc["mac"] = WiFi.macAddress();
    String out;
    serializeJson(doc, out);
    _server.send(200, "application/json", out);
}

void LocalAPIServer::handleNightCycle() {
    sendCORS();
    if (_server.method() == HTTP_POST) {
        if (_server.hasArg("plain")) {
            JsonDocument doc;
            DeserializationError err = deserializeJson(doc, _server.arg("plain"));
            if (!err) {
                bool enabled = doc["enabled"] | true;
                float targetTemp = doc["target_temp"] | 27.0f;
                int8_t h = doc["hour"] | -1;
                int8_t m = doc["min"] | -1;
                if (doc["epoch"].is<uint32_t>()) {
                    uint32_t ep = doc["epoch"].as<uint32_t>();
                    if (ep > 1700000000) {
                        timeval tv = { .tv_sec = (time_t)ep, .tv_usec = 0 };
                        settimeofday(&tv, nullptr);
                    }
                }
                _automation.setNightCycle(enabled, targetTemp, h, m);
            }
        } else {
            _automation.setNightCycle(!_automation.isNightCycleEnabled(), 27.0f);
        }
    }

    JsonDocument resp;
    resp["enabled"] = _automation.isNightCycleEnabled();
    resp["stage"] = (uint8_t)_automation.getNightCycleStage();
    resp["stage_str"] = _automation.getNightCycleStageStr();
    resp["target_temp"] = 27.0f;
    resp["stage_remaining_s"] = _automation.getNightCycleStageRemainingSec();
    resp["ac_power"] = _ac.getState().power;
    const DHTReading& dht = _dht.getLatestReading();
    resp["room_temp"] = dht.valid ? dht.temperature_c : 0.0f;

    uint8_t curH = 0, curM = 0, curS = 0;
    bool hasClock = _automation.getLocalTime(curH, curM, curS);
    resp["clock_synced"] = hasClock;
    char timeBuf[16];
    snprintf(timeBuf, sizeof(timeBuf), "%02u:%02u:%02u", curH, curM, curS);
    resp["local_time"] = timeBuf;

    String out;
    serializeJson(resp, out);
    _server.send(200, "application/json", out);
}

} // namespace ac::api
