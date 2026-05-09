#include <main.h>
#include <HardwareSerial.h>

// WiFi credentials for ESP32 Access Point
const char* ssid = "KAYAK_UI";
const char* password = "12345678";

WebServer server(80);
UIdata currentCommand;
bool radarEnabled = false;

GPSDataStruct latestGps;
bool gpsValid = false;
unsigned long lastGpsMillis = 0;

static const uint8_t GPS_START_BYTE = 0xAA;
static const size_t GPS_PAYLOAD_LEN = 84;
uint8_t gpsPayload[GPS_PAYLOAD_LEN];
size_t gpsPayloadIndex = 0;
bool gpsSync = false;

unsigned long lastBlink = 0;

// HTML Root
void handleRoot() {
  String html = "<!DOCTYPE html>";
  html += "<html>";
  html += "<head>";
  html += "<title>Kayak Control</title>";
  html += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";
  html += "</head>";
  html += "<body>";
  html += "<div class=\"container\">";
  html += "<div class=\"header\">";
  html += "<div class=\"header_left\">";
  html += "<h1 class=\"header-text\">Automated Kayak</h1>";
  html += "</div>";
  html += "<div class=\"header_right\">";
  html += "<h3 id=\"datetime\">Loading...</h3>";
  html += "</div>";
  html += "</div>";
  html += "<div class=\"body_left\">";
  html += "<h1 style=\"text-align:center;\">Command Buttons</h1>";
  html += "<hr style=\"height:2px;border-width:0;color:black;background-color:black\">";
  html += "<button class=\"button mode-btn\" onclick=\"selectMode(0)\">Disable</button>";
  html += "<button class=\"button mode-btn\" onclick=\"selectMode(1)\">Manual Move</button>";
  html += "<button class=\"button mode-btn\" onclick=\"selectMode(2)\">Anchor</button>";
  html += "<button class=\"button mode-btn\" onclick=\"selectMode(3)\">Follow Shoreline</button>";
  html += "<button class=\"button mode-btn\" onclick=\"selectMode(4)\">Manual Motors</button>";

  // Direction section (mode 1 only)
  html += "<div id=\"dirSection\" style=\"display:none; margin-top:20px;\">";
  html += "<h2 style=\"text-align:center; color:white;\">Direction:</h2>";
  html += "<button class=\"button dir-btn\" onclick=\"selectDir(2)\">Forward</button>";
  html += "<button class=\"button dir-btn\" onclick=\"selectDir(3)\">Reverse</button>";
  html += "<button class=\"button dir-btn\" onclick=\"selectDir(0)\">Turn Left</button>";
  html += "<button class=\"button dir-btn\" onclick=\"selectDir(1)\">Turn Right</button>";
  html += "</div>";

  // Global speed section (modes 1 and 3)
  html += "<div id=\"speedSection\" style=\"display:none; margin-top:20px;\">";
  html += "<h2 style=\"text-align:center; color:white;\">Speed: <span id=\"speedVal\">0</span>%</h2>";
  html += "<input type=\"range\" min=\"0\" max=\"100\" value=\"0\" id=\"speedSlider\" style=\"width:90%; height:25px;\">";
  html += "</div>";

  // Individual motor speed section (mode 4 only)
  html += "<div id=\"motorSection\" style=\"display:none; margin-top:20px;\">";
  html += "<h2 style=\"text-align:center; color:white;\">Motor Speeds</h2>";

  html += "<div class=\"motor-row\">";
  html += "<label class=\"motor-label\">Front Left (M1): <span id=\"m1Val\">0</span>%</label>";
  html += "<input type=\"range\" min=\"0\" max=\"100\" value=\"0\" id=\"m1Slider\" class=\"motor-slider\" oninput=\"updateMotor(1,this.value)\">";
  html += "</div>";

  html += "<div class=\"motor-row\">";
  html += "<label class=\"motor-label\">Front Right (M2): <span id=\"m2Val\">0</span>%</label>";
  html += "<input type=\"range\" min=\"0\" max=\"100\" value=\"0\" id=\"m2Slider\" class=\"motor-slider\" oninput=\"updateMotor(2,this.value)\">";
  html += "</div>";

  html += "<div class=\"motor-row\">";
  html += "<label class=\"motor-label\">Rear Left (M3): <span id=\"m3Val\">0</span>%</label>";
  html += "<input type=\"range\" min=\"0\" max=\"100\" value=\"0\" id=\"m3Slider\" class=\"motor-slider\" oninput=\"updateMotor(3,this.value)\">";
  html += "</div>";

  html += "<div class=\"motor-row\">";
  html += "<label class=\"motor-label\">Rear Right (M4): <span id=\"m4Val\">0</span>%</label>";
  html += "<input type=\"range\" min=\"0\" max=\"100\" value=\"0\" id=\"m4Slider\" class=\"motor-slider\" oninput=\"updateMotor(4,this.value)\">";
  html += "</div>";

  html += "<button class=\"button\" onclick=\"zeroAllMotors()\" style=\"margin-top:10px; background-color:#800000;\">Zero All Motors</button>";
  html += "</div>";

  html += "<button class=\"button buttonEXE\" onclick=\"executeCommand()\" style=\"margin-top:30px; background-color:#ff6600;\">EXECUTE COMMAND</button>";
  html += "<h2 style=\"text-align:center; color:white; margin-top:20px;\" id=\"feedback\">Select a mode to begin</h2>";
  html += "</div>";

  html += "<div class=\"body_right\">";
  html += "<h1 style=\"text-align:center;\">Subsystem Status</h1>";
  html += "<hr style=\"height:2px;border-width:0;color:black;background-color:black\">";
  html += "<h1 style=\"text-align:left;\">Power: <span style=\"color:#00ff00;\">ON</span></h1>";
  html += "<h1 style=\"text-align:left;\">Motors: <span style=\"color:#00ff00;\">ON</span></h1>";
  html += "<h1 style=\"text-align:left;\">GPS: <span style=\"color:#00ff00;\">ON</span></h1>";
  html += "<div id=\"gpsInfo\" style=\"margin-left:10px; margin-bottom:10px; color:white;\">";
  html += "<div>Pos N (deg): <span id=\"gpsPosN\">--</span></div>";
  html += "<div>Pos E (deg): <span id=\"gpsPosE\">--</span></div>";
  html += "<div>Elev (m): <span id=\"gpsPosD\">--</span></div>";
  html += "<div>Vel N (mm/s): <span id=\"gpsVelN\">--</span></div>";
  html += "<div>Vel E (mm/s): <span id=\"gpsVelE\">--</span></div>";
  html += "<div>Vel D (mm/s): <span id=\"gpsVelD\">--</span></div>";
  html += "<div>Yaw (deg): <span id=\"gpsYaw\">--</span></div>";
  html += "<div>UTC: <span id=\"gpsUtc\">--</span></div>";
  html += "<div>Device ID: <span id=\"gpsId\">--</span></div>";
  html += "<div>Age (ms): <span id=\"gpsAge\">--</span></div>";
  html += "</div>";
  html += "<h1 style=\"text-align:left;\">Sonar: <span style=\"color:#00ff00;\">ON</span></h1>";
  html += "<h1 style=\"text-align:left;\">Radar: <span id=\"radarStatus\" style=\"color:#ff3333;\">OFF</span></h1>";
  html += "<button class=\"button\" id=\"radarToggleBtn\" onclick=\"toggleRadar()\" style=\"margin-top:10px; background-color:#0066cc;\">RADAR: OFF</button>";
  html += "</div>";
  html += "</div>";

  // ── JavaScript ──────────────────────────────────────────────────────────────
  html += "<script>";
  html += "var currentMode = null;";
  html += "var currentSpeed = 0;";
  html += "var currentDir = null;";
  html += "var motorSpeeds = [0, 0, 0, 0];"; // index 0-3 = M1-M4
  html += "var radarOn = false;";

  html += "function updateTime() {";
  html += "  var now = new Date();";
  html += "  document.getElementById('datetime').textContent = now.toLocaleString();";
  html += "}";
  html += "setInterval(updateTime, 1000);";
  html += "updateTime();";

  html += "function updateGps() {";
  html += "  fetch('/gps')";
  html += "    .then(function(r) { return r.json(); })";
  html += "    .then(function(d) {";
  html += "      document.getElementById('gpsPosN').textContent = d.pos_n;";
  html += "      document.getElementById('gpsPosE').textContent = d.pos_e;";
  html += "      document.getElementById('gpsPosD').textContent = d.pos_d;";
  html += "      document.getElementById('gpsVelN').textContent = d.vel_n;";
  html += "      document.getElementById('gpsVelE').textContent = d.vel_e;";
  html += "      document.getElementById('gpsVelD').textContent = d.vel_d;";
  html += "      document.getElementById('gpsYaw').textContent = d.yaw;";
  html += "      document.getElementById('gpsUtc').textContent = d.utc;";
  html += "      document.getElementById('gpsId').textContent = d.device_id;";
  html += "      document.getElementById('gpsAge').textContent = d.age_ms;";
  html += "    })";
  html += "    .catch(function() {});";
  html += "}";
  html += "setInterval(updateGps, 1000);";
  html += "updateGps();";

  html += "function updateMotor(num, val) {";
  html += "  motorSpeeds[num - 1] = parseInt(val);";
  html += "  document.getElementById('m' + num + 'Val').textContent = val;";
  html += "}";

  html += "function zeroAllMotors() {";
  html += "  for(var i = 1; i <= 4; i++) {";
  html += "    motorSpeeds[i-1] = 0;";
  html += "    document.getElementById('m' + i + 'Slider').value = 0;";
  html += "    document.getElementById('m' + i + 'Val').textContent = 0;";
  html += "  }";
  html += "}";

  html += "function selectMode(mode) {";
  html += "  currentMode = mode;";
  html += "  var modeBtns = document.querySelectorAll('.mode-btn');";
  html += "  for(var i = 0; i < modeBtns.length; i++) {";
  html += "  modeBtns[i].style.backgroundColor = '';";
  html += "    modeBtns[i].style.color = '';";
  html += "  }";
  html += "  event.target.style.backgroundColor = 'white';";
  html += "  event.target.style.color = '#cc0000';";
  html += "  document.getElementById('dirSection').style.display    = 'none';";
  html += "  document.getElementById('speedSection').style.display  = 'none';";
  html += "  document.getElementById('motorSection').style.display  = 'none';";
  html += "  if(mode == 0) {";
  html += "    currentSpeed = 0; currentDir = 2;";
  html += "    document.getElementById('feedback').textContent = 'System disabled';";
  html += "  } else if(mode == 1) {";
  html += "    document.getElementById('dirSection').style.display   = 'block';";
  html += "    document.getElementById('speedSection').style.display = 'block';";
  html += "    document.getElementById('feedback').textContent = 'Select direction and speed';";
  html += "  } else if(mode == 2) {";
  html += "    currentSpeed = 0; currentDir = 2;";
  html += "    document.getElementById('feedback').textContent = 'Ready to anchor';";
  html += "  } else if(mode == 3) {";
  html += "    document.getElementById('speedSection').style.display = 'block';";
  html += "    currentDir = 2;";
  html += "    document.getElementById('feedback').textContent = 'Set speed for shoreline following';";
  html += "  } else if(mode == 4) {";
  html += "    document.getElementById('motorSection').style.display = 'block';";
  html += "    document.getElementById('feedback').textContent = 'Set individual motor speeds';";
  html += "  }";
  html += "}";

  html += "function selectDir(dir) {";
  html += "  currentDir = dir;";
  html += "  var dirBtns = document.querySelectorAll('.dir-btn');";
  html += "  for(var i = 0; i < dirBtns.length; i++) {";
  html += "  dirBtns[i].style.backgroundColor = '';";
  html += "    dirBtns[i].style.color = '';";
  html += "  }";
  html += "  event.target.style.backgroundColor = 'white';";
  html += "  event.target.style.color = '#cc0000';";
  html += "  var dirNames = ['LEFT', 'RIGHT', 'FORWARD', 'REVERSE'];";
  html += "  document.getElementById('feedback').textContent = 'Direction: ' + dirNames[dir] + ', Speed: ' + currentSpeed;";
  html += "}";

  html += "document.getElementById('speedSlider').addEventListener('input', function() {";
  html += "  currentSpeed = parseInt(this.value);";
  html += "  document.getElementById('speedVal').textContent = currentSpeed;";
  html += "});";

  html += "function executeCommand() {";
  html += "  if(currentMode === null) {";
  html += "    document.getElementById('feedback').textContent = 'ERROR: Select a mode!';";
  html += "    return;";
  html += "  }";
  html += "  if(currentMode == 1 && currentDir === null) {";
  html += "    document.getElementById('feedback').textContent = 'ERROR: Select a direction!';";
  html += "    return;";
  html += "  }";
  html += "  var finalDir = (currentDir !== null) ? currentDir : 2;";
  html += "  var url = '/command?mode=' + currentMode";
  html += "           + '&speed=' + currentSpeed";
  html += "           + '&dir='   + finalDir";
  html += "           + '&m1='    + motorSpeeds[0]";
  html += "           + '&m2='    + motorSpeeds[1]";
  html += "           + '&m3='    + motorSpeeds[2]";
  html += "           + '&m4='    + motorSpeeds[3];";
  html += "  document.getElementById('feedback').textContent = 'Sending command...';";
  html += "  fetch(url)";
  html += "    .then(function(r) { return r.text(); })";
  html += "    .then(function(d) {";
  html += "      document.getElementById('feedback').textContent = 'Command executed successfully!';";
  html += "    })";
  html += "    .catch(function(e) {";
  html += "      document.getElementById('feedback').textContent = 'ERROR: Connection failed';";
  html += "    });";
  html += "}";
  html += "function setRadarStatus(isOn) {";
  html += "  radarOn = isOn;";
  html += "  var status = document.getElementById('radarStatus');";
  html += "  var btn = document.getElementById('radarToggleBtn');";
  html += "  status.textContent = isOn ? 'ON' : 'OFF';";
  html += "  status.style.color = isOn ? '#00ff00' : '#ff3333';";
  html += "  btn.textContent = isOn ? 'RADAR: ON' : 'RADAR: OFF';";
  html += "}";
  html += "function toggleRadar() {";
  html += "  var nextState = !radarOn;";
  html += "  fetch('/radar?on=' + (nextState ? 1 : 0))";
  html += "    .then(function(r) { return r.text(); })";
  html += "    .then(function() { setRadarStatus(nextState); })";
  html += "    .catch(function() {});";
  html += "}";
  html += "</script>";

  // ── CSS ─────────────────────────────────────────────────────────────────────
  html += "<style>";
  html += "body { margin:0; padding:0; background:#1a1a1a; }";
  html += ".container {";
  html += "  display:grid;";
  html += "  grid-template-columns:1fr;";
  html += "  grid-template-rows:auto auto auto;";
  html += "  min-height:100vh;";
  html += "}";
  html += ".header { background:#cc0000; padding:15px; position:relative; border-bottom:4px solid black; }";
  html += ".header_left { text-align:center; padding-right:0; }";
  html += ".header-text { color:white; font-size:24px; margin:5px 0; font-weight:bold; letter-spacing:2px; text-transform:uppercase; }";
  html += ".header_right {";
  html += "  display:flex; align-items:center; justify-content:center;";
  html += "  gap:10px; margin-top:10px;";
  html += "}";
  html += ".header_right h3 { color:white; font-size:14px; margin:0; }";
  html += ".body_left { background:#1a1a1a; padding:20px; overflow-y:auto; border-right:2px solid #cc0000; }";
  html += ".body_left h1, .body_left h2 { color:white; }";
  html += ".body_right { background:#1a1a1a; padding:20px; overflow-y:auto; }";
  html += ".body_right h1 { color:white; font-size:20px; }";
  html += ".button {";
  html += "  border:2px solid black; color:white; background-color:#cc0000;";
  html += "  width:100%; text-align:center; font-size:18px;";
  html += "  margin:8px 0; padding:15px; cursor:pointer; font-weight:bold; text-transform:uppercase; letter-spacing:1px;";
  html += "}";
  html += ".button:hover { background-color:#ff1a1a; }";
  html += ".buttonEXE { background-color:white !important; color:#cc0000 !important; border:3px solid #cc0000 !important; font-weight:bold; font-size:20px; }";
  html += ".buttonEXE:hover { background-color:#f0f0f0 !important; }";

  // Motor slider rows
  html += ".motor-row { margin: 12px 0; }";
  html += ".motor-label { color:white; font-size:16px; display:block; margin-bottom:4px; }";
  html += ".motor-slider { width:90%; height:25px; display:block; accent-color:#cc0000; }";
  html += "input[type=range] { accent-color:#cc0000; }";
  html += "hr { border-color:#cc0000 !important; background-color:#cc0000 !important; }";

  // Responsive layout
  html += "@media (min-width:768px) {";
  html += "  .container { grid-template-columns:1fr 1fr; grid-template-rows:120px 1fr; }";
  html += "  .header { grid-area:1/1/2/3; }";
  html += "  .header_left { text-align:left; padding-right:150px; }";
  html += "  .header-text { font-size:32px; }";
  html += "  .header_right {";
  html += "    position:absolute; right:4%; top:50%; transform:translateY(-50%);";
  html += "    flex-direction:column; align-items:flex-end; margin-top:0;";
  html += "  }";
  html += "  .header_right h3 { font-size:16px; }";
  html += "  .body_left { grid-area:2/1/3/2; }";
  html += "  .body_right { grid-area:2/2/3/3; }";
  html += "}";
  html += "</style>";

  html += "</body></html>";
  server.send(200, "text/html", html);
}

// Handle commands from web interface
void handleCommand() {
  // Parse URL parameters
  int mode  = server.arg("mode").toInt();
  int dir   = server.arg("dir").toInt();
  int speed = server.arg("speed").toInt();
  int m1    = server.arg("m1").toInt();
  int m2    = server.arg("m2").toInt();
  int m3    = server.arg("m3").toInt();
  int m4    = server.arg("m4").toInt();

  // Populate UIdata struct
  currentCommand.mode            = (operatingMode)mode;
  currentCommand.directionToTurn = (direction)dir;
  currentCommand.speed           = (uint8_t)speed;
  currentCommand.motor1          = (uint8_t)m1;
  currentCommand.motor2          = (uint8_t)m2;
  currentCommand.motor3          = (uint8_t)m3;
  currentCommand.motor4          = (uint8_t)m4;

  // ── UART packet (8 bytes, motor state) ─────────────────────────────────
  // Byte 0: start byte (0x69)
  // Byte 1: mode
  // Byte 2: direction  (0 for mode 4)
  // Byte 3: speed      (0 for mode 4)
  // Byte 4: motor1     (0 for modes 0-3)
  // Byte 5: motor2     (0 for modes 0-3)
  // Byte 6: motor3     (0 for modes 0-3)
  // Byte 7: motor4     (0 for modes 0-3)
  Serial2.write((uint8_t)0x69);
  Serial2.write((uint8_t)currentCommand.mode);
  Serial2.write((uint8_t)currentCommand.directionToTurn);
  Serial2.write(currentCommand.speed);
  Serial2.write(currentCommand.motor1);
  Serial2.write(currentCommand.motor2);
  Serial2.write(currentCommand.motor3);
  Serial2.write(currentCommand.motor4);

  server.send(200, "text/plain", "OK");
}

void handleRadarCommand() {
  int on = server.arg("on").toInt();
  radarEnabled = (on != 0);

  Serial2.write((uint8_t)0x67);
  Serial2.write((uint8_t)(radarEnabled ? 1 : 0));
  for (uint8_t i = 0; i < 6; ++i) {
    Serial2.write((uint8_t)0x00);
  }

  server.send(200, "text/plain", "OK");
}

void decodeGpsPayload(const uint8_t *payload) {
  size_t offset = 0;

  memcpy(&latestGps.world_position.N, payload + offset, 8); offset += 8;
  memcpy(&latestGps.world_position.E, payload + offset, 8); offset += 8;
  memcpy(&latestGps.world_position.D, payload + offset, 8); offset += 8;

  memcpy(&latestGps.velocity.N, payload + offset, 8); offset += 8;
  memcpy(&latestGps.velocity.E, payload + offset, 8); offset += 8;
  memcpy(&latestGps.velocity.D, payload + offset, 8); offset += 8;

  memcpy(&latestGps.rotation.N, payload + offset, 8); offset += 8;
  memcpy(&latestGps.rotation.E, payload + offset, 8); offset += 8;
  memcpy(&latestGps.rotation.D, payload + offset, 8); offset += 8;

  memcpy(&latestGps.utc_date.year, payload + offset, 2); offset += 2;
  latestGps.utc_date.month = payload[offset++];
  latestGps.utc_date.day = payload[offset++];

  latestGps.utc_time.hour = payload[offset++];
  latestGps.utc_time.min = payload[offset++];
  latestGps.utc_time.sec = payload[offset++];

  memcpy(latestGps.device_id, payload + offset, 5); offset += 5;

  gpsValid = true;
  lastGpsMillis = millis();
}

void handleGpsUart() {
  while (Serial2.available() > 0) {
    uint8_t b = (uint8_t)Serial2.read();

    if (!gpsSync) {
      if (b == GPS_START_BYTE) {
        gpsSync = true;
        gpsPayloadIndex = 0;
      }
      continue;
    }

    gpsPayload[gpsPayloadIndex++] = b;
    if (gpsPayloadIndex >= GPS_PAYLOAD_LEN) {
      decodeGpsPayload(gpsPayload);
      gpsSync = false;
    }
  }
}

void handleGps() {
  String json = "{";
  if (!gpsValid) {
    json += "\"pos_n\":\"--\",";
    json += "\"pos_e\":\"--\",";
    json += "\"pos_d\":\"--\",";
    json += "\"vel_n\":\"--\",";
    json += "\"vel_e\":\"--\",";
    json += "\"vel_d\":\"--\",";
    json += "\"yaw\":\"--\",";
    json += "\"utc\":\"--\",";
    json += "\"device_id\":\"--\",";
    json += "\"age_ms\":\"--\"";
  } else {
    unsigned long age = millis() - lastGpsMillis;
    String idStr = "";
    for (int i = 0; i < 5; ++i) {
      if (latestGps.device_id[i] < 16) {
        idStr += "0";
      }
      idStr += String(latestGps.device_id[i], HEX);
      if (i < 4) {
        idStr += ":";
      }
    }

    String utcStr = String(latestGps.utc_date.year) + "-" +
                    String(latestGps.utc_date.month) + "-" +
                    String(latestGps.utc_date.day) + " " +
                    String(latestGps.utc_time.hour) + ":" +
                    String(latestGps.utc_time.min) + ":" +
                    String(latestGps.utc_time.sec);

    json += "\"pos_n\":" + String(latestGps.world_position.N, 6) + ",";
    json += "\"pos_e\":" + String(latestGps.world_position.E, 6) + ",";
    json += "\"pos_d\":" + String(latestGps.world_position.D, 2) + ",";
    json += "\"vel_n\":" + String(latestGps.velocity.N, 2) + ",";
    json += "\"vel_e\":" + String(latestGps.velocity.E, 2) + ",";
    json += "\"vel_d\":" + String(latestGps.velocity.D, 2) + ",";
    json += "\"yaw\":" + String(latestGps.rotation.E, 2) + ",";
    json += "\"utc\":\"" + utcStr + "\",";
    json += "\"device_id\":\"" + idStr + "\",";
    json += "\"age_ms\":" + String(age);
  }
  json += "}";

  server.send(200, "application/json", json);
}

void setup() {
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.print("Setting as access point ");
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  Serial.println("");
  Serial.println("ESP32 Wi-Fi Access Point ready!");
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  server.on("/", handleRoot);
  server.on("/command", handleCommand);
  server.on("/radar", handleRadarCommand);
  server.on("/gps", handleGps);

  server.begin();
  Serial.println("HTTP server started");

  Serial2.begin(115200, SERIAL_8N1, 16, 17);
}

void loop() {
  server.handleClient();
  handleGpsUart();

  unsigned long currentMillis = millis();
  if (currentMillis - lastBlink >= 1000) {
    lastBlink = currentMillis;
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
}