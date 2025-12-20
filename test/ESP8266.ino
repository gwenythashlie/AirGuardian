/*
 * =====================================================================
 * AIR GUARDIAN - ESP32 MAIN CONTROLLER FIRMWARE
 * =====================================================================
 * Description: Central controller for air quality monitoring system
 * Features:
 *   - Multi-sensor data acquisition (DHT11, MQ2, MQ7, Dust, Sound)
 *   - Automated barrier control with servo motor
 *   - Real-time air quality assessment
 *   - WiFi AP mode with web dashboard
 *   - RESTful API endpoints
 *   - Alert system with severity levels
 *   - EEPROM-based configuration persistence
 * 
 * Author: Gwenyth Ashlie Villanueva
 * Version: 2.0.0
 * Last Updated: 2025
 * =====================================================================
 */

 #include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


// ESP8266 BARRIER SLAVE - WITH OLED DISPLAY

// This ESP8266 connects to the ESP32's WiFi network and receives
// barrier control commands from the master ESP32 controller
// Displays air quality status on 128x64 OLED screen
/

// ===== OLED DISPLAY CONFIGURATION =====
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1        // Reset pin (not used, set to -1)
#define SCREEN_ADDRESS 0x3C  // I2C address (usually 0x3C or 0x3D)

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


// ===== WIFI CONFIGURATION (CONNECT TO ESP32 AP) =====
const char* ssid = "Group1_Air_Quality";      // ESP32's AP SSID
const char* password = "11111111";             // ESP32's AP Password

// ===== STATIC IP SETTINGS =====
IPAddress local_IP(192, 168, 4, 2);            // ESP8266's IP
IPAddress gateway(192, 168, 4, 1);             // ESP32's IP (gateway)
IPAddress subnet(255, 255, 255, 0);

ESP8266WebServer server(8888);                 // Web server on port 8888
WiFiClient wifiClient;                         // WiFi client for HTTP requests

// ===== HARDWARE PINS (ESP8266) =====
#define SERVO_PIN 14  // D5 (GPIO 14) -> Servo Orange Wire
#define LED_PIN   12  // D6 (GPIO 12) -> LED Long Leg (+)
// I2C pins D1 (GPIO 5 - SCL) and D2 (GPIO 4 - SDA) are used for OLED

Servo barrierServo;

#define SERVO_OPEN_ANGLE  180
#define SERVO_CLOSED_ANGLE 0

// ===== SYSTEM STATE =====
struct SystemState {
  bool servoOpen = true;
  bool autoMode = true;
  String lastCommand = "NONE";
  String lastAirQuality = "UNKNOWN";
  unsigned long lastESP32Check = 0;
  unsigned long lastServoCommand = 0;
  unsigned long lastDisplayUpdate = 0;
  bool esp32Connected = false;
  int wifiSignal = 0;
  float temperature = 0.0;
  float humidity = 0.0;
  int mq2 = 0;
  int mq7 = 0;
  int dust = 0;
} state;

// ===== TIMING CONSTANTS =====
const unsigned long ESP32_CHECK_INTERVAL = 2000;  // Check ESP32 every 2 seconds
const unsigned long WIFI_RECONNECT_INTERVAL = 5000; // Reconnect WiFi every 5 seconds if lost
const unsigned long DISPLAY_UPDATE_INTERVAL = 1000; // Update display every 1 second


// OLED DISPLAY FUNCTIONS

void initDisplay() {
  Serial.println("\n[OLED] Initializing display...");
  
  // Initialize I2C with default pins (SDA=GPIO4/D2, SCL=GPIO5/D1)
  Wire.begin();
  
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("[OLED] ✗ SSD1306 allocation failed!");
    // Show error on serial but continue without display
    return;
  }
  
  Serial.println("[OLED] ✓ Display initialized successfully");
  
  // Clear display
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  // Show splash screen
  display.setTextSize(2);
  display.setCursor(10, 10);
  display.println("AIR");
  display.println(" GUARDIAN");
  display.setTextSize(1);
  display.setCursor(20, 50);
  display.println("ESP8266 Slave");
  display.display();
  delay(2000);
}

void updateDisplay() {
  display.clearDisplay();
  
  // Title
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("=== AIR GUARDIAN ===");
  
  // Air Quality Status (Large and centered)
  display.setTextSize(2);
  int16_t x = 0;
  if (state.lastAirQuality == "GOOD") {
    x = 25;
  } else if (state.lastAirQuality == "MODERATE") {
    x = 4;
  } else if (state.lastAirQuality == "POOR") {
    x = 25;
  } else if (state.lastAirQuality == "HAZARDOUS") {
    x = 4;
  } else {
    x = 10;
  }
  
  display.setCursor(x, 15);
  display.println(state.lastAirQuality);
  
  // Barrier Status
  display.setTextSize(1);
  display.setCursor(0, 35);
  display.print("Barrier: ");
  display.println(state.servoOpen ? "OPEN" : "CLOSED");
  
  // Mode
  display.setCursor(0, 45);
  display.print("Mode: ");
  display.println(state.autoMode ? "AUTO" : "MANUAL");
  
  // Connection Status
  display.setCursor(0, 55);
  if (state.esp32Connected) {
    display.print("ESP32:OK ");
    display.print(state.wifiSignal);
    display.print("dBm");
  } else {
    display.println("ESP32: OFFLINE");
  }
  
  display.display();
}

void displayError(String message) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("ERROR:");
  display.println();
  display.println(message);
  display.display();
}

void displayConnecting() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 10);
  display.println("Connecting to");
  display.println("ESP32 Master...");
  display.println();
  display.print("SSID: ");
  display.println(ssid);
  display.display();
}

// 
// SERVO CONTROL FUNCTIONS
// 
bool moveServoToOpen() {
  if (state.servoOpen) {
    Serial.println("[Servo] Already open, skipping");
    return true;
  }
  
  Serial.println("[Servo] Opening barrier...");
  
  barrierServo.write(SERVO_OPEN_ANGLE);
  delay(500); 
  
  state.servoOpen = true;
  state.lastCommand = "OPEN_SUCCESS";
  state.lastServoCommand = millis();
  
  // LED stays ON when barrier is open
  digitalWrite(LED_PIN, HIGH);
  Serial.println("[Servo] ✓ Barrier opened");
  Serial.println("[LED] ON - Barrier is open");
  
  // Update display immediately
  updateDisplay();
  
  return true;
}

bool moveServoToClosed() {
  if (!state.servoOpen) {
    Serial.println("[Servo] Already closed, skipping");
    return true;
  }
  
  Serial.println("[Servo] Closing barrier...");
  
  barrierServo.write(SERVO_CLOSED_ANGLE);
  delay(500);
  
  state.servoOpen = false;
  state.lastCommand = "CLOSE_SUCCESS";
  state.lastServoCommand = millis();
  
  // LED blinks twice when barrier closes
  digitalWrite(LED_PIN, LOW);
  delay(200);
  digitalWrite(LED_PIN, HIGH);
  delay(300);
  digitalWrite(LED_PIN, LOW);
  delay(200);
  digitalWrite(LED_PIN, HIGH);
  delay(300);
  digitalWrite(LED_PIN, LOW);
  
  Serial.println("[Servo] ✓ Barrier closed");
  Serial.println("[LED] OFF - Barrier is closed (blinked twice)");
  
  // Update display immediately
  updateDisplay();
  
  return true;
}


// WEB SERVER HANDLERS

void handleServoCommand() {
  if (!server.hasArg("cmd")) {
    server.send(400, "application/json", "{\"error\":\"Missing cmd parameter\"}");
    return;
  }
   
  String cmd = server.arg("cmd");
  Serial.println("[API] Received command: " + cmd);
   
  if (cmd == "open") {
    moveServoToOpen();
  } else if (cmd == "close") {
    moveServoToClosed();
  } else if (cmd == "auto") {
    state.autoMode = true;
    state.lastCommand = "AUTO_MODE";
    Serial.println("[Mode] Switched to AUTO mode");
    updateDisplay();
  } else if (cmd == "manual") {
    state.autoMode = false;
    state.lastCommand = "MANUAL_MODE";
    Serial.println("[Mode] Switched to MANUAL mode");
    updateDisplay();
  } else {
    server.send(400, "application/json", "{\"error\":\"Unknown command\"}");
    return;
  }
   
  String json = "{";
  json += "\"status\":\"" + state.lastCommand + "\",";
  json += "\"open\":" + String(state.servoOpen ? "true" : "false") + ",";
  json += "\"autoMode\":" + String(state.autoMode ? "true" : "false");
  json += "}";
  
  server.send(200, "application/json", json);
}

void handleStatus() {
  String json = "{";
  json += "\"servo_angle\":" + String(barrierServo.read()) + ",";
  json += "\"servo_open\":" + String(state.servoOpen ? "true" : "false") + ",";
  json += "\"auto_mode\":" + String(state.autoMode ? "true" : "false") + ",";
  json += "\"last_command\":\"" + state.lastCommand + "\",";
  json += "\"air_quality\":\"" + state.lastAirQuality + "\",";
  json += "\"esp32_connected\":" + String(state.esp32Connected ? "true" : "false") + ",";
  json += "\"wifi_rssi\":" + String(WiFi.RSSI());
  json += "}";
  
  server.send(200, "application/json", json);
}

void handleRoot() {
  String html = R"=====(
    <!DOCTYPE html><html>
    <head><meta charset="utf-8"><title>ESP8266 Barrier Slave</title>
    <style>
      body{font-family:sans-serif;padding:20px;background:#1a1f3a;color:#e2e8f0;text-align:center}
      .container{max-width:600px;margin:0 auto;background:#1e293b;padding:30px;border-radius:20px;box-shadow:0 20px 40px rgba(0,212,255,0.2)}
      h1{color:#00d4ff;margin-bottom:10px}
      .status{padding:20px;margin:20px 0;border-radius:10px;background:rgba(0,212,255,0.1);border:2px solid #00d4ff}
      .status-label{font-size:14px;color:#64748b;text-transform:uppercase;letter-spacing:1px}
      .status-value{font-size:24px;font-weight:bold;color:#00ff88;margin-top:5px}
      button{padding:20px 40px;margin:10px;font-size:18px;border:none;border-radius:10px;cursor:pointer;transition:all 0.3s}
      .open{background:#00ff88;color:#000}
      .close{background:#ff0055;color:white}
      button:hover{transform:translateY(-2px);box-shadow:0 10px 20px rgba(0,0,0,0.3)}
      .info{font-size:12px;color:#64748b;margin-top:20px}
      .connected{color:#00ff88}
      .disconnected{color:#ff0055}
    </style></head>
    <body>
      <div class="container">
        <h1>🚧 Barrier Control</h1>
        <p style="color:#64748b">ESP8266 Slave with OLED</p>
        
        <div class="status">
          <div class="status-label">Barrier Status</div>
          <div class="status-value" id="servoStatus">Loading...</div>
        </div>
        
        <div class="status">
          <div class="status-label">Air Quality</div>
          <div class="status-value" id="airQuality">--</div>
        </div>
        
        <div style="margin:30px 0">
          <button class="open" onclick="sendCmd('open')">🔓 OPEN</button>
          <button class="close" onclick="sendCmd('close')">🔒 CLOSE</button>
        </div>
        
        <div class="info">
          <div>Mode: <span id="mode">--</span></div>
          <div>ESP32: <span id="esp32Status">--</span></div>
          <div>WiFi Signal: <span id="rssi">--</span> dBm</div>
        </div>
      </div>
      
      <script>
        function sendCmd(cmd){
          fetch('/servo?cmd='+cmd)
            .then(r=>r.json())
            .then(d=>{
              updateStatus();
            })
            .catch(e=>console.error(e));
        }
        
        function updateStatus(){
          fetch('/status')
            .then(r=>r.json())
            .then(d=>{
              document.getElementById('servoStatus').innerText = d.servo_open ? 'OPEN' : 'CLOSED';
              document.getElementById('servoStatus').style.color = d.servo_open ? '#00ff88' : '#ff0055';
              document.getElementById('mode').innerText = d.auto_mode ? 'AUTO' : 'MANUAL';
              document.getElementById('airQuality').innerText = d.air_quality;
              document.getElementById('esp32Status').innerText = d.esp32_connected ? 'Connected' : 'Disconnected';
              document.getElementById('esp32Status').className = d.esp32_connected ? 'connected' : 'disconnected';
              document.getElementById('rssi').innerText = d.wifi_rssi;
            })
            .catch(e=>console.error(e));
        }
        
        setInterval(updateStatus, 2000);
        updateStatus();
      </script>
    </body></html>
  )=====";
  
  server.send(200, "text/html", html);
}

// ESP32 COMMUNICATION

void checkESP32Status() {
  if (WiFi.status() != WL_CONNECTED) {
    state.esp32Connected = false;
    state.lastAirQuality = "NO WIFI";
    return;
  }
  
  HTTPClient http;
  http.begin(wifiClient, "http://192.168.4.1/api/sensors"); // ESP32's IP
  http.setTimeout(2000); // 2 second timeout
  
  int httpCode = http.GET();
  
  if (httpCode == HTTP_CODE_OK) {
    state.esp32Connected = true;
    String payload = http.getString();
    
    // Parse air quality status
    int statusIndex = payload.indexOf("\"Status\":\"");
    if (statusIndex > 0) {
      int startQuote = statusIndex + 10;
      int endQuote = payload.indexOf("\"", startQuote);
      String newStatus = payload.substring(startQuote, endQuote);
      
      if (state.lastAirQuality != newStatus) {
        state.lastAirQuality = newStatus;
        Serial.println("[ESP32] Air Quality: " + state.lastAirQuality);
      }
    }
    
    // Parse barrier state from ESP32
    int barrierOpenIndex = payload.indexOf("\"BarrierOpen\":");
    if (barrierOpenIndex > 0) {
      bool esp32BarrierOpen = payload.substring(barrierOpenIndex + 14, barrierOpenIndex + 18) == "true";
      
      // Sync barrier state with ESP32 in AUTO mode
      if (state.autoMode) {
        if (esp32BarrierOpen && !state.servoOpen) {
          Serial.println("[Auto] ESP32 barrier is open, opening local barrier");
          moveServoToOpen();
        } else if (!esp32BarrierOpen && state.servoOpen) {
          Serial.println("[Auto] ESP32 barrier is closed, closing local barrier");
          moveServoToClosed();
        }
      }
    }
    
    state.wifiSignal = WiFi.RSSI();
    
  } else {
    state.esp32Connected = false;
    state.lastAirQuality = "OFFLINE";
    Serial.println("[ESP32] Connection failed, code: " + String(httpCode));
  }
  
  http.end();
}

 
// WIFI MANAGEMENT

void setupWiFi() {
  Serial.println("\n[WiFi] Connecting to ESP32 AP: " + String(ssid));
  displayConnecting();
  
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  
  // Configure static IP
  if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("[WiFi] ✗ Failed to configure static IP");
  } else {
    Serial.println("[WiFi] Static IP configured: " + local_IP.toString());
  }
  
  WiFi.begin(ssid, password);
  
  Serial.print("[WiFi] Connecting");
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 30) {
    delay(500);
    Serial.print(".");
    // Quick blink during connection
    bool currentLedState = digitalRead(LED_PIN);
    digitalWrite(LED_PIN, !currentLedState);
    delay(100);
    digitalWrite(LED_PIN, currentLedState);
    tries++;
  }
  
  Serial.println();
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("[WiFi] ✓ Connected to ESP32!");
    Serial.println("[WiFi] IP Address: " + WiFi.localIP().toString());
    Serial.println("[WiFi] Gateway (ESP32): " + WiFi.gatewayIP().toString());
    Serial.println("[WiFi] Signal Strength: " + String(WiFi.RSSI()) + " dBm");
    
    // Success: 3 quick blinks then restore barrier state
    bool barrierWasOpen = state.servoOpen;
    for(int i=0; i<3; i++) { 
      digitalWrite(LED_PIN, HIGH); 
      delay(100); 
      digitalWrite(LED_PIN, LOW); 
      delay(100); 
    }
    // Restore LED to barrier state
    digitalWrite(LED_PIN, barrierWasOpen ? HIGH : LOW);
    
    // Show connected on OLED
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 20);
    display.println("WiFi Connected!");
    display.println();
    display.print("IP: ");
    display.println(WiFi.localIP());
    display.display();
    delay(2000);
    
  } else {
    Serial.println("[WiFi] ✗ Failed to connect!");
    displayError("WiFi Failed!\nRestarting...");
    delay(3000);
    ESP.restart();
  }
}

void checkWiFiConnection() {
  static unsigned long lastCheck = 0;
  
  if (millis() - lastCheck < WIFI_RECONNECT_INTERVAL) {
    return;
  }
  
  lastCheck = millis();
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi] Connection lost! Reconnecting...");
    state.esp32Connected = false;
    state.lastAirQuality = "NO WIFI";
    updateDisplay();
    
    WiFi.reconnect();
    
    int tries = 0;
    while (WiFi.status() != WL_CONNECTED && tries < 20) {
      delay(500);
      tries++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("[WiFi] ✓ Reconnected!");
      // Restore LED to barrier state after reconnection
      digitalWrite(LED_PIN, state.servoOpen ? HIGH : LOW);
    } else {
      Serial.println("[WiFi] ✗ Reconnection failed, restarting...");
      delay(1000);
      ESP.restart();
    }
  }
}
 
// SETUP

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n========================================");
  Serial.println("  ESP8266 BARRIER SLAVE + OLED");
  Serial.println("========================================\n");

  // Initialize OLED Display FIRST
  initDisplay();

  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Serial.println("[LED] Initialized on GPIO " + String(LED_PIN));

  // Initialize Servo
  barrierServo.attach(SERVO_PIN);
  barrierServo.write(SERVO_OPEN_ANGLE);
  Serial.println("[Servo] Initialized on GPIO " + String(SERVO_PIN));
  Serial.println("[Servo] Initial position: OPEN");
  
  // LED ON because barrier starts open
  digitalWrite(LED_PIN, HIGH);
  Serial.println("[LED] ON - Barrier is open");
  
  // Connect to ESP32's WiFi
  setupWiFi();

  // Start web server
  Serial.println("\n[Server] Starting web server...");
  server.on("/", handleRoot);
  server.on("/servo", handleServoCommand);
  server.on("/status", handleStatus);
  server.begin();
  
  Serial.println("[Server] Web server started on port 8888");
  Serial.println("[Server] Access at: http://" + WiFi.localIP().toString() + ":8888");
  
  Serial.println("\n========================================");
  Serial.println("        SYSTEM READY");
  Serial.println("========================================\n");
  
  // Initial ESP32 check
  delay(1000);
  checkESP32Status();
  updateDisplay();
}


// MAIN LOOP
 
void loop() {
  // Handle web server requests
  server.handleClient();
  
  // Check WiFi connection
  checkWiFiConnection();
  
  // Check ESP32 status and sync barrier
  if (millis() - state.lastESP32Check >= ESP32_CHECK_INTERVAL) {
    checkESP32Status();
    state.lastESP32Check = millis();
  }
  
  // Update OLED display
  if (millis() - state.lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
    updateDisplay();
    state.lastDisplayUpdate = millis();
  }
  
  delay(10);
}