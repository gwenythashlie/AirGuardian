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

#include <WiFi.h>
#include <WebServer.h>
#include <DHT.h>
#include <EEPROM.h>
#include <ESP32Servo.h>
#include <time.h>
#include <stdlib.h>
#include <vector>

// =====================================================================
// WIFI CONFIGURATION
// =====================================================================
const char* ssid = "Group1_Air_Quality";
const char* password = "11111111";

WebServer server(80);

// =====================================================================
// HARDWARE PIN DEFINITIONS (ESP32)
// =====================================================================
// Analog Sensors
#define MQ2_PIN      34  // Smoke/Gas sensor (ADC1_CH6)
#define MQ7_PIN      33  // Carbon Monoxide sensor (ADC1_CH5)
#define DUST_PIN     32  // Dust/Particulate sensor (ADC1_CH4)
#define SOUND_PIN    35  // Sound level sensor (ADC1_CH7)

// Digital Sensors
#define DHT_PIN      4   // Temperature & Humidity sensor
#define DHTTYPE      DHT11

// Actuators
#define DUST_LED_PIN 18  // IR LED for dust sensor
#define SERVO_PIN    15  // Barrier servo motor

// Visual/Audio Indicators
#define LED_GREEN    14  // Good air quality indicator
#define LED_YELLOW   27  // Moderate air quality indicator
#define LED_RED      26  // Poor/Hazardous air quality indicator
#define BUZZER_PIN   13  // Alert buzzer
#define FAN_PIN      12  // Ventilation fan control

// =====================================================================
// SENSOR OBJECTS
// =====================================================================
DHT dht(DHT_PIN, DHTTYPE);
Servo barrierServo;

// =====================================================================
// SYSTEM CONFIGURATION & THRESHOLDS
// =====================================================================
// Air Quality Thresholds
struct AirQualityThresholds {
  int mq2_moderate = 1800;
  int mq2_poor = 2400;
  int mq2_hazardous = 3000;
  
  int mq7_moderate = 1800;
  int mq7_poor = 2400;
  int mq7_hazardous = 3000;
  
  int dust_moderate = 1000;
  int dust_poor = 2000;
  int dust_hazardous = 3000;
} thresholds;

// System State Structure
struct SystemState {
  // Sensor readings
  float temperature = 0.0;
  float humidity = 0.0;
  int mq2Value = 0;
  int mq7Value = 0;
  int dustValue = 0;
  int soundValue = 0;
  
  // Air quality status
  String airQualityStatus = "GOOD";
  
  // Barrier control
  bool barrierOpen = true;
  bool barrierAutoMode = true;
  unsigned long barrierClosedTime = 0;
  unsigned long barrierDuration = 1800000; // 30 minutes default
  
  // System status
  bool esp8266Connected = false;
  unsigned long lastSensorRead = 0;
  unsigned long lastESP8266Ping = 0;
} state;

// Alert Structure
struct Alert {
  String message;
  String severity; // "CRITICAL" or "INFO"
  uint64_t timestamp;
  int mq2;
  int mq7;
  int dust;
  float temp;
  float humidity;
};

std::vector<Alert> alertHistory;
const int MAX_ALERTS = 50;
const unsigned long SENSOR_ALERT_COOLDOWN = 60000; // 60s between identical sensor alerts

struct SensorAlertTracker {
  unsigned long mq2 = 0;
  unsigned long mq7 = 0;
  unsigned long dust = 0;
} lastSensorAlert;

uint64_t bootMonotonicMs = 0;
bool timeSynced = false;
bool manualTimeSet = false;
int64_t manualTimeOffsetMs = 0;

// =====================================================================
// EEPROM CONFIGURATION
// =====================================================================
#define EEPROM_SIZE 64
#define EEPROM_BARRIER_MODE_ADDR 0
#define EEPROM_BARRIER_DURATION_ADDR 4

// =====================================================================
// DUST SENSOR TIMING (Sharp GP2Y10 compatible)
// =====================================================================
#define DUST_SAMPLE_TIME 280
#define DUST_DELAY_TIME 40
#define DUST_OFF_TIME 9680
int dustBaselineAdc = 520;            // Baseline determined at startup
const int DUST_CONVERSION = 15;       // Multiplier to convert baseline-adjusted ADC to µg/m³
const uint8_t DUST_AVG_SAMPLES = 5;   // Samples per reading for stability

// MQ Sensors calibration params
int MQ2_BASELINE_ADC = 0;
int MQ7_BASELINE_ADC = 0;
const uint8_t MQ_AVG_SAMPLES = 10;
const int MQ2_CONVERSION = 50;   // Higher sensitivity for smoke detection
const int MQ2_SENSITIVITY_MULTIPLIER = 6;   // Amplify deviation from baseline
const int MQ7_SENSITIVITY_MULTIPLIER = 1;   // Amplify deviation from baseline
// =====================================================================
// FUNCTION PROTOTYPES
// =====================================================================
void setupWiFi();
void setupTime();
void setupSensors();
void setupActuators();
void setupServer();
void readSensors();
void assessAirQuality();
void controlBarrier();
void checkSensorAlerts();
void updateIndicators();
void handleRoot();
void handleSensorsAPI();
void handleAlertsAPI();
void handleNotFound();
void addAlert(String message, String severity);
void loadEEPROMConfig();
void saveEEPROMConfig();
int readDustSensor();
void checkESP8266Connection();
String getAirQualityJSON();
uint64_t currentTimestampMs();

// =====================================================================
// SETUP FUNCTION
// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n");
  Serial.println("====================================");
  Serial.println("   AIR GUARDIAN SYSTEM BOOTING");
  Serial.println("====================================");
  
  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);
  loadEEPROMConfig();
  
  // Setup components
  setupSensors();
  setupActuators();
  setupWiFi();
  setupTime();
  setupServer();
  
  Serial.println("====================================");
  Serial.println("   SYSTEM READY");
  Serial.println("====================================\n");
  
  // Initial readings
  readSensors();
  assessAirQuality();
  
  // Welcome alert
  addAlert("System initialized successfully", "INFO");
}

// =====================================================================
// MAIN LOOP
// =====================================================================
void loop() {
  server.handleClient();
  
  // Read sensors every second for faster responsiveness
  if (millis() - state.lastSensorRead >= 1000) {
    readSensors();
    assessAirQuality();
    checkSensorAlerts();
    controlBarrier();
    updateIndicators();
    state.lastSensorRead = millis();
  }
  
  // Check ESP8266 connection every 5 seconds
  if (millis() - state.lastESP8266Ping >= 5000) {
    checkESP8266Connection();
    state.lastESP8266Ping = millis();
  }
  
  delay(10);
}

// =====================================================================
// WIFI SETUP
// =====================================================================
void setupWiFi() {
  Serial.println("\n[WiFi] Configuring Access Point...");
  Serial.print("[WiFi] SSID: ");
  Serial.println(ssid);
  
  // Configure as Access Point
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  
  IPAddress IP = WiFi.softAPIP();
  Serial.print("[WiFi] AP IP Address: ");
  Serial.println(IP);
  Serial.println("[WiFi] Access Point Ready!");
}

// Attempt to sync time for real timestamps; fall back to monotonic millis if NTP is unreachable
void setupTime() {
  bootMonotonicMs = millis();
  timeSynced = false;
  manualTimeSet = false;

  // Set timezone to Asia/Manila (UTC+8) without DST
  setenv("TZ", "PHT-8", 1);
  tzset();
  
  configTime(8 * 3600, 0, "pool.ntp.org", "time.nist.gov", "time.google.com");

  int retries = 0;
  const int maxRetries = 10;
  while (retries < maxRetries) {
    time_t now = time(nullptr);
    if (now > 1700000000) { // Roughly 2023-11-14 epoch to ensure validity
      timeSynced = true;
      Serial.printf("[Time] Synced. Epoch: %ld\n", now);
      return;
    }
    retries++;
    delay(200);
  }

  Serial.println("[Time] NTP sync failed - using device uptime for timestamps");
}

uint64_t currentTimestampMs() {
  if (timeSynced) {
    time_t now = time(nullptr);
    if (now > 0) {
      return static_cast<uint64_t>(now) * 1000ULL;
    }
  }
  if (manualTimeSet) {
    int64_t adjusted = static_cast<int64_t>(millis()) + manualTimeOffsetMs;
    if (adjusted > 0) {
      return static_cast<uint64_t>(adjusted);
    }
  }
  return static_cast<uint64_t>(millis());
}

// =====================================================================
// SENSOR INITIALIZATION
// =====================================================================
void setupSensors() {
  Serial.println("\n[Sensors] Initializing...");
  
  // Initialize DHT sensor
  dht.begin();
  Serial.println("[Sensors] DHT11 initialized");
  
  // Configure analog pins
  pinMode(MQ2_PIN, INPUT);
  pinMode(MQ7_PIN, INPUT);
  pinMode(DUST_PIN, INPUT);
  pinMode(SOUND_PIN, INPUT);
  
  // Configure dust sensor LED
  pinMode(DUST_LED_PIN, OUTPUT);
  digitalWrite(DUST_LED_PIN, LOW);
  
  Serial.println("[Sensors] Analog sensors configured");
  Serial.println("[Sensors] Warming up sensors (10s)...");
  
  // Sensor warm-up period
  for (int i = 10; i > 0; i--) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("\n[Sensors] Ready!");

  // Establish baselines for MQ sensors after warm-up (assumed clean air)
  long mq2Sum = 0, mq7Sum = 0;
  for (int i = 0; i < 50; i++) {
    mq2Sum += analogRead(MQ2_PIN);
    mq7Sum += analogRead(MQ7_PIN);
    delay(20);
  }
  MQ2_BASELINE_ADC = mq2Sum / 50;
  MQ7_BASELINE_ADC = mq7Sum / 50;
  Serial.printf("[Sensors] MQ2 baseline=%d, MQ7 baseline=%d\n", MQ2_BASELINE_ADC, MQ7_BASELINE_ADC);

  // Dust sensor baseline sampling (assume clean air during warm-up)
  long dustSum = 0;
  for (int i = 0; i < 60; i++) {
    digitalWrite(DUST_LED_PIN, LOW);
    delayMicroseconds(280);
    int sample = analogRead(DUST_PIN);
    delayMicroseconds(40);
    digitalWrite(DUST_LED_PIN, HIGH);
    delayMicroseconds(9680);
    dustSum += sample;
    delay(10);
  }
  dustBaselineAdc = dustSum / 60;
  Serial.printf("[Sensors] Dust baseline=%d\n", dustBaselineAdc);
}

// =====================================================================
// ACTUATOR INITIALIZATION
// =====================================================================
void setupActuators() {
  Serial.println("\n[Actuators] Initializing...");
  
  // LED indicators
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  
  // Buzzer and fan
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  
  // Initial state: all off
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_RED, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(FAN_PIN, LOW);
  
  // Initialize servo
  barrierServo.attach(SERVO_PIN);
  if (state.barrierOpen) {
    barrierServo.write(90); // Open position
  } else {
    barrierServo.write(0);  // Closed position
  }
  
  Serial.println("[Actuators] LEDs configured");
  Serial.println("[Actuators] Buzzer configured");
  Serial.println("[Actuators] Fan configured");
  Serial.println("[Actuators] Servo configured");
  Serial.println("[Actuators] Ready!");
}

// =====================================================================
// WEB SERVER SETUP
// =====================================================================
void setupServer() {
  Serial.println("\n[Server] Configuring endpoints...");
  
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/sensors", HTTP_GET, handleSensorsAPI);
  server.on("/api/alerts", HTTP_GET, handleAlertsAPI);
  server.onNotFound(handleNotFound);
  
  server.begin();
  Serial.println("[Server] Web server started");
  Serial.println("[Server] Endpoints:");
  Serial.println("  - GET /");
  Serial.println("  - GET /api/sensors");
  Serial.println("  - GET /api/alerts");
}

// =====================================================================
// SENSOR READING
// =====================================================================
void readSensors() {
  // Read DHT11 (Temperature & Humidity)
  // Robust DHT reads with retries and smoothing
  float t = NAN, h = NAN;
  for (int i = 0; i < 3; i++) {
    t = dht.readTemperature();
    h = dht.readHumidity();
    if (!isnan(t) && !isnan(h)) break;
    delay(50);
  }
  if (isnan(t) || isnan(h)) {
    // Keep previous values if read fails to avoid spikes
    Serial.println("[Sensors] Warning: DHT read failed, keeping previous values");
  } else {
    // Exponential moving average for stability
    state.temperature = (state.temperature == 0.0) ? t : (0.7f * state.temperature + 0.3f * t);
    state.humidity = (state.humidity == 0.0) ? h : (0.7f * state.humidity + 0.3f * h);
  }
  
  // Read analog sensors - direct ADC reads for real-time values
  analogRead(MQ2_PIN);
  delay(10);
  int mq2 = analogRead(MQ2_PIN);

  analogRead(MQ7_PIN);
  delay(10);
  int mq7 = analogRead(MQ7_PIN);
  
  if (mq2 < MQ2_BASELINE_ADC) {
    MQ2_BASELINE_ADC = (MQ2_BASELINE_ADC * 95 + mq2 * 5) / 100; // slowly adapt baseline downwards
  }
  if (mq7 < MQ7_BASELINE_ADC) {
    MQ7_BASELINE_ADC = (MQ7_BASELINE_ADC * 95 + mq7 * 5) / 100;
  }

  int mq2Delta = mq2 - MQ2_BASELINE_ADC;
  int mq7Delta = mq7 - MQ7_BASELINE_ADC;
  if (mq2Delta < 0) mq2Delta = 0;
  if (mq7Delta < 0) mq7Delta = 0;

  state.mq2Value = mq2 + mq2Delta * MQ2_SENSITIVITY_MULTIPLIER;
  state.mq7Value = mq7 + mq7Delta * MQ7_SENSITIVITY_MULTIPLIER;
  
  // Read sound sensor with peak-to-peak detection
  unsigned long startMillis = millis();
  unsigned long signalMax = 0;
  unsigned long signalMin = 4095;
  unsigned int sampleWindow = 50;

  while (millis() - startMillis < sampleWindow) {
    int sample = analogRead(SOUND_PIN);
    
    if (sample < 4095) { 
      if (sample > signalMax) signalMax = sample;
      else if (sample < signalMin) signalMin = sample;
    }
  }
  
  int sound = signalMax - signalMin;
  if (sound < 50) sound = 0;
  state.soundValue = sound;
  
  // Read dust sensor
  state.dustValue = readDustSensor();
  
  // Debug output
  Serial.printf("[Sensors] Temp: %.1f°C | Hum: %.1f%% | MQ2: %d | MQ7: %d | Dust: %d | Sound: %d\n",
                state.temperature, state.humidity, state.mq2Value, 
                state.mq7Value, state.dustValue, state.soundValue);
}

// =====================================================================
// DUST SENSOR READING (Sharp GP2Y10)
// =====================================================================
int readDustSensor() {
  long accumulator = 0;

  for (uint8_t i = 0; i < DUST_AVG_SAMPLES; i++) {
    digitalWrite(DUST_LED_PIN, LOW);
    delayMicroseconds(280);
    int sample = analogRead(DUST_PIN);
    delayMicroseconds(40);
    digitalWrite(DUST_LED_PIN, HIGH);
    delayMicroseconds(9680);
    accumulator += sample;
  }

  int dustRaw = accumulator / DUST_AVG_SAMPLES;
  int dustCorrected = dustRaw - dustBaselineAdc;
  if (dustCorrected < 0) dustCorrected = 0;
  
  int dust = dustCorrected * DUST_CONVERSION;
  return dust;
}

// =====================================================================
// AIR QUALITY ASSESSMENT
// =====================================================================
void assessAirQuality() {
  String previousStatus = state.airQualityStatus;
  
  // Count hazardous, poor, and moderate conditions
  int hazardCount = 0, poorCount = 0, moderateCount = 0;
  
  // Check MQ2 (Smoke/Gas)
  if (state.mq2Value >= thresholds.mq2_hazardous) hazardCount++;
  else if (state.mq2Value >= thresholds.mq2_poor) poorCount++;
  else if (state.mq2Value >= thresholds.mq2_moderate) moderateCount++;
  
  // Check MQ7 (Carbon Monoxide)
  if (state.mq7Value >= thresholds.mq7_hazardous) hazardCount++;
  else if (state.mq7Value >= thresholds.mq7_poor) poorCount++;
  else if (state.mq7Value >= thresholds.mq7_moderate) moderateCount++;
  
  // Check Dust
  if (state.dustValue >= thresholds.dust_hazardous) hazardCount++;
  else if (state.dustValue >= thresholds.dust_poor) poorCount++;
  else if (state.dustValue >= thresholds.dust_moderate) moderateCount++;
  
  // Determine overall status
  if (hazardCount > 0) {
    state.airQualityStatus = "HAZARDOUS";
  } else if (poorCount > 0) {
    state.airQualityStatus = "POOR";
  } else if (moderateCount > 0) {
    state.airQualityStatus = "MODERATE";
  } else {
    state.airQualityStatus = "GOOD";
  }
  
  // Generate alerts on status change
  if (state.airQualityStatus != previousStatus) {
    String message = "Air quality changed to " + state.airQualityStatus;
    String severity = (state.airQualityStatus == "HAZARDOUS" || 
                       state.airQualityStatus == "POOR") ? "CRITICAL" : "INFO";
    addAlert(message, severity);
    
    Serial.printf("[AirQuality] Status: %s -> %s\n", 
                  previousStatus.c_str(), state.airQualityStatus.c_str());
  }
}

void checkSensorAlerts() {
  unsigned long nowMs = millis();

  if (state.mq2Value >= thresholds.mq2_hazardous && nowMs - lastSensorAlert.mq2 > SENSOR_ALERT_COOLDOWN) {
    addAlert("MQ2 levels hazardous", "CRITICAL");
    lastSensorAlert.mq2 = nowMs;
  }

  if (state.mq7Value >= thresholds.mq7_hazardous && nowMs - lastSensorAlert.mq7 > SENSOR_ALERT_COOLDOWN) {
    addAlert("MQ7 levels hazardous", "CRITICAL");
    lastSensorAlert.mq7 = nowMs;
  }

  if (state.dustValue >= thresholds.dust_hazardous && nowMs - lastSensorAlert.dust > SENSOR_ALERT_COOLDOWN) {
    addAlert("Dust concentration hazardous", "CRITICAL");
    lastSensorAlert.dust = nowMs;
  }
}

// =====================================================================
// BARRIER CONTROL LOGIC
// =====================================================================
void controlBarrier() {
  if (state.barrierAutoMode) {
    // Auto mode: close barrier if HAZARDOUS or POOR
    if ((state.airQualityStatus == "HAZARDOUS" || 
         state.airQualityStatus == "POOR") && state.barrierOpen) {
      // Close barrier
      state.barrierOpen = false;
      state.barrierClosedTime = millis();
      barrierServo.write(0);
      addAlert("Barrier automatically closed due to poor air quality", "CRITICAL");
      Serial.println("[Barrier] AUTO CLOSE - Poor air detected");
    }
    
    // Auto reopen after duration if air quality improved
    if (!state.barrierOpen && 
        (millis() - state.barrierClosedTime >= state.barrierDuration) &&
        (state.airQualityStatus == "GOOD" || state.airQualityStatus == "MODERATE")) {
      state.barrierOpen = true;
      barrierServo.write(90);
      addAlert("Barrier automatically reopened - air quality improved", "INFO");
      Serial.println("[Barrier] AUTO OPEN - Air quality improved");
    }
  }
  
  // Handle manual commands from API
  if (server.hasArg("barrier_cmd")) {
    String cmd = server.arg("barrier_cmd");
    
    if (cmd == "open") {
      state.barrierOpen = true;
      barrierServo.write(90);
      addAlert("Barrier manually opened", "INFO");
      Serial.println("[Barrier] MANUAL OPEN");
    } else if (cmd == "close") {
      state.barrierOpen = false;
      state.barrierClosedTime = millis();
      barrierServo.write(0);
      addAlert("Barrier manually closed", "INFO");
      Serial.println("[Barrier] MANUAL CLOSE");
    } else if (cmd == "auto") {
      state.barrierAutoMode = true;
      saveEEPROMConfig();
      addAlert("Barrier switched to AUTO mode", "INFO");
      Serial.println("[Barrier] Mode: AUTO");
    } else if (cmd == "manual") {
      state.barrierAutoMode = false;
      saveEEPROMConfig();
      addAlert("Barrier switched to MANUAL mode", "INFO");
      Serial.println("[Barrier] Mode: MANUAL");
    }
  }
}

// =====================================================================
// INDICATOR UPDATES (LEDs, Buzzer, Fan)
// =====================================================================
void updateIndicators() {
  // Turn off all LEDs first
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_RED, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  
  // Set indicators based on air quality
  if (state.airQualityStatus == "GOOD") {
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(FAN_PIN, LOW);
  } else if (state.airQualityStatus == "MODERATE") {
    digitalWrite(LED_YELLOW, HIGH);
    digitalWrite(FAN_PIN, LOW);
  } else if (state.airQualityStatus == "POOR") {
    digitalWrite(LED_RED, HIGH);
    digitalWrite(FAN_PIN, HIGH);
  } else if (state.airQualityStatus == "HAZARDOUS") {
    digitalWrite(LED_RED, HIGH);
    digitalWrite(FAN_PIN, HIGH);
    // Pulse buzzer
    digitalWrite(BUZZER_PIN, (millis() / 500) % 2);
  }
}

// =====================================================================
// ALERT MANAGEMENT
// =====================================================================
void addAlert(String message, String severity) {
  Alert newAlert;
  newAlert.message = message;
  newAlert.severity = severity;
  newAlert.timestamp = currentTimestampMs();  // Use real system time
  newAlert.mq2 = state.mq2Value;
  newAlert.mq7 = state.mq7Value;
  newAlert.dust = state.dustValue;
  newAlert.temp = state.temperature;
  newAlert.humidity = state.humidity;
  
  alertHistory.insert(alertHistory.begin(), newAlert);
  
  // Limit alert history
  if (alertHistory.size() > MAX_ALERTS) {
    alertHistory.pop_back();
  }
  
  Serial.print("[");
  Serial.print(severity);
  Serial.print("] ");
  Serial.println(message);
}

// =====================================================================
// EEPROM PERSISTENCE
// =====================================================================
void loadEEPROMConfig() {
  EEPROM.get(EEPROM_BARRIER_MODE_ADDR, state.barrierAutoMode);
  EEPROM.get(EEPROM_BARRIER_DURATION_ADDR, state.barrierDuration);

  // Enforce the requested 30-minute duration even if an older value is stored
  state.barrierDuration = 1800000;
  
  Serial.printf("[EEPROM] Loaded: AutoMode=%d, Duration=%lu\n", 
                state.barrierAutoMode, state.barrierDuration);
}

void saveEEPROMConfig() {
  EEPROM.put(EEPROM_BARRIER_MODE_ADDR, state.barrierAutoMode);
  EEPROM.put(EEPROM_BARRIER_DURATION_ADDR, state.barrierDuration);
  EEPROM.commit();
  
  Serial.println("[EEPROM] Configuration saved");
}

// =====================================================================
// ESP8266 CONNECTION CHECK
// =====================================================================
void checkESP8266Connection() {
  int clientCount = WiFi.softAPgetStationNum();
  state.esp8266Connected = (clientCount > 0);
  
  if (state.esp8266Connected) {
    Serial.printf("[ESP8266] Connected - %d client(s)\n", clientCount);
  }
}

// =====================================================================
// HTTP HANDLERS
// =====================================================================

// Root endpoint - serves the dashboard HTML
void handleRoot() {
  Serial.println("[Server] Request: GET /");
  
  /*
   * ====================================================================
   * >>> CUSTOM HTML INSERTION POINT <<<
   * ====================================================================
   * Replace the content between R"rawliteral( and )rawliteral" with your
   * custom HTML dashboard code. The HTML provided in your document should
   * be pasted here.
   * ====================================================================
   */
  
  const char* html = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <meta name="description" content="Real-time air quality monitoring system with automated barrier control">
  <title>AIR GUARDIAN - Air Quality Monitor</title>
  <style>
    /* ========================================
       CSS CUSTOM PROPERTIES
       ======================================== */
    :root {
      /* Brand Colors */
      --primary: #00d4ff;
      --secondary: #0099ff;
      --success: #00ff88;
      --warning: #ffb000;
      --danger: #ff0055;
      --critical: #ff3333;
      
      /* Theme Colors - Dark Mode */
      --dark-bg: #0f172a;
      --dark-card: #1e293b;
      
      /* Theme Colors - Light Mode */
      --light-bg: #f8fafc;
      --light-card: #ffffff;
      
      /* Spacing */
      --spacing-sm: 0.5rem;
      --spacing-md: 1rem;
      --spacing-lg: 1.5rem;
      --spacing-xl: 2rem;
      
      /* Border Radius */
      --radius-sm: 0.5rem;
      --radius-md: 0.75rem;
      --radius-lg: 1rem;
      --radius-xl: 1.5rem;
      --radius-2xl: 2rem;
      
      /* Transitions */
      --transition-fast: 0.3s ease;
      --transition-medium: 0.5s ease;
      --transition-slow: 0.8s cubic-bezier(0.68, -0.55, 0.265, 1.55);
    }

    /* ========================================
       GLOBAL RESETS & BASE STYLES
       ======================================== */
    * {
      margin: 0;
      padding: 0;
      box-sizing: border-box;
    }

    body {
      font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
      background: linear-gradient(135deg, var(--dark-bg) 0%, #1a1f3a 100%);
      color: #e2e8f0;
      min-height: 100vh;
      transition: background var(--transition-fast);
    }

    body.light-mode {
      background: linear-gradient(135deg, #f1f5f9 0%, #e2e8f0 100%);
      color: #1e293b;
    }

    /* ========================================
       HEADER COMPONENT
       ======================================== */
    header {
      position: sticky;
      top: 0;
      z-index: 40;
      backdrop-filter: blur(12px);
      border-bottom: 1px solid rgba(0, 212, 255, 0.1);
      background: rgba(15, 23, 42, 0.9);
      padding: var(--spacing-lg);
    }

    body.light-mode header {
      background: rgba(248, 250, 252, 0.9);
      border-bottom-color: rgba(0, 100, 150, 0.1);
    }

    .header-content {
      max-width: 1400px;
      margin: 0 auto;
      display: flex;
      align-items: center;
      justify-content: space-between;
    }

    .logo-section {
      display: flex;
      align-items: center;
      gap: var(--spacing-md);
    }

    .logo-section .icon {
      font-size: 2.5rem;
    }

    .logo-section h1 {
      font-size: 1.875rem;
      font-weight: 900;
      background: linear-gradient(90deg, var(--primary), var(--secondary));
      -webkit-background-clip: text;
      -webkit-text-fill-color: transparent;
      background-clip: text;
    }

    .logo-section p {
      color: #64748b;
      font-size: 0.875rem;
    }

    body.light-mode .logo-section p {
      color: #94a3b8;
    }

    /* ========================================
       THEME TOGGLE BUTTON
       ======================================== */
    .theme-toggle {
      width: 2.75rem;
      height: 2.75rem;
      border-radius: var(--radius-md);
      background: var(--dark-card);
      border: none;
      cursor: pointer;
      display: flex;
      align-items: center;
      justify-content: center;
      color: #fbbf24;
      font-size: 1.5rem;
      transition: all var(--transition-fast);
    }

    body.light-mode .theme-toggle {
      background: #e2e8f0;
      color: #1e293b;
    }

    .theme-toggle:hover {
      transform: scale(1.1);
      box-shadow: 0 0 20px rgba(0, 212, 255, 0.3);
    }

    /* ========================================
       MAIN CONTENT CONTAINER
       ======================================== */
    main {
      max-width: 1400px;
      margin: 0 auto;
      padding: var(--spacing-xl) var(--spacing-lg);
    }

    /* ========================================
       NAVIGATION TABS
       ======================================== */
    .nav-tabs {
      display: flex;
      gap: var(--spacing-sm);
      margin-bottom: var(--spacing-xl);
      overflow-x: auto;
      padding-bottom: var(--spacing-sm);
    }

    .tab-btn {
      display: flex;
      align-items: center;
      gap: var(--spacing-sm);
      padding: 0.75rem var(--spacing-lg);
      border-radius: var(--radius-md);
      border: none;
      cursor: pointer;
      font-weight: 600;
      white-space: nowrap;
      background: transparent;
      color: #64748b;
      transition: all var(--transition-fast);
    }

    body.light-mode .tab-btn {
      color: #64748b;
    }

    .tab-btn.active {
      background: linear-gradient(90deg, var(--primary), var(--secondary));
      color: white;
      box-shadow: 0 0 20px rgba(0, 212, 255, 0.4);
      transform: scale(1.05);
    }

    .tab-btn:hover:not(.active) {
      color: #e2e8f0;
      background: rgba(0, 212, 255, 0.1);
    }

    /* ========================================
       TAB CONTENT PANELS
       ======================================== */
    .tab-content {
      display: none;
      animation: fadeIn 0.5s ease-out;
    }

    .tab-content.active {
      display: block;
    }

    @keyframes fadeIn {
      from {
        opacity: 0;
        transform: translateY(10px);
      }
      to {
        opacity: 1;
        transform: translateY(0);
      }
    }

    /* ========================================
       CARD COMPONENT
       ======================================== */
    .card {
      background: rgba(30, 41, 59, 0.8);
      border: 1px solid rgba(0, 212, 255, 0.2);
      border-radius: var(--radius-xl);
      padding: var(--spacing-lg);
      backdrop-filter: blur(10px);
      transition: all var(--transition-fast);
    }

    body.light-mode .card {
      background: rgba(255, 255, 255, 0.9);
      border-color: rgba(0, 100, 150, 0.1);
    }

    .card:hover {
      box-shadow: 0 20px 40px rgba(0, 212, 255, 0.2);
    }

    /* ========================================
       GRID LAYOUT SYSTEM
       ======================================== */
    .grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
      gap: var(--spacing-lg);
      margin-bottom: var(--spacing-xl);
    }

    .grid-full {
      grid-column: 1 / -1;
    }

    /* ========================================
       SENSOR CARDS
       ======================================== */
    .sensor-card {
      position: relative;
      overflow: hidden;
    }

    .sensor-card::before {
      content: '';
      position: absolute;
      top: 0;
      left: 0;
      right: 0;
      bottom: 0;
      background: linear-gradient(135deg, rgba(0, 212, 255, 0.1) 0%, transparent 100%);
      opacity: 0;
      transition: opacity var(--transition-fast);
    }

    .sensor-card:hover::before {
      opacity: 1;
    }

    .sensor-card:hover {
      transform: translateY(-5px) scale(1.02);
    }

    .sensor-header {
      display: flex;
      align-items: center;
      justify-content: space-between;
      margin-bottom: var(--spacing-md);
    }

    .sensor-label {
      font-size: 0.875rem;
      font-weight: 600;
      color: #64748b;
      text-transform: uppercase;
      letter-spacing: 0.05em;
    }

    body.light-mode .sensor-label {
      color: #94a3b8;
    }

    .sensor-icon {
      font-size: 1.25rem;
      transition: transform var(--transition-fast);
    }

    .sensor-card:hover .sensor-icon {
      transform: scale(1.3);
    }

    .sensor-value {
      font-size: 2.5rem;
      font-weight: 900;
      background: linear-gradient(90deg, var(--primary), var(--secondary));
      -webkit-background-clip: text;
      -webkit-text-fill-color: transparent;
      background-clip: text;
      margin: var(--spacing-sm) 0;
    }

    .sensor-unit {
      font-size: 0.875rem;
      color: #64748b;
    }

    body.light-mode .sensor-unit {
      color: #94a3b8;
    }

    /* ========================================
       STATUS CARD (AIR QUALITY)
       ======================================== */
    .status-card {
      text-align: center;
      padding: var(--spacing-xl);
    }

    .status-icon {
      font-size: 4rem;
      margin-bottom: var(--spacing-md);
      animation: pulse 2s ease-in-out infinite;
    }

    @keyframes pulse {
      0%, 100% { transform: scale(1); }
      50% { transform: scale(1.1); }
    }

    .status-title {
      font-size: 3rem;
      font-weight: 900;
      margin-bottom: var(--spacing-sm);
    }

    .status-desc {
      color: #64748b;
      font-size: 1.125rem;
    }

    /* Status Color Classes */
    .status-good { color: var(--success); }
    .status-moderate { color: var(--warning); }
    .status-poor { color: #ff6b35; }
    .status-hazardous { color: var(--danger); }

    /* ========================================
       LED INDICATORS (UI-only) - Reference Image Design
       ======================================== */
    .led-indicators {
      display: flex;
      gap: 4rem;
      justify-content: center;
      align-items: flex-start;
      margin-top: var(--spacing-xl);
      flex-wrap: wrap;
    }
    .led-item {
      display: flex;
      flex-direction: column;
      align-items: center;
      gap: 1rem;
    }
    .led {
      width: 80px;
      height: 80px;
      border-radius: 50%;
      opacity: 0.2;
      box-shadow: 0 0 0 2px rgba(255,255,255,0.15);
      transition: all 0.3s ease;
      border: 3px solid rgba(255,255,255,0.2);
      position: relative;
    }
    .led-green { background: #00ff88; }
    .led-yellow { background: #ffb000; }
    .led-red { background: #ff0055; }
    .led.active { 
      opacity: 1;
      box-shadow: 0 0 20px currentColor, 0 0 40px currentColor, 0 0 60px currentColor, inset 0 0 15px currentColor;
      border-color: currentColor;
      filter: brightness(1.2);
    }
    .led-label {
      font-size: 0.95rem;
      font-weight: 700;
      text-transform: uppercase;
      letter-spacing: 0.1em;
      opacity: 0.35;
      transition: all 0.3s ease;
      color: #94a3b8;
    }
    .led-item.active .led-label {
      opacity: 1;
      font-size: 1rem;
    }
    .led-item.active.item-green .led-label { color: #00ff88; }
    .led-item.active.item-yellow .led-label { color: #ffb000; }
    .led-item.active.item-red .led-label { color: #ff0055; }

    /* ========================================
       BARRIER CONTROL SECTION
       ======================================== */
    .barrier-section {
      text-align: center;
    }

    .barrier-icon {
      font-size: 5rem;
      margin: var(--spacing-lg) 0;
      transition: all var(--transition-slow);
      display: inline-block;
      transform-origin: top center;
    }

    .barrier-icon.closed {
      transform: rotateZ(-85deg) scaleY(0.95);
      animation: doorClosing var(--transition-slow);
    }

    .barrier-icon.open {
      transform: rotateZ(0deg) scaleY(1);
      animation: doorOpening var(--transition-slow);
    }

    @keyframes doorClosing {
      0% { transform: rotateZ(0deg) scaleY(1); }
      50% { transform: rotateZ(-42.5deg) scaleY(0.97); }
      100% { transform: rotateZ(-85deg) scaleY(0.95); }
    }

    @keyframes doorOpening {
      0% { transform: rotateZ(-85deg) scaleY(0.95); }
      50% { transform: rotateZ(-42.5deg) scaleY(0.97); }
      100% { transform: rotateZ(0deg) scaleY(1); }
    }

    .barrier-status {
      font-size: 2.5rem;
      font-weight: 900;
      margin: var(--spacing-md) 0;
    }

    .barrier-mode-toggle {
      display: flex;
      gap: var(--spacing-md);
      justify-content: center;
      margin: var(--spacing-xl) 0;
      flex-wrap: wrap;
    }

    /* ========================================
       MODE TOGGLE BUTTONS
       ======================================== */
    .mode-btn {
      padding: var(--spacing-md) var(--spacing-xl);
      border-radius: var(--radius-md);
      border: 2px solid var(--primary);
      background: transparent;
      color: var(--primary);
      font-weight: 600;
      cursor: pointer;
      transition: all var(--transition-fast);
      display: flex;
      align-items: center;
      gap: var(--spacing-sm);
      font-size: 1rem;
      position: relative;
      overflow: hidden;
    }

    .mode-btn::before {
      content: '';
      position: absolute;
      top: 50%;
      left: 50%;
      width: 0;
      height: 0;
      background: rgba(0, 212, 255, 0.3);
      border-radius: 50%;
      transform: translate(-50%, -50%);
      transition: width 0.6s, height 0.6s;
    }

    .mode-btn:hover::before {
      width: 300px;
      height: 300px;
    }

    .mode-btn.active {
      background: linear-gradient(90deg, var(--primary), var(--secondary));
      color: white;
      box-shadow: 0 0 20px rgba(0, 212, 255, 0.5);
      transform: scale(1.05);
    }

    .mode-btn:hover {
      box-shadow: 0 0 15px rgba(0, 212, 255, 0.3);
    }

    /* ========================================
       MANUAL CONTROL BUTTONS
       ======================================== */
    .manual-controls {
      display: flex;
      gap: var(--spacing-md);
      justify-content: center;
      flex-wrap: wrap;
      margin-top: var(--spacing-lg);
    }

    .action-btn {
      padding: var(--spacing-md) var(--spacing-xl);
      border-radius: var(--radius-md);
      border: none;
      font-weight: 600;
      cursor: pointer;
      transition: all var(--transition-fast);
      font-size: 1rem;
      display: flex;
      align-items: center;
      gap: var(--spacing-sm);
      position: relative;
      overflow: hidden;
    }

    .action-btn::before {
      content: '';
      position: absolute;
      top: 50%;
      left: 50%;
      width: 0;
      height: 0;
      background: rgba(255, 255, 255, 0.3);
      border-radius: 50%;
      transform: translate(-50%, -50%);
      transition: width 0.6s, height 0.6s;
    }

    .action-btn:hover::before {
      width: 300px;
      height: 300px;
    }

    .action-btn-open {
      background: linear-gradient(90deg, var(--success), #00cc66);
      color: #000;
    }

    .action-btn-open:hover {
      box-shadow: 0 0 30px rgba(0, 255, 136, 0.6);
      transform: translateY(-3px);
    }

    .action-btn-close {
      background: linear-gradient(90deg, var(--danger), #cc0044);
      color: white;
    }

    .action-btn-close:hover {
      box-shadow: 0 0 30px rgba(255, 0, 85, 0.6);
      transform: translateY(-3px);
    }

    /* ========================================
       COUNTDOWN DISPLAY
       ======================================== */
    .countdown-section {
      margin-top: var(--spacing-xl);
    }

    .countdown-label {
      font-size: 1.125rem;
      font-weight: 600;
      color: #64748b;
      text-transform: uppercase;
      letter-spacing: 0.05em;
      text-align: center;
      margin-bottom: var(--spacing-md);
    }

    .countdown-display {
      background: rgba(30, 41, 59, 0.5);
      border: 1px solid rgba(0, 212, 255, 0.2);
      border-radius: var(--radius-lg);
      padding: var(--spacing-lg);
      text-align: center;
      font-family: 'Courier New', monospace;
      font-size: 3rem;
      font-weight: 900;
      letter-spacing: 0.1em;
    }

    body.light-mode .countdown-display {
      background: rgba(226, 232, 240, 0.5);
    }

    /* ========================================
       TIME INTERVAL TABS
       ======================================== */
    .time-interval-tabs {
      display: flex;
      gap: var(--spacing-sm);
      margin-bottom: var(--spacing-lg);
      justify-content: center;
      flex-wrap: wrap;
    }

    .time-btn {
      padding: var(--spacing-sm) var(--spacing-md);
      border-radius: var(--radius-sm);
      border: 1px solid rgba(0, 212, 255, 0.3);
      background: transparent;
      color: #64748b;
      font-weight: 600;
      cursor: pointer;
      transition: all var(--transition-fast);
      font-size: 0.875rem;
    }

    .time-btn.active {
      background: linear-gradient(90deg, var(--primary), var(--secondary));
      color: white;
      border-color: var(--primary);
      box-shadow: 0 0 15px rgba(0, 212, 255, 0.4);
    }

    .time-btn:hover {
      border-color: var(--primary);
      color: var(--primary);
    }

    /* ========================================
       CHART CONTAINER
       ======================================== */
    .chart-container {
      position: relative;
      width: 100%;
      height: 450px;
      margin-bottom: var(--spacing-xl);
      background: rgba(30, 41, 59, 0.5);
      border-radius: var(--radius-lg);
      padding: var(--spacing-lg);
      border: 1px solid rgba(0, 212, 255, 0.1);
    }

    body.light-mode .chart-container {
      background: rgba(226, 232, 240, 0.5);
    }

    canvas {
      width: 100% !important;
      height: 100% !important;
    }

    .legend {
      display: flex;
      gap: var(--spacing-xl);
      justify-content: center;
      font-size: 0.875rem;
      margin-top: var(--spacing-md);
      flex-wrap: wrap;
    }

    .legend-item {
      display: flex;
      align-items: center;
      gap: var(--spacing-sm);
      color: #94a3b8;
    }

    .legend-dot {
      width: 12px;
      height: 12px;
      border-radius: 50%;
    }

    /* ========================================
       ALERTS SECTION
       ======================================== */
    .alerts-container {
      display: flex;
      gap: var(--spacing-lg);
      flex-wrap: wrap;
    }

    .alerts-tabs {
      display: flex;
      gap: var(--spacing-sm);
      margin-bottom: var(--spacing-lg);
      border-bottom: 2px solid rgba(0, 212, 255, 0.2);
    }

    .alerts-tab-btn {
      padding: 0.75rem var(--spacing-lg);
      border: none;
      background: transparent;
      color: #64748b;
      font-weight: 600;
      cursor: pointer;
      border-bottom: 3px solid transparent;
      transition: all var(--transition-fast);
    }

    .alerts-tab-btn.active {
      color: var(--primary);
      border-bottom-color: var(--primary);
    }

    .alerts-list {
      max-height: 600px;
      overflow-y: auto;
    }

    .alert-item {
      padding: 1.25rem;
      border-radius: var(--radius-lg);
      margin-bottom: var(--spacing-md);
      border-left: 4px solid;
      transition: all var(--transition-fast);
      backdrop-filter: blur(10px);
    }

    .alert-item:hover {
      transform: translateX(5px);
      box-shadow: 0 10px 30px rgba(0, 0, 0, 0.2);
    }

    .alert-critical {
      background: linear-gradient(135deg, rgba(255, 0, 85, 0.15) 0%, rgba(255, 0, 85, 0.05) 100%);
      border-left-color: var(--danger);
    }

    .alert-info {
      background: linear-gradient(135deg, rgba(0, 212, 255, 0.15) 0%, rgba(0, 212, 255, 0.05) 100%);
      border-left-color: var(--primary);
    }

    .alert-header {
      display: flex;
      justify-content: space-between;
      align-items: start;
      margin-bottom: 0.75rem;
    }

    .alert-msg {
      font-weight: 600;
      font-size: 0.95rem;
    }

    .alert-critical .alert-msg {
      color: #ff4466;
    }

    .alert-info .alert-msg {
      color: var(--primary);
    }

    .alert-time {
      font-size: 0.75rem;
      color: #64748b;
    }

    .alert-details {
      font-size: 0.8rem;
      color: #94a3b8;
      line-height: 1.5;
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: var(--spacing-sm);
    }

    .no-alerts {
      text-align: center;
      padding: var(--spacing-xl);
      color: #64748b;
    }

    /* ========================================
       MODAL OVERLAY & DIALOG
       ======================================== */
    .modal-overlay {
      display: none;
      position: fixed;
      top: 0;
      left: 0;
      right: 0;
      bottom: 0;
      background: rgba(0, 0, 0, 0.8);
      backdrop-filter: blur(5px);
      z-index: 1000;
      align-items: center;
      justify-content: center;
      animation: fadeIn 0.3s ease;
    }

    .modal-overlay.active {
      display: flex;
    }

    .modal {
      background: linear-gradient(135deg, var(--dark-card) 0%, var(--dark-bg) 100%);
      border: 2px solid var(--danger);
      border-radius: var(--radius-2xl);
      padding: 3rem var(--spacing-xl);
      max-width: 500px;
      width: 90%;
      text-align: center;
      box-shadow: 0 0 60px rgba(255, 0, 85, 0.5);
      animation: slideUp 0.5s ease;
    }

    body.light-mode .modal {
      background: linear-gradient(135deg, var(--light-bg) 0%, #e2e8f0 100%);
      color: var(--dark-card);
      border-color: var(--critical);
    }

    @keyframes slideUp {
      from {
        opacity: 0;
        transform: translateY(30px);
      }
      to {
        opacity: 1;
        transform: translateY(0);
      }
    }

    .modal-icon {
      font-size: 4rem;
      margin-bottom: var(--spacing-md);
      animation: pulse 1s ease-in-out infinite;
    }

    .modal-title {
      font-size: 2rem;
      font-weight: 900;
      color: var(--danger);
      margin-bottom: var(--spacing-sm);
    }

    .modal-desc {
      color: #94a3b8;
      margin-bottom: var(--spacing-xl);
      line-height: 1.6;
    }

    .modal-actions {
      display: flex;
      gap: var(--spacing-md);
      justify-content: center;
      flex-wrap: wrap;
    }

    .modal-btn {
      padding: var(--spacing-md) var(--spacing-xl);
      border-radius: var(--radius-md);
      border: none;
      font-weight: 600;
      cursor: pointer;
      transition: all var(--transition-fast);
      font-size: 1rem;
      position: relative;
      overflow: hidden;
    }

    .modal-btn::before {
      content: '';
      position: absolute;
      top: 50%;
      left: 50%;
      width: 0;
      height: 0;
      background: rgba(255, 255, 255, 0.3);
      border-radius: 50%;
      transform: translate(-50%, -50%);
      transition: width 0.6s, height 0.6s;
    }

    .modal-btn:hover::before {
      width: 300px;
      height: 300px;
    }

    .modal-btn-close-barrier {
      background: linear-gradient(90deg, var(--danger), #cc0044);
      color: white;
    }

    .modal-btn-close-barrier:hover {
      box-shadow: 0 0 30px rgba(255, 0, 85, 0.6);
      transform: translateY(-3px);
    }

    .modal-btn-dismiss {
      background: transparent;
      color: var(--primary);
      border: 2px solid var(--primary);
    }

    .modal-btn-dismiss:hover {
      background: rgba(0, 212, 255, 0.1);
      box-shadow: 0 0 20px rgba(0, 212, 255, 0.4);
    }

    /* ========================================
       TOAST NOTIFICATION
       ======================================== */
    .toast {
      position: fixed;
      bottom: var(--spacing-xl);
      right: var(--spacing-xl);
      max-width: 400px;
      padding: var(--spacing-lg);
      border-radius: var(--radius-lg);
      border-left: 4px solid;
      background: rgba(30, 41, 59, 0.95);
      backdrop-filter: blur(10px);
      z-index: 999;
      animation: slideIn 0.4s ease;
      box-shadow: 0 10px 40px rgba(0, 0, 0, 0.3);
    }

    body.light-mode .toast {
      background: rgba(248, 250, 252, 0.95);
      color: var(--dark-card);
    }

    @keyframes slideIn {
      from {
        opacity: 0;
        transform: translateX(400px);
      }
      to {
        opacity: 1;
        transform: translateX(0);
      }
    }

    .toast.critical { border-left-color: var(--danger); }
    .toast.info { border-left-color: var(--primary); }
    .toast.success { border-left-color: var(--success); }

    .toast-title {
      font-weight: 600;
      margin-bottom: 0.25rem;
    }

    .toast.critical .toast-title { color: var(--danger); }
    .toast.info .toast-title { color: var(--primary); }
    .toast.success .toast-title { color: var(--success); }

    .toast-msg {
      font-size: 0.9rem;
      color: #94a3b8;
      line-height: 1.5;
    }

    .toast-close {
      position: absolute;
      top: var(--spacing-md);
      right: var(--spacing-md);
      background: none;
      border: none;
      color: #64748b;
      cursor: pointer;
      font-size: 1.5rem;
      transition: color var(--transition-fast);
    }

    .toast-close:hover {
      color: var(--primary);
    }

    /* ========================================
       CUSTOM SCROLLBAR
       ======================================== */
    .alerts-list::-webkit-scrollbar {
      width: 8px;
    }

    .alerts-list::-webkit-scrollbar-track {
      background: rgba(0, 212, 255, 0.1);
      border-radius: 10px;
    }

    .alerts-list::-webkit-scrollbar-thumb {
      background: rgba(0, 212, 255, 0.3);
      border-radius: 10px;
    }

    .alerts-list::-webkit-scrollbar-thumb:hover {
      background: rgba(0, 212, 255, 0.5);
    }

    /* ========================================
       RESPONSIVE DESIGN
       ======================================== */
    @media (max-width: 768px) {
      main {
        padding: var(--spacing-md);
      }

      .grid {
        grid-template-columns: 1fr;
      }

      .sensor-value {
        font-size: 2rem;
      }

      .status-title {
        font-size: 2rem;
      }

      .barrier-icon {
        font-size: 3.5rem;
      }

      .modal {
        padding: var(--spacing-xl) var(--spacing-lg);
      }

      .toast {
        right: var(--spacing-md);
        bottom: var(--spacing-md);
        max-width: calc(100% - 2rem);
      }

      .nav-tabs {
        flex-wrap: nowrap;
      }

      .alert-details {
        grid-template-columns: 1fr;
      }
    }
  </style>
</head>
<body>
  <!-- ========================================
       HAZARD ALERT MODAL
       ======================================== -->
  <div class="modal-overlay" id="hazardModal" role="dialog" aria-labelledby="modalTitle" aria-describedby="modalDesc">
    <div class="modal">
      <div class="modal-icon" aria-hidden="true">🚨</div>
      <h2 class="modal-title" id="modalTitle">HAZARDOUS LEVEL DETECTED</h2>
      <p class="modal-desc" id="modalDesc">
        Dangerous air quality levels detected. Take immediate action to protect occupants.
      </p>
      <div class="modal-actions">
        <button class="modal-btn modal-btn-close-barrier" onclick="closeBarrier()" aria-label="Close barrier immediately">
          🔒 Close Barrier
        </button>
        <button class="modal-btn modal-btn-dismiss" onclick="dismissModal()" aria-label="Dismiss alert">
          Dismiss
        </button>
      </div>
    </div>
  </div>

  <!-- ========================================
       HEADER SECTION
       ======================================== -->
  <header role="banner">
    <div class="header-content">
      <div class="logo-section">
        <div class="icon" aria-hidden="true">🌫️</div>
        <div>
          <h1>AIR GUARDIAN</h1>
          <p>Real-time Air Quality Monitor</p>
        </div>
      </div>
      <button class="theme-toggle" onclick="toggleTheme()" title="Toggle Dark/Light Mode" aria-label="Toggle theme">
        ☀️
      </button>
    </div>
  </header>

  <!-- ========================================
       MAIN CONTENT
       ======================================== -->
  <main role="main">
    <!-- Navigation Tabs -->
    <nav class="nav-tabs" role="tablist" aria-label="Main navigation">
      <button class="tab-btn active" onclick="switchTab('dashboard')" role="tab" aria-selected="true" aria-controls="dashboard">
        📊 Dashboard
      </button>
      <button class="tab-btn" onclick="switchTab('barrier')" role="tab" aria-selected="false" aria-controls="barrier">
        🚧 Barrier
      </button>
      <button class="tab-btn" onclick="switchTab('alerts')" role="tab" aria-selected="false" aria-controls="alerts">
        ⚠️ Alerts
      </button>
      <button class="tab-btn" onclick="switchTab('history')" role="tab" aria-selected="false" aria-controls="history">
        📈 History
      </button>
    </nav>

    <!-- ========================================
         DASHBOARD TAB
         ======================================== -->
    <section class="tab-content active" id="dashboard" role="tabpanel" aria-labelledby="Dashboard">
      <div class="grid">
        <!-- Overall Air Quality Status -->
        <article class="card grid-full status-card" id="statusCard">
          <div class="status-icon" aria-hidden="true">✅</div>
          <h2 class="status-title status-good" id="statusTitle">GOOD</h2>
          <p class="status-desc" id="statusDesc">Air is safe</p>
          <div class="led-indicators" aria-label="Air quality LEDs">
            <div id="ledItemGreen" class="led-item item-green">
              <div id="ledGreen" class="led led-green" title="Green: GOOD" aria-hidden="true"></div>
              <span class="led-label">Safe</span>
            </div>
            <div id="ledItemYellow" class="led-item item-yellow">
              <div id="ledYellow" class="led led-yellow" title="Yellow: MODERATE" aria-hidden="true"></div>
              <span class="led-label">Warning</span>
            </div>
            <div id="ledItemRed" class="led-item item-red">
              <div id="ledRed" class="led led-red" title="Red: POOR/HAZARDOUS" aria-hidden="true"></div>
              <span class="led-label">Hazard</span>
            </div>
          </div>
        </article>

        <!-- Temperature Sensor -->
        <article class="card sensor-card">
          <div class="sensor-header">
            <span class="sensor-label">🌡️ Temperature</span>
            <span class="sensor-icon" aria-hidden="true">🌡️</span>
          </div>
          <div class="sensor-value" id="tempValue" aria-label="Temperature reading">--</div>
          <div class="sensor-unit">°C</div>
        </article>

        <!-- Humidity Sensor -->
        <article class="card sensor-card">
          <div class="sensor-header">
            <span class="sensor-label">💧 Humidity</span>
            <span class="sensor-icon" aria-hidden="true">💧</span>
          </div>
          <div class="sensor-value" id="humValue" aria-label="Humidity reading">--</div>
          <div class="sensor-unit">%</div>
        </article>

        <!-- MQ2 Smoke Sensor -->
        <article class="card sensor-card">
          <div class="sensor-header">
            <span class="sensor-label">🚬 MQ2 (Smoke)</span>
            <span class="sensor-icon" aria-hidden="true">🚬</span>
          </div>
          <div class="sensor-value" id="mq2Value" aria-label="Smoke level reading">--</div>
          <div class="sensor-unit">ppm</div>
        </article>

        <!-- MQ7 Carbon Monoxide Sensor -->
        <article class="card sensor-card">
          <div class="sensor-header">
            <span class="sensor-label">☠️ MQ7 (CO)</span>
            <span class="sensor-icon" aria-hidden="true">☠️</span>
          </div>
          <div class="sensor-value" id="mq7Value" aria-label="Carbon monoxide reading">--</div>
          <div class="sensor-unit">ppm</div>
        </article>

        <!-- Dust Sensor -->
        <article class="card sensor-card">
          <div class="sensor-header">
            <span class="sensor-label">🌫️ Dust</span>
            <span class="sensor-icon" aria-hidden="true">🌫️</span>
          </div>
          <div class="sensor-value" id="dustValue" aria-label="Dust level reading">--</div>
          <div class="sensor-unit">µg/m³</div>
        </article>

        <!-- Sound Level Sensor -->
        <article class="card sensor-card">
          <div class="sensor-header">
            <span class="sensor-label">🔊 Sound</span>
            <span class="sensor-icon" aria-hidden="true">🔊</span>
          </div>
          <div class="sensor-value" id="soundValue" aria-label="Sound level reading">--</div>
          <div class="sensor-unit">dB</div>
        </article>
      </div>
    </section>

    <!-- ========================================
         BARRIER CONTROL TAB
         ======================================== -->
    <section class="tab-content" id="barrier" role="tabpanel" aria-labelledby="Barrier">
      <article class="card barrier-section">
        <div class="barrier-icon open" id="barrierIcon" aria-hidden="true">🚧</div>
        <h2 class="barrier-status" id="barrierStatus">🟢 OPEN</h2>
        <p style="color: #64748b; margin-bottom: 2rem;">Barrier Status</p>

        <!-- Mode Toggle: Auto vs Manual -->
        <div class="barrier-mode-toggle" role="radiogroup" aria-label="Barrier control mode">
          <button class="mode-btn active" id="autoModeBtn" onclick="setMode('auto')" role="radio" aria-checked="true">
            ⚙️ Auto Mode
          </button>
          <button class="mode-btn" id="manualModeBtn" onclick="setMode('manual')" role="radio" aria-checked="false">
            🖱️ Manual Mode
          </button>
        </div>

        <!-- Manual Control Buttons (Hidden by default) -->
        <div class="manual-controls" id="manualControls" style="display: none;">
          <button class="action-btn action-btn-open" onclick="barrierCmd('open')" aria-label="Open barrier manually">
            🔓 Open Barrier
          </button>
          <button class="action-btn action-btn-close" onclick="barrierCmd('close')" aria-label="Close barrier manually">
            🔒 Close Barrier
          </button>
        </div>

        <!-- Countdown Timer -->
        <div class="countdown-section">
          <h3 class="countdown-label">⏱️ Countdown</h3>
          <div class="countdown-display" id="countdownDisplay" aria-live="polite" aria-atomic="true">--:--:--</div>
        </div>
      </article>
    </section>

    <!-- ========================================
         ALERTS TAB
         ======================================== -->
    <section class="tab-content" id="alerts" role="tabpanel" aria-labelledby="Alerts">
      <article class="card">
        <h2 style="margin-bottom: 1.5rem; font-size: 1.5rem;">⚠️ Alert Dashboard</h2>
        
        <!-- Alert Filter Tabs -->
        <div class="alerts-tabs" role="tablist" aria-label="Alert filters">
          <button class="alerts-tab-btn active" onclick="filterAlerts('all')" role="tab" aria-selected="true">
            All Alerts
          </button>
          <button class="alerts-tab-btn" onclick="filterAlerts('critical')" role="tab" aria-selected="false">
            Critical
          </button>
          <button class="alerts-tab-btn" onclick="filterAlerts('info')" role="tab" aria-selected="false">
            Info
          </button>
        </div>
        
        <!-- Alerts List -->
        <div class="alerts-list" id="alertsList" role="log" aria-live="polite" aria-relevant="additions">
          <div class="no-alerts">No alerts</div>
        </div>
      </article>
    </section>

    <!-- ========================================
         HISTORY TAB (CHARTS)
         ======================================== -->
    <section class="tab-content" id="history" role="tabpanel" aria-labelledby="History">
      <article class="card">
        <h2 style="margin-bottom: 1.5rem; font-size: 1.5rem;">📈 Sensor History</h2>
        
        <!-- Air Quality Charts Section -->
        <section style="margin-bottom: 2rem;">
          <h3 style="margin-bottom: 1rem; color: #00d4ff; font-size: 1.1rem;">Air Quality Sensors</h3>
          
          <!-- Time Interval Selector -->
          <div class="time-interval-tabs" role="tablist" aria-label="Air quality time interval">
            <button class="time-btn active" onclick="switchTimeInterval('1h', 'airquality')" role="tab" aria-selected="true">
              Live
            </button>
            <button class="time-btn" onclick="switchTimeInterval('6h', 'airquality')" role="tab" aria-selected="false">
              6h
            </button>
            <button class="time-btn" onclick="switchTimeInterval('1d', 'airquality')" role="tab" aria-selected="false">
              1d
            </button>
            <button class="time-btn" onclick="switchTimeInterval('1w', 'airquality')" role="tab" aria-selected="false">
              1w
            </button>
            <button class="time-btn" onclick="switchTimeInterval('1m', 'airquality')" role="tab" aria-selected="false">
              1M
            </button>
          </div>
          
          <!-- Chart Canvas -->
          <div class="chart-container">
            <canvas id="airQualityChart" aria-label="Air quality sensor history chart"></canvas>
          </div>
          
          <!-- Chart Legend -->
          <div class="legend" role="list" aria-label="Chart legend">
            <div class="legend-item" role="listitem">
              <div class="legend-dot" style="background-color: #00ff88;" aria-hidden="true"></div>
              <span>MQ2 (Smoke)</span>
            </div>
            <div class="legend-item" role="listitem">
              <div class="legend-dot" style="background-color: #ffb000;" aria-hidden="true"></div>
              <span>MQ7 (CO)</span>
            </div>
            <div class="legend-item" role="listitem">
              <div class="legend-dot" style="background-color: #00d4ff;" aria-hidden="true"></div>
              <span>Dust</span>
            </div>
          </div>
        </section>

        <!-- Environmental Conditions Charts Section -->
        <section style="margin-bottom: 2rem;">
          <h3 style="margin-bottom: 1rem; color: #00d4ff; font-size: 1.1rem;">Environmental Conditions</h3>
          
          <!-- Time Interval Selector -->
          <div class="time-interval-tabs" role="tablist" aria-label="Environmental time interval">
            <button class="time-btn active" onclick="switchTimeInterval('1h', 'environmental')" role="tab" aria-selected="true">
              Live
            </button>
            <button class="time-btn" onclick="switchTimeInterval('6h', 'environmental')" role="tab" aria-selected="false">
              6h
            </button>
            <button class="time-btn" onclick="switchTimeInterval('1d', 'environmental')" role="tab" aria-selected="false">
              1d
            </button>
            <button class="time-btn" onclick="switchTimeInterval('1w', 'environmental')" role="tab" aria-selected="false">
              1w
            </button>
            <button class="time-btn" onclick="switchTimeInterval('1m', 'environmental')" role="tab" aria-selected="false">
              1M
            </button>
          </div>
          
          <!-- Chart Canvas -->
          <div class="chart-container">
            <canvas id="environmentalChart" aria-label="Environmental conditions history chart"></canvas>
          </div>
          
          <!-- Chart Legend -->
          <div class="legend" role="list" aria-label="Chart legend">
            <div class="legend-item" role="listitem">
              <div class="legend-dot" style="background-color: #ff6b9d;" aria-hidden="true"></div>
              <span>Temperature</span>
            </div>
            <div class="legend-item" role="listitem">
              <div class="legend-dot" style="background-color: #4ecdc4;" aria-hidden="true"></div>
              <span>Humidity</span>
            </div>
            <div class="legend-item" role="listitem">
              <div class="legend-dot" style="background-color: #ffe66d;" aria-hidden="true"></div>
              <span>Sound</span>
            </div>
          </div>
        </section>
      </article>
    </section>
  </main>

  <!-- ========================================
       JAVASCRIPT APPLICATION LOGIC
       ======================================== -->
  <script>
    'use strict';

    /* ==========================================
       APPLICATION STATE
       ========================================== */
    const AppState = {
      darkMode: true,
      barrierMode: 'auto',
      allAlerts: [],
      currentAlertFilter: 'all',
      chartData: {
        labels: [],
        timestamps: [],
        mq2: [],
        mq7: [],
        dust: [],
        temp: [],
        hum: [],
        sound: []
      },
      lastHazardStatus: null,
      hazardAlertCooldown: 0,
      currentTimeInterval: {
        airquality: '1h',
        environmental: '1h'
      }
    };

    /* ==========================================
       THEME MANAGEMENT
       ========================================== */
    function toggleTheme() {
      AppState.darkMode = !AppState.darkMode;
      document.body.classList.toggle('light-mode', !AppState.darkMode);
      localStorage.setItem('darkMode', AppState.darkMode ? 'true' : 'false');
      document.querySelector('.theme-toggle').textContent = AppState.darkMode ? '☀️' : '🌙';
    }

    // Initialize theme from localStorage
    function initializeTheme() {
      const savedTheme = localStorage.getItem('darkMode');
      if (savedTheme === 'false') {
        AppState.darkMode = false;
        document.body.classList.add('light-mode');
        document.querySelector('.theme-toggle').textContent = '🌙';
      }
    }

    /* ==========================================
       TAB NAVIGATION
       ========================================== */
    function switchTab(tabName) {
      const tabs = document.querySelectorAll('.tab-content');
      const buttons = document.querySelectorAll('.tab-btn');

      // Remove active class from all tabs and buttons
      tabs.forEach(tab => tab.classList.remove('active'));
      buttons.forEach(btn => {
        btn.classList.remove('active');
        btn.setAttribute('aria-selected', 'false');
      });

      // Activate selected tab and button
      const selectedTab = document.getElementById(tabName);
      const selectedButton = event.target.closest('.tab-btn');
      
      selectedTab.classList.add('active');
      selectedButton.classList.add('active');
      selectedButton.setAttribute('aria-selected', 'true');

      // Redraw charts when history tab is opened
      if (tabName === 'history') {
        setTimeout(() => {
          drawChart('airQualityChart', ['mq2', 'mq7', 'dust']);
          drawChart('environmentalChart', ['temp', 'hum', 'sound']);
        }, 100);
      }
    }

    /* ==========================================
       TIME INTERVAL MANAGEMENT
       ========================================== */
    function switchTimeInterval(interval, chartType) {
      AppState.currentTimeInterval[chartType] = interval;
      
      const buttons = event.target.closest('.time-interval-tabs').querySelectorAll('.time-btn');
      buttons.forEach(btn => {
        btn.classList.remove('active');
        btn.setAttribute('aria-selected', 'false');
      });
      
      event.target.classList.add('active');
      event.target.setAttribute('aria-selected', 'true');

      // Redraw appropriate chart
      if (chartType === 'airquality') {
        drawChart('airQualityChart', ['mq2', 'mq7', 'dust']);
      } else {
        drawChart('environmentalChart', ['temp', 'hum', 'sound']);
      }
    }

    /* ==========================================
       ALERT FILTERING
       ========================================== */
    function filterAlerts(filter) {
      AppState.currentAlertFilter = filter;
      
      const buttons = document.querySelectorAll('.alerts-tab-btn');
      buttons.forEach(btn => {
        btn.classList.remove('active');
        btn.setAttribute('aria-selected', 'false');
      });
      
      event.target.classList.add('active');
      event.target.setAttribute('aria-selected', 'true');
      
      updateAlerts(AppState.allAlerts);
    }

    /* ==========================================
       BARRIER CONTROL
       ========================================== */
    function setMode(mode, sendCommand = true) {
      AppState.barrierMode = mode;
      
      const autoBtn = document.getElementById('autoModeBtn');
      const manualBtn = document.getElementById('manualModeBtn');
      const manualControls = document.getElementById('manualControls');
      
      autoBtn.classList.toggle('active', mode === 'auto');
      autoBtn.setAttribute('aria-checked', mode === 'auto');
      
      manualBtn.classList.toggle('active', mode === 'manual');
      manualBtn.setAttribute('aria-checked', mode === 'manual');
      
      manualControls.style.display = mode === 'manual' ? 'flex' : 'none';
      
      if (sendCommand) {
        fetch(`/api/sensors?barrier_cmd=${mode === 'auto' ? 'auto' : 'manual'}`)
          .catch(err => console.error('Barrier mode error:', err));
      }
    }

    function barrierCmd(cmd) {
      fetch(`/api/sensors?barrier_cmd=${cmd}`)
        .catch(err => console.error('Barrier command error:', err));
      
      showToast(`Barrier ${cmd}`, `Sending ${cmd} command...`, 'info');
      
      // Trigger barrier animation
      const icon = document.getElementById('barrierIcon');
      icon.classList.remove('open', 'closed');
      setTimeout(() => {
        icon.classList.add(cmd === 'open' ? 'open' : 'closed');
      }, 50);
    }

    function closeBarrier() {
      barrierCmd('close');
      dismissModal();
      showToast('⚠️ Safety Action', 'Barrier closed. Notify occupants to wear masks.', 'critical');
    }

    /* ==========================================
       MODAL CONTROL
       ========================================== */
    function showHazardModal() {
      if (AppState.hazardAlertCooldown > Date.now()) return;
      
      document.getElementById('hazardModal').classList.add('active');
      AppState.hazardAlertCooldown = Date.now() + 30000; // 30 second cooldown
    }

    function dismissModal() {
      document.getElementById('hazardModal').classList.remove('active');
    }

    /* ==========================================
       TOAST NOTIFICATIONS
       ========================================== */
    function showToast(title, message, severity = 'info') {
      const toast = document.createElement('div');
      toast.className = `toast ${severity}`;
      toast.setAttribute('role', 'alert');
      toast.setAttribute('aria-live', 'assertive');
      
      toast.innerHTML = `
        <button class="toast-close" onclick="this.parentElement.remove()" aria-label="Close notification">×</button>
        <div class="toast-title">${title}</div>
        <div class="toast-msg">${message}</div>
      `;
      
      document.body.appendChild(toast);
      
      // Auto-remove after 5 seconds
      setTimeout(() => toast.remove(), 5000);
    }

    /* ==========================================
       SENSOR DATA UI UPDATE
       ========================================== */
    function updateUI(data) {
      // Update sensor values
      document.getElementById('tempValue').textContent = data.Temperature.toFixed(1);
      document.getElementById('humValue').textContent = data.Humidity.toFixed(1);
      document.getElementById('mq2Value').textContent = data.MQ2;
      document.getElementById('mq7Value').textContent = data.MQ7;
      document.getElementById('dustValue').textContent = data.Dust;
      document.getElementById('soundValue').textContent = data.Sound;

      // Update air quality status
      const statusConfig = {
        'GOOD': { icon: '✅', color: 'status-good', desc: 'Air is safe' },
        'MODERATE': { icon: '⚠️', color: 'status-moderate', desc: 'Caution advised' },
        'POOR': { icon: '⚠️', color: 'status-poor', desc: 'Poor conditions' },
        'HAZARDOUS': { icon: '🚨', color: 'status-hazardous', desc: 'Critical alert' }
      };

      const config = statusConfig[data.Status];
      if (config) {
        document.getElementById('statusCard').querySelector('.status-icon').textContent = config.icon;
        document.getElementById('statusTitle').textContent = data.Status;
        document.getElementById('statusTitle').className = `status-title ${config.color}`;
        document.getElementById('statusDesc').textContent = config.desc;
      }

      // Show hazard modal if status becomes hazardous
      if (data.Status === 'HAZARDOUS' && AppState.lastHazardStatus !== 'HAZARDOUS') {
        showHazardModal();
        showToast('🚨 Critical Alert', 'Hazardous air quality detected! Close the barrier immediately.', 'critical');
      }
      AppState.lastHazardStatus = data.Status;

      // Update LED indicators
      const ledG = document.getElementById('ledGreen');
      const ledY = document.getElementById('ledYellow');
      const ledR = document.getElementById('ledRed');
      const ledItemG = document.getElementById('ledItemGreen');
      const ledItemY = document.getElementById('ledItemYellow');
      const ledItemR = document.getElementById('ledItemRed');
      
      const isGood = data.Status === 'GOOD';
      const isModerate = data.Status === 'MODERATE';
      const isPoorOrHazard = data.Status === 'POOR' || data.Status === 'HAZARDOUS';
      
      ledG.classList.toggle('active', isGood);
      ledY.classList.toggle('active', isModerate);
      ledR.classList.toggle('active', isPoorOrHazard);
      ledItemG.classList.toggle('active', isGood);
      ledItemY.classList.toggle('active', isModerate);
      ledItemR.classList.toggle('active', isPoorOrHazard);

      // Update barrier mode
      AppState.barrierMode = data.BarrierAutoMode ? 'auto' : 'manual';
      setMode(AppState.barrierMode, false);

      // Update barrier status
      const barrierOpen = data.BarrierOpen;
      document.getElementById('barrierStatus').textContent = barrierOpen ? '🟢 OPEN' : '🔴 CLOSED';
      
      const icon = document.getElementById('barrierIcon');
      if (barrierOpen && icon.classList.contains('closed')) {
        icon.classList.remove('closed');
        icon.classList.add('open');
      } else if (!barrierOpen && icon.classList.contains('open')) {
        icon.classList.remove('open');
        icon.classList.add('closed');
      }

      // Update countdown timer
      if (!barrierOpen && data.BarrierExpiryIn > 0) {
        const totalSeconds = Math.floor(data.BarrierExpiryIn / 1000);
        const hours = Math.floor(totalSeconds / 3600);
        const minutes = Math.floor((totalSeconds % 3600) / 60);
        const seconds = totalSeconds % 60;
        
        document.getElementById('countdownDisplay').textContent = 
          `${String(hours).padStart(2, '0')}:${String(minutes).padStart(2, '0')}:${String(seconds).padStart(2, '0')}`;
      } else {
        document.getElementById('countdownDisplay').textContent = '--:--:--';
      }

      // Update chart data
      const now = new Date();
      const time = `${String(now.getHours()).padStart(2, '0')}:${String(now.getMinutes()).padStart(2, '0')}`;
      const timestamp = Date.now();
      
      AppState.chartData.labels.push(time);
      AppState.chartData.timestamps.push(timestamp);
      AppState.chartData.mq2.push(data.MQ2);
      AppState.chartData.mq7.push(data.MQ7);
      AppState.chartData.dust.push(data.Dust);
      AppState.chartData.temp.push(data.Temperature);
      AppState.chartData.hum.push(data.Humidity);
      AppState.chartData.sound.push(data.Sound);

      // Limit chart data to last 100 points
      if (AppState.chartData.labels.length > 100) {
        AppState.chartData.labels.shift();
        AppState.chartData.timestamps.shift();
        AppState.chartData.mq2.shift();
        AppState.chartData.mq7.shift();
        AppState.chartData.dust.shift();
        AppState.chartData.temp.shift();
        AppState.chartData.hum.shift();
        AppState.chartData.sound.shift();
      }
    }

    /* ==========================================
       ALERTS UI UPDATE
       ========================================== */
    function updateAlerts(alerts) {
      AppState.allAlerts = alerts;
      const alertsList = document.getElementById('alertsList');
      
      // Filter alerts based on current filter
      let filtered = alerts;
      if (AppState.currentAlertFilter === 'critical') {
        filtered = alerts.filter(a => a.sev === 'CRITICAL');
      } else if (AppState.currentAlertFilter === 'info') {
        filtered = alerts.filter(a => a.sev === 'INFO');
      }

      // Display message if no alerts
      if (filtered.length === 0) {
        alertsList.innerHTML = '<div class="no-alerts">No alerts</div>';
        return;
      }

      // Render alert items (limit to 20)
      alertsList.innerHTML = filtered.slice(0, 20).map(alert => `
        <div class="alert-item ${alert.sev === 'CRITICAL' ? 'alert-critical' : 'alert-info'}">
          <div class="alert-header">
            <div class="alert-msg">${alert.msg}</div>
            <div class="alert-time">${new Date(alert.ts).toLocaleTimeString()}</div>
          </div>
          <div class="alert-details">
            <div>MQ2: <strong>${alert.mq2}</strong> | MQ7: <strong>${alert.mq7}</strong></div>
            <div>Dust: <strong>${alert.dust}</strong> µg/m³ | Temp: <strong>${alert.temp.toFixed(1)}</strong>°C | Hum: <strong>${(typeof alert.humidity === 'number' ? alert.humidity : Number(alert.humidity || 0)).toFixed(1)}</strong>%</div>
          </div>
        </div>
      `).join('');
    }

    /* ==========================================
       CHART RENDERING
       ========================================== */
    function getWindowMs(chartType) {
      const interval = AppState.currentTimeInterval[chartType] || '1h';
      const map = {
        '1h': 60 * 60 * 1000,
        '6h': 6 * 60 * 60 * 1000,
        '1d': 24 * 60 * 60 * 1000,
        '1w': 7 * 24 * 60 * 60 * 1000,
        '1m': 30 * 24 * 60 * 60 * 1000
      };
      return map[interval] || map['1h'];
    }

    function filterChartData(datasets, chartType) {
      const windowMs = getWindowMs(chartType);
      const nowTs = Date.now();
      const startTs = nowTs - windowMs;
      const indices = [];

      AppState.chartData.timestamps.forEach((ts, idx) => {
        if (ts >= startTs) {
          indices.push(idx);
        }
      });

      const filteredTs = indices.map(i => AppState.chartData.timestamps[i]);
      const filteredSeries = {};
      datasets.forEach(name => {
        filteredSeries[name] = indices.map(i => AppState.chartData[name][i] || 0);
      });

      return { filteredTs, filteredSeries, startTs, windowMs };
    }

    function drawChart(canvasId, datasets) {
      const canvas = document.getElementById(canvasId);
      if (!canvas) return;

      const ctx = canvas.getContext('2d');
      const rect = canvas.getBoundingClientRect();
      canvas.width = rect.width;
      canvas.height = rect.height;

      const w = canvas.width;
      const h = canvas.height;

      // Clear canvas
      ctx.fillStyle = 'transparent';
      ctx.fillRect(0, 0, w, h);

      // Define colors for each dataset
      const colors = {
        mq2: '#00ff88',
        mq7: '#ffb000',
        dust: '#00d4ff',
        temp: '#ff6b9d',
        hum: '#4ecdc4',
        sound: '#ffe66d'
      };

      const { filteredTs, filteredSeries, startTs, windowMs } = filterChartData(datasets, canvasId === 'airQualityChart' ? 'airquality' : 'environmental');

      // Draw time axis grid to make gaps visible
      ctx.strokeStyle = 'rgba(255,255,255,0.08)';
      ctx.lineWidth = 1;
      ctx.beginPath();
      for (let i = 0; i <= 4; i++) {
        const x = (i / 4) * w;
        ctx.moveTo(x, h);
        ctx.lineTo(x, 0);
      }
      ctx.stroke();

      const allValues = [];
      datasets.forEach(name => {
        allValues.push(...filteredSeries[name]);
      });
      const maxVal = Math.max(...allValues, 1);
      const gapThreshold = windowMs / 30; // Break line when gaps exceed threshold

      // Function to draw a line for a dataset
      const drawLine = (arr, color) => {
        if (arr.length === 0 || filteredTs.length === 0) return;
        
        ctx.strokeStyle = color;
        ctx.lineWidth = 3;
        ctx.lineCap = 'round';
        ctx.lineJoin = 'round';
        ctx.beginPath();
        
        arr.forEach((v, i) => {
          const x = ((filteredTs[i] - startTs) / windowMs) * w;
          const y = h - (v / maxVal) * (h * 0.9);
          
          if (i === 0 || (i > 0 && filteredTs[i] - filteredTs[i - 1] > gapThreshold)) {
            ctx.moveTo(x, y);
          } else {
            ctx.lineTo(x, y);
          }
        });
        
        ctx.stroke();
      };

      // Draw each dataset
      datasets.forEach(dataset => {
        if (filteredSeries[dataset] && filteredSeries[dataset].length > 0) {
          drawLine(filteredSeries[dataset], colors[dataset]);
        }
      });
    }

    /* ==========================================
       API COMMUNICATION
       ========================================== */
    async function fetchSensors() {
      try {
        const response = await fetch(`/api/sensors?client_epoch=${Date.now()}`);
        if (response.ok) {
          const data = await response.json();
          updateUI(data);
        }
      } catch (error) {
        console.error('Sensor fetch error:', error);
      }
    }

    async function fetchAlerts() {
      try {
        const response = await fetch('/api/alerts');
        if (response.ok) {
          const data = await response.json();
          updateAlerts(data);
        }
      } catch (error) {
        console.error('Alerts fetch error:', error);
      }
    }

    /* ==========================================
       APPLICATION INITIALIZATION & EVENT LISTENERS
       ========================================== */
    function initializeApp() {
      // Initialize theme
      initializeTheme();
      
      // Initial data fetch
      fetchSensors();
      fetchAlerts();

      // Set up polling intervals
      const sensorInterval = setInterval(fetchSensors, 2000);
      const alertInterval = setInterval(fetchAlerts, 3000);

      // Clean up intervals on page unload
      window.addEventListener('beforeunload', () => {
        clearInterval(sensorInterval);
        clearInterval(alertInterval);
      });

      // Modal click-outside-to-close
      document.getElementById('hazardModal').addEventListener('click', (e) => {
        if (e.target.id === 'hazardModal') {
          dismissModal();
        }
      });
    }

    // Start application when DOM is ready
    if (document.readyState === 'loading') {
      document.addEventListener('DOMContentLoaded', initializeApp);
    } else {
      initializeApp();
    }
  </script>
</body>
</html>
  )rawliteral";
  
  server.send(200, "text/html", html);
}

// Sensors API endpoint
void handleSensorsAPI() {
  if (!timeSynced && server.hasArg("client_epoch")) {
    String epochStr = server.arg("client_epoch");
    uint64_t clientEpoch = strtoull(epochStr.c_str(), nullptr, 10);
    if (clientEpoch > 1700000000000ULL) { // sanity check: after 2023
      manualTimeOffsetMs = static_cast<int64_t>(clientEpoch) - static_cast<int64_t>(millis());
      manualTimeSet = true;
    }
  }

  String json = getAirQualityJSON();
  server.send(200, "application/json", json);
  Serial.println("[Server] Request: GET /api/sensors");
}

// Alerts API endpoint
void handleAlertsAPI() {
  String json = "[";
  
  for (size_t i = 0; i < alertHistory.size(); i++) {
    if (i > 0) json += ",";
    
    Alert& alert = alertHistory[i];
    json += "{";
    json += "\"msg\":\"" + alert.message + "\",";
    json += "\"sev\":\"" + alert.severity + "\",";
    json += "\"ts\":" + String((uint64_t)alert.timestamp) + ",";
    json += "\"mq2\":" + String(alert.mq2) + ",";
    json += "\"mq7\":" + String(alert.mq7) + ",";
    json += "\"dust\":" + String(alert.dust) + ",";
    json += "\"temp\":" + String(alert.temp, 1) + ",";
    json += "\"humidity\":" + String(alert.humidity, 1);
    json += "}";
  }
  
  json += "]";
  
  server.send(200, "application/json", json);
  Serial.println("[Server] Request: GET /api/alerts");
}

// 404 handler
void handleNotFound() {
  String message = "404: Not Found\n\n";
  message += "URI: " + server.uri() + "\n";
  message += "Method: " + String((server.method() == HTTP_GET) ? "GET" : "POST") + "\n";
  
  server.send(404, "text/plain", message);
  Serial.println("[Server] 404: " + server.uri());
}

// =====================================================================
// JSON BUILDER FOR SENSOR DATA
// =====================================================================
String getAirQualityJSON() {
  String json = "{";
  
  // Sensor readings
  json += "\"Temperature\":" + String(state.temperature, 1) + ",";
  json += "\"Humidity\":" + String(state.humidity, 1) + ",";
  json += "\"MQ2\":" + String(state.mq2Value) + ",";
  json += "\"MQ7\":" + String(state.mq7Value) + ",";
  json += "\"Dust\":" + String(state.dustValue) + ",";
  json += "\"Sound\":" + String(state.soundValue) + ",";
  
  // Status
  json += "\"Status\":\"" + state.airQualityStatus + "\",";
  
  // Barrier info
  json += "\"BarrierOpen\":" + String(state.barrierOpen ? "true" : "false") + ",";
  json += "\"BarrierAutoMode\":" + String(state.barrierAutoMode ? "true" : "false") + ",";
  
  unsigned long expiryTime = 0;
  if (!state.barrierOpen) {
    unsigned long elapsed = millis() - state.barrierClosedTime;
    if (elapsed < state.barrierDuration) {
      expiryTime = state.barrierDuration - elapsed;
    }
  }
  json += "\"BarrierExpiryIn\":" + String(expiryTime) + ",";
  json += "\"TimeSynced\":" + String((timeSynced || manualTimeSet) ? "true" : "false") + ",";
  
  // Connection status
  json += "\"ESP8266Connected\":" + String(state.esp8266Connected ? "true" : "false");
  
  json += "}";
  
  return json;
}