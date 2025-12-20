# 🌬️ AIR GUARDIAN  
## Smart Air Quality Monitoring System 🛡️

**Version:** 2.0.0  
**Platform:** ESP32  
**Author:** Air Guardian Development Team (Group 1)

---

## 📖 Overview

**Air Guardian** is an advanced IoT-based air quality monitoring system designed to enhance safety in indoor environments. Powered by an **ESP32**, the system continuously monitors **temperature, humidity, smoke, carbon monoxide, dust, and sound levels** in real time.

A key feature of Air Guardian is its **Automated Barrier Control System 🚧**, which physically restricts access to a room when hazardous air conditions are detected. The system also provides a **standalone web dashboard 📊** for live data visualization and monitoring.

---

## ✨ Key Features

### 🔍 Multi-Sensor Fusion
- DHT11 – Temperature & Humidity  
- MQ2 – Smoke & Gas Detection  
- MQ7 – Carbon Monoxide Detection  
- Dust Sensor – Particulate Matter Detection  
- Sound Sensor – Noise Level Monitoring  

### 🚧 Automated Safety Barrier
- Servo motor automatically closes the barrier when air quality reaches **POOR** or **HAZARDOUS** levels
- Supports **Auto** and **Manual** control modes

### 🌐 Standalone Web Dashboard
- ESP32 operates in **WiFi Access Point (AP) mode**
- View live sensor data through gauges and charts
- No external internet connection required

<img width="1097" height="741" alt="Screenshot 2025-12-19 011257" src="https://github.com/user-attachments/assets/f9258992-e0a9-4536-ade4-2271fcd89f80" />
<img width="1611" height="855" alt="Screenshot 2025-12-19 010414" src="https://github.com/user-attachments/assets/00c9fc18-c3b2-4659-9fd6-f0609e5357d3" />


### 🔔 Intelligent Alert System
- Logs critical events such as hazardous gas levels
- Timestamped alerts for easier incident tracking

### 💾 Auto-Save Settings
- Barrier mode (Auto / Manual) is saved using **EEPROM**
- Settings persist even after power loss or reset

### 🚦 Visual & Audio Feedback
- Traffic light LED indicators:
  - 🟢 Green – Safe
  - 🟡 Yellow – Warning
  - 🔴 Red – Hazardous
- Buzzer and fan activation for immediate alerts and ventilation

### 🔌 RESTful API
- JSON-based endpoints for system integration
- Example endpoint:
