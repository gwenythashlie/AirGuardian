# 🌬️ AIR GUARDIAN 💻
## Smart Air Quality Monitoring System 🛡️

**Version:** 2.0.0  
**Platform:** ESP32 and ESP8266 
**Author:** Air Guardian Development Team (Group 1)

---

## 📖 Overview

**Air Guardian** is an advanced IoT-based air quality monitoring system designed to enhance safety in indoor environments. Powered by an **ESP32**, the system continuously monitors **temperature, humidity, smoke, carbon monoxide, dust, and sound levels** in real time.

A key feature of Air Guardian is its **Automated Barrier Control System 🚧**, which physically restricts access to a room when hazardous air conditions are detected. The system also provides a **standalone web dashboard 📊** for live data visualization and monitoring.

---

## Key Features
 Listed below are the the fucntions featured in the miniature/prototype:

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
<img width="1070" height="746" alt="Screenshot 2025-12-19 010010" src="https://github.com/user-attachments/assets/00a944be-d24c-48af-9e58-913b4d023a9e" />
<img width="1059" height="669" alt="Screenshot 2025-12-19 005855" src="https://github.com/user-attachments/assets/aa7f24be-d156-4038-bfcc-c3fc62c8c20a" />


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
- Buzzer and fan activation for immediate alerts
- I2C OLED for display using ESP8266

### 🔌 RESTful API
- JSON-based endpoints for system integration
- Example endpoint:

### 🔌 Actual Prototype
<img width="1113" height="789" alt="Screenshot 2025-12-19 080651" src="https://github.com/user-attachments/assets/f3a82fa0-bd86-40c8-9227-f063170767de" />
<img width="624" height="800" alt="Screenshot 2025-12-19 080303" src="https://github.com/user-attachments/assets/ab471211-fd53-42ea-8d88-2414a4d6990c" />
