# HydroV1 - Automated Hydroponic Controller

## 📌 Project Overview
HydroV1 is an Arduino-based **automated hydroponic nutrient and pH management system**.  
It continuously monitors water quality parameters (pH, nutrient concentration via TDS, and tank levels) and automatically adjusts dosing pumps to maintain optimal growing conditions for different crop profiles.  

The system includes a **web-based dashboard** for live monitoring and control, an **LCD display** for local status updates, and EEPROM-based **calibration storage** for accuracy.

---

## ✨ Features
- Real-time monitoring of:
  - pH levels
  - TDS/EC (nutrient concentration)
  - Reservoir and container levels (via ultrasonic sensors)
- Automatic dosing control using relays (acid, base, nutrient, dump)
- Built-in buzzer alerts for low levels and errors
- Multiple crop profiles (leafy greens, fruiting crops, root crops, herbs, etc.)
- Local 16x2 LCD display with RGB backlight
- WiFi connectivity with built-in web dashboard
- EEPROM storage for sensor calibration
- Manual calibration via web interface

---

## 🛠 Hardware Requirements
- **Arduino UNO R4 WiFi** (or compatible with WiFiS3)
- **Sensors**:
  - Analog pH sensor (A0)
  - TDS sensor (A1)
  - Ultrasonic sensors (HC-SR04 or similar) for:
    - Reservoir
    - Acid container
    - Base container
    - Nutrient container
- **Actuators**:
  - 4x Relay module for:
    - Acid pump
    - Base pump
    - Nutrient pump
    - Dump valve/pump
- **Other**:
  - DFRobot RGB LCD1602
  - Piezo buzzer
  - WiFi network access

---

## 💻 Software Requirements
- **Arduino IDE 2.x**
- Libraries:
  - `WiFiS3.h` (Arduino R4 WiFi support)
  - `DFRobot_RGBLCD1602.h`
  - `EEPROM.h`

---

## ⚙️ Installation & Setup
1. Clone or download this repository.
2. Open `main.ino` in Arduino IDE.
3. Install required libraries via Arduino Library Manager.
4. Update WiFi credentials:
   ```cpp
   char ssid[] = "your_wifi_name";
   char pass[] = "your_wifi_password";
   ```
5. Optionally configure static IP settings.
6. Upload the code to your Arduino board.

---

## 🔄 User Flow
1. **Startup**:
   - LCD displays boot info.
   - WiFi connects and starts local server.
2. **Monitoring**:
   - Sensors continuously read pH, TDS, and water levels.
   - Data is shown on the LCD and dashboard.
3. **Control**:
   - System doses acid/base/nutrient when readings deviate from profile targets.
   - Attempts limited per cycle to prevent overdosing.
4. **Alerts**:
   - Low-level reservoirs trigger buzzer + dashboard alerts.
5. **Calibration**:
   - Accessible from dashboard interface.
   - Values saved to EEPROM for persistence.

---

## ⚡ Customization
- Crop profiles are defined in the `profiles[]` struct:
  ```cpp
  const Settings profiles[] = {
    {"Leafy", 5.9, 700, 0.4, 140, 5, 5, 5000UL, 5000UL, 100UL, 3000UL},
    {"Fruiting", 6.0, 1200, ... },
    ...
  };
  ```
  Modify these values to suit your crop requirements.

---

## 🐞 Troubleshooting
- **No WiFi connection** → Check SSID/password and router IP settings.
- **LCD not displaying** → Verify I2C address (default `0x2D`).
- **pH/TDS inaccurate** → Run calibration via dashboard and store values in EEPROM.
- **No dosing action** → Confirm relays are wired correctly and power supply is adequate.

---

## 📜 License
This project is released under the MIT License.  
You are free to use, modify, and distribute it with attribution.
