// HydroV1_final
#include <WiFiS3.h>
#include <DFRobot_RGBLCD1602.h>
#include <EEPROM.h>

// -------------------- Pin Definitions --------------------
const int RELAY_ACID     = 2;
const int RELAY_BASE     = 3;
const int RELAY_NUTRIENT = 4;
const int RELAY_DUMP     = 5;

#define PH_PIN A0 // PH Sensor
#define TDS_PIN A1 // TDS Sensor
#define BUZZER_PIN A2 // Buzzer

// Ultrasonic sensors
const int TRIG_RES  = 6,  ECHO_RES  = 7;   // Reservoir
const int TRIG_ACID = 8,  ECHO_ACID = 9;   // Acid
const int TRIG_BASE = 10, ECHO_BASE = 11;  // Base
const int TRIG_NUT  = 12, ECHO_NUT  = 13;  // Nutrient

// -------------------- LCD --------------------
DFRobot_RGBLCD1602 lcd(0x2D, 16, 2);

// -------------------- WiFi / Server --------------------
char ssid[] = "wifiname"; // Change this
char pass[] = "wifipassword"; // Change this
WiFiServer server(80);

// Optional static IP (comment out WiFi.config if you prefer DHCP)
IPAddress local_IP(192, 168, 1, 50);
IPAddress dns(8, 8, 8, 8);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

// -------------------- Calibration (EEPROM) --------------------
struct Calibration {
  float phOffset;
  float phScale;
  float tdsOffset;
  float tdsScale;
} calib;

const int EEPROM_ADDR = 0;

// -------------------- Settings / Profiles --------------------
struct Settings {
  const char* name;
  float targetPH;
  int   targetTDS;
  float phTolerance;
  int   tdsTolerance;
  int   tdsMaxAttempts;
  int   phMaxAttempts;
  unsigned long mixTime;
  unsigned long stabilizeTime;
  unsigned long doseUnitTime;
  unsigned long maxDosePerAttempt;
};

const Settings profiles[] = {
  // Leafy greens (lettuce, spinach, kale, basil, herbs)
  {
    "Leafy",
    5.9,    // target pH
    700,    // target TDS (ppm)
    0.4,    // pH tolerance
    140,    // TDS tolerance
    5,      // max pH attempts
    5,      // max TDS attempts
    5000UL, // mixing time (ms)
    5000UL, // stabilization time (ms)
    100UL,  // dose unit time (ms)
    3000UL  // max dose per attempt (ms)
  }, 

  // Fruiting crops (tomato, pepper, cucumber, beans, strawberry, etc.)
  {
    "Fruiting",
    6.1,    // target pH
    1900,   // target TDS (ppm)
    0.3,    // pH tolerance
    525,    // TDS tolerance
    5,      // max pH attempts
    5,      // max TDS attempts
    6000UL, // mixing time (ms)
    6000UL, // stabilization time (ms)
    120UL,  // dose unit time (ms)
    3500UL  // max dose per attempt (ms)
  },

  // Root crops (carrot, radish, beet, turnip, etc.)
  {
    "Root",
    6.3,    // target pH
    1050,   // target TDS (ppm)
    0.2,    // pH tolerance
    350,    // TDS tolerance
    5,      // max pH attempts
    5,      // max TDS attempts
    4000UL, // mixing time (ms)
    4000UL, // stabilization time (ms)
    100UL,  // dose unit time (ms)
    2500UL  // max dose per attempt (ms)
  }
};


const int PROFILE_COUNT = sizeof(profiles) / sizeof(profiles[0]);

Settings config;            // active config (set when profile chosen)
String currentProfile = ""; // profile name
bool profileSelected = false;

// -------------------- Globals --------------------
bool tdsMaxedOut = false;
bool phMaxedOut  = false;
String _lastLine1 = "";
String _lastLine2 = "";
String logBuffer = "";

// relay states cache (for UI text)
bool relayState_acid = false;
bool relayState_base = false;
bool relayState_nut  = false;
bool relayState_dump = false;

// -------------------- State Machine --------------------
enum SystemState { BOOT, IDLE, RUNNING, PAUSED, CANCELLED, DONE };
SystemState state = BOOT;

// -------------------- pH & TDS Meter --------------------
// TDS sensor
#define VREF 5.0              // analog reference voltage(Volt) of the ADC
#define SCOUNT  30            // number of samples for TDS
int analogBuffer[SCOUNT];     // store raw ADC values
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;

// pH sensor
#define PH_ARRAY_LEN 40
int pHArray[PH_ARRAY_LEN];
int pHArrayIndex = 0;

// sampling timers and values
unsigned long lastTdsSample = 0;
unsigned long lastPhSample = 0;

float rawPHValue = 0.0;   // raw computed pH (before calibration)
float rawTDSValue = 0.0;  // raw computed TDS (before calibration)
float latestPH = 0.0;     // calibrated pH shown to system/UI
float latestTDS = 0.0;    // calibrated TDS shown to system/UI
float waterTemperature = 25.0; // default for TDS compensation (user can modify in code)

// median filtering
int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++) bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
  else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  return bTemp;
}

double avergearray(int* arr, int number) {
  int i;
  int max, min;
  long amount = 0;
  if (number <= 0) return 0;

  if (number < 5) {
    for (i = 0; i < number; i++) amount += arr[i];
    return amount / number;
  } else {
    if (arr[0] < arr[1]) { min = arr[0]; max = arr[1]; }
    else { min = arr[1]; max = arr[0]; }
    for (i = 2; i < number; i++) {
      if (arr[i] < min) { amount += min; min = arr[i]; }
      else if (arr[i] > max) { amount += max; max = arr[i]; }
      else amount += arr[i];
    }
    return (double)amount / (number - 2);
  }
}

// sensor update (non-blocking) - call frequently from loop and waitWithClients
void updateSensors() {
  unsigned long now = millis();
  // TDS sample every 40 ms
  if (now - lastTdsSample >= 40) {
    lastTdsSample = now;
    analogBuffer[analogBufferIndex] = analogRead(TDS_PIN);
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT) analogBufferIndex = 0;

    // compute median voltage and TDS using algorithm from example
    for (int i = 0; i < SCOUNT; i++) analogBufferTemp[i] = analogBuffer[i];
    float averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 1024.0;
    float compensationCoefficient = 1.0 + 0.02 * (waterTemperature - 25.0);
    float compensationVoltage = averageVoltage / compensationCoefficient;
    float tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage
                      - 255.86 * compensationVoltage * compensationVoltage
                      + 857.39 * compensationVoltage) * 0.5;
    rawTDSValue = tdsValue; // store raw (uncalibrated)
    latestTDS = rawTDSValue * calib.tdsScale + calib.tdsOffset;
  }

  // pH sample every 20 ms
  if (now - lastPhSample >= 20) {
    lastPhSample = now;
    pHArray[pHArrayIndex++] = analogRead(PH_PIN);
    if (pHArrayIndex >= PH_ARRAY_LEN) pHArrayIndex = 0;
    float phVoltage = avergearray(pHArray, PH_ARRAY_LEN) * 5.0 / 1024.0;
    float pHValue = 3.5 * phVoltage + 0.00; // Offset handled via calibration
    rawPHValue = pHValue;
    latestPH = rawPHValue * calib.phScale + calib.phOffset;
  }
}

// -------------------- Helpers --------------------
void relayOn(int pin)  { digitalWrite(pin, LOW); }
void relayOff(int pin) { digitalWrite(pin, HIGH); }

void updateRelayStatesCache() {
  relayState_acid = (digitalRead(RELAY_ACID) == LOW);
  relayState_base = (digitalRead(RELAY_BASE) == LOW);
  relayState_nut  = (digitalRead(RELAY_NUTRIENT) == LOW);
  relayState_dump = (digitalRead(RELAY_DUMP) == LOW);
}

void buzzerAlert() { tone(BUZZER_PIN, 1000, 200); }

void addLog(const String &msg) {
  logBuffer += msg + "\n";
  if (logBuffer.length() > 12000) logBuffer = logBuffer.substring(logBuffer.length() - 10000);
  Serial.println(msg);
}


// -------------------- LCD Message Helper --------------------
void showMsg(const String &line1, const String &line2, unsigned long ms) {
  // Only update if message changed (to reduce flicker)
  if (line1 != _lastLine1 || line2 != _lastLine2) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(line1.substring(0, 16)); // fit to 16 chars
    lcd.setCursor(0, 1);
    lcd.print(line2.substring(0, 16));
    _lastLine1 = line1;
    _lastLine2 = line2;
  }

  // optional wait with clients (so WiFi dashboard still works)
  if (ms > 0) waitWithClients(ms);
}


// -------------------- wait helper --------------------
void serveClient(WiFiClient &client); // forward decl

void waitWithClients(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    updateSensors();
    WiFiClient c = server.available();
    if (c) serveClient(c);
    delay(40);
    if (state == CANCELLED) return;
    if (state == PAUSED) {
      // while paused keep serving clients and updating sensors
      while (state == PAUSED) {
        updateSensors();
        WiFiClient c2 = server.available();
        if (c2) serveClient(c2);
        delay(80);
        if (state == CANCELLED) return;
      }
      start = millis(); // restart the wait after resume
    }
  }
}

// -------------------- Calibration EEPROM --------------------
void loadCalibration() {
  EEPROM.get(EEPROM_ADDR, calib);
  // basic sanity check - if data looks garbage, reset to defaults
  if (isnan(calib.phOffset) || isnan(calib.phScale) || calib.phScale == 0.0 || calib.tdsScale == 0.0) {
    calib.phOffset = 0.0; calib.phScale = 1.0;
    calib.tdsOffset = 0.0; calib.tdsScale = 1.0;
  }
  addLog("CAL: Calibration loaded");
}

void saveCalibration() {
  EEPROM.put(EEPROM_ADDR, calib);
  addLog("CAL: Calibration saved");
}

void resetCalibration() {
  calib.phOffset = 0.0; calib.phScale = 1.0;
  calib.tdsOffset = 0.0; calib.tdsScale = 1.0;
  saveCalibration();
  addLog("CAL: Reset to defaults");
}

// -------------------- Tank helpers --------------------
long readLevelOnce(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return 30;
  return (long)(duration * 0.034 / 2.0);
}

int getContainerPercent(int trigPin, int echoPin) {
  const int samples = 5;
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += readLevelOnce(trigPin, echoPin);
    delay(15);
  }
  long avg = sum / samples;
  int percent = map((int)avg, 2, 30, 100, 0);
  return constrain(percent, 0, 100);
}

void checkTank(const char* name, int trigPin, int echoPin) {
  int percent = getContainerPercent(trigPin, echoPin);
  while (percent < 80) {
    if (state == CANCELLED) return;
    if (state == PAUSED) { waitWithClients(1000); continue; }
    showMsg(String(name) + ": " + percent + "%", "Refill to >=80%", 1500);
    buzzerAlert();
    addLog("SYS: " + String(name) + " tank low (" + String(percent) + "%) — waiting for refill");
    percent = getContainerPercent(trigPin, echoPin);
  }
  showMsg(String(name) + ": " + percent + "%", "OK", 1000);
  addLog("SYS: " + String(name) + " tank OK (" + String(percent) + "%)");
}

void waitForRefill(const char* name, int trigPin, int echoPin, int minPercent) {
  int percent = getContainerPercent(trigPin, echoPin);
  while (percent < minPercent) {
    if (state == CANCELLED) return;
    if (state == PAUSED) { waitWithClients(1000); continue; }
    showMsg(String(name) + ": " + percent + "%", "Refill Needed!", 1500);
    buzzerAlert();
    addLog("SYS: " + String(name) + " tank low (" + String(percent) + "%) — needs >= " + String(minPercent) + "%");
    percent = getContainerPercent(trigPin, echoPin);
  }
  showMsg(String(name) + ": " + percent + "%", "OK", 1000);
  addLog("SYS: " + String(name) + " tank OK (" + String(percent) + "%)");
}

// -------------------- Mixing / Stabilizing / Correction --------------------
void mixingCountdown(unsigned long duration) {
  addLog("SYS: Mixing started (" + String(duration/1000) + "s)");

  unsigned long start = millis();
  while (millis() - start < duration) {
    if (state == CANCELLED) return;
    if (state == PAUSED) { waitWithClients(1000); continue; }

    unsigned long remaining = (duration - (millis() - start)) / 1000;
    showMsg("Mixing...", "Time: " + String(remaining) + "s", 1000);

    // log once per second
    addLog("SYS: Mixing... remaining " + String(remaining) + "s, pH=" + String(latestPH,2) + ", TDS=" + String(latestTDS,0));
  }

  addLog("SYS: Mixing complete");
}

void stabilizationWait(unsigned long duration) {
  addLog("SYS: Stabilization started (" + String(duration/1000) + "s)");

  unsigned long start = millis();
  while (millis() - start < duration) {
    if (state == CANCELLED) return;
    if (state == PAUSED) { waitWithClients(1000); continue; }

    unsigned long remaining = (duration - (millis() - start)) / 1000;
    showMsg("Stabilizing...", "Time: " + String(remaining) + "s", 1000);

    // log once per second
    addLog("SYS: Stabilizing... remaining " + String(remaining) + "s, pH=" + String(latestPH,2) + ", TDS=" + String(latestTDS,0));
  }

  addLog("SYS: Stabilization complete");
}

int correctTDS(int initialTDS) {
  tdsMaxedOut = false;
  int tds = initialTDS;

  addLog("SYS: Starting TDS correction, initial TDS=" + String(initialTDS));

  // already within tolerance
  if (abs(tds - config.targetTDS) <= config.tdsTolerance) {
    showMsg("TDS Result:" + String(tds), "Stable", 2000);
    addLog("SYS: TDS already stable at " + String(tds));
    return tds;
  }

  int attempt = 0;
  while (attempt < config.tdsMaxAttempts) {
    if (state == CANCELLED) return tds;
    if (state == PAUSED) { waitWithClients(1000); continue; }

    attempt++;
    showMsg("TDS Attempt " + String(attempt) + "/" + String(config.tdsMaxAttempts),
            "Target:" + String(config.targetTDS), 1500);
    addLog("SYS: TDS correction attempt " + String(attempt) + " of " + String(config.tdsMaxAttempts));

    if (tds < config.targetTDS - config.tdsTolerance) {
      waitForRefill("Nutrient", TRIG_NUT, ECHO_NUT, 20);
      if (state == CANCELLED) return tds;
      showMsg("Correcting TDS", "Adding Nutrients", 2000);
      addLog("SYS: Adding Nutrients to increase TDS");

      unsigned long doseTime = ((unsigned long)(config.targetTDS - tds) / 10UL) * config.doseUnitTime;
      if (doseTime == 0) doseTime = config.doseUnitTime;
      if (doseTime > config.maxDosePerAttempt) doseTime = config.maxDosePerAttempt;

      relayOn(RELAY_NUTRIENT);
      waitWithClients(doseTime);
      relayOff(RELAY_NUTRIENT);

      showMsg("Dosing Done", "", 1000);
      addLog("SYS: Nutrient dosing complete (" + String(doseTime) + " ms)");
    } 
    else if (tds > config.targetTDS + config.tdsTolerance) {
      showMsg("Correcting TDS", "Diluting...", 2000);
      addLog("SYS: Diluting solution to decrease TDS");

      unsigned long dumpTime = ((unsigned long)(tds - config.targetTDS) / 10UL) * config.doseUnitTime;
      if (dumpTime == 0) dumpTime = config.doseUnitTime;
      if (dumpTime > config.maxDosePerAttempt) dumpTime = config.maxDosePerAttempt;

      relayOn(RELAY_DUMP);
      waitWithClients(dumpTime);
      relayOff(RELAY_DUMP);

      showMsg("Dilution Done", "", 2000);
      addLog("SYS: Dilution complete (" + String(dumpTime) + " ms)");

      showMsg("Add Water", "Refill Reservoir", 2000);
      addLog("SYS: Waiting for reservoir refill");
      waitForRefill("Reservoir", TRIG_RES, ECHO_RES, 80);
    }

    // wait for mixing + stabilization
    mixingCountdown(config.mixTime);
    stabilizationWait(config.stabilizeTime);

    showMsg("Rechecking TDS", "", 2000);
    tds = readTDS();
    addLog("SYS: Rechecked TDS = " + String(tds));

    if (abs(tds - config.targetTDS) <= config.tdsTolerance) {
      showMsg("TDS Result:" + String(tds), "Stable", 2000);
      addLog("SYS: TDS stabilized at " + String(tds));
      return tds;
    } else {
      showMsg("TDS Result:" + String(tds), "Unstable", 2000);
      addLog("SYS: TDS still unstable (" + String(tds) + ")");
    }
  }

  // max attempts reached
  tdsMaxedOut = true;
  showMsg("TDS Result:" + String(tds), "Unstable", 2000);
  addLog("SYS: ALERT — TDS max attempts reached, last TDS=" + String(tds));
  return tds;
}


float correctPH(float initialPH) {
  phMaxedOut = false;
  float ph = initialPH;

  addLog("SYS: Starting pH correction, initial pH=" + String(initialPH, 2));

  // already within tolerance
  if (fabs(ph - config.targetPH) <= config.phTolerance) {
    showMsg("pH Result:" + String(ph, 2), "Stable", 2000);
    addLog("SYS: pH already stable at " + String(ph, 2));
    return ph;
  }

  int attempt = 0;
  while (attempt < config.phMaxAttempts) {
    if (state == CANCELLED) return ph;
    if (state == PAUSED) { waitWithClients(1000); continue; }

    attempt++;
    showMsg("pH Attempt " + String(attempt) + "/" + String(config.phMaxAttempts),
            "Target:" + String(config.targetPH, 2), 1500);
    addLog("SYS: pH correction attempt " + String(attempt) + " of " + String(config.phMaxAttempts));

    if (ph < config.targetPH - config.phTolerance) {
      waitForRefill("Base", TRIG_BASE, ECHO_BASE, 20);
      if (state == CANCELLED) return ph;
      showMsg("Correcting pH", "Adding Base", 2000);
      addLog("SYS: Adding Base to increase pH");

      unsigned long doseTime = (unsigned long)((config.targetPH - ph) / 0.1f * config.doseUnitTime);
      if (doseTime == 0) doseTime = config.doseUnitTime;
      if (doseTime > config.maxDosePerAttempt) doseTime = config.maxDosePerAttempt;

      relayOn(RELAY_BASE);
      waitWithClients(doseTime);
      relayOff(RELAY_BASE);

      showMsg("Dosing Done", "", 1000);
      addLog("SYS: Base dosing complete (" + String(doseTime) + " ms)");
    } 
    else if (ph > config.targetPH + config.phTolerance) {
      waitForRefill("Acid", TRIG_ACID, ECHO_ACID, 20);
      if (state == CANCELLED) return ph;
      showMsg("Correcting pH", "Adding Acid", 2000);
      addLog("SYS: Adding Acid to decrease pH");

      unsigned long doseTime = (unsigned long)((ph - config.targetPH) / 0.1f * config.doseUnitTime);
      if (doseTime == 0) doseTime = config.doseUnitTime;
      if (doseTime > config.maxDosePerAttempt) doseTime = config.maxDosePerAttempt;

      relayOn(RELAY_ACID);
      waitWithClients(doseTime);
      relayOff(RELAY_ACID);

      showMsg("Dosing Done", "", 1000);
      addLog("SYS: Acid dosing complete (" + String(doseTime) + " ms)");
    }

    // wait for mixing + stabilization
    mixingCountdown(config.mixTime);
    stabilizationWait(config.stabilizeTime);

    showMsg("Rechecking pH", "", 2000);
    ph = readPH();
    addLog("SYS: Rechecked pH = " + String(ph, 2));

    if (fabs(ph - config.targetPH) <= config.phTolerance) {
      showMsg("pH Result:" + String(ph, 2), "Stable", 2000);
      addLog("SYS: pH stabilized at " + String(ph, 2));
      return ph;
    } else {
      showMsg("pH Result:" + String(ph, 2), "Unstable", 2000);
      addLog("SYS: pH still unstable (" + String(ph, 2) + ")");
    }
  }

  // max attempts reached
  phMaxedOut = true;
  showMsg("pH Result:" + String(ph, 2), "Unstable", 2000);
  addLog("SYS: ALERT — pH max attempts reached, last pH=" + String(ph, 2));
  return ph;
}


// -------------------- Final Dump --------------------
void finalDump() {
  addLog("SYS: Final dump starting...");

  for (int i = 5; i > 0; i--) {
    if (state == CANCELLED) return;
    String msg = "Dumping in " + String(i) + "s";
    showMsg(msg, "Prepare...", 1000);
    buzzerAlert();

    // log countdown
    addLog("SYS: " + msg);
  }

  showMsg("Final Dumping...", "", 2000);
  relayOn(RELAY_DUMP);
  addLog("SYS: Dumping now...");

  while (getContainerPercent(TRIG_RES, ECHO_RES) > 0) {
    if (state == CANCELLED) break;
    if (state == PAUSED) { waitWithClients(1000); continue; }
    waitWithClients(1000);
  }

  relayOff(RELAY_DUMP);
  showMsg("Dumping Done", "", 2000);
  addLog("SYS: Final dump completed");
}

// -------------------- Profile Management (web only) --------------------
void setProfile(const String &profile) {
  for (int i = 0; i < PROFILE_COUNT; i++) {
    if (profile == profiles[i].name) {
      config = profiles[i];
      currentProfile = profiles[i].name;
      profileSelected = true;
      addLog("SYS: Profile selected: " + profile);
      showMsg("Target Set:", profile, 1500);
      buzzerAlert();
      // auto-start when user clicks a profile button (per request)
      state = RUNNING;
      addLog("SYS: Auto-starting " + profile);
      return;
    }
  }
  addLog("SYS: Profile not found: " + profile);
}

void cancelCycle() {
  if (state == RUNNING || state == PAUSED) {
    state = CANCELLED;
    addLog("SYS: Cancel requested");
    showMsg("Cancelling...", "", 1000);
    buzzerAlert(); delay(120); buzzerAlert();
    profileSelected = false;
    currentProfile = "";
  }
}

// -------------------- WiFi --------------------
void connectWiFi() {
  addLog("SYS: Connecting WiFi...");
  showMsg("Connecting WiFi...", String(ssid), 2000);
  WiFi.config(local_IP, dns, gateway, subnet);
  WiFi.begin(ssid, pass);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start < 15000)) {
    updateSensors();
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    IPAddress ip = WiFi.localIP();
    showMsg("WiFi OK", ip.toString(), 2000);
    addLog("SYS: WiFi Connected IP: " + ip.toString());
    buzzerAlert(); delay(150); buzzerAlert();
    server.begin();
    addLog("SYS: Web server started");
  } else {
    showMsg("WiFi FAIL", "Retry Later", 2000);
    addLog("SYS: WiFi FAIL");
    server.begin(); // start server anyway
  }
}

// -------------------- Server / Web UI --------------------
String urlDecode(const String &src) {
  String r = "";
  char c;
  for (int i = 0; i < src.length(); i++) {
    c = src[i];
    if (c == '+') r += ' ';
    else if (c == '%') {
      if (i + 2 < src.length()) {
        String hex = src.substring(i+1, i+3);
        char val = (char) strtol(hex.c_str(), NULL, 16);
        r += val;
        i += 2;
      }
    } else r += c;
  }
  return r;
}

void serveClient(WiFiClient &client) {
  String req = client.readStringUntil('\r');
  client.flush();

  // /status endpoint
  if (req.indexOf("GET /status") >= 0) {
    updateSensors(); updateRelayStatesCache();
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: application/json");
    client.println("Connection: close");
    client.println();
    client.print("{");
    client.print("\"ph\":"); client.print(latestPH, 2); client.print(",");
    client.print("\"tds\":"); client.print(latestTDS, 0); client.print(",");
    client.print("\"res\":"); client.print(getContainerPercent(TRIG_RES,ECHO_RES)); client.print(",");
    client.print("\"acid\":"); client.print(getContainerPercent(TRIG_ACID,ECHO_ACID)); client.print(",");
    client.print("\"base\":"); client.print(getContainerPercent(TRIG_BASE,ECHO_BASE)); client.print(",");
    client.print("\"nut\":"); client.print(getContainerPercent(TRIG_NUT,ECHO_NUT)); client.print(",");
    client.print("\"relay_acid\":\""); client.print(relayState_acid?"ON":"OFF"); client.print("\",");
    client.print("\"relay_base\":\""); client.print(relayState_base?"ON":"OFF"); client.print("\",");
    client.print("\"relay_nut\":\""); client.print(relayState_nut?"ON":"OFF"); client.print("\",");
    client.print("\"relay_dump\":\""); client.print(relayState_dump?"ON":"OFF"); client.print("\",");
    client.print("\"state\":\"");
    switch (state) {
      case BOOT: client.print("BOOT"); break;
      case IDLE: client.print("IDLE"); break;
      case RUNNING: client.print("RUNNING"); break;
      case PAUSED: client.print("PAUSED"); break;
      case CANCELLED: client.print("CANCELLED"); break;
      case DONE: client.print("DONE"); break;
    }
    client.print("\",");
    client.print("\"profile\":\""); client.print(currentProfile); client.print("\",");
    client.print("\"targets\":{\"ph\":"); client.print(config.targetPH,2); client.print(",\"tds\":"); client.print(config.targetTDS);
    client.print("},");
    client.print("\"cal\":{\"phOffset\":"); client.print(calib.phOffset,6); client.print(",\"phScale\":"); client.print(calib.phScale,6);
    client.print(",\"tdsOffset\":"); client.print(calib.tdsOffset,6); client.print(",\"tdsScale\":"); client.print(calib.tdsScale,6);
    client.print("}");
    client.print("}");
  }
  // logs
  else if (req.indexOf("GET /logs") >= 0) {
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/plain");
    client.println("Connection: close");
    client.println();
    client.print(logBuffer);
  }
  // clear logs
  else if (req.indexOf("GET /clearLogs") >= 0) {
    logBuffer = "";
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/plain");
    client.println("Connection: close");
    client.println();
    client.print("Logs cleared");
    addLog("SYS: Logs cleared via web");
  }
  // setProfile
  else if (req.indexOf("GET /setProfile") >= 0) {
    int p = req.indexOf("profile=");
    String prof = "";
    if (p >= 0) {
      String tail = req.substring(p + 8);
      int spaceIdx = tail.indexOf(' ');
      if (spaceIdx > 0) tail = tail.substring(0, spaceIdx);
      prof = urlDecode(tail);
    }
    if (prof.length()) {
      setProfile(prof);
    } else {
      addLog("WEB: setProfile missing profile param");
    }
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/plain");
    client.println("Connection: close");
    client.println();
    client.print("Profile set: " + prof);
  }
  // control (pause/cancel)
  else if (req.indexOf("GET /control?action=") >= 0) {
    int idx = req.indexOf("GET /control?action=");
    String tail = req.substring(idx + 20);
    int spaceIdx = tail.indexOf(' ');
    if (spaceIdx > 0) tail = tail.substring(0, spaceIdx);
    String action = urlDecode(tail);
    if (action == "pause") {
      if (state == RUNNING) state = PAUSED;
      else if (state == PAUSED) state = RUNNING;
      client.println("HTTP/1.1 200 OK"); client.println("Content-Type: text/plain"); client.println("Connection: close"); client.println();
      client.print("Pause/Resume toggled");
      addLog("WEB: Pause/Resume toggled");
    } else if (action == "cancel") {
      cancelCycle();
      client.println("HTTP/1.1 200 OK"); client.println("Content-Type: text/plain"); client.println("Connection: close"); client.println();
      client.print("Cancel requested");
    } else {
      client.println("HTTP/1.1 400 Bad Request"); client.println("Content-Type: text/plain"); client.println("Connection: close"); client.println();
      client.print("Unknown action");
    }
  }
  // relay control: /relay?which=acid|base|nut|dump&state=on|off
  else if (req.indexOf("GET /relay") >= 0) {
    int p = req.indexOf("which=");
    String which = "";
    String stateParam = "";
    if (p >= 0) {
      String tail = req.substring(p + 6);
      int amp = tail.indexOf('&');
      if (amp > 0) {
        which = tail.substring(0, amp);
        String rest = tail.substring(amp + 1);
        int spaceIdx = rest.indexOf(' ');
        if (spaceIdx > 0) rest = rest.substring(0, spaceIdx);
        int sp = rest.indexOf("state=");
        if (sp >= 0) stateParam = rest.substring(sp + 6);
      } else {
        int spaceIdx = tail.indexOf(' ');
        if (spaceIdx > 0) tail = tail.substring(0, spaceIdx);
        which = tail;
      }
    }
    which = urlDecode(which);
    stateParam = urlDecode(stateParam);
    bool ok = false;
    String resp = "Invalid";
    if (which.length() && stateParam.length()) {
      String s = stateParam;
      s.toLowerCase();
      bool turnOn = (s == "on");
      if (which == "acid") {
        if (turnOn) { relayOn(RELAY_ACID); relayState_acid = true; } else { relayOff(RELAY_ACID); relayState_acid = false; }
        ok = true;
      } else if (which == "base") {
        if (turnOn) { relayOn(RELAY_BASE); relayState_base = true; } else { relayOff(RELAY_BASE); relayState_base = false; }
        ok = true;
      } else if (which == "nut") {
        if (turnOn) { relayOn(RELAY_NUTRIENT); relayState_nut = true; } else { relayOff(RELAY_NUTRIENT); relayState_nut = false; }
        ok = true;
      } else if (which == "dump") {
        if (turnOn) { relayOn(RELAY_DUMP); relayState_dump = true; } else { relayOff(RELAY_DUMP); relayState_dump = false; }
        ok = true;
      }
    }
    client.println("HTTP/1.1 200 OK"); client.println("Content-Type: text/plain"); client.println("Connection: close"); client.println();
    if (ok) {
      resp = "Relay " + which + " set to " + stateParam;
      client.print(resp);
      addLog("WEB: " + resp);
    } else {
      client.print("Invalid relay command");
    }
  }
  // calibrate endpoint: /calibrate?ph=7.00 or /calibrate?tds=1413 or /calibrate?reset=1
  else if (req.indexOf("GET /calibrate") >= 0) {
    int qIdx = req.indexOf('?');
    String responseText = "OK";
    if (qIdx >= 0) {
      String qs = req.substring(qIdx + 1);
      int spaceIdx = qs.indexOf(' ');
      if (spaceIdx > 0) qs = qs.substring(0, spaceIdx);
      if (qs.indexOf("ph=") >= 0) {
        int p = qs.indexOf("ph=");
        String val = qs.substring(p + 3);
        float phKnown = val.toFloat();
        // use rawPHValue (uncalibrated) to compute offset
        float measured = rawPHValue;
        calib.phOffset = phKnown - (measured * calib.phScale);
        saveCalibration();
        responseText = "pH calibrated: known=" + String(phKnown,2) + " measured=" + String(measured,2) + " offset=" + String(calib.phOffset,6);
        buzzerAlert();
      } else if (qs.indexOf("tds=") >= 0) {
        int p = qs.indexOf("tds=");
        String val = qs.substring(p + 4);
        float tdsKnown = val.toFloat();
        float measured = rawTDSValue;
        if (measured != 0) calib.tdsScale = tdsKnown / measured;
        saveCalibration();
        responseText = "TDS calibrated: known=" + String(tdsKnown,1) + " measured=" + String(measured,1) + " scale=" + String(calib.tdsScale,6);
        buzzerAlert();
      } else if (qs.indexOf("reset=1") >= 0) {
        resetCalibration();
        responseText = "Calibration reset";
        buzzerAlert();
      } else {
        responseText = "No valid param";
      }
    } else {
      responseText = "No params";
    }
    client.println("HTTP/1.1 200 OK"); client.println("Content-Type: text/plain"); client.println("Connection: close"); client.println();
    client.print(responseText);
  }
  // getCalibration
  else if (req.indexOf("GET /getCalibration") >= 0) {
    client.println("HTTP/1.1 200 OK"); client.println("Content-Type: application/json"); client.println("Connection: close"); client.println();
    client.print("{");
    client.print("\"phOffset\":"); client.print(calib.phOffset,6); client.print(",");
    client.print("\"phScale\":"); client.print(calib.phScale,6); client.print(",");
    client.print("\"tdsOffset\":"); client.print(calib.tdsOffset,6); client.print(",");
    client.print("\"tdsScale\":"); client.print(calib.tdsScale,6);
    client.print("}");
  }
// root: dashboard HTML
else {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println("Connection: close");
  client.println();
  client.println("<!doctype html><html><head><meta charset='utf-8'><title>Hydroponics Dashboard</title>");
  client.println("<meta name='viewport' content='width=device-width,initial-scale=1'/>");
  client.println("<style>");
  client.println("body{font-family:\"Segoe UI\",Roboto,Arial,sans-serif;background:#f5f7fa;margin:0;padding:20px;color:#222}");
  client.println("h2{margin:0 0 18px 0;font-weight:600;color:#111}");
  client.println(".grid{display:grid;grid-template-columns:1fr 340px;gap:20px;align-items:start}");
  client.println(".card{background:#fff;padding:16px;border-radius:10px;box-shadow:0 2px 6px rgba(0,0,0,0.05)}");
  client.println(".full{grid-column:1/-1}");
  client.println(".section-title{font-weight:600;font-size:13px;margin:0 0 10px 0;color:#444;text-transform:uppercase;letter-spacing:0.5px}");
  client.println(".section-block{padding:10px 12px;border-radius:8px;background:#fafafa;margin-bottom:12px;font-size:14px;line-height:1.6}");
  client.println(".button-row{display:flex;flex-wrap:wrap;gap:8px}");
  client.println(".button-row button{flex:1;min-width:0;padding:8px;border-radius:8px;border:1px solid #ccc;background:#fff;cursor:pointer}");
  client.println(".button-row button:hover{background:#f0f2f5}");
  client.println(".relay-toggle{cursor:pointer;text-decoration:underline;color:#0077cc;margin-right:10px}");
  client.println(".relay-toggle:hover{color:#004f88}");
  client.println(".input-group{margin-top:10px}");
  client.println(".input-group label{display:block;font-size:13px;margin-bottom:4px;color:#333}");
  client.println(".input-group input{width:120px;padding:6px;border:1px solid #bbb;border-radius:6px;font-size:13px;margin-right:6px}");
  client.println("#logs{height:220px;overflow:auto;background:#111;color:#0f0;padding:12px;font-family:monospace;font-size:13px;border-radius:8px}");
  client.println("</style></head><body>");
  client.println("<h2>Hydroponics Dashboard</h2>");
  client.println("<div class='grid'>");

  // left column
  client.println("<div class='card'>");
  client.println("<div class='section-title'>System Status</div>");
  client.println("<div class='section-block' id='statusTop'>Loading...</div>");

  client.println("<div class='section-title'>Readings</div>");
  client.println("<div class='section-block' id='readings'>Loading...</div>");

  client.println("<div class='section-title'>Calibration</div>");
  client.println("<div class='section-block'><span class='small'>");
  client.println("pH Offset: <span id='phOff'>...</span><br>");
  client.println("pH Scale: <span id='phScale'>...</span><br>");
  client.println("TDS Offset: <span id='tdsOff'>...</span><br>");
  client.println("TDS Scale: <span id='tdsScale'>...</span>");
  client.println("</span></div>");
  client.println("</div>");

  // right column
  client.println("<div class='card'>");
  client.println("<div class='section-title'>Profiles</div>");
  client.println("<div class='button-row'>");
  client.println("<button class='profileBtn' onclick=\"setAndStart('Leafy')\">Leafy</button>");
  client.println("<button class='profileBtn' onclick=\"setAndStart('Fruiting')\">Fruiting</button>");
  client.println("<button class='profileBtn' onclick=\"setAndStart('Root')\">Root</button>");
  client.println("</div>");

  client.println("<div class='section-title'>Controls</div>");
  client.println("<div class='button-row'><button onclick=\"control('pause')\" id='pauseBtn'>Pause</button><button onclick=\"control('cancel')\">Cancel</button></div>");

  client.println("<div class='section-title'>Calibration Actions</div>");
  client.println("<div class='input-group'><label>pH calibration (known pH)</label><input id='calPH' type='number' step='0.01' placeholder='7.00'/><button onclick='calibratePH()'>Calibrate pH</button></div>");
  client.println("<div class='input-group'><label>TDS calibration (known ppm)</label><input id='calTDS' type='number' step='1' placeholder='1413'/><button onclick='calibrateTDS()'>Calibrate TDS</button></div>");
  client.println("<div class='input-group'><button onclick='resetCal()'>Reset Calibration</button></div>");
  client.println("</div>");

  // logs full width
  client.println("<div class='card full'><div style='display:flex;justify-content:space-between;align-items:center;margin-bottom:12px'><div class='section-title' style='margin:0'>Logs</div><div><button onclick='clearLogs()'>Clear logs</button></div></div><pre id='logs'></pre></div>");

  client.println("</div>"); // grid end

  // JS
  client.println("<script>");
  client.println("function fetchText(u){return fetch(u).then(r=>r.text());}");
  client.println("function fetchJSON(u){return fetch(u).then(r=>r.json());}");
  client.println("function update(){fetchJSON('/status').then(d=>{");

  // --- Status line with ranges ---
 client.println("let phTol=(d.profile=='Leafy'?0.4:(d.profile=='Fruiting'?0.3:0.2));");
client.println("let tdsTol=(d.profile=='Leafy'?140:(d.profile=='Fruiting'?525:350));");

client.println("let phLow=(d.targets.ph-phTol).toFixed(1);");
client.println("let phHigh=(d.targets.ph+phTol).toFixed(1);");
client.println("let tdsLow=d.targets.tds-tdsTol;");
client.println("let tdsHigh=d.targets.tds+tdsTol;");

client.println("document.getElementById('statusTop').innerHTML='<b>Profile:</b> '+d.profile+' | <b>State:</b> '+d.state+' | <b>Target pH:</b> '+d.targets.ph+' ('+phLow+'–'+phHigh+') | <b>Target TDS:</b> '+d.targets.tds+' ppm ('+tdsLow+'–'+tdsHigh+')';");


  // --- Readings ---
  client.println("document.getElementById('readings').innerHTML='<b>pH:</b> '+d.ph+' | <b>TDS:</b> '+d.tds+' ppm<br><b>Containers:</b> Reservoir: '+d.res+'% | Acid: '+d.acid+'% | Base: '+d.base+'% | Nutrient: '+d.nut+'%<br><b>Pump Control:</b> <span class=relay-toggle onclick=\"toggleRelay(\\'acid\\')\">Acid:'+d.relay_acid+'</span> | <span class=relay-toggle onclick=\"toggleRelay(\\'base\\')\">Base:'+d.relay_base+'</span> | <span class=relay-toggle onclick=\"toggleRelay(\\'nut\\')\">Nut:'+d.relay_nut+'</span> | <span class=relay-toggle onclick=\"toggleRelay(\\'dump\\')\">Dump:'+d.relay_dump+'</span>';");

  client.println("if(d.state=='PAUSED'){document.getElementById('pauseBtn').innerText='Resume';} else {document.getElementById('pauseBtn').innerText='Pause';}");
  client.println("if(d.state=='RUNNING'||d.state=='PAUSED'){document.querySelectorAll('.profileBtn').forEach(b=>b.disabled=true);} else {document.querySelectorAll('.profileBtn').forEach(b=>b.disabled=false);}");
  client.println("if(d.cal){document.getElementById('phOff').innerText=Number(d.cal.phOffset).toFixed(4);document.getElementById('phScale').innerText=Number(d.cal.phScale).toFixed(4);document.getElementById('tdsOff').innerText=Number(d.cal.tdsOffset).toFixed(4);document.getElementById('tdsScale').innerText=Number(d.cal.tdsScale).toFixed(4);}");
  client.println("}).catch(e=>{});}");
  client.println("function updateLogs(){fetchText('/logs').then(t=>{let l=document.getElementById('logs');l.textContent=t;l.scrollTop=l.scrollHeight;}).catch(e=>{});}");
  client.println("function setAndStart(p){fetch('/setProfile?profile='+encodeURIComponent(p)).then(r=>r.text()).then(t=>{update();updateLogs();});}");
  client.println("function control(a){fetch('/control?action='+a).then(r=>r.text()).then(t=>{update();updateLogs();});}");
  client.println("function calibratePH(){let v=document.getElementById('calPH').value;if(!v){alert('Enter known pH');return;}fetch('/calibrate?ph='+encodeURIComponent(v)).then(r=>r.text()).then(t=>{update();});}");
  client.println("function calibrateTDS(){let v=document.getElementById('calTDS').value;if(!v){alert('Enter known TDS');return;}fetch('/calibrate?tds='+encodeURIComponent(v)).then(r=>r.text()).then(t=>{update();});}");
  client.println("function resetCal(){if(!confirm('Reset calibration?'))return;fetch('/calibrate?reset=1').then(r=>r.text()).then(t=>{update();});}");
  client.println("function clearLogs(){fetch('/clearLogs').then(r=>r.text()).then(t=>{document.getElementById('logs').textContent='';});}");
  client.println("function toggleRelay(w){fetch('/status').then(r=>r.json()).then(d=>{let c='OFF';if(w=='acid')c=d.relay_acid;else if(w=='base')c=d.relay_base;else if(w=='nut')c=d.relay_nut;else if(w=='dump')c=d.relay_dump;let n=(c=='ON')?'off':'on';fetch('/relay?which='+w+'&state='+n).then(r=>r.text()).then(t=>{update();updateLogs();});});}");
  client.println("setInterval(update,2000);setInterval(updateLogs,2000);update();updateLogs();");
  client.println("</script></body></html>");
}


  delay(1);
  client.stop();
}

// -------------------- Cycle (main logic) --------------------
void runCycle() {
  if (!profileSelected) return;
  if (state != RUNNING) return;

  showMsg("Set Target:", currentProfile, 1000);
  
  buzzerAlert();

 addLog("SYS: Starting cycle with profile: " + currentProfile);

  // Display full target ranges on LCD/logs
  float phLow  = config.targetPH - config.phTolerance;
  float phHigh = config.targetPH + config.phTolerance;
  int tdsLow   = config.targetTDS - config.tdsTolerance;
  int tdsHigh  = config.targetTDS + config.tdsTolerance;

  showMsg("Target: " + currentProfile,
          "pH:" + String(config.targetPH,2) + " (" + String(phLow,1) + "-" + String(phHigh,1) + ")", 3000);
  addLog("SYS: Target pH: " + String(config.targetPH,2) + " range [" + String(phLow,1) + " - " + String(phHigh,1) + "]");
  addLog("SYS: Target TDS: " + String(config.targetTDS) + " ppm range [" + String(tdsLow) + " - " + String(tdsHigh) + "]");

  int tds = readTDS();
  float ph = readPH();

  // --- Step 1: Adjust pH first ---
  showMsg("Checking pH...", String(ph,2) + " vs " + String(config.targetPH,2), 2000);
  ph = correctPH(ph);
  if (state == CANCELLED) return;

  // --- Step 2: Adjust TDS second ---
  if (!phMaxedOut) {
    showMsg("Checking TDS...", String(tds) + " vs " + String(config.targetTDS), 2000);
    tds = correctTDS(tds);
  } else {
    addLog("pH unstable - skipping TDS");
  }
  if (state == CANCELLED) return;

  // --- Step 3: Finalize ---
  if (tdsMaxedOut || phMaxedOut) {
    showMsg("Releasing:", "Unstable", 3000);
    finalDump();
  } else {
    showMsg("Releasing:", "Stable", 3000);
    finalDump();
  }

  if (state != CANCELLED) {
    showMsg("Operation Done", "", 2000);
    addLog("SYS: cycle complete");
    buzzerAlert(); delay(200); buzzerAlert();
  } else {
    addLog("SYS: cycle cancelled");
  }

  profileSelected = false;
  currentProfile = "";
  state = IDLE;
}


// -------------------- Setup --------------------
void setup() {
  Serial.begin(115200);

  pinMode(RELAY_ACID, OUTPUT);    relayOff(RELAY_ACID);
  pinMode(RELAY_BASE, OUTPUT);    relayOff(RELAY_BASE);
  pinMode(RELAY_NUTRIENT, OUTPUT);relayOff(RELAY_NUTRIENT);
  pinMode(RELAY_DUMP, OUTPUT);    relayOff(RELAY_DUMP);
  pinMode(BUZZER_PIN, OUTPUT);
 

  pinMode(TRIG_RES, OUTPUT); pinMode(ECHO_RES, INPUT);
  pinMode(TRIG_ACID, OUTPUT); pinMode(ECHO_ACID, INPUT);
  pinMode(TRIG_BASE, OUTPUT); pinMode(ECHO_BASE, INPUT);
  pinMode(TRIG_NUT, OUTPUT); pinMode(ECHO_NUT, INPUT);

  lcd.init();
  lcd.setRGB(0, 255, 0);

  addLog("SYS: Initializing");
  buzzerAlert();
  showMsg("System Init.", "", 2000);

  loadCalibration();
  connectWiFi();

  // show tank statuses with readable timing
  showMsg("Checking Tanks...", "", 2000);
  checkTank("Reservoir", TRIG_RES, ECHO_RES);
  checkTank("Acid", TRIG_ACID, ECHO_ACID);
  checkTank("Base", TRIG_BASE, ECHO_BASE);
  checkTank("Nutrient", TRIG_NUT, ECHO_NUT);

  showMsg("System Ready", "", 0); // sticky
  addLog("SYS: Ready");

  state = IDLE;
}

// -------------------- Main Loop --------------------
void loop() {
  updateSensors();
  WiFiClient client = server.available();
  if (client) serveClient(client);

  switch (state) {
    case BOOT: break;
    case IDLE:
      if (_lastLine1 != "Set Target:") showMsg("Set Target:", "", 0);
      break;
    case RUNNING:
      runCycle();
      break;
    case PAUSED:
      waitWithClients(500);
      break;
    case CANCELLED:
      showMsg("Operation", "Cancelled", 2000);
      addLog("SYS: Cancelled");
      profileSelected = false;
      currentProfile = "";
      state = IDLE;
      break;
    case DONE:
      state = IDLE;
      break;
  }
}

// -------------------- Small wrappers used by cycle (non-blocking read) --------------------
float readPH() {
  // return last measured calibrated pH
  updateSensors();
  return latestPH;
}

int readTDS() {
  updateSensors();
  return (int)latestTDS;
}
// -------------------- End --------------------
