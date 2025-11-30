/* 
  Mega2560 + SIM800A + HW-61 (I2C PCF8574) + 4x4 Keypad + Multi-sensor -> ThingSpeak + 20x4 LCD
  - Auto-detect I2C LCD address (fallback 0x27)
  - SIM800 on Serial1 (RX1=19, TX1=18)
  - HW-61 I2C on Wire (SDA=20, SCL=21 on Mega)
  - Keypad connected to configurable digital pins
  - Sensors: soil moisture x2, pH, soil temp (OneWire), NPK, EC, fertility, DHT, MQ-135, MQ-2, LDR, wind/rain
  - Keys: 1..9,0 show sensors; A = averages, B = wind/rain, C = manual fast cycle, D = return auto
*/

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Make IRAM_ATTR no-op on AVR (e.g., Mega)
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

// ----------------- CONFIG -----------------
const String APN  = "internet";                 // <-- set your APN
const String USER = "";
const String PASS = "";

const String THING_SPEAK_API_URL  = "http://api.thingspeak.com/update";
const String THING_SPEAK_API_KEY  = "J5M8NQ8C6RMH4859"; // <-- replace with your Write API key

const bool USE_SSL = false;
const unsigned long HTTPACTION_TIMEOUT = 20000UL;
const unsigned long SAPBR_OPEN_TIMEOUT = 30000UL;
const unsigned long SEND_INTERVAL_MS = 20000UL; // >=15s for ThingSpeak

// ----------------- Sensor Pinout (Mega) -----------------
#define PIN_SOIL_MOISTURE_PRIMARY A0
#define PIN_SOIL_MOISTURE_SECONDARY A1
#define PIN_SOIL_PH A2
#define PIN_SOIL_NPK A3
#define PIN_SOIL_EC A4
#define PIN_SOIL_FERTILITY A5

#define PIN_SOIL_TEMP_ONEWIRE 6   // OneWire bus pin
#define PIN_DHT 7                 // DHT data pin
#define DHTTYPE DHT22

#define PIN_MQ135 A6
#define PIN_MQ2 A7

#define PIN_LDR A8
#define PIN_RAIN_INT 2
#define PIN_ANEMOMETER_INT 3
#define PIN_WIND_DIR A9

// ----------------- Keypad Wiring (change pins if needed) -----------------
const byte ROWS = 4;
const byte COLS = 4;
const byte rowPins[ROWS] = {30, 31, 32, 33}; // R1..R4
const byte colPins[COLS] = {34, 35, 36, 37}; // C1..C4

char keysMap[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

Keypad keypad = Keypad(makeKeymap(keysMap), rowPins, colPins, ROWS, COLS);

// ----------------- Calibration & timing -----------------
const float ANEMOMETER_CAL_FACTOR = 0.66; // m/s per (pulse/sec) — calibrate your anemometer
const float RAIN_MM_PER_TIP = 0.2794;     // mm per tip
const unsigned long WIND_MEASURE_INTERVAL_MS = 5000;
const unsigned long RAIN_MEASURE_INTERVAL_MS = 60000;
const unsigned long DISPLAY_PAGE_MS = 4000UL; // ms per page auto
const unsigned long MANUAL_DISPLAY_MS = 10000UL; // 10s for manual keypad display

// ----------------- Libraries init -----------------
DHT dht(PIN_DHT, DHTTYPE);
OneWire oneWire(PIN_SOIL_TEMP_ONEWIRE);
DallasTemperature soilTempSensor(&oneWire);

// LCD pointer (initialized after I2C scan)
LiquidCrystal_I2C *lcd = nullptr;
uint8_t lcdAddrDetected = 0x27; // fallback

// SIM port pointer (Serial1)
Stream *simPort = nullptr;

// ----------------- Globals -----------------
volatile unsigned long windPulseCount = 0;
volatile unsigned long rainTipCount = 0;

unsigned long lastDisplayPageAt = 0;
uint8_t currentDisplayPage = 0;
const uint8_t TOTAL_PAGES = 4;

enum DisplayMode { AUTO_PAGES, MANUAL_KEY };
DisplayMode displayMode = AUTO_PAGES;
unsigned long manualDisplayUntil = 0;
char lastKeyPressed = 0;

unsigned long lastSendAt = 0;
unsigned long lastRainSampleAt = 0;

// ----------------- Prototypes -----------------
String readSim(unsigned long timeout = 1200);
void flushSim();
String sendCmdRead(const char *cmd, unsigned long waitMs = 1000);
bool sendCmdExpect(const char *cmd, const char *expect, unsigned long timeout = 3000);
bool waitHTTPAction(int &outStatus, int &outLen, unsigned long timeoutMs = HTTPACTION_TIMEOUT);
String getSAPBRip();
bool openBearer();
void closeBearer();
String buildThingSpeakURL_fromValues(float f1, float f2, float f3, float f4,
                                     float f5, float f6, float f7, float f8);

float analogToPct(int raw);
float readSoilMoisturePrimary();
float readSoilMoistureSecondary();
float readSoilPH();
float readSoilNPK();
float readSoilEC();
float readSoilFertility();
float readAirTemp();
float readAirHumidity();
float readMQ135();
float readMQ2();
float readLight();
float readWindDirection();
float readSoilTemp();
float measureWindSpeedMillis(unsigned long intervalMs);
float measureRainSinceLast(unsigned long &outTips);

void lcdClearWriteLines(const String lines[4]);
void updateLCDPage(uint8_t page,
                   float soilMoisturePrimary, float soilMoistureSecondary,
                   float soilPH, float soilTemp,
                   float soilNPK, float soilEC, float soilFertility,
                   float airTemp, float airHumidity,
                   float mq135, float mq2, float light,
                   float windSpeed_m_s, float windDir, float rain_mm);

uint8_t detectI2CAddress();
void showSensorOnLCD(char key);

// ----------------- Implementations -----------------

// SIM helper implementations
String readSim(unsigned long timeout) {
  unsigned long start = millis();
  String s;
  while (millis() - start < timeout) {
    while (simPort->available()) {
      s += (char)simPort->read();
    }
  }
  return s;
}
void flushSim() { while (simPort->available()) simPort->read(); }
String sendCmdRead(const char *cmd, unsigned long waitMs) {
  flushSim();
  Serial.print(F("> "));
  Serial.println(cmd);
  simPort->println(cmd);
  delay(20);
  String r = readSim(waitMs);
  if (r.length()) Serial.print(r);
  else Serial.println(F("[no response]"));
  return r;
}
bool sendCmdExpect(const char *cmd, const char *expect, unsigned long timeout) {
  flushSim();
  Serial.print(F("> "));
  Serial.println(cmd);
  simPort->println(cmd);
  unsigned long start = millis();
  String resp = "";
  while (millis() - start < timeout) {
    while (simPort->available()) resp += (char)simPort->read();
    if (resp.indexOf(expect) != -1) { Serial.print(resp); return true; }
    if (resp.indexOf("ERROR") != -1 || resp.indexOf("CME ERROR") != -1) { Serial.print(resp); return false; }
  }
  Serial.print(resp);
  Serial.println(F("\n-- TIMEOUT waiting for expected response --"));
  return false;
}
bool waitHTTPAction(int &outStatus, int &outLen, unsigned long timeoutMs) {
  unsigned long t0 = millis();
  String accum;
  while (millis() - t0 < timeoutMs) {
    while (simPort->available()) {
      char c = (char)simPort->read();
      accum += c;
      Serial.write(c);
    }
    int idx = accum.indexOf("+HTTPACTION:");
    if (idx >= 0) {
      int eol = accum.indexOf('\n', idx);
      String line = (eol>idx) ? accum.substring(idx, eol) : accum.substring(idx);
      Serial.print(F("\nParsed: "));
      Serial.println(line);
      int p1 = line.indexOf(',');
      int p2 = (p1>0) ? line.indexOf(',', p1+1) : -1;
      if (p1>0 && p2>p1) {
        String s1 = line.substring(p1+1, p2);
        String s2 = line.substring(p2+1);
        outStatus = s1.toInt();
        outLen = s2.toInt();
        return true;
      }
    }
  }
  Serial.println(F("\nNo +HTTPACTION seen within timeout."));
  return false;
}
String getSAPBRip() {
  String r = sendCmdRead("AT+SAPBR=2,1", 1500);
  int p = r.indexOf("+SAPBR:");
  if (p >= 0) {
    int q = r.indexOf('"', p);
    if (q >= 0) {
      int q2 = r.indexOf('"', q+1);
      if (q2 > q) {
        String ip = r.substring(q+1, q2);
        return ip;
      }
    }
  }
  return "";
}
bool openBearer() {
  Serial.println(F("Opening SAPBR bearer..."));
  if (!sendCmdExpect("AT+SAPBR=3,1,\"Contype\",\"GPRS\"", "OK", 1500)) return false;
  String apnCmd = String("AT+SAPBR=3,1,\"APN\",\"") + APN + "\"";
  if (!sendCmdExpect(apnCmd.c_str(), "OK", 2000)) {
    Serial.println(F("Failed to set APN."));
    return false;
  }
  if (USER.length()) {
    String u = String("AT+SAPBR=3,1,\"USER\",\"") + USER + "\"";
    sendCmdRead(u.c_str(), 1000);
  }
  if (PASS.length()) {
    String p = String("AT+SAPBR=3,1,\"PWD\",\"") + PASS + "\"";
    sendCmdRead(p.c_str(), 1000);
  }
  if (!sendCmdExpect("AT+SAPBR=1,1", "OK", SAPBR_OPEN_TIMEOUT)) {
    Serial.println(F("AT+SAPBR=1,1 failed."));
    return false;
  }
  String ip = getSAPBRip();
  Serial.print(F("SAPBR IP: "));
  Serial.println(ip.length() ? ip : F("[no IP]"));
  if (ip.length() == 0 || ip == "0.0.0.0") return false;
  return true;
}
void closeBearer() { sendCmdRead("AT+SAPBR=0,1", 1000); }

// ThingSpeak builder
String buildThingSpeakURL_fromValues(float f1, float f2, float f3, float f4,
                                     float f5, float f6, float f7, float f8) {
  String url = THING_SPEAK_API_URL + "?api_key=" + THING_SPEAK_API_KEY;
  url += "&field1=" + String(f1, 2);
  url += "&field2=" + String(f2, 2);
  url += "&field3=" + String(f3, 2);
  url += "&field4=" + String(f4, 2);
  url += "&field5=" + String(f5, 2);
  url += "&field6=" + String(f6, 2);
  url += "&field7=" + String(f7, 2);
  url += "&field8=" + String(f8, 2);
  return url;
}

// Sensor helpers
float analogToPct(int raw) { return (float)raw / 1023.0 * 100.0; }
float readSoilMoisturePrimary() { return analogToPct(analogRead(PIN_SOIL_MOISTURE_PRIMARY)); }
float readSoilMoistureSecondary() { return analogToPct(analogRead(PIN_SOIL_MOISTURE_SECONDARY)); }
float readSoilPH() { return (float)analogRead(PIN_SOIL_PH) / 1023.0 * 14.0; } // placeholder mapping
float readSoilNPK() { return analogToPct(analogRead(PIN_SOIL_NPK)); }
float readSoilEC() { return analogToPct(analogRead(PIN_SOIL_EC)); }
float readSoilFertility() { return analogToPct(analogRead(PIN_SOIL_FERTILITY)); }
float readAirTemp() { float t = dht.readTemperature(); return isnan(t) ? NAN : t; }
float readAirHumidity() { float h = dht.readHumidity(); return isnan(h) ? NAN : h; }
float readMQ135() { return analogToPct(analogRead(PIN_MQ135)); }
float readMQ2()   { return analogToPct(analogRead(PIN_MQ2)); }
float readLight() { return analogToPct(analogRead(PIN_LDR)); }
float readWindDirection() { return (float)analogRead(PIN_WIND_DIR) / 1023.0 * 360.0; }
float readSoilTemp() {
  soilTempSensor.requestTemperatures();
  delay(10);
  float tC = soilTempSensor.getTempCByIndex(0);
  if (tC == DEVICE_DISCONNECTED_C) return NAN;
  return tC;
}

// Wind & rain ISRs and measurements
void IRAM_ATTR anemometerISR() { windPulseCount++; }
void IRAM_ATTR rainISR() { rainTipCount++; }

float measureWindSpeedMillis(unsigned long intervalMs) {
  noInterrupts();
  unsigned long startCount = windPulseCount;
  interrupts();
  delay(intervalMs);
  noInterrupts();
  unsigned long endCount = windPulseCount;
  interrupts();
  unsigned long pulses = (endCount - startCount);
  float pulsesPerSec = (float)pulses / (intervalMs / 1000.0);
  return pulsesPerSec * ANEMOMETER_CAL_FACTOR;
}

float measureRainSinceLast(unsigned long &outTips) {
  noInterrupts();
  unsigned long tips = rainTipCount;
  rainTipCount = 0;
  interrupts();
  outTips = tips;
  return (float)tips * RAIN_MM_PER_TIP;
}

// LCD helpers
String fit20(const String &s) {
  String t = s;
  if (t.length() > 20) t = t.substring(0, 20);
  while (t.length() < 20) t += ' ';
  return t;
}

void lcdClearWriteLines(const String lines[4]) {
  if (lcd) {
    lcd->clear();
    for (int i = 0; i < 4; ++i) {
      lcd->setCursor(0, i);
      lcd->print(lines[i]);
    }
  }
}

void updateLCDPage(uint8_t page,
                   float soilMoisturePrimary, float soilMoistureSecondary,
                   float soilPH, float soilTemp,
                   float soilNPK, float soilEC, float soilFertility,
                   float airTemp, float airHumidity,
                   float mq135, float mq2, float light,
                   float windSpeed_m_s, float windDir, float rain_mm)
{
  String lines[4];
  if (page == 0) {
    lines[0] = fit20("SoilPrm:" + String(soilMoisturePrimary,1) + "% Sec:" + String(soilMoistureSecondary,1) + "%");
    lines[1] = fit20("pH:" + String(soilPH,2) + " S.T:" + (isnan(soilTemp) ? "N/A" : String(soilTemp,1)+"C"));
    lines[2] = fit20("NPK:" + String(soilNPK,1) + "% EC:" + String(soilEC,1) + "%");
    lines[3] = fit20("Fert:" + String(soilFertility,1) + "% Rain:" + String(rain_mm,2) + "mm");
  } else if (page == 1) {
    lines[0] = fit20("AirT:" + (isnan(airTemp) ? "N/A" : String(airTemp,1)+"C") + " Hum:" + (isnan(airHumidity) ? "N/A" : String(airHumidity,1)+"%"));
    lines[1] = fit20("MQ135:" + String(mq135,1) + "% MQ2:" + String(mq2,1) + "%");
    lines[2] = fit20("Light:" + String(light,1) + "%");
    lines[3] = fit20("--------------------");
  } else if (page == 2) {
    lines[0] = fit20("WindSpd:" + String(windSpeed_m_s,2) + " m/s");
    lines[1] = fit20("WindDir:" + String(windDir,0) + " deg");
    lines[2] = fit20("WindTicks:" + String(windPulseCount));
    lines[3] = fit20("RainTicks:" + String(rainTipCount));
  } else {
    lines[0] = fit20("SoilT:" + (isnan(soilTemp) ? "N/A" : String(soilTemp,1)+"C") + " pH:" + String(soilPH,2));
    lines[1] = fit20("Soil M P/S:" + String(soilMoisturePrimary,1) + "/" + String(soilMoistureSecondary,1));
    lines[2] = fit20("Air T/H:" + (isnan(airTemp) ? "N/A" : String(airTemp,1)+"C/") + (isnan(airHumidity) ? "N/A" : String(airHumidity,1)+"%"));
    lines[3] = fit20("Light:" + String(light,1) + "% MQ135:" + String(mq135,1));
  }
  lcdClearWriteLines(lines);
}

// I2C scan
uint8_t detectI2CAddress() {
  Wire.begin(); // SDA=20, SCL=21 on Mega
  uint8_t found = 0;
  Serial.println(F("Scanning I2C bus for devices..."));
  for (uint8_t addr = 3; addr < 0x78; ++addr) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.print(F("I2C device found at 0x"));
      if (addr < 16) Serial.print('0');
      Serial.print(String(addr, HEX));
      Serial.println();
      if (found == 0) found = addr; // pick first found
    }
  }
  if (found) {
    Serial.print(F("Using detected LCD I2C address: 0x"));
    if (found < 16) Serial.print('0');
    Serial.println(String(found, HEX));
    return found;
  } else {
    Serial.println(F("No I2C device found. Falling back to 0x27."));
    return 0x27;
  }
}

// Keypad display behavior
void showSensorOnLCD(char key) {
  // Read sensors (single snapshot)
  float soilMoisturePrimary = readSoilMoisturePrimary();
  float soilMoistureSecondary = readSoilMoistureSecondary();
  float soilPH = readSoilPH();
  float soilTemp = readSoilTemp();
  float soilNPK = readSoilNPK();
  float soilEC = readSoilEC();
  float soilFertility = readSoilFertility();
  float airTemp = readAirTemp();
  float airHumidity = readAirHumidity();
  float mq135 = readMQ135();
  float mq2 = readMQ2();
  float light = readLight();
  float windDir = readWindDirection();
  float windSpeed_m_s = measureWindSpeedMillis(1000); // quick sample for display
  unsigned long tips; float rain_mm = measureRainSinceLast(tips);
  // restore tips to keep upload accounting
  noInterrupts(); rainTipCount += tips; interrupts();

  String l[4];
  switch (key) {
    case '1':
      l[0] = fit20("Soil Moisture (P)");
      l[1] = fit20("Value: " + String(soilMoisturePrimary,1) + " %");
      l[2] = fit20("--------------------");
      l[3] = fit20("Press any key...");
      break;
    case '2':
      l[0] = fit20("Soil Moisture (S)");
      l[1] = fit20("Value: " + String(soilMoistureSecondary,1) + " %");
      l[2] = fit20("--------------------");
      l[3] = fit20("Press any key...");
      break;
    case '3':
      l[0] = fit20("Soil pH (4502C)");
      l[1] = fit20("pH: " + String(soilPH,2));
      l[2] = fit20("--------------------");
      l[3] = fit20("Press any key...");
      break;
    case '4':
      l[0] = fit20("Soil Temp (OneWire)");
      l[1] = fit20("T: " + (isnan(soilTemp) ? String("N/A") : String(soilTemp,1)+" C"));
      l[2] = fit20("--------------------");
      l[3] = fit20("Press any key...");
      break;
    case '5':
      l[0] = fit20("Soil Nutrient (NPK)");
      l[1] = fit20("NPK Index: " + String(soilNPK,1) + "%");
      l[2] = fit20("--------------------");
      l[3] = fit20("Press any key...");
      break;
    case '6':
      l[0] = fit20("Soil Salinity / EC");
      l[1] = fit20("EC Index: " + String(soilEC,1) + "%");
      l[2] = fit20("--------------------");
      l[3] = fit20("Press any key...");
      break;
    case '7':
      l[0] = fit20("Soil Fertility");
      l[1] = fit20("Fertility: " + String(soilFertility,1) + "%");
      l[2] = fit20("--------------------");
      l[3] = fit20("Press any key...");
      break;
    case '8':
      l[0] = fit20("Air Temperature (DHT)");
      l[1] = fit20("T: " + (isnan(airTemp) ? String("N/A") : String(airTemp,1)+" C"));
      l[2] = fit20("Hum: " + (isnan(airHumidity) ? String("N/A") : String(airHumidity,1)+" %"));
      l[3] = fit20("Press any key...");
      break;
    case '9':
      l[0] = fit20("Air Humidity (DHT)");
      l[1] = fit20("Humidity: " + (isnan(airHumidity) ? String("N/A") : String(airHumidity,1) + " %"));
      l[2] = fit20("--------------------");
      l[3] = fit20("Press any key...");
      break;
    case '0':
      l[0] = fit20("Light (LDR)");
      l[1] = fit20("Light: " + String(light,1) + " %");
      l[2] = fit20("--------------------");
      l[3] = fit20("Press any key...");
      break;
    case 'A': {
      float avgSoilMoist = (soilMoisturePrimary + soilMoistureSecondary) / 2.0;
      float avgQuality = (soilNPK + soilEC + soilFertility) / 3.0;
      l[0] = fit20("AVERAGES SUMMARY");
      l[1] = fit20("Soil Moist Avg: " + String(avgSoilMoist,1) + "%");
      l[2] = fit20("Soil Qual Avg: " + String(avgQuality,1) + "%");
      l[3] = fit20("Press any key...");
      } break;
    case 'B':
      l[0] = fit20("Wind & Rain Summary");
      l[1] = fit20("Wind: " + String(windSpeed_m_s,2) + " m/s");
      l[2] = fit20("Dir: " + String(windDir,0) + " deg");
      l[3] = fit20("Rain: " + String(rain_mm,2) + " mm");
      break;
    case 'C':
      displayMode = MANUAL_KEY;
      manualDisplayUntil = millis() + MANUAL_DISPLAY_MS;
      currentDisplayPage = 0;
      updateLCDPage(currentDisplayPage,
                    readSoilMoisturePrimary(), readSoilMoistureSecondary(),
                    readSoilPH(), readSoilTemp(),
                    readSoilNPK(), readSoilEC(), readSoilFertility(),
                    readAirTemp(), readAirHumidity(),
                    readMQ135(), readMQ2(), readLight(),
                    0.0, readWindDirection(), 0.0);
      return;
    case 'D':
      displayMode = AUTO_PAGES;
      manualDisplayUntil = 0;
      currentDisplayPage = 0;
      return;
    default:
      l[0] = fit20("Unknown key");
      l[1] = fit20("Press 0-9,A,B,C,D");
      l[2] = fit20("--------------------");
      l[3] = fit20("Press any key...");
      break;
  }
  lcdClearWriteLines(l);
}

// ----------------- Setup & Loop -----------------
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  // SIM port
  Serial1.begin(9600);
  simPort = &Serial1;

  // Detect I2C address and init LCD
  lcdAddrDetected = detectI2CAddress();
  lcd = new LiquidCrystal_I2C(lcdAddrDetected, 20, 4);
  lcd->init();
  lcd->backlight();
  lcd->clear();

  // Show startup info
  lcd->setCursor(0,0); lcd->print("LCD Addr:");
  char buf[8]; sprintf(buf, "0x%02X", lcdAddrDetected);
  lcd->setCursor(0,1); lcd->print(buf);
  lcd->setCursor(0,2); lcd->print("Keypad Ready");
  lcd->setCursor(0,3); lcd->print("Starting...");
  delay(1000);

  // sensors
  dht.begin();
  soilTempSensor.begin();

  // interrupts
  pinMode(PIN_ANEMOMETER_INT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ANEMOMETER_INT), anemometerISR, FALLING);
  pinMode(PIN_RAIN_INT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_RAIN_INT), rainISR, FALLING);

  // Basic SIM checks
  sendCmdRead("AT", 500);
  sendCmdRead("ATE0", 200);
  sendCmdRead("AT+CPIN?", 800);
  sendCmdRead("AT+CSQ", 800);
  sendCmdRead("AT+CREG?", 800);

  lastDisplayPageAt = millis();
}

void loop() {
  unsigned long now = millis();

  // Keypad handling (non-blocking)
  char k = keypad.getKey();
  if (k) {
    lastKeyPressed = k;
    displayMode = MANUAL_KEY;
    manualDisplayUntil = millis() + MANUAL_DISPLAY_MS;
    showSensorOnLCD(k);
  }

  // Expire manual mode
  if (displayMode == MANUAL_KEY && millis() > manualDisplayUntil) {
    displayMode = AUTO_PAGES;
    currentDisplayPage = 0;
  }

  // Auto page cycling
  if (displayMode == AUTO_PAGES) {
    if (now - lastDisplayPageAt >= DISPLAY_PAGE_MS) {
      // cheap reads for display
      float soilMoisturePrimary = analogToPct(analogRead(PIN_SOIL_MOISTURE_PRIMARY));
      float soilMoistureSecondary = analogToPct(analogRead(PIN_SOIL_MOISTURE_SECONDARY));
      float soilPH = (float)analogRead(PIN_SOIL_PH) / 1023.0 * 14.0;
      float soilTemp = soilTempSensor.getTempCByIndex(0);
      float soilNPK = analogToPct(analogRead(PIN_SOIL_NPK));
      float soilEC = analogToPct(analogRead(PIN_SOIL_EC));
      float soilFertility = analogToPct(analogRead(PIN_SOIL_FERTILITY));
      float airTemp = dht.readTemperature();
      float airHumidity = dht.readHumidity();
      float mq135 = analogToPct(analogRead(PIN_MQ135));
      float mq2 = analogToPct(analogRead(PIN_MQ2));
      float light = analogToPct(analogRead(PIN_LDR));
      float windDir = (float)analogRead(PIN_WIND_DIR) / 1023.0 * 360.0;
      unsigned long rainTicksLocal;
      noInterrupts(); rainTicksLocal = rainTipCount; interrupts();
      float rain_mm = (float)rainTicksLocal * RAIN_MM_PER_TIP;
      float windSpeed_m_s = 0.0; // avoid blocking here
      updateLCDPage(currentDisplayPage,
                    soilMoisturePrimary, soilMoistureSecondary,
                    soilPH, soilTemp,
                    soilNPK, soilEC, soilFertility,
                    airTemp, airHumidity,
                    mq135, mq2, light,
                    windSpeed_m_s, windDir, rain_mm);
      currentDisplayPage++;
      if (currentDisplayPage >= TOTAL_PAGES) currentDisplayPage = 0;
      lastDisplayPageAt = now;
    }
  }

  // Mirror SIM output to Serial console
  while (simPort->available()) Serial.write(simPort->read());

  // Throttle ThingSpeak uploads
  if (now - lastSendAt < SEND_INTERVAL_MS) return;
  lastSendAt = now;

  // Full sensor read & LCD update (page 0)
  float soilMoisturePrimary = readSoilMoisturePrimary();
  float soilMoistureSecondary = readSoilMoistureSecondary();
  float soilPH = readSoilPH();
  float soilTemp = readSoilTemp();
  float soilNPK = readSoilNPK();
  float soilEC = readSoilEC();
  float soilFertility = readSoilFertility();

  float airTemp = readAirTemp();
  float airHumidity = readAirHumidity();
  float mq135 = readMQ135();
  float mq2 = readMQ2();
  float light = readLight();
  float windDir = readWindDirection();

  float windSpeed_m_s = measureWindSpeedMillis(WIND_MEASURE_INTERVAL_MS);

  unsigned long rainTips = 0;
  float rain_mm = 0.0;
  unsigned long now2 = millis();
  if (now2 - lastRainSampleAt >= RAIN_MEASURE_INTERVAL_MS) {
    rain_mm = measureRainSinceLast(rainTips);
    lastRainSampleAt = now2;
  } else {
    noInterrupts(); rainTips = rainTipCount; interrupts();
    rain_mm = (float)rainTips * RAIN_MM_PER_TIP;
  }

  updateLCDPage(0,
                soilMoisturePrimary, soilMoistureSecondary,
                soilPH, soilTemp,
                soilNPK, soilEC, soilFertility,
                airTemp, airHumidity,
                mq135, mq2, light,
                windSpeed_m_s, windDir, rain_mm);
  currentDisplayPage = 1;
  lastDisplayPageAt = millis();

  // Ensure bearer and send to ThingSpeak
  String ip = getSAPBRip();
  if (ip.length() == 0 || ip == "0.0.0.0") {
    Serial.println(F("No valid SAPBR IP — opening bearer..."));
    if (!openBearer()) {
      Serial.println(F("Failed to open bearer. Check APN, SIM, signal and power."));
      delay(10000);
      return;
    }
  } else {
    Serial.print(F("Existing SAPBR IP: ")); Serial.println(ip);
  }

  String url = buildThingSpeakURL_fromValues(
    soilMoisturePrimary,
    soilMoistureSecondary,
    soilPH,
    soilTemp,
    soilNPK,
    soilEC,
    airTemp,
    airHumidity
  );
  Serial.print(F("Final URL: ")); Serial.println(url);

  sendCmdRead("AT+HTTPINIT", 1200);
  sendCmdRead("AT+HTTPPARA=\"CID\",1", 800);
  if (USE_SSL) sendCmdRead("AT+HTTPSSL=1", 800);
  else sendCmdRead("AT+HTTPSSL=0", 800);

  String urlCmd = String("AT+HTTPPARA=\"URL\",\"") + url + "\"";
  sendCmdRead(urlCmd.c_str(), 2000);

  sendCmdRead("AT+HTTPACTION=0", 200);
  int httpStatus = -1; int httpLen = 0;
  bool got = waitHTTPAction(httpStatus, httpLen, HTTPACTION_TIMEOUT);
  if (got) {
    Serial.print(F("+HTTPACTION result status="));
    Serial.print(httpStatus);
    Serial.print(F(" len="));
    Serial.println(httpLen);
    if (httpStatus == 200) {
      sendCmdRead("AT+HTTPREAD", 2000);
    } else if (httpStatus >= 600) {
      Serial.print(F("Module-level HTTP error (6xx): "));
      Serial.println(httpStatus);
      sendCmdRead("AT+CSQ", 800);
      sendCmdRead("AT+CGATT?", 800);
      sendCmdRead("AT+SAPBR=2,1", 800);
      sendCmdRead("AT+SAPBR=0,1", 800);
      delay(1000);
      if (!openBearer()) Serial.println(F("Reopen bearer failed."));
    } else {
      Serial.print(F("HTTP returned non-200 status: "));
      Serial.println(httpStatus);
      sendCmdRead("AT+HTTPREAD", 2000);
    }
  } else {
    Serial.println(F("No +HTTPACTION notification received (timeout)."));
    sendCmdRead("AT+CSQ", 800);
    sendCmdRead("AT+CGATT?", 800);
  }

  sendCmdRead("AT+HTTPTERM", 800);
  delay(50);
}
