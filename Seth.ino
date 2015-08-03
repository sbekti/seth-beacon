#include <SoftwareSerial.h>
#include <TinyGPS++.h>

#define ID 1
#define APN "internet"
#define ARDUINO_RESET_PIN 7
#define SIM800_RESET_PIN 8

SoftwareSerial gpsSerial(11, 12); // RX, TX
SoftwareSerial gsmSerial(9, 10); // RX, TX
TinyGPSPlus gps;

char latSign, lngSign;
uint16_t latDeg, lngDeg;
uint32_t latFrac, lngFrac;
uint32_t date, time, speed, course, altitude, satellites, hdop;
uint32_t age, last;
char gsmSerialBuffer[256];
uint8_t failureCount;

void gpsUpdateValues();
uint8_t gsmInit();
uint8_t gsmSetup(const char* apn);
void gsmProcessSMS(uint16_t id, const char* sender, const char* text);
void gsmCheckSMS();
int16_t gsmGetSignalQuality();
void gsmGetBatteryLevel(uint8_t* percentage, uint16_t* voltage);
uint8_t gsmSendGETRequest(const char* url);
uint8_t gsmSendPOSTRequest(const char* url, const char* params);
void gsmPurgeSerialBuffer();
uint8_t gsmSendCommand(const char* cmd, uint16_t timeout = 5000, const char* expected = 0);
uint8_t gsmSendCommand_P(const __FlashStringHelper* cmd, uint16_t timeout = 5000, const char* expected = 0);
void reboot();

const char format_0[] PROGMEM = "id=%d&lat=%c%u.%lu&lng=%c%u.%lu&date=%lu&time=%lu&speed=%lu&course=%lu&alt=%lu&sat=%lu&hdop=%lu&age=%lu&charge=%d&voltage=%u&signal=%d";
const char format_1[] PROGMEM = "id=%d&charge=%d&voltage=%u&signal=%d";

void gpsUpdateValues() {
  if (gps.location.isUpdated()) {
    age = gps.location.age();
    latSign = gps.location.rawLat().negative ? '-' : '+';
    latDeg = gps.location.rawLat().deg;
    latFrac = gps.location.rawLat().billionths;
    lngSign = gps.location.rawLng().negative ? '-' : '+';
    lngDeg = gps.location.rawLng().deg;
    lngFrac = gps.location.rawLng().billionths;
  }

  if (gps.date.isUpdated()) {
    date = gps.date.value();
  }

  if (gps.time.isUpdated()) {
    time = gps.time.value();
  }

  if (gps.speed.isUpdated()) {
    speed = gps.speed.value();
  }

  if (gps.course.isUpdated()) {
    course = gps.course.value();
  }

  if (gps.altitude.isUpdated()) {
    altitude = gps.altitude.value();
  }

  if (gps.satellites.isUpdated()) {
    satellites = gps.satellites.value();
  }

  if (gps.hdop.isUpdated()) {
    hdop = gps.hdop.value();
  }
}

uint8_t gsmInit() {
  pinMode(SIM800_RESET_PIN, OUTPUT);
  digitalWrite(SIM800_RESET_PIN, HIGH);
  delay(1000);
  digitalWrite(SIM800_RESET_PIN, LOW);
  delay(1000);
  digitalWrite(SIM800_RESET_PIN, HIGH);
  delay(3000);

  if (!gsmSendCommand_P(F("AT"))) {
    return 1;
  }
  
  if (!gsmSendCommand_P(F("AT+IPR=9600"))) {
    return 2;
  }
  
  if (!gsmSendCommand_P(F("AT+CFUN=1"))) {
    return 3;
  }

  return 0;
}

uint8_t gsmSetup(const char* apn) {
  bool success = false;

  for (uint8_t n = 0; n < 30; n++) {
    if (gsmSendCommand_P(F("AT+CREG?"))) {
      char *p = strstr(gsmSerialBuffer, "0,");

      if (p) {
        char mode = *(p + 2);

        if (mode == '1' || mode == '5') {
          success = true;
          break;
        }
      }
    }

    delay(1000);
  }

  if (!success) {
    return 1;
  }

  if (!gsmSendCommand_P(F("AT+CGATT?"))) {
    return 2;
  }

  if (!gsmSendCommand_P(F("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\""))) {
    return 3;
  }

  gsmSerial.listen();
  gsmSerial.print(F("AT+SAPBR=3,1,\"APN\",\""));
  gsmSerial.print(apn);
  gsmSerial.println(F("\""));

  if (!gsmSendCommand(0)) {
    return 4;
  }

  if (!gsmSendCommand_P(F("ATS0=1"))) {
    return 5;
  }

  if (!gsmSendCommand_P(F("AT+CMGF=1"))) {
    return 6;
  }

  if (!gsmSendCommand_P(F("AT+CPMS=\"SM\",\"SM\",\"SM\""))) {
    return 7;
  }

  return 0;
}

void gsmProcessSMS(uint16_t id, const char* sender, const char* text) {
  Serial.println(id);
  Serial.println(sender);
  Serial.println(text);

  if (strcmp(text, "reboot") == 0) reboot();

  char buffer[12];
  sprintf(buffer, "AT+CMGD=%u", id);
  gsmSendCommand(buffer);
}

void gsmCheckSMS() {
  gsmSendCommand_P(F("AT+CMGL=\"ALL\""));

  char* p = strstr(gsmSerialBuffer, "CMGL: ");

  while (p) {
    p += 6;

    uint8_t i;
    char index[3];
    char sender[20];
    char text[161];

    i = 0;
    memset(index, 0, sizeof(index));

    while (*p != ',') {
      index[i++] = *p++;
    }

    while (*p != '"') ++p; ++p;
    while (*p != '"') ++p; ++p;
    while (*p != '"') ++p; ++p;

    i = 0;
    memset(sender, 0, sizeof(sender));

    while (*p != '"') {
      sender[i++] = *p++;
    }

    while (*p != '\n') ++p; ++p;

    i = 0;
    memset(text, 0, sizeof(text));

    while (*p != '\r') {
      text[i++] = *p++;
    }

    uint16_t id = atoi(index);
    gsmProcessSMS(id, sender, text);

    p = strstr(p, "CMGL: ");
  };
}

int16_t gsmGetSignalQuality() {
  gsmSendCommand_P(F("AT+CSQ"));

  char* p = strstr(gsmSerialBuffer, "CSQ: ");

  if (p) {
    p += 5;
    uint8_t i = 0;
    char buffer[3];

    while (*p != ',') {
      buffer[i++] = *(p++);
    }

    int n = atoi(buffer);
    if (n == 99 || n == -1) return 0;
    return n * 2 - 114;
  }

  return 0;
}

void gsmGetBatteryLevel(uint8_t* percentage, uint16_t* voltage) {
  gsmSendCommand_P(F("AT+CBC"));

  char *p = strstr(gsmSerialBuffer, "CBC: ");

  if (p) {
    p += 7;
    uint8_t i;
    char buffer[5];

    i = 0;
    memset(buffer, 0, sizeof(buffer));

    while (*p != ',') {
      buffer[i++] = *(p++);
    }

    *percentage = atoi(buffer);

    ++p;
    i = 0;
    memset(buffer, 0, sizeof(buffer));

    while (*p != '\r') {
      buffer[i++] = *(p++);
    }

    *voltage = atoi(buffer);
  };
}

uint8_t gsmSendPOSTRequest(const char* url, const char* params) {
  if (!gsmSendCommand_P(F("AT+SAPBR=1,1"), 10000)) {
    return 1;
  }

  if (!gsmSendCommand_P(F("AT+SAPBR=2,1"))) {
    return 2;
  }

  if (!gsmSendCommand_P(F("AT+HTTPINIT"))) {
    return 3;
  }

  if (!gsmSendCommand_P(F("AT+HTTPPARA=\"CID\",1"))) {
    return 4;
  }

  gsmSerial.listen();
  gsmSerial.print(F("AT+HTTPPARA=\"URL\",\""));
  gsmSerial.print(url);
  gsmSerial.println(F("\""));

  if (!gsmSendCommand(0)) {
    return 5;
  }

  if (!gsmSendCommand("AT+HTTPPARA=\"UA\",\"bekti-tracker\"")) {
    return 6;
  }

  if (!gsmSendCommand("AT+HTTPPARA=\"CONTENT\",\"application/x-www-form-urlencoded\"")) {
    return 7;
  }

  if (!gsmSendCommand("AT+HTTPPARA=\"TIMEOUT\",30")) {
    return 8;
  }

  char paramsBuffer[25];
  uint16_t paramsLength = strlen(params);
  sprintf(paramsBuffer, "AT+HTTPDATA=%d,10000", paramsLength);

  if (!gsmSendCommand(paramsBuffer, 5000, "DOWNLOAD")) {
    return 9;
  }

  gsmSerial.listen();
  gsmSerial.print(params);

  if (!gsmSendCommand(0)) {
    return 10;
  }

  if (!gsmSendCommand_P(F("AT+HTTPACTION=1"), 45000, "+HTTPACTION:")) {
    return 11;
  }

  if (!gsmSendCommand_P(F("AT+HTTPTERM"))) {
    return 12;
  }

  if (!gsmSendCommand_P(F("AT+SAPBR=0,1"))) {
    return 13;
  }

  return 0;
}

void gsmPurgeSerialBuffer() {
  while (gsmSerial.available())  {
    gsmSerial.read();
  }
}

uint8_t gsmSendCommand(const char* cmd, uint16_t timeout, const char* expected) {
  gsmSerial.listen();

  if (cmd) {
    gsmPurgeSerialBuffer();
    gsmSerial.println(cmd);
  }

  uint32_t t = millis();
  uint8_t n = 0;

  do {
    if (gsmSerial.available()) {
      char c = gsmSerial.read();
      Serial.write(c);

      if (n >= sizeof(gsmSerialBuffer) - 1) {
        // buffer full, discard first half
        n = sizeof(gsmSerialBuffer) / 2 - 1;
        memcpy(gsmSerialBuffer, gsmSerialBuffer + sizeof(gsmSerialBuffer) / 2, n);
      }

      gsmSerialBuffer[n++] = c;
      gsmSerialBuffer[n] = 0;

      if (strstr(gsmSerialBuffer, expected ? expected : "OK\r")) {
        return n;
      }
    }
  } while (millis() - t < timeout);

  return 0;
}

uint8_t gsmSendCommand_P(const __FlashStringHelper* cmd, uint16_t timeout, const char* expected) {
  gsmSerial.listen();

  if (cmd) {
    gsmPurgeSerialBuffer();
    gsmSerial.println(cmd);
  }

  uint32_t t = millis();
  uint8_t n = 0;

  do {
    if (gsmSerial.available()) {
      char c = gsmSerial.read();
      Serial.write(c);

      if (n >= sizeof(gsmSerialBuffer) - 1) {
        // buffer full, discard first half
        n = sizeof(gsmSerialBuffer) / 2 - 1;
        memcpy(gsmSerialBuffer, gsmSerialBuffer + sizeof(gsmSerialBuffer) / 2, n);
      }

      gsmSerialBuffer[n++] = c;
      gsmSerialBuffer[n] = 0;

      if (strstr(gsmSerialBuffer, expected ? expected : "OK\r")) {
        return n;
      }
    }
  } while (millis() - t < timeout);

  return 0;
}

void reboot() {
  digitalWrite(ARDUINO_RESET_PIN, LOW);
}

void setup() {
  digitalWrite(ARDUINO_RESET_PIN, HIGH);
  pinMode(ARDUINO_RESET_PIN, OUTPUT);
  
  Serial.begin(9600);
  gpsSerial.begin(9600);
  gsmSerial.begin(9600);

  for (;;) {
    Serial.println(F("Initializing GSM... "));
    uint8_t ret = gsmInit();
    
    if (ret == 0) {
      Serial.println(F("OK"));
      break;
    }
    
    Serial.print(F("Error code: "));
    Serial.println(ret);
  }

  for (;;) {
    Serial.println(F("Setting up network... "));
    uint8_t ret = gsmSetup(APN);
    
    if (ret == 0) {
      Serial.println(F("OK"));
      break;
    }
    
    Serial.print(F("Error code: "));
    Serial.println(ret);
  }
}

void loop() {
  gpsSerial.listen();

  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
    gpsUpdateValues();
  }

  if (millis() - last > 10000) {
    Serial.println(F("Interval"));

    gsmCheckSMS();

    uint8_t batteryPercentage;
    uint16_t batteryVoltage;
    gsmGetBatteryLevel(&batteryPercentage, &batteryVoltage);
    int16_t signalStrength = gsmGetSignalQuality();
    char paramsBuffer[200];

    if (gps.location.isValid()) {
      sprintf_P(paramsBuffer, format_0, ID, latSign, latDeg, latFrac, lngSign, lngDeg, lngFrac, date, time, speed, course, altitude, satellites, hdop, age, batteryPercentage, batteryVoltage, signalStrength);
    } else {
      sprintf_P(paramsBuffer, format_1, ID, batteryPercentage, batteryVoltage, signalStrength);
    }

    uint8_t ret = gsmSendPOSTRequest("seth.bekti.io/api/v1/location", paramsBuffer);

    if (ret != 0) {
      ++failureCount;
    } else {
      failureCount = 0;
    }

    if (failureCount >= 3) {
      reboot();
    }

    last = millis();
  }
}
