#include <SoftwareSerial.h>
#include <TinyGPS++.h>

#define ID    1
#define APN   "internet"

SoftwareSerial gpsSerial(11, 12); // RX, TX
SoftwareSerial gsmSerial(9, 10); // RX, TX
TinyGPSPlus gps;

char latSign, lngSign;
uint16_t latDeg, lngDeg;
uint32_t latFrac, lngFrac;
uint32_t date, time, speed, course, altitude, satellites, hdop;
uint32_t age, last;
char gsmSerialBuffer[256];

void gpsUpdateValues();
uint8_t gsmInit();
uint8_t gsmSetup(const char* apn);
int16_t gsmGetSignalQuality();
void gsmGetBatteryLevel(uint8_t* percentage, uint16_t* voltage);
uint8_t gsmSendGETRequest(const char* url);
uint8_t gsmSendPOSTRequest(const char* url, const char* params);
void gsmPurgeSerialBuffer();
uint8_t gsmSendCommand(const char* cmd, uint16_t timeout = 2000, const char* expected = 0);

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
  if (gsmSendCommand("AT")) {
    gsmSendCommand("AT+IPR=9600");
    gsmSendCommand("ATE0");
    gsmSendCommand("AT+CFUN=1", 10000);
    
    return 0;
  }
  
  return 1;
}

uint8_t gsmSetup(const char* apn) {
  bool success = false;
  
  for (byte n = 0; n < 30; n++) {
    if (gsmSendCommand("AT+CREG?", 2000)) {
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
  
  if (!gsmSendCommand("AT+CGATT?", 5000)) {
    return 2;
  }
    
  if (!gsmSendCommand("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"", 5000)) {
    return 3;
  }

  gsmSerial.listen();
  gsmSerial.print("AT+SAPBR=3,1,\"APN\",\"");
  gsmSerial.print(apn);
  gsmSerial.println('\"');
  
  if (!gsmSendCommand(0, 5000)) {
    return 4;
  }

  return 0;
}

int16_t gsmGetSignalQuality() {
  gsmSendCommand("AT+CSQ", 5000);
  
  char *p = strstr(gsmSerialBuffer, "CSQ: ");
  
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
  gsmSendCommand("AT+CBC", 5000);
  
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

uint8_t gsmSendGETRequest(const char* url) {
  if (!gsmSendCommand("AT+SAPBR=1,1", 10000)) {
    return 1;
  }
  
  if (!gsmSendCommand("AT+SAPBR=2,1", 10000)) {
    return 2;
  }
  
  if (!gsmSendCommand("AT+HTTPINIT", 5000)) {
    return 3;
  }
  
  if (!gsmSendCommand("AT+HTTPPARA=\"CID\",1", 5000)) {
    return 4;
  }

  gsmSerial.listen();
  gsmSerial.print("AT+HTTPPARA=\"URL\",\"");
  gsmSerial.print(url);
  gsmSerial.println('\"');
  
  if (!gsmSendCommand(0, 5000)) {
    return 5;
  }

  if (!gsmSendCommand("AT+HTTPPARA=\"UA\",\"bekti-tracker\"", 5000)) {
    return 6;
  }

  if (!gsmSendCommand("AT+HTTPPARA=\"TIMEOUT\",30", 5000)) {
    return 7;
  }
  
  if (!gsmSendCommand("AT+HTTPACTION=0", 45000, "+HTTPACTION:")) {
    return 8;
  }
  
  if (!gsmSendCommand("AT+HTTPTERM", 5000)) {
    return 9;
  }

  if (!gsmSendCommand("AT+SAPBR=0,1", 10000)) {
    return 10;
  }
}

uint8_t gsmSendPOSTRequest(const char* url, const char* params) {
  if (!gsmSendCommand("AT+SAPBR=1,1", 10000)) {
    return 1;
  }
  
  if (!gsmSendCommand("AT+SAPBR=2,1", 10000)) {
    return 2;
  }
  
  if (!gsmSendCommand("AT+HTTPINIT", 5000)) {
    return 3;
  }
  
  if (!gsmSendCommand("AT+HTTPPARA=\"CID\",1", 5000)) {
    return 4;
  }

  gsmSerial.listen();
  gsmSerial.print("AT+HTTPPARA=\"URL\",\"");
  gsmSerial.print(url);
  gsmSerial.println('\"');
  
  if (!gsmSendCommand(0, 5000)) {
    return 5;
  }

  if (!gsmSendCommand("AT+HTTPPARA=\"UA\",\"bekti-tracker\"", 5000)) {
    return 6;
  }

  if (!gsmSendCommand("AT+HTTPPARA=\"CONTENT\",\"application/x-www-form-urlencoded\"", 5000)) {
    return 7;
  }

  if (!gsmSendCommand("AT+HTTPPARA=\"TIMEOUT\",30", 5000)) {
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

  if (!gsmSendCommand(0, 5000)) {
    return 10;
  }
  
  if (!gsmSendCommand("AT+HTTPACTION=1", 45000, "+HTTPACTION:")) {
    return 11;
  }
  
  if (!gsmSendCommand("AT+HTTPTERM")) {
    return 12;
  }

  if (!gsmSendCommand("AT+SAPBR=0,1", 10000)) {
    return 13;
  }
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

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
  gsmSerial.begin(9600);

  for (;;) {
    Serial.print(F("Initializing GSM... "));
    while (!gsmInit());
    Serial.println(F("OK"));
  
    Serial.print(F("Setting up network... "));
    uint8_t ret = gsmSetup(APN);
    if (ret == 0) break;
    Serial.print(F("Error code: "));
    Serial.println(ret);
  }
  
  Serial.println(F("OK"));
}

void loop() {
  gpsSerial.listen();
  
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
    gpsUpdateValues();
  }

  if (millis() - last > 10000) {
    Serial.println(F("Interval"));

    uint8_t batteryPercentage;
    uint16_t batteryVoltage;
    gsmGetBatteryLevel(&batteryPercentage, &batteryVoltage);
    int16_t signalStrength = gsmGetSignalQuality();

    if (gps.location.isValid()) {
      char paramsBuffer[200];
      sprintf(paramsBuffer, "id=%d&lat=%c%u.%lu&lng=%c%u.%lu&date=%lu&time=%lu&speed=%lu&course=%lu&alt=%lu&sat=%lu&hdop=%lu&age=%lu&charge=%d&voltage=%u&signal=%d", ID, latSign, latDeg, latFrac, lngSign, lngDeg, lngFrac, date, time, speed, course, altitude, satellites, hdop, age, batteryPercentage, batteryVoltage, signalStrength);
      Serial.println(paramsBuffer);
      gsmSendPOSTRequest("seth.bekti.io/api/v1/location", paramsBuffer);
    } else {
      char paramsBuffer[50];
      sprintf(paramsBuffer, "id=%d&charge=%d&voltage=%u&signal=%d", ID, batteryPercentage, batteryVoltage, signalStrength);
      gsmSendPOSTRequest("seth.bekti.io/api/v1/location", paramsBuffer);
    }
    
    last = millis();
  }
}
