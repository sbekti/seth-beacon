#include <avr/eeprom.h>
#include <ctype.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

#define ARDUINO_RESET_PIN 7
#define GSM_RESET_PIN 8

SoftwareSerial _gps_serial(11, 12); // RX, TX
SoftwareSerial _gsm_serial(9, 10); // RX, TX

TinyGPSPlus _gps;
TinyGPSCustom _gps_fix(_gps, "GPGSA", 2);

uint32_t _last_upload;
uint32_t _last_sms_check;
uint8_t _failure_count;

uint16_t _device_id;
uint16_t EEMEM _device_id_E;

uint32_t _interval;
uint16_t EEMEM _interval_E;

char _apn[25];
char EEMEM _apn_E[25];

char _gsm_serial_buffer[256];

char _lat[12];
char _lng[12];
char _speed[8];
char _course[8];
char _altitude[8];
uint8_t _date_day;
uint8_t _date_month;
uint16_t _date_year;
uint8_t _time_hour;
uint8_t _time_minute;
uint8_t _time_second;
uint32_t _satellites;
uint32_t _hdop;
uint32_t _age;
uint32_t _fix;

const char _format_post[] PROGMEM = "id=%u&fix=%lu&lat=%s&lng=%s&alt=%s&speed=%s&course=%s&date=%02u%02u%04u&time=%02u%02u%02u&sat=%lu&hdop=%lu&age=%lu&charge=%u&voltage=%u&signal=%d";
const char _format_sms[] PROGMEM = "#%u %luD Fix %02u/%02u/%04u %02u:%02u:%02u maps.google.com/maps?q=loc:%s,%s alt=%s sat=%lu hdop=%lu age=%lu chg=%u vtg=%u sig=%d";

void gsm_reset();
uint8_t gsm_init();
void gsm_check_sms();
void gsm_process_sms(uint16_t id, const char* sender, const char* text);
uint8_t gsm_send_sms(const char* destination, const char* text);
int8_t gsm_get_signal_quality();
void gsm_get_battery_level(uint8_t* percentage, uint16_t* voltage);
uint8_t gsm_send_post_request(const char* url, const char* params);
void gsm_purge_serial_buffer();
uint8_t gsm_send_command(const char* command, uint16_t timeout = 2000, const char* expected = 0);
uint8_t gsm_send_command_P(const __FlashStringHelper* command, uint16_t timeout = 2000, const char* expected = 0);
void reboot();

void gps_init_values();
void gps_update_values();

void gsm_reset() {
  pinMode(GSM_RESET_PIN, OUTPUT);
  digitalWrite(GSM_RESET_PIN, HIGH);
  delay(1000);
  digitalWrite(GSM_RESET_PIN, LOW);
  delay(1000);
  digitalWrite(GSM_RESET_PIN, HIGH);
  delay(1000);
}

uint8_t gsm_init() {
  if (gsm_send_command_P(F("AT"))) {
    return 1;
  }

  if (gsm_send_command_P(F("AT+IPR=9600"))) {
    return 2;
  }

  if (gsm_send_command_P(F("ATE0"))) {
    return 3;
  }

  if (gsm_send_command_P(F("AT+CFUN=1"))) {
    return 4;
  }

  bool success = false;

  for (uint8_t n = 0; n < 30; n++) {
    if (gsm_send_command_P(F("AT+CREG?")) == 0) {
      char *p = strstr(_gsm_serial_buffer, "0,");

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
    return 5;
  }

  if (gsm_send_command_P(F("AT+CMGF=1"))) {
    return 6;
  }

  if (gsm_send_command_P(F("AT+CPMS=\"SM\",\"SM\",\"SM\""))) {
    return 7;
  }

  eeprom_read_block((void*)&_apn, (const void*)&_apn_E, sizeof(_apn) - 1);
  Serial.print(F("Using APN: "));
  Serial.println(_apn);

  if (gsm_send_command_P(F("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\""))) {
    return 8;
  }

  _gsm_serial.listen();
  _gsm_serial.print(F("AT+SAPBR=3,1,\"APN\",\""));
  _gsm_serial.print(_apn);
  _gsm_serial.println(F("\""));

  if (gsm_send_command(0)) {
    return 9;
  }

  if (gsm_send_command_P(F("ATS0=1"))) {
    return 10;
  }

  return 0;
}

void gsm_check_sms() {
  gsm_send_command_P(F("AT+CMGL=\"ALL\""));
  
  char* p = strstr(_gsm_serial_buffer, "CMGL: ");

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
      if (i >= sizeof(index)) break;
    }

    while (*p != '"') ++p; ++p;
    while (*p != '"') ++p; ++p;
    while (*p != '"') ++p; ++p;

    i = 0;
    memset(sender, 0, sizeof(sender));

    while (*p != '"') {
      sender[i++] = *p++;
      if (i >= sizeof(sender)) break;
    }

    while (*p != '\n') ++p; ++p;

    i = 0;
    memset(text, 0, sizeof(text));

    while (*p != '\r') {
      text[i++] = *p++;
      if (i >= sizeof(text)) break;
    }

    uint16_t id = atoi(index);
    gsm_process_sms(id, sender, text);

    p = strstr(p, "CMGL: ");
  }
}

void gsm_process_sms(uint16_t id, const char* sender, char* text) {
  char buffer[12];
  sprintf(buffer, "AT+CMGD=%u", id);
  gsm_send_command(buffer);

  for (uint8_t i = 0; text[i]; ++i) {
    text[i] = tolower(text[i]);
  }

  if (strncmp_P(text, PSTR("reboot"), 6) == 0) {
    gsm_send_sms(sender, "OK");
    reboot();
  }

  if ((strncmp_P(text, PSTR("id"), 2) == 0) && (strlen(text) > 3)) {
    char param[5];
    memset(param, 0, sizeof(param));

    uint8_t i = 0;
    text += 3;

    while (*text) {
      param[i++] = *text++;
      if (i >= sizeof(param) - 1) break;
    }

    uint16_t device_id = atoi(param);

    if (device_id > 0) {
      eeprom_write_word(&_device_id_E, device_id);
      _device_id = device_id;
      
      char reply[10];
      sprintf_P(reply, PSTR("Device ID: %u"), _device_id);
      gsm_send_sms(sender, reply);

      Serial.print(F("Device ID updated: "));
      Serial.println(_device_id);
    }
  }

  if ((strncmp_P(text, PSTR("interval"), 8) == 0) && (strlen(text) > 9)) {
    char param[5];
    memset(param, 0, sizeof(param));

    uint8_t i = 0;
    text += 9;

    while (*text) {
      param[i++] = *text++;
      if (i >= sizeof(param) - 1) break;
    }

    uint16_t interval = atoi(param);

    if (interval > 0) {
      eeprom_write_word(&_interval_E, interval);
      _interval = interval * 1000;
      
      char reply[25];
      sprintf_P(reply, PSTR("Interval: %lu ms"), _interval);
      gsm_send_sms(sender, reply);

      Serial.print(F("Interval updated: "));
      Serial.println(_interval);
    }
  }

  if ((strncmp_P(text, PSTR("apn"), 3) == 0) && (strlen(text) > 4)) {
    char param[25];
    memset(param, 0, sizeof(param));

    uint8_t i = 0;
    text += 4;

    while (*text) {
      param[i++] = *text++;
      if (i >= sizeof(param) - 1) break;
    }

    if (strlen(param) > 0) {
      eeprom_write_block((const void*)&param, (void*)&_apn_E, sizeof(_apn) - 1);
      memcpy(_apn, param, sizeof(_apn));

      char reply[30];
      sprintf_P(reply, PSTR("APN: %s"), _apn);
      gsm_send_sms(sender, reply);

      Serial.print(F("APN updated: "));
      Serial.println(_apn);
    }
  }

  if (strncmp_P(text, PSTR("location"), 8) == 0) {
    uint8_t battery_percentage;
    uint16_t battery_voltage;
    gsm_get_battery_level(&battery_percentage, &battery_voltage);
    int8_t signal_strength = gsm_get_signal_quality();
    
    char reply[161];
    sprintf_P(reply, _format_sms, _fix, _device_id, _date_day, _date_month, _date_year, _time_hour, _time_minute, _time_second, _lat, _lng, _altitude, _satellites, _hdop, _age, battery_percentage, battery_voltage, signal_strength);
    gsm_send_sms(sender, reply);

    Serial.print(F("Location query: "));
    Serial.println(reply);
  }
}

uint8_t gsm_send_sms(const char* destination, const char* text) {
  _gsm_serial.listen();
  _gsm_serial.print(F("AT+CMGS=\""));
  _gsm_serial.print(destination);
  _gsm_serial.println(F("\""));

  if (!gsm_send_command(0, 2000, ">")) {
    return 1;
  }

  _gsm_serial.print(text);
  _gsm_serial.write(0x1A);
  _gsm_serial.write(0x0D);
  _gsm_serial.write(0x0A);

  if (!gsm_send_command(0)) {
    return 2;
  }
}

void gps_init_values() {
  dtostrf(0, 4, 6, _lat);
  dtostrf(0, 4, 6, _lng);
  dtostrf(0, 4, 2, _speed);
  dtostrf(0, 4, 2, _course);
  dtostrf(0, 4, 2, _altitude);
  _date_day = 0;
  _date_month = 0;
  _date_year = 0;
  _time_hour = 0;
  _time_minute = 0;
  _time_second = 0;
  _satellites = 0;
  _hdop = 0;
  _age = 0;
  _fix = 0;
}

void gps_update_values() {
  if (_gps.location.isUpdated() && _gps.location.isValid()) {
    _age = _gps.location.age();
    dtostrf(_gps.location.lat(), 4, 6, _lat);
    dtostrf(_gps.location.lng(), 4, 6, _lng);
  }

  if (_gps.date.isUpdated() && _gps.date.isValid()) {
    _date_day = _gps.date.day();
    _date_month = _gps.date.month();
    _date_year = _gps.date.year();
  }

  if (_gps.time.isUpdated() && _gps.time.isValid()) {
    _time_hour = _gps.time.hour();
    _time_minute = _gps.time.minute();
    _time_second = _gps.time.second();
  }

  if (_gps.speed.isUpdated() && _gps.speed.isValid()) {
    dtostrf(_gps.speed.kmph(), 4, 2, _speed);
  }

  if (_gps.course.isUpdated() && _gps.course.isValid()) {
    dtostrf(_gps.course.deg(), 4, 2, _course);
  }

  if (_gps.altitude.isUpdated() && _gps.altitude.isValid()) {
    dtostrf(_gps.altitude.meters(), 4, 2, _altitude);
  }

  if (_gps.satellites.isUpdated() && _gps.satellites.isValid()) {
    _satellites = _gps.satellites.value();
  }

  if (_gps.hdop.isUpdated() && _gps.hdop.isValid()) {
    _hdop = _gps.hdop.value();
  }

  if (_gps_fix.isUpdated() && _gps_fix.isValid()) {
    _fix = atoi(_gps_fix.value());
  }
}

int8_t gsm_get_signal_quality() {
  gsm_send_command_P(F("AT+CSQ"));
  
  char* p = strstr(_gsm_serial_buffer, "CSQ: ");

  if (p) {
    p += 5;
    uint8_t i = 0;
    char buffer[4];

    while (*p != ',') {
      buffer[i++] = *(p++);
      if (i >= sizeof(buffer)) break;
    }

    int n = atoi(buffer);
  
    return n;
  }

  return 0;
}

void gsm_get_battery_level(uint8_t* percentage, uint16_t* voltage) {
  gsm_send_command_P(F("AT+CBC"));

  char *p = strstr(_gsm_serial_buffer, "CBC: ");

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

uint8_t gsm_send_post_request(const char* url, const char* params) {
  if (gsm_send_command_P(F("AT+SAPBR=1,1"), 10000)) {
    return 1;
  }

  if (gsm_send_command_P(F("AT+SAPBR=2,1"))) {
    return 2;
  }

  if (gsm_send_command_P(F("AT+HTTPINIT"))) {
    return 3;
  }

  if (gsm_send_command_P(F("AT+HTTPPARA=\"CID\",1"))) {
    return 4;
  }

  _gsm_serial.listen();
  _gsm_serial.print(F("AT+HTTPPARA=\"URL\",\""));
  _gsm_serial.print(url);
  _gsm_serial.println(F("\""));

  if (gsm_send_command(0)) {
    return 5;
  }

  if (gsm_send_command_P(F("AT+HTTPPARA=\"UA\",\"bekti-tracker\""))) {
    return 6;
  }

  if (gsm_send_command_P(F("AT+HTTPPARA=\"CONTENT\",\"application/x-www-form-urlencoded\""))) {
    return 7;
  }

  if (gsm_send_command_P(F("AT+HTTPPARA=\"TIMEOUT\",30"))) {
    return 8;
  }

  char params_buffer[25];
  uint16_t params_length = strlen(params);
  sprintf(params_buffer, "AT+HTTPDATA=%d,10000", params_length);

  if (gsm_send_command(params_buffer, 5000, "DOWNLOAD")) {
    return 9;
  }

  _gsm_serial.listen();
  _gsm_serial.print(params);

  if (gsm_send_command(0)) {
    return 10;
  }

  if (gsm_send_command_P(F("AT+HTTPACTION=1"), 45000, "+HTTPACTION:")) {
    return 11;
  }

  if (gsm_send_command_P(F("AT+HTTPTERM"))) {
    return 12;
  }

  if (gsm_send_command_P(F("AT+SAPBR=0,1"))) {
    return 13;
  }

  return 0;
}

void gsm_purge_serial_buffer() {
  while (_gsm_serial.available())  {
    _gsm_serial.read();
  }
}

uint8_t gsm_send_command(const char* command, uint16_t timeout, const char* expected) {
  _gsm_serial.listen();

  if (command) {
    gsm_purge_serial_buffer();
    _gsm_serial.println(command);
  }

  uint32_t t = millis();
  uint16_t n = 0;

  do {
    if (_gsm_serial.available()) {
      char c = _gsm_serial.read();
      //Serial.write(c);

      if (n >= sizeof(_gsm_serial_buffer) - 1) {
        n = sizeof(_gsm_serial_buffer) - 2;
        memcpy(_gsm_serial_buffer, _gsm_serial_buffer  + 1, n);
      }

      _gsm_serial_buffer[n++] = c;
      _gsm_serial_buffer[n] = 0;

      if (expected && strstr(_gsm_serial_buffer, expected)) {
        return 0;
      } else if (strstr_P(_gsm_serial_buffer, PSTR("OK\r"))) {
        return 0;
      } else if (strstr_P(_gsm_serial_buffer, PSTR("ERROR\r"))) {
        return 1;
      }
    }
  } while (millis() - t < timeout);

  return 2;
}

uint8_t gsm_send_command_P(const __FlashStringHelper* command, uint16_t timeout, const char* expected) {
  _gsm_serial.listen();

  if (command) {
    gsm_purge_serial_buffer();
    _gsm_serial.println(command);
  }

  uint32_t t = millis();
  uint16_t n = 0;

  do {
    if (_gsm_serial.available()) {
      char c = _gsm_serial.read();
      //Serial.write(c);

      if (n >= sizeof(_gsm_serial_buffer) - 1) {
        n = sizeof(_gsm_serial_buffer) - 2;
        memcpy(_gsm_serial_buffer, _gsm_serial_buffer  + 1, n);
      }

      _gsm_serial_buffer[n++] = c;
      _gsm_serial_buffer[n] = 0;

      if (expected && strstr(_gsm_serial_buffer, expected)) {
        return 0;
      } else if (strstr_P(_gsm_serial_buffer, PSTR("OK\r"))) {
        return 0;
      } else if (strstr_P(_gsm_serial_buffer, PSTR("ERROR\r"))) {
        return 1;
      }
    }
  } while (millis() - t < timeout);

  return 2;
}

void reboot() {
  digitalWrite(ARDUINO_RESET_PIN, LOW);
}

void setup() {
  digitalWrite(ARDUINO_RESET_PIN, HIGH);
  pinMode(ARDUINO_RESET_PIN, OUTPUT);
  
  Serial.begin(9600);
  _gps_serial.begin(9600);
  _gsm_serial.begin(9600);

  Serial.println(F("Bekti Tracker 0.1.0"));

  _device_id = eeprom_read_word(&_device_id_E);
  Serial.print(F("Device ID: "));
  Serial.println(_device_id);

  _interval = eeprom_read_word(&_interval_E) * 1000;
  Serial.print(F("Update interval: "));
  Serial.println(_interval);

  for (;;) {
    gsm_reset();
    uint8_t result = gsm_init();
    if (result == 0) break;
  }

  gps_init_values();
}

void loop() {
  _gps_serial.listen();

  while (_gps_serial.available()) {
    char c = _gps_serial.read();
    _gps.encode(c);
    gps_update_values();
  }

  if (millis() - _last_sms_check > 5000) {
    Serial.println(F("Check SMS"));
    gsm_check_sms();
    _last_sms_check = millis();
  }

  if (millis() - _last_upload > _interval) {
    uint8_t battery_percentage = 78;
    uint16_t battery_voltage = 4700;
    int8_t signal_strength = 20;
  
    char params_buffer[200];
    sprintf_P(params_buffer, _format_post, _device_id, _fix, _lat, _lng, _altitude, _speed, _course, _date_day, _date_month, _date_year, _time_hour, _time_minute, _time_second, _satellites, _hdop, _age, battery_percentage, battery_voltage, signal_strength);
    Serial.println(params_buffer);

    uint8_t result = gsm_send_post_request("seth.bekti.io/api/v1/location", params_buffer);

    if (result != 0) {
      ++_failure_count;
    } else {
      _failure_count = 0;
    }

    if (_failure_count >= 1) {
      reboot();
    }

    _last_upload = millis();
  }
}
