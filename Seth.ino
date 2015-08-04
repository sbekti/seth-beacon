#include <avr/eeprom.h>
#include <ctype.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

#define APN "internet"
#define ARDUINO_RESET_PIN 7
#define SIM800_RESET_PIN 8

SoftwareSerial _gps_serial(11, 12); // RX, TX
SoftwareSerial _gsm_serial(9, 10); // RX, TX
TinyGPSPlus _gps;

uint32_t _last_upload, _last_sms_check;
char _gsm_serial_buffer[256];
uint8_t _failure_count;

uint16_t _device_id;
uint16_t EEMEM _device_id_E;

uint32_t _interval;
uint16_t EEMEM _interval_E;

char _apn[25];
char EEMEM _apn_E[25];

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

void gps_update_values();
uint8_t gsm_init();
uint8_t gsm_setup();
uint8_t gsm_send_sms(const char* destination, const char* text);
void gsm_process_sms(uint16_t id, const char* sender, const char* text);
void gsm_check_sms();
int8_t gsm_get_signal_quality();
void gsm_get_battery_level(uint8_t* percentage, uint16_t* voltage);
uint8_t gsm_send_post_request(const char* url, const char* params);
void gsm_purge_serial_buffer();
uint8_t gsm_send_command(const char* cmd, uint16_t timeout = 5000, const char* expected = 0);
uint8_t gsm_send_command_P(const __FlashStringHelper* cmd, uint16_t timeout = 5000, const char* expected = 0);
void reboot();

const char _format_0[] PROGMEM = "id=%u&lat=%s&lng=%s&alt=%s&speed=%s&course=%s&date=%02u%02u%04u&time=%02u%02u%02u&sat=%lu&hdop=%lu&age=%lu&charge=%u&voltage=%u&signal=%d";
const char _format_1[] PROGMEM = "id=%u&charge=%u&voltage=%u&signal=%u";
const char _format_2[] PROGMEM = "#%u %02u/%02u/%04u %02u:%02u:%02u maps.google.com/maps?q=loc:%s,%s alt=%s sat=%lu hdop=%lu age=%lu chg=%u vtg=%u sig=%d";

void gps_update_values() {
  if (_gps.location.isUpdated()) {
    _age = _gps.location.age();
    dtostrf(_gps.location.lat(), 4, 6, _lat);
    dtostrf(_gps.location.lng(), 4, 6, _lng);
  }

  if (_gps.date.isUpdated()) {
    _date_day = _gps.date.day();
    _date_month = _gps.date.month();
    _date_year = _gps.date.year();
  }

  if (_gps.time.isUpdated()) {
    _time_hour = _gps.time.hour();
    _time_minute = _gps.time.minute();
    _time_second = _gps.time.second();
  }

  if (_gps.speed.isUpdated()) {
    dtostrf(_gps.speed.kmph(), 4, 2, _speed);
  }

  if (_gps.course.isUpdated()) {
    dtostrf(_gps.course.deg(), 4, 2, _course);
  }

  if (_gps.altitude.isUpdated()) {
    dtostrf(_gps.altitude.meters(), 4, 2, _altitude);
  }

  if (_gps.satellites.isUpdated()) {
    _satellites = _gps.satellites.value();
  }

  if (_gps.hdop.isUpdated()) {
    _hdop = _gps.hdop.value();
  }
}

uint8_t gsm_init() {
  pinMode(SIM800_RESET_PIN, OUTPUT);
  digitalWrite(SIM800_RESET_PIN, HIGH);
  delay(1000);
  digitalWrite(SIM800_RESET_PIN, LOW);
  delay(1000);
  digitalWrite(SIM800_RESET_PIN, HIGH);
  delay(3000);

  if (!gsm_send_command_P(F("AT"))) {
    return 1;
  }

  if (!gsm_send_command_P(F("AT+IPR=9600"))) {
    return 2;
  }

  if (!gsm_send_command_P(F("ATE0"))) {
    return 3;
  }

  if (!gsm_send_command_P(F("AT+CFUN=1"))) {
    return 4;
  }

  return 0;
}

uint8_t gsm_setup() {
  bool success = false;

  for (uint8_t n = 0; n < 30; n++) {
    if (gsm_send_command_P(F("AT+CREG?"))) {
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
    return 1;
  }

  if (!gsm_send_command_P(F("AT+CMGF=1"))) {
    return 2;
  }

  if (!gsm_send_command_P(F("AT+CPMS=\"SM\",\"SM\",\"SM\""))) {
    return 3;
  }

  gsm_check_sms();

  eeprom_read_block((void*)&_apn, (const void*)&_apn_E, sizeof(_apn) - 1);
  Serial.print(F("Using APN: "));
  Serial.println(_apn);

  if (!gsm_send_command_P(F("AT+CGATT?"))) {
    return 4;
  }

  if (!gsm_send_command_P(F("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\""))) {
    return 5;
  }

  _gsm_serial.listen();
  _gsm_serial.print(F("AT+SAPBR=3,1,\"APN\",\""));
  _gsm_serial.print(_apn);
  _gsm_serial.println(F("\""));

  if (!gsm_send_command(0)) {
    return 6;
  }

  if (!gsm_send_command_P(F("ATS0=1"))) {
    return 7;
  }

  return 0;
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
    sprintf_P(reply, _format_2, _device_id, _date_day, _date_month, _date_year, _time_hour, _time_minute, _time_second, _lat, _lng, _altitude, _satellites, _hdop, _age, battery_percentage, battery_voltage, signal_strength);
    gsm_send_sms(sender, reply);

    Serial.print(F("Location query: "));
    Serial.println(reply);
  }
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
  };
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
  if (!gsm_send_command_P(F("AT+SAPBR=1,1"), 10000)) {
    return 1;
  }

  if (!gsm_send_command_P(F("AT+SAPBR=2,1"))) {
    return 2;
  }

  if (!gsm_send_command_P(F("AT+HTTPINIT"))) {
    return 3;
  }

  if (!gsm_send_command_P(F("AT+HTTPPARA=\"CID\",1"))) {
    return 4;
  }

  _gsm_serial.listen();
  _gsm_serial.print(F("AT+HTTPPARA=\"URL\",\""));
  _gsm_serial.print(url);
  _gsm_serial.println(F("\""));

  if (!gsm_send_command(0)) {
    return 5;
  }

  if (!gsm_send_command_P(F("AT+HTTPPARA=\"UA\",\"bekti-tracker\""))) {
    return 6;
  }

  if (!gsm_send_command_P(F("AT+HTTPPARA=\"CONTENT\",\"application/x-www-form-urlencoded\""))) {
    return 7;
  }

  if (!gsm_send_command_P(F("AT+HTTPPARA=\"TIMEOUT\",30"))) {
    return 8;
  }

  char params_buffer[25];
  uint16_t params_length = strlen(params);
  sprintf(params_buffer, "AT+HTTPDATA=%d,10000", params_length);

  if (!gsm_send_command(params_buffer, 5000, "DOWNLOAD")) {
    return 9;
  }

  _gsm_serial.listen();
  _gsm_serial.print(params);

  if (!gsm_send_command(0)) {
    return 10;
  }

  if (!gsm_send_command_P(F("AT+HTTPACTION=1"), 45000, "+HTTPACTION:")) {
    return 11;
  }

  if (!gsm_send_command_P(F("AT+HTTPTERM"))) {
    return 12;
  }

  if (!gsm_send_command_P(F("AT+SAPBR=0,1"))) {
    return 13;
  }

  return 0;
}

void gsm_purge_serial_buffer() {
  while (_gsm_serial.available())  {
    _gsm_serial.read();
  }
}

uint8_t gsm_send_command(const char* cmd, uint16_t timeout, const char* expected) {
  _gsm_serial.listen();

  if (cmd) {
    gsm_purge_serial_buffer();
    _gsm_serial.println(cmd);
  }

  uint32_t t = millis();
  uint8_t n = 0;

  do {
    if (_gsm_serial.available()) {
      char c = _gsm_serial.read();
      Serial.write(c);

      if (n >= sizeof(_gsm_serial_buffer) - 1) {
        // buffer full, discard first half
        n = sizeof(_gsm_serial_buffer) / 2 - 1;
        memcpy(_gsm_serial_buffer, _gsm_serial_buffer + sizeof(_gsm_serial_buffer) / 2, n);
      }

      _gsm_serial_buffer[n++] = c;
      _gsm_serial_buffer[n] = 0;

      if (strstr(_gsm_serial_buffer, expected ? expected : "OK\r")) {
        return n;
      }
    }
  } while (millis() - t < timeout);

  return 0;
}

uint8_t gsm_send_command_P(const __FlashStringHelper* cmd, uint16_t timeout, const char* expected) {
  _gsm_serial.listen();

  if (cmd) {
    gsm_purge_serial_buffer();
    _gsm_serial.println(cmd);
  }

  uint32_t t = millis();
  uint8_t n = 0;

  do {
    if (_gsm_serial.available()) {
      char c = _gsm_serial.read();
      Serial.write(c);

      if (n >= sizeof(_gsm_serial_buffer) - 1) {
        // buffer full, discard first half
        n = sizeof(_gsm_serial_buffer) / 2 - 1;
        memcpy(_gsm_serial_buffer, _gsm_serial_buffer + sizeof(_gsm_serial_buffer) / 2, n);
      }

      _gsm_serial_buffer[n++] = c;
      _gsm_serial_buffer[n] = 0;

      if (strstr(_gsm_serial_buffer, expected ? expected : "OK\r")) {
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

  Serial.println(F("Bekti Tracker 0.1.0"));

  _device_id = eeprom_read_word(&_device_id_E);
  Serial.print(F("Device ID: "));
  Serial.println(_device_id);

  _interval = eeprom_read_word(&_interval_E) * 1000;
  Serial.print(F("Update interval: "));
  Serial.println(_interval);

  Serial.begin(9600);
  _gps_serial.begin(9600);
  _gsm_serial.begin(9600);

  for (;;) {
    Serial.println(F("Initializing GSM... "));
    uint8_t ret = gsm_init();

    if (ret == 0) {
      Serial.println(F("OK"));
      break;
    }

    Serial.print(F("Error code: "));
    Serial.println(ret);
  }

  for (;;) {
    Serial.println(F("Setting up network... "));
    uint8_t ret = gsm_setup();

    if (ret == 0) {
      Serial.println(F("OK"));
      break;
    }

    Serial.print(F("Error code: "));
    Serial.println(ret);
  }
}

void loop() {
  _gps_serial.listen();

  while (_gps_serial.available()) {
    _gps.encode(_gps_serial.read());
    gps_update_values();
  }

  if (millis() - _last_sms_check > 5000) {
    Serial.print(F("Check SMS"));
    gsm_check_sms();
    _last_sms_check = millis();
  }

  if (millis() - _last_upload > _interval) {
    Serial.print(F("Interval "));
    Serial.println(_interval);

    gsm_check_sms();

    uint8_t battery_percentage;
    uint16_t battery_voltage;
    gsm_get_battery_level(&battery_percentage, &battery_voltage);
    int8_t signal_strength = gsm_get_signal_quality();
    char params_buffer[200];
    if (_gps.location.isValid()) {
      sprintf_P(params_buffer, _format_0, _device_id, _lat, _lng, _altitude, _speed, _course, _date_day, _date_month, _date_year, _time_hour, _time_minute, _time_second, _satellites, _hdop, _age, battery_percentage, battery_voltage, signal_strength);
    } else {
      sprintf_P(params_buffer, _format_1, _device_id, battery_percentage, battery_voltage, signal_strength);
    }

    uint8_t ret = gsm_send_post_request("seth.bekti.io/api/v1/location", params_buffer);

    if (ret != 0) {
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
