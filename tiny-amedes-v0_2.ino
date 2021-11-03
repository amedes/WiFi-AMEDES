/*
 * ESP-WROOM-02 + BME280 + AS3935
 * TINY-AMEDES
 */

#include "FS.h"
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

extern "C" {
#include "user_interface.h"
}

//#define USE_XIVELY 1
#define USE_SYSLOG 1
#define USE_NTP 1

#if defined(USE_SYSLOG) || defined(USE_NTP)
#include <WiFiUDP.h>
#endif

#ifdef USE_XIVELY
//#include <b64.h>
//#include <HttpClient.h>

//#include <CountingStream.h>
//#include <Xively.h>
//#include <XivelyClient.h>
//#include <XivelyDatastream.h>
//#include <XivelyFeed.h>
//#include <Xively.h>

#define XIVELY_TINY "tiny3639"
#define FEED_ID XXXXXXXX
#define FEED_ID_STR "XXXXXXXX"
#define XIVELY_KEY "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"
char xivelyKey[] = XIVELY_KEY;
#endif

ESP8266WebServer server(80);

#ifdef USE_SYSLOG
WiFiUDP udp;
#endif

#ifdef USE_NTP
#include <Time.h>
#define NTP_PORT 123
#endif

#define BME280_0 0x76
#define BME280_1 0x77
#define BME280_DIG_LEN 26
#define BME280_ID 0x60

#define I2C_SDA 5  // IO5
#define I2C_SCL 4  // IO4
#define LED_PIN 12 // IO12
#define SW_PIN  13 // IO13

#define TINY_CONF "/tiny.conf"

#define DEBUG 1
#define USE_RTC_RAM 1
#define USE_LCD 1

#ifdef USE_RTC_RAM
#define RTC_MAGIC 0x55aaaa55
#define RTC_RAM_START 65
#endif

ADC_MODE(ADC_VCC);

// for AP mode
String ap_ssid = "TINY";
String ap_password = "tinyamedes";

#define DEFAULT_INTERVAL 600 // second

// WiFi client
String sta_ssid;
String sta_password;
// Location info.
String location;
float height;
// irc info.
String nickname;
int interval = DEFAULT_INTERVAL;

int vcc_val;
uint16_t dig_T1;
int16_t  dig_T2;
int16_t  dig_T3;

uint16_t dig_P1;
int16_t  dig_P2;
int16_t  dig_P3;
int16_t  dig_P4;
int16_t  dig_P5;
int16_t  dig_P6;
int16_t  dig_P7;
int16_t  dig_P8;
int16_t  dig_P9;

uint8_t  dig_H1;
int16_t  dig_H2;
uint8_t  dig_H3;
int16_t  dig_H4;
int16_t  dig_H5;
int8_t   dig_H6;

uint32_t adc_P;
uint32_t adc_T;
uint32_t adc_H;

#define IRC_SERVER "irc.ircnet.ne.jp"
#define IRC_CHANNEL "#tini-ja"

const int ircPort = 6660;
const char ircHost[] = IRC_SERVER;
String ircChannel = IRC_CHANNEL;

#ifdef USE_SYSLOG
int useSyslog = 0;
String syslogServer = "192.168.0.1";
#endif

int bme280_0 = 0;
int bme280_1 = 0;

int16_t stts751_temperature = 0x8000;

int lightning_energy;
int lightning_distance;
int lightning_detect = 0;
int lightning_wdth;
int lightning_nf_lev;
int lightning_cap;

uint32_t startMillis;
uint32_t startMicros;

#define BME280_RESET 0xe0 // reset reg. addr
#define BME280_RESET_MAGIC 0xb6 // reset word
#define BME280_CTRL_HUM 0xf2
#define BME280_STATUS 0xf3
#define BME280_CTRL_MEAS 0xf4
#define BME280_CONFIG 0xf5

#define INT_NH 0x01 // Noise level too high
#define INT_D  0x04 // Disturber detected
#define INT_L  0x08 // Lightning interrupt
#define AS3935_INIT 0x1000 // Lightning Sensor initialized

// STTS751 I2C addresses
uint8_t stts751_addrs[] = {
  0x48, 0x49, 0x38, 0x39, 0x4a, 0x4b, 0x3a, 0x3b
};

#define STTS751_N (sizeof(stts751_addrs) / sizeof(uint8_t))

int bme280_exist(int bme280)
{
  // read BME280 ID check
  uint8_t id;
  int c;

  c = readmulti(bme280, 0xd0, &id, 1);
  if (c == 1 && id == BME280_ID) {
    return 1;
  }
  return 0;
}

int bme280_busy(int bme280)
{
  int c;
  uint8_t s;

  c = readmulti(bme280, BME280_STATUS, &s, 1);
  if (c == 1) {
    return s & 0x09;
  }
  return 0;
}

#define BME280_MEAS_TEMP 1
#define BME280_MEAS_PRES 2
#define BME280_MEAS_HUMI 4

#define BME280_MEAS_DATA_LEN 8

#define BME280_OVERSAMPLE 5 // 16 samples per measure

#define BME280_MODE_FORCE 0x01 // force mode

#define BME280_CALIB0 0x88
#define BME280_CALIB26 0xe1
#define BEM280_CALIB26_LEN 7

int bme280_measurement(int bme280) // parameter is I2C address 0x76 or 0x77
{
  uint8_t meas_temp = 0;
  uint8_t meas_pres = 0;
  uint8_t meas_humi = 0;
  int i;
  uint8_t dig[BME280_DIG_LEN];
  int c;

  write_reg(bme280, BME280_RESET, BME280_RESET_MAGIC); // reset BME280
  while (bme280_busy(bme280)) {
    Serial.print("*");
    delay(10);
  }
  Serial.println();

  // read caliblation data from BME280
  c = readmulti(bme280, BME280_CALIB0, dig, BME280_DIG_LEN);

  i = 0;
  dig_T1 = make_short(&dig[i]); i += 2;
  dig_T2 = make_short(&dig[i]); i += 2;
  dig_T3 = make_short(&dig[i]); i += 2;

  Serial.println();
  Serial.print("dig_T1:"); Serial.println(dig_T1);
  Serial.print("dig_T2:"); Serial.println(dig_T2);
  Serial.print("dig_T3:"); Serial.println(dig_T3);

  dig_P1 = make_short(&dig[i]); i += 2;
  dig_P2 = make_short(&dig[i]); i += 2;
  dig_P3 = make_short(&dig[i]); i += 2;
  dig_P4 = make_short(&dig[i]); i += 2;
  dig_P5 = make_short(&dig[i]); i += 2;
  dig_P6 = make_short(&dig[i]); i += 2;
  dig_P7 = make_short(&dig[i]); i += 2;
  dig_P8 = make_short(&dig[i]); i += 2;
  dig_P9 = make_short(&dig[i]); i += 3; // skip 0xa0;

  Serial.println();
  Serial.print("dig_P1:"); Serial.println(dig_P1);
  Serial.print("dig_P2:"); Serial.println(dig_P2);
  Serial.print("dig_P3:"); Serial.println(dig_P3);
  Serial.print("dig_P4:"); Serial.println(dig_P4);
  Serial.print("dig_P5:"); Serial.println(dig_P5);
  Serial.print("dig_P6:"); Serial.println(dig_P6);
  Serial.print("dig_P7:"); Serial.println(dig_P7);
  Serial.print("dig_P8:"); Serial.println(dig_P8);
  Serial.print("dig_P9:"); Serial.println(dig_P9);

  dig_H1 = dig[i];

  c = readmulti(bme280, BME280_CALIB26, dig, BEM280_CALIB26_LEN);

  i = 0;
  dig_H2 = make_short(&dig[i]); i += 2;
  dig_H3 = dig[i]; i++;
  dig_H4 = (dig[i] << 4) | (dig[i + 1] & 0x0f); i++; // 0xE4 / 0xE5[3:0]
  dig_H5 = (dig[i] >> 4) | (dig[i + 1] << 4); i += 2;
  dig_H6 = dig[i];

  Serial.println();
  Serial.print("dig_H1:"); Serial.println(dig_H1);
  Serial.print("dig_H2:"); Serial.println(dig_H2);
  Serial.print("dig_H3:"); Serial.println(dig_H3);
  Serial.print("dig_H4:"); Serial.println(dig_H4);
  Serial.print("dig_H5:"); Serial.println(dig_H5);
  Serial.print("dig_H6:"); Serial.println(dig_H6);

  uint8_t buf[BME280_MEAS_DATA_LEN];

  write_reg(bme280, BME280_CTRL_HUM, BME280_OVERSAMPLE); // humidity to be measured
  uint8_t cmd = (BME280_OVERSAMPLE << 5) | (BME280_OVERSAMPLE << 2) | BME280_MODE_FORCE; // temp and press
#define BME280_MEAS_CMD 0xb5 // 16 oversampling for temp, press and humi
  write_reg(bme280, BME280_CTRL_MEAS, BME280_MEAS_CMD); // temp and pressure are measured with forced mode

  Serial.print(millis() - startMillis);
  Serial.println("ms");
  //delay(10);

  // wait for measument completion
  while (bme280_busy(bme280)) {
    Serial.print(millis() - startMillis);
    Serial.print(", ");
    //Serial.print("*");
    delay(10);
  }
  Serial.println();
  Serial.print(millis() - startMillis);
  Serial.println("ms");

#define BME280_MEAS_DATA 0xf7

  i = readmulti(bme280, BME280_MEAS_DATA, buf, BME280_MEAS_DATA_LEN);
  if (i != BME280_MEAS_DATA_LEN) {
    Serial.println("BME280 measument error");
    return 0;
  }

  i = 0;
  adc_P = make_uint20(&buf[i]); i += 3;
  adc_T = make_uint20(&buf[i]); i += 3;
  adc_H = make_uint16(&buf[i]);

  Serial.print("adc_T:");
  Serial.println(adc_T);

  Serial.print("adc_P:");
  Serial.println(adc_P);

  Serial.print("adc_H:");
  Serial.println(adc_H);

  return 1;
}

#ifdef USE_XIVELY

#define CRLF "\r\n"
#define HTTP_VERSION "HTTP/1.1"
#define XIVELY_API_METHOD "PUT "
#define XIVELY_API_HOST "api.xively.com"
#define XIVELY_API_PATH "/v2/feeds/"
#define XIVELY_API_EXT ".csv"
#define XIVELY_API_KEY "X-ApiKey:"
#define HTTP_HOST "Host:"
#define HTTP_PORT 80
#define HTTP_LENGTH "Content-Length:"
#define HTTP_CONNECTION_CLOSE "Connection: close"

#define CHANNELID_0 "temperature"
#define CHANNELID_1 "humidity"
#define CHANNELID_2 "pressure"
#define CHANNELID_3 "battery"
#define CHANNELID_4 "RSSI"
#define CHANNELID_5 "uptime"
#define CHANNELID_6 "LightningDistance"
#define CHANNELID_7 "LightningEnergy"

int sendXively(float temp, float humi, float pres, float batt, int rssi, int uptime)
{
  String header =
    XIVELY_API_METHOD XIVELY_API_PATH FEED_ID_STR XIVELY_API_EXT " " HTTP_VERSION
    CRLF
    HTTP_HOST XIVELY_API_HOST
    CRLF
    XIVELY_API_KEY XIVELY_KEY
    CRLF
    HTTP_CONNECTION_CLOSE
    CRLF;
  String body;

  body  = CHANNELID_0 "," + String(temp, 1) + CRLF;
  body += CHANNELID_1 "," + String(humi, 1) + CRLF;
  body += CHANNELID_2 "," + String(pres, 1) + CRLF;
  body += CHANNELID_3 "," + String(batt, 3) + CRLF;
  body += CHANNELID_4 "," + String(rssi) + CRLF;
  if (uptime) body += CHANNELID_5 "," + String(uptime) + CRLF;

  // lightning
  if (lightning_detect == INT_L) {
    body += CHANNELID_5 "," + String(lightning_distance) + CRLF;
    body += CHANNELID_6 "," + String(lightning_energy) + CRLF;
  }

  header += HTTP_LENGTH + String(body.length()) + CRLF CRLF;

  int status = -1;
  WiFiClient client;
  if (client.connect(XIVELY_API_HOST, HTTP_PORT)) {
    Serial.println("connected to Xively");

    // send http message
    client.print(header + body);
    Serial.println(header + body);

    int timeout = millis() + 1000; // 1 sec
    while (millis() < timeout) {
      while (client.available()) {
        String s = client.readStringUntil('\n');
        Serial.println(s);
        if (s.startsWith(HTTP_VERSION)) {
          status = s.substring(sizeof(HTTP_VERSION)).toInt();
        }
      }
      if (status > 0) break;
      delay(10);
    }
    client.stop();
    delay(100);
  }
  return status;
}
#if 0
int sendXively(float temp, float humi, float pres)
{
  struct MY_CHANNELS {
    char *id;
    int type;
  } myChannels[] = {
    { (char *)"temperature", DATASTREAM_FLOAT },
    { (char *)"humidity",    DATASTREAM_FLOAT },
    { (char *)"pressure",    DATASTREAM_FLOAT }
  };

  char channelId0[] = "temperature";
  char channelId1[] = "humidity";
  char channelId2[] = "pressure";

#define NUM_STREAMS (sizeof(myChannels) / sizeof(myChannels[0]))

  XivelyDatastream datastreams[NUM_STREAMS] = {
    //    XivelyDatastream(channelId0, strlen(channelId0), DATASTREAM_FLOAT),
    //    XivelyDatastream(channelId1, strlen(channelId1), DATASTREAM_FLOAT),
    //    XivelyDatastream(channelId2, strlen(channelId2), DATASTREAM_FLOAT)
    XivelyDatastream(myChannels[0].id, strlen(myChannels[0].id), myChannels[0].type),
    XivelyDatastream(myChannels[1].id, strlen(myChannels[1].id), myChannels[1].type),
    XivelyDatastream(myChannels[2].id, strlen(myChannels[2].id), myChannels[2].type)
  };

  XivelyFeed feed(FEED_ID, datastreams, NUM_STREAMS);

  WiFiClient client;
  XivelyClient xivelyclient(client);

  datastreams[0].setFloat(temp);
  datastreams[1].setFloat(humi);
  datastreams[2].setFloat(pres);

  return xivelyclient.put(feed, xivelyKey);
}
#endif
#endif

uint32_t ChipId;
char tinyname[9] = "tiny";


#ifdef USE_RTC_RAM
uint32_t last_uptime = 0;
#endif

void setup(void) {
  // record start time
  startMicros = micros();
  startMillis = startMicros / 1000;

  // initialize GPIO for LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); // LED off

  // read Vcc voltage
  vcc_val = ESP.getVcc();

  ChipId = ESP.getChipId();
  strcat(tinyname, (String(ChipId % 10000 + 10000).substring(1)).c_str());

  // I2C initialize
  Wire.begin(I2C_SDA, I2C_SCL); // SDA:IO4, CLK,IO5;
  Serial.begin(115200, SERIAL_8N1);
  delay(10);

  // output some debug info
  //  WiFi.printDiag(Serial);
  //  Serial.setDebugOutput(true);

  Serial.println();

  // system time
  Serial.print("system_get_time() = ");
  Serial.print(system_get_time());
  Serial.println(" us");

  int rtc_time;
  int rtc_cali;
  // rtc time
  Serial.print("system_get_rtc_time() = ");
  Serial.println(rtc_time = system_get_rtc_time());

  // rtc calibration
  Serial.print("system_rtc_clock_cali_proc() = ");
  Serial.println(rtc_cali = system_rtc_clock_cali_proc());
  Serial.print("rtc time = ");
  Serial.println((long)rtc_time * rtc_cali >> 12);

  Serial.print("wifi_station_get_connect_status() = ");
  Serial.println(wifi_station_get_connect_status());

#ifdef USE_RTC_RAM
  // read RTC RAM
  uint32_t rtc_ram[2];
  if (system_rtc_mem_read(RTC_RAM_START, rtc_ram, sizeof(rtc_ram))) {
    Serial.print("rtc_ram[0] = ");
    Serial.println(rtc_ram[0], HEX);
    Serial.print("rtc_ram[1] = ");
    Serial.println(rtc_ram[1]);

    if (rtc_ram[0] == RTC_MAGIC) {
      last_uptime = rtc_ram[1];
      if (vcc_val > (3 * 1024)) { // if Vcc > 3.0V
        // LED on if powered by USB
        digitalWrite(LED_PIN, LOW);
      }
    } else {
      // LED on if first time
      digitalWrite(LED_PIN, LOW);
      Serial.println("rtc magic is invalid");
    }
  } else {
    Serial.println("system_rtc_mem_read() fail!");
  }
#endif

  // Switch sense
  pinMode(SW_PIN, INPUT_PULLUP);

  Serial.println();
  Serial.println("Started.");
  Serial.print(millis() - startMillis);
  Serial.println(" ms");

  // Lightning Sensor
  lightning();

  // mount Flash File System
  if (SPIFFS.begin()) {
    Serial.println("SPIFFS mount success");
  } else if (SPIFFS.format()) {
    Serial.println("SPIFFS format success");
  } else {
    Serial.println("SPIFFS mount fail");
    return;
  }

  // Is switch on ?
  if (digitalRead(SW_PIN) == LOW) {
    Serial.println("SW is ON");
    // go config mode
    config();
  } else {
    amedes();
  }
}

// pressure compansation
float press_comp(float pres, float temp, float height)
{
  const float g = 9.8;
  const float R = 287.05;
  float Tvm;
  float tm = temp + 0.0025 * height;
  float em;
  struct PC_CONST {
    float T;
    float A;
    float B;
    float C;
  } pc[] = {
    { -30.0,  0,         0,       0.090 },
    {   0.0,  0.000489,  0.0300,  0.550 },
    {  20.0,  0.002850,  0.0165,  0.550 },
    {  33.8, -0.006933,  0.4687, -4.580 },
    {  99.0,  0,         0,       3.340 }
  }, *ppc;

  for (ppc = &pc[0]; ppc < &pc[sizeof(pc) / sizeof(pc[0]) - 1]; ppc++) {
    if (tm < ppc->T) break;
  }

  em = ppc->A * tm * tm + ppc->B * tm + ppc->C;
  Tvm = 273.15 + tm + em;

  return pres * exp(g * height / (R * Tvm));
}

#ifdef USE_NTP
uint32 micro2fixp(int micro)
{
  uint32 f = 1 << 31; // 0.5 sec
  uint32 fix = 0;
  int m = 500000; // 0.5 sec
  int i;

  while (m > 0) {
    if (micro > m) {
      micro -= m;
      fix += f;
    }
    m >>= 1;
    f >>= 1;
  }

  return fix; // fixed point fraction
}
#endif

// TINY-AMEDES main
void amedes(void)
{
  int32_t temp;
  uint32_t pres;
  int32_t humi;
  int i;
  int next_state;
  float p0;

  // read config file
  read_conf_file();

  Serial.print(millis() - startMillis);
  Serial.println(" ms");

  // set station mode
  if (wifi_get_opmode() != WIFI_STA) {
    WiFi.mode(WIFI_STA);
    Serial.println("WiFi.mode(WIFI_STA)");
  }
  Serial.print(millis() - startMillis);
  Serial.println(" ms");

  struct station_config stationConf;
  int status;
  if (!wifi_station_get_config_default(&stationConf)
      || strcmp((char *)stationConf.ssid, sta_ssid.c_str())
      || strcmp((char *)stationConf.password, sta_password.c_str())
      || wifi_station_get_auto_connect() == 0) {
    status = WiFi.begin(sta_ssid.c_str(), sta_password.c_str());
    Serial.print("WiFi.begin(");
    Serial.print(sta_ssid);
    Serial.print("): ");
    Serial.println(status);
  }

  Serial.print(millis() - startMillis);
  Serial.println(" ms");

  // measure temp by stts751
  stts751();

  if (bme280_exist(BME280_0)) {
    Serial.println("BME280_0 found!");
    bme280_0 = 1;
  }
  if (bme280_exist(BME280_1)) {
    Serial.println("BME280_1 found!");
    bme280_1 = 1;
  }
  if (!bme280_0 && !bme280_1 && (uint16_t)stts751_temperature == 0x8000) {
    Serial.print("No BME280 and STTS751 exist");
    //return;
  }

  // irc message
  String head;
  String msg;
  String tail;

  head =
    "NICK " + nickname + "\r\n" // Nick name
    "USER " + tinyname + " 0 * :TINY-AMEDES\r\n"
    "PRIVMSG " + ircChannel + " :";

  // do measurement

#define power(x, y) exp(y * log(x))
  //#define power(x, y) pow(y, y)

  msg = location + " Weather:";

#define LOC_N 2
  String loc[LOC_N] = { "Outside", "Room" };
  // Outside and Room temp.
  for (int i = 0; i < LOC_N; i++) {
    if (!bme280_exist(BME280_0 + i)) continue;

    if (bme280_measurement(BME280_0 + i) == 0) continue;

    // measurement is not done
    if (adc_T == 0x80000 && adc_P == 0x80000 && adc_H == 0x8000) continue;

    // temperatur = temp / 100 C
    temp = BME280_compensate_T_int32(adc_T); // must be first
    // pressure = pres / 256 Pa
    pres = BME280_compensate_P_int64(adc_P);
    // humidity = humi / 1024 %rH
    humi = BME280_compensate_H_int32(adc_H);

    // pressure compansation
    //p0 = pres / 25600.0 * power(1.0 +  0.65 * height / (temp + 27315.0), 5.257);
    p0 = press_comp(pres / 25600.0, temp / 100.0, height);
    Serial.print("p:");
    Serial.println(pres / 25600.0);
    Serial.print("p0:");
    Serial.println(p0);

    msg += " " + loc[i] + ":" + String(temp / 100.0, 1) + "C "
           + " Atm:" + String(p0, 1) + "hPa "
           + " Humidity(" + loc[i] + "):" + String(humi / 1024.0, 1) + "% ";
  }

  // STTS751
  int n = 0;
  for (i = 0; i < STTS751_N; i++) {
    uint8_t addr = stts751_addrs[i];

    if (stts751_exist(addr)) {
      int16_t temp = stts751_temp(addr);

      msg += " Room";
      if (n > 0) {
        msg += String(n);
      }
      msg += ":" + String(temp / 256.0, 1) + "C ";
      n++;
    }
  }

  // Data send to internet
  Serial.println();
  Serial.print(millis() - startMillis);
  Serial.println(" ms");

  // Wait for WiFi connection
  int old_status = -1;
  Serial.println("Wait WiFi connection");
  int cli = millis() + 10 * 1000; // wait 10 sec
  while ((status = WiFi.status()) != WL_CONNECTED && millis() < timeout) {
    if (status != old_status) {
      Serial.print("WiFi.status() = ");
      Serial.println(status);
      old_status = status;
    }
    delay(10);
  }

  Serial.print(millis() - startMillis);
  Serial.println(" ms");

  if (status != WL_CONNECTED) {
    Serial.println("time out!");
    return;
  }

  // send NTP request
#ifdef USE_NTP
#define NTP_PORT 123
#define NTP_HOST "ntp.nict.jp"
#define NTP_MSG_SIZE (12 * 4)
#define NTP_LI 0
#define NTP_VN 4
#define NTP_MODE 3

  WiFiUDP ntp;
  String ntp_host = NTP_HOST;
  byte ntp_msg[NTP_MSG_SIZE];
  time_t ntp_time_int;

  if (ntp.begin(NTP_PORT)) {
    if (ntp.beginPacket(ntp_host.c_str(), NTP_PORT)) {

      memset(ntp_msg, 0, NTP_MSG_SIZE);

      ntp_msg[0] = (NTP_LI << 6) | (NTP_VN << 3) | NTP_MODE; // LI, Version, Mode
      *(uint32 *)&ntp_msg[40] = micros(); // Transmit Timestamp

      ntp.write(ntp_msg, NTP_MSG_SIZE);
      ntp.endPacket();
    } else {
      Serial.println("ntp.beginPacket() failed!");
    }
  } else {
    Serial.println("ntp.begin() failed!");
  }
#endif

  Serial.println("WiFi connected");
  Serial.print("Encryption type:");
  Serial.println(WiFi.encryptionType(0));
  Serial.print("RSSI:");
  int rssi = WiFi.RSSI();
  Serial.println(rssi);
  Serial.print("local IP: ");
  Serial.println(WiFi.localIP());

#ifdef USE_LCD
  // LCD reset
  digitalWrite(LED_PIN, LOW);
  delayMicroseconds(100);
  digitalWrite(LED_PIN, HIGH);
  
  lcd_init();
  lcd_out("RSSI");
  lcd_cr();
  lcd_out((String(rssi) + "dBm").c_str());
#endif

  // Lightning interrupt
  switch (lightning_detect) {
    case INT_NH:
      msg += " LightningNoise:" + String(lightning_nf_lev) + " ";
      break;
    case INT_D:
      msg += " LightningDisturber:" + String(lightning_wdth) + " ";
      break;
    case INT_L:
      msg += " LightningDistance:" + String(lightning_distance) + "km "
             + " LightningEnergy:" + String(lightning_energy) + " ";
      break;
    case AS3935_INIT:
      msg += " LightningCapacitors:" + String(lightning_cap) + " ";
      break;
  }

  // add Vcc and RSSI
  msg += " Vcc:" + String(vcc_val / 1024.0, 3) + "V "
         " RSSI:" + String(rssi) + "dBm ";

  // add last uptime stored in RTC RAM
#ifdef USE_RTC_RAM
  if (last_uptime > 0) {
    msg += " uptime:" + String(last_uptime / 1000) + "ms ";
  }
#else
  msg += " uptime:" + String(millis() - startMillis) + "ms ";
#endif

  // add "QUIT"
  tail = "\r\nQUIT\r\n";

  // send message to syslog server
#ifdef USE_SYSLOG

#define SYSLOG_PORT 514
#define SYSLOG_HOST "192.168.5.32"
#define SYSLOG_PRI "<190>" // local7(23) * 8 + Informational(6)

  if (useSyslog) {
    const char *s;
    int len;
    String syslog_msg = String(SYSLOG_PRI) + msg;

    if (!udp.begin(SYSLOG_PORT)) {
      Serial.println("udp.begin failed");
    }
    if (!udp.beginPacket(syslogServer.c_str(), SYSLOG_PORT)) {
      Serial.println("udp.beginPacket failed");
    }
    syslog_msg += "ts:" + String(millis()) + "ms";
    s = syslog_msg.c_str();
    len = strlen(s);
    if (udp.write(s, len) != len) {
      Serial.println("udp.write failed");
    }
    Serial.print("syslog:");
    Serial.println(s);
    if (!udp.endPacket()) {
      Serial.println("udp.endPacket failed");
    }
    udp.stop();
  }
#endif

  WiFiClient client;
  randomSeed(ESP.getCycleCount()); // connect irc server randowm port
  if (client.connect(ircHost, random(ircPort, ircPort + 10))) {
    Serial.println("connected to irc server");

#ifdef USE_NTP
    String timestamp = " time:";

    if (ntp.parsePacket()) {
      ntp.read(ntp_msg, NTP_MSG_SIZE);
      Serial.print("NTP RTT: ");
      Serial.println(micros() - * (uint32 *)&ntp_msg[24]);

      ntp_time_int = 0;
      for (i = 0; i < 4; i++) {
        ntp_time_int *= 0x100;
        ntp_time_int += ntp_msg[40 + i];
      }
      Serial.print("NTP sec:");
      Serial.println(ntp_time_int);

#define NTP_UNIX_DIFF 2208988800UL

      ntp_time_int -= NTP_UNIX_DIFF;
      ntp_time_int += 9 * 60 * 60; // +9:00

      Serial.print("NTP UNIX sec:");
      Serial.println(ntp_time_int);

      setTime(ntp_time_int);

      timestamp += String(year()) + "-";
      if (month() < 10) timestamp += "0";
      timestamp += String(month()) + "-";
      if (day() < 10) timestamp += "0";
      timestamp += String(day()) + "T";

      if (hour() < 10) timestamp += "0";
      timestamp += String(hour()) + ":";
      if (minute() < 10) timestamp += "0";
      timestamp += String(minute()) + ":";
      if (second() < 10) timestamp += "0";
      timestamp += String(second()) + "+09:00 ";

      Serial.print("NTP ");
      Serial.println(timestamp);

#ifdef USE_LCD
    {
       char buf[9];
       
      lcd_clr();
      sprintf(buf, "%02d/%02d/%02d", year() % 100, month(), day());
      lcd_out(buf);
      lcd_cr();
      sprintf(buf, "%02d:%02d:%02d", hour(), minute(), second());
       lcd_out(buf);
    }
#endif

    }
    ntp.stop();

    // sending message
    client.print(head + msg + timestamp + tail);
    Serial.println(head + msg + timestamp + tail);

#else // USE_NTP
    client.print(head + msg + tail);
    Serial.println(head + msg + tail);
#endif

#define WAIT_ERROR 1

#ifdef WAIT_ERROR
    // wait for "ERROR" message, it indicates connection close
    int timeout = millis() + 1000; // wait 1 sec
    while (millis() < timeout) {
      if (client.available()) {
        String s = client.readStringUntil('\n');
        //Serial.println(s);
        if (s.startsWith("ERROR")) {
          Serial.println("ERROR received");
          break;
        }
      } else {
        delay(10);
      }
    }
#endif

    // disconnect from irc server
    client.stop();
    //delay(100); // wait for sending FIN
  }

#ifdef USE_XIVELY
  if (!strcmp(tinyname, XIVELY_TINY)) {
    Serial.println(millis() - startMillis);
    Serial.print("sendXively() = ");
    Serial.println(
      sendXively(temp / 100.0, humi / 1024.0, p0, vcc_val / 1024.0, rssi, last_uptime / 1000)
    );
    Serial.println(millis() - startMillis);
  }
#endif

  // do not call disconnect() which clears WiFi SSID and Password
  //WiFi.disconnect();
}

#define SLEEP_600S (600 * 1000 * 1000) // 10 min
#define SLEEP_60S (60 * 1000 * 1000) // 1 min

void loop(void)
{
  Serial.print("uptime: ");
  Serial.print(millis());
  Serial.println(" ms");

  // LED off
  digitalWrite(LED_PIN, HIGH);

#ifdef USE_RTC_RAM
  // save uptime to RTC RAM
  uint32_t rtc_ram[2] = { RTC_MAGIC, micros() - startMicros };
  if (!system_rtc_mem_write(RTC_RAM_START, rtc_ram, sizeof(rtc_ram))) {
    Serial.println("system_rtc_mem_write() fail!");
  }
  Serial.print("rtc_ram[0] = ");
  Serial.println(rtc_ram[0], HEX);
  Serial.print("rtc_ram[1] = ");
  Serial.println(rtc_ram[1]);
#endif

  //int t = SLEEP_600S - (micros() - startMicros);
  int t = interval * 1000 * 1000;
#if 0
  if (t < 0) t = SLEEP_600S;
  Serial.print("sleep:");
  Serial.print(t);
  Serial.println(" us");
#endif
  ESP.deepSleep(t, WAKE_RF_DEFAULT);
  while (1) delay(1000); // sleep forever
}

uint16_t make_short(uint8_t *buf)
{
  return *buf | *(buf + 1) << 8;
}

uint32_t make_uint20(uint8_t *buf)
{
  return *buf << 12 | *(buf + 1) << 4 | *(buf + 2) >> 4;
}

uint16_t make_uint16(uint8_t *buf)
{
  return *buf << 8 | *(buf + 1);
}

#define BME280_S32_t int32_t
#define BME280_U32_t uint32_t
#define BME280_S64_t int64_t

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// t_fine carries fine temperature as global value
int32_t t_fine;
int32_t BME280_compensate_T_int32(int32_t adc_T)
{
  int32_t var1, var2, T;
  var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) *
          ((int32_t)dig_T3)) >> 14;
  t_fine = var1 + var2;
  T = (t_fine * 5 + 128) >> 8;
  return T;
}

// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
// Output value of “47445” represents 47445/1024 = 46.333 %RH
uint32_t BME280_compensate_H_int32(int32_t adc_H)
{
  int32_t v_x1_u32r;
  v_x1_u32r = (t_fine - ((int32_t)76800));
  v_x1_u32r = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * v_x1_u32r)) +
                 ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)dig_H6)) >> 10) * (((v_x1_u32r *
                     ((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
                     ((int32_t)dig_H2) + 8192) >> 14));
  v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
  v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
  v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
  return (uint32_t)(v_x1_u32r >> 12);
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
uint32_t BME280_compensate_P_int64(int32_t adc_P)
{
  BME280_S64_t var1, var2, p;
  var1 = ((BME280_S64_t)t_fine) - 128000;
  var2 = var1 * var1 * (BME280_S64_t)dig_P6;
  var2 = var2 + ((var1 * (BME280_S64_t)dig_P5) << 17);
  var2 = var2 + (((BME280_S64_t)dig_P4) << 35);
  var1 = ((var1 * var1 * (BME280_S64_t)dig_P3) >> 8) + ((var1 * (BME280_S64_t)dig_P2) << 12);
  var1 = (((((BME280_S64_t)1) << 47) + var1)) * ((BME280_S64_t)dig_P1) >> 33;
  if (var1 == 0)
  {
    return 0; // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((BME280_S64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((BME280_S64_t)dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((BME280_S64_t)dig_P7) << 4);
  return (uint32_t)p;
}
// Returns pressure in Pa as unsigned 32 bit integer. Output value of “96386” equals 96386 Pa = 963.86 hPa
uint32_t BME280_compensate_P_int32(int32_t adc_P)
{
  int32_t var1, var2;
  uint32_t p;
  var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
  var2 = (((var1 >> 2) * (var1 >> 2)) >> 11 ) * ((int32_t)dig_P6);
  var2 = var2 + ((var1 * ((int32_t)dig_P5)) << 1);
  var2 = (var2 >> 2) + (((int32_t)dig_P4) << 16);
  var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13 )) >> 3) + ((((int32_t)dig_P2) * var1) >> 1)) >> 18;
  var1 = ((((32768 + var1)) * ((int32_t)dig_P1)) >> 15);
  if (var1 == 0)
  {
    return 0; // avoid exception caused by division by zero
  }
  p = (((uint32_t)(((int32_t)1048576) - adc_P) - (var2 >> 12))) * 3125;
  if (p < 0x80000000)
  {
    p = (p << 1) / ((uint32_t)var1);
  }
  else
  {
    p = (p / (uint32_t)var1) * 2;
  }
  var1 = (((int32_t)dig_P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
  var2 = (((int32_t)(p >> 2)) * ((int32_t)dig_P8)) >> 13;
  p = (uint32_t)((int32_t)p + ((var1 + var2 + dig_P7) >> 4));
  return p;
}

void write_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t cmd)
{
  Wire.beginTransmission(dev_addr);
  Wire.write(reg_addr);
  Wire.write(cmd);
  Wire.endTransmission();
}

int readmulti(uint8_t dev_addr, uint8_t reg_addr, uint8_t *buf, size_t len)
{
  int cnt;
  int i;

  Wire.beginTransmission(dev_addr);

  i = Wire.write(reg_addr);
  if ( i != 1) {
    Wire.endTransmission(true);
    return 0; // error
  }

  i = Wire.endTransmission(false);
  if (i != 0) {
    Wire.endTransmission(true);
    return 0; // error
  }

  cnt = Wire.requestFrom(dev_addr, len, true);
  if (cnt != len) {
    Wire.endTransmission(true);
    return 0; // error
  }
  for (i = 0; i < len && Wire.available() > 0; i++) {
    buf[i] = Wire.read();
  }

  cnt = i;
  Wire.endTransmission(true);

  return cnt;
}

// TINY-AMEDES Configuration routine

String magic; // magic number for integrity check

String byte2hex(byte x)
{
  String s;

  if (x < 0x10) s = "0";
  s += String(x, HEX);
  return s;
}

char *msg = NULL;

void handleRoot(void) {
  String webString;
  byte mac[6];
  int i;

  webString = "TINY-AMEDES " + String(tinyname) + "<br/><hr>";

  if (msg) {
    webString += String(msg) + "<br/><hr>";
    msg = NULL;
  }

  webString += "Mac address: ";
  WiFi.macAddress(mac);

  for (i = 0; i < 5; i++) {
    webString += byte2hex(mac[i]) + ":";
  }
  webString += byte2hex(mac[i]) + "<br/>";

  read_conf_file();

  magic = String(random(0xffffffff), HEX);
  webString += "<form action='/conf'>"
               "Ssid:<input type='text' name='ssid' value='" + sta_ssid + "' /><br/>"
#if 0
               "Pass:<input type='password' name='pass' value='" + sta_password + "'/><br/>"
#else
               "Pass:<input type='text' name='pass' value='" + sta_password + "'/><br/>"
#endif
               "Location:<input type='text' name='location' value='" + location + "'/><br/>"
               "Height:<input type='text' name='height' value='" + String(height) + "'/>m<br/>"
               "Ncikname:<input type='text' name='nick' value='" + nickname + "'/><br/>"
               "Interval:<input type='text' name='interval' value='" + String(interval) + "'/>sec.<br/>"
               "IRC Channel:<input type='text' name='channel' value='" + ircChannel + "'/><br/>"
               "<input type='checkbox' name='useSyslog' value='yes'" + String(useSyslog ? " checked='checked'" : "") + "/>Use syslog<br/>"
               "Syslog server:<input type='text' name='syslogServer' value='" + syslogServer + "'/><br>"
               "<input type='checkbox' name='reset' value='yes'>Factory reset<br/>"
               "<input type='hidden' name='magic' value='" + magic + "'/>"
               ;

#if 1
  // display BME280 data

#define LOC_NN 2
  String loc[LOC_NN] = { "Outside", "Room" };
  // Outside and Room temp.

  webString += "<hr>";
  for (int i = 0; i < LOC_N; i++) {
    int temp, pres, humi;
    float p0;

    if (!bme280_exist(BME280_0 + i)) continue;

    if (bme280_measurement(BME280_0 + i) == 0) continue;

    // measurement is not done
    if (adc_T == 0x80000 && adc_P == 0x80000 && adc_H == 0x8000) continue;

    // temperatur = temp / 100 C
    temp = BME280_compensate_T_int32(adc_T); // must be first
    // pressure = pres / 256 Pa
    pres = BME280_compensate_P_int64(adc_P);
    // humidity = humi / 1024 %rH
    humi = BME280_compensate_H_int32(adc_H);

    // pressure compansation
    p0 = pres / 25600.0 * power(1.0 +  0.65 * height / (temp + 27315.0), 5.257);
    Serial.print("p:");
    Serial.println(pres / 25600.0);
    Serial.print("p0:");
    Serial.println(p0);

    webString += "BME280(" + loc[i] + "): temp:" + String(temp / 100.0, 1) + "C, "
                 +  "atm:" + String(p0, 1) + "hPa, "
                 + "humi:" + String(humi / 1024.0, 1) + "%<br/>";
  }
#endif
  webString +=
    "<hr>"
    "<input type='submit' name='OK'>"
    "<input type='reset' name='reset'>"
    "</form>"
    ;

  server.send(200, "text/html", webString);
}

#define RESET_TIME (600 * 1000) // 10 min.
int timeout;

String urlDecode(String s)
{
  int i;
  String t = s;
  char str[4];
  char c[2];

  t.replace("+", " ");

  for (i = 0x20; i < 0x80; i++) {
    sprintf(str, "%%%02X", i); // upper case
    sprintf(c, "%c", i);
    t.replace(str, c);
  }

  return t;
}

void handleConf(void)
{
  String webString;
  String ssid;
  String pass;
  String loc;
  String height;
  String nick;
  String interval;
  String channel;
  String factory_reset;
  int i;
  File f;

  timeout = millis() + RESET_TIME;
  msg = NULL;

  // integrity check
  if (magic.compareTo(server.arg("magic")) != 0) {
    msg = (char *)"Error! Form is too old";
    handleRoot();
    return;
  }

  if (server.arg("reset") == "yes") {
    // remove config file
    SPIFFS.remove(TINY_CONF);
    handleRoot();
    return;
  }

  ssid = urlDecode(server.arg("ssid"));
  pass = urlDecode(server.arg("pass"));
  loc = urlDecode(server.arg("location"));
  height = urlDecode(server.arg("height"));
  nick = urlDecode(server.arg("nick"));
  interval = urlDecode(server.arg("interval"));
  channel = urlDecode(server.arg("channel"));
  useSyslog = server.arg("useSyslog") == "yes";
  syslogServer = urlDecode(server.arg("syslogServer"));

  f = SPIFFS.open(TINY_CONF, "w");
  if (!f) {
    Serial.println("config file open error");

    msg = (char *)"Error! Could not save config data";
    handleRoot();
    return;
  }

  f.print("ssid:");
  f.println(ssid);
  f.print("password:");
  f.println(pass);
  f.print("location:");
  f.println(loc);
  f.print("height:");
  f.println(height);
  f.print("nickname:");
  f.println(nick);
  f.print("interval:");
  f.println(interval);
  f.print("channel:");
  f.println(channel);
  f.print("useSyslog:");
  f.println(useSyslog);
  f.print("syslogServer:");
  f.println(syslogServer);

  f.close();

  handleRoot();
}

#if 0
void handleReset(void)
{
  // remove config file
  SPIFFS.remove(TINY_CONF);

  handleRoot();
}
#endif


void config(void) {
  // read configuration file
  read_conf_file();

  Serial.println("Configuring access point...");

  Serial.print("ssid:");
  Serial.println(tinyname);
  Serial.print("password:");
  Serial.println(ap_password);

  WiFi.disconnect();
  WiFi.mode(WIFI_AP);
  WiFi.softAP(tinyname, ap_password.c_str());

  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  server.on("/", handleRoot);
  server.on("/conf", handleConf);
  //server.on("/reset", handleReset);

  randomSeed(ESP.getCycleCount()); // inirialize random seed

  server.begin();
  Serial.println("HTTP server started");

  Serial.println("handleClient()");
  //timeout = millis() + RESET_TIME; // time out 5 min.

  //while (millis() < timeout) { // time out 300 sec for save battery
  while (1) {
    server.handleClient();
    delay(1);
    if (millis() & 0x400) {
      digitalWrite(LED_PIN, HIGH);
    } else {
      digitalWrite(LED_PIN, LOW);
    }
  }
  //ESP.restart(); // restart system
}

void read_conf_file(void)
{
  // default value
  sta_ssid = "";
  sta_password = "";
  location = "unknown";
  height = 0;
  nickname = tinyname;
  interval = DEFAULT_INTERVAL;
  ircChannel = IRC_CHANNEL;
  useSyslog = 0;
  syslogServer = "192.168.0.1";

  // read conf file
  File f = SPIFFS.open(TINY_CONF, "r");
  if (!f) {
    Serial.println("Config file cannot open");
    return;
  }

  while (f.available()) {
    String s = f.readStringUntil(':');

    if (s.startsWith("ssid")) {
      sta_ssid = f.readStringUntil('\r');
      Serial.print("ssid:");
      Serial.println(sta_ssid);
    } else if (s.startsWith("password")) {
      sta_password = f.readStringUntil('\r');
      Serial.print("password:");
      Serial.println(sta_password);
    } else if (s.startsWith("location")) {
      location = f.readStringUntil('\r');
      Serial.print("location:");
      Serial.println(location);
    } else if (s.startsWith("height")) {
      height = f.parseFloat();
      Serial.print("height:");
      Serial.println(height);
    } else if (s.startsWith("nickname")) {
      nickname = f.readStringUntil('\r');
      Serial.print("nickname:");
      Serial.println(nickname);
    } else if (s.startsWith("interval")) {
      interval = f.parseInt();
      Serial.print("interval:");
      Serial.println(interval);
    } else if (s.startsWith("channel")) {
      ircChannel = f.readStringUntil('\r');
      Serial.print("channel:");
      Serial.println(ircChannel);
    } else if (s.startsWith("useSyslog")) {
      useSyslog = f.parseInt();
      Serial.print("useSyslog:");
      Serial.println(useSyslog);
    } else if (s.startsWith("syslogServer")) {
      syslogServer = f.readStringUntil('\r');
      Serial.print("syslogServer:");
      Serial.println(syslogServer);
    } else {
      Serial.print("invalid string:");
      Serial.println(s);
    }
    f.readStringUntil('\n'); // discard LF
  }
  f.close();
}

#ifdef USE_LCD
/*
 * LCD routine
 */

#define LCD 0x3e // I2C address

void lcd_init(void)
{
  uint8_t cmd_init[] = {0x38, 0x39, 0x14, 0x70, 0x56, 0x6c, 0x38, 0x0c, 0x01};

  if (vcc_val < (3 * 1024)) cmd_init[4] |= 0x01; // change contrast value if battery
  delay(40);
  command(LCD, cmd_init, sizeof(cmd_init));
  delay(2);  // > 1.08ms
}

void lcd_out(const char str[])
{
  int len = strlen(str);

  if (len > 8) len = 8;
  write(LCD, (uint8_t*)str, len);
}

void lcd_cr(void)
{
  uint8_t cmd_cr[] = {0xc0};
  command(LCD, cmd_cr, sizeof(cmd_cr));
}

void lcd_clr(void)
{
  uint8_t cmd_clr[] = { 0x01 };
  command(LCD, cmd_clr, sizeof(cmd_clr));
  delay(1);
}
#endif

void command(uint8_t addr, uint8_t *cmd, size_t len) {
  size_t i;
  int n;
  
  for (i = 0; i < len; i++) {
    Wire.beginTransmission(addr);
    Wire.write(0x00);
    Wire.write(cmd[i]);
    n = Wire.endTransmission();
    delayMicroseconds(27 * 2);    // 26.3us
    if (n != 0) {
      Serial.print("I2C command error: ");
      Serial.println(n);
      return;
    }
  }
}

void write(uint8_t addr, uint8_t *cmd, size_t len) {
  size_t i;
  int n;
  
  for (i = 0; i < len; i++) {
    Wire.beginTransmission(addr);
    Wire.write(0x40);
    Wire.write(cmd[i]);
    n = Wire.endTransmission();
    delayMicroseconds(27 * 2);    // 26.3us
    if (n != 0) {
      Serial.print("I2C write error: ");
      Serial.println(n);
      return;
    }
  }
}

/*
 * Lightning Sensor
 */

#define AS3935 0x00
#define IRQ_PIN 14  // IO14 connect to IRQ of AS3935

#define PRESET_DEFAULT 0x3c
#define CALIB_RCO 0x3d
#define DIRECT_COMMAND 0x96

void lightning(void)
{
  uint8_t reg[11];
  int c;
  int i;
  int irq = 0;

  // read out IRQ pin state
  pinMode(IRQ_PIN, INPUT);
  if (digitalRead(IRQ_PIN) == HIGH) {
    irq = 1;
  }

  // read AS3935 registers
  c = readmulti(AS3935, 0x00, reg, 9);
  if (c != 9) {
    // No AS3935
    return;
  }
  c = readmulti(AS3935, 0x3a, &reg[9], 2);
  if (c != 2) {
    return;
  }

  Serial.print("Lightning Sensor reg:");
  for (i = 0; i < 11; i++) {
    Serial.print(String(reg[i], HEX) + " ");
  }
  Serial.println();

  // AFE gain
#define INDOOR  (0x12 << 1) // Indoor
#define OUTDOOR (0x0e << 1) // Outdoor

  // check existence of AS3935
  switch (reg[0x00] & 0x3e) {
    case INDOOR: // AS3935 exist and need initialization
      as3935_init(reg);
      irq = 0;
      break;
    case OUTDOOR:
      break;
    default: // someting wrong
      // reset AS3935
      write_reg(AS3935, PRESET_DEFAULT, DIRECT_COMMAND);
      return;
  }

  Serial.println("AS3935 found!");

  // randomly reset AS3935
#define AS3935_RESET_P 1008 // reset probability is 1/1008
  if (!irq) {
    randomSeed(ESP.getCycleCount());
    if (random(AS3935_RESET_P) == 0) {
      as3935_init(reg);
      return;
    }
  }

  if (irq) {
    Serial.println("AS3935 generates interrupt");

    lightning_detect = reg[0x03] & 0x0f;
    switch (lightning_detect) {
      case INT_NH:
        Serial.println("Noise level too high");
        if ((i = reg[0x01] & 0x70) < 0x70) {
          i += 0x10;
          i |= reg[0x01] & 0x8f;
          write_reg(AS3935, 0x01, i);
        }
        lightning_nf_lev = (i & 0x70) >> 4;
        break;
      case INT_D:
        Serial.println("Disturber detected");
        if ((i = reg[0x01] & 0x0f) < 0x0f) {
          i += 1;
          lightning_wdth = i;
          i |= reg[0x01] & 0xf0;
          write_reg(AS3935, 0x01, i);
        } else { // WDTH == 0x0f
          lightning_wdth = i;
          i = (reg[0x03] & 0xf0) | 0x20;
          write_reg(AS3935, 0x03, i);
        }
        break;
      case INT_L:
        Serial.println("Lightning interrupt");
        lightning_energy = reg[0x04] | reg[0x05] << 8 | (reg[0x06] & 0x0f) << 16;
        lightning_distance = reg[0x07] & 0x3f;
        break;
      default:
        Serial.print("Unknown interrupt:");
        Serial.print(lightning_detect);
        Serial.println();
    }
  }
}

// Lighting sensor calibration
volatile int irq_cnt;
int time_out;
volatile int calib_done;
volatile int time0;
volatile int time1;

void as3935_init(uint8_t reg[])
{
  delay(1000); // You must go away from AS3935 as fast as possible when turn on AMEDES
  write_reg(AS3935, PRESET_DEFAULT, DIRECT_COMMAND); // reset AS3935
  // set AFE gain
  write_reg(AS3935, 0x00, OUTDOOR);

#define DIV_PARAM 3 // 0:16, 1:32, 2:64, 3:128
#define LCO_FDIV (DIV_PARAM << 6)
#define DIV_RATIO (16 << DIV_PARAM)

  // Antenna Tunig
  write_reg(AS3935, 0x03, LCO_FDIV); // set division ratio 1/16 - 1/128

#define DISP_LCO 0x80
#define IRQ_COUNT 1024
#define FREQ_500K (500 * 1000)

  int tun_cap; // capacitors setting 0 - 15
  int sel_cap = -1;
  double diff = FREQ_500K;
  double d;
  double freq;
  int i;

  Serial.println("LCO calibration");
  for (tun_cap = 0; tun_cap < 16; tun_cap++) {
    irq_cnt = IRQ_COUNT;
    calib_done = 0;
    time0 = time1 = 0;
    i = 200; // timeout 2 sec

    attachInterrupt(IRQ_PIN, count, RISING); // set interrupt handler

    write_reg(AS3935, 0x08, DISP_LCO | tun_cap); // set capaciters and LCO output

    while (!calib_done && i-- > 0) { // wait for completion of calibration
      delay(10);
    }
    detachInterrupt(IRQ_PIN); // disable interrupt handler
    write_reg(AS3935, 0x08, tun_cap); // stop LCO output

    if (i <= 0) { // timeout, calibration fail
      continue;
    }

    freq = (1000.0 * 1000 * DIV_RATIO * IRQ_COUNT) / (double)(time1 - time0) ;
    Serial.print("tun_cap:");
    Serial.print(tun_cap);
    Serial.print(", freq:");
    Serial.println(freq);

    d = freq - FREQ_500K;
    if (d < 0) d = -d;

    if (d < diff) {
      sel_cap = tun_cap;
      diff = d;
    }
  }

#define POWER_DOWN 1

  if (sel_cap < 0) { // calibration fail
    Serial.println("AS3935 calibration fail");
    write_reg(AS3935, 0x00, OUTDOOR | POWER_DOWN); // set Power-down
    return;
  }
  write_reg(AS3935, 0x08, sel_cap);
  Serial.print("selected tun_cap:");
  Serial.println(sel_cap);

  lightning_detect = AS3935_INIT;
  lightning_cap = sel_cap;

  // Calibrate TRCO and SRCO
  write_reg(AS3935, CALIB_RCO, DIRECT_COMMAND); // calibrate RCO
}

void count(void)
{
  int now = micros();

  if (time0 == 0) {
    time0 = now;
    return;
  }
  if (--irq_cnt == 0) {
    time1 = now;
    calib_done = 1;
  }
}

#define STTS751_ADDR 0x39

#define STTS751_TEMP_HIGH_ADDR 0x00
#define STTS751_STATUS_ADDR 0x01
#define STTS751_TEMP_LOW_ADDR 0x02
#define STTS751_CONFIGURATION_ADDR 0x03
#define STTS751_ONE_SHOT 0x0f
#define STTS751_PRODUCT_ID_ADDR 0xfd
#define STTS751_MANUFACTURER_ID_ADDR 0xfe
#define STTS751_REVISION_NUMBER_ADDR 0xff

int stts751_exist(uint8_t addr)
{
  uint8_t reg;

  if (!readmulti(addr, STTS751_PRODUCT_ID_ADDR, &reg, 1)) return 0;
  if ((reg & 0xfe) != 0) return 0;

  if (!readmulti(addr, STTS751_MANUFACTURER_ID_ADDR, &reg, 1)) return 0;
  if (reg != 0x53) return 0;

  if (!readmulti(addr, STTS751_REVISION_NUMBER_ADDR, &reg, 1)) return 0;
  if (reg != 0x01) return 0;

  return 1;
}

void stts751_init(uint8_t addr)
{
  write_reg(addr, STTS751_CONFIGURATION_ADDR, 0xcc);  // stand by mode, resolution 12bit
}

int stts751_temp(uint8_t addr)
{
  uint8_t reg;
  int16_t temp;

  // start measurement
  write_reg(addr, STTS751_ONE_SHOT, 0);

  while (readmulti(addr, STTS751_STATUS_ADDR, &reg, 1) && (reg & 0x80)) {
    delay(10);
  }

  readmulti(addr, STTS751_TEMP_HIGH_ADDR, &reg, 1);
  temp = reg << 8;

  readmulti(addr, STTS751_TEMP_LOW_ADDR, &reg, 1);
  temp |= reg;

  return temp;
}

void stts751(void)
{
  int i;
  uint8_t addr;

  for (i = 0; i < STTS751_N; i++) {
    addr = stts751_addrs[i];
    if (stts751_exist(addr)) {
      Serial.print("STTS751(");
      Serial.print(addr, HEX);
      Serial.print("): ");

      stts751_init(addr);
      stts751_temperature = stts751_temp(addr);

      Serial.print(stts751_temperature / 256.0, 1);
      Serial.println("C");
    }
  }
}

