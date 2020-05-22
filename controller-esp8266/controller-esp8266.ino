#define CMMC_USE_ALIAS
#include <Arduino.h>
#include <Ticker.h>
#include <ESP8266WiFi.h>
#include <CMMC_SimplePair.h>
#include <CMMC_Utils.h>
#include <CMMC_ESPNow.h>
#include <CMMC_LED.h>
#include <CMMC_BootMode.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <time.h>
#ifdef ESP8266
extern "C" {
#include "user_interface.h"
}
#include "ESP8266WiFi.h"
#include <functional>
#include <map>
#include "FS.h"
#endif#include <CMMC_BootMode.h>
#include "modules/ESPNowModule.h"
// #include <SoftwareSerial.h>
#include <Button2.h>

#include "data_type.h"

uint8_t self_mac[6];
uint8_t master_mac[6];
uint8_t slave_mac[6];

char self_mac_string[13];
char slave_mac_string[13];
uint32_t recv_count = 0;

#define LED_PIN                 (16)
#define BUTTON_PIN              (0)

Button2 button = Button2(BUTTON_PIN);

char* espnowMsg[300];
bool dirty = false;
u8 currentSleepTimeMinuteByte = 5;

int mode;

CMMC_SimplePair simplePair;
CMMC_ESPNow espNow;
CMMC_Utils utils;
CMMC_LED led(LED_PIN, LOW);

uint8_t mmm[6];
Ticker blinker;

void flip() {
  led.toggle();
}

void evt_callback(u8 status, u8* sa, const u8* data);

void start_config_mode() {
  uint8_t* controller_addr = utils.getESPNowControllerMacAddress();
  utils.printMacAddress(controller_addr);
  simplePair.begin(MASTER_MODE, evt_callback);
  simplePair.debug([](const char* s) {
    Serial.println(s);
    // Serial.println("simple pair debug")
    // Serial.println(s);
  });
  simplePair.set_message(controller_addr, 6);
  simplePair.start();
}


int counter = 0;
// #include <CMMC_RX_Parser.h>
// CMMC_RX_Parser parser(&Serial);

uint32_t _time;
// CMMC_PACKET_T pArr[30];

void tripleClick(Button2& btn) {
    Serial.println("triple click\n");
    blinker.attach_ms(100, flip);
    esp_now_deinit();
    start_config_mode();
}

bool saveConfig() {
  StaticJsonDocument<200> doc;
  doc["slave_mac"] = String(slave_mac_string);
  File configFile = LittleFS.open("config.json", "w");
  if (!configFile) {
    Serial.println("Failed to open config file for writing");
    return false;
  }

  Serial.print(F("Serializing to file. Size = "));
  uint16 size = serializeJson(doc, configFile);

  Serial.println(size);
  configFile.close();
  return true;
}

void listDir(const char * dirname) {
  Serial.printf("Listing directory: %s\n", dirname);

  Dir root = LittleFS.openDir(dirname);

  while (root.next()) {
    File file = root.openFile("r");
    Serial.print("  FILE: ");
    Serial.print(root.fileName());
    Serial.print("  SIZE: ");
    Serial.println(file.size());
    file.close();
  }
}

void setup()
{
  Serial.begin(115200);
  delay(10);
  WiFi.disconnect(true);
  WiFi.softAPdisconnect();
  // WiFi.mode(WIFI_STA); // Station mode for esp-now controller
  // wifi_set_opmode(SOFTAP_MODE);
  LittleFS.begin();
  // File configFile = LittleFS.open("mac.json", "w");
  Serial.println("Mounting FS...");
   if (!LittleFS.begin()) {
     Serial.println("Failed to mount file system");
     return;
   }

   // saveConfig();
   listDir("/");
   loadConfig();


  led.init();
  led.low();
  // char buf[13];
  uint8_t* self_mac_ptr = CMMC::getESPNowSlaveMacAddress();
  memcpy(self_mac, self_mac_ptr, 6);
  Serial.println();
  Serial.printf("WITH MAC: ");
  CMMC::printMacAddress((uint8_t*)self_mac_ptr);
  CMMC::macByteToString(self_mac, self_mac_string);
  Serial.println("Controller Mode");
  Serial.println(self_mac_string);

  button.setDoubleClickHandler(tripleClick);

  espNow.init(NOW_MODE_CONTROLLER);
  espNow.enable_retries(false);
  espNow.on_message_sent([](uint8_t *macaddr, u8 status) {
    led.toggle();
  });

  // module = this;
  espNow.on_message_recv([](uint8_t * macaddr, uint8_t * data, uint8_t len) {
    recv_count++;
    // Serial.println("recv");
    // user_espnow_sent_at = millis();
    led.toggle();
  });

}

bool loadConfig() {
  File configFile = LittleFS.open("config.json", "r");
  if (!configFile) {
    Serial.println("Failed to open config file");
    return false;
  }

  size_t size = configFile.size();
  if (size > 1024) {
    Serial.println("Config file size is too large");
    return false;
  }

  // Allocate a buffer to store contents of the file.
  std::unique_ptr<char[]> buf(new char[size]);

  // We don't use String here because ArduinoJson library requires the input
  // buffer to be mutable. If you don't use ArduinoJson, you may as well
  // use configFile.readString instead.
  configFile.readBytes(buf.get(), size);

  // Serial.println(String(buf));

  StaticJsonDocument<200> doc;
  auto error = deserializeJson(doc, buf.get());
  if (error) {
    Serial.println("Failed to parse config file");
    return false;
  }

  const char* mac = doc["slave_mac"];
  Serial.print("MAC=");
  Serial.println(mac);


  // Real world application would store these values in some variables for
  // later use.

  return true;
}


#include <CMMC_TimeOut.h>
CMMC_TimeOut ct;
uint32_t prev = millis();

void loop()
{
  button.loop();
  if (millis() % 2000 == 0) {
    Serial.println(recv_count/2);
    recv_count = 0;
    delay(10);
  }
}

void str2Hex(const char* text, char* buffer) {
  size_t len = strlen(text);
  for (int i = 0 ; i < len; i++) {
    sprintf(buffer + i * 2, "%02x", text[i]);
  }
}

void toHexString(const u8 array[], size_t len, char buffer[]) {
  for (unsigned int i = 0; i < len; i++)
  {
    byte nib1 = (array[i] >> 4) & 0x0F;
    byte nib2 = (array[i] >> 0) & 0x0F;
    buffer[i * 2 + 0] = nib1  < 0xA ? '0' + nib1  : 'A' + nib1  - 0xA;
    buffer[i * 2 + 1] = nib2  < 0xA ? '0' + nib2  : 'A' + nib2  - 0xA;
  }
  buffer[len * 2] = '\0';
}

void evt_callback(u8 status, u8* sa, const u8* data) {
  if (status == 0) {
    Serial.printf("[CSP_EVENT_SUCCESS] STATUS: %d\r\n", status);
    Serial.printf("WITH KEY: ");
    utils.dump(data, 16);
    Serial.printf("WITH MAC: ");
    utils.dump(sa, 6);
  // uint8_t* self_mac_ptr = CMMC::getESPNowSlaveMacAddress();
  // memcpy(self_mac, self_mac_ptr, 6);
    memcpy(slave_mac, sa, 6);
    CMMC::macByteToString(slave_mac, slave_mac_string);
    saveConfig();
    led.high();
    ESP.reset();
  }
  else {
    Serial.printf("[CSP_EVENT_ERROR] %d: %s\r\n", status, (const char*)data);
  }
}
