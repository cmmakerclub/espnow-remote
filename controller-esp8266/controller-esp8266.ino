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
    start_config_mode();
}

uint8_t self_mac[6];
uint8_t master_mac[6];

bool saveConfig() {
  StaticJsonDocument<200> doc;
  doc["serverName"] = "api.example.com";
  doc["accessToken"] = "128du9as8du12eoue8da98h123ueh9h98";

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
    Serial.print(file.size());
    file.close();
    // struct tm * tmstruct = localtime(&cr);
    // Serial.printf("    CREATION: %d-%02d-%02d %02d:%02d:%02d\n", (tmstruct->tm_year) + 1900, (tmstruct->tm_mon) + 1, tmstruct->tm_mday, tmstruct->tm_hour, tmstruct->tm_min, tmstruct->tm_sec);
    // tmstruct = localtime(&lw);
    // Serial.printf("  LAST WRITE: %d-%02d-%02d %02d:%02d:%02d\n", (tmstruct->tm_year) + 1900, (tmstruct->tm_mon) + 1, tmstruct->tm_mday, tmstruct->tm_hour, tmstruct->tm_min, tmstruct->tm_sec);
  }
}

void setup()
{
  Serial.begin(115200);
  WiFi.mode(WIFI_STA); // Station mode for esp-now controller
  WiFi.disconnect();
  LittleFS.begin();
  // File configFile = LittleFS.open("mac.json", "w");
  Serial.println("Mounting FS...");
   if (!LittleFS.begin()) {
     Serial.println("Failed to mount file system");
     return;
   }

   // saveConfig();
   listDir("/");


  // char buf[13];
  char self_mac_string[13];
  uint8_t* slave_addr = CMMC::getESPNowSlaveMacAddress();
  memcpy(self_mac, slave_addr, 6);

  Serial.println();
  Serial.printf("WITH MAC: ");
  CMMC::printMacAddress((uint8_t*)slave_addr);
  CMMC::macByteToString(self_mac, self_mac_string);
  // CMMC::printMacAddress((uint8_t*)self_mac_string);
  // Serial.begin(9600);
  // Serial.println("Controller Mode");
  Serial.println("Controller Mode");
  Serial.println(self_mac_string);
  led.init();
  button.setDoubleClickHandler(tripleClick);

  // blinker.attach_ms(100, flip);
  // parser.on_command_arrived([](CMMC_SERIAL_PACKET_T * packet, size_t len) {
  //  swSerial.printf("ON_PARSER at (%lums)", millis());
  //  swSerial.printf("CMD->0x%2x\r\n", packet->cmd);
  //  swSerial.printf("LEN->%lu\r\n", packet->len);
  //   CMMC::dump((u8*)packet->data, 4);
  //   CMMC::dump((u8*)packet->data+4, 4);
  //  swSerial.printf("HEAP = %lu\r\n", ESP.getFreeHeap());
  //   if (packet->cmd == CMMC_SLEEP_TIME_CMD) {
  //     memcpy(&time, packet->data, 4);
  //     currentSleepTimeMinuteByte = time;
  //     if (time > 255) {
  //       currentSleepTimeMinuteByte = 254;
  //     }
  //   }
  // });

  uint32_t wait_config = 1000;
  // pinMode(PROD_MODE_PIN, INPUT_PULLUP);
  // uint32_t wait_config = 1000;
  // if (digitalRead(PROD_MODE_PIN) == LOW) {
  //   wait_config = 0;
  // }

  // Serial.printf("wait_config = %d \r\n", wait_config);
  // CMMC_BootMode bootMode(&mode, BUTTON_PIN);
  // bootMode.init();
  // bootMode.check([](int mode) {
  //   if (mode == BootMode::MODE_CONFIG) {
  //     start_config_mode();
  //   }
  //   else if (mode == BootMode::MODE_RUN) {
  //     led.high();
  //     Serial.print("Initializing... Controller..");
  //     espNow.init(NOW_MODE_CONTROLLER);
  //     espNow.on_message_recv([](uint8_t *macaddr, uint8_t *data, uint8_t len) {
  //       Serial.print("FROM: ");
  //       // CMMC::dump(macaddr, 6);
  //       memcpy(mmm, macaddr, 6);
  //       dirty = true;
  //       led.toggle();
  //       static CMMC_PACKET_T wrapped;
  //       static CMMC_SENSOR_DATA_T packet;
  //       memcpy(&packet, data, sizeof(packet));
  //       memcpy(&wrapped.data, &packet, sizeof(packet));
  //       wrapped.ms = millis();
  //       wrapped.sleepTime = currentSleepTimeMinuteByte;
  //       wrapped.data.field9 = analogRead(A0) * 0.0051724137931034f * 100;
  //       wrapped.sum = CMMC::checksum((uint8_t*) &wrapped, sizeof(wrapped) - sizeof(wrapped.sum));
  //       Serial.printf("sizeof wrapped packet = %d\r\n", sizeof(wrapped));
  //       // pArr[pArrIdx] = wrapped;
  //       // pArrIdx = (pArrIdx + 1) % 30;
  //       // toHexString((u8*)  &wrapped, sizeof(CMMC_PACKET_T), (char*)espnowMsg);
  //       // CMMC_Utils::dump((u8*)&wrapped, sizeof(wrapped));
  //       //swSerial.println(swSerial.write((byte*)&wrapped, sizeof(wrapped)));
  //     });
  //
  //     espNow.on_message_sent([](uint8_t *macaddr,  uint8_t status) {
  //       dirty = false;
  //     });
  //   }
  //   else {
  //     // unhandled
  //   }
  // }, wait_config);
}

#include <CMMC_TimeOut.h>
CMMC_TimeOut ct;
uint32_t prev = millis();

void loop()
{
  button.loop();

  // while (mode == BootMode::MODE_CONFIG) {
  //   ct.timeout_ms(60000);
  //   while (1 && !ct.is_timeout()) {
  //     delay(500);
  //     led.toggle();
  //   }
  //   swSerial.println("Simple Pair Wait timeout.");
  //   ESP.reset();
  // }

  // if (mode == BootMode::MODE_RUN) {
  //   while (dirty) {
  //     swSerial.printf("SENT SLEEP_TIME BACK = %u MINUTE\r\n", currentSleepTimeMinuteByte);
  //     espNow.send(mmm, &currentSleepTimeMinuteByte, 1);
  //   }
  // }

  // while(Serial.available()) {
  //   u8 incomingByte = Serial.read();
  //   if (incomingByte < 255 && (incomingByte >0)) {
  //     currentSleepTimeMinuteByte = incomingByte;
  //   }
  // }

  // ct.timeout_ms(5000);
  // while (digitalRead(13) == LOW) {
  //   if (ct.is_timeout()) {
  //     ESP.reset();
  //   }
  // }
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
    led.high();
    ESP.reset();
  }
  else {
    Serial.printf("[CSP_EVENT_ERROR] %d: %s\r\n", status, (const char*)data);
  }
}
