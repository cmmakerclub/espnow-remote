#define CMMC_USE_ALIAS
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <CMMC_SimplePair.h>
#include <CMMC_Utils.h>
#include <CMMC_ESPNow.h>
#include <CMMC_LED.h>
#include <CMMC_BootMode.h>
#include <CMMC_NB_IoT.h>
#include "data_type.h"

#include <SoftwareSerial.h>
#define rxPin                  12
#define txPin                  14
#define LED_PIN                 2
#define BUTTON_PIN              0
#define PROD_MODE_PIN          13


void str2Hex(const char* text, char* buffer);
void toHexString(const u8 array[], size_t len, char buffer[]);

char* espnowMsg[300];
bool dirty = false;
bool isNbConnected = false;
u8 currentSleepTimeMinuteByte = 5; 
//String token = "c7f549d0-1858-11e8-8630-b33e669e2295";
String token = "b98ce7b0-185b-11e8-8630-b33e669e2295";
char tokenHex[100];

SoftwareSerial swSerial(rxPin, txPin);
CMMC_NB_IoT nb(&swSerial);
int mode;

CMMC_SimplePair instance;
CMMC_ESPNow espNow;
CMMC_Utils utils;
CMMC_LED led(LED_PIN, LOW);

uint8_t mmm[6];

void evt_callback(u8 status, u8* sa, const u8* data);
void setup_hardware() {
  Serial.begin(57600);
  swSerial.begin(9600);

  Serial.println();
  led.init();
  delay(50);
  Serial.println();
  Serial.print(F("Starting Application... at ("));
  Serial.print(millis());
  Serial.println("ms)");

  str2Hex(token.c_str(), tokenHex);
  Serial.println(tokenHex);
  nb.setDebugStream(&Serial);

  nb.onDeviceReboot([]() {
    Serial.println(F("[user] Device rebooted."));
    // nb.queryDeviceInfo();
    // delay(1000);
  }); nb.onDeviceReady([]() {
    Serial.println("[user] Device Ready!");
  });

  nb.onDeviceInfo([](CMMC_NB_IoT::DeviceInfo device) {
    Serial.print(F("# Module IMEI-->  "));
    Serial.println(device.imei);
    Serial.print(F("# Firmware ver-->  "));
    Serial.println(device.firmware);
    Serial.print(F("# IMSI SIM-->  "));
    Serial.println(device.imsi);
  });

  nb.onMessageArrived([](char *text, size_t len, uint8_t socketId, char* ip, uint16_t port) {
    char buffer[100];
    sprintf(buffer, "++ [recv:] socketId=%u, ip=%s, port=%u, len=%d bytes (%lums)", socketId, ip, port, len, millis());
    Serial.println(buffer);
  });

  nb.onConnecting([]() {
    Serial.println("Connecting to NB-IoT...");
    delay(500);
  });

  nb.onConnected([]() {
    Serial.print("[user] NB-IoT Network connected at (");
    Serial.print(millis());
    Serial.println("ms)");
    Serial.println(nb.createUdpSocket("103.20.205.85", 5683, UDPConfig::ENABLE_RECV));
    Serial.println(nb.createUdpSocket("103.212.181.167", 55566, UDPConfig::ENABLE_RECV));
    isNbConnected = 1;
    delay(1000);
  });

  nb.rebootModule();
  // nb.begin();
  // nb.activate();
}

void start_config_mode() {
  uint8_t* controller_addr = utils.getESPNowControllerMacAddress();
  utils.printMacAddress(controller_addr);
  instance.begin(MASTER_MODE, evt_callback);
  instance.debug([](const char* s) {
    Serial.println(s);
  });
  instance.set_message(controller_addr, 6);
  instance.start();
}


int counter = 0;

#include <CMMC_RX_Parser.h>
CMMC_RX_Parser parser(&swSerial);

uint32_t time;
CMMC_PACKET_T pArr[30];
int pArrIdx = 0;


void setup()
{
  setup_hardware();
  Serial.println("Controller Mode");
  // parser.on_command_arrived([](CMMC_SERIAL_PACKET_T * packet, size_t len) {
  //   Serial.printf("ON_PARSER at (%lums)", millis());
  //   Serial.printf("CMD->0x%2x\r\n", packet->cmd);
  //   Serial.printf("LEN->%lu\r\n", packet->len);
  //   CMMC::dump((u8*)packet->data, 4);
  //   CMMC::dump((u8*)packet->data+4, 4);
  //   Serial.printf("HEAP = %lu\r\n", ESP.getFreeHeap());
  //   if (packet->cmd == CMMC_SLEEP_TIME_CMD) {
  //     memcpy(&time, packet->data, 4);
  //     currentSleepTimeMinuteByte = time;
  //     if (time > 255) {
  //       currentSleepTimeMinuteByte = 254;
  //     }
  //   }
  // });

  pinMode(PROD_MODE_PIN, INPUT_PULLUP);
  uint32_t wait_config = 1000;
  if (digitalRead(PROD_MODE_PIN) == LOW) {
    wait_config = 0;
  }
  Serial.printf("wait_config = %d \r\n", wait_config);
  CMMC_BootMode bootMode(&mode, BUTTON_PIN);
  bootMode.init();
  bootMode.check([](int mode) {
    Serial.printf("done.... mode = %d \r\n", mode);
    if (mode == BootMode::MODE_CONFIG) {
      start_config_mode();
    }
    else if (mode == BootMode::MODE_RUN) {
      led.high();
      Serial.print("Initializing... Controller..");
      espNow.init(NOW_MODE_CONTROLLER);
      espNow.on_message_recv([](uint8_t *macaddr, uint8_t *data, uint8_t len) {
        Serial.print("FROM: ");
        CMMC::dump(macaddr, 6);
        memcpy(mmm, macaddr, 6);
        dirty = true;
        led.toggle();
        static CMMC_PACKET_T wrapped;
        static CMMC_SENSOR_DATA_T packet;
        memcpy(&packet, data, sizeof(packet));
        memcpy(&wrapped.data, &packet, sizeof(packet));
        wrapped.ms = millis();
        wrapped.sleepTime = currentSleepTimeMinuteByte;
        wrapped.data.field9 = analogRead(A0) * 0.0051724137931034f * 100; 
        wrapped.sum = CMMC::checksum((uint8_t*) &wrapped, sizeof(wrapped) - sizeof(wrapped.sum));
        Serial.printf("sizeof wrapped packet = %d\r\n", sizeof(wrapped));
        pArr[pArrIdx] = wrapped;
        pArrIdx = (pArrIdx + 1) % 30;
        // toHexString((u8*)  &wrapped, sizeof(CMMC_PACKET_T), (char*)espnowMsg);
        // Serial.write((byte*)&wrapped, sizeof(wrapped));
        // CMMC_Utils::dump((u8*)&wrapped, sizeof(wrapped));
        // Serial.println(swSerial.write((byte*)&wrapped, sizeof(wrapped)));
      });

      espNow.on_message_sent([](uint8_t *macaddr,  uint8_t status) {
        dirty = false;
      });
    }
    else {
      // unhandled
    }
  }, wait_config);
}

#include <CMMC_TimeOut.h>
CMMC_TimeOut ct;
uint32_t prev = millis();

void loop()
{
  while (mode == BootMode::MODE_CONFIG) {
    ct.timeout_ms(60000);
    while (1 && !ct.is_timeout()) {
      delay(500);
      led.toggle();
    }
    Serial.println("Simple Pair Wait timeout.");
    ESP.reset();
  }

  if (mode == BootMode::MODE_RUN) {
    nb.loop();
    while (dirty) {
      Serial.printf("SENT SLEEP_TIME BACK = %u MINUTE\r\n", currentSleepTimeMinuteByte);
      espNow.send(mmm, &currentSleepTimeMinuteByte, 1);
    }
    if ( (pArrIdx > 0) && (isNbConnected)) {
      char buffer[500];
      char b[500];
      Serial.printf("pArrIdx = %d\r\n", pArrIdx);
      for (int i = pArrIdx - 1; i >= 0; i--) {
        Serial.printf("reading idx = %d\r\n", i);
        toHexString((u8*)  &pArr[i], sizeof(CMMC_PACKET_T), (char*)espnowMsg);
        sprintf(b, "{\"payload\": \"%s\"}", espnowMsg);
        str2Hex(b, buffer);
        String p3 = "";
        p3 += String("40");
        p3 += String("020579");
        p3 += String("b5");
        p3 += String("4e42496f54"); // NB-IoT
        p3 += String("0d");
        p3 += String("17");
        p3 +=  String(tokenHex);
        p3 += String("ff");
        p3 += String(buffer);
        int rt = 0;
        while (true) {
          swSerial.flush();
          if (nb.sendMessageHex(p3.c_str(), 0)) {
            Serial.println(">> [ais] socket0: send ok.");
            swSerial.flush();
            pArrIdx--;
            break;
          }
          else {
            Serial.println(">> [ais] socket0: send failed.");
            if (++rt > 5) {
              swSerial.flush();
              break;
            }
          }
        }
      }
      prev = millis();
    }
  }

  ct.timeout_ms(5000);
  while (digitalRead(13) == LOW) {
    if (ct.is_timeout()) {
      ESP.reset();
    }
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
    led.high();
    ESP.reset();
  }
  else {
    Serial.printf("[CSP_EVENT_ERROR] %d: %s\r\n", status, (const char*)data);
  }
}