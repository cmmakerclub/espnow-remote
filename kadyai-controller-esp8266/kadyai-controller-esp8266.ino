#define CMMC_USE_ALIAS
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <CMMC_SimplePair.h>
#include <CMMC_Utils.h>
#include <CMMC_ESPNow.h>
#include <CMMC_LED.h>
#include <CMMC_BootMode.h>
#include <CMMC_BootMode.h>
#include <SoftwareSerial.h>

#include "data_type.h"

#define LED_PIN                 2
#define BUTTON_PIN              0
#define PROD_MODE_PIN          13

#define rxPin                  12
#define txPin                  14

char* espnowMsg[300];
bool dirty = false;
u8 currentSleepTimeMinuteByte = 5;

int mode;

CMMC_SimplePair instance;
CMMC_ESPNow espNow;
CMMC_Utils utils;
CMMC_LED led(LED_PIN, LOW);

uint8_t mmm[6];
SoftwareSerial swSerial(rxPin, txPin);

void evt_callback(u8 status, u8* sa, const u8* data);

void start_config_mode() {
  uint8_t* controller_addr = utils.getESPNowControllerMacAddress();
  utils.printMacAddress(controller_addr);
  instance.begin(MASTER_MODE, evt_callback);
  instance.debug([](const char* s) {
    swSerial.println(s);
  });
  instance.set_message(controller_addr, 6);

  instance.start();
}


int counter = 0;
#include <CMMC_RX_Parser.h>
CMMC_RX_Parser parser(&Serial);

uint32_t _time;
// CMMC_PACKET_T pArr[30];


void setup()
{
  Serial.begin(9600);
  swSerial.begin(9600);
  swSerial.println("Controller Mode");
  Serial.println("Controller Mode");
  led.init();
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

  pinMode(PROD_MODE_PIN, INPUT_PULLUP);
  uint32_t wait_config = 1000;
  if (digitalRead(PROD_MODE_PIN) == LOW) {
    wait_config = 0;
  }
  swSerial.printf("wait_config = %d \r\n", wait_config);
  CMMC_BootMode bootMode(&mode, BUTTON_PIN);
  bootMode.init();
  bootMode.check([](int mode) {
    if (mode == BootMode::MODE_CONFIG) {
      start_config_mode();
    }
    else if (mode == BootMode::MODE_RUN) {
      led.high();
      swSerial.print("Initializing... Controller..");
      espNow.init(NOW_MODE_CONTROLLER);
      espNow.on_message_recv([](uint8_t *macaddr, uint8_t *data, uint8_t len) {
        swSerial.print("FROM: ");
        // CMMC::dump(macaddr, 6);
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
        swSerial.printf("sizeof wrapped packet = %d\r\n", sizeof(wrapped));
        // pArr[pArrIdx] = wrapped;
        // pArrIdx = (pArrIdx + 1) % 30;
        // toHexString((u8*)  &wrapped, sizeof(CMMC_PACKET_T), (char*)espnowMsg);
        Serial.write((byte*)&wrapped, sizeof(wrapped));
        // CMMC_Utils::dump((u8*)&wrapped, sizeof(wrapped));
        //swSerial.println(swSerial.write((byte*)&wrapped, sizeof(wrapped)));
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
    swSerial.println("Simple Pair Wait timeout.");
    ESP.reset();
  }

  if (mode == BootMode::MODE_RUN) {
    while (dirty) {
      swSerial.printf("SENT SLEEP_TIME BACK = %u MINUTE\r\n", currentSleepTimeMinuteByte);
      espNow.send(mmm, &currentSleepTimeMinuteByte, 1);
    }
  }

  while(Serial.available()) {
    u8 incomingByte = Serial.read();
    if (incomingByte < 255 && (incomingByte >0)) {
      currentSleepTimeMinuteByte = incomingByte;
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
    swSerial.printf("[CSP_EVENT_SUCCESS] STATUS: %d\r\n", status);
    swSerial.printf("WITH KEY: ");
    utils.dump(data, 16);
    swSerial.printf("WITH MAC: ");
    utils.dump(sa, 6);
    led.high();
    ESP.reset();
  }
  else {
    swSerial.printf("[CSP_EVENT_ERROR] %d: %s\r\n", status, (const char*)data);
  }
}
