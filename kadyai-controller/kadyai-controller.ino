#define CMMC_USE_ALIAS

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <CMMC_SimplePair.h>
#include <CMMC_Utils.h>
#include <CMMC_ESPNow.h>
#include <CMMC_LED.h>
#include <CMMC_BootMode.h>
#include "data_type.h"

u8 currentSleepTimeMinuteByte = 60; 
#include <SoftwareSerial.h>
#define rxPin 12
#define txPin 14

bool serialBusy = false;
bool dirty = false;

// SoftwareSerial swSerial(rxPin, txPin, false, 128);
SoftwareSerial swSerial(rxPin, txPin);

#define LED_PIN                 2
#define BUTTON_PIN              0
#define PROD_MODE_PIN         13

int mode;

CMMC_SimplePair instance;
CMMC_ESPNow espNow;
CMMC_Utils utils;
CMMC_LED led(LED_PIN, LOW);

uint8_t mmm[6];

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
void setup_hardware() {
  Serial.begin(57600);
  swSerial.begin(9600);

  Serial.println();
  led.init();
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


void setup()
{
  setup_hardware();
  Serial.println("Controller Mode");
  parser.on_command_arrived([](CMMC_SERIAL_PACKET_T * packet, size_t len) {
    Serial.printf("ON_PARSER at (%lums)", millis());
    Serial.printf("CMD->0x%2x\r\n", packet->cmd);
    Serial.printf("LEN->%lu\r\n", packet->len);
    CMMC::dump((u8*)packet->data, 4);
    CMMC::dump((u8*)packet->data+4, 4);
    Serial.printf("HEAP = %lu\r\n", ESP.getFreeHeap());
    if (packet->cmd == CMMC_SLEEP_TIME_CMD) {
      memcpy(&time, packet->data, 4);
      currentSleepTimeMinuteByte = time; 
      if (time > 255) {
        currentSleepTimeMinuteByte = 254;
      }
    }
  });

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
        serialBusy = true;
        led.toggle(); 
        static CMMC_PACKET_T wrapped; 
        static CMMC_SENSOR_DATA_T packet;
        memcpy(&packet, data, sizeof(packet)); 
        memcpy(&wrapped.data, &packet, sizeof(packet));
        wrapped.ms = millis(); 
        wrapped.sleepTime = currentSleepTimeMinuteByte;
        wrapped.sum = CMMC::checksum((uint8_t*) &wrapped, sizeof(wrapped) - sizeof(wrapped.sum));
        Serial.printf("sizeof wrapped packet = %d\r\n", sizeof(wrapped));
        // Serial.write((byte*)&wrapped, sizeof(wrapped));
        CMMC_Utils::dump((u8*)&wrapped, sizeof(wrapped));
        Serial.println(swSerial.write((byte*)&wrapped, sizeof(wrapped)));
      });

      espNow.on_message_sent([](uint8_t *macaddr,  uint8_t status) {
        serialBusy = false;
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
  if (serialBusy == false) {
    parser.process();
    delay(1);
  }

  while (dirty) {
    Serial.printf("SENT SLEEP_TIME BACK = %u MINUTE\r\n", currentSleepTimeMinuteByte);
    espNow.send(mmm, &currentSleepTimeMinuteByte, 1);
    delay(1);
  }

  ct.timeout_ms(5000);
  while (digitalRead(13) == LOW) {
    if (ct.is_timeout()) {
      ESP.reset();
    }
  }
}
