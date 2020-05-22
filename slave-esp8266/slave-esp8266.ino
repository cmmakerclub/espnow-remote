#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <CMMC_SimplePair.h>
#include <CMMC_ESPNow.h>
#include <CMMC_LED.h>

#define LED_PIN (2)
CMMC_ESPNow espNow;
CMMC_LED led(LED_PIN, LOW);
CMMC_SimplePair instance;

void dump(const u8* data, size_t size) {
  for (size_t i = 0; i < size - 1; i++) {
    Serial.printf("%02x ", data[i]);
  }
  Serial.printf("%02x", data[size - 1]);
  Serial.println();
}

uint8_t master_mac[6];
bool ready = false;
uint32_t sent_count = 0;


void evt_callback(u8 status, u8* sa, const u8* data) {
  if (status == 0) {
    Serial.printf("[CSP_EVENT_SUCCESS] STATUS: %d\r\n", status);
    Serial.printf("WITH KEY: "); dump(data, 16);
    Serial.printf("FROM MAC: "); dump(sa, 6);
    memcpy(master_mac, sa, 6);
    espNow.init(NOW_MODE_SLAVE);
    espNow.enable_retries(true);
    // static CMMC_LED *led;
    // static ESPNowModule* module;
    // led = ((CMMC_Legend*) os)->getBlinker();
    // led->detach();
    espNow.on_message_sent([](uint8_t *macaddr, u8 status) {
      sent_count++;
      led.toggle();
      // Serial.println("on sent");
      // led.toggle();
    });
    // module = this;
    espNow.on_message_recv([](uint8_t * macaddr, uint8_t * data, uint8_t len) {
      // Serial.println("on recv");
      // user_espnow_sent_at = millis();
      // led.toggle();
      // Serial.printf("RECV: len = %u byte, sleepTime = %lu at(%lu ms)\r\n", len, data[0], millis());
      // module->_go_sleep(data[0]);
    });
    ready = true;
  }
  else {
    Serial.printf("[CSP_EVENT_ERROR] %d: %s\r\n", status, (const char*)data);
  }
}

typedef struct __attribute((__packed__)) {
  uint8_t from[6];
  uint8_t to[6];
  uint8_t type = 0;
  uint8_t type2 = 0;
} CMMC_SENSOR_DATA_T;


CMMC_SENSOR_DATA_T data;

void setup()
{
  wifi_station_set_auto_connect(0);
  led.init();
  Serial.begin(115200);
  instance.debug([](const char* c) {
    Serial.println(c);
  });

  instance.begin(SLAVE_MODE, evt_callback);
  instance.start();
  // delay(2000);
  // delay(5000);
  // 60 01 94 3f f7 aa
}


uint32_t count = 0;
uint32_t prev = millis();
void loop()
{
  count++;
//  Serial.println("XYA");

  if (ready) {
    if (sent_count > 0 &&  sent_count % 100 == 0 ) {
      // Serial.print(sent_count);
      // Serial.println(" msg/s");
      Serial.printf("diff = %lu, rate = %f\r\n", millis() - prev, (float)sent_count/( (millis() - prev)/1000.0));
      sent_count = 0;
      count = 0;
      prev = millis();
    }
    // espNow.send(master_mac, (u8*) &data, sizeof(data), [&]() {
    //   Serial.println("Sent!");
    //     // Serial.printf("espnow sending timeout. sleepTimeM = %lu\r\n", _defaultDeepSleep_m);
    //     // _go_sleep(_defaultDeepSleep_m);
    //   }, 0);
    esp_now_send(master_mac, (u8*) &data, sizeof(data));
    delay(20);
  }

}
