#include "CMMC_BootMode.h"

void CMMC_BootMode::init() {
  pinMode(this->_button_pin, INPUT_PULLUP);
}

void CMMC_BootMode::check(cmmc_boot_mode_cb_t cb, uint32_t wait) {
  delay(wait);
  if (cb != NULL) this->_user_boot_mode_cb = cb;

  if (digitalRead(this->_button_pin) == LOW) {
    *_target_mode  = MODE_CONFIG;
  }
  else {
    *_target_mode  = MODE_RUN;
  }

  // finally fire the cb
  _user_boot_mode_cb(*_target_mode);
}

void CMMC_BootMode::debug(cmmc_debug_cb_t cb) {
  if (cb != NULL) {
    this->_user_debug_cb = cb;
  }
}

uint32_t CMMC_BootMode::calculateCRC32(const uint8_t *data, size_t length) {
  uint32_t crc = 0xffffffff;
  while (length--) {
    uint8_t c = *data++;
    for (uint32_t i = 0x80; i > 0; i >>= 1) {
      bool bit = crc & 0x80000000;
      if (c & i) {
        bit = !bit;
      }
      crc <<= 1;
      if (bit) {
        crc ^= 0x04c11db7;
      }
    }
  }
  return crc;
}

//prints all rtcData, including the leading crc32
void CMMC_BootMode::printMemory() {
  char buf[3];
  uint8_t *ptr = (uint8_t *)&(this->x);
  for (size_t i = 0; i < sizeof(rtcData); i++) {
    sprintf(buf, "%02X", ptr[i]);
    Serial.print(buf);
    if ((i + 1) % 32 == 0) {
      Serial.println();
    } else {
      Serial.print(" ");
    }
  }
  Serial.println();
}
