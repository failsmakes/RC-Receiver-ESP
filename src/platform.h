#pragma once
// =============================================================================
//  platform.h — ESP8266 / ESP32 Soyutlama Katmanı
//
//  Bu dosya iki kartın farklı API'lerini tek bir arayüze indirger:
//    • PWM (analogWrite vs LEDC)
//    • WiFi (ESP8266WiFi vs WiFi)
//    • ESP-NOW (espnow.h vs esp_now.h + callback imzaları)
//    • millis / delay → her iki platformda aynı
// =============================================================================

#include "config.h"

// ─── WiFi ve ESP-NOW include'ları ────────────────────────────────────────────
#if BOARD_TYPE == BOARD_ESP8266
  #include <ESP8266WiFi.h>
  #include <WiFiUdp.h>
  #if RX_INPUT_SOURCE == INPUT_ESPNOW
    #include <espnow.h>
  #endif
#elif BOARD_TYPE == BOARD_ESP32
  #include <WiFi.h>
  #include <WiFiUdp.h>
  #if RX_INPUT_SOURCE == INPUT_ESPNOW
    #include <esp_now.h>
  #endif
#endif

// ─── PWM soyutlaması ─────────────────────────────────────────────────────────
#if BOARD_TYPE == BOARD_ESP8266

  inline void platformPwmSetup() {
    analogWriteFreq(MOTOR_PWM_FREQ);
    analogWriteRange(MOTOR_PWM_MAX);
  }
  inline void platformPwmWrite(uint8_t pin, int value) {
    analogWrite(pin, value);
  }

#elif BOARD_TYPE == BOARD_ESP32

  // LEDC: 2 kanal (IN1, IN2)
  inline void platformPwmSetup() {
    ledcSetup(MOTOR_LEDC_CH1, MOTOR_PWM_FREQ, MOTOR_LEDC_RES);
    ledcSetup(MOTOR_LEDC_CH2, MOTOR_PWM_FREQ, MOTOR_LEDC_RES);
    ledcAttachPin(MOTOR_IN1_PIN, MOTOR_LEDC_CH1);
    ledcAttachPin(MOTOR_IN2_PIN, MOTOR_LEDC_CH2);
  }
  // pin parametresi IN1 veya IN2 pini — ilgili LEDC kanalını seçer
  inline void platformPwmWrite(uint8_t pin, int value) {
    if (pin == MOTOR_IN1_PIN) ledcWrite(MOTOR_LEDC_CH1, value);
    else                      ledcWrite(MOTOR_LEDC_CH2, value);
  }

#endif

// ─── ADC okuma (normalize: 0.0–1.0) ─────────────────────────────────────────
inline float platformAdcRead() {
  long sum = 0;
  for (int i = 0; i < VBAT_SAMPLES; i++) {
    sum += analogRead(VBAT_ADC_PIN);
    delayMicroseconds(300);
  }
  return (float)sum / VBAT_SAMPLES / VBAT_ADC_MAX;
}

// ─── ESP-NOW soyutlaması ─────────────────────────────────────────────────────
#if RX_INPUT_SOURCE == INPUT_ESPNOW

  #if BOARD_TYPE == BOARD_ESP8266
    // ESP8266 gönderme
    inline bool platformEspNowSend(const uint8_t* mac, const uint8_t* data, int len) {
      return esp_now_send((uint8_t*)mac, (uint8_t*)data, len) == 0;
    }

  #elif BOARD_TYPE == BOARD_ESP32
    // ESP32 gönderme
    inline bool platformEspNowSend(const uint8_t* mac, const uint8_t* data, int len) {
      return esp_now_send(mac, data, len) == ESP_OK;
    }
  #endif

  // ESP-NOW callback imza farkını soyutla
  // ESP8266: void cb(uint8_t* mac, uint8_t* data, uint8_t len)
  // ESP32  : void cb(const uint8_t* mac, const uint8_t* data, int len)
  // Receiver.cpp her iki imzayı da derleyebilecek şekilde yazılmıştır.

#endif // INPUT_ESPNOW
