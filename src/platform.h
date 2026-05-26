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
    #include <espnow.h>
#elif BOARD_TYPE == BOARD_ESP32
  #include <WiFi.h>
  #include <WiFiUdp.h>
    #include <esp_now.h>
#endif

// ─── ESP-NOW soyutlaması ─────────────────────────────────────────────────────

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

