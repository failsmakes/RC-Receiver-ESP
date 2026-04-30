#pragma once
// =============================================================================
//  RC CAR — RECEIVER CONFIG  (ESP8266 NodeMCU v3)
// =============================================================================

// --- WiFi AP (Android bağlantısı) --------------------------------------------
#define WIFI_AP_SSID      "ESP_RC_CAR"
#define WIFI_AP_PASSWORD  "12345678"          // min 8 karakter
#define WIFI_AP_CHANNEL   1                   // ESP-NOW kanalıyla aynı olmalı!
#define UDP_PORT          4210

// --- ESP-NOW -----------------------------------------------------------------
// Transmitter'ın MAC adresini buraya yazın (transmitter Serial'dan basar)
#define TX_MAC   { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }  // <-- değiştirin!

// --- MOTOR PİNLERİ  (RZ7886 — IN1/IN2 PWM) ----------------------------------
#define MOTOR_IN1_PIN   D1      // GPIO5
#define MOTOR_IN2_PIN   D2      // GPIO4
#define MOTOR_PWM_FREQ  20000   // analogWriteFreq
#define MOTOR_PWM_MAX   255

// --- SERVO PİNİ --------------------------------------------------------------
#define SERVO_PIN        D4     // GPIO2
#define SERVO_CENTER     90
#define SERVO_MAX_LEFT   0
#define SERVO_MAX_RIGHT  180

// --- DEADBAND / TRIM ---------------------------------------------------------
#define THROTTLE_DEADBAND   5
#define STEER_DEADBAND      3
#define TRIM_MIN           -20
#define TRIM_MAX            20

// --- GÜVENLİK TIMEOUT --------------------------------------------------------
// Bu kadar ms içinde paket gelmezse araç durur
#define FAILSAFE_MS   500

// --- SBUS ÇIKIŞI -------------------------------------------------------------
//  TX pini: SoftwareSerial ile herhangi bir dijital pin kullanılabilir.
//  D3 (GPIO0) varsayılan — programlama sırasında bu pini SBUS alıcısından ayırın!
//  Alternatif öneriler: D0(GPIO16), D7(GPIO13)
//
//  SBUS_INVERT_SW = true  → SoftwareSerial inverted mod, harici inverter GEREKMEZ
//  SBUS_INVERT_SW = false → Normal lojik, harici NPN transistör veya 74HC04 gerekir
//
#define SBUS_TX_PIN      D3     // GPIO0
#define SBUS_INVERT_SW   true   // true = yazılımsal inversion (harici devre yok)

//  SBUS Kanal Atamaları (0-indexed)
//  Kanal değerleri: 172 (min) … 992 (merkez) … 1811 (max)
#define SBUS_CH_THROTTLE  0   // CH1 — Gaz
#define SBUS_CH_STEER     1   // CH2 — Yön (trim dahil)
#define SBUS_CH_TRIM_RAW  2   // CH3 — Ham trim değeri (bilgi amaçlı)
// CH4-CH16 (index 3-15): 992 (merkez) sabit — ileriye dönük genişleme
