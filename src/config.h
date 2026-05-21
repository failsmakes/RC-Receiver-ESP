#pragma once
// =============================================================================
//  RC CAR — RECEIVER CONFIG
//  ESP8266 ve ESP32 kartlarını destekler.
// =============================================================================

// =============================================================================
//  1. KART SEÇİMİ
// =============================================================================
//  BOARD_ESP8266  → NodeMCU v3 / Wemos D1 mini vb.
//  BOARD_ESP32    → ESP32 DevKit, ESP32-WROOM vb.
//
#define BOARD_ESP8266   1
#define BOARD_ESP32     2
#ifndef BOARD_TYPE
  #define BOARD_TYPE      BOARD_ESP8266   // Default: change via build_flags in platformio.ini
#endif

// =============================================================================
//  2. KONTROL KAYNAĞI SEÇİMİ
// =============================================================================
//  Receiver hangi transmitterdan komut alacak?
//
//  INPUT_ESPNOW   → Fiziksel ESP8266/ESP32 transmitter (RC kumandalı)
//                   Android UDP da paralel kaynak olarak çalışır.
//  INPUT_ANDROID  → Sadece WiFi UDP / Android uygulama (ESP-NOW devre dışı)
//  INPUT_PS3      → PS3 DualShock Bluetooth (sadece ESP32)
//                   Kütüphane: https://github.com/jvpernis/esp32-ps3
//  INPUT_PS4      → PS4 DualShock Bluetooth (sadece ESP32)
//                   Kütüphane: https://github.com/pablomarquez76/PS4_Controller_Host
//
//  NOT: INPUT_PS3 ve INPUT_PS4 yalnızca BOARD_ESP32 ile kullanılabilir.
//
#define INPUT_ESPNOW    1
#define INPUT_ANDROID   2
#define INPUT_PS3       3
#define INPUT_PS4       4

#define RX_INPUT_SOURCE INPUT_ANDROID    // ← buradan değiştir

// Derleme zamanı kontrol
#if (RX_INPUT_SOURCE == INPUT_PS3 || RX_INPUT_SOURCE == INPUT_PS4) && (BOARD_TYPE == BOARD_ESP8266)
  #error "PS3/PS4 kontrolcü desteği yalnizca ESP32 ile kullanilabilir!"
#endif

// =============================================================================
//  3. WiFi & UDP  (INPUT_ESPNOW ve INPUT_ANDROID için)
// =============================================================================
#define WIFI_AP_SSID        "ESP_RC_CAR"
#define WIFI_AP_PASSWORD    "12345678"      // min 8 karakter
#define WIFI_AP_CHANNEL     1               // ESP-NOW kanalıyla aynı olmalı!
#define UDP_PORT            4210
#define TELEMETRY_PORT      4211
#define TELEMETRY_INTERVAL_MS  100

// =============================================================================
//  4. ESP-NOW  (INPUT_ESPNOW için)
// =============================================================================
//  Transmitter'ın MAC adresini buraya yazın (TX Serial'dan basar)
#define TX_MAC   { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }   // ← değiştirin!

// =============================================================================
//  5. PIN TANIMLARI
// =============================================================================

#if BOARD_TYPE == BOARD_ESP8266
  // ── ESP8266 NodeMCU v3 ────────────────────────────────────────────────────
  // Motor (RZ7886 — IN1/IN2 PWM)
  #define MOTOR_IN1_PIN    5     // D1 = GPIO5
  #define MOTOR_IN2_PIN    4     // D2 = GPIO4
  #define MOTOR_PWM_FREQ   20000
  #define MOTOR_PWM_MAX    255
  // Servo
  #define SERVO_PIN        2     // D4 = GPIO2
  // SBUS TX
  #define SBUS_TX_PIN      0     // D3 = GPIO0
  // Gyro I2C  (D1/D2 motor ile çakışır → D6/D7 kullan)
  #define GYRO_SDA_PIN     12    // D6 = GPIO12
  #define GYRO_SCL_PIN     13    // D7 = GPIO13
  // ADC
  #define VBAT_ADC_PIN     A0
  #define VBAT_ADC_MAX     1023.0f
  #define VBAT_ADC_REF     3.3f

#elif BOARD_TYPE == BOARD_ESP32
  // ── ESP32-C3 SUPERMINI (21 pin) ──────────────────────────────────────────────
  // Motor (RZ7886 — IN1/IN2 LEDC PWM)
  #define MOTOR_IN1_PIN    4
  #define MOTOR_IN2_PIN    3
  #define MOTOR_PWM_FREQ   20000
  #define MOTOR_PWM_MAX    255
  #define MOTOR_LEDC_CH1   0    // LEDC kanal 0 → IN1
  #define MOTOR_LEDC_CH2   1    // LEDC kanal 1 → IN2
  #define MOTOR_LEDC_RES   8    // 8-bit (0-255)
  // Servo
  #define SERVO_PIN        7
  // SBUS TX
  #define SBUS_TX_PIN      6
  // Gyro I2C
  #define GYRO_SDA_PIN     8
  #define GYRO_SCL_PIN     9
  // ADC  (ESP32 ADC1_CH6 = GPIO34, giriş only, 12-bit 0-4095)
  #define VBAT_ADC_PIN     5  //0,1,2,3,4,5 ADC pins
  #define VBAT_ADC_MAX     4095.0f
  #define VBAT_ADC_REF     3.3f

#endif

// =============================================================================
//  6. MOTOR & SERVO PARAMETRELER
// =============================================================================
#define SERVO_CENTER     90
#define SERVO_MAX_LEFT   0
#define SERVO_MAX_RIGHT  180
#define THROTTLE_DEADBAND   5
#define STEER_DEADBAND      3
#define FAILSAFE_MS         500

// =============================================================================
//  7. SBUS ÇIKIŞI
// =============================================================================
#define SBUS_INVERT_SW      true   // yazılımsal inversion
#define SBUS_CH_THROTTLE    0
#define SBUS_CH_STEER       1

// =============================================================================
//  8. PİL VOLTAJ ÖLÇÜMÜ
// =============================================================================
//  Gerilim bölücü: Vout = Vbat × R2 / (R1+R2)  < ADC_REF
//  R1=47kΩ, R2=10kΩ → maks 2.2V  (hem 2S hem 3S güvenli)
#define VBAT_R1              47.0f
#define VBAT_R2              10.0f
#define VBAT_SAMPLES         16
#define VBAT_INTERVAL_MS     500
// Hücre tespiti eşikleri
#define CELL_DETECT_MIN_V    4.5f    // altında → pil yok
#define CELL_DETECT_2S_MAX   8.9f    // altında 2S, üstünde 3S
// Düşük voltaj koruması
#define CELL_MIN_VOLTAGE     3.4f    // V/hücre — motor engelleme
#define CELL_RECOVER_VOLTAGE 3.5f    // V/hücre — histerezis açma

// =============================================================================
//  9. MOTOR BEEP (Açılış ses sinyali)
// =============================================================================
#define BEEP_PWM_VALUE       80
#define BEEP_ON_MS           80
#define BEEP_OFF_MS          200
#define BEEP_CELL_PAUSE_MS   700
#define BEEP_REPEAT_COUNT    2

// =============================================================================
//  10. GYRO (MPU6050)
// =============================================================================
#define GYRO_I2C_ADDRESS     0x68
#define GYRO_AXIS            2      // 0=X 1=Y 2=Z(yaw)
#define GYRO_GAIN_DEFAULT    50
#define GYRO_DIRECTION_DEFAULT 1
#define GYRO_LOOP_MS         5
#define GYRO_LPF_CUTOFF_HZ   15.0f
#define GYRO_PID_P           8.0f
#define GYRO_PID_I           0.0f
#define GYRO_PID_D          -0.4f
#define GYRO_PID_D_LPF_HZ   10.0f
#define GYRO_OUTPUT_SCALE    800.0f

// =============================================================================
//  11. PS3 / PS4 KONTROLCÜ AYARLARI
// =============================================================================
//
//  PS3: github.com/jvpernis/esp32-ps3
//    • Ps3.begin("XX:XX:XX:XX:XX:XX") ile başlatılır.
//    • Stickler int8_t (-128..+127), L2/R2 uint8_t (0..255)
//    • Bağlantı: Kontrolcüde PS tuşuna basın.
//
//  PS4: github.com/pablomarquez76/PS4_Controller_Host
//    • PS4.begin() veya PS4.begin("XX:XX:XX:XX:XX:XX") ile başlatılır.
//    • Stickler int8_t (-128..+127, Y ekseni zaten çevrilmiş), L2/R2 uint8_t (0..255)
//    • Bağlantı: Kontrolcüde Share + PS tuşuna aynı anda basın.
//
//  PS_BT_MAC: ESP32'nin Bluetooth MAC adresi.
//    "" (boş string) bırakırsanız varsayılan MAC kullanılır.
//    PS3 kütüphanesi belirli bir MAC bekler — eşleştirme için gereklidir.
//    esp32-ps3 README'sindeki "Ps3Address" örneğini çalıştırıp MAC'i öğrenin.
//
#define PS_BT_MAC   "5c:6d:20:3f:d2:f8"   // örn: "01:02:03:04:05:06" — boş = varsayılan MAC

//  Thumbstick deadband: int8_t değer aralığında (-128..+127)
//  Merkez etrafı bu değer kadar serbest bölge
#define PS_STICK_DEADBAND    10

//  Gyro gain adımı: L1 azaltır, R1 artırır (her basışta)
#define PS_GYRO_GAIN_STEP    5

//  Trim ölçeği: L2/R2 analog (0..255) → trim değeri (0..PS_TRIM_SCALE)
//  Net trim = R2-L2 farkından hesaplanır
#define PS_TRIM_SCALE        30

//  Düğme eşlemeleri (her iki kontrolcüde aynı mantık):
//    Sol stick Y  → Gaz (ileri/geri)
//    Sağ stick X  → Yön (sol/sağ)
//    L2 analog    → Trim sola
//    R2 analog    → Trim sağa
//    L1           → Gyro gain -5 (kenar tetikli)
//    R1           → Gyro gain +5 (kenar tetikli)
//    L3 (sol bas) → Gyro direction toggle (kenar tetikli)
//    Cross (×)    → 1s basılı tut → Trim sıfırla
