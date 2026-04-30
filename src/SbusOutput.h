#pragma once
// =============================================================================
//  SbusOutput — ESP8266 için 16 Kanal SBUS Encoder + SoftwareSerial Çıkışı
//
//  SBUS Protokol:
//    Baud   : 100 000 bps
//    Format : 8E2  (8-bit, Even parity, 2 stop bit)
//    Sinyal : Inverted UART  (HIGH=0, LOW=1)
//    Frame  : 25 byte, ~3ms aktif + ~14ms boşluk ≈ 20ms periyot
//    Frame yapısı:
//      Byte  0    : 0x0F  (header)
//      Byte  1-22 : 16 kanal × 11 bit, LSB-first bit packing
//      Byte 23    : Flags (bit2=frame_lost, bit3=failsafe_activated)
//      Byte 24    : 0x00  (footer)
//    Kanal değer aralığı: 172 (min) … 992 (merkez) … 1811 (max)
//
//  ESP8266 Hardware UART inversion DESTEKLEMEDİĞİNDEN SoftwareSerial kullanılır.
//
//  DONANIM SEÇENEKLERİ:
//  ┌─────────────────────────────────────────────────────────────┐
//  │ A) SoftwareSerial inverse=true (önerilen, harici devre yok) │
//  │    ESP8266 GPIO → SBUS alıcı pinine direkt bağlayın         │
//  │    config.h: SBUS_INVERT_SW = true                          │
//  ├─────────────────────────────────────────────────────────────┤
//  │ B) SoftwareSerial inverse=false + NPN Transistör            │
//  │    GPIO → 1kΩ → NPN Baz                                     │
//  │    NPN Kolektör → SBUS hattı                                 │
//  │    NPN Emiter → GND                                          │
//  │    SBUS hattı → 10kΩ → 3.3V (pull-up)                      │
//  │    config.h: SBUS_INVERT_SW = false                         │
//  ├─────────────────────────────────────────────────────────────┤
//  │ C) 74HC04 veya benzeri inverter entegresi                   │
//  │    config.h: SBUS_INVERT_SW = false                         │
//  └─────────────────────────────────────────────────────────────┘
//
//  NOT — 8E2 Parity:
//    SoftwareSerial 8N1 ile çalışır (parity üretmez).
//    Çoğu SBUS alıcısı pratikte parity bitini kontrol etmez —
//    bu firmware parity üretmeden çalışır. Sıkı parity kontrolü
//    yapan alıcılar için harici UART çipi (SC16IS750 vb.) gerekir.
// =============================================================================

#include <Arduino.h>
#include <SoftwareSerial.h>

class SbusOutput {
public:
  // ---- Protokol sabitleri (public — dışarıdan erişilebilir) ------------------
  static constexpr uint16_t CH_MIN   = 172;
  static constexpr uint16_t CH_MAX   = 1811;
  static constexpr uint16_t CH_MID   = 992;
  static constexpr uint8_t  CH_COUNT = 16;
  static constexpr uint32_t BAUD     = 100000;
  static constexpr uint32_t FRAME_MS = 20;    // çıkış periyodu

  // ---- Kanal tamponu ----------------------------------------------------------
  uint16_t channels[CH_COUNT];

  // ---- Yapıcı -----------------------------------------------------------------
  // txPin    : SoftwareSerial TX pini (RX kullanılmaz, -1 ver)
  // swInvert : true = sinyal inverted (SBUS doğru lojik), false = normal
  explicit SbusOutput(int txPin, bool swInvert = true)
      : _ss(/*rx=*/-1, /*tx=*/txPin, /*inverse_logic=*/swInvert)
  {
    for (auto& ch : channels) ch = CH_MID;
  }

  // ---- begin — setup() içinde çağırın ----------------------------------------
  void begin() {
    _ss.begin(BAUD);
    _lastMs = 0;
  }

  // ---- update — loop() içinde çağırın ----------------------------------------
  // FRAME_MS aralığında otomatik frame gönderir.
  void update() {
    if (millis() - _lastMs < FRAME_MS) return;
    _lastMs = millis();
    _sendFrame();
  }

  // ---- Kolaylık: RC (-100…+100) → SBUS değeri --------------------------------
  static uint16_t rcToSbus(int rc) {
    rc = constrain(rc, -100, 100);
    return (uint16_t)map(rc, -100, 100, (long)CH_MIN, (long)CH_MAX);
  }

  // ---- Failsafe flag ----------------------------------------------------------
  void setFailsafe(bool fs) { _failsafe = fs; }

private:
  SoftwareSerial _ss;
  uint32_t       _lastMs   = 0;
  bool           _failsafe = false;

  static constexpr uint8_t HEADER      = 0x0F;
  static constexpr uint8_t FOOTER      = 0x00;
  static constexpr uint8_t FRAME_BYTES = 25;

  // ---- Frame oluştur ve gönder ------------------------------------------------
  void _sendFrame() {
    uint8_t frame[FRAME_BYTES] = {};
    _buildFrame(frame);
    _ss.write(frame, FRAME_BYTES);
  }

  // ---- 16 × 11 bit → 22 byte packing -----------------------------------------
  //
  //  SBUS bit düzeni (little-endian, LSB-first per channel):
  //
  //  Byte 1 : [ CH1[7:0] ]
  //  Byte 2 : [ CH2[4:0] | CH1[10:8] ]
  //  Byte 3 : [ CH2[10:5] | ... ]
  //  ...
  //
  void _buildFrame(uint8_t* f) {
    f[0] = HEADER;

    // Bit packing
    uint32_t buf    = 0;   // en fazla 32 bit tampon yeterli
    int      filled = 0;   // tamponda dolu bit sayısı
    int      byteN  = 1;   // f[] yazma pozisyonu

    for (uint8_t ch = 0; ch < CH_COUNT; ch++) {
      uint16_t v = constrain(channels[ch], CH_MIN, CH_MAX);
      buf    |= ((uint32_t)v << filled);
      filled += 11;

      while (filled >= 8) {
        f[byteN++] = (uint8_t)(buf & 0xFF);
        buf    >>= 8;
        filled  -= 8;
      }
    }
    // Kalan bitler (176 mod 8 = 0, yani hiç kalmaz — ama güvenlik için)
    if (filled > 0) f[byteN++] = (uint8_t)(buf & 0xFF);

    // Flags byte (index 23)
    f[23] = _failsafe ? 0x08 : 0x00;  // bit3 = failsafe_activated

    f[24] = FOOTER;
  }
};
