// =============================================================================
//  RC CAR — RECEIVER  (ESP8266 NodeMCU v3)
//
//  Veri kaynakları (öncelik sırası):
//    1. ESP-NOW  ← Fiziksel transmitter
//    2. WiFi UDP port 4210 ← Android uygulaması (AP modu)
//
//  Çıkışlar (paralel):
//    • RZ7886 motor sürücü (IN1/IN2 — PWM)
//    • Servo (yön) — gyro düzeltmesi uygulanmış
//    • SBUS 16 kanal (SoftwareSerial inverted)
//
//  Pil Yönetimi:
//    • Açılışta voltaj ölçülerek 2S/3S otomatik tespit edilir
//    • Hücre sayısı kadar motor titreşim darbesi (bip sesi) üretilir
//    • Hücre başına voltaj CELL_MIN_VOLTAGE altına düşerse motor kilitlenir
//    • Voltaj CELL_RECOVER_VOLTAGE üzerine çıkana kadar kilit açılmaz
//
//  Telemetri (UDP JSON):
//    {"seq":N,"t":T,"s":S,"v":V,"cells":C,"cv":CV,"lv":B,
//     "gg":GG,"gd":GD,"gc":GC,"gr":GR}
//      cells : tespit edilen hücre sayısı (0/2/3)
//      cv    : hücre başına voltaj
//      lv    : düşük voltaj kilidi aktif mi (0/1)
// =============================================================================

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <espnow.h>
#include <Servo.h>
#include <ArduinoJson.h>
#include "config.h"
#include "SbusOutput.h"
#include "GyroProcessor.h"

// =============================================================================
//  PİL YÖNETİCİSİ
// =============================================================================
class BatteryManager {
public:
  BatteryManager() : _cells(0), _voltage(0.0f), _cellVoltage(0.0f),
                     _lowVoltage(false), _initialized(false), _lastMs(0) {}

  // ── Ham ADC → gerçek pil voltajı (gerilim bölücü kompanzasyonu) ──────────
  static float readRawVoltage(int samples = VBAT_SAMPLES) {
    long sum = 0;
    for (int i = 0; i < samples; i++) {
      sum += analogRead(A0);
      delayMicroseconds(300);
    }
    float adcVal = (float)sum / samples;
    float vAdc   = adcVal * (VBAT_ADC_REF / VBAT_ADC_MAX);
    return vAdc * ((VBAT_R1 + VBAT_R2) / VBAT_R2);
  }

  // ── Açılışta hücre tespiti + beep ────────────────────────────────────────
  //  setup() içinde, WiFi başlamadan ÖNCE çağrılmalı.
  //  (WiFi init ADC'yi gürültülendirebilir)
  void begin() {
    delay(200);                           // kapasitör şarj süresi
    _voltage = readRawVoltage(32);        // fazla örnek → kararlı ölçüm

    // Hücre sayısını voltajdan belirle
    if (_voltage < CELL_DETECT_MIN_V) {
      _cells = 0;
      Serial.printf("[VBAT] Pil tespit edilemedi: %.2fV\n", _voltage);
    } else if (_voltage <= CELL_DETECT_2S_MAX) {
      _cells = 2;
    } else {
      _cells = 3;
    }

    _updateCellVoltage();
    _initialized = true;

    Serial.printf("[VBAT] %.2fV  %dS  %.2fV/hücre  Düşük:%s\n",
                  _voltage, _cells, _cellVoltage,
                  _lowVoltage ? "EVET" : "HAYIR");

    if (_cells > 0) {
      _playStartupBeep(_cells);
    }
  }

  // ── Periyodik voltaj güncellemesi (loop içinde) ───────────────────────────
  void update() {
    if (millis() - _lastMs < VBAT_INTERVAL_MS) return;
    _lastMs = millis();
    _voltage = readRawVoltage(VBAT_SAMPLES);
    _updateCellVoltage();
  }

  // Getters
  float   voltage()       const { return _voltage; }
  float   cellVoltage()   const { return _cellVoltage; }
  uint8_t cells()         const { return _cells; }
  bool    isLowVoltage()  const { return _lowVoltage; }
  bool    isInitialized() const { return _initialized; }

  // 0.0–1.0 doluluk seviyesi (Android UI için)
  float levelFraction() const {
    if (_cells == 0) return 0.0f;
    return constrain((_cellVoltage - CELL_MIN_VOLTAGE) /
                     (4.2f - CELL_MIN_VOLTAGE), 0.0f, 1.0f);
  }

private:
  uint8_t  _cells;
  float    _voltage;
  float    _cellVoltage;
  bool     _lowVoltage;
  bool     _initialized;
  uint32_t _lastMs;

  // ── Hücre voltajını güncelle + histerezis kilidi ─────────────────────────
  void _updateCellVoltage() {
    if (_cells == 0) {
      _cellVoltage = 0.0f;
      _lowVoltage  = true;
      return;
    }
    _cellVoltage = _voltage / (float)_cells;

    if (!_lowVoltage && _cellVoltage < CELL_MIN_VOLTAGE) {
      _lowVoltage = true;
      Serial.printf("[VBAT] ⚠ DÜŞÜK VOLTAJ %.2fV/hücre — MOTOR KİLİTLENDİ\n",
                    _cellVoltage);
    } else if (_lowVoltage && _cellVoltage > CELL_RECOVER_VOLTAGE) {
      _lowVoltage = false;
      Serial.printf("[VBAT] Voltaj toparlandi %.2fV/hücre — motor serbest\n",
                    _cellVoltage);
    }
  }

  // ── Açılış bip: hücre sayısı kadar motor titreşim darbesi ────────────────
  //
  //  Her "bip" = BEEP_ON_MS süre motor titreşimi → buz açık → sessizlik
  //  Grup sonu bekleme = BEEP_CELL_PAUSE_MS
  //  BEEP_REPEAT_COUNT kez tekrar edilir
  //
  //  Örnek 2S, 2 tekrar:
  //    [■■][■■]  ─── bekleme ───  [■■][■■]
  //  Örnek 3S, 2 tekrar:
  //    [■■■][■■■]  ─── bekleme ───  [■■■][■■■]
  //
  void _playStartupBeep(uint8_t cellCount) {
    Serial.printf("[BEEP] %dS → %d×%d darbe\n",
                  cellCount, BEEP_REPEAT_COUNT, cellCount);

    // Motor pinlerini ayarla (motorSetup() henüz çağrılmamış olabilir)
    pinMode(MOTOR_IN1_PIN, OUTPUT);
    pinMode(MOTOR_IN2_PIN, OUTPUT);
    analogWriteFreq(MOTOR_PWM_FREQ);
    analogWriteRange(MOTOR_PWM_MAX);
    analogWrite(MOTOR_IN1_PIN, 0);
    analogWrite(MOTOR_IN2_PIN, 0);

    for (int rep = 0; rep < BEEP_REPEAT_COUNT; rep++) {
      for (uint8_t i = 0; i < cellCount; i++) {
        // Kısa ileri darbe — motor titreşir, mekanik bip sesi üretir
        analogWrite(MOTOR_IN1_PIN, BEEP_PWM_VALUE);
        analogWrite(MOTOR_IN2_PIN, 0);
        delay(BEEP_ON_MS);
        // Sustur
        analogWrite(MOTOR_IN1_PIN, 0);
        analogWrite(MOTOR_IN2_PIN, 0);
        // Darbeler arası boşluk (son darbeden sonra yapma)
        if (i < cellCount - 1) delay(BEEP_OFF_MS);
      }
      // Gruplar arası bekleme (son tekrarda yapma)
      if (rep < BEEP_REPEAT_COUNT - 1) delay(BEEP_CELL_PAUSE_MS);
    }

    // Güvenlik: motoru sıfırla
    analogWrite(MOTOR_IN1_PIN, 0);
    analogWrite(MOTOR_IN2_PIN, 0);
    delay(300);
  }
};

// =============================================================================
//  VERİ YAPILARI  (ESP-NOW)
// =============================================================================
struct __attribute__((packed)) RCPacket {
  int8_t  throttle;    // -100 … +100
  int8_t  steer;       // -100 … +100
  uint8_t seq;
  int8_t  gyroGain;    // 0–100
  int8_t  gyroDir;     // +1 veya -1
};

struct __attribute__((packed)) TelemetryPacket {
  uint8_t ack_seq;
  int8_t  ack_throttle;
  int8_t  ack_steer;
  float   ack_vBat;
  uint8_t ack_rssi;
  int8_t  ack_gyroGain;
  int8_t  ack_gyroDir;
  uint8_t ack_cells;        // tespit edilen hücre sayısı
  uint8_t ack_lowVoltage;   // 0 = normal, 1 = kilitli
};

// =============================================================================
//  GLOBAL NESNELER
// =============================================================================
BatteryManager battery;
GyroProcessor  gyro;

RCPacket  current   = {0, 0, 0, GYRO_GAIN_DEFAULT, GYRO_DIRECTION_DEFAULT};
bool      prevFwd   = false;
uint32_t  lastPktMs = 0;
uint8_t   txMac[6]  = TX_MAC;

WiFiUDP   udpCmd;
WiFiUDP   udpTelemetry;
char      udpBuf[128];
IPAddress androidIp;
bool      androidKnown    = false;
uint32_t  lastTelemetryMs = 0;

Servo      steerServo;
SbusOutput sbus(SBUS_TX_PIN, SBUS_INVERT_SW);

// =============================================================================
//  MOTOR  (RZ7886)
// =============================================================================
void motorSetup() {
  pinMode(MOTOR_IN1_PIN, OUTPUT);
  pinMode(MOTOR_IN2_PIN, OUTPUT);
  analogWriteFreq(MOTOR_PWM_FREQ);
  analogWriteRange(MOTOR_PWM_MAX);
  analogWrite(MOTOR_IN1_PIN, 0);
  analogWrite(MOTOR_IN2_PIN, 0);
}

void motorFree() {
  analogWrite(MOTOR_IN1_PIN, 0);
  analogWrite(MOTOR_IN2_PIN, 0);
}

void motorBrake() {
  analogWrite(MOTOR_IN1_PIN, MOTOR_PWM_MAX);
  analogWrite(MOTOR_IN2_PIN, MOTOR_PWM_MAX);
}

// Düşük voltaj kilidi dahil motor sürme
void motorDrive(int t) {
  if (battery.isLowVoltage()) {
    motorFree();
    prevFwd = false;
    return;
  }
  if (abs(t) <= THROTTLE_DEADBAND) {
    motorFree();
    prevFwd = false;
    return;
  }
  int pwm = map(abs(t), THROTTLE_DEADBAND, 100, 0, MOTOR_PWM_MAX);
  pwm = constrain(pwm, 0, MOTOR_PWM_MAX);

  if (t > 0) {
    analogWrite(MOTOR_IN1_PIN, pwm);
    analogWrite(MOTOR_IN2_PIN, 0);
    prevFwd = true;
  } else {
    if (prevFwd) {
      motorBrake();
      delay(80);
      motorFree();
      prevFwd = false;
    } else {
      analogWrite(MOTOR_IN1_PIN, 0);
      analogWrite(MOTOR_IN2_PIN, pwm);
      prevFwd = false;
    }
  }
}

// =============================================================================
//  SERVO
// =============================================================================
void servoSetup() {
  steerServo.attach(SERVO_PIN);
  steerServo.write(SERVO_CENTER);
}

void servoUpdate(int s) {
  if (abs(s) <= STEER_DEADBAND) s = 0;
  int deg = map(s, -100, 100, SERVO_MAX_LEFT, SERVO_MAX_RIGHT);
  steerServo.write(deg);
}

// =============================================================================
//  SBUS
// =============================================================================
void sbusUpdate(int throttle, int finalSteer) {
  sbus.channels[SBUS_CH_THROTTLE] = SbusOutput::rcToSbus(throttle);
  sbus.channels[SBUS_CH_STEER]    = SbusOutput::rcToSbus(finalSteer);
  for (uint8_t i = 2; i < 16; i++)
    sbus.channels[i] = SbusOutput::CH_MID;
  sbus.setFailsafe(false);
}

// =============================================================================
//  TÜM ÇIKIŞLARI GÜNCELLE
// =============================================================================
void applyRC(const RCPacket& p) {
  gyro.setGain(p.gyroGain);
  gyro.setDirection(p.gyroDir);

  int finalSteer = gyro.process(p.steer);

  motorDrive(p.throttle);
  servoUpdate(finalSteer);
  sbusUpdate(p.throttle, finalSteer);
  lastPktMs = millis();
}

// =============================================================================
//  FAILSAFE
// =============================================================================
void applyFailsafe() {
  motorFree();
  prevFwd = false;
  steerServo.write(SERVO_CENTER);
  for (auto& ch : sbus.channels) ch = SbusOutput::CH_MID;
  sbus.setFailsafe(true);
  gyro.resetPid();
}

// =============================================================================
//  UDP TELEMETRİ → ANDROID
// =============================================================================
void sendTelemetryUdp() {
  if (!androidKnown) return;
  if (millis() - lastTelemetryMs < TELEMETRY_INTERVAL_MS) return;
  lastTelemetryMs = millis();

  char buf[192];
  snprintf(buf, sizeof(buf),
    "{\"seq\":%u,\"t\":%d,\"s\":%d,\"v\":%.2f,"
    "\"cells\":%u,\"cv\":%.2f,\"lv\":%u,"
    "\"gg\":%d,\"gd\":%d,\"gc\":%d,\"gr\":%.1f}",
    (unsigned)current.seq,
    (int)current.throttle,
    gyro.getCorrection() + (int)current.steer,
    battery.voltage(),
    (unsigned)battery.cells(),
    battery.cellVoltage(),
    battery.isLowVoltage() ? 1u : 0u,
    gyro.getGain(),
    gyro.getDirection(),
    gyro.getCorrection(),
    gyro.getRawRate()
  );

  udpTelemetry.beginPacket(androidIp, TELEMETRY_PORT);
  udpTelemetry.write((uint8_t*)buf, strlen(buf));
  udpTelemetry.endPacket();
}

// =============================================================================
//  ESP-NOW ACK → Transmitter
// =============================================================================
void sendEspNowAck(const RCPacket& p) {
  TelemetryPacket tp;
  tp.ack_seq        = p.seq;
  tp.ack_throttle   = p.throttle;
  tp.ack_steer      = p.steer;
  tp.ack_vBat       = battery.voltage();
  tp.ack_rssi       = 0;
  tp.ack_gyroGain   = (int8_t)gyro.getGain();
  tp.ack_gyroDir    = (int8_t)gyro.getDirection();
  tp.ack_cells      = battery.cells();
  tp.ack_lowVoltage = battery.isLowVoltage() ? 1 : 0;
  esp_now_send(txMac, (uint8_t*)&tp, sizeof(tp));
}

// =============================================================================
//  ESP-NOW CALLBACK
// =============================================================================
void onDataRecv(uint8_t* mac, uint8_t* data, uint8_t len) {
  if (len != sizeof(RCPacket)) return;
  RCPacket p;
  memcpy(&p, data, sizeof(p));
  current = p;
  applyRC(current);
  sendEspNowAck(current);
}

// =============================================================================
//  UDP KOMUT PARSE
// =============================================================================
void parseUDP(const char* buf, IPAddress senderIp) {
  androidIp    = senderIp;
  androidKnown = true;

  JsonDocument doc;
  if (deserializeJson(doc, buf)) return;

  RCPacket p;
  p.throttle = constrain((int)(doc["G"] | 0), -100, 100);
  p.steer    = constrain((int)(doc["Y"] | 0), -100, 100);
  p.seq      = current.seq + 1;
  p.gyroGain = doc["GG"].is<int>()
               ? constrain((int)doc["GG"], 0, 100)
               : (int8_t)gyro.getGain();
  p.gyroDir  = doc["GD"].is<int>()
               ? ((int)doc["GD"] >= 0 ? 1 : -1)
               : (int8_t)gyro.getDirection();

  current = p;
  applyRC(current);
}

// =============================================================================
//  SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("=== RC RECEIVER BAŞLATIYOR ===");

  // 1. PİL — WiFi'dan önce! ADC gürültüsünü önlemek için
  battery.begin();

  // 2. MOTOR & SERVO
  motorSetup();
  servoSetup();

  // 3. SBUS
  sbus.begin();
  Serial.printf("[SBUS] TX:%d  Inverted:%s\n",
                SBUS_TX_PIN, SBUS_INVERT_SW ? "EVET" : "HAYIR");

  // 4. GYRO
  gyro.begin();

  // 5. WiFi AP
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD, WIFI_AP_CHANNEL);
  Serial.printf("[WiFi] AP:%s  IP:%s  MAC:%s\n",
                WIFI_AP_SSID,
                WiFi.softAPIP().toString().c_str(),
                WiFi.macAddress().c_str());

  udpCmd.begin(UDP_PORT);
  udpTelemetry.begin(TELEMETRY_PORT + 100);

  // 6. ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("[ESP-NOW] HATA — yeniden başlatılıyor");
    ESP.restart();
  }
  esp_now_register_recv_cb(onDataRecv);
  esp_now_add_peer(txMac, ESP_NOW_ROLE_COMBO, WIFI_AP_CHANNEL, NULL, 0);

  applyFailsafe();

  // Özet
  if (battery.cells() == 0) {
    Serial.println("[VBAT] ⛔ Pil yok — motor kalıcı kilitli");
  } else if (battery.isLowVoltage()) {
    Serial.printf("[VBAT] ⚠ %dS pil düşük (%.2fV/hücre) — kilitli\n",
                  battery.cells(), battery.cellVoltage());
  } else {
    Serial.printf("[VBAT] ✓ %dS  %.2fV  %.2fV/hücre — hazır\n",
                  battery.cells(), battery.voltage(), battery.cellVoltage());
  }

  Serial.println("=== HAZIR ===");
}

// =============================================================================
//  LOOP
// =============================================================================
void loop() {
  // UDP komut
  int len = udpCmd.parsePacket();
  if (len > 0 && len < (int)sizeof(udpBuf)) {
    IPAddress senderIp = udpCmd.remoteIP();
    udpCmd.read(udpBuf, len);
    udpBuf[len] = '\0';
    parseUDP(udpBuf, senderIp);
  }

  // Failsafe
  if (millis() - lastPktMs > FAILSAFE_MS) {
    applyFailsafe();
  }

  // Voltaj güncelle (histerezis kilidi burada güncellenir)
  battery.update();

  // Düşük voltaj durumunda motoru anında durdur
  if (battery.isLowVoltage()) {
    motorFree();
    prevFwd = false;
  }

  // Telemetri gönder
  sendTelemetryUdp();

  // SBUS frame
  sbus.update();

  yield();
}
