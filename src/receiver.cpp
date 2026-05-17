// =============================================================================
//  RC CAR — RECEIVER
//  Desteklenen kartlar : ESP8266, ESP32
//  Desteklenen kaynaklar: ESP-NOW, Android UDP, PS3, PS4
//
//  config.h'den seçim yapılır:
//    BOARD_TYPE       → BOARD_ESP8266 / BOARD_ESP32
//    RX_INPUT_SOURCE  → INPUT_ESPNOW / INPUT_ANDROID / INPUT_PS3 / INPUT_PS4
//
//  Çıkışlar:
//    • RZ7886 motor sürücü (PWM)
//    • Servo (yön)
//    • SBUS 16 kanal
//
//  Pil yönetimi:
//    • 2S/3S otomatik tespit + motor beep
//    • Düşük voltaj → motor kilidi
// =============================================================================

#include <Arduino.h>
#include <Servo.h>
#include <ArduinoJson.h>
#include "config.h"
#include "platform.h"
#include "SbusOutput.h"
#include "GyroProcessor.h"

// ─── PS3/PS4: Bluepad32 (sadece ESP32) ───────────────────────────────────────
#if RX_INPUT_SOURCE == INPUT_PS3 || RX_INPUT_SOURCE == INPUT_PS4
  #include <Bluepad32.h>
  static GamepadPtr bp32Gamepad = nullptr;

  void onConnectedGamepad(GamepadPtr gp) {
    bp32Gamepad = gp;
    Serial.println("[BT] Kontrolcü baglandi!");
  }
  void onDisconnectedGamepad(GamepadPtr gp) {
    bp32Gamepad = nullptr;
    Serial.println("[BT] Kontrolcü baglantisi kesildi.");
  }
#endif

// =============================================================================
//  PİL YÖNETİCİSİ
// =============================================================================
class BatteryManager {
public:
  BatteryManager()
    : _cells(0), _voltage(0), _cellVoltage(0),
      _lowVoltage(false), _initialized(false), _lastMs(0) {}

  static float readRawVoltage(int samples = VBAT_SAMPLES) {
    float frac = 0;
    for (int i = 0; i < samples; i++) {
      frac += (float)analogRead(VBAT_ADC_PIN) / VBAT_ADC_MAX;
      delayMicroseconds(300);
    }
    frac /= samples;
    return frac * VBAT_ADC_REF * ((VBAT_R1 + VBAT_R2) / VBAT_R2);
  }

  void begin() {
    delay(200);
    _voltage = readRawVoltage(32);
    _cells   = (_voltage < CELL_DETECT_MIN_V) ? 0
             : (_voltage <= CELL_DETECT_2S_MAX) ? 2 : 3;
    _updateCellVoltage();
    _initialized = true;
    Serial.printf("[VBAT] %.2fV  %dS  %.2fV/h  Dusuk:%s\n",
                  _voltage, _cells, _cellVoltage, _lowVoltage ? "EVET" : "HAYIR");
    if (_cells > 0) _playStartupBeep(_cells);
  }

  void update() {
    if (millis() - _lastMs < VBAT_INTERVAL_MS) return;
    _lastMs  = millis();
    _voltage = readRawVoltage(VBAT_SAMPLES);
    _updateCellVoltage();
  }

  float   voltage()      const { return _voltage; }
  float   cellVoltage()  const { return _cellVoltage; }
  uint8_t cells()        const { return _cells; }
  bool    isLowVoltage() const { return _lowVoltage; }

private:
  uint8_t  _cells;
  float    _voltage, _cellVoltage;
  bool     _lowVoltage, _initialized;
  uint32_t _lastMs;

  void _updateCellVoltage() {
    if (_cells == 0) { _cellVoltage = 0; _lowVoltage = true; return; }
    _cellVoltage = _voltage / _cells;
    if (!_lowVoltage && _cellVoltage < CELL_MIN_VOLTAGE) {
      _lowVoltage = true;
      Serial.printf("[VBAT] DUSUK! %.2fV/h — MOTOR KILITLI\n", _cellVoltage);
    } else if (_lowVoltage && _cellVoltage > CELL_RECOVER_VOLTAGE) {
      _lowVoltage = false;
      Serial.printf("[VBAT] Toparlandı %.2fV/h — serbest\n", _cellVoltage);
    }
  }

  void _playStartupBeep(uint8_t n) {
    Serial.printf("[BEEP] %dS → %dx%d darbe\n", n, BEEP_REPEAT_COUNT, n);
    // Pinleri doğrudan hazırla (motorSetup öncesi çağrılabilir)
    pinMode(MOTOR_IN1_PIN, OUTPUT);
    pinMode(MOTOR_IN2_PIN, OUTPUT);
    platformPwmSetup();
    platformPwmWrite(MOTOR_IN1_PIN, 0);
    platformPwmWrite(MOTOR_IN2_PIN, 0);
    for (int rep = 0; rep < BEEP_REPEAT_COUNT; rep++) {
      for (uint8_t i = 0; i < n; i++) {
        platformPwmWrite(MOTOR_IN1_PIN, BEEP_PWM_VALUE);
        delay(BEEP_ON_MS);
        platformPwmWrite(MOTOR_IN1_PIN, 0);
        if (i < n - 1) delay(BEEP_OFF_MS);
      }
      if (rep < BEEP_REPEAT_COUNT - 1) delay(BEEP_CELL_PAUSE_MS);
    }
    platformPwmWrite(MOTOR_IN1_PIN, 0);
    platformPwmWrite(MOTOR_IN2_PIN, 0);
    delay(300);
  }
};

// =============================================================================
//  VERİ YAPILARI
// =============================================================================
struct __attribute__((packed)) RCPacket {
  int8_t  throttle;
  int8_t  steer;
  uint8_t seq;
  int8_t  gyroGain;
  int8_t  gyroDir;
};

struct __attribute__((packed)) TelemetryPacket {
  uint8_t ack_seq;
  int8_t  ack_throttle;
  int8_t  ack_steer;
  float   ack_vBat;
  uint8_t ack_rssi;
  int8_t  ack_gyroGain;
  int8_t  ack_gyroDir;
  uint8_t ack_cells;
  uint8_t ack_lowVoltage;
};

// =============================================================================
//  GLOBAL NESNELER
// =============================================================================
BatteryManager battery;
GyroProcessor  gyro;

RCPacket  current   = {0, 0, 0, GYRO_GAIN_DEFAULT, GYRO_DIRECTION_DEFAULT};
bool      prevFwd   = false;
uint32_t  lastPktMs = 0;

#if RX_INPUT_SOURCE == INPUT_ESPNOW
  uint8_t txMac[6] = TX_MAC;
#endif

#if RX_INPUT_SOURCE == INPUT_ESPNOW || RX_INPUT_SOURCE == INPUT_ANDROID
  WiFiUDP   udpCmd;
  WiFiUDP   udpTelemetry;
  char      udpBuf[192];
  IPAddress androidIp;
  bool      androidKnown    = false;
  uint32_t  lastTelemetryMs = 0;
#endif

Servo      steerServo;
SbusOutput sbus(SBUS_TX_PIN, SBUS_INVERT_SW);

// PS3/PS4 için trim ve gyro ayar durumu
#if RX_INPUT_SOURCE == INPUT_PS3 || RX_INPUT_SOURCE == INPUT_PS4
  int  psTrip       = 0;
  int  psGyroGain   = GYRO_GAIN_DEFAULT;
  int  psGyroDir    = GYRO_DIRECTION_DEFAULT;
  bool psL1Prev     = false;
  bool psR1Prev     = false;
  bool psL3Prev     = false;
  bool psCrossPrev  = false;
  uint32_t psCrossHoldMs = 0;
#endif

// =============================================================================
//  MOTOR (RZ7886)
// =============================================================================
void motorSetup() {
  pinMode(MOTOR_IN1_PIN, OUTPUT);
  pinMode(MOTOR_IN2_PIN, OUTPUT);
  platformPwmSetup();
  platformPwmWrite(MOTOR_IN1_PIN, 0);
  platformPwmWrite(MOTOR_IN2_PIN, 0);
}

void motorFree() {
  platformPwmWrite(MOTOR_IN1_PIN, 0);
  platformPwmWrite(MOTOR_IN2_PIN, 0);
}

void motorBrake() {
  platformPwmWrite(MOTOR_IN1_PIN, MOTOR_PWM_MAX);
  platformPwmWrite(MOTOR_IN2_PIN, MOTOR_PWM_MAX);
}

void motorDrive(int t) {
  if (battery.isLowVoltage()) { motorFree(); prevFwd = false; return; }
  if (abs(t) <= THROTTLE_DEADBAND) { motorFree(); prevFwd = false; return; }

  int pwm = map(abs(t), THROTTLE_DEADBAND, 100, 0, MOTOR_PWM_MAX);
  pwm = constrain(pwm, 0, MOTOR_PWM_MAX);

  if (t > 0) {
    platformPwmWrite(MOTOR_IN1_PIN, pwm);
    platformPwmWrite(MOTOR_IN2_PIN, 0);
    prevFwd = true;
  } else {
    if (prevFwd) { motorBrake(); delay(80); motorFree(); prevFwd = false; }
    else {
      platformPwmWrite(MOTOR_IN1_PIN, 0);
      platformPwmWrite(MOTOR_IN2_PIN, pwm);
    }
  }
}

// =============================================================================
//  SERVO & SBUS
// =============================================================================
void servoSetup() { steerServo.attach(SERVO_PIN); steerServo.write(SERVO_CENTER); }

void servoUpdate(int s) {
  if (abs(s) <= STEER_DEADBAND) s = 0;
  steerServo.write(map(s, -100, 100, SERVO_MAX_LEFT, SERVO_MAX_RIGHT));
}

void sbusUpdate(int t, int s) {
  sbus.channels[SBUS_CH_THROTTLE] = SbusOutput::rcToSbus(t);
  sbus.channels[SBUS_CH_STEER]    = SbusOutput::rcToSbus(s);
  for (uint8_t i = 2; i < 16; i++) sbus.channels[i] = SbusOutput::CH_MID;
  sbus.setFailsafe(false);
}

// =============================================================================
//  ÇIKIŞLARI GÜNCELLE
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

void applyFailsafe() {
  motorFree(); prevFwd = false;
  steerServo.write(SERVO_CENTER);
  for (auto& ch : sbus.channels) ch = SbusOutput::CH_MID;
  sbus.setFailsafe(true);
  gyro.resetPid();
}

// =============================================================================
//  UDP TELEMETRİ
// =============================================================================
#if RX_INPUT_SOURCE == INPUT_ESPNOW || RX_INPUT_SOURCE == INPUT_ANDROID
void sendTelemetryUdp() {
  if (!androidKnown) return;
  if (millis() - lastTelemetryMs < TELEMETRY_INTERVAL_MS) return;
  lastTelemetryMs = millis();
  char buf[192];
  snprintf(buf, sizeof(buf),
    "{\"seq\":%u,\"t\":%d,\"s\":%d,\"v\":%.2f,"
    "\"cells\":%u,\"cv\":%.2f,\"lv\":%u,"
    "\"gg\":%d,\"gd\":%d,\"gc\":%d,\"gr\":%.1f}",
    (unsigned)current.seq, (int)current.throttle,
    gyro.getCorrection() + (int)current.steer,
    battery.voltage(), (unsigned)battery.cells(),
    battery.cellVoltage(), battery.isLowVoltage() ? 1u : 0u,
    gyro.getGain(), gyro.getDirection(),
    gyro.getCorrection(), gyro.getRawRate()
  );
  udpTelemetry.beginPacket(androidIp, TELEMETRY_PORT);
  udpTelemetry.write((uint8_t*)buf, strlen(buf));
  udpTelemetry.endPacket();
}
#endif

// =============================================================================
//  ESP-NOW CALLBACK
// =============================================================================
#if RX_INPUT_SOURCE == INPUT_ESPNOW

void sendEspNowAck(const RCPacket& p) {
  TelemetryPacket tp = {};
  tp.ack_seq        = p.seq;
  tp.ack_throttle   = p.throttle;
  tp.ack_steer      = p.steer;
  tp.ack_vBat       = battery.voltage();
  tp.ack_gyroGain   = gyro.getGain();
  tp.ack_gyroDir    = gyro.getDirection();
  tp.ack_cells      = battery.cells();
  tp.ack_lowVoltage = battery.isLowVoltage() ? 1 : 0;
  platformEspNowSend(txMac, (const uint8_t*)&tp, sizeof(tp));
}

// ESP8266 ve ESP32'nin callback imzaları farklı — her ikisini derle
#if BOARD_TYPE == BOARD_ESP8266
void onDataRecv(uint8_t* mac, uint8_t* data, uint8_t len) {
#elif BOARD_TYPE == BOARD_ESP32
void onDataRecv(const uint8_t* mac, const uint8_t* data, int len) {
#endif
  if ((size_t)len != sizeof(RCPacket)) return;
  memcpy(&current, data, sizeof(RCPacket));
  applyRC(current);
  sendEspNowAck(current);
}

#endif // INPUT_ESPNOW

// =============================================================================
//  UDP KOMUT PARSE  (Android ve ESPNOW modu)
// =============================================================================
#if RX_INPUT_SOURCE == INPUT_ESPNOW || RX_INPUT_SOURCE == INPUT_ANDROID
void parseUDP(const char* buf, IPAddress senderIp) {
  androidIp    = senderIp;
  androidKnown = true;
  JsonDocument doc;
  if (deserializeJson(doc, buf)) return;
  RCPacket p;
  p.throttle = constrain((int)(doc["G"] | 0), -100, 100);
  p.steer    = constrain((int)(doc["Y"] | 0), -100, 100);
  p.seq      = current.seq + 1;
  p.gyroGain = doc["GG"].is<int>() ? constrain((int)doc["GG"], 0, 100) : (int8_t)gyro.getGain();
  p.gyroDir  = doc["GD"].is<int>() ? ((int)doc["GD"] >= 0 ? 1 : -1)   : (int8_t)gyro.getDirection();
  current = p;
  applyRC(current);
}
#endif

// =============================================================================
//  PS3 / PS4 KONTROLCÜ İŞLEME  (Bluepad32)
// =============================================================================
#if RX_INPUT_SOURCE == INPUT_PS3 || RX_INPUT_SOURCE == INPUT_PS4
void processGamepad() {
  if (!bp32Gamepad || !bp32Gamepad->isConnected()) return;

  // ── Analog stickler ──────────────────────────────────────────────────────
  // Sol stick Y → Throttle. Bluepad32: -512..+512 (merkez=0)
  // İleri = negatif Y → throttle pozitif
  int rawThrottle = -bp32Gamepad->axisY();   // sol stick Y, ters çevir = ileri pozitif
  int rawSteer    =  bp32Gamepad->axisRX();  // sağ stick X

  // -512..+512 → -100..+100
  int t = constrain(rawThrottle * 100 / 512, -100, 100);
  int s = constrain(rawSteer    * 100 / 512, -100, 100);

  // Stick deadband
  if (abs(t) < PS_STICK_DEADBAND * 100 / 512) t = 0;
  if (abs(s) < PS_STICK_DEADBAND * 100 / 512) s = 0;

  // ── L2/R2 → Trim ─────────────────────────────────────────────────────────
  // Bluepad32: brake() = L2 (0-1023), throttle() = R2 (0-1023)
  int l2 = bp32Gamepad->brake();    // 0-1023
  int r2 = bp32Gamepad->throttle(); // 0-1023
  psTrip = map(r2 - l2, -1023, 1023, -PS_TRIM_SCALE, PS_TRIM_SCALE);
  psTrip = constrain(psTrip, -PS_TRIM_SCALE, PS_TRIM_SCALE);

  // ── L1/R1 → Gyro Gain ±5 (kenar tetikli) ─────────────────────────────────
  bool l1 = bp32Gamepad->l1();
  bool r1 = bp32Gamepad->r1();
  if (l1 && !psL1Prev) { psGyroGain = constrain(psGyroGain - PS_GYRO_GAIN_STEP, 0, 100); }
  if (r1 && !psR1Prev) { psGyroGain = constrain(psGyroGain + PS_GYRO_GAIN_STEP, 0, 100); }
  psL1Prev = l1; psR1Prev = r1;

  // ── L3 (sol stick bas) → Gyro Direction toggle ───────────────────────────
  bool l3 = bp32Gamepad->thumbL();
  if (l3 && !psL3Prev) { psGyroDir = -psGyroDir; Serial.printf("[PS] Gyro dir: %+d\n", psGyroDir); }
  psL3Prev = l3;

  // ── Cross (×) basılı tut 1s → Trim sıfırla ──────────────────────────────
  bool cross = bp32Gamepad->a();  // Bluepad32: a() = Cross/A
  if (cross && !psCrossPrev) psCrossHoldMs = millis();
  if (cross && (millis() - psCrossHoldMs > 1000)) { psTrip = 0; }
  psCrossPrev = cross;

  // ── RCPacket oluştur ve uygula ────────────────────────────────────────────
  current.throttle = (int8_t)t;
  current.steer    = (int8_t)constrain(s + psTrip, -100, 100);
  current.seq++;
  current.gyroGain = (int8_t)psGyroGain;
  current.gyroDir  = (int8_t)psGyroDir;
  applyRC(current);
}
#endif

// =============================================================================
//  SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== RC RECEIVER BASLIYOR ===");
  Serial.printf("[BOARD] %s\n",
    (BOARD_TYPE == BOARD_ESP32) ? "ESP32" : "ESP8266");
  Serial.printf("[INPUT] %s\n",
    (RX_INPUT_SOURCE == INPUT_ESPNOW)  ? "ESP-NOW" :
    (RX_INPUT_SOURCE == INPUT_ANDROID) ? "Android UDP" :
    (RX_INPUT_SOURCE == INPUT_PS3)     ? "PS3 Bluetooth" : "PS4 Bluetooth");

  // 1. Pil (WiFi/BT öncesi)
  battery.begin();

  // 2. Motor & Servo
  motorSetup();
  servoSetup();

  // 3. SBUS
  sbus.begin();
  Serial.printf("[SBUS] TX:%d\n", SBUS_TX_PIN);

  // 4. Gyro
  gyro.begin();

// ── WiFi + ESP-NOW ────────────────────────────────────────────────────────
#if RX_INPUT_SOURCE == INPUT_ESPNOW || RX_INPUT_SOURCE == INPUT_ANDROID
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD, WIFI_AP_CHANNEL);
  Serial.printf("[WiFi] AP:%s  IP:%s\n",
                WIFI_AP_SSID, WiFi.softAPIP().toString().c_str());
  udpCmd.begin(UDP_PORT);
  udpTelemetry.begin(TELEMETRY_PORT + 100);
#endif

#if RX_INPUT_SOURCE == INPUT_ESPNOW
  #if BOARD_TYPE == BOARD_ESP8266
    if (esp_now_init() != 0) { Serial.println("[ESP-NOW] HATA"); ESP.restart(); }
    esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
    esp_now_register_recv_cb(onDataRecv);
    esp_now_add_peer(txMac, ESP_NOW_ROLE_COMBO, WIFI_AP_CHANNEL, NULL, 0);
  #elif BOARD_TYPE == BOARD_ESP32
    if (esp_now_init() != ESP_OK) { Serial.println("[ESP-NOW] HATA"); ESP.restart(); }
    esp_now_register_recv_cb(onDataRecv);
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, txMac, 6);
    peerInfo.channel = WIFI_AP_CHANNEL;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
  #endif
  Serial.printf("[ESP-NOW] TX MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                txMac[0],txMac[1],txMac[2],txMac[3],txMac[4],txMac[5]);
#endif

// ── Bluetooth PS3/PS4 ─────────────────────────────────────────────────────
#if RX_INPUT_SOURCE == INPUT_PS3 || RX_INPUT_SOURCE == INPUT_PS4
  BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
  BP32.forgetBluetoothKeys();   // önceki eşleşmeleri unut — temiz başlangıç
  #if RX_INPUT_SOURCE == INPUT_PS3
    Serial.println("[BT] PS3 modu — kontrolcüyü PS butonu ile baglayin.");
  #else
    Serial.println("[BT] PS4 modu — kontrolcüyü Share+PS butonuyla baglayin.");
  #endif
#endif

  applyFailsafe();

  if (battery.cells() == 0)
    Serial.println("[VBAT] PIl yok — motor kalici kilitli");
  else
    Serial.printf("[VBAT] %dS  %.2fV  %.2fV/h  %s\n",
      battery.cells(), battery.voltage(), battery.cellVoltage(),
      battery.isLowVoltage() ? "KILITLI" : "HAZIR");

  Serial.println("=== HAZIR ===");
}

// =============================================================================
//  LOOP
// =============================================================================
void loop() {

// ── UDP Komut (ESPNOW + ANDROID modları) ─────────────────────────────────
#if RX_INPUT_SOURCE == INPUT_ESPNOW || RX_INPUT_SOURCE == INPUT_ANDROID
  {
    int len = udpCmd.parsePacket();
    if (len > 0 && len < (int)sizeof(udpBuf)) {
      IPAddress ip = udpCmd.remoteIP();
      udpCmd.read(udpBuf, len);
      udpBuf[len] = '\0';
      parseUDP(udpBuf, ip);
    }
  }
#endif

// ── Gamepad (PS3/PS4) ─────────────────────────────────────────────────────
#if RX_INPUT_SOURCE == INPUT_PS3 || RX_INPUT_SOURCE == INPUT_PS4
  BP32.update();
  processGamepad();
#endif

  // Failsafe
  if (millis() - lastPktMs > FAILSAFE_MS) applyFailsafe();

  // Voltaj güncelle
  battery.update();
  if (battery.isLowVoltage()) { motorFree(); prevFwd = false; }

// ── UDP Telemetri (ESPNOW + ANDROID) ─────────────────────────────────────
#if RX_INPUT_SOURCE == INPUT_ESPNOW || RX_INPUT_SOURCE == INPUT_ANDROID
  sendTelemetryUdp();
#endif

  // SBUS
  sbus.update();

#if BOARD_TYPE == BOARD_ESP8266
  yield();
#endif
}
