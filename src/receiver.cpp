// =============================================================================
//  RC CAR — RECEIVER
//  Desteklenen kartlar : ESP8266, ESP32
//  Desteklenen kaynaklar: ESP-NOW, Android UDP, PS3, PS4
//
//  config.h'den seçim yapılır:
//    BOARD_TYPE       → BOARD_ESP8266 / BOARD_ESP32
//    RX_INPUT_SOURCE  → INPUT_NOPS / INPUT_PS3 / INPUT_PS4
//
//  PS3 kütüphanesi : github.com/jvpernis/esp32-ps3
//  PS4 kütüphanesi : github.com/pablomarquez76/PS4_Controller_Host
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

#include "config.h"
#include <FS_MX1508.h>
#include <Arduino.h>
#if BOARD_TYPE == BOARD_ESP8266
  #include <Servo.h>
#elif BOARD_TYPE == BOARD_ESP32
  #include <ESP32Servo.h>
#endif
#include <ArduinoJson.h>
#include "platform.h"
#include "SbusOutput.h"
#include "GyroProcessor.h"

// =============================================================================
//  PS3 — esp32-ps3 kütüphanesi (sadece ESP32)
//
//  API özeti:
//    Ps3.begin("MAC")          → başlat (MAC: ESP32'nin BT adresi)
//    Ps3.isConnected()         → bool
//    Ps3.attachOnConnect(cb)   → bağlantı callback
//    Ps3.attachOnDisconnect(cb)→ kesinti callback
//    Ps3.data.analog.stick.lx  → int8_t (-128..+127) sol stick X
//    Ps3.data.analog.stick.ly  → int8_t (-128..+127) sol stick Y (aşağı = pozitif)
//    Ps3.data.analog.stick.rx  → int8_t sağ stick X
//    Ps3.data.analog.stick.ry  → int8_t sağ stick Y
//    Ps3.data.analog.button.l2 → uint8_t (0..255)
//    Ps3.data.analog.button.r2 → uint8_t (0..255)
//    Ps3.data.button.l1        → uint8_t (0 veya 1)
//    Ps3.data.button.r1        → uint8_t (0 veya 1)
//    Ps3.data.button.l3        → uint8_t (sol stick bas)
//    Ps3.data.button.cross     → uint8_t
// =============================================================================
#if RX_INPUT_SOURCE == INPUT_PS3
  #include <Ps3Controller.h>

  static bool ps3Connected = false;
  void onPs3Connect()    { ps3Connected = true;  Serial.println("[PS3] Baglandi!"); }
  void onPs3Disconnect() { ps3Connected = false; Serial.println("[PS3] Baglanti kesildi."); }
#endif

// =============================================================================
//  PS4 — PS4_Controller_Host kütüphanesi (sadece ESP32)
//
//  API özeti:
//    PS4.begin()               → başlat (varsayılan MAC)
//    PS4.begin("MAC")          → başlat (belirli MAC)
//    PS4.isConnected()         → bool
//    PS4.attachOnConnect(cb)   → bağlantı callback
//    PS4.attachOnDisconnect(cb)→ kesinti callback
//    PS4.LStickX()             → int8_t sol stick X
//    PS4.LStickY()             → int8_t sol stick Y (zaten çevrilmiş: yukarı = pozitif)
//    PS4.RStickX()             → int8_t sağ stick X
//    PS4.RStickY()             → int8_t sağ stick Y
//    PS4.L2Value()             → uint8_t (0..255)
//    PS4.R2Value()             → uint8_t (0..255)
//    PS4.L1()                  → bool
//    PS4.R1()                  → bool
//    PS4.L3()                  → bool (sol stick bas)
//    PS4.Cross()               → bool
//    PS4.setLed(r,g,b)         → LED rengi ayarla
// =============================================================================
#if RX_INPUT_SOURCE == INPUT_PS4
  #include <PS4Controller.h>

  static bool ps4Connected = false;
  void onPs4Connect()    { ps4Connected = true;  PS4.setLed(0, 255, 0); Serial.println("[PS4] Baglandi!"); }
  void onPs4Disconnect() { ps4Connected = false; Serial.println("[PS4] Baglanti kesildi."); }
#endif
MX1508 motorA(MOTOR_IN1_PIN, MOTOR_IN2_PIN);
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
    return frac * VBAT_ADC_REF * ((VBAT_R1 + VBAT_R2) / VBAT_R2) * VBAT_CF;
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
    _cells   = (_voltage < CELL_DETECT_MIN_V) ? 0
             : (_voltage <= CELL_DETECT_2S_MAX) ? 2 : 3;
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
    motorA.setResolution(MOTOR_PWM_RES);
    motorA.setFrequency(BEEP_FREQUENCY);
    motorA.motorStop();
    for (int rep = 0; rep < BEEP_REPEAT_COUNT; rep++) {
      for (uint8_t i = 0; i < n; i++) {
        motorA.motorGo(BEEP_PWM_VALUE);
        delay(BEEP_ON_MS);
        motorA.motorBrake(100);
        if (i < n - 1) delay(BEEP_OFF_MS);
      }
      if (rep < BEEP_REPEAT_COUNT - 1) delay(BEEP_CELL_PAUSE_MS);
    }
    motorA.motorStop();
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
int8_t    speed = 0;
uint32_t  lastPktMs = 0;

#if RX_INPUT_SOURCE == INPUT_NOPS
  uint8_t txMac[6] = TX_MAC;
#endif

#if RX_INPUT_SOURCE == INPUT_NOPS
  WiFiUDP   udpCmd;
  WiFiUDP   udpTelemetry;
  char      udpBuf[192];
  IPAddress androidIp;
  bool      androidKnown    = false;
  uint32_t  lastTelemetryMs = 0;
#endif

Servo      steerServo;
SbusOutput sbus(SBUS_TX_PIN, SBUS_INVERT_SW);

// PS3/PS4 ortak durum değişkenleri
#if RX_INPUT_SOURCE == INPUT_PS3 || RX_INPUT_SOURCE == INPUT_PS4
  static int  psTrim       = 0;
  static int  psGyroGain   = GYRO_GAIN_DEFAULT;
  static int  psGyroDir    = GYRO_DIRECTION_DEFAULT;
  // Kenar tetikleme için önceki buton durumları
  static bool psL1Prev     = false;
  static bool psR1Prev     = false;
  static bool psL3Prev     = false;
  static bool psCrossPrev  = false;
  static uint32_t psCrossHoldMs = 0;
#endif

// =============================================================================
//  MOTOR (RZ7886)
// =============================================================================
void motorSetup() {
  motorA.setResolution(MOTOR_PWM_RES);
  motorA.setFrequency(MOTOR_PWM_FREQ);
}

void motorDrive(int t) {
  if (battery.isLowVoltage()) { motorA.motorStop(); prevFwd = false; return; }
  if (THROTTLE_RAMP){ 
    if (t>speed) speed += THROTTLE_RAMP;
    if (t<speed) speed -= THROTTLE_RAMP;
    } else speed = t;
  int pwm = map(abs(speed), -100, 100, -1*MOTOR_MAX_RATE, MOTOR_MAX_RATE);
  if (t > THROTTLE_DEADBAND) {
    motorA.motorGoP(pwm);
    if (pwm>0) prevFwd = true;
  } else if (prevFwd && (abs(t) > THROTTLE_DEADBAND)) { motorA.motorBrake(pwm);
  } else if (abs(t) > THROTTLE_DEADBAND) {
      if (pwm<0) prevFwd = false;
      motorA.motorGoP(pwm);
  } else if (abs(t) <= THROTTLE_DEADBAND) { motorA.motorStop(); prevFwd = false; }
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
  motorA.motorStop(); prevFwd = false;
  steerServo.write(SERVO_CENTER);
  for (auto& ch : sbus.channels) ch = SbusOutput::CH_MID;
  sbus.setFailsafe(true);
  gyro.resetPid();
}

// =============================================================================
//  UDP TELEMETRİ
// =============================================================================
#if RX_INPUT_SOURCE == INPUT_NOPS
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
//  ESP-NOW
// =============================================================================
#if RX_INPUT_SOURCE == INPUT_NOPS
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
#endif  // INPUT_ESPNOW

// =============================================================================
//  UDP KOMUT PARSE
// =============================================================================
#if RX_INPUT_SOURCE == INPUT_NOPS
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
//  PS3 KONTROLCÜ İŞLEME — esp32-ps3
//
//  Stick değerleri int8_t (-128..+127) → -100..+100 ölçeklenir
//  LY: yukarı = negatif değer → throttle için ters çevir
//  RX: sağa = pozitif değer → steer olarak direkt kullan
// =============================================================================
#if RX_INPUT_SOURCE == INPUT_PS3
void processPs3() {
  if (!ps3Connected || !Ps3.isConnected()) { return; }
  // ── Stickler → -100..+100 ────────────────────────────────────────────────
  // ly: yukarı = negatif → ters çevir
  int t = constrain((int)Ps3.data.analog.stick.ly * -100 / 127, -100, 100);
  int s = constrain((int)Ps3.data.analog.stick.rx *  100 / 127, -100, 100);
  if (abs(t) < PS_STICK_DEADBAND) t = 0;
  if (abs(s) < PS_STICK_DEADBAND) s = 0;

  // ── L2/R2 → Trim ─────────────────────────────────────────────────────────
  // l2/r2 uint8_t (0..255) — fark → trim
  int l2 = Ps3.data.analog.button.l2;
  int r2 = Ps3.data.analog.button.r2;
  psTrim = map(r2 - l2, -255, 255, -PS_TRIM_SCALE, PS_TRIM_SCALE);
  psTrim = constrain(psTrim, -PS_TRIM_SCALE, PS_TRIM_SCALE);

  // ── L1/R1 → Gyro Gain (kenar tetikli) ────────────────────────────────────
  bool l1 = Ps3.data.button.l1;
  bool r1 = Ps3.data.button.r1;
  if (l1 && !psL1Prev) psGyroGain = constrain(psGyroGain - PS_GYRO_GAIN_STEP, 0, 100);
  if (r1 && !psR1Prev) psGyroGain = constrain(psGyroGain + PS_GYRO_GAIN_STEP, 0, 100);
  psL1Prev = l1;
  psR1Prev = r1;

  // ── L3 (sol stick bas) → Gyro Direction toggle ───────────────────────────
  bool l3 = Ps3.data.button.l3;
  if (l3 && !psL3Prev) {
    psGyroDir = -psGyroDir;
    Serial.printf("[PS3] Gyro dir: %+d\n", psGyroDir);
  }
  psL3Prev = l3;

  // ── Cross basılı tut 1s → Trim sıfırla ───────────────────────────────────
  bool cross = Ps3.data.button.cross;
  if (cross && !psCrossPrev) psCrossHoldMs = millis();
  if (cross && (millis() - psCrossHoldMs > 1000)) { psTrim = 0; }
  psCrossPrev = cross;

  // ── RCPacket oluştur ──────────────────────────────────────────────────────
  current.throttle = (int8_t)t;
  current.steer    = (int8_t)constrain(s + psTrim, -100, 100);
  current.seq++;
  current.gyroGain = (int8_t)psGyroGain;
  current.gyroDir  = (int8_t)psGyroDir;
  applyRC(current);
}
#endif  // INPUT_PS3

// =============================================================================
//  PS4 KONTROLCÜ İŞLEME — PS4_Controller_Host
//
//  LStickY() zaten çevrilmiş: yukarı = pozitif → throttle direkt kullan
//  RStickX() sağa = pozitif → steer olarak direkt kullan
// =============================================================================
#if RX_INPUT_SOURCE == INPUT_PS4
void processPs4() {
  if (!ps4Connected || !PS4.isConnected()) return;

  // ── Stickler → -100..+100 ────────────────────────────────────────────────
  // LStickY() yukarı=pozitif (kütüphane zaten çevirmiş)
  int t = constrain((int)PS4.LStickY() * 100 / 127, -100, 100);
  int s = constrain((int)PS4.RStickX() * 100 / 127, -100, 100);
  if (abs(t) < PS_STICK_DEADBAND) t = 0;
  if (abs(s) < PS_STICK_DEADBAND) s = 0;

  // ── L2/R2 → Trim ─────────────────────────────────────────────────────────
  int l2 = PS4.L2Value();  // uint8_t 0..255
  int r2 = PS4.R2Value();
  psTrim = map(r2 - l2, -255, 255, -PS_TRIM_SCALE, PS_TRIM_SCALE);
  psTrim = constrain(psTrim, -PS_TRIM_SCALE, PS_TRIM_SCALE);

  // ── L1/R1 → Gyro Gain (kenar tetikli) ────────────────────────────────────
  bool l1 = PS4.L1();
  bool r1 = PS4.R1();
  if (l1 && !psL1Prev) psGyroGain = constrain(psGyroGain - PS_GYRO_GAIN_STEP, 0, 100);
  if (r1 && !psR1Prev) psGyroGain = constrain(psGyroGain + PS_GYRO_GAIN_STEP, 0, 100);
  psL1Prev = l1;
  psR1Prev = r1;

  // ── L3 (sol stick bas) → Gyro Direction toggle ───────────────────────────
  bool l3 = PS4.L3();
  if (l3 && !psL3Prev) {
    psGyroDir = -psGyroDir;
    Serial.printf("[PS4] Gyro dir: %+d\n", psGyroDir);
    // Kontrolcüye yön ile renk geri bildirimi: normal=mavi, ters=kırmızı
    if (psGyroDir > 0) PS4.setLed(0, 0, 255);
    else               PS4.setLed(255, 0, 0);
  }
  psL3Prev = l3;

  // ── Cross basılı tut 1s → Trim sıfırla ───────────────────────────────────
  bool cross = PS4.Cross();
  if (cross && !psCrossPrev) psCrossHoldMs = millis();
  if (cross && (millis() - psCrossHoldMs > 1000)) {
    psTrim = 0;
    PS4.setLed(0, 255, 0);  // yeşil: trim sıfırlandı
  }
  psCrossPrev = cross;

  // ── RCPacket oluştur ──────────────────────────────────────────────────────
  current.throttle = (int8_t)t;
  current.steer    = (int8_t)constrain(s + psTrim, -100, 100);
  current.seq++;
  current.gyroGain = (int8_t)psGyroGain;
  current.gyroDir  = (int8_t)psGyroDir;
  applyRC(current);
}
#endif  // INPUT_PS4

// =============================================================================
//  SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== RC RECEIVER BASLIYOR ===");
  Serial.printf("[BOARD] %s\n", (BOARD_TYPE == BOARD_ESP32) ? "ESP32" : "ESP8266");
  Serial.printf("[INPUT] %s\n",
    (RX_INPUT_SOURCE == INPUT_NOPS)  ? "ESP-NOW & Android UDP" :
    (RX_INPUT_SOURCE == INPUT_PS3)     ? "PS3 (esp32-ps3)" : "PS4 (PS4_Controller_Host)");

  // 1. Pil (WiFi/BT öncesi — ADC gürültüsünü önler)
  battery.begin();

  // 2. Motor & Servo
  motorSetup();
  servoSetup();

  // 3. SBUS
  sbus.begin();
  Serial.printf("[SBUS] TX:%d\n", SBUS_TX_PIN);

  // 4. Gyro
  gyro.begin();

  // ── WiFi + UDP (ESPNOW ve ANDROID modları) ────────────────────────────────
#if RX_INPUT_SOURCE == INPUT_NOPS
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD, WIFI_AP_CHANNEL);
  Serial.printf("[WiFi] AP:%s  IP:%s\n",
                WIFI_AP_SSID, WiFi.softAPIP().toString().c_str());
  udpCmd.begin(UDP_PORT);
  udpTelemetry.begin(TELEMETRY_PORT + 100);
#endif

  // ── ESP-NOW peer kaydı ────────────────────────────────────────────────────
#if RX_INPUT_SOURCE == INPUT_NOPS
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

  // ── PS3 — esp32-ps3 ───────────────────────────────────────────────────────
#if RX_INPUT_SOURCE == INPUT_PS3
  Ps3.attachOnConnect(onPs3Connect);
  Ps3.attachOnDisconnect(onPs3Disconnect);

  // PS_BT_MAC boşsa varsayılan MAC ile başlat, doluysa o MAC ile
  if (strlen(PS_BT_MAC) > 0)
    Ps3.begin(PS_BT_MAC);
  else
    Ps3.begin();
  String address = Ps3.getAddress();
  Serial.println("[PS3] Hazir. Kontrolcuyu PS butonu ile baglayın.");
  
    // Get and print the MAC address
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_BT);
  
  Serial.print("[PS3] BT MAC: ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", mac[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.println();

#endif

  // ── PS4 — PS4_Controller_Host ─────────────────────────────────────────────
#if RX_INPUT_SOURCE == INPUT_PS4
  PS4.attachOnConnect(onPs4Connect);
  PS4.attachOnDisconnect(onPs4Disconnect);

  if (strlen(PS_BT_MAC) > 0)
    PS4.begin(PS_BT_MAC);
  else
    PS4.begin();

  Serial.println("[PS4] Hazir. Share + PS butonlariyla baglayın.");
#endif

  applyFailsafe();

  if (battery.cells() == 0)
    Serial.println("[VBAT] Pil yok — motor kalici kilitli");
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

  // ── UDP Komut ────────────────────────────────────────────────────────────
#if RX_INPUT_SOURCE == INPUT_NOPS
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

  // ── PS3 kontrolcü işle ───────────────────────────────────────────────────
#if RX_INPUT_SOURCE == INPUT_PS3
  processPs3();
#endif

  // ── PS4 kontrolcü işle ───────────────────────────────────────────────────
#if RX_INPUT_SOURCE == INPUT_PS4
  processPs4();
#endif

  // Failsafe
  if (millis() - lastPktMs > FAILSAFE_MS) applyFailsafe();

  // Voltaj güncelle
  battery.update();
  if (battery.isLowVoltage()) { motorA.motorStop(); prevFwd = false; }

  // ── UDP Telemetri ─────────────────────────────────────────────────────────
#if RX_INPUT_SOURCE == INPUT_NOPS
  sendTelemetryUdp();
#endif

  // SBUS
  sbus.update();

#if BOARD_TYPE == BOARD_ESP8266
  yield();
#endif
}
