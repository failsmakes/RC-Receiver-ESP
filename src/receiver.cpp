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
//  Telemetri çıkışları:
//    • ESP-NOW → Transmitter'a ACK
//    • WiFi UDP port 4211 → Android'e JSON telemetri
//        {"seq":<0-255>,"t":<-100..100>,"s":<-100..100>,"v":<voltaj>,
//         "gg":<gain>,"gd":<direction>,"gc":<gyro_correction>,"gr":<raw_rate>}
//
//  Gyro Ayar Komutları (UDP / ESP-NOW):
//    {"G":throttle,"Y":steer,"GG":gain,"GD":direction}
//      GG: Gyro gain  0–100
//      GD: Gyro yön  +1 veya -1
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

// -----------------------------------------------------------------------------
//  VERİ YAPILARI  (ESP-NOW)
// -----------------------------------------------------------------------------
struct __attribute__((packed)) RCPacket {
  int8_t  throttle;   // -100 … +100
  int8_t  steer;      // -100 … +100
  uint8_t seq;
  int8_t  gyroGain;   // 0–100  (transmitterdan gelen gyro gain)
  int8_t  gyroDir;    // +1 veya -1 (transmitterdan gelen gyro yön)
};

struct __attribute__((packed)) TelemetryPacket {  // ESP-NOW ACK
  uint8_t ack_seq;
  int8_t  ack_throttle;
  int8_t  ack_steer;
  float   ack_vBat;
  uint8_t ack_rssi;
  int8_t  ack_gyroGain;
  int8_t  ack_gyroDir;
};

// -----------------------------------------------------------------------------
//  GLOBAL NESNELER
// -----------------------------------------------------------------------------
RCPacket  current   = {0, 0, 0, GYRO_GAIN_DEFAULT, GYRO_DIRECTION_DEFAULT};
bool      prevFwd   = false;
uint32_t  lastPktMs = 0;
uint8_t   txMac[6]  = TX_MAC;

WiFiUDP   udpCmd;
WiFiUDP   udpTelemetry;
char      udpBuf[128];
IPAddress androidIp;
bool      androidKnown = false;

Servo         steerServo;
SbusOutput    sbus(SBUS_TX_PIN, SBUS_INVERT_SW);
GyroProcessor gyro;

float    vBat          = 0.0f;
uint32_t lastVbatMs    = 0;
uint32_t lastTelemetryMs = 0;

// -----------------------------------------------------------------------------
//  MOTOR  (RZ7886)
// -----------------------------------------------------------------------------
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

void motorDrive(int t) {
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

// -----------------------------------------------------------------------------
//  SERVO
// -----------------------------------------------------------------------------
void servoSetup() {
  steerServo.attach(SERVO_PIN);
  steerServo.write(SERVO_CENTER);
}

void servoUpdate(int s) {
  if (abs(s) <= STEER_DEADBAND) s = 0;
  int deg = map(s, -100, 100, SERVO_MAX_LEFT, SERVO_MAX_RIGHT);
  steerServo.write(deg);
}

// -----------------------------------------------------------------------------
//  SBUS
// -----------------------------------------------------------------------------
void sbusUpdate(int throttle, int finalSteer) {
  sbus.channels[SBUS_CH_THROTTLE] = SbusOutput::rcToSbus(throttle);
  sbus.channels[SBUS_CH_STEER]    = SbusOutput::rcToSbus(finalSteer);
  for (uint8_t i = 2; i < 16; i++)
    sbus.channels[i] = SbusOutput::CH_MID;
  sbus.setFailsafe(false);
}

// -----------------------------------------------------------------------------
//  PİL VOLTAJ OKUMA
// -----------------------------------------------------------------------------
void vbatUpdate() {
  if (millis() - lastVbatMs < VBAT_INTERVAL_MS) return;
  lastVbatMs = millis();

  long sum = 0;
  for (int i = 0; i < VBAT_SAMPLES; i++) {
    sum += analogRead(A0);
    delayMicroseconds(200);
  }
  float adcVal = (float)sum / VBAT_SAMPLES;
  float vAdc   = adcVal * (VBAT_ADC_REF / VBAT_ADC_MAX);
  vBat         = vAdc * ((VBAT_R1 + VBAT_R2) / VBAT_R2);
}

// -----------------------------------------------------------------------------
//  GYRO PARAMETRELERİNİ GÜNCELLE
//  Paket içinden yeni gain/direction değeri geliyorsa gyro'ya uygula
// -----------------------------------------------------------------------------
void applyGyroSettings(const RCPacket& p) {
  gyro.setGain(p.gyroGain);
  gyro.setDirection(p.gyroDir);
}

// -----------------------------------------------------------------------------
//  TÜM ÇIKIŞLARI GÜNCELLE
// -----------------------------------------------------------------------------
void applyRC(const RCPacket& p) {
  // Gyro ayarlarını güncelle
  applyGyroSettings(p);

  // Gyro düzeltmesi uygulanmış steer hesapla
  int finalSteer = gyro.process(p.steer);

  motorDrive(p.throttle);
  servoUpdate(finalSteer);
  sbusUpdate(p.throttle, finalSteer);
  lastPktMs = millis();
}

// -----------------------------------------------------------------------------
//  FAILSAFE
// -----------------------------------------------------------------------------
void applyFailsafe() {
  motorFree();
  prevFwd = false;
  steerServo.write(SERVO_CENTER);
  for (auto& ch : sbus.channels) ch = SbusOutput::CH_MID;
  sbus.setFailsafe(true);
  gyro.resetPid();
}

// -----------------------------------------------------------------------------
//  UDP TELEMETRİ → ANDROID
//  Format: {"seq":N,"t":T,"s":S,"v":V,"gg":GG,"gd":GD,"gc":GC,"gr":GR}
//    seq : son paket sayacı
//    t   : throttle -100..+100
//    s   : nihai steer (gyro uygulanmış) -100..+100
//    v   : pil voltajı
//    gg  : aktif gyro gain (0-100)
//    gd  : aktif gyro direction (+1 / -1)
//    gc  : gyro düzeltme miktarı (-100..+100)
//    gr  : ham gyro açısal hızı (°/s, 1 ondalık)
// -----------------------------------------------------------------------------
void sendTelemetryUdp() {
  if (!androidKnown) return;
  if (millis() - lastTelemetryMs < TELEMETRY_INTERVAL_MS) return;
  lastTelemetryMs = millis();

  char buf[128];
  snprintf(buf, sizeof(buf),
    "{\"seq\":%u,\"t\":%d,\"s\":%d,\"v\":%.2f,\"gg\":%d,\"gd\":%d,\"gc\":%d,\"gr\":%.1f}",
    (unsigned)current.seq,
    (int)current.throttle,
    gyro.getCorrection() + (int)current.steer,  // nihai steer
    vBat,
    gyro.getGain(),
    gyro.getDirection(),
    gyro.getCorrection(),
    gyro.getRawRate()
  );

  udpTelemetry.beginPacket(androidIp, TELEMETRY_PORT);
  udpTelemetry.write((uint8_t*)buf, strlen(buf));
  udpTelemetry.endPacket();
}

// -----------------------------------------------------------------------------
//  ESP-NOW ACK → Transmitter
// -----------------------------------------------------------------------------
void sendEspNowAck(const RCPacket& p) {
  TelemetryPacket tp;
  tp.ack_seq      = p.seq;
  tp.ack_throttle = p.throttle;
  tp.ack_steer    = p.steer;
  tp.ack_vBat     = vBat;
  tp.ack_rssi     = 0;
  tp.ack_gyroGain = (int8_t)gyro.getGain();
  tp.ack_gyroDir  = (int8_t)gyro.getDirection();
  esp_now_send(txMac, (uint8_t*)&tp, sizeof(tp));
}

// -----------------------------------------------------------------------------
//  ESP-NOW CALLBACK
// -----------------------------------------------------------------------------
void onDataRecv(uint8_t* mac, uint8_t* data, uint8_t len) {
  if (len != sizeof(RCPacket)) return;
  RCPacket p;
  memcpy(&p, data, sizeof(p));
  current = p;
  applyRC(current);
  sendEspNowAck(current);
}

// -----------------------------------------------------------------------------
//  UDP KOMUT PARSE
//  Android gönderimi: {"G":throttle,"Y":steer,"GG":gain,"GD":direction}
//  GG ve GD opsiyonel — gelmezse mevcut değer korunur
// -----------------------------------------------------------------------------
void parseUDP(const char* buf, IPAddress senderIp) {
  androidIp    = senderIp;
  androidKnown = true;

  JsonDocument doc;
  if (deserializeJson(doc, buf)) return;

  RCPacket p;
  p.throttle = constrain((int)(doc["G"] | 0), -100, 100);
  p.steer    = constrain((int)(doc["Y"] | 0), -100, 100);
  p.seq      = current.seq + 1;

  // Gyro parametreleri — gelmezse mevcut değer korun
  p.gyroGain = doc["GG"].is<int>()
               ? constrain((int)doc["GG"], 0, 100)
               : (int8_t)gyro.getGain();
  p.gyroDir  = doc["GD"].is<int>()
               ? ((int)doc["GD"] >= 0 ? 1 : -1)
               : (int8_t)gyro.getDirection();

  current = p;
  applyRC(current);
}

// -----------------------------------------------------------------------------
//  SETUP
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  motorSetup();
  servoSetup();

  sbus.begin();
  Serial.println();
  Serial.println("-------------------------------");
  Serial.printf("[SBUS] TX:%d  SW-Inverted:%s\n",
                SBUS_TX_PIN, SBUS_INVERT_SW ? "EVET" : "HAYIR");

  // Gyro başlat
  gyro.begin();

  // WiFi AP
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD, WIFI_AP_CHANNEL);
  Serial.printf("[WiFi] AP: %s  IP: %s  AP MAC: %s  MAC: %s\n",
                WIFI_AP_SSID, WiFi.softAPIP().toString().c_str(),
                WiFi.softAPmacAddress().c_str(), WiFi.macAddress().c_str());

  udpCmd.begin(UDP_PORT);
  udpTelemetry.begin(TELEMETRY_PORT + 100);
  Serial.printf("[UDP] Komut port: %d  Telemetri hedef port: %d\n",
                UDP_PORT, TELEMETRY_PORT);

  // ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("[ESP-NOW] HATA — yeniden başlatılıyor");
    ESP.restart();
  }
  esp_now_register_recv_cb(onDataRecv);
  esp_now_add_peer(txMac, ESP_NOW_ROLE_COMBO, WIFI_AP_CHANNEL, NULL, 0);

  applyFailsafe();
  Serial.println("[RC RECEIVER] Hazır.");
}

// -----------------------------------------------------------------------------
//  LOOP
// -----------------------------------------------------------------------------
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

  // Voltaj oku
  vbatUpdate();

  // Telemetri gönder
  sendTelemetryUdp();

  // SBUS frame
  sbus.update();

  yield();
}
