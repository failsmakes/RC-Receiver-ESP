// =============================================================================
//  RC CAR — RECEIVER  (ESP8266 NodeMCU v3)
//
//  Veri kaynakları (öncelik sırası):
//    1. ESP-NOW  ← Fiziksel transmitter
//    2. WiFi UDP ← Android uygulaması (AP modu)
//
//  Çıkışlar (paralel, her güncellenmede eş zamanlı):
//    • RZ7886 motor sürücü (IN1/IN2 — PWM)
//    • Servo (yön, trim dahil)
//    • SBUS 16 kanal (100kbps inverted UART — SoftwareSerial)
//        CH1 = Throttle  (-100…+100 → 172…1811)
//        CH2 = Steer+Trim (servo ile aynı değer)
//        CH3 = Trim (ham, ölçekli)
//        CH4-CH16 = 992 (merkez, sabit)
//
//  Telemetri (ESP-NOW geri yolu):
//    • ACK → Transmitter'a gönderilir
//
//  SBUS Donanım Notu:
//    SBUS_INVERT_SW = true  → SoftwareSerial inverted, harici inverter YOK
//    SBUS_INVERT_SW = false → GPIO normal, harici NPN/74HC04 gerekli
// =============================================================================

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <espnow.h>
#include <Servo.h>
#include <ArduinoJson.h>
#include "config.h"
#include "SbusOutput.h"

// -----------------------------------------------------------------------------
//  VERİ YAPILARI  (ESP-NOW)
// -----------------------------------------------------------------------------
struct __attribute__((packed)) RCPacket {
  int8_t  throttle;   // -100 … +100
  int8_t  steer;      // -100 … +100
  int8_t  trim;       // -20  … +20
  uint8_t seq;
};

struct __attribute__((packed)) TelemetryPacket {
  uint8_t ack_seq;
  uint8_t rssi;
};

// -----------------------------------------------------------------------------
//  GLOBAL NESNELER
// -----------------------------------------------------------------------------
RCPacket  current   = {0, 0, 0, 0};
bool      prevFwd   = false;
uint32_t  lastPktMs = 0;
uint8_t   txMac[6]  = TX_MAC;

WiFiUDP    udp;
char       udpBuf[128];
Servo      steerServo;
SbusOutput sbus(SBUS_TX_PIN, SBUS_INVERT_SW);

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

void servoUpdate(int s, int trim) {
  if (abs(s) <= STEER_DEADBAND) s = 0;
  int deg = map(s, -100, 100, SERVO_MAX_LEFT, SERVO_MAX_RIGHT);
  deg = constrain(deg + trim, SERVO_MAX_LEFT - 10, SERVO_MAX_RIGHT + 10);
  steerServo.write(deg);
  Serial.printf("Servo: %d\n", deg);
}

// -----------------------------------------------------------------------------
//  SBUS KANALLARI GÜNCELLE
// -----------------------------------------------------------------------------
void sbusUpdate(const RCPacket& p) {
  // CH1 — Throttle: ham gaz değeri
  sbus.channels[SBUS_CH_THROTTLE] = SbusOutput::rcToSbus(p.throttle);

  // CH2 — Steer + Trim: servo ile birebir aynı değer
  int steerTrimmed = constrain((int)p.steer + (int)p.trim, -100, 100);
  sbus.channels[SBUS_CH_STEER] = SbusOutput::rcToSbus(steerTrimmed);

  // CH3 — Trim ham: -20…+20 → SBUS aralığına map
  sbus.channels[SBUS_CH_TRIM_RAW] = (uint16_t)map(
    constrain((int)p.trim, -20, 20),
    -20, 20,
    SbusOutput::CH_MIN, SbusOutput::CH_MAX
  );

  // CH4-CH16 — Merkez sabit
  for (uint8_t i = 3; i < 16; i++)
    sbus.channels[i] = SbusOutput::CH_MID;

  sbus.setFailsafe(false);
}

// -----------------------------------------------------------------------------
//  TÜM ÇIKIŞLARI GÜNCELLE
// -----------------------------------------------------------------------------
void applyRC(const RCPacket& p) {
  motorDrive(p.throttle);
  servoUpdate(p.steer, p.trim);
  sbusUpdate(p);
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
  sbus.channels[SBUS_CH_THROTTLE] = SbusOutput::CH_MID;
  sbus.setFailsafe(true);
}

// -----------------------------------------------------------------------------
//  ESP-NOW CALLBACK
// -----------------------------------------------------------------------------
void sendTelemetry(uint8_t ack_seq) {
  TelemetryPacket tp = {ack_seq, 0};
  esp_now_send(txMac, (uint8_t*)&tp, sizeof(tp));
}

void onDataRecv(uint8_t* mac, uint8_t* data, uint8_t len) {
  if (len != sizeof(RCPacket)) return;
  RCPacket p;
  memcpy(&p, data, sizeof(p));
  current = p;
  applyRC(current);
  sendTelemetry(p.seq);
}

// -----------------------------------------------------------------------------
//  UDP JSON PARSE
// -----------------------------------------------------------------------------
void parseUDP(const char* buf) {
  JsonDocument doc;
  Serial.println(buf);
  if (deserializeJson(doc, buf)) return;
  RCPacket p;
  p.throttle = constrain((int)(doc["G"]    | 0), -100, 100);
  p.steer    = constrain((int)(doc["Y"]    | 0), -100, 100);
  p.trim     = constrain((int)(doc["T"] | 0), TRIM_MIN, TRIM_MAX);
  p.seq      = current.seq + 1;
  current    = p;
  applyRC(current);
  Serial.printf("UDP Packet - Throttle: %d, Steer: %d, Trim: %d, Seq: %d\n", p.throttle, p.steer, p.trim, p.seq);
}

// -----------------------------------------------------------------------------
//  SETUP
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("Merhaba");
  motorSetup();
  servoSetup();

  sbus.begin();

  Serial.printf("[SBUS] TX:%d  SW-Inverted:%s\n",
                SBUS_TX_PIN, SBUS_INVERT_SW ? "EVET" : "HAYIR");
  Serial.println("[SBUS] CH1=Throttle | CH2=Steer+Trim | CH3=TrimRaw | CH4-16=Merkez");

  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD, WIFI_AP_CHANNEL);
  Serial.printf("[WiFi] AP: %s  IP: %s\n",
                WIFI_AP_SSID, WiFi.softAPIP().toString().c_str());

  udp.begin(UDP_PORT);
  Serial.printf("[UDP] Port: %d\n", UDP_PORT);

  if (esp_now_init() != 0) {
    Serial.println("[ESP-NOW] HATA — yeniden başlatılıyor");
  //  ESP.restart();
  }
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_register_recv_cb(onDataRecv);
  esp_now_add_peer(txMac, ESP_NOW_ROLE_COMBO, WIFI_AP_CHANNEL, NULL, 0);

  applyFailsafe();
  Serial.println("[RC RECEIVER] Hazır.");
}

// -----------------------------------------------------------------------------
//  LOOP
// -----------------------------------------------------------------------------
void loop() {
  // UDP
  int len = udp.parsePacket();
  if (len > 0 && len < (int)sizeof(udpBuf)) {
    udp.read(udpBuf, len);
    udpBuf[len] = '\0';
    //Serial.println(udpBuf);
    parseUDP(udpBuf);
  }

  // Failsafe
  if (millis() - lastPktMs > FAILSAFE_MS) {
    applyFailsafe();
  }

  // SBUS frame — 20ms'de bir otomatik gönderir
  sbus.update();

  // Call yield() to prevent ESP8266 watchdog resets during long loops
  yield(); 
}
