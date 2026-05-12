#pragma once
// =============================================================================
//  GyroProcessor — ESP8266 için MPU6050 tabanlı RC Drift Gyro
//
//  Algoritma: openRCDG (github.com/dot1nt/openRCDG) projesinden adapte edildi.
//
//  Çalışma prensibi:
//    1. MPU6050'den seçilen eksenin açısal hız değeri okunur (°/s)
//    2. PT1 alçak geçiren filtre ile gürültü azaltılır
//    3. PID kontrolör hata sinyali üretir (hedef: yaw_rate = 0)
//    4. PID çıkışı gain ile ölçeklenir ve direction ile yön ayarlanır
//    5. Ölçeklenen gyro düzeltmesi sürücünün steer komutuna eklenir
//
//  Servo çıkışı:
//    finalSteer = clamp(userSteer + gyroCorrection, -100, 100)
//
//  Gain = 0   → Saf kullanıcı kontrolü (gyro devre dışı)
//  Gain = 100 → Maksimum gyro etkisi
// =============================================================================

#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "config.h"

// -----------------------------------------------------------------------------
//  PT1 Alçak Geçiren Filtre
//  Birinci dereceden IIR filtre — openRCDG filter.h'den adapte
// -----------------------------------------------------------------------------
class PT1Filter {
public:
  PT1Filter(float cutoffHz, float loopTimeSec) {
    float o = 2.0f * PI * cutoffHz * loopTimeSec;
    _k = o / (o + 1.0f);
    _lastOut = 0.0f;
  }

  float update(float input) {
    _lastOut = _lastOut + _k * (input - _lastOut);
    return _lastOut;
  }

  void reset() { _lastOut = 0.0f; }

private:
  float _k;
  float _lastOut;
};

// -----------------------------------------------------------------------------
//  PID Kontrolör
//  openRCDG pid.h/pid.cc'den adapte
//  Setpoint: 0 (araç düz gitmeli)
//  Hata: filtrelenmiş gyro açısal hız değeri
// -----------------------------------------------------------------------------
class PidController {
public:
  PidController(float p, float i, float d, float dFilterHz, float loopTimeSec)
    : _Kp(p), _Ki(i), _Kd(d), _loopTime(loopTimeSec),
      _dFilter(dFilterHz, loopTimeSec)
  {
    reset();
  }

  float update(float gyroRate) {
    float error = gyroRate;

    float proportional = error;

    _integral += error * _loopTime;
    _integral = constrain(_integral, -50.0f, 50.0f);

    float derivative = (error - _lastError) / _loopTime;
    float derivFiltered = _dFilter.update(derivative);

    float output = _Kp * proportional
                 + _Ki * _integral
                 + _Kd * derivFiltered;

    _lastError = error;
    return output;
  }

  void reset() {
    _integral  = 0.0f;
    _lastError = 0.0f;
    _dFilter.reset();
  }

private:
  float _Kp, _Ki, _Kd;
  float _loopTime;
  float _integral;
  float _lastError;
  PT1Filter _dFilter;
};

// -----------------------------------------------------------------------------
//  MPU6050 Sürücüsü
//  openRCDG mpu6050.cc'den adapte — Wire.begin() dışarıda çağrılır
// -----------------------------------------------------------------------------
class Mpu6050Driver {
public:
  static constexpr uint8_t  I2C_ADDR        = GYRO_I2C_ADDRESS;
  static constexpr uint32_t I2C_CLK         = 400000;
  static constexpr uint8_t  REG_PWR_MGMT    = 0x6B;
  static constexpr uint8_t  REG_GYRO_CONFIG = 0x1B;
  static constexpr uint8_t  REG_GYRO_X      = 0x43;
  static constexpr uint8_t  REG_GYRO_Y      = 0x45;
  static constexpr uint8_t  REG_GYRO_Z      = 0x47;
  // FS_SEL = 2 → ±1000°/s, LSB = 32.8 LSB/(°/s)
  static constexpr float    GYRO_LSB        = 32.8f;

  bool begin() {
    Wire.setClock(I2C_CLK);
    // Uyku modundan çık
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(REG_PWR_MGMT);
    Wire.write(0x00);
    if (Wire.endTransmission() != 0) return false;
    delayMicroseconds(200);
    // FS_SEL = 2 → ±1000°/s
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(REG_GYRO_CONFIG);
    Wire.write(0x02 << 3);
    Wire.endTransmission();
    return true;
  }

  // axis: 0=X, 1=Y, 2=Z  →  °/s cinsinden döndürür
  float readAxis(uint8_t axis) {
    uint8_t reg;
    switch (axis) {
      case 0:  reg = REG_GYRO_X; break;
      case 1:  reg = REG_GYRO_Y; break;
      default: reg = REG_GYRO_Z; break;
    }
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(I2C_ADDR, (uint8_t)2);
    int16_t raw = (int16_t)((Wire.read() << 8) | Wire.read());
    return (float)raw / GYRO_LSB;
  }
};

// -----------------------------------------------------------------------------
//  GyroProcessor — Ana Gyro İşleme Sınıfı
// -----------------------------------------------------------------------------
class GyroProcessor {
public:
  GyroProcessor()
    : _gyroFilter(GYRO_LPF_CUTOFF_HZ,    GYRO_LOOP_MS / 1000.0f)
    , _pid(GYRO_PID_P, GYRO_PID_I, GYRO_PID_D,
           GYRO_PID_D_LPF_HZ, GYRO_LOOP_MS / 1000.0f)
    , _gain(GYRO_GAIN_DEFAULT)
    , _direction(GYRO_DIRECTION_DEFAULT)
    , _enabled(true)
    , _lastMs(0)
  {}

  // setup() içinde çağır
  bool begin() {
    Wire.begin(GYRO_SDA_PIN, GYRO_SCL_PIN);
    bool ok = _mpu.begin();
    if (!ok) {
      Serial.println("[GYRO] MPU6050 bulunamadı! SDA/SCL bağlantısını kontrol edin.");
      _enabled = false;
    } else {
      Serial.printf("[GYRO] MPU6050 hazır. Gain:%d Dir:%+d\n", _gain, _direction);
    }
    return ok;
  }

  // loop() içinde çağır — yeterli süre geçmişse işler, aksi halde atlar
  // userSteer: -100..+100 sürücünün yön komutu
  // return   : -100..+100 gyro düzeltmesi uygulanmış nihai yön değeri
  int process(int userSteer) {
    if (!_enabled || _gain == 0) return userSteer;

    uint32_t now = millis();
    if (now - _lastMs < GYRO_LOOP_MS) return _lastOutput;
    _lastMs = now;

    // 1. Ham gyro oku (°/s)
    float rawRate = _mpu.readAxis(GYRO_AXIS);

    // 2. Alçak geçiren filtre
    float filteredRate = _gyroFilter.update(rawRate);

    // 3. PID — setpoint 0, hata = filtrelenmiş gyro hızı
    float pidOut = _pid.update(filteredRate);

    // 4. Gain (0–100) ile ölçekle
    float gainFactor = _gain / 100.0f;

    // 5. Direction uygula ve çıkışı -100..+100 aralığına indir
    float correction = pidOut * gainFactor * _direction / GYRO_OUTPUT_SCALE;
    correction = constrain(correction, -100.0f, 100.0f);

    // 6. Kullanıcı steer + gyro düzeltmesi
    int finalSteer = constrain((int)(userSteer + correction), -100, 100);

    _lastOutput    = finalSteer;
    _lastRawRate   = rawRate;
    _lastCorrection = (int)correction;
    return finalSteer;
  }

  // Ayarları güncelle (Android veya Transmitter'dan gelir)
  void setGain(int gain)           { _gain      = constrain(gain, 0, 100); }
  void setDirection(int direction) { _direction = (direction >= 0) ? 1 : -1; }

  // Durum bilgisi
  int   getGain()        const { return _gain; }
  int   getDirection()   const { return _direction; }
  float getRawRate()     const { return _lastRawRate; }
  int   getCorrection()  const { return _lastCorrection; }
  bool  isEnabled()      const { return _enabled; }

  void resetPid() { _pid.reset(); _gyroFilter.reset(); }

private:
  Mpu6050Driver _mpu;
  PT1Filter     _gyroFilter;
  PidController _pid;

  int   _gain;
  int   _direction;
  bool  _enabled;
  uint32_t _lastMs;

  int   _lastOutput     = 0;
  float _lastRawRate    = 0.0f;
  int   _lastCorrection = 0;
};
