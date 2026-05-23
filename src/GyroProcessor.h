#pragma once
// =============================================================================
//  GyroProcessor.h — MPU6050 tabanlı RC Drift Gyro
//  ESP8266 ve ESP32 uyumlu. Wire.begin() pin argümanları her iki platformda çalışır.
// =============================================================================

#include <Arduino.h>
#include <Wire.h>
#include "config.h"

// ─── PT1 Alçak Geçiren Filtre ────────────────────────────────────────────────
class PT1Filter {
public:
  PT1Filter(float cutoffHz, float loopTimeSec) {
    float o = 2.0f * PI * cutoffHz * loopTimeSec;
    _k = o / (o + 1.0f);
    _lastOut = 0.0f;
  }
  float update(float input) {
    _lastOut += _k * (input - _lastOut);
    return _lastOut;
  }
  void reset() { _lastOut = 0.0f; }
private:
  float _k, _lastOut;
};

// ─── PID Kontrolör ───────────────────────────────────────────────────────────
class PidController {
public:
  PidController(float p, float i, float d, float dFilterHz, float loopTimeSec)
    : _Kp(p), _Ki(i), _Kd(d), _loopTime(loopTimeSec),
      _dFilter(dFilterHz, loopTimeSec)
  { reset(); }

  float update(float gyroRate) {
    float proportional  = gyroRate;
    _integral          += gyroRate * _loopTime;
    _integral           = constrain(_integral, -50.0f, 50.0f);
    float derivative    = (gyroRate - _lastError) / _loopTime;
    float derivFiltered = _dFilter.update(derivative);
    float output        = _Kp * proportional + _Ki * _integral + _Kd * derivFiltered;
    _lastError = gyroRate;
    return output;
  }
  void reset() { _integral = 0.0f; _lastError = 0.0f; _dFilter.reset(); }

private:
  float _Kp, _Ki, _Kd, _loopTime, _integral, _lastError;
  PT1Filter _dFilter;
};

// ─── MPU6050 Sürücüsü ────────────────────────────────────────────────────────
class Mpu6050Driver {
public:
  static constexpr uint8_t ADDR           = GYRO_I2C_ADDRESS;
  static constexpr uint8_t REG_PWR        = 0x6B;
  static constexpr uint8_t REG_GYRO_CFG  = 0x1B;
  static constexpr uint8_t REG_GYRO_X    = 0x43;
  static constexpr uint8_t REG_GYRO_Y    = 0x45;
  static constexpr uint8_t REG_GYRO_Z    = 0x47;
  static constexpr float   LSB           = 32.8f;  // FS_SEL=2 → ±1000°/s

  bool begin() {
    Wire.setClock(400000);
    Wire.beginTransmission(ADDR);
    Wire.write(REG_PWR); Wire.write(0x00);
    if (Wire.endTransmission() != 0) return false;
    delayMicroseconds(200);
    Wire.beginTransmission(ADDR);
    Wire.write(REG_GYRO_CFG); Wire.write(0x02 << 3);
    Wire.endTransmission();
    return true;
  }

  float readAxis(uint8_t axis) {
    uint8_t reg = (axis == 0) ? REG_GYRO_X : (axis == 1) ? REG_GYRO_Y : REG_GYRO_Z;
    Wire.beginTransmission(ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(ADDR, (uint8_t)2);
    int16_t raw = (int16_t)((Wire.read() << 8) | Wire.read());
    return (float)raw / LSB;
  }
};

// ─── Ana Gyro İşleyici ───────────────────────────────────────────────────────
class GyroProcessor {
public:
  GyroProcessor()
    : _gyroFilter(GYRO_LPF_CUTOFF_HZ, GYRO_LOOP_MS / 1000.0f)
    , _pid(GYRO_PID_P, GYRO_PID_I, GYRO_PID_D, GYRO_PID_D_LPF_HZ, GYRO_LOOP_MS / 1000.0f)
    , _gain(GYRO_GAIN_DEFAULT), _direction(GYRO_DIRECTION_DEFAULT)
    , _enabled(true), _lastMs(0), _lastOutput(0), _lastRawRate(0.0f), _lastCorrection(0)
  {}

  bool begin() {
    Wire.begin(GYRO_SDA_PIN, GYRO_SCL_PIN);
    bool ok = _mpu.begin();
    if (!ok) {
      Serial.println("[GYRO] MPU6050 bulunamadi!");
      _enabled = false;
    } else {
      Serial.printf("[GYRO] Hazir. Gain:%d Dir:%+d\n", _gain, _direction);
    }
    return ok;
  }

  int process(int userSteer) {
    if (!_enabled || _gain == 0) return userSteer;
    uint32_t now = millis();
    if (now - _lastMs < (uint32_t)GYRO_LOOP_MS) return _lastOutput;
    _lastMs = now;

    float rawRate      = _mpu.readAxis(GYRO_AXIS);
    float filteredRate = _gyroFilter.update(rawRate);
    float pidOut       = _pid.update(filteredRate);
    float correction   = constrain(pidOut * (_gain / 100.0f) * _direction / GYRO_OUTPUT_SCALE, -100.0f, 100.0f);
    int finalSteer     = constrain((int)(userSteer + correction), -100, 100);

    _lastOutput     = finalSteer;
    _lastRawRate    = rawRate;
    _lastCorrection = int(finalSteer - userSteer);      //correction;
    return finalSteer;
  }

  void setGain(int g)      { _gain      = constrain(g, 0, 100); }
  void setDirection(int d) { _direction = (d >= 0) ? 1 : -1; }
  int   getGain()          const { return _gain; }
  int   getDirection()     const { return _direction; }
  float getRawRate()       const { return _lastRawRate; }
  int getCorrection()      const { return _lastCorrection; }
  bool  isEnabled()        const { return _enabled; }
  void  resetPid()               { _pid.reset(); _gyroFilter.reset(); }

private:
  Mpu6050Driver _mpu;
  PT1Filter     _gyroFilter;
  PidController _pid;
  int _gain, _direction;
  bool _enabled;
  uint32_t _lastMs;
  int _lastOutput;
  float _lastRawRate;
  int _lastCorrection;
};
