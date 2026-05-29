#pragma once
// =============================================================================
//  GyroProcessor.h — ArduDrift algoritmasından adapte edilmiş RC Drift Gyro
//
//  Orijinal: github.com/JetMagpie/ArduDrift
//
//  Algoritma özeti:
//    1. MPU6050'den gyro (Z ekseni, °/s) ve ivmemetre (X,Y eksenleri, g) okunur
//    2. PT1 filtreler ile gürültü temizlenir
//    3. Açısal hız (gyro) + açısal ivme tahmini + yatay ivme (kayma yönü düzeltmesi)
//       birleşik olarak karşı direksiyon (counter-steer) hesaplar
//    4. Açı integrali ile uzun süreli drift hafızası tutulur
//    5. S-curve (üstel eğri) ile doğrusal olmayan hassasiyet ayarı
//    6. Sonuç servo düzeltmesine ve isteğe bağlı olarak throttle'a uygulanır
//
//  Seri port komutları (runtime ayar):
//    gyro report on/off       → anlık veri akışı
//    gyro set PARAM DEGER     → parametre değiştir (örn: gyro set GAIN 0.003)
//    gyro get PARAM           → parametre oku
//    gyro throttle on/off     → throttle etkisini aç/kapat
//    gyro help                → komut listesi
//
//  Throttle etkisi (GYRO_THROTTLE_EFFECT):
//    Araç kayarken throttle hafifçe kesilir → daha uzun, kontrollü drift
//    Serial'dan "gyro throttle on/off" ile toggle edilir
// =============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "config.h"

// =============================================================================
//  PT1 Alçak Geçiren Filtre
// =============================================================================
class PT1Filter {
public:
  PT1Filter() : _alpha(0), _out(0), _init(false) {}

  void setup(float cutoffHz, float sampleHz) {
    float dt = 1.0f / sampleHz;
    float rc = 1.0f / (2.0f * PI * cutoffHz);
    _alpha = dt / (dt + rc);
    _init  = false;
  }

  float update(float in) {
    if (!_init) { _out = in; _init = true; }
    _out += _alpha * (in - _out);
    return _out;
  }

  void reset() { _init = false; _out = 0; }
  float value() const { return _out; }

private:
  float _alpha, _out;
  bool  _init;
};

// =============================================================================
//  MPU6050 Sürücüsü — Gyro Z + İvmemetre X,Y
// =============================================================================
class Mpu6050Driver {
public:
  static constexpr uint8_t ADDR          = GYRO_I2C_ADDRESS;
  static constexpr uint8_t REG_PWR       = 0x6B;
  static constexpr uint8_t REG_GYRO_CFG = 0x1B;
  static constexpr uint8_t REG_ACC_CFG  = 0x1C;
  static constexpr uint8_t REG_ACC_X    = 0x3B;
  static constexpr uint8_t REG_GYRO_Z   = 0x47;
  // FS_SEL=2 → ±1000°/s → 32.8 LSB/(°/s)
  // AFS_SEL=1 → ±4g     → 8192 LSB/g
  static constexpr float GYRO_LSB = 32.8f;
  static constexpr float ACC_LSB  = 8192.0f;

  bool begin() {
    Wire.setClock(400000);
    // Uyandır
    if (!writeReg(REG_PWR, 0x00)) return false;
    delayMicroseconds(500);
    // Gyro: FS_SEL=2 (±1000°/s)
    writeReg(REG_GYRO_CFG, 0x02 << 3);
    // Accel: AFS_SEL=1 (±4g)
    writeReg(REG_ACC_CFG,  0x01 << 3);
    return true;
  }

  // Gyro Z (°/s) + Accel X,Y (g) — tek I2C transferinde 5 register oku
  bool readAll(float &gyroZ, float &accX, float &accY) {
    // ACCEL_X (0x3B,3C), ACCEL_Y (0x3D,3E), ACCEL_Z (0x3F,40), TEMP(0x41,42),
    // GYRO_X(0x43,44), GYRO_Y(0x45,46), GYRO_Z(0x47,48)
    // Tümü okuyup sadece gerekenleri al
    Wire.beginTransmission(ADDR);
    Wire.write(REG_ACC_X);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom(ADDR, (uint8_t)14);
    if (Wire.available() < 14) return false;
    int16_t ax = readWord();
    int16_t ay = readWord();
    readWord(); // az — kullanılmıyor
    readWord(); // temp — kullanılmıyor
    readWord(); // gx — kullanılmıyor
    readWord(); // gy — kullanılmıyor
    int16_t gz = readWord();
    accX  = (float)ax / ACC_LSB;
    accY  = (float)ay / ACC_LSB;
    gyroZ = (float)gz / GYRO_LSB;
    return true;
  }

private:
  bool writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(ADDR);
    Wire.write(reg); Wire.write(val);
    return Wire.endTransmission() == 0;
  }
  int16_t readWord() {
    int16_t val = (int16_t)(Wire.read() << 8);
    val |= Wire.read();
    return val;
  }
};

// =============================================================================
//  DriftGyroProcessor — ArduDrift algoritması
// =============================================================================
class DriftGyroProcessor {
public:

  // ── Ayarlanabilir parametreler (Serial ile runtime değiştirilebilir) ───────
  struct Params {
    float loopHz         = GYRO_LOOP_HZ;      // işlem döngü frekansı
    float imuFilterHz    = GYRO_IMU_FILTER_HZ; // gyro/accel LPF kesim frekansı
    float servoFilterHz  = GYRO_SERVO_FILTER_HZ;// servo çıkış LPF
    float angAccFilterHz = GYRO_ANGACC_FILTER_HZ;// açısal ivme LPF

    float gain           = GYRO_K_GAIN;        // ana kazanç (0.001–0.01 arası)
    float defaultGain    = GYRO_DEFAULT_GAIN;  // kumandadan gain gelmediyse

    float gyroRate       = GYRO_ANGVEL_RATE;   // açısal hız katkı oranı
    float angAccRate     = GYRO_ANGACC_RATE;   // açısal ivme katkı oranı
    float accRate        = GYRO_ACC_RATE;      // yatay ivme düzeltme oranı
    float angleRate      = GYRO_ANGLE_RATE;    // açı integrali katkı oranı
    float angleLimit     = GYRO_ANGLE_LIMIT;   // açı integrali üst sınırı (°)
    float angHalfLife    = GYRO_ANG_HALF_LIFE; // açı integral yarı ömrü (s)
    float outputRange    = GYRO_OUTPUT_RANGE;  // çıkış doyma sınırı (0–1 arası normalize)

    float gyroCurve      = GYRO_EXP_CURVE;     // gyro S-eğrisi üsteli (-0.5…+0.5)
    float angvelZero     = GYRO_ANGVEL_ZERO;   // açısal hız sıfır ofseti (°/s)

    // Throttle etkisi
    float throttleEffect = GYRO_THROTTLE_RATE; // düzeltme → throttle azaltma oranı
    bool  throttleEnable = false;              // Serial komutla toggle
  };

  DriftGyroProcessor() : _enabled(false), _gain(GYRO_GAIN_DEFAULT),
                          _direction(GYRO_DIRECTION_DEFAULT), _lastMs(0),
                          _angleIntegral(0), _angAccIntegral(0),
                          _lastGyroActive(0), _firstRun(true),
                          _reportEnabled(false) {}

  // setup() içinde çağır
  bool begin() {
    Wire.begin(GYRO_SDA_PIN, GYRO_SCL_PIN);
    bool ok = _mpu.begin();
    if (!ok) {
      Serial.println("[GYRO] MPU6050 bulunamadi!");
      _enabled = false;
      return false;
    }
    _setupFilters();
    _enabled = true;
    Serial.printf("[GYRO] Hazir. Gain:%d Dir:%+d Loop:%.0fHz\n",
                  _gain, _direction, _p.loopHz);
    printHelp();
    return true;
  }

  // loop() içinde çağır — Serial komut işle
  void handleSerial() {
    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\r') continue;
      if (c == '\n') {
        _serialBuf[_serialIdx] = '\0';
        _parseCommand(_serialBuf);
        _serialIdx = 0;
      } else if (_serialIdx < sizeof(_serialBuf) - 1) {
        _serialBuf[_serialIdx++] = c;
      }
    }
  }

  // Ana işleme — userSteer: -100..+100, userThrottle: -100..+100
  // finalThrottle: gyro etkisi uygulanmış throttle
  // return: gyro uygulanmış finalSteer (-100..+100)
  int process(int userSteer, int userThrottle, int &finalThrottle) {
    finalThrottle = userThrottle;
    if (!_enabled || _gain == 0) {
      _lastCorrection = 0;
      return userSteer;
    }

    uint32_t now = millis();
    float dt     = (float)(now - _lastMs) / 1000.0f;
    if (dt < (1.0f / _p.loopHz) * 0.8f) {
      // Henüz zaman dolmadı — son sonucu döndür
      if (_p.throttleEnable) finalThrottle = _lastFinalThrottle;
      return _lastFinalSteer;
    }
    if (dt > 0.1f) dt = 1.0f / _p.loopHz; // timeout sonrası sıfırla
    _lastMs = now;

    // ── 1. Sensör oku ────────────────────────────────────────────────────────
    float rawGz, rawAx, rawAy;
    if (!_mpu.readAll(rawGz, rawAx, rawAy)) {
      return _lastFinalSteer; // okuma hatası → son değeri koru
    }

    // ── 2. Filtrele ─────────────────────────────────────────────────────────
    float gz = _gyroFilter.update(rawGz);
    float ax = _accelXFilter.update(rawAx);
    // ay şimdilik kullanılmıyor ama okunuyor

    // ── 3. Yön uygula + sıfır ofset ─────────────────────────────────────────
    float angVel = (gz + _p.angvelZero) * _direction;

    // ── 4. ArduDrift counter-steer hesabı ───────────────────────────────────
    float correction = _calculateCounterSteer(angVel, ax, dt);

    // ── 5. Gain ölçekle (kumanda gain 0-100 → _p.gain ile çarp) ─────────────
    float gainFactor = (_gain / 100.0f) * _p.defaultGain;
    correction *= gainFactor * _p.gain;

    // ── 6. S-curve ──────────────────────────────────────────────────────────
    if (fabsf(_p.gyroCurve) > 0.001f) {
      float norm = correction / _p.outputRange;
      norm       = _applySCurve(norm, _p.gyroCurve);
      correction = norm * _p.outputRange;
    }

    // ── 7. Doyum sınırı ─────────────────────────────────────────────────────
    correction = constrain(correction, -_p.outputRange, _p.outputRange);

    // ── 8. Servo çıkış filtresi ─────────────────────────────────────────────
    float corrFilt = _servoFilter.update(correction);

    // ── 9. -100..+100 aralığına ölçekle ─────────────────────────────────────
    int steerCorr  = (int)(corrFilt * 100.0f);
    int finalSteer = constrain(userSteer + steerCorr, -100, 100);

    // ── 10. Throttle etkisi (opsiyonel) ─────────────────────────────────────
    if (_p.throttleEnable && userThrottle > 0) {
      float absCorr     = fabsf(corrFilt);
      int   throttleCut = (int)(absCorr * _p.throttleEffect * 100.0f);
      finalThrottle     = constrain(userThrottle - throttleCut, 0, 100);
    }

    _lastCorrection    = steerCorr;
    _lastFinalSteer    = finalSteer;
    _lastFinalThrottle = finalThrottle;
    _lastRawRate       = rawGz;
    _lastAngVel        = angVel;
    _lastCorrRaw       = corrFilt;

    // ── Report ──────────────────────────────────────────────────────────────
    if (_reportEnabled) {
      static uint32_t rptMs = 0;
      if (now - rptMs >= 100) {
        rptMs = now;
        Serial.printf("[GYRO] gz:%.1f angVel:%.1f corr:%.3f steer:%d thr:%d\n",
                      rawGz, angVel, corrFilt, finalSteer, finalThrottle);
      }
    }

    return finalSteer;
  }

  // Setter / Getter
  void setGain(int g)      { _gain      = constrain(g, 0, 100); }
  void setDirection(int d) { _direction = (d >= 0) ? 1.0f : -1.0f; }
  int   getGain()          const { return _gain; }
  int   getDirection()     const { return (int)_direction; }
  float getRawRate()       const { return _lastRawRate; }
  int   getCorrection()    const { return _lastCorrection; }
  bool  isEnabled()        const { return _enabled; }
  bool  throttleEnabled()  const { return _p.throttleEnable; }

  void resetPid() {
    _angleIntegral = 0;
    _angAccIntegral = 0;
    _firstRun = true;
    _servoFilter.reset();
    _gyroFilter.reset();
    _accelXFilter.reset();
  }

  void printHelp() {
    Serial.println("[GYRO] Komutlar:");
    Serial.println("  gyro report on/off");
    Serial.println("  gyro throttle on/off  (hiza etki)");
    Serial.println("  gyro get PARAM");
    Serial.println("  gyro set PARAM DEGER");
    Serial.println("  Parametreler: GAIN K_GAIN GYRO_RATE ACC_RATE ANG_RATE");
    Serial.println("                ANG_LIMIT HALF_LIFE OUTPUT_RANGE CURVE ZERO");
    Serial.println("                THROTTLE_RATE");
  }

private:
  Mpu6050Driver _mpu;
  PT1Filter _gyroFilter, _accelXFilter, _servoFilter;

  Params   _p;
  int      _gain;
  float    _direction;
  bool     _enabled;
  uint32_t _lastMs;

  // ArduDrift durum değişkenleri
  float    _angleIntegral;
  float    _angAccIntegral;
  uint32_t _lastGyroActive;
  bool     _firstRun;

  // Son değerler
  int      _lastCorrection    = 0;
  int      _lastFinalSteer    = 0;
  int      _lastFinalThrottle = 0;
  float    _lastRawRate       = 0;
  float    _lastAngVel        = 0;
  float    _lastCorrRaw       = 0;

  // Serial
  char     _serialBuf[64];
  uint8_t  _serialIdx = 0;
  bool     _reportEnabled;

  void _setupFilters() {
    _gyroFilter.setup(_p.imuFilterHz,    _p.loopHz);
    _accelXFilter.setup(_p.imuFilterHz,  _p.loopHz);
    _servoFilter.setup(_p.servoFilterHz, _p.loopHz);
  }

  // ArduDrift calculateCounterSteerByKinematics() uyarlaması
  float _calculateCounterSteer(float angVel, float accelX, float dt) {
    const float DEADBAND_GYRO  = 0.8f;  // °/s
    const float DEADBAND_ACCEL = 0.2f;  // g
    const float DEADBAND_TIMEOUT = 0.25f; // s

    const float reduction = powf(2.0f, -dt / _p.angHalfLife);

    if (_firstRun) {
      _lastGyroActive = millis();
      _firstRun = false;
    }

    float counterSteer = 0.0f;
    uint32_t now = millis();

    if (fabsf(angVel) > DEADBAND_GYRO) {
      _lastGyroActive = now;

      // Açısal hız bileşeni
      counterSteer += angVel * _p.gyroRate;

      // Açısal ivme bileşeni (türev filtresi ile)
      float angAccComp = _p.angAccFilterHz * (angVel - _angAccIntegral);
      _angAccIntegral += angAccComp;
      counterSteer += angAccComp * _p.angAccRate / _p.loopHz;

      // Açı integrali (drift hafızası)
      float angleIncrement = angVel;
      if (fabsf(accelX) > DEADBAND_ACCEL) {
        // Yatay ivme: kayma yönü ile açısal hareketi ayırt et
        angleIncrement -= accelX * _p.accRate * 10.0f;
      }

      float nextIntegral = _angleIntegral + angleIncrement;
      float limit = _p.angleLimit * _p.loopHz;
      if (fabsf(nextIntegral) <= limit) {
        _angleIntegral = nextIntegral * reduction;
      }

    } else {
      // Deadband içi: zamanla integrel boz
      float elapsed = (float)(now - _lastGyroActive) / 1000.0f;
      if (elapsed > DEADBAND_TIMEOUT) {
        _angleIntegral *= reduction;
      }
    }

    // Açı bileşenini ekle
    float realAngle = _angleIntegral / _p.loopHz;
    counterSteer += realAngle * _p.angleRate;

    return counterSteer;
  }

  // ArduDrift S-curve (üstel hassasiyet eğrisi)
  float _applySCurve(float x, float exp_param) {
    if (fabsf(x) < 0.001f) return 0.0f;
    if (fabsf(exp_param) < 0.001f) return x;

    float absX   = fabsf(x);
    float signX  = (x > 0) ? 1.0f : -1.0f;
    float basePow = powf(100.0f, exp_param);

    float num = (powf(basePow, absX) / basePow) - (1.0f / basePow);
    float den = 1.0f - (1.0f / basePow);

    return signX * (num / den);
  }

  // Serial komut parser
  void _parseCommand(char* buf) {
    // Sadece "gyro ..." ile başlayanları işle
    if (strncmp(buf, "gyro ", 5) != 0) return;
    char* cmd = buf + 5;

    if (strncmp(cmd, "report ", 7) == 0) {
      _reportEnabled = (strncmp(cmd + 7, "on", 2) == 0);
      Serial.printf("[GYRO] Report: %s\n", _reportEnabled ? "ON" : "OFF");

    } else if (strncmp(cmd, "throttle ", 9) == 0) {
      _p.throttleEnable = (strncmp(cmd + 9, "on", 2) == 0);
      Serial.printf("[GYRO] Throttle etkisi: %s\n", _p.throttleEnable ? "ON" : "OFF");

    } else if (strncmp(cmd, "help", 4) == 0) {
      printHelp();

    } else if (strncmp(cmd, "get ", 4) == 0) {
      _getParam(cmd + 4);

    } else if (strncmp(cmd, "set ", 4) == 0) {
      char name[32]; float val;
      if (sscanf(cmd + 4, "%31s %f", name, &val) == 2) {
        _setParam(name, val);
      }
    }
  }

  void _getParam(const char* name) {
    #define GPGET(N, F) if (strcmp(name, #N) == 0) { Serial.printf("[GYRO] " #N " = %.4f\n", F); return; }
    GPGET(GAIN,         (float)_gain)
    GPGET(K_GAIN,       _p.gain)
    GPGET(GYRO_RATE,    _p.gyroRate)
    GPGET(ACC_RATE,     _p.accRate)
    GPGET(ANG_RATE,     _p.angleRate)
    GPGET(ANG_LIMIT,    _p.angleLimit)
    GPGET(HALF_LIFE,    _p.angHalfLife)
    GPGET(OUTPUT_RANGE, _p.outputRange)
    GPGET(CURVE,        _p.gyroCurve)
    GPGET(ZERO,         _p.angvelZero)
    GPGET(THROTTLE_RATE,_p.throttleEffect)
    Serial.printf("[GYRO] Bilinmeyen parametre: %s\n", name);
    #undef GPGET
  }

  void _setParam(const char* name, float val) {
    #define GPSET(N, F, MIN, MAX) \
      if (strcmp(name, #N) == 0) { \
        if (val >= MIN && val <= MAX) { F = val; Serial.printf("[GYRO] " #N " = %.4f\n", val); } \
        else Serial.printf("[GYRO] Aralik disi! " #N ": %.1f - %.1f\n", (float)MIN, (float)MAX); \
        return; }
    GPSET(K_GAIN,       _p.gain,          0.0001f, 0.1f)
    GPSET(GYRO_RATE,    _p.gyroRate,      0,       20)
    GPSET(ACC_RATE,     _p.accRate,       0,       20)
    GPSET(ANG_RATE,     _p.angleRate,     0,       20)
    GPSET(ANG_LIMIT,    _p.angleLimit,    10,      360)
    GPSET(HALF_LIFE,    _p.angHalfLife,   0.01f,   5)
    GPSET(OUTPUT_RANGE, _p.outputRange,   0.1f,    1)
    GPSET(CURVE,        _p.gyroCurve,    -0.5f,    0.5f)
    GPSET(ZERO,         _p.angvelZero,   -20,      20)
    GPSET(THROTTLE_RATE,_p.throttleEffect,0,       2)
    // GAIN kumandadan yönetilir ama manual de ayarlanabilir
    if (strcmp(name, "GAIN") == 0) {
      _gain = constrain((int)val, 0, 100);
      Serial.printf("[GYRO] GAIN = %d\n", _gain);
      return;
    }
    Serial.printf("[GYRO] Bilinmeyen parametre: %s\n", name);
    #undef GPSET
  }
};

// Global erişim için tek örnek
using GyroProcessor = DriftGyroProcessor;
