#pragma once
// =============================================================================
//  RC CAR — RECEIVER CONFIG  (ESP8266 NodeMCU v3)
// =============================================================================

// --- WiFi AP (Android bağlantısı) --------------------------------------------
#define WIFI_AP_SSID      "ESP_RC_CAR"
#define WIFI_AP_PASSWORD  "12345678"          // min 8 karakter
#define WIFI_AP_CHANNEL   1                   // ESP-NOW kanalıyla aynı olmalı!
#define UDP_PORT          4210

// --- ESP-NOW -----------------------------------------------------------------
// Transmitter'ın MAC adresini buraya yazın (transmitter Serial'dan basar)
#define TX_MAC   { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }  // <-- değiştirin!

// --- MOTOR PİNLERİ  (RZ7886 — IN1/IN2 PWM) ----------------------------------
#define MOTOR_IN1_PIN   D1      // GPIO5
#define MOTOR_IN2_PIN   D2      // GPIO4
#define MOTOR_PWM_FREQ  20000   // analogWriteFreq
#define MOTOR_PWM_MAX   255

// --- SERVO PİNİ --------------------------------------------------------------
#define SERVO_PIN        D4     // GPIO2
#define SERVO_CENTER     90
#define SERVO_MAX_LEFT   0
#define SERVO_MAX_RIGHT  180

// --- DEADBAND / TRIM ---------------------------------------------------------
#define THROTTLE_DEADBAND   5
#define STEER_DEADBAND      3
#define TRIM_MIN           -20
#define TRIM_MAX            20

// --- GÜVENLİK TIMEOUT --------------------------------------------------------
// Bu kadar ms içinde paket gelmezse araç durur
#define FAILSAFE_MS   500

// --- PİL VOLTAJ ÖLÇÜMÜ -------------------------------------------------------
//  ESP8266 NodeMCU v3: ADC0 (A0) 0-3.3V arası okur (0-1023).
//  Pil voltajını gerilim bölücü ile A0'a bağlayın.
//  Örnek: 7.4V LiPo → R1=30kΩ, R2=10kΩ → maks 1.85V (güvenli)
//
#define VBAT_R1              30.0f   // kΩ — üst direnç (pil + tarafa)
#define VBAT_R2              10.0f   // kΩ — alt direnç (GND'e)
#define VBAT_ADC_REF         3.3f    // V — ADC referans gerilimi
#define VBAT_ADC_MAX         1023.0f // ADC çözünürlüğü
#define VBAT_SAMPLES         8       // ortalama için örnek sayısı
#define VBAT_INTERVAL_MS     500     // voltaj okuma sıklığı

// --- TELEMETRİ UDP -----------------------------------------------------------
#define TELEMETRY_PORT       4211    // Android'in dinlediği port
#define TELEMETRY_INTERVAL_MS 100   // telemetri gönderim sıklığı (ms)

// --- SBUS ÇIKIŞI -------------------------------------------------------------
#define SBUS_TX_PIN      D3     // GPIO0
#define SBUS_INVERT_SW   true   // true = yazılımsal inversion (harici devre yok)

#define SBUS_CH_THROTTLE  0   // CH1 — Gaz
#define SBUS_CH_STEER     1   // CH2 — Yön (gyro çıkışı dahil)
#define SBUS_CH_TRIM_RAW  2   // CH3 — Ham trim değeri (bilgi amaçlı)
// CH4-CH16 (index 3-15): 992 (merkez) sabit

// =============================================================================
//  GYRO (MPU6050) AYARLARI
// =============================================================================

// --- I2C Pinleri -------------------------------------------------------------
//  ESP8266 NodeMCU v3 varsayılan I2C pinleri:
//    SDA → D2 (GPIO4)  —  NOT: Motor IN2 ile çakışır!
//    SCL → D1 (GPIO5)  —  NOT: Motor IN1 ile çakışır!
//
//  Çakışmayı önlemek için farklı I2C pinleri kullanın:
//    SDA → D6 (GPIO12)
//    SCL → D7 (GPIO13)
//
//  Wire.begin(GYRO_SDA_PIN, GYRO_SCL_PIN) ile başlatılır.
#define GYRO_SDA_PIN    D6    // GPIO12
#define GYRO_SCL_PIN    D7    // GPIO13

// --- MPU6050 I2C Adresi ------------------------------------------------------
//  AD0 pini GND → 0x68  |  AD0 pini 3.3V → 0x69
#define GYRO_I2C_ADDRESS  0x68

// --- Gyro Ekseni Seçimi -------------------------------------------------------
//  RC araca göre Z ekseni genellikle yaw (sürüş yönü dönüşü) için kullanılır.
//  0 = X ekseni (pitch)
//  1 = Y ekseni (roll)
//  2 = Z ekseni (yaw) ← RC drift gyro için standart
#define GYRO_AXIS  2

// --- Varsayılan Gyro Parametreleri -------------------------------------------
//  Bu değerler Android / Transmitter'dan üzerine yazılabilir.
//  Gain: 0–100 arası (0 = gyro kapalı, 100 = maksimum etki)
//  Direction: 1 = normal, -1 = ters
#define GYRO_GAIN_DEFAULT       50    // 0–100
#define GYRO_DIRECTION_DEFAULT   1    // 1 veya -1

// --- Gyro Loop Hızı ----------------------------------------------------------
//  Gyro işleme döngüsünün hedef periyodu (ms)
//  1000 Hz için 1ms, 500 Hz için 2ms önerilir.
//  ESP8266'da WiFi/UDP işlemleri nedeniyle 5ms (200 Hz) gerçekçi bir değerdir.
#define GYRO_LOOP_MS  5

// --- PT1 Alçak Geçiren Filtre (Low Pass Filter) ------------------------------
//  Gyro ham verisindeki yüksek frekanslı gürültüyü azaltır.
//  Kesim frekansı (Hz): düşük → daha yumuşak ama daha yavaş yanıt
//                        yüksek → daha hızlı ama daha gürültülü
#define GYRO_LPF_CUTOFF_HZ   15.0f

// --- PID Katsayıları ---------------------------------------------------------
//  openRCDG algoritmasından adapte edilmiştir.
//  P: Ana düzeltme kuvveti — yükseltmek gyro'yu daha agresif yapar
//  I: Sürekli hata birikimi — genellikle 0 tutulur
//  D: Türev frenleme — negatif değer titreşimi önler
#define GYRO_PID_P    8.0f
#define GYRO_PID_I    0.0f
#define GYRO_PID_D   -0.4f

// --- PID D-terimi Filtresi ---------------------------------------------------
#define GYRO_PID_D_LPF_HZ  10.0f

// --- Gyro Çıkış Ölçekleme ----------------------------------------------------
//  PID çıkışı bu değere bölünerek -100..+100 servo aralığına indirgenir.
//  Araca göre ayarlanmalı — genellikle 500–1500 arası
#define GYRO_OUTPUT_SCALE  800.0f
