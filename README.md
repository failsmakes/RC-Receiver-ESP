# RC Receiver ESP

ESP8266 ve ESP32 kartları için RC drift araba alıcı (receiver) yazılımı.

Motor sürücü, servo, SBUS çıkışı, ArduDrift algoritmalı MPU6050 gyro ve LiPo pil yönetimini tek bir kod tabanında destekler. Kart tipi ve PS kontrolcü seçimi `platformio.ini`'den yapılır; başka bir setup adımı gerekmez.

---

## İçindekiler

- [Özellikler](#özellikler)
- [Kontrol Kaynakları ve Öncelik Sistemi](#kontrol-kaynakları-ve-öncelik-sistemi)
- [Proje Yapısı](#proje-yapısı)
- [Donanım Bağlantıları](#donanım-bağlantıları)
- [Kurulum — VSCode + PlatformIO](#kurulum--vscode--platformio)
- [Derleme ve Yükleme](#derleme-ve-yükleme)
- [Konfigürasyon](#konfigürasyon)
- [PS3 Eşleştirme](#ps3-eşleştirme)
- [PS4 Eşleştirme](#ps4-eşleştirme)
- [Kontrolcü Düğme Eşlemeleri](#kontrolcü-düğme-eşlemeleri-ps3--ps4)
- [Gyro — Drift Stabilizasyon Sistemi](#gyro--drift-stabilizasyon-sistemi)
- [Pil Yönetimi](#pil-yönetimi)
- [Telemetri](#telemetri)

---

## Özellikler

- **Çift platform:** ESP8266 (NodeMCU v3) ve ESP32 (DevKit) aynı kaynak koddan derlenir
- **Çoklu eş zamanlı kaynak:** ESP-NOW + Android UDP her zaman aktif; PS3 veya PS4 ek kaynak olarak eklenebilir
- **Motor sürücü:** FS_MX1508 kütüphanesi — ileri/geri/fren, deadband histerezisi, geri vitese geçiş koruması
- **Servo yön kontrolü:** -100…+100 → 0°…180°
- **SBUS çıkışı:** 16 kanal, 100 kbps, yazılımsal inversion
- **ArduDrift gyro algoritması:** Açısal hız + açısal ivme + yatay ivme + açı integrali birleşik counter-steer; S-curve çıkış; isteğe bağlı throttle etkisi
- **LiPo pil yönetimi:** 2S/3S otomatik tespit, açılışta motor beep sinyali, hücre voltajı bazlı düşük voltaj kilidi
- **Çift telemetri:** UDP JSON (Android) + ESP-NOW ACK (TX kumanda)

---

## Kontrol Kaynakları ve Öncelik Sistemi

ESP-NOW ve Android UDP **her zaman** aktiftir — kart tipinden ve PS seçiminden bağımsız olarak her build'de derlenir. PS3 veya PS4 ise `RX_INPUT_SOURCE` seçimine göre **ek kaynak** olarak eklenir.

```
┌──────────────────────────────────────────────────────┐
│  Öncelik (yüksekten düşüğe)                          │
│                                                      │
│  1. PS3 / PS4  — bağlıyken diğer kaynaklar yoksayılır│
│  2. ESP-NOW    ─┐                                    │
│  3. Android UDP─┘ PS bağlı değilken ikisi aktif      │
└──────────────────────────────────────────────────────┘
```

PS bağlantısı kesildiğinde ESP-NOW ve Android otomatik devreye girer; yeniden başlatma veya ayar gerekmez. Telemetri ise öncelikten bağımsız olarak her zaman hem Android'e hem ESP-NOW TX'e gönderilir.

| Ortam | Board | Aktif Kaynaklar |
|---|---|---|
| `nodemcuv2` | ESP8266 | ESP-NOW + Android UDP |
| `esp32_nops` | ESP32 | ESP-NOW + Android UDP |
| `esp32_ps3` | ESP32 | ESP-NOW + Android UDP + PS3 |
| `esp32_ps4` | ESP32 | ESP-NOW + Android UDP + PS4 |

---

## Proje Yapısı

```
RC-Receiver-ESP/
├── platformio.ini       # Build ortamları ve kütüphane bağımlılıkları
└── src/
    ├── config.h         # Kart, kaynak, pin ve tüm parametre tanımları
    ├── platform.h       # ESP8266/ESP32 WiFi + ESP-NOW soyutlama katmanı
    ├── GyroProcessor.h  # ArduDrift algoritması (MPU6050 + filtreler + counter-steer)
    ├── SbusOutput.h     # SBUS 16 kanal encoder
    └── receiver.cpp     # Ana uygulama
```

---

## Donanım Bağlantıları

### ESP8266 (NodeMCU v3)

| Sinyal | NodeMCU Pin | GPIO |
|---|---|---|
| Motor IN1 (PWM) | D1 | GPIO5 |
| Motor IN2 (PWM) | D2 | GPIO4 |
| Servo | D4 | GPIO2 |
| SBUS TX | D3 | GPIO0 |
| Gyro SDA | D6 | GPIO12 |
| Gyro SCL | D7 | GPIO13 |
| Batarya ADC | A0 | ADC0 |

> D1/D2 motor sürücüsüne ayrıldığından gyro I2C için D6/D7 kullanılır.

### ESP32 (DevKit v1)

| Sinyal | GPIO |
|---|---|
| Motor IN1 (LEDC PWM) | GPIO25 |
| Motor IN2 (LEDC PWM) | GPIO26 |
| Servo | GPIO27 |
| SBUS TX | GPIO17 |
| Gyro SDA | GPIO18 |
| Gyro SCL | GPIO19 |
| Batarya ADC | GPIO34 (ADC1, giriş-only) |

### MPU6050 Bağlantısı

```
MPU6050        ESP8266 (NodeMCU)    ESP32
VCC  ───────── 3.3V                 3.3V
GND  ───────── GND                  GND
SDA  ───────── D6 (GPIO12)          GPIO18
SCL  ───────── D7 (GPIO13)          GPIO19
AD0  ───────── GND  (adres 0x68)    GND
INT  ───────── bağlanmaz            bağlanmaz
```

### Batarya Voltaj Bölücü

ESP8266 A0 ve ESP32 GPIO34 maksimum 3.3V kabul eder. LiPo voltajını gerilim bölücüyle düşürün:

```
Vbat ──── R1 (100 kΩ) ──┬── ADC pini
                         │
                      R2 (20 kΩ)
                         │
                        GND
```

`config.h` varsayılanları: `VBAT_R1 = 100.0`, `VBAT_R2 = 20.0`

Ölçüm üst sınırı: 3.3 × (100+20)/20 = **19.8V** — 3S LiPo için güvenlidir.

---

## Kurulum — VSCode + PlatformIO

### Gereksinimler

- [Visual Studio Code](https://code.visualstudio.com/)
- VSCode Extensions panelinden **PlatformIO IDE** eklentisi

### Adımlar

1. VSCode'u açın, **File → Open Folder** ile `RC-Receiver-ESP` klasörünü seçin
2. PlatformIO `platformio.ini`'yi otomatik tanır; ilk açılışta araçları indirir (birkaç dakika)
3. `platformio.ini` içindeki `port = com3` satırını kendi COM portunuzla değiştirin
4. Kütüphaneler `lib_deps`'ten otomatik indirilir

---

## Derleme ve Yükleme

### PlatformIO Arayüzü (önerilen)

1. Sol kenar çubuğunda **PlatformIO** simgesine (karınca) tıklayın
2. **Project Tasks** altından hedef ortamı seçin
3. **Upload** butonuna tıklayın

### Terminal (PlatformIO CLI)

```bash
# Derle
pio run -e nodemcuv2
pio run -e esp32_nops
pio run -e esp32_ps3
pio run -e esp32_ps4

# Derle ve yükle
pio run -e esp32_ps3 --target upload

# Seri monitör
pio device monitor -e esp32_ps3
```

> Linux/Mac port izni: `sudo usermod -aG dialout $USER`

---

## Konfigürasyon

Çoğu ayar ortam seçimiyle otomatik yapılır. Yalnızca aşağıdakileri `config.h`'te düzenleyin:

### Transmitter MAC Adresi (ESP-NOW)

```c
#define TX_MAC   { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF }
```

Transmitter, açılışta MAC adresini Serial'a yazar.

### WiFi AP Ayarları

```c
#define WIFI_AP_SSID      "ESP_RC_CAR"
#define WIFI_AP_PASSWORD  "12345678"
```

### Diğer Parametreler

| Parametre | Varsayılan | Açıklama |
|---|---|---|
| `CELL_MIN_VOLTAGE` | `3.4f` | Motor kilidi hücre voltajı (V) |
| `THROTTLE_DEADBAND` | `10` | Motor deadband (0–100) |
| `PS_STICK_DEADBAND` | `10` | PS stick serbest bölgesi |
| `PS_GYRO_GAIN_STEP` | `5` | L1/R1 başına gain adımı |

---

## PS3 Eşleştirme

1. `esp32_ps3` ortamını yükleyin
2. Serial Monitor'ü açın (115200 baud), şu satırı not alın:
   ```
   [PS3] Hazir. BT MAC (SixaxisPairTool ile PS3'e yaz): AA:BB:CC:DD:EE:FF
   ```
3. PS3'ü USB ile bilgisayara bağlayın
4. [SixaxisPairTool](https://sixaxispairtool.en.lo4d.com)'u açın, MAC'i girin → **Update**
5. USB'yi çıkarın, **PS** butonuna basın → bağlanır

---

## PS4 Eşleştirme

1. `esp32_ps4` ortamını yükleyin
2. PS4'ü kapalı konuma getirin (PS butonuna 15s basın)
3. **Share + PS** butonlarına aynı anda basın — ışık hızlı yanıp söner
4. Serial'da `[PS4] Baglandi!` mesajını bekleyin; LED yeşile döner

---

## Kontrolcü Düğme Eşlemeleri (PS3 / PS4)

| Düğme / Eksen | İşlev |
|---|---|
| Sol stick Y | Gaz (ileri/geri) |
| Sağ stick X | Yön (sol/sağ) |
| L2 analog | Trim sola |
| R2 analog | Trim sağa |
| L1 | Gyro gain −5 (kenar tetikli) |
| R1 | Gyro gain +5 (kenar tetikli) |
| L3 (sol stick bas) | Gyro direction toggle |
| Cross (×) 1s basılı | Trim sıfırla |

**PS4 LED geri bildirimi:**

| Durum | Renk |
|---|---|
| Bağlandı | Yeşil |
| Gyro direction: normal | Mavi |
| Gyro direction: ters | Kırmızı |
| Trim sıfırlandı | Yeşil |

---

## Gyro — Drift Stabilizasyon Sistemi

### Genel Bakış

Gyro sistemi, aracın yaw (yatay dönüş) hareketini MPU6050 sensörüyle ölçerek sürücünün direksiyon komutuna otomatik counter-steer (karşı direksiyon) düzeltmesi ekler. Temel amacı drift sırasında aracın 360° dönüp kontrolden çıkmasını önlemek ve sürücünün daha az müdahaleyle daha uzun ve kararlı bir kayma çizgisi tutmasına yardımcı olmaktır.

Algoritma [ArduDrift](https://github.com/JetMagpie/ArduDrift) projesinden ESP8266/ESP32 için adapte edilmiştir. Orijinal PID tabanlı sistemin yerini alan bu yaklaşım; açısal hız, açısal ivme, yatay ivme ve açı integrali bileşenlerini birleştirerek daha doğal ve öngörülü bir counter-steer üretir.

---

### Algoritma — 3 Bileşenli Counter-Steer

Toplam düzeltme üç bileşenin toplamından oluşur:

```
counterSteer = (angVel  × ANGVEL_RATE)
             + (angAcc  × ANGACC_RATE)
             + (angle   × ANGLE_RATE)
```

Bu ham değer kazanç, S-curve ve çıkış filtresiyle şekillendirilip servo düzeltmesi olarak uygulanır.

#### Bileşen 1 — Açısal Hız (`GYRO_ANGVEL_RATE`)

MPU6050'nin Z ekseninden okunan yaw hızı (°/s). Araç dönerken anlık karşı direksiyon üretir. En hızlı tepki veren bileşendir; agresif sürüşte belirleyicidir.

Artırılırsa: anlık tepki güçlenir, aşırıya kaçarsa salınım başlar.
Azaltılırsa: daha yumuşak, gecikmeli tepki.

#### Bileşen 2 — Açısal İvme (`GYRO_ANGACC_RATE`)

Açısal hızın türevi — dönüşün ne kadar hızlı başladığını/bittiğini ölçer. Gyro henüz büyük bir değer üretmeden düzeltmeye başlar; öngörülü tepki sağlar.

Artırılırsa: öngörü güçlenir, dönüş başlamadan müdahale.
Azaltılırsa: daha az öngörü, daha az titreşim riski.

#### Bileşen 3 — Açı İntegrali (`GYRO_ANGLE_RATE` / `GYRO_ANG_HALF_LIFE`)

Gyro değeri zaman içinde integre edilir ve yarı ömürle yavaşça söndürülür. "Drift hafızası" işlevi görür: araç uzun süre aynı yöne dönmeye devam ederse düzeltme birikir; araç düzleşmeye başlarken de hafızada kalan düzeltme geçişi yumuşatır.

`ANGLE_RATE`: bileşenin ağırlığı.
`ANGLE_LIMIT`: integralin maksimum değeri (taşmayı önler).
`HALF_LIFE`: hafızanın ne kadar hızlı silineceği. 0.05s = kısa hafıza, 0.5s = uzun hafıza.

---

### Yatay İvme Düzeltmesi (`GYRO_ACC_RATE`)

MPU6050'nin X ekseni ivmemetresi, salt rotasyonel dönüş ile gerçek lateral kayma arasındaki farkı tespit etmek için açı integraline korektif bir terim ekler. Araç sadece dönerken farklı, kayarken farklı bir ivme imzası üretir; bu sayede gereksiz düzeltme miktarı azalır.

Artırılırsa: kayma tespiti güçlenir, dönüş sırasında daha az yanlış düzeltme.
Azaltılırsa: ivmemetre etkisi azalır, sadece gyro'ya yaklaşır.

---

### S-Curve Çıkış (`GYRO_EXP_CURVE`)

Düzeltme değeri servo çıkışına uygulanmadan önce üstel bir eğriden geçirilir. Küçük açılarda duyarlılık artırılırken büyük açılarda yumuşatma sağlanır (veya tersi).

```
GYRO_EXP_CURVE =  0.00  → doğrusal
GYRO_EXP_CURVE = -0.18  → merkez duyarlı (varsayılan)
GYRO_EXP_CURVE = +0.30  → kenar duyarlı
```

---

### Kazanç Sistemi

Gyro düzeltmesinin toplam büyüklüğü iki kazanç değeriyle kontrol edilir:

**Kumanda Gain (0–100):** Kumandadan veya PS kontrolcüden gelen gain. 0 = gyro tamamen kapalı. Sürüş sırasında anlık değiştirilebilir.

**K-Gain (`GYRO_K_GAIN`):** İnce hassasiyet ölçeği. Genel kazanç çok yüksek veya çok düşükse bu değerle küçük adımlarla ayarlayın.

```
Toplam düzeltme = counterSteer × K_GAIN × DEFAULT_GAIN × (kumandaGain / 100)
```

---

### Throttle Etkisi

Drift sırasında gaz kısmen kesilirse kayma uzar ve kontrol edilmesi kolaylaşır. Bu özellik varsayılan olarak **kapalı**dır; Serial üzerinden anlık açılıp kapanabilir.

```
finalThrottle = userThrottle − (|düzeltme| × THROTTLE_RATE × 100)
```

`THROTTLE_RATE = 0.3` ile tam düzeltme anında throttle yaklaşık %30 kısılır.

---

### Filtreler

| Filtre | Parametre | Varsayılan | Açıklama |
|---|---|---|---|
| Gyro / Accel LPF | `GYRO_IMU_FILTER_HZ` | 30 Hz | Ham sensör gürültüsünü keser |
| Servo çıkış LPF | `GYRO_SERVO_FILTER_HZ` | 120 Hz | Servo titremesini önler |
| Açısal ivme LPF | `GYRO_ANGACC_FILTER_HZ` | 30 Hz | İvme bileşeni gürültüsünü azaltır |

IMU filtresi düşürülürse daha az gecikme ama daha fazla titreşim; artırılırsa daha az titreşim ama daha yavaş tepki.

---

### config.h Parametre Referansı

```c
// Döngü hızı
#define GYRO_LOOP_HZ            100.0f  // Gyro işlem frekansı (Hz)

// Filtreler
#define GYRO_IMU_FILTER_HZ      30.0f   // Gyro/Accel LPF (Hz)
#define GYRO_SERVO_FILTER_HZ    120.0f  // Servo çıkış LPF (Hz)
#define GYRO_ANGACC_FILTER_HZ   30.0f   // Açısal ivme LPF (Hz)

// Kazanç
#define GYRO_K_GAIN             0.003f  // İnce hassasiyet ölçeği (0.001–0.01)
#define GYRO_DEFAULT_GAIN       200.0f  // Kumanda gain çarpanı

// Bileşen oranları
#define GYRO_ANGVEL_RATE        1.1f    // Açısal hız katkısı
#define GYRO_ANGACC_RATE        1.0f    // Açısal ivme katkısı (öngörü)
#define GYRO_ACC_RATE           0.5f    // Yatay ivme düzeltmesi (kayma yönü)
#define GYRO_ANGLE_RATE         1.0f    // Açı integrali katkısı (drift hafızası)

// Açı integrali
#define GYRO_ANGLE_LIMIT        90.0f   // Maksimum açı birikimi (°)
#define GYRO_ANG_HALF_LIFE      0.15f   // Sönme yarı ömrü (s)

// Çıkış şekillendirme
#define GYRO_OUTPUT_RANGE       0.95f   // Normalize doyum sınırı (0–1)
#define GYRO_EXP_CURVE         -0.18f   // S-curve üsteli (-0.5…+0.5)

// Sıfır ofseti
#define GYRO_ANGVEL_ZERO        0.0f    // Gyro sıfır kalibrasyonu (°/s)

// Throttle etkisi
#define GYRO_THROTTLE_RATE      0.3f    // Düzeltme → throttle azaltma oranı
```

---

### Serial Port Komutları (Çalışma Zamanı Ayarı)

Kart çalışırken **115200 baud** Serial Monitor'e aşağıdaki komutlar gönderilebilir. Parametreler kalıcı değildir; yeniden başlatmada `config.h` değerlerine döner.

#### Veri Akışı

```
gyro report on       →  100ms aralıkla gyro verisi basmaya başlar
gyro report off      →  veri akışını durdurur
```

Örnek çıktı:
```
[GYRO] gz:18.3 angVel:18.3 corr:0.042 steer:23 thr:95
```

| Alan | Açıklama |
|---|---|
| `gz` | Ham gyro Z değeri (°/s) |
| `angVel` | Filtrelenmiş + yön uygulanmış açısal hız (°/s) |
| `corr` | Servo çıkışına eklenen normalize düzeltme (−1…+1) |
| `steer` | Nihai direksiyon değeri (kullanıcı + düzeltme) |
| `thr` | Nihai gaz değeri (throttle etkisi varsa kısılmış) |

#### Throttle Etkisi

```
gyro throttle on     →  drift sırasında gaz kısma aktif
gyro throttle off    →  gaz kısma devre dışı (varsayılan)
```

#### Parametre Okuma ve Yazma

```
gyro get PARAM
gyro set PARAM DEGER
gyro help            →  tüm komutları listeler
```

| Komut | Aralık | Etkisi |
|---|---|---|
| `gyro set K_GAIN 0.004` | 0.0001–0.1 | Genel kazancı artır/azalt |
| `gyro set GYRO_RATE 1.5` | 0–20 | Anlık tepki gücü |
| `gyro set ANGACC_RATE 0.8` | 0–20 | Öngörü tepkisi |
| `gyro set ACC_RATE 0.3` | 0–20 | Kayma yönü düzeltmesi |
| `gyro set ANG_RATE 0.8` | 0–20 | Drift hafızası katkısı |
| `gyro set ANG_LIMIT 60` | 10–360 | Hafıza üst sınırı (°) |
| `gyro set HALF_LIFE 0.1` | 0.01–5 | Hafıza sönme hızı (s) |
| `gyro set OUTPUT_RANGE 0.8` | 0.1–1 | Çıkış doyum sınırı |
| `gyro set CURVE -0.25` | −0.5–+0.5 | S-curve şiddeti |
| `gyro set ZERO 1.2` | −20–+20 | Gyro sıfır ofseti (°/s) |
| `gyro set THROTTLE_RATE 0.4` | 0–2 | Throttle etkisi oranı |

---

### Ayar Önerileri

**Gyro yanıt vermiyor veya çok zayıf:**
- `K_GAIN` artırın: `gyro set K_GAIN 0.005`
- `GYRO_RATE` artırın: `gyro set GYRO_RATE 1.5`

**Gyro çok agresif / araç salınım yapıyor:**
- `K_GAIN` azaltın: `gyro set K_GAIN 0.002`
- `SERVO_FILTER_HZ` düşürün: `GYRO_SERVO_FILTER_HZ 60` (config.h'te)
- `CURVE` daha negatife çekin: `gyro set CURVE -0.30`

**Drift çok erken bitiyor:**
- `HALF_LIFE` artırın: `gyro set HALF_LIFE 0.25` (daha uzun hafıza)
- `ANG_RATE` artırın: `gyro set ANG_RATE 1.3`

**Drift kontrolden çıkıyor / araç tur atıyor:**
- `ANGLE_LIMIT` düşürün: `gyro set ANG_LIMIT 60`
- `HALF_LIFE` düşürün: `gyro set HALF_LIFE 0.08` (hafıza hızlı temizlenir)

**Düşük hızda gyro titreşimi:**
- `IMU_FILTER_HZ` düşürün: `GYRO_IMU_FILTER_HZ 20` (config.h'te)
- Gyro sıfır ofsetini kalibre edin: araç düz dururken `gyro report on` ile drift değerini ölçün, negatifini `ZERO` olarak ayarlayın: `gyro set ZERO 1.5`

**Daha uzun, kontrollü drift:**
```
gyro throttle on
gyro set THROTTLE_RATE 0.25
```

---

## Pil Yönetimi

Açılışta voltaj ölçülerek hücre sayısı otomatik belirlenir:

| Voltaj Aralığı | Sonuç |
|---|---|
| < 4.5V | Pil yok — motor kalıcı kilitli |
| 4.5V – 8.9V | 2S LiPo |
| > 8.9V | 3S LiPo |

Tespitin ardından hücre sayısı kadar motor titreşim darbesi üretilir (2 kez tekrar):
- 2S → bip · bip
- 3S → bip · bip · bip

Çalışma sırasında hücre başına voltaj `CELL_MIN_VOLTAGE` (3.4V) eşiğinin altına düşerse motor kilitlenir. Voltaj `CELL_RECOVER_VOLTAGE` (3.5V) üzerine çıkana kadar kilit açılmaz (histerezis).

---

## Telemetri

### UDP → Android (JSON, 100ms)

```json
{
  "seq":   123,    // paket sayacı
  "t":      75,    // throttle (gyro throttle etkisi uygulanmış, -100..+100)
  "s":      10,    // nihai direksiyon (kullanıcı + gyro düzeltmesi)
  "v":    7.82,    // toplam pil voltajı (V)
  "cells":   2,    // hücre sayısı (0 / 2 / 3)
  "cv":   3.91,    // hücre başına voltaj (V)
  "lv":      0,    // düşük voltaj kilidi (0=normal, 1=kilitli)
  "gg":     50,    // aktif gyro gain (0-100)
  "gd":      1,    // gyro direction (+1 / -1)
  "gc":     -3,    // gyro düzeltme miktarı (servo birimleri)
  "gr":   12.4     // ham gyro açısal hızı (°/s)
}
```

### ESP-NOW ACK → TX Kumanda (binary)

Her ESP-NOW paketi alındığında TX'e `TelemetryPacket` struct'ı gönderilir:

| Alan | Tip | Açıklama |
|---|---|---|
| `ack_seq` | uint8 | Paket sayacı |
| `ack_throttle` | int8 | Uygulanan throttle |
| `ack_steer` | int8 | Uygulanan direksiyon |
| `ack_vBat` | float | Pil voltajı (V) |
| `ack_gyroGain` | int8 | Aktif gyro gain |
| `ack_gyroDir` | int8 | Gyro direction |
| `ack_cells` | uint8 | Hücre sayısı |
| `ack_lowVoltage` | uint8 | Düşük voltaj kilidi (0/1) |
