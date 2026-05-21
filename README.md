# RC Receiver ESP

ESP8266 ve ESP32 kartları için RC araba alıcı (receiver) yazılımı.

Motor sürücü, servo, SBUS çıkışı, MPU6050 gyro ve LiPo pil yönetimini tek bir kod tabanında destekler. Kontrol kaynağı (ESP-NOW, Android UDP, PS3, PS4) ve kart tipi `config.h` üzerinden veya `platformio.ini` build flag'leriyle seçilir.

---

## İçindekiler

- [Özellikler](#özellikler)
- [Desteklenen Kartlar ve Kontrol Kaynakları](#desteklenen-kartlar-ve-kontrol-kaynakları)
- [Proje Yapısı](#proje-yapısı)
- [Donanım Bağlantıları](#donanım-bağlantıları)
- [Kurulum — VSCode + PlatformIO](#kurulum--vscode--platformio)
- [Konfigürasyon](#konfigürasyon)
- [Derleme ve Yükleme](#derleme-ve-yükleme)
- [Seri Port Çıkışı](#seri-port-çıkışı)
- [PS3 Eşleştirme](#ps3-eşleştirme)
- [PS4 Eşleştirme](#ps4-eşleştirme)
- [Kontrolcü Düğme Eşlemeleri](#kontrolcü-düğme-eşlemeleri-ps3--ps4)
- [Pil Yönetimi](#pil-yönetimi)
- [Telemetri Paketi](#telemetri-paketi)

---

## Özellikler

- **Çift platform:** ESP8266 (NodeMCU v3) ve ESP32 (DevKit) aynı kaynak koddan derlenir
- **4 kontrol kaynağı:** ESP-NOW, Android UDP, PS3 Bluetooth, PS4 Bluetooth
- **Motor sürücü:** RZ7886 — ileri/geri/fren, deadband, geri vitese geçiş koruması
- **Servo yön kontrolü:** -100…+100 → 0°…180°
- **SBUS çıkışı:** 16 kanal, 100 kbps, yazılımsal inversion
- **MPU6050 Gyro:** PT1 filtreli PID yaw stabilizasyonu (drift gyro); gain ve direction kumandadan ayarlanır
- **LiPo pil yönetimi:** 2S/3S otomatik tespit, açılışta motor beep sinyali, hücre voltajı düşük voltaj kilidi
- **Telemetri:** UDP JSON (Android'e) ve ESP-NOW ACK (transmitter'a)

---

## Desteklenen Kartlar ve Kontrol Kaynakları

| Kontrol Kaynağı | ESP8266 | ESP32 | PlatformIO Ortamı |
|---|:---:|:---:|---|
| ESP-NOW (fiziksel TX) | ✓ | ✓ | `nodemcuv2` / `esp32_espnow` |
| Android UDP | ✓ | ✓ | `nodemcuv2` / `esp32_espnow` |
| PS3 DualShock (Bluetooth) | ✗ | ✓ | `esp32_ps3` |
| PS4 DualShock (Bluetooth) | ✗ | ✓ | `esp32_ps4` |

> ESP-NOW seçildiğinde Android UDP da paralel kaynak olarak aktif kalır.  
> PS3/PS4 seçildiğinde ESP-NOW ve Android UDP devre dışı olur.

---

## Proje Yapısı

```
RC-Receiver-ESP/
├── platformio.ini          # Build ortamları ve kütüphane bağımlılıkları
└── src/
    ├── config.h            # Kart, kaynak, pin ve parametre seçimleri
    ├── platform.h          # ESP8266/ESP32 PWM ve ESP-NOW soyutlama katmanı
    ├── GyroProcessor.h     # PT1 filtre + PID + MPU6050 sürücüsü
    ├── SbusOutput.h        # SBUS 16 kanal encoder
    └── receiver.cpp        # Ana uygulama
```

---

## Donanım Bağlantıları

### ESP8266 (NodeMCU v3)

| Sinyal | Pin | GPIO |
|---|---|---|
| Motor IN1 (PWM) | D1 | GPIO5 |
| Motor IN2 (PWM) | D2 | GPIO4 |
| Servo | D4 | GPIO2 |
| SBUS TX | D3 | GPIO0 |
| Gyro SDA | D6 | GPIO12 |
| Gyro SCL | D7 | GPIO13 |
| Batarya ADC | A0 | ADC0 |

> Gyro için D6/D7 kullanılır çünkü D1/D2 motor sürücüsüne ayrılmıştır.

### ESP32 (DevKit v1)

| Sinyal | GPIO |
|---|---|
| Motor IN1 (LEDC PWM) | GPIO25 |
| Motor IN2 (LEDC PWM) | GPIO26 |
| Servo | GPIO27 |
| SBUS TX | GPIO17 |
| Gyro SDA | GPIO21 |
| Gyro SCL | GPIO22 |
| Batarya ADC | GPIO34 (ADC1_CH6, giriş-only) |

### Batarya Voltaj Bölücü

ESP8266 A0 ve ESP32 GPIO34 maksimum 3.3V kabul eder. LiPo voltajını aşağıdaki gerilim bölücüyle düşürün:

```
Vbat ──┬── R1 (47 kΩ) ──┬── ADC
       │                 │
      GND           R2 (10 kΩ)
                         │
                        GND
```

Bu değerler 3S LiPo (12.6V maks) için de güvenlidir: Vout_max = 12.6 × 10/57 ≈ 2.2V.

---

## Kurulum — VSCode + PlatformIO

### 1. Gereksinimler

- [Visual Studio Code](https://code.visualstudio.com/) kurulu olmalı
- VSCode **Extensions** panelinden **PlatformIO IDE** eklentisi kurulmalı

### 2. Projeyi Aç

1. VSCode'u başlatın
2. **File → Open Folder** ile `RC-Receiver-ESP` klasörünü açın
3. PlatformIO eklentisi `platformio.ini` dosyasını otomatik olarak tanır ve gerekli araçları indirir (ilk açılışta birkaç dakika sürebilir)

### 3. Kütüphaneleri İndir

Kütüphaneler `platformio.ini`'deki `lib_deps` tanımlarından otomatik indirilir. Ek bir işlem gerekmez.

PS3 ve PS4 kütüphaneleri GitHub'dan çekilir:
- `esp32_ps3` ortamı: [github.com/jvpernis/esp32-ps3](https://github.com/jvpernis/esp32-ps3)
- `esp32_ps4` ortamı: [github.com/pablomarquez76/PS4_Controller_Host](https://github.com/pablomarquez76/PS4_Controller_Host)

---

## Konfigürasyon

Yükleme yapmadan önce `src/config.h` dosyasını düzenleyin.

### Kart ve Kaynak Seçimi

```c
// config.h içinde —  değerleri aşağıdaki tabloya göre ayarlayın
#define BOARD_TYPE      BOARD_ESP8266   // veya BOARD_ESP32
#define RX_INPUT_SOURCE INPUT_ANDROID   // veya INPUT_ESPNOW / INPUT_PS3 / INPUT_PS4
```

> **Not:** `BOARD_TYPE` ve `RX_INPUT_SOURCE` değerleri `platformio.ini` build flag'leriyle de override edilebilir. PlatformIO kullanırken doğrudan doğru ortamı (`esp32_ps3`, `esp32_ps4` vb.) seçmek yeterlidir — `config.h`'e dokunmanıza gerek yoktur.

### ESP-NOW için Transmitter MAC Adresi

```c
#define TX_MAC   { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF }
```

Transmitter kartının MAC adresini Serial Monitor'dan öğrenebilirsiniz (transmitter firmware açılışta basar).

### PS3 / PS4 için Bluetooth MAC Adresi

```c
#define PS_BT_MAC   ""   // boş bırakın = varsayılan MAC
// veya belirli bir MAC ile zorlamak için:
#define PS_BT_MAC   "01:02:03:04:05:06"
```

PS3 kütüphanesi (`esp32-ps3`) belirli bir MAC adresiyle eşleştirme yapıldığında daha stabil çalışır. ESP32'nin Bluetooth MAC adresini öğrenmek için `esp32-ps3` kütüphanesiyle gelen `Ps3Address` örneğini bir kez çalıştırın, çıkan MAC adresini buraya yazın.

### Diğer Önemli Parametreler

| Parametre | Varsayılan | Açıklama |
|---|---|---|
| `WIFI_AP_SSID` | `"ESP_RC_CAR"` | Android'in bağlandığı AP adı |
| `WIFI_AP_PASSWORD` | `"12345678"` | AP şifresi |
| `CELL_MIN_VOLTAGE` | `3.4f` | Motor kilidi hücre eşiği (V) |
| `GYRO_GAIN_DEFAULT` | `50` | Varsayılan gyro gain (0-100) |
| `PS_STICK_DEADBAND` | `10` | Stick boş bölgesi |
| `PS_GYRO_GAIN_STEP` | `5` | L1/R1 başına gain adımı |

---

## Derleme ve Yükleme

### VSCode PlatformIO Arayüzü (önerilen)

1. Sol taraftaki **PlatformIO** simgesine tıklayın (karınca simgesi)
2. **Project Tasks** altında kullanmak istediğiniz ortamı açın:
   - `nodemcuv2` → ESP8266 NodeMCU v3
   - `esp32_espnow` → ESP32, ESP-NOW/Android
   - `esp32_ps3` → ESP32, PS3 Bluetooth
   - `esp32_ps4` → ESP32, PS4 Bluetooth
3. Kartı USB'ye takın
4. `platformio.ini` içindeki `upload_port` değerini kendi COM portunuzla değiştirin (Windows: `COMx`, Linux/Mac: `/dev/ttyUSBx` veya `/dev/cu.usbserial-x`)
5. **Upload** butonuna tıklayın — önce derleme, ardından yükleme otomatik yapılır

### Terminal (PlatformIO CLI)

VSCode alt kısmındaki terminal sekmesinden veya harici terminalden:

```powershell
# Sadece derle (yükleme yok)
pio run -e nodemcuv2
pio run -e esp32_espnow
pio run -e esp32_ps3
pio run -e esp32_ps4

# Derle ve yükle
pio run -e nodemcuv2 --target upload
pio run -e esp32_espnow --target upload
pio run -e esp32_ps3 --target upload
pio run -e esp32_ps4 --target upload

# Seri monitor aç
pio device monitor -e nodemcuv2
pio device monitor -e esp32_ps4
```

> Linux/Mac'te port izni hatası alırsanız:  
> `sudo usermod -aG dialout $USER` komutu çalıştırıp oturumu yeniden açın.

---

## Seri Port Çıkışı

Kart açılışta şu bilgileri 115200 baud ile yazar:

```
=== RC RECEIVER BASLIYOR ===
[BOARD] ESP32
[INPUT] PS4 (PS4_Controller_Host)
[VBAT] 7.82V  2S  3.91V/h  Dusuk:HAYIR
[BEEP] 2S → 2x2 darbe
[SBUS] TX:17
[GYRO] Hazir. Gain:50 Dir:+1
[WiFi] — (PS modunda kapalı)
[PS4] Hazir. Share + PS butonlariyla baglayin.
[VBAT] 2S  7.82V  3.91V/h  HAZIR
=== HAZIR ===
```

---

## PS3 Eşleştirme

1. `esp32_ps3` ortamını derleyip ESP32'ye yükleyin
2. Seri monitor'ü açın (115200 baud)
3. **PS3 kontrolcünün USB kablosunu çıkarın** (kablosuz mod için)
4. Kontrolcü üzerindeki **PS** butonuna basın
5. Seri monitörde `[PS3] Baglandi!` mesajını bekleyin
6. İlk bağlantıda eşleştirme sorun çıkarırsa: kontrolcüyü USB ile bilgisayara bağlayıp `SixaxisPairTool` ile ESP32'nin MAC adresini kontrolcüye yazın

---

## PS4 Eşleştirme

1. `esp32_ps4` ortamını derleyip ESP32'ye yükleyin
2. Seri monitor'ü açın (115200 baud)
3. PS4 kontrolcüyü **kapalı** konuma getirin (15 saniye PS tuşuna basın)
4. **Share** ve **PS** tuşlarına **aynı anda** basın — ışık hızlı yanıp sönmeye başlar (eşleştirme modu)
5. Seri monitörde `[PS4] Baglandi!` mesajını bekleyin — kontrolcü LED'i yeşile döner
6. Bağlantı sorunları yaşanırsa `PS4Data.ino` örneğindeki `removePairedDevices()` fonksiyonunu bir kez çalıştırın

---

## Kontrolcü Düğme Eşlemeleri (PS3 / PS4)

| Düğme / Eksen | İşlev |
|---|---|
| Sol stick Y ekseni | Gaz (ileri/geri) |
| Sağ stick X ekseni | Yön (sol/sağ) |
| L2 analog | Trim sola |
| R2 analog | Trim sağa |
| L1 | Gyro gain −5 (kenar tetikli) |
| R1 | Gyro gain +5 (kenar tetikli) |
| L3 (sol stick bas) | Gyro direction toggle |
| Cross (×) — 1s basılı tut | Trim sıfırla |

**PS4'e özel LED geri bildirimi:**

| Durum | LED Rengi |
|---|---|
| Bağlandı | Yeşil |
| Gyro direction: normal | Mavi |
| Gyro direction: ters | Kırmızı |
| Trim sıfırlandı | Yeşil |

---

## Pil Yönetimi

Receiver açılışta batarya voltajını ölçerek hücre sayısını otomatik tespit eder:

| Voltaj aralığı | Sonuç |
|---|---|
| < 4.5V | Pil yok — motor kalıcı kilitli |
| 4.5V – 8.9V | 2S LiPo |
| > 8.9V | 3S LiPo |

Tespitin ardından **hücre sayısı kadar motor titreşim darbesi** üretilir (2S → bip-bip, 3S → bip-bip-bip). Bu sinyal 2 kez tekrar edilir.

Çalışma sırasında hücre başına voltaj `CELL_MIN_VOLTAGE` (varsayılan 3.4V) eşiğinin altına düşerse motor kilitlenir. Voltaj `CELL_RECOVER_VOLTAGE` (3.5V) üzerine çıkana kadar kilit açılmaz (histerezis).

---

## Telemetri Paketi

### UDP → Android (JSON, 100ms periyot)

```json
{
  "seq":  123,     // paket sayacı
  "t":    75,      // throttle  -100..+100
  "s":    10,      // nihai steer (gyro dahil) -100..+100
  "v":    7.82,    // toplam pil voltajı (V)
  "cells": 2,      // hücre sayısı (0/2/3)
  "cv":   3.91,    // hücre başına voltaj (V)
  "lv":   0,       // düşük voltaj kilidi (0=normal, 1=kilitli)
  "gg":   50,      // aktif gyro gain (0-100)
  "gd":   1,       // gyro direction (+1/-1)
  "gc":   -3,      // gyro düzeltme miktarı (-100..+100)
  "gr":   12.4     // ham gyro açısal hızı (°/s)
}
```

### ESP-NOW ACK → Transmitter (binary, packed struct)

Transmitter'a aynı bilgiler binary `TelemetryPacket` struct'ı olarak gönderilir (9 byte).
