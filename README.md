# Smart Chicken Close House - ESP32 FreeRTOS

Proyek ini adalah sistem kendali dan pemantauan IoT pintar untuk kandang ayam berkonsep *Close House*. Sistem ini dibangun menggunakan mikrokontroler **ESP32** dan memanfaatkan **FreeRTOS** untuk menjalankan berbagai tugas (multitasking) secara efisien dan real-time, mulai dari pembacaan sensor, logika kontrol aktuator, hingga pengiriman data ke server MQTT.

<img width="1105" height="730" alt="{F8005D3D-1975-4C30-8A96-CD319165AB9A}" src="https://github.com/user-attachments/assets/7e7f6ede-512f-4635-9d7d-e04c084f0646" />


---

## Fitur Utama

* **Multitasking Real-time (FreeRTOS):** Pembacaan sensor, pembaruan layar LCD, pengiriman MQTT, dan kontrol aktuator berjalan secara konkuren tanpa saling memblokir (non-blocking).
* **Sistem Kontrol Iklim (Climate Control):** Mengontrol *blower* dan *exhaust fan* secara otomatis berdasarkan suhu (DHT22) dan kadar gas amonia (MQ-135).
* **Pemberian Pakan Otomatis:** Menggunakan sensor ultrasonik (HC-SR04) untuk mendeteksi sisa pakan dan menyalakan motor pakan secara otomatis saat wadah kosong.
* **Pemantauan Berat:** Terintegrasi dengan sensor Load Cell (HX711) untuk memantau berat pakan atau berat ayam peliharaan.
* **Telemetri IoT (MQTT):** Mengirimkan data pemantauan kandang (suhu, kelembapan, amonia, berat, status relay) ke *broker* MQTT publik secara berkala dalam format JSON.
* **Mode Darurat (Emergency):** Dilengkapi tombol darurat berbasis interupsi (ISR) yang akan mengaktifkan seluruh sistem ventilasi secara paksa saat ditekan.
* **Sistem Keamanan & Monitoring (Watchdog):** Memiliki task khusus untuk memantau penggunaan *Stack* (High Water Mark) dan *Heap* memori, serta dilengkapi Hardware Watchdog Timer (WDT) untuk mencegah sistem *hang*.

---

## Komponen & Konfigurasi Pin

| Komponen / Modul | Pin ESP32 | Keterangan |
| :--- | :--- | :--- |
| **DHT22** | GPIO 23 | Sensor Suhu & Kelembapan |
| **MQ-135** | GPIO 34 (ADC1_CH6) | Sensor Gas Amonia |
| **HC-SR04** | Trig: 18, Echo: 19 | Sensor Ultrasonik (Jarak Pakan) |
| **HX711** | DT: 4, SCK: 5 | Modul ADC Timbangan / Load Cell |
| **DS3231 RTC** | SDA: 21, SCL: 22 | Real Time Clock (I2C) |
| **LCD 20x4** | SDA: 21, SCL: 22 | Layar Display I2C |
| **Relay Blower** | GPIO 25 | Active-HIGH |
| **Relay Exhaust**| GPIO 26 | Active-HIGH |
| **Relay Motor** | GPIO 27 | Active-HIGH (Motor Pakan) |
| **Tombol Darurat**| GPIO 14 | Active-LOW (INPUT_PULLUP) |

---

## Arsitektur FreeRTOS

Sistem ini membagi beban kerja ke dalam 6 *Task* independen yang disebar ke dalam 2 *Core* prosesor ESP32:

1.  **`sensorTask` (Prioritas 3, Core 1):** Membaca DHT22, MQ-135, dan HC-SR04. Mengirim data ke antrean (*Queue*).
2.  **`scaleTask` (Prioritas 2, Core 0):** Membaca berat dari HX711 secara intensif (*bit-banging*).
3.  **`controlTask` (Prioritas 5, Core 1):** Task tertinggi. Menerima data dari *Queue* dan mengeksekusi logika *Relay*. Juga menangani *Deferred Processing* dari tombol darurat (ISR).
4.  **`lcdTask` (Prioritas 1, Core 0):** Menampilkan status sistem, waktu RTC, dan data sensor ke LCD 20x4.
5.  **`mqttTask` (Prioritas 1, Core 0):** Mengelola koneksi WiFi dan mempublikasikan data (*publish*) ke MQTT Broker.
6.  **`watchdogTask` (Prioritas 1, Core 0):** Memantau memori sistem dan memberikan sinyal *alive* untuk mencegah *kernel panic*.

---

## Topik MQTT

Proyek ini menggunakan *broker* HiveMQ secara *default* (`broker.hivemq.com`). Berikut adalah *topic* yang digunakan:

* **Data Sensor:** `kandangayam/sensor` (Format: JSON Payload)
* **Peringatan Darurat:** `kandangayam/emergency` (Payload: "ACTIVE")
* **Monitor Memori (Stack):** `kandangayam/stackmonitor` (Format: JSON Payload)

---

## Cara Menjalankan Proyek

### 1. Kebutuhan *Library* (Pustaka)
Pastikan Anda telah menginstal pustaka berikut di Arduino IDE / PlatformIO:
* `WiFi.h` (Bawaan ESP32)
* `PubSubClient` (oleh Nick O'Leary)
* `LiquidCrystal_I2C` (oleh Frank de Brabander)
* `DHT sensor library` (oleh Adafruit)
* `HX711 Arduino Library` (oleh Bogdan Necula)
* `RTClib` (oleh Adafruit)

### 2. Konfigurasi Jaringan
Sebelum mengunggah kode (*upload*), ubah konfigurasi WiFi pada baris kode berikut agar sesuai dengan *router* atau *hotspot* Anda:
```cpp
const char* ssid     = "NAMA_WIFI_ANDA";
const char* password = "PASSWORD_WIFI_ANDA";
