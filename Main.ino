#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <HX711.h>
#include <RTClib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_task_wdt.h"

// =======================
// PIN CONFIGURATION (R13)
// =======================
#define HX_DT          4    // HX711 Data
#define HX_SCK         5    // HX711 Clock
#define TRIG_PIN       18   // HC-SR04 Trigger
#define ECHO_PIN       19   // HC-SR04 Echo
#define MQ135_PIN      34   // MQ-135 Analog (ADC1_CH6)
#define DHT_PIN        23   // DHT22 Data
#define DHT_TYPE       DHT22

// Relay (Active-HIGH: HIGH = ON, LOW = OFF)
#define RELAY_BLOWER   25   // Blower / Kipas Pendingin
#define RELAY_EXHAUST  26   // Exhaust Fan / Pembuang Gas
#define RELAY_MOTOR    27   // Motor Pakan Otomatis
#define EMERGENCY_BTN  14   // Tombol Darurat (Active LOW, PULLUP)

// =======================
// THRESHOLD CONFIGURATION
// =======================
// --- Suhu & Gas ---
#define SUHU_IDEAL         23.0f   // °C — blower & exhaust nyala jika suhu > nilai ini
#define SUHU_NORMAL_LOW    20.0f   // °C — blower & exhaust mati jika suhu < nilai ini
#define AMBANG_AMONIA      30      // % (0–100) — exhaust nyala paksa jika gas > nilai ini
#define AMBANG_AMONIA_LOW  20      // % — exhaust mati jika gas < nilai ini (dan suhu normal)

// --- Pakan (Ultrasonik) ---
#define JARAK_PAKAN_KOSONG 30.0f   // cm — jarak > ini → wadah kosong → motor ON
#define JARAK_PAKAN_PENUH  10.0f   // cm — jarak ≤ ini → wadah penuh → motor OFF

// =======================
// WIFI & MQTT
// =======================
const char* ssid        = "Wokwi-GUEST";
const char* password    = "";
const char* mqtt_server = "broker.hivemq.com";
const char* TOPIC_DATA  = "kandangayam/sensor";
const char* TOPIC_EMRG  = "kandangayam/emergency";
const char* TOPIC_STACK = "kandangayam/stackmonitor";

WiFiClient   espClient;
PubSubClient client(espClient);

// =======================
// PERIPHERAL OBJECTS (R5)
// =======================
LiquidCrystal_I2C lcd(0x27, 20, 4); // LCD I2C 20 kolom x 4 baris
DHT               dht(DHT_PIN, DHT_TYPE);
HX711             scale;
RTC_DS3231        rtc;

// =======================
// SENSOR DATA STRUCTURE
// =======================
typedef struct {
  float temperature; // °C
  float humidity;    // %
  int   gas;         // 0–100 (mapped dari ADC)
  float distance;    // cm (jarak pakan)
  float weight;      // kg (berat ayam/pakan)
} SensorData;

// =======================
// RTOS OBJECTS (R4)
// =======================
// Sinkronisasi 1: Queue — transfer data sensor dari sensorTask ke controlTask
QueueHandle_t     sensorQueue;
// Sinkronisasi 2: Mutex — proteksi shared memory currentData (multiple reader/writer)
SemaphoreHandle_t sensorMutex;
// Sinkronisasi 3: Binary Semaphore — sinyal ke watchdogTask bahwa sistem aktif
SemaphoreHandle_t systemAliveSem;

// Shared memory (dilindungi mutex)
SensorData        currentData = {25.0, 50.0, 0, 15.0, 0.0};

// Task handles untuk stack monitoring (R8)
TaskHandle_t hSensorTask   = NULL;
TaskHandle_t hScaleTask    = NULL;
TaskHandle_t hControlTask  = NULL;
TaskHandle_t hLcdTask      = NULL;
TaskHandle_t hMqttTask     = NULL;
TaskHandle_t hWatchdogTask = NULL;

// ISR flag — volatile agar tidak di-optimasi compiler (R3)
volatile bool emergencyMode = false;

// Mutex untuk critical section ISR debounce
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// =======================
// FUNCTION DECLARATIONS
// =======================
void sensorTask   (void *pvParameters);
void scaleTask    (void *pvParameters);
void controlTask  (void *pvParameters);
void lcdTask      (void *pvParameters);
void mqttTask     (void *pvParameters);
void watchdogTask (void *pvParameters);
void IRAM_ATTR emergencyISR();
void setup_wifi();
void reconnectMQTT();

// =======================
// ISR — EMERGENCY BUTTON (R3)
// Deferred processing: ISR hanya set flag, logika ditangani di controlTask
// IRAM_ATTR memastikan fungsi ada di IRAM agar bisa diakses saat flash di-cache
// =======================
void IRAM_ATTR emergencyISR() {
  // Debounce: abaikan jika < 200ms dari interrupt terakhir
  static uint32_t last_isr_time = 0;
  uint32_t now = xTaskGetTickCountFromISR();
  if ((now - last_isr_time) > pdMS_TO_TICKS(200)) {
    emergencyMode = !emergencyMode; // Toggle mode darurat
    last_isr_time = now;
  }
  // Tidak ada context-switch di ISR → deferred ke controlTask (R3)
}

// =======================
// WIFI SETUP
// =======================
void setup_wifi() {
  Serial.print("[WiFi] Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  uint8_t retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 20) {
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial.print(".");
    retry++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WiFi] Connected! IP: " + WiFi.localIP().toString());
  } else {
    Serial.println("\n[WiFi] FAILED — sistem berjalan offline");
  }
}

// =======================
// MQTT RECONNECT
// =======================
void reconnectMQTT() {
  uint8_t attempt = 0;
  while (!client.connected() && attempt < 3) {
    esp_task_wdt_reset();
    // ID unik agar tidak bentrok di public broker
    String id = "ESP32Kandang_" + String(random(0xffff), HEX);
    Serial.print("[MQTT] Connecting id=" + id + " ... ");
    if (client.connect(id.c_str())) {
      Serial.println("OK");
      return;
    }
    Serial.println("FAIL, retry in 2s");
    vTaskDelay(pdMS_TO_TICKS(2000));
    attempt++;
  }
}

// ============================================================
// TASK 1: SENSOR TASK
// Priority: 3 | Core: 1 | Period: 1000ms
// Fungsi: Baca DHT22, MQ-135, HC-SR04 → simpan ke currentData & Queue
// Kontribusi anggota: [Nama Anggota 1]
// ============================================================
void sensorTask(void *pvParameters) {
  esp_task_wdt_add(NULL); // Daftarkan ke watchdog timer
  SensorData data;
  static float lastDistance = 15.0f;

  Serial.println("[sensorTask] Started on Core " + String(xPortGetCoreID()));

  for (;;) {
    esp_task_wdt_reset(); // Reset WDT agar tidak trigger panic

    // --- 1. Baca Suhu & Kelembapan (DHT22) ---
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    data.temperature = isnan(t) ? 25.0f : t;
    data.humidity    = isnan(h) ? 50.0f : h;

    // --- 2. Baca Gas Amonia (MQ-135, ADC) ---
    // Non-blocking: analogRead langsung, tidak ada pulseIn
    int rawGas = analogRead(MQ135_PIN);
    data.gas = constrain(map(rawGas, 0, 4095, 0, 100), 0, 100);

    // --- 3. Baca Jarak Pakan (HC-SR04) ---
    // Non-blocking I/O (R9): timeout 15000µs agar tidak hang jika echo tidak kembali
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 15000); // timeout 15ms
    float rawDist = (duration == 0) ? lastDistance : (duration * 0.034f / 2.0f);

    // Low-pass filter untuk smoothing nilai jarak (hindari noise spike)
    if (rawDist < 100.0f) {
      data.distance = (lastDistance * 0.7f) + (rawDist * 0.3f);
      lastDistance  = data.distance;
    } else {
      data.distance = lastDistance; // Nilai outlier → pertahankan terakhir
    }

    // --- 4. Tulis ke Shared Memory (proteksi Mutex) ---
    if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      currentData.temperature = data.temperature;
      currentData.humidity    = data.humidity;
      currentData.gas         = data.gas;
      currentData.distance    = data.distance;
      data.weight             = currentData.weight; // Ambil weight dari scaleTask
      xSemaphoreGive(sensorMutex);
    }

    // --- 5. Kirim ke Queue (untuk controlTask) ---
    // Non-blocking: jika queue penuh, data baru dibuang (tidak block sensorTask)
    xQueueSend(sensorQueue, &data, pdMS_TO_TICKS(100));

    Serial.printf("[Sensor] T=%.1f°C H=%.1f%% Gas=%d%% Dist=%.1fcm W=%.2fkg\n",
                  data.temperature, data.humidity, data.gas, data.distance, data.weight);

    vTaskDelay(pdMS_TO_TICKS(1000)); // Period 1 detik
  }
}

// ============================================================
// TASK 2: SCALE TASK (Timbangan HX711)
// Priority: 2 | Core: 0 | Period: 200ms
// Fungsi: Baca berat dari HX711 → update currentData.weight
// Kontribusi anggota: [Nama Anggota 2]
// ============================================================
void scaleTask(void *pvParameters) {
  esp_task_wdt_add(NULL);

  Serial.println("[scaleTask] Started on Core " + String(xPortGetCoreID()));

  for (;;) {
    esp_task_wdt_reset();

    if (scale.is_ready()) {
      // Critical section: baca HX711 bit-banging (sensitif terhadap interrupt)
      portENTER_CRITICAL(&mux);
      float w = scale.get_units(1);
      portEXIT_CRITICAL(&mux);

      // Update shared memory weight (proteksi Mutex)
      if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        currentData.weight = w;
        xSemaphoreGive(sensorMutex);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(200)); // Period 200ms — lebih cepat dari sensor lain
  }
}

// ============================================================
// TASK 3: CONTROL TASK (Logika Aktuator)
// Priority: 5 (TERTINGGI) | Core: 1 | Period: event-driven
// Fungsi: Baca Queue → kendalikan Relay berdasarkan threshold
// Kontribusi anggota: [Nama Anggota 3]
// ============================================================
void controlTask(void *pvParameters) {
  esp_task_wdt_add(NULL);
  SensorData data;
  bool prevEmergency = false;

  Serial.println("[controlTask] Started on Core " + String(xPortGetCoreID()));

  for (;;) {
    esp_task_wdt_reset();

    // ── DEFERRED PROCESSING dari ISR (R3) ──
    // Jika mode darurat aktif, nyalakan semua relay ventilasi
    if (emergencyMode) {
      if (!prevEmergency) {
        // Transisi masuk emergency: nyalakan blower + exhaust
        digitalWrite(RELAY_BLOWER,  HIGH);
        digitalWrite(RELAY_EXHAUST, HIGH);
        digitalWrite(RELAY_MOTOR,   LOW);  // Stop motor pakan saat darurat
        Serial.println("[Control] ⚠ EMERGENCY MODE ACTIVE — Blower & Exhaust ON");
        prevEmergency = true;
      }
      // Buang data queue agar tidak menumpuk (non-blocking receive)
      xQueueReceive(sensorQueue, &data, 0);
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    // Tandai recovery dari emergency
    if (prevEmergency) {
      Serial.println("[Control] ✓ Emergency cleared — returning to normal logic");
      prevEmergency = false;
    }

    // ── LOGIKA NORMAL: Tunggu data dari Queue (blocking) ──
    if (xQueueReceive(sensorQueue, &data, portMAX_DELAY) == pdTRUE) {

      // ── 1. KONTROL SUHU → RELAY_BLOWER ──
      // Threshold: SUHU_IDEAL = 23.0°C | SUHU_NORMAL_LOW = 20.0°C
      if (data.temperature > SUHU_IDEAL) {
        digitalWrite(RELAY_BLOWER, HIGH); // Suhu terlalu panas → kipas ON
        Serial.printf("[Control] Blower ON  (T=%.1f > %.1f°C)\n",
                      data.temperature, SUHU_IDEAL);
      } else if (data.temperature < SUHU_NORMAL_LOW) {
        digitalWrite(RELAY_BLOWER, LOW);  // Suhu sudah dingin → kipas OFF
        Serial.printf("[Control] Blower OFF (T=%.1f < %.1f°C)\n",
                      data.temperature, SUHU_NORMAL_LOW);
      }
      // Jika antara 20–23°C → pertahankan state sebelumnya (hysteresis)

      // ── 2. KONTROL GAS & SUHU → RELAY_EXHAUST ──
      // Threshold: AMBANG_AMONIA = 30 | AMBANG_AMONIA_LOW = 20
      if (data.temperature > SUHU_IDEAL || data.gas > AMBANG_AMONIA) {
        digitalWrite(RELAY_EXHAUST, HIGH); // Gas tinggi / suhu panas → buang udara
        Serial.printf("[Control] Exhaust ON  (T=%.1f | Gas=%d%%)\n",
                      data.temperature, data.gas);
      } else if (data.temperature < SUHU_NORMAL_LOW && data.gas < AMBANG_AMONIA_LOW) {
        digitalWrite(RELAY_EXHAUST, LOW);  // Kondisi aman → exhaust OFF
        Serial.printf("[Control] Exhaust OFF (T=%.1f | Gas=%d%%)\n",
                      data.temperature, data.gas);
      }

      // ── 3. KONTROL PAKAN → RELAY_MOTOR ──
      // Threshold: JARAK_PAKAN_KOSONG = 30cm | JARAK_PAKAN_PENUH = 10cm
      if (data.distance > JARAK_PAKAN_KOSONG) {
        digitalWrite(RELAY_MOTOR, HIGH); // Jarak jauh → wadah kosong → isi pakan
        Serial.printf("[Control] Motor ON  (Dist=%.1fcm > %.1fcm — PAKAN KOSONG)\n",
                      data.distance, JARAK_PAKAN_KOSONG);
      } else if (data.distance <= JARAK_PAKAN_PENUH) {
        digitalWrite(RELAY_MOTOR, LOW);  // Jarak dekat → wadah penuh → stop motor
        Serial.printf("[Control] Motor OFF (Dist=%.1fcm <= %.1fcm — PAKAN PENUH)\n",
                      data.distance, JARAK_PAKAN_PENUH);
      }
      // Jika antara 10–30cm → pertahankan state (transisi, tidak ubah relay)
    }
  }
}

// ============================================================
// TASK 4: LCD TASK (Tampilan Sensor + RTC)
// Priority: 1 | Core: 0 | Period: 1000ms
// Fungsi: Tampilkan data sensor + waktu di LCD 20x4
// Kontribusi anggota: [Nama Anggota 4]
// ============================================================
void lcdTask(void *pvParameters) {
  esp_task_wdt_add(NULL);
  SensorData data;

  Serial.println("[lcdTask] Started on Core " + String(xPortGetCoreID()));

  for (;;) {
    esp_task_wdt_reset();

    // Baca shared memory (proteksi Mutex)
    if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      data = currentData;
      xSemaphoreGive(sensorMutex);
    }

    DateTime now = rtc.now();
    char timeStr[9]; // "HH:MM:SS"
    char dateStr[9]; // "DD/MM/YY"
    sprintf(timeStr, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
    sprintf(dateStr, "%02d/%02d/%02d", now.day(), now.month(), now.year() % 100);

    lcd.clear();

    if (emergencyMode) {
      // ── Mode Darurat ──
      lcd.setCursor(4, 0); lcd.print(timeStr);
      lcd.setCursor(1, 1); lcd.print("!! EMERGENCY !!");
      lcd.setCursor(0, 2); lcd.print("BLOWER & EXHAUST ON");
      lcd.setCursor(2, 3); lcd.print("TEKAN UTK RESET");
    } else {
      // ── Mode Normal ──
      // Baris 0: Suhu | Berat
      lcd.setCursor(0, 0);
      lcd.print("T:"); lcd.print(data.temperature, 1); lcd.print((char)223); lcd.print("C");
      lcd.setCursor(10, 0);
      lcd.print("W:"); lcd.print(data.weight, 1); lcd.print("kg");

      // Baris 1: Kelembapan | Waktu
      lcd.setCursor(0, 1);
      lcd.print("H:"); lcd.print(data.humidity, 1); lcd.print("%");
      lcd.setCursor(11, 1); lcd.print(timeStr);

      // Baris 2: Gas Amonia | Tanggal
      lcd.setCursor(0, 2);
      lcd.print("G:"); lcd.print(data.gas); lcd.print("%");
      // Indikator threshold amonia
      lcd.setCursor(7, 2);
      lcd.print(data.gas > AMBANG_AMONIA ? "!" : " ");
      lcd.setCursor(11, 2); lcd.print(dateStr);

      // Baris 3: Jarak Pakan | Status Pakan
      lcd.setCursor(0, 3);
      lcd.print("F:"); lcd.print(data.distance, 1); lcd.print("cm");
      lcd.setCursor(10, 3);
      if      (data.distance > JARAK_PAKAN_KOSONG) lcd.print("ISIULANG");
      else if (data.distance <= JARAK_PAKAN_PENUH) lcd.print("PENUH   ");
      else                                          lcd.print("SEDANG  ");
    }

    vTaskDelay(pdMS_TO_TICKS(1000)); // Update setiap detik
  }
}

// ============================================================
// TASK 5: MQTT TASK (IoT Telemetri)
// Priority: 1 | Core: 0 | Period: 3000ms
// Fungsi: Kirim data sensor ke MQTT broker (tema IoT Smart Farm, R10)
// Kontribusi anggota: [Nama Anggota 5]
// ============================================================
void mqttTask(void *pvParameters) {
  esp_task_wdt_add(NULL);
  SensorData data;

  Serial.println("[mqttTask] Started on Core " + String(xPortGetCoreID()));

  for (;;) {
    esp_task_wdt_reset();

    // Cek & reconnect jika MQTT putus
    if (!client.connected()) reconnectMQTT();
    client.loop(); // Proses pesan masuk (non-blocking)

    // Baca shared memory
    if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      data = currentData;
      xSemaphoreGive(sensorMutex);
    }

    // ── Payload JSON sensor ──
    char payload[256];
    snprintf(payload, sizeof(payload),
      "{\"temperature\":%.1f,\"humidity\":%.1f,\"gas\":%d,"
      "\"distance\":%.1f,\"weight\":%.2f,\"emergency\":%d,"
      "\"blower\":%d,\"exhaust\":%d,\"motor\":%d}",
      data.temperature, data.humidity, data.gas,
      data.distance, data.weight, emergencyMode ? 1 : 0,
      digitalRead(RELAY_BLOWER), digitalRead(RELAY_EXHAUST), digitalRead(RELAY_MOTOR)
    );

    if (client.connected()) {
      client.publish(TOPIC_DATA, payload);
      Serial.println("[MQTT] Published: " + String(payload));

      // Kirim alert darurat ke topic terpisah
      if (emergencyMode) {
        client.publish(TOPIC_EMRG, "ACTIVE");
        Serial.println("[MQTT] Emergency alert published!");
      }
    }

    vTaskDelay(pdMS_TO_TICKS(3000)); // Kirim setiap 3 detik
  }
}

// ============================================================
// TASK 6: WATCHDOG TASK (System Monitor)
// Priority: 1 | Core: 0 | Period: 5000ms
// Fungsi: Monitor sistem, cetak stack usage semua task (R8)
// Kontribusi anggota: [Nama Anggota 6]
// ============================================================
void watchdogTask(void *pvParameters) {
  esp_task_wdt_add(NULL);
  uint32_t iteration = 0;

  Serial.println("[watchdogTask] Started on Core " + String(xPortGetCoreID()));

  for (;;) {
    esp_task_wdt_reset();

    // Sinyal ke binary semaphore bahwa sistem masih hidup
    xSemaphoreGive(systemAliveSem);

    iteration++;
    Serial.println("\n===== SYSTEM STATUS [iter=" + String(iteration) + "] =====");
    Serial.printf("  Free Heap   : %u bytes\n", esp_get_free_heap_size());
    Serial.printf("  Emergency   : %s\n", emergencyMode ? "ACTIVE" : "Normal");

    // ── Stack Usage Monitor (R8) ──
    // uxTaskGetStackHighWaterMark mengembalikan minimum free stack (words)
    // Makin kecil = makin dekat stack overflow; idealnya > 512 bytes
    if (hSensorTask)   Serial.printf("  [sensorTask]   HWM: %u words\n",
                                      uxTaskGetStackHighWaterMark(hSensorTask));
    if (hScaleTask)    Serial.printf("  [scaleTask]    HWM: %u words\n",
                                      uxTaskGetStackHighWaterMark(hScaleTask));
    if (hControlTask)  Serial.printf("  [controlTask]  HWM: %u words\n",
                                      uxTaskGetStackHighWaterMark(hControlTask));
    if (hLcdTask)      Serial.printf("  [lcdTask]      HWM: %u words\n",
                                      uxTaskGetStackHighWaterMark(hLcdTask));
    if (hMqttTask)     Serial.printf("  [mqttTask]     HWM: %u words\n",
                                      uxTaskGetStackHighWaterMark(hMqttTask));
    if (hWatchdogTask) Serial.printf("  [watchdogTask] HWM: %u words\n",
                                      uxTaskGetStackHighWaterMark(hWatchdogTask));

    // Publish stack monitor via MQTT (jika terkoneksi)
    if (client.connected()) {
      char stackPayload[200];
      snprintf(stackPayload, sizeof(stackPayload),
        "{\"heap\":%u,\"sensor_hwm\":%u,\"control_hwm\":%u,\"mqtt_hwm\":%u}",
        esp_get_free_heap_size(),
        hSensorTask  ? uxTaskGetStackHighWaterMark(hSensorTask)  : 0,
        hControlTask ? uxTaskGetStackHighWaterMark(hControlTask) : 0,
        hMqttTask    ? uxTaskGetStackHighWaterMark(hMqttTask)    : 0
      );
      client.publish(TOPIC_STACK, stackPayload);
    }

    Serial.println("===========================================\n");
    vTaskDelay(pdMS_TO_TICKS(5000)); // Monitor setiap 5 detik
  }
}

// =======================
// SETUP
// =======================
void setup() {
  Serial.begin(115200);
  Serial.println("\n\n========================================");
  Serial.println("  SMART CHICKEN CLOSE HOUSE - ESP32  ");
  Serial.println("  FreeRTOS v" + String(tskKERNEL_VERSION_NUMBER));
  Serial.println("========================================\n");

  // ── GPIO Setup ──
  pinMode(TRIG_PIN,      OUTPUT);
  pinMode(ECHO_PIN,      INPUT);
  pinMode(RELAY_BLOWER,  OUTPUT);
  pinMode(RELAY_EXHAUST, OUTPUT);
  pinMode(RELAY_MOTOR,   OUTPUT);

  // Status awal: semua relay OFF
  digitalWrite(RELAY_BLOWER,  LOW);
  digitalWrite(RELAY_EXHAUST, LOW);
  digitalWrite(RELAY_MOTOR,   LOW);

  // Emergency button: PULLUP, trigger ISR saat FALLING (tombol ditekan)
  pinMode(EMERGENCY_BTN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(EMERGENCY_BTN), emergencyISR, FALLING);

  // ── Peripheral Init ──
  dht.begin();
  Wire.begin(21, 22); // I2C: SDA=21, SCL=22
  scale.begin(HX_DT, HX_SCK);

  if (!rtc.begin()) {
    Serial.println("[RTC] ERROR: DS3231 tidak ditemukan!");
  } else {
    // Sync waktu RTC jika belum diset (opsional)
    if (rtc.lostPower()) {
      Serial.println("[RTC] Power lost, setting compile time...");
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    Serial.println("[RTC] OK");
  }

  // Kalibrasi HX711 (faktor 420 untuk Wokwi slider)
  scale.set_scale(420.0f);
  scale.tare();
  Serial.println("[HX711] Calibrated, scale=420.0, tare done");

  // LCD init
  lcd.init();
  lcd.backlight();
  lcd.setCursor(2, 0); lcd.print("SMART CHICKEN");
  lcd.setCursor(3, 1); lcd.print("CLOSE HOUSE");
  lcd.setCursor(1, 2); lcd.print("Sistem Kandang v2");
  lcd.setCursor(0, 3); lcd.print("FreeRTOS ESP32 2024");
  delay(2500);
  lcd.clear();

  // ── WiFi & MQTT ──
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setBufferSize(512); // Perluas buffer untuk payload besar

  // ── FreeRTOS Objects (R4) ──
  // Queue: kapasitas 10 item SensorData
  sensorQueue    = xQueueCreate(10, sizeof(SensorData));
  // Mutex: proteksi currentData
  sensorMutex    = xSemaphoreCreateMutex();
  // Binary semaphore: watchdog alive signal
  systemAliveSem = xSemaphoreCreateBinary();

  if (!sensorQueue || !sensorMutex || !systemAliveSem) {
    Serial.println("[ERROR] RTOS object creation FAILED!");
    while (1); // Halt
  }
  Serial.println("[RTOS] Queue, Mutex, Semaphore created OK");

  // ── Hardware Watchdog Timer (10 detik timeout) ──
  esp_task_wdt_config_t wdt_cfg = {
    .timeout_ms   = 10000,
    .idle_core_mask = (1 << 0) | (1 << 1),
    .trigger_panic  = true
  };
  esp_task_wdt_init(&wdt_cfg);
  Serial.println("[WDT] Hardware watchdog configured (10s timeout)");

  // ── Task Creation (R2 — 6 task, prioritas berbeda) ──
  // Memory allocation strategy (R7):
  //   - Stack dialokasi dari heap FreeRTOS saat xTaskCreate
  //   - Ukuran stack disesuaikan kebutuhan tiap task:
  //     * sensorTask 4096: butuh buffer DHT, sprintf, dll
  //     * scaleTask 2048: ringan, hanya baca HX711
  //     * controlTask 4096: logika kompleks + Serial printf
  //     * lcdTask 4096: buffer string lcd, RTC format
  //     * mqttTask 4096: buffer JSON besar + WiFi stack overhead
  //     * watchdogTask 2048: minimal, hanya Serial print
  xTaskCreatePinnedToCore(sensorTask,   "SensorTask",   4096, NULL, 3, &hSensorTask,   1);
  xTaskCreatePinnedToCore(scaleTask,    "ScaleTask",    2048, NULL, 2, &hScaleTask,    0);
  xTaskCreatePinnedToCore(controlTask,  "ControlTask",  4096, NULL, 5, &hControlTask,  1);
  xTaskCreatePinnedToCore(lcdTask,      "LcdTask",      4096, NULL, 1, &hLcdTask,      0);
  xTaskCreatePinnedToCore(mqttTask,     "MqttTask",     4096, NULL, 1, &hMqttTask,     0);
  xTaskCreatePinnedToCore(watchdogTask, "WatchdogTask", 2048, NULL, 1, &hWatchdogTask, 0);

  Serial.println("[Setup] All tasks created. Scheduler running...\n");
}

// loop() kosong — semua logika dikelola oleh FreeRTOS tasks
void loop() {
  // FreeRTOS scheduler mengelola semua task
  // loop() berjalan sebagai idle task di Core 1
  vTaskDelay(portMAX_DELAY);
}
