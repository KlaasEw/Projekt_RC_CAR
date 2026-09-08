#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <cstring>
#include <cstdio>

const uint8_t TEL_MAGIC = 0xA5;
const uint8_t TEL_VERSION = 1;

typedef struct __attribute__((packed)) {
  uint8_t  magic;
  uint8_t  version;
  uint16_t seq;
  uint32_t t_ms;

  int16_t  rx_gas;
  int16_t  rx_servo;
  uint8_t  rx_control;
  uint8_t  rx_ok;

  int16_t  cmd_gas;
  int16_t  cmd_servo;

  float    ax, ay, az;
  float    gx, gy, gz;
  float    temp_c;

  float    alt_rel_m;

  uint16_t vbat_mv;

  uint16_t lidar_mm[3];
  uint8_t  danger[3];
  uint8_t  danger_used;

  uint8_t  flags;
} TelemetryV1;

static_assert(sizeof(TelemetryV1) == 63, "TelemetryV1 muss 63 Byte sein");

TelemetryV1 tel;
TelemetryV1 telQueued;
volatile bool telPending = false;
portMUX_TYPE telMux = portMUX_INITIALIZER_UNLOCKED;

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);
void printTelCsv();

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("=== ESP-NOW Receiver gestartet ===");

  WiFi.mode(WIFI_STA);
  Serial.print("Eigene MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.println("Erwarte Pakete vom Transmitter...");

  if (esp_now_init() != ESP_OK) {
    Serial.println("Fehler: ESP-NOW Init fehlgeschlagen");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  if (!telPending) {
    return;
  }

  portENTER_CRITICAL(&telMux);
  tel = telQueued;
  telPending = false;
  portEXIT_CRITICAL(&telMux);

  printTelCsv();
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (len != (int)sizeof(TelemetryV1)) {
    return;
  }

  TelemetryV1 incoming;
  memcpy(&incoming, incomingData, sizeof(incoming));
  if (incoming.magic != TEL_MAGIC || incoming.version != TEL_VERSION) {
    return;
  }

  portENTER_CRITICAL(&telMux);
  telQueued = incoming;
  telPending = true;
  portEXIT_CRITICAL(&telMux);
}

void printTelCsv() {
  char line[256];
  snprintf(
      line, sizeof(line),
      "TEL,%u,%u,%lu,%d,%d,%u,%u,%d,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%u,%u,%u,%u,%u,%u,%u,%u,%u\n",
      (unsigned)tel.version,
      (unsigned)tel.seq,
      (unsigned long)tel.t_ms,
      (int)tel.rx_gas,
      (int)tel.rx_servo,
      (unsigned)tel.rx_control,
      (unsigned)tel.rx_ok,
      (int)tel.cmd_gas,
      (int)tel.cmd_servo,
      tel.ax, tel.ay, tel.az,
      tel.gx, tel.gy, tel.gz,
      tel.temp_c, tel.alt_rel_m,
      (unsigned)tel.vbat_mv,
      (unsigned)tel.lidar_mm[0],
      (unsigned)tel.lidar_mm[1],
      (unsigned)tel.lidar_mm[2],
      (unsigned)tel.danger[0],
      (unsigned)tel.danger[1],
      (unsigned)tel.danger[2],
      (unsigned)tel.danger_used,
      (unsigned)tel.flags);
  Serial.print(line);
}
