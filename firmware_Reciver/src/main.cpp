#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <cstring>

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
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (len != (int)sizeof(TelemetryV1)) {
    return;
  }

  memcpy(&tel, incomingData, sizeof(tel));

  if (tel.magic != TEL_MAGIC || tel.version != TEL_VERSION) {
    return;
  }

  printTelCsv();
}

void printTelCsv() {
  Serial.print("TEL,");
  Serial.print(tel.version);
  Serial.print(',');
  Serial.print(tel.seq);
  Serial.print(',');
  Serial.print(tel.t_ms);
  Serial.print(',');
  Serial.print(tel.rx_gas);
  Serial.print(',');
  Serial.print(tel.rx_servo);
  Serial.print(',');
  Serial.print(tel.rx_control);
  Serial.print(',');
  Serial.print(tel.rx_ok);
  Serial.print(',');
  Serial.print(tel.cmd_gas);
  Serial.print(',');
  Serial.print(tel.cmd_servo);
  Serial.print(',');
  Serial.print(tel.ax, 3);
  Serial.print(',');
  Serial.print(tel.ay, 3);
  Serial.print(',');
  Serial.print(tel.az, 3);
  Serial.print(',');
  Serial.print(tel.gx, 3);
  Serial.print(',');
  Serial.print(tel.gy, 3);
  Serial.print(',');
  Serial.print(tel.gz, 3);
  Serial.print(',');
  Serial.print(tel.temp_c, 3);
  Serial.print(',');
  Serial.print(tel.alt_rel_m, 3);
  Serial.print(',');
  Serial.print(tel.vbat_mv);
  Serial.print(',');
  Serial.print(tel.lidar_mm[0]);
  Serial.print(',');
  Serial.print(tel.lidar_mm[1]);
  Serial.print(',');
  Serial.print(tel.lidar_mm[2]);
  Serial.print(',');
  Serial.print(tel.danger[0]);
  Serial.print(',');
  Serial.print(tel.danger[1]);
  Serial.print(',');
  Serial.print(tel.danger[2]);
  Serial.print(',');
  Serial.print(tel.danger_used);
  Serial.print(',');
  Serial.println(tel.flags);
}
