#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

typedef struct struct_message {
  int gas;
  int servo;
  int control;
} struct_message;

struct_message myData;

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Gas: ");
  Serial.print(myData.gas);
  Serial.print("  Servo: ");
  Serial.print(myData.servo);
  Serial.print("  Control: ");
  Serial.println(myData.control);
}

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