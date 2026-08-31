#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

const int RC_Gas_Pin = 33;
const int RC_Servo_Pin = 4;
const int RC_Control_Pin = 32;

uint8_t broadcastAddress[] = {0x1C, 0xC3, 0xAB, 0xBC, 0x17, 0x28};

// Muss exakt zur Receiver-Struktur passen (int, nicht float)
typedef struct struct_message {
  int gas;
  int servo;
  int control;
} struct_message;

struct_message myData;

esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("=== ESP-NOW Transmitter gestartet ===");

  pinMode(RC_Gas_Pin, INPUT);
  pinMode(RC_Servo_Pin, INPUT);
  pinMode(RC_Control_Pin, INPUT);

  WiFi.mode(WIFI_STA);
  Serial.print("Eigene MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("Fehler: ESP-NOW Init fehlgeschlagen");
    return;
  }
  Serial.println("ESP-NOW bereit");

  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Fehler: Peer konnte nicht hinzugefuegt werden");
    return;
  }
  Serial.println("Peer registriert, starte Loop...");
}

void loop() {
  unsigned long gasPulse = pulseIn(RC_Gas_Pin, HIGH, 25000);
  unsigned long servoPulse = pulseIn(RC_Servo_Pin, HIGH, 25000);
  unsigned long controlPulse = pulseIn(RC_Control_Pin, HIGH, 25000);

  myData.gas = map((int)gasPulse, 999, 2000, -100, 100);
  myData.servo = map((int)servoPulse, 999, 2000, 45, -45);
  myData.control = (controlPulse < 1500) ? 1 : 0;

  Serial.print("Gas: ");
  Serial.print(myData.gas);
  Serial.print("  Servo: ");
  Serial.print(myData.servo);
  Serial.print("  Control: ");
  Serial.println(myData.control);


  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
  if (result != ESP_OK) {
    Serial.println("Fehler beim Senden");
  }

  delay(200);
}