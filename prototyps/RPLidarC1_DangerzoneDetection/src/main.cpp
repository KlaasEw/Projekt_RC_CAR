#include <Arduino.h>

// Lidar an UART2: ESP-RX <- Lidar-TX (gelb), ESP-TX -> Lidar-RX (gruen)
const int LIDAR_RX_PIN = 16;
const int LIDAR_TX_PIN = 17;

float minDistLeft = 99999.0;
float minDistCenter = 99999.0;
float minDistRight = 99999.0;

const float MIN_VALID_DISTANCE = 100.0;  // mm, 0 = kein Echo

byte paket[5];
int paketIndex = 0;

void processLidarPoint(float angle, float distance, bool isNewScan) {
  if (isNewScan) {
    Serial.printf("Links: %.0f mm | Mitte: %.0f mm | Rechts: %.0f mm\n",
                  minDistLeft == 99999.0 ? 0 : minDistLeft,
                  minDistCenter == 99999.0 ? 0 : minDistCenter,
                  minDistRight == 99999.0 ? 0 : minDistRight);

    minDistLeft = 99999.0;
    minDistCenter = 99999.0;
    minDistRight = 99999.0;
  }

  if (distance < MIN_VALID_DISTANCE) return;
  if (distance > 12000.0) return;

  // 0° = vorne (Kabel/Markierung). Je Zone 30°.
  // Links: 315-345°   Mitte: 345-15°   Rechts: 15-45°
  if (angle >= 315.0 && angle < 345.0) {
    if (distance < minDistLeft) minDistLeft = distance;
  } else if (angle >= 345.0 || angle < 15.0) {
    if (distance < minDistCenter) minDistCenter = distance;
  } else if (angle >= 15.0 && angle < 45.0) {
    if (distance < minDistRight) minDistRight = distance;
  }
}

void lidarBefehl(byte cmd) {
  Serial2.write(0xA5);
  Serial2.write(cmd);
  Serial2.flush();
}

void lidarBefehlWert(byte cmd, uint16_t wert) {
  byte frame[6];
  frame[0] = 0xA5;
  frame[1] = cmd;
  frame[2] = 2;
  frame[3] = wert & 0xFF;
  frame[4] = (wert >> 8) & 0xFF;
  frame[5] = frame[0] ^ frame[1] ^ frame[2] ^ frame[3] ^ frame[4];
  Serial2.write(frame, 6);
  Serial2.flush();
}

bool warteAufAntwort(unsigned long timeoutMs) {
  unsigned long start = millis();
  int vorher = 0;

  while (millis() - start < timeoutMs) {
    if (!Serial2.available()) {
      delay(1);
      continue;
    }

    int b = Serial2.read();
    if (vorher == 0xA5 && b == 0x5A) {
      for (int i = 0; i < 5; i++) {
        unsigned long waitStart = millis();
        while (!Serial2.available()) {
          if (millis() - waitStart > timeoutMs) return false;
        }
        Serial2.read();
      }
      return true;
    }
    vorher = b;
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial2.setRxBufferSize(4096);
  Serial2.begin(460800, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
  delay(50);

  lidarBefehl(0x25);  // STOP
  delay(50);
  lidarBefehl(0x40);  // RESET
  delay(2000);
  while (Serial2.available()) Serial2.read();

  lidarBefehlWert(0xA8, 600);  // Motor RPM
  lidarBefehlWert(0xF0, 660);  // Motor PWM
  delay(1200);
  while (Serial2.available()) Serial2.read();

  lidarBefehl(0x20);  // SCAN starten
  if (!warteAufAntwort(4000)) {
    Serial.println("Lidar startet nicht. Kabel und 5V pruefen.");
  } else {
    Serial.println("Lidar laeuft.");
  }
}

void loop() {
  while (Serial2.available()) {
    paket[paketIndex] = Serial2.read();
    paketIndex++;

    if (paketIndex < 5) continue;

    // Slamtec-Paket: Startbit und Checkbit muessen passen
    int startBit = paket[0] & 0x01;
    int startBitInv = (paket[0] >> 1) & 0x01;
    int checkBit = paket[1] & 0x01;

    if (startBit == startBitInv || checkBit != 1) {
      for (int i = 0; i < 4; i++) paket[i] = paket[i + 1];
      paketIndex = 4;
      continue;
    }

    bool neuerUmlauf = (startBit == 1);
    float winkel = ((paket[1] >> 1) | (paket[2] << 7)) / 64.0;
    if (winkel >= 360.0) winkel = winkel - 360.0;
    float distanz = (paket[3] | (paket[4] << 8)) / 4.0;

    paketIndex = 0;
    processLidarPoint(winkel, distanz, neuerUmlauf);
  }
}
