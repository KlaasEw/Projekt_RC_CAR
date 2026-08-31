#include <Arduino.h>
#include "Adafruit_VL53L0X.h"

// XSHUT Pins
#define SENSOR1_XSHUT 25
#define SENSOR2_XSHUT 26
#define SENSOR3_XSHUT 27

// Eindeutige I2C-Adressen (Standard ist 0x29, wir verschieben sie)
#define ADDR_SENSOR1 0x30
#define ADDR_SENSOR2 0x31
#define ADDR_SENSOR3 0x32

// Drei eigenständige Sensor-Objekte im Speicher halten
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();

void setup() {
  Serial.begin(115200);
  Wire.setClock(100000);
  while (!Serial) { delay(1); }
  delay(100);

  Serial.println("Starte saubere Adress-Zuweisung...");

  // Alle Steuer-Pins vorbereiten
  pinMode(SENSOR1_XSHUT, OUTPUT);
  pinMode(SENSOR2_XSHUT, OUTPUT);
  pinMode(SENSOR3_XSHUT, OUTPUT);

  // SCHRITT 1: Alle Sensoren komplett abschalten (Reset)
  digitalWrite(SENSOR1_XSHUT, LOW);
  digitalWrite(SENSOR2_XSHUT, LOW);
  digitalWrite(SENSOR3_XSHUT, LOW);
  delay(50); // Genug Zeit geben, damit Restspannungen abgebaut werden

  // SCHRITT 2: Sensor 1 aktivieren & Adresse zuweisen
  digitalWrite(SENSOR1_XSHUT, HIGH);
  delay(20); // Aufwachphase abwarten
  if (!lox1.begin(ADDR_SENSOR1)) {
    Serial.println("Kritischer Fehler: Sensor 1 reagiert nicht!");
    while (1);
  }
  Serial.println("Sensor 1 erfolgreich auf Adresse 0x30 registriert.");

  // SCHRITT 3: Sensor 2 aktivieren & Adresse zuweisen
  digitalWrite(SENSOR2_XSHUT, HIGH);
  delay(20);
  if (!lox2.begin(ADDR_SENSOR2)) {
    Serial.println("Kritischer Fehler: Sensor 2 reagiert nicht!");
    while (1);
  }
  Serial.println("Sensor 2 erfolgreich auf Adresse 0x31 registriert.");

  // SCHRITT 4: Sensor 3 aktivieren & Adresse zuweisen
  digitalWrite(SENSOR3_XSHUT, HIGH);
  delay(20);
  if (!lox3.begin(ADDR_SENSOR3)) {
    Serial.println("Kritischer Fehler: Sensor 3 reagiert nicht!");
    while (1);
  }
  Serial.println("Sensor 3 erfolgreich auf Adresse 0x32 registriert.");

  Serial.println("--- Alle Systeme bereit. Starte Messung... ---");
}

void loop() {
  VL53L0X_RangingMeasurementData_t m1, m2, m3;

  // Parallele Abfrage über die eindeutigen Adressen
  lox1.rangingTest(&m1, false);
  delay(30); 
  lox2.rangingTest(&m2, false);
  delay(30); 
  lox3.rangingTest(&m3, false);
  delay(30); 

  // Sensor 1 ausgeben
  Serial.print("S1: ");
  if (m1.RangeStatus != 4 && m1.RangeMilliMeter < 8000) Serial.print(m1.RangeMilliMeter);
  else Serial.print("Out of range");

  // Sensor 2 ausgeben
  Serial.print(" mm | S2: ");
  if (m2.RangeStatus != 4 && m2.RangeMilliMeter < 8000) Serial.print(m2.RangeMilliMeter);
  else Serial.print("Out of range");

  // Sensor 3 ausgeben
  Serial.print(" mm | S3: ");
  if (m3.RangeStatus != 4 && m3.RangeMilliMeter < 8000) Serial.print(m3.RangeMilliMeter);
  else Serial.print("Out of range");

  Serial.println(" mm");

  delay(250); // Kurze Pause für bessere Lesbarkeit im Monitor
}

