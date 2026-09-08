/*
Autor: Klaas Ewald
Version: 1.0
Datum: 12.09.2026
Zielsetzung: Notbremsassistent für ein RC-Auto

Der Mikrocontroller wird zwischen Receiver sowie ESC und Servo geschaltet.
Im ersten Fahrmodus werden die Empfängerwerte unverändert durchgereicht.
Im zweiten Fahrmodus wird die Ansteuerung basierend auf Sensorergebnissen
beeinflusst. Ziel ist es, eine Kollision des Autos mit anderen Objekten
zu verhindern. Dazu wird die Umgebung mit dem Lidar-Sensor gescannt.
Wird ein Hindernis erkannt, soll die Geschwindigkeit reduziert und im
Zweifelsfall das Fahrzeug gestoppt werden.

Zusätzlich sollen über ESP-NOW Telemetriedaten, die am Fahrzeug gemessen
wurden, an einen zweiten ESP übertragen werden. Dieser ESP dient als
Brücke zu einem auf dem PC laufenden Visualisierungsdashboard.
Die Kommunikation zwischen dem zweiten ESP und dem PC erfolgt über die
serielle Schnittstelle.

Hardware:
Auto:            Mali Racing BigHammer 2
Motor:           QweenHobby 3670-2650KV
ESC:             Hobbywing QuicRun WP 10BL120 G2 120A
Servo:           B7018 9kg
Receiver:        FlySky FS-iA6B
Steuerung:       FlySky FS-i6X
Akku:            Conrad Energy 3S 25C (1800 mAh, 11,1 V)
Mikrocontroller: ESP32 NodeMCU
Sensoren:        BME280 (Druck, Temperatur, Luftfeuchte)
                 MPU6050 (Beschleunigung und Gyroskop)
                 Slamtec RPLidar C1
Zubehör:         Logic-Level-Shifter
                 TS7805 (Spannungsregler 5 V)


======================================================================================================*/


//Bibliotheken
//======================================================================================================
#include <Arduino.h>
#include <ESP32Servo.h>
#include <cstring>
#include <cmath>
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <cstdio>

//Variablen
//======================================================================================================

//Variablen RPLidar C1
const int LIDAR_RX_PIN = 16;
const int LIDAR_TX_PIN = 17;

float minDistLeft = 99999.0;
float minDistCenter = 99999.0;
float minDistRight = 99999.0;

const float MIN_VALID_DISTANCE = 100.0;  // mm, 0 = kein Echo
const int16_t STEER_DEADZONE = 15;       // Grad, darunter gilt Geradeaus

byte paket[5];
int paketIndex = 0;
bool lidarReady = false;
unsigned long lastLidarScanMs = 0;
unsigned long lastLidarByteMs = 0;
unsigned long lastTelMs = 0;

struct LidarShare {
  uint16_t mm[3];
  uint8_t danger[3];
  uint8_t valid;
};
volatile LidarShare lidarShare = {};

//Variablen MPU6050 & BME280
static const int I2C_SDA_PIN = 21;
static const int I2C_SCL_PIN = 22;
static const float ALT_FILTER = 0.35f;

Adafruit_MPU6050 mpu;
Adafruit_BME280 bme;
bool mpuReady = false;
bool bmeReady = false;

float gyroOffsetX = 0;
float gyroOffsetY = 0;
float gyroOffsetZ = 0;
float pressureBaselinePa = 101325.0f;
float altFiltered = 0;

//Variablen ESP Now
uint8_t receiverAddress[] = {0x1C, 0xC3, 0xAB, 0xBC, 0x17, 0x28};

esp_now_peer_info_t peerInfo;
bool espNowReady = false;

//Variablen IBUS
const int IBUS_RX_PIN = 34;
const int MOTOR_PIN = 33;
const int SERVO_PIN = 32;

const int PWM_MIN_US = 1000;
const int PWM_NEUTRAL_US = 1500;
const int PWM_MAX_US = 2000;

Servo motor;
Servo steering;

const int IBUS_PACKET_SIZE = 32;
const unsigned long IBUS_STALE_MS = 100;

// 0-basiert: CH1=0 ... CH14=13
const int IBUS_CH_GAS = 1;
const int IBUS_CH_SERVO = 3;
const int IBUS_CH_CONTROL = 4;

uint8_t ibusBuf[IBUS_PACKET_SIZE];
int ibusIndex = 0;
uint16_t ibusChannels[14];
bool ibusValid = false;
unsigned long ibusLastMs = 0;

//Variablen Telemetrie Struct
const uint8_t TEL_MAGIC = 0xA5;
const uint8_t TEL_VERSION = 1;

const uint8_t FLAG_MPU = (1 << 0);
const uint8_t FLAG_BME = (1 << 1);
const uint8_t FLAG_LIDAR = (1 << 2);
const uint8_t FLAG_FAILSAFE = (1 << 3);
const uint8_t FLAG_DANGER = (1 << 4);
const uint8_t FLAG_VBAT = (1 << 5);

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

//Prototypen
//======================================================================================================

int mapGas(int pulseUs);
int mapServo(int pulseUs);
int mapControl(int pulseUs);
void writeMotor(int gas);
void writeSteering(int servo);
bool ibusChecksumOk();
void parseIbus();
void initTelemetry();
void fillRxFromIbus();
void computeCmd();
void applyCmd();
bool readMpu(TelemetryV1 &data);
float relativeAltitude(float pressurePa);
void readBme(TelemetryV1 &data);
void calibrateGyro();
void calibrateAltitude();
bool initBme();
void processLidarPoint(float angle, float distance, bool isNewScan);
void parseLidar();
void lidarTask(void *pv);
void applyLidarToTel();
void lidarBefehl(byte cmd);
void lidarBefehlWert(byte cmd, uint16_t wert);
bool warteAufAntwort(unsigned long timeoutMs);
uint16_t sectorToMm(float minDist);
uint8_t dangerFromMm(uint16_t mm);
void publishLidarScan();
void finishLidarScan();
void clearLidarTelemetry();
void updateDangerUsed();
void serialDebug();

//Setup
//======================================================================================================

void setup() {
  //Initialisierung der Seriellen Schnittstelle
  Serial.begin(115200);
  Serial.println("Start ESP_32");

  //Initialisierung der Telemetrie
  initTelemetry();

  //Initialisierung der Motoren und Servos
  motor.setPeriodHertz(50);
  steering.setPeriodHertz(50);
  motor.attach(MOTOR_PIN, PWM_MIN_US, PWM_MAX_US);
  steering.attach(SERVO_PIN, PWM_MIN_US, PWM_MAX_US);
  motor.writeMicroseconds(PWM_NEUTRAL_US);
  steering.writeMicroseconds(PWM_NEUTRAL_US);

  //Initialisierung der IBUS Schnittstelle
  Serial1.setRxBufferSize(1024);
  Serial1.begin(115200, SERIAL_8N1, IBUS_RX_PIN, -1);

  //Initialisierung WiFi-Modul
  WiFi.mode(WIFI_STA);
  Serial.print("Eigene MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("Fehler: ESP-NOW Init fehlgeschlagen");
  } else {
    memcpy(peerInfo.peer_addr, receiverAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Fehler: Peer konnte nicht hinzugefuegt werden");
    } else {
      espNowReady = true;
      Serial.println("ESP-NOW bereit");
    }
  }

  //Initialisierung des I2C-Interface
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);
  delay(50);

  //Initialisierung des MPU6050
  if (!mpu.begin(MPU6050_I2CADDR_DEFAULT, &Wire)) {
    Serial.println("Fehler: MPU6050 Init fehlgeschlagen");
  } else {
    mpuReady = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
    calibrateGyro();
    Serial.println("MPU6050 bereit");
  }

  //Initialisierung des BME280
  if (!initBme()) {
    Serial.println("Fehler: BME280 Init fehlgeschlagen");
  } else {
    bmeReady = true;
    calibrateAltitude();
    Serial.println("BME280 bereit");
  }

  //Initialisierung des RPLidar C1
  Serial2.setRxBufferSize(8192);
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
    Serial.println("Fehler: RPLidar Init fehlgeschlagen");
  } else {
    lidarReady = true;
    lastLidarScanMs = millis();
    lastLidarByteMs = millis();
    xTaskCreatePinnedToCore(lidarTask, "lidar", 4096, NULL, 3, NULL, 1); //Erstellt einen neuen Task für die Lidar-Verarbeitung, um die Hauptschleife nicht zu blockieren. Höhere Priorität als die Hauptschleife. Auch auf Kern 1 erstellen.
    Serial.println("RPLidar bereit");
  }

  //Ende des Setup
  Serial.println("StartUp complet");

}

//Loop
//======================================================================================================

void loop() {
  parseIbus();

  if (millis() - lastTelMs < 20) {
    return;
  }
  lastTelMs = millis();

  fillRxFromIbus();
  applyLidarToTel();

  computeCmd();
  applyCmd();

  //Auslesen MPU6050
  if (mpuReady) {
    readMpu(tel);
  }

  //Auslesen BME280
  if (bmeReady) {
    readBme(tel);
  } else {
    tel.alt_rel_m = 0;
    tel.flags &= ~FLAG_BME;
    if (!(tel.flags & FLAG_MPU)) {
      tel.temp_c = 0;
    }
  }

  //Serial Ausgabe für Debugging
  serialDebug();

  //ESP-NOW Senden
  if (espNowReady) {
    esp_err_t result = esp_now_send(receiverAddress, (uint8_t *)&tel, sizeof(tel));
    if (result != ESP_OK) {
      Serial.println("Fehler beim Senden");
    }
  }
}

//Funktionen
//======================================================================================================

int mapGas(int pulseUs) {
  return map(pulseUs, 999, 2000, -100, 100);
}

int mapServo(int pulseUs) {
  return map(pulseUs, 999, 2000, 45, -45);
}

int mapControl(int pulseUs) {
  return (pulseUs < 1500) ? 1 : 0;
}

void writeMotor(int gas) {
  motor.writeMicroseconds(constrain(map(gas, -100, 100, PWM_MIN_US, PWM_MAX_US), PWM_MIN_US, PWM_MAX_US));
}

void writeSteering(int servo) {
  steering.writeMicroseconds(constrain(map(servo, 45, -45, PWM_MIN_US, PWM_MAX_US), PWM_MIN_US, PWM_MAX_US));
}

bool ibusChecksumOk() {
  uint16_t sum = 0;
  for (int i = 0; i < 30; i++) {
    sum += ibusBuf[i];
  }
  uint16_t checksum = ibusBuf[30] | (ibusBuf[31] << 8);
  return checksum == (uint16_t)(0xFFFF - sum);
}

void parseIbus() {
  while (Serial1.available()) {
    uint8_t b = Serial1.read();

    if (ibusIndex == 0) {
      if (b != 0x20) {
        continue;
      }
      ibusBuf[ibusIndex++] = b;
      continue;
    }

    if (ibusIndex == 1 && b != 0x40) {
      ibusIndex = 0;
      if (b == 0x20) {
        ibusBuf[ibusIndex++] = b;
      }
      continue;
    }

    ibusBuf[ibusIndex++] = b;
    if (ibusIndex < IBUS_PACKET_SIZE) {
      continue;
    }

    if (ibusChecksumOk()) {
      for (int i = 0; i < 14; i++) {
        ibusChannels[i] = ibusBuf[2 + i * 2] | (ibusBuf[3 + i * 2] << 8);
      }
      ibusValid = true;
      ibusLastMs = millis();
    }
    ibusIndex = 0;
  }
}

void initTelemetry() {
  memset(&tel, 0, sizeof(tel));
  tel.magic = TEL_MAGIC;
  tel.version = TEL_VERSION;
}

void fillRxFromIbus() {
  bool ibusFresh = ibusValid && (millis() - ibusLastMs) < IBUS_STALE_MS;

  tel.t_ms = millis();
  tel.seq++;

  if (!ibusFresh) {
    tel.rx_ok = 0;
    return;
  }

  tel.rx_ok = 1;
  tel.rx_gas = (int16_t)mapGas(ibusChannels[IBUS_CH_GAS]);
  tel.rx_servo = (int16_t)mapServo(ibusChannels[IBUS_CH_SERVO]);
  tel.rx_control = (uint8_t)mapControl(ibusChannels[IBUS_CH_CONTROL]);
}

void computeCmd() {
  if (!tel.rx_ok) {
    tel.cmd_gas = 0;
    tel.flags |= FLAG_FAILSAFE;
    tel.flags &= ~FLAG_DANGER;
    return;
  }

  tel.flags &= ~FLAG_FAILSAFE;
  tel.flags &= ~FLAG_DANGER;

  int16_t gas = tel.rx_gas;
  if (tel.rx_control != 1) {
    if (!(tel.flags & FLAG_LIDAR)) { //Lidar Sensor fehlerhaft
      if (gas > 15) {
        gas = 15;
      }
      if (gas < -20) {
        gas = -20;
      }
      tel.flags |= FLAG_DANGER;
    } else if (tel.danger_used == 1) {  // Hinweis: 3000–5000 mm
      if (gas > 40) {
        gas = 40;
      }
      if (gas < -40) {
        gas = -40;
      }
    } else if (tel.danger_used == 2) {  // Warnung: 1000–3000 mm
      if (gas > 20) {
        gas = 20;
      }
      if (gas < -20) {
        gas = -20;
      }
      tel.flags |= FLAG_DANGER;
    } else if (tel.danger_used == 3) {  // Stopp: < 1000 mm
      if (gas > 10) {
        gas = 10;
      }
      if (gas < -15) {
        gas = -15;
      }
      tel.flags |= FLAG_DANGER;
    } else {  // frei: > 4000 mm 
      if (gas > 60) {
        gas = 60;
      }
      if (gas < -60) {
        gas = -60;
      }
    }
  }

  tel.cmd_gas = gas;
  tel.cmd_servo = tel.rx_servo;
}

void applyCmd() {
  writeMotor(tel.cmd_gas);
  writeSteering(tel.cmd_servo);
}

bool readMpu(TelemetryV1 &data) {
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t mpuTemp;
  if (!mpu.getEvent(&accel, &gyro, &mpuTemp)) {
    data.ax = 0;
    data.ay = 0;
    data.az = 0;
    data.gx = 0;
    data.gy = 0;
    data.gz = 0;
    data.flags &= ~FLAG_MPU;
    return false;
  }

  data.ax = accel.acceleration.x / SENSORS_GRAVITY_STANDARD;
  data.ay = accel.acceleration.y / SENSORS_GRAVITY_STANDARD;
  data.az = accel.acceleration.z / SENSORS_GRAVITY_STANDARD;
  data.gx = gyro.gyro.x * RAD_TO_DEG - gyroOffsetX;
  data.gy = gyro.gyro.y * RAD_TO_DEG - gyroOffsetY;
  data.gz = gyro.gyro.z * RAD_TO_DEG - gyroOffsetZ;
  data.temp_c = mpuTemp.temperature;
  data.flags |= FLAG_MPU;
  return true;
}

float relativeAltitude(float pressurePa) {
  if (pressureBaselinePa < 1.0f) {
    return 0;
  }
  return 44330.0f * (1.0f - pow(pressurePa / pressureBaselinePa, 0.1903f));
}

void readBme(TelemetryV1 &data) {
  float t = bme.readTemperature();
  float pressurePa = bme.readPressure();

  if (isnan(t) || pressurePa <= 10000.0f) {
    data.alt_rel_m = 0;
    data.flags &= ~FLAG_BME;
    if (!(data.flags & FLAG_MPU)) {
      data.temp_c = 0;
    }
    return;
  }

  data.temp_c = t;
  float altRaw = relativeAltitude(pressurePa);
  altFiltered += ALT_FILTER * (altRaw - altFiltered);
  data.alt_rel_m = altFiltered;
  data.flags |= FLAG_BME;
}

void calibrateGyro() {
  const int samples = 200;
  float sumX = 0;
  float sumY = 0;
  float sumZ = 0;
  int ok = 0;

  for (int i = 0; i < samples; i++) {
    if (!readMpu(tel)) {
      delay(10);
      continue;
    }
    sumX += tel.gx;
    sumY += tel.gy;
    sumZ += tel.gz;
    ok++;
    delay(10);
  }

  if (ok == 0) {
    return;
  }

  gyroOffsetX = sumX / ok;
  gyroOffsetY = sumY / ok;
  gyroOffsetZ = sumZ / ok;
}

void calibrateAltitude() {
  const int samples = 40;
  float sumP = 0;
  int ok = 0;
  for (int i = 0; i < samples; i++) {
    float p = bme.readPressure();
    if (p > 10000.0f) {
      sumP += p;
      ok++;
    }
    delay(25);
  }
  if (ok > 0) {
    pressureBaselinePa = sumP / ok;
  }
  altFiltered = 0;
}

bool initBme() {
  if (bme.begin(0x76, &Wire) || bme.begin(0x77, &Wire)) {
    bme.setSampling(
        Adafruit_BME280::MODE_NORMAL,
        Adafruit_BME280::SAMPLING_X1,
        Adafruit_BME280::SAMPLING_X16,
        Adafruit_BME280::SAMPLING_NONE,
        Adafruit_BME280::FILTER_X16,
        Adafruit_BME280::STANDBY_MS_0_5);
    return true;
  }
  return false;
}

//Lidar Task (FreeRTOS Task)
void lidarTask(void *pv) {
  (void)pv;
  for (;;) {
    parseLidar();
    vTaskDelay(1);
  }
}

void applyLidarToTel() {
  tel.lidar_mm[0] = lidarShare.mm[0];
  tel.lidar_mm[1] = lidarShare.mm[1];
  tel.lidar_mm[2] = lidarShare.mm[2];
  tel.danger[0] = lidarShare.danger[0];
  tel.danger[1] = lidarShare.danger[1];
  tel.danger[2] = lidarShare.danger[2];
  if (lidarShare.valid) {
    tel.flags |= FLAG_LIDAR;
  } else {
    tel.flags &= ~FLAG_LIDAR;
  }
  updateDangerUsed();
}

void parseLidar() {
  if (!lidarReady) {
    clearLidarTelemetry();
    return;
  }

  int processed = 0;
  while (Serial2.available()) {
    lastLidarByteMs = millis();
    paket[paketIndex] = Serial2.read();
    paketIndex++;

    if (paketIndex < 5) continue;

    int startBit = paket[0] & 0x01;
    int startBitInv = (paket[0] >> 1) & 0x01;
    int checkBit = paket[1] & 0x01;

    if (startBit == startBitInv || checkBit != 1) {
      for (int i = 0; i < 4; i++) paket[i] = paket[i + 1];
      paketIndex = 4;
      if (++processed >= 256) {
        break;
      }
      continue;
    }

    float winkel = ((paket[1] >> 1) | (paket[2] << 7)) / 64.0;
    if (winkel >= 360.0) winkel = winkel - 360.0;
    float distanz = (paket[3] | (paket[4] << 8)) / 4.0;

    paketIndex = 0;
    processLidarPoint(winkel, distanz, startBit == 1);
    if (++processed >= 256) {
      break;
    }
  }

  unsigned long now = millis();
  if (lastLidarByteMs != 0 && (now - lastLidarByteMs) > 300) {
    while (Serial2.available()) {
      Serial2.read();
    }
    paketIndex = 0;
    minDistLeft = 99999.0;
    minDistCenter = 99999.0;
    minDistRight = 99999.0;
    clearLidarTelemetry();
    return;
  }

  if (lastLidarScanMs != 0 && (now - lastLidarScanMs) >= 180) {
    finishLidarScan();
  }
}

void processLidarPoint(float angle, float distance, bool isNewScan) {
  if (isNewScan) {
    unsigned long now = millis();
    if (now - lastLidarScanMs >= 80) {
      finishLidarScan();
    }
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

uint16_t sectorToMm(float minDist) {
  if (minDist >= 99999.0f) {
    return 0;
  }
  return (uint16_t)minDist;
}

uint8_t dangerFromMm(uint16_t mm) {
  if (mm == 0) {
    return 0;
  }
  if (mm < 1000) {
    return 3;
  }
  if (mm < 3000) {
    return 2;
  }
  if (mm <= 5000) {
    return 1;
  }
  return 0;
}

void publishLidarScan() {
  lidarShare.mm[0] = sectorToMm(minDistLeft);
  lidarShare.mm[1] = sectorToMm(minDistCenter);
  lidarShare.mm[2] = sectorToMm(minDistRight);
  lidarShare.danger[0] = dangerFromMm(lidarShare.mm[0]);
  lidarShare.danger[1] = dangerFromMm(lidarShare.mm[1]);
  lidarShare.danger[2] = dangerFromMm(lidarShare.mm[2]);
  lidarShare.valid = 1;
}

void finishLidarScan() {
  bool scanHatDaten = (minDistLeft < 99999.0f) ||
                     (minDistCenter < 99999.0f) ||
                     (minDistRight < 99999.0f);
  if (scanHatDaten) {
    publishLidarScan();
  }
  minDistLeft = 99999.0;
  minDistCenter = 99999.0;
  minDistRight = 99999.0;
  lastLidarScanMs = millis();
}

void clearLidarTelemetry() {
  lidarShare.mm[0] = 0;
  lidarShare.mm[1] = 0;
  lidarShare.mm[2] = 0;
  lidarShare.danger[0] = 0;
  lidarShare.danger[1] = 0;
  lidarShare.danger[2] = 0;
  lidarShare.valid = 0;
}

void updateDangerUsed() {
  if (!(tel.flags & FLAG_LIDAR)) {
    tel.danger_used = 0;
    return;
  }

  // rx_servo: links +, rechts −
  if (tel.rx_servo > STEER_DEADZONE) {
    tel.danger_used = tel.danger[2];
  } else if (tel.rx_servo < -STEER_DEADZONE) {
    tel.danger_used = tel.danger[0];
  } else {
    tel.danger_used = tel.danger[1];
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

void serialDebug() {
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