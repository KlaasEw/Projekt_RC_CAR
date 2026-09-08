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

  //Ende des Setup
  Serial.println("StartUp complet");

}

void loop() {
  //IBUS
  parseIbus();      //IBUS Auslesen
  fillRxFromIbus(); //Map IBUS zu Tel Struct

  //Befehle zur Motor und Servo Ansteruerung berechnen und anwenden
  computeCmd();     //Befehle aus Strukt Informationen berechnen
  applyCmd();       //Berechnete Befehle auf Motor und Servo anwenden

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

  //Serial Ausgabe
  Serial.print("rx_ok: ");
  Serial.print(tel.rx_ok);
  Serial.print(" rx_gas: ");
  Serial.print(tel.rx_gas);
  Serial.print(" cmd_gas: ");
  Serial.print(tel.cmd_gas);
  Serial.print(" rx_servo: ");
  Serial.print(tel.rx_servo);
  Serial.print(" cmd_servo: ");
  Serial.print(tel.cmd_servo);
  Serial.print(" rx_control: ");
  Serial.print(tel.rx_control);
  Serial.print(" flags: ");
  Serial.println(tel.flags);

  //ESP-NOW Senden
  if (espNowReady) {
    esp_err_t result = esp_now_send(receiverAddress, (uint8_t *)&tel, sizeof(tel));
    if (result != ESP_OK) {
      Serial.println("Fehler beim Senden");
    }
  }

  delay(20);
}

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

  int16_t gas = tel.rx_gas;
  if (tel.rx_control != 1) {
    gas = (int16_t)(gas * 20 / 100);
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