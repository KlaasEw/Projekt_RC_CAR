#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

static const int I2C_SDA_PIN = 21;
static const int I2C_SCL_PIN = 22;
static const float ALT_FILTER = 0.35f;

Adafruit_MPU6050 mpu;
Adafruit_BME280 bme;

typedef struct struct_mpu {
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  float tempC;
  float altRel;
} struct_mpu;

struct_mpu mpuData;

float gyroOffsetX = 0;
float gyroOffsetY = 0;
float gyroOffsetZ = 0;
float pressureBaselinePa = 101325.0f;
float altFiltered = 0;

bool readMpu(struct_mpu &data) {
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t mpuTemp;
  if (!mpu.getEvent(&accel, &gyro, &mpuTemp)) {
    return false;
  }

  data.ax = accel.acceleration.x / SENSORS_GRAVITY_STANDARD;
  data.ay = accel.acceleration.y / SENSORS_GRAVITY_STANDARD;
  data.az = accel.acceleration.z / SENSORS_GRAVITY_STANDARD;
  data.gx = gyro.gyro.x * RAD_TO_DEG - gyroOffsetX;
  data.gy = gyro.gyro.y * RAD_TO_DEG - gyroOffsetY;
  data.gz = gyro.gyro.z * RAD_TO_DEG - gyroOffsetZ;
  return true;
}

float relativeAltitude(float pressurePa) {
  if (pressureBaselinePa < 1.0f) {
    return 0;
  }
  return 44330.0f * (1.0f - pow(pressurePa / pressureBaselinePa, 0.1903f));
}

void readBme(struct_mpu &data) {
  data.tempC = bme.readTemperature();
  float pressurePa = bme.readPressure();
  float altRaw = relativeAltitude(pressurePa);
  altFiltered += ALT_FILTER * (altRaw - altFiltered);
  data.altRel = altFiltered;
}

void calibrateGyro() {
  const int samples = 200;
  float sumX = 0;
  float sumY = 0;
  float sumZ = 0;
  int ok = 0;

  for (int i = 0; i < samples; i++) {
    if (!readMpu(mpuData)) {
      delay(10);
      continue;
    }
    sumX += mpuData.gx;
    sumY += mpuData.gy;
    sumZ += mpuData.gz;
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

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);
  delay(50);

  if (!mpu.begin(MPU6050_I2CADDR_DEFAULT, &Wire)) {
    while (true) {
      delay(1000);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  if (!initBme()) {
    while (true) {
      delay(1000);
    }
  }

  calibrateGyro();
  calibrateAltitude();
}

void loop() {
  if (!readMpu(mpuData)) {
    delay(500);
    return;
  }
  readBme(mpuData);

  Serial.printf("DATA,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.1f,%.3f\n",
                mpuData.ax, mpuData.ay, mpuData.az,
                mpuData.gx, mpuData.gy, mpuData.gz,
                mpuData.tempC, mpuData.altRel);

  delay(50);
}
