#include <Arduino.h>
#include <ESP32Servo.h>

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

void setup() {
  Serial.begin(115200);
  Serial.println("Start ESP_32");

  motor.setPeriodHertz(50);
  steering.setPeriodHertz(50);
  motor.attach(MOTOR_PIN, PWM_MIN_US, PWM_MAX_US);
  steering.attach(SERVO_PIN, PWM_MIN_US, PWM_MAX_US);
  motor.writeMicroseconds(PWM_NEUTRAL_US);
  steering.writeMicroseconds(PWM_NEUTRAL_US);

  Serial1.setRxBufferSize(1024);
  Serial1.begin(115200, SERIAL_8N1, IBUS_RX_PIN, -1);
}

void loop() {
  parseIbus();

  bool ibusFresh = ibusValid && (millis() - ibusLastMs) < IBUS_STALE_MS;
  if (!ibusFresh) {
    motor.writeMicroseconds(PWM_NEUTRAL_US);
    Serial.println("IBUS: kein Frame");
    delay(20);
    return;
  }

  int gas = mapGas(ibusChannels[IBUS_CH_GAS]);
  int servo = mapServo(ibusChannels[IBUS_CH_SERVO]);
  int control = mapControl(ibusChannels[IBUS_CH_CONTROL]);

  switch (control) {
    case 1:
      break;
    case 0:
    default:
      gas = gas * 20 / 100;
      break;
  }

  writeMotor(gas);
  writeSteering(servo);

  Serial.print("Gas: ");
  Serial.print(gas);
  Serial.print(" Servo: ");
  Serial.print(servo);
  Serial.print(" Control: ");
  Serial.println(control);

  delay(20);
}
