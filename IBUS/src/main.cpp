#include <Arduino.h>

const int RC_Gas_Pin = 33;
const int RC_Servo_Pin = 4;
const int RC_Control_Pin = 32;

void setup() {
  Serial.begin(115200);     // debug info
  Serial.println("Start ESP_32");
  pinMode(RC_Gas_Pin, INPUT);
  pinMode(RC_Servo_Pin, INPUT);
  pinMode(RC_Control_Pin,INPUT);
}


void loop() {

  int Gas_Width = map(pulseIn(RC_Gas_Pin, HIGH, 25000),999,2000,-100,100);
  int Servo_Width = map(pulseIn(RC_Servo_Pin, HIGH, 25000),999,2000,45,-45);
  int Control_Width = pulseIn(RC_Control_Pin, HIGH, 25000);
  if (Control_Width < 1500)
  {
    Control_Width = 1;
  }else
  {
    Control_Width = 0;
  }
  
  Serial.print("Gas: ");
  Serial.print(Gas_Width);
  Serial.print(" Servo: ");
  Serial.print(Servo_Width);
  Serial.print(" Control: ");
  Serial.println(Control_Width);
 
  delay(20);
}