#include "MT6701.h"

MT6701Sensor encoder = MT6701Sensor();
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  delay(1000);
  Serial.println("3");
  delay(1000);
  Serial.println("2");
  delay(1000);
  Serial.println("1");
  delay(1000);

  encoder.init();
}

float prev_angle = 0.0f;
float curr_angle = 0.0f;
void loop() {

  curr_angle = encoder.getSensorAngle();
  if (abs(curr_angle - prev_angle) > 0.05) {
    Serial.println(curr_angle);
    prev_angle = curr_angle;
  }
  delay(100);
}




