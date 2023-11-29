
#include "FOCMotor.h"

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("3");
  delay(1000);
  Serial.println("2");
  delay(1000);
  Serial.println("1");
  delay(1000);

  FOCMotorInit(5); // power supply voltage [5V]
}

void loop() {
// 1:closed-loop angle demo
// 0:closed-loop velocity demo
#if 0 //   
  float Kp = 0.056;
  float Sensor_Angle = sensor.getAngle();
  FOCSetTorque(_constrain(Kp * (getSerialDate() + FOCGetSensorDirection() * Sensor_Angle) * 180 / PI, -2, 2), electricalAngle());
  serialReceiveUserCommand();
  sensor.update();
#else
  FOCSetTorque(getSerialDate(), electricalAngle()); // The value of "getSerialDate()" is obtained through the serial port.
  serialReceiveUserCommand();                       // Default velocity is zero. Used serial port to set velocity
  sensor.update();
#endif
}
