
#include "FOCMotor.h"

/****************************************************************************************************
 *                                          FOC Motor
 ****************************************************************************************************/
float voltage_power_supply = 5;
int sensor_direction = -1;  // anticlockwise
int pole_pairs = 7;
float zero_electric_angle = 0;
float Ualpha, Ubeta = 0, Ua = 0, Ub = 0, Uc = 0;

MTC6300Driver drive = MTC6300Driver();
MT6701Sensor sensor = MT6701Sensor();

void FOCMotorInit(float power_supply) {
  voltage_power_supply = power_supply;

  drive.init();
  sensor.init();
}

void FOCMotorAlignSensor() {
  // align sensor
  FOCSetTorque(3, _3PI_2);
  delay(700);
  zero_electric_angle = electricalAngle();
  FOCSetTorque(0, _3PI_2);
  Serial.print("2.zero_electric_angle:");
  Serial.println(zero_electric_angle);
}

int FOCGetSensorDirection() {
  return sensor_direction;
}

float electricalAngle() {
  return _normalizeAngle((float)(sensor_direction * pole_pairs) * sensor.getMechanicalAngle() - zero_electric_angle);
}

float _normalizeAngle(float angle) {
  float a = fmod(angle, 2 * PI);
  return a >= 0 ? a : (a + 2 * PI);
}

// Set PWM to control motors
void setPwm(float Ua, float Ub, float Uc) {
  // 限制上限
  Ua = _constrain(Ua, 0.0f, voltage_power_supply);
  Ub = _constrain(Ub, 0.0f, voltage_power_supply);
  Uc = _constrain(Uc, 0.0f, voltage_power_supply);
  // 计算占空比
  // 限制占空比从0到1
  float dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
  float dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
  float dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);

  dc_a = (dc_a * 100 * 255) / 100;
  dc_b = (dc_b * 100 * 255) / 100;
  dc_c = (dc_c * 100 * 255) / 100;
  // printf("%d,%d,%d\r\n", (int)dc_a, (int)dc_b, (int)dc_c);
  drive.setPwmValue((int)dc_a, (int)dc_b, (int)dc_c);
}

void FOCSetTorque(float Uq, float angle_el) {
  Uq = _constrain(Uq, -voltage_power_supply / 2, voltage_power_supply / 2);

  angle_el = _normalizeAngle(angle_el + zero_electric_angle);
  // 帕克逆变换
  Ualpha = -Uq * sin(angle_el);
  Ubeta = Uq * cos(angle_el);

  // 克拉克逆变换
  Ua = Ualpha + voltage_power_supply / 2;
  Ub = (sqrt(3) * Ubeta - Ualpha) / 2 + voltage_power_supply / 2;
  Uc = (-Ualpha - sqrt(3) * Ubeta) / 2 + voltage_power_supply / 2;
  setPwm(Ua, Ub, Uc);
}

/****************************************************************************************************
 *                                           PID 
 ****************************************************************************************************/
PIDController velocity_pid = PIDController{ 2, 0, 0, 100000, voltage_power_supply / 2 };
PIDController angle_pid = PIDController{ 2, 0, 0, 100000, 100 };

void PIDSetVelocityParam(float P,float I,float D,float ramp)
{
  velocity_pid.P = P;
  velocity_pid.I = I;
  velocity_pid.D = D;
  velocity_pid.output_ramp = ramp;
}

void PIDSetAngleParam(float P,float I,float D,float ramp)
{
  angle_pid.P = P;
  angle_pid.I = I;
  angle_pid.D = D;
  angle_pid.output_ramp = ramp;
}

float PIDGetVelocityOutput(float error)
{
  return velocity_pid(error);
}

float PIDGetAngleOutput(float error)
{
  return angle_pid(error);
}

/****************************************************************************************************
 *                                      Lowpass filter 
 ****************************************************************************************************/
LowPassFilter lowpass_filter = LowPassFilter(0.01);

float LFGetVelocity()
{
  float v = sensor.getVelocity();
  float lf = lowpass_filter(sensor_direction * v);
  return lf;
}

/****************************************************************************************************
 *                                      Test Interface 
 ****************************************************************************************************/
// #define DEF_DEBUG_ENABLE

void SetVelocityLoop(float target)
{
  float lf_velocity = LFGetVelocity();
  float Uq = PIDGetVelocityOutput((target + lf_velocity) * 180 / PI);
  FOCSetTorque(Uq, electricalAngle());

#ifdef DEF_DEBUG_ENABLE
  static int cnt = 0;
  cnt++;
  if(cnt>20){
    cnt = 0;
    printf("%.2f,%.2f,%.2f\n", lf_velocity, Uq, electricalAngle());
  }
#endif
}

void SetVelocityAngleLoop(float target)
{
  float angle = sensor.getAngle() * sensor_direction;
  float angle_pid_out = PIDGetAngleOutput((target + angle) * 180 / PI);
  float Uq = PIDGetVelocityOutput(angle_pid_out);
  FOCSetTorque(Uq, electricalAngle());

#ifdef DEF_DEBUG_ENABLE
  static int cnt = 0;
  cnt++;
  if(cnt>20){
    cnt = 0;
    printf("%.2f,%.2f,%.2f,%.2f\n", angle, angle_pid_out, Uq, electricalAngle());
  }
#endif
}

void SetForceAngleLoop(float target)
{
  float angle = sensor.getAngle() * sensor_direction;
  float Uq = PIDGetAngleOutput((target + angle) * 180 / PI);
  FOCSetTorque(Uq, electricalAngle());
#ifdef DEF_DEBUG_ENABLE
  static int cnt = 0;
  cnt++;
  if(cnt>20){
    cnt = 0;
    printf("%.2f,%.2f,%.2f\n", angle, Uq, electricalAngle());
  }
#endif
}

void SetTorque(float target)
{
  FOCSetTorque(target, electricalAngle());
}


/****************************************************************************************************
 *                                        Serial port 
 ****************************************************************************************************/
float motor_target = 0;
int commaPosition;
String serialReceiveUserCommand() {

  // a string to hold incoming data
  static String received_chars;

  String command = "";

  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the string buffer:
    received_chars += inChar;

    // end of user input
    if (inChar == '\n') {

      // execute the user command
      command = received_chars;

      commaPosition = command.indexOf('\n');  //检测字符串中的逗号
      if (commaPosition != -1)                //如果有逗号存在就向下执行
      {
        motor_target = command.substring(0, commaPosition).toDouble();  //电机角度
        Serial.println(motor_target);
      }
      // reset the command buffer
      received_chars = "";
    }
  }
  return command;
}

float getSerialDate() {
  return motor_target;
}
