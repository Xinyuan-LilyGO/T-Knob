#include "MTC6300.h"
#include "MT6701.h"

#define _PI 3.14159265359f
#define _3PI_2 4.71238898038f  // 3PI/2
#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

MTC6300MotorDriver drive = MTC6300MotorDriver();
MT6701Sensor encode = MT6701Sensor();

float voltage_limit = 5;
float voltage_power_supply = 5;
float shaft_angle = 0, open_loop_timestamp = 0;
float zero_electric_angle = 0, Ualpha, Ubeta = 0, Ua = 0, Ub = 0, Uc = 0, dc_a = 0, dc_b = 0, dc_c = 0;
int PP = 7, DIR = -1;

float _electricalAngle() {
  return _normalizeAngle((float)(DIR * PP) * encode.getSensorAngle() - zero_electric_angle);
}


float _normalizeAngle(float angle) {
  float a = fmod(angle, 2 * PI);
  return a >= 0 ? a : (a + 2 * PI);
}

// 设置PWM到控制器输出
void setPwm(float Ua, float Ub, float Uc) {
  // 限制上限
  Ua = _constrain(Ua, 0.0f, voltage_limit);
  Ub = _constrain(Ub, 0.0f, voltage_limit);
  Uc = _constrain(Uc, 0.0f, voltage_limit);
  // 计算占空比
  // 限制占空比从0到1
  dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
  dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
  dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);

  dc_a = (dc_a * 100 * 255) / 100;
  dc_b = (dc_b * 100 * 255) / 100;
  dc_c = (dc_c * 100 * 255) / 100;
  printf("%d,%d,%d\r\n", (int)dc_a, (int)dc_b, (int)dc_c);
  drive.setPwmValue((int)dc_a, (int)dc_b, (int)dc_c);
}


void setPhaseVoltage(float Uq, float Ud, float angle_el) {


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

  drive.init();
  encode.init();
  

  // align sensor
  setPhaseVoltage(3, 0, _3PI_2);
  delay(3000);
  zero_electric_angle = _electricalAngle();

  setPhaseVoltage(0, 0, _3PI_2);
  printf("1---%0.3f, %0.3f, %0.3f\n", encode.getAngle(), encode.getSensorAngle(), _electricalAngle());
  printf("1---%0.3f, %0.3f, %0.3f\n", encode.getAngle(), encode.getSensorAngle(), _electricalAngle());
  Serial.print("2.zero_electric_angle:");
  Serial.println(zero_electric_angle);
}

//==============串口接收==============
float motor_target=0;
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

void loop() {
  // put your main code here, to run repeatedly:
  // Serial.println(encode.getAngle());
  float Sensor_Angle = encode.getAngle();
  float Kp = 0.056;
  float Uq = _constrain(Kp * (motor_target - DIR * Sensor_Angle) * 180 / PI, -2.5, 2.5);
  setPhaseVoltage(motor_target, 0, _electricalAngle());
  serialReceiveUserCommand();

  // printf("%0.3f, %0.3f, %0.3f\n", Sensor_Angle, Uq, _electricalAngle());
  // delay(100);
}
