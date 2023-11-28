#include "MTC6300.h"
#include "MT6701.h"

MTC6300MotorDriver drive = MTC6300MotorDriver();
MT6701Sensor encode = MT6701Sensor();

#define _sign(a) ( ( (a) < 0 )  ?  -1   : ( (a) > 0 ) )
#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

float voltage_power_supply = 5;
float velocity_limit = 3;
float shaft_angle = 0, open_loop_timestamp = 0;
float zero_electric_angle = 0, Ualpha, Ubeta = 0, Ua = 0, Ub = 0, Uc = 0, dc_a = 0, dc_b = 0, dc_c = 0;

float angle_value = 3;
int rotation_direction = 1;  // 1:anticlockwise; -1:clockwise

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
}

// 电角度求解
float _electricalAngle(float shaft_angle, int pole_pairs) {
  return (shaft_angle * pole_pairs);
}

float _normalizeAngle(float angle) {
  float a = fmod(angle, 2 * PI);
  return a >= 0 ? a : (a + 2 * PI);
}

// 设置PWM到控制器输出
void setPwm(float Ua, float Ub, float Uc) {
  // 计算占空比
  // 限制占空比从0到1
  dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
  dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
  dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);

  dc_a = (dc_a * 100 * 255) / 100;
  dc_b = (dc_b * 100 * 255) / 100;
  dc_c = (dc_c * 100 * 255) / 100;
  printf("%d,%d,%d\n", (int)dc_a, (int)dc_b, (int)dc_c);
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

float angleOpenloop(float target_angle) {
  // get current timestamp
  unsigned long now_us = micros();
  // calculate the sample time from last call
  float Ts = (now_us - open_loop_timestamp) * 1e-6f;
  // quick fix for strange cases (micros overflow + timestamp not defined)
  if (Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

  if(abs( target_angle - shaft_angle ) > abs(velocity_limit*Ts)){
    shaft_angle += _sign(target_angle - shaft_angle) * abs( velocity_limit )*Ts;
    // shaft_velocity = velocity_limit;
  }else{
    shaft_angle = target_angle;
    // shaft_velocity = 0;
  }


  // use voltage limit or current limit
  float Uq = voltage_power_supply / 3;  

  setPhaseVoltage(Uq, 0, _electricalAngle(_normalizeAngle(shaft_angle), 7));  // set the maximal allowed voltage (voltage_limit) with the necessary angle

  open_loop_timestamp = now_us;  // save timestamp for next call

  return Uq;
}

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
  angleOpenloop(motor_target);
  serialReceiveUserCommand();
}
