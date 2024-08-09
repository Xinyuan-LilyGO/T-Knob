#include "MTC6300.h"
#include "MT6701.h"

MTC6300MotorDriver drive = MTC6300MotorDriver();
MT6701Sensor encode = MT6701Sensor();

#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

float voltage_power_supply = 5;
float shaft_angle = 0, open_loop_timestamp = 0;
float zero_electric_angle = 0, Ualpha, Ubeta = 0, Ua = 0, Ub = 0, Uc = 0, dc_a = 0, dc_b = 0, dc_c = 0;

float velocity_value = 3;
int rotation_direction = 1;  // 1:anticlockwise; -1:clockwise

static void update_motor_speed_callback(void *arg) {
  static bool flag = false;
  if (flag) {
    velocity_value -= 0.1;
  } else {
    velocity_value += 0.1;
  }

  if (velocity_value > 50.0) {
    flag = true;
  }
  if (velocity_value < 0.2) {
    flag = false;

    if (rotation_direction == 1)
      rotation_direction = -1;
    else if (rotation_direction == -1)
      rotation_direction = 1;
  }
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

  esp_timer_handle_t periodic_timer = NULL;
  const esp_timer_create_args_t periodic_timer_args = {
    .callback = update_motor_speed_callback,
  };
  ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 20000));
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

//开环速度函数
float velocityOpenloop(float target_velocity) {
  // get current timestamp
  unsigned long now_us = micros();
  // calculate the sample time from last call
  float Ts = (now_us - open_loop_timestamp) * 1e-6f;
  // quick fix for strange cases (micros overflow + timestamp not defined)
  if (Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

  // calculate the necessary angle to achieve target velocity
  shaft_angle = _normalizeAngle(shaft_angle + target_velocity * Ts);

  // use voltage limit or current limit
  float Uq = voltage_power_supply / 3;  

  setPhaseVoltage(Uq, 0, _electricalAngle(shaft_angle, 7));  // set the maximal allowed voltage (voltage_limit) with the necessary angle

  open_loop_timestamp = now_us;  // save timestamp for next call

  return Uq;
}

void loop() {
  // put your main code here, to run repeatedly:
  velocityOpenloop(rotation_direction * velocity_value);
}
