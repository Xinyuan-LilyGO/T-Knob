#define DIRECTION_OF_ROTATION 1  // 1：顺时针；0：逆时针
#define PIN_UH 6
#define PIN_UL 5
#define PIN_VH 1
#define PIN_VL 3
#define PIN_WH 0
#define PIN_WL 4

#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

bool direction_of_rotation = 0;  // 1：顺时针；0：逆时针

float voltage_power_supply = 12.6;
float shaft_angle = 0, open_loop_timestamp = 0;
float zero_electric_angle = 0, Ualpha, Ubeta = 0, Ua = 0, Ub = 0, Uc = 0, dc_a = 0, dc_b = 0, dc_c = 0;


void motor_init() {
  pinMode(PIN_UH, OUTPUT);  // AH
  pinMode(PIN_UL, OUTPUT);  // AL
  pinMode(PIN_VH, OUTPUT);  // BH
  pinMode(PIN_VL, OUTPUT);  // BL
  pinMode(PIN_WH, OUTPUT);  // CH
  pinMode(PIN_WL, OUTPUT);  // CL
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

  motor_init();
}

void set_au_bd(int u, int v, int w) {
  analogWrite(PIN_UH, u);
  digitalWrite(PIN_UL, LOW);
  analogWrite(PIN_VH, 0);
  digitalWrite(PIN_VL, HIGH);
  analogWrite(PIN_WH, 0);
  digitalWrite(PIN_WL, LOW);
}

void set_au_cd(int u, int v, int w) {
  analogWrite(PIN_UH, u);
  digitalWrite(PIN_UL, LOW);
  analogWrite(PIN_VH, 0);
  digitalWrite(PIN_VL, LOW);
  analogWrite(PIN_WH, 0);
  digitalWrite(PIN_WL, HIGH);
}

void set_bu_cd(int u, int v, int w) {
  analogWrite(PIN_UH, 0);
  digitalWrite(PIN_UL, LOW);
  analogWrite(PIN_VH, v);
  digitalWrite(PIN_VL, LOW);
  analogWrite(PIN_WH, 0);
  digitalWrite(PIN_WL, HIGH);
}

void set_bu_ad(int u, int v, int w) {
  analogWrite(PIN_UH, 0);
  digitalWrite(PIN_UL, HIGH);
  analogWrite(PIN_VH, v);
  digitalWrite(PIN_VL, LOW);
  analogWrite(PIN_WH, 0);
  digitalWrite(PIN_WL, LOW);
}

void set_cu_ad(int u, int v, int w) {
  analogWrite(PIN_UH, 0);
  digitalWrite(PIN_UL, HIGH);
  analogWrite(PIN_VH, 0);
  digitalWrite(PIN_VL, LOW);
  analogWrite(PIN_WH, w);
  digitalWrite(PIN_WL, LOW);
}

void set_cu_bd(int u, int v, int w) {
  analogWrite(PIN_UH, 0);
  digitalWrite(PIN_UL, LOW);
  analogWrite(PIN_VH, 0);
  digitalWrite(PIN_VL, HIGH);
  analogWrite(PIN_WH, w);
  digitalWrite(PIN_WL, LOW);
}

void motor_run(int pwma, int pwmb, int pwmc) {
  static int phase_idx = 0;
  void (*motor_drive[6])(int, int, int) = { set_au_bd, set_au_cd, set_bu_cd, set_bu_ad, set_cu_ad, set_cu_bd };
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();  //读取当前时间

  if (currentMillis - previousMillis >= 5) {
    if (direction_of_rotation == 1) {
      motor_drive[phase_idx++](pwma, pwmb, pwmc);
      if (phase_idx > 5)
        phase_idx = 0;
    } else {
      motor_drive[phase_idx--](pwma, pwmb, pwmc);
      if (phase_idx < 0)
        phase_idx = 5;
    }
    previousMillis = currentMillis;
  }
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
  printf("%d, %d, %d\n", (int)dc_a, (int)dc_b, (int)dc_c);
  motor_run((int)dc_a, (int)dc_b, (int)dc_c);
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
  unsigned long now_us = micros();
  float Ts = (now_us - open_loop_timestamp) * 1e-6f;

  if (Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

  shaft_angle = _normalizeAngle(shaft_angle + target_velocity * Ts);
  float Uq = voltage_power_supply / 3;

  setPhaseVoltage(Uq, 0, _electricalAngle(shaft_angle, 8));

  open_loop_timestamp = now_us;  //用于计算下一个时间间隔

  return Uq;
}


void loop() {
  // put your main code here, to run repeatedly:
  // motor_run(255, 255, 255);
  velocityOpenloop(10);
}
