

#include "driver/mcpwm.h"
// #include "driver/mcpwm_gen.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "MT6701.h"

#define USD_MCPWM_TMC6300 1
#define PWM_DUTY_CYCLE 0


#define PIN_UH 6
#define PIN_UL 5
#define PIN_VH 1
#define PIN_VL 3
#define PIN_WH 0
#define PIN_WL 4

#define NOT_SET -12345.0
#define _isset(a) ((a) != (NOT_SET))
#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
// ABI bus frequency - would be better to take it from somewhere
// but I did nto find a good exposed variable
#define _MCPWM_FREQ 160e6f
// preferred pwm resolution default
#define _PWM_RES_DEF 2048
// min resolution
#define _PWM_RES_MIN 1500
// max resolution
#define _PWM_RES_MAX 3000
// pwm frequency
#define _PWM_FREQUENCY 25000      // default
#define _PWM_FREQUENCY_MAX 50000  // mqx

MT6701Sensor encoder = MT6701Sensor();
float dc_a;  //!< currently set duty cycle on phaseA
float dc_b;  //!< currently set duty cycle on phaseB
float dc_c;  //!< currently set duty cycle on phaseC

long pwm_frequency;          //!< pwm frequency value in hertz
float voltage_power_supply;  //!< power supply voltage
float voltage_limit;         //!< limiting voltage set to the motor
float dead_zone;             //!< a percentage of dead-time(zone) (both high and low side in low) for each pwm cycle [0,1]


void _configureTimerFrequency(long pwm_frequency, mcpwm_dev_t *mcpwm_num, mcpwm_unit_t mcpwm_unit, float dead_zone) {
  mcpwm_config_t pwm_config;
  pwm_config.counter_mode = MCPWM_UP_DOWN_COUNTER;     // Up-down counter (triangle wave)
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;            // Active HIGH
  pwm_config.frequency = 2 * pwm_frequency;            // set the desired freq - just a placeholder for now https://github.com/simplefoc/Arduino-FOC/issues/76
  mcpwm_init(mcpwm_unit, MCPWM_TIMER_0, &pwm_config);  // Configure PWM0A & PWM0B with above settings
  mcpwm_init(mcpwm_unit, MCPWM_TIMER_1, &pwm_config);  // Configure PWM1A & PWM1B with above settings
  mcpwm_init(mcpwm_unit, MCPWM_TIMER_2, &pwm_config);  // Configure PWM2A & PWM2B with above settings

  if (_isset(dead_zone)) {
    // dead zone is configured
    float dead_time = (float)(_MCPWM_FREQ / (pwm_frequency)) * dead_zone;
    mcpwm_deadtime_enable(mcpwm_unit, MCPWM_TIMER_0, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, dead_time / 2.0, dead_time / 2.0);
    mcpwm_deadtime_enable(mcpwm_unit, MCPWM_TIMER_1, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, dead_time / 2.0, dead_time / 2.0);
    mcpwm_deadtime_enable(mcpwm_unit, MCPWM_TIMER_2, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, dead_time / 2.0, dead_time / 2.0);
  }
  delay(100);

  mcpwm_stop(mcpwm_unit, MCPWM_TIMER_0);
  mcpwm_stop(mcpwm_unit, MCPWM_TIMER_1);
  mcpwm_stop(mcpwm_unit, MCPWM_TIMER_2);

  // manual configuration due to the lack of config flexibility in mcpwm_init()
  mcpwm_num->clk_cfg.clk_prescale = 0;
  // calculate prescaler and period
  // step 1: calculate the prescaler using the default pwm resolution
  // prescaler = bus_freq / (pwm_freq * default_pwm_res) - 1
  int prescaler = ceil((double)_MCPWM_FREQ / (double)_PWM_RES_DEF / 2.0f / (double)pwm_frequency) - 1;
  // constrain prescaler
  prescaler = _constrain(prescaler, 0, 128);
  // now calculate the real resolution timer period necessary (pwm resolution)
  // pwm_res = bus_freq / (pwm_freq * (prescaler + 1))
  int resolution_corrected = (double)_MCPWM_FREQ / 2.0f / (double)pwm_frequency / (double)(prescaler + 1);
  // if pwm resolution too low lower the prescaler
  if (resolution_corrected < _PWM_RES_MIN && prescaler > 0)
    resolution_corrected = (double)_MCPWM_FREQ / 2.0f / (double)pwm_frequency / (double)(--prescaler + 1);
  resolution_corrected = _constrain(resolution_corrected, _PWM_RES_MIN, _PWM_RES_MAX);

  // set prescaler
  mcpwm_num->timer[0].timer_cfg0.timer_prescale = prescaler;
  mcpwm_num->timer[1].timer_cfg0.timer_prescale = prescaler;
  mcpwm_num->timer[2].timer_cfg0.timer_prescale = prescaler;
  delay(1);
  // set period
  mcpwm_num->timer[0].timer_cfg0.timer_period = resolution_corrected;
  mcpwm_num->timer[1].timer_cfg0.timer_period = resolution_corrected;
  mcpwm_num->timer[2].timer_cfg0.timer_period = resolution_corrected;
  delay(1);
  mcpwm_num->timer[0].timer_cfg0.timer_period_upmethod = 0;
  mcpwm_num->timer[1].timer_cfg0.timer_period_upmethod = 0;
  mcpwm_num->timer[2].timer_cfg0.timer_period_upmethod = 0;
  delay(1);
  // delay(1);
  // restart the timers
  mcpwm_start(mcpwm_unit, MCPWM_TIMER_0);
  mcpwm_start(mcpwm_unit, MCPWM_TIMER_1);
  mcpwm_start(mcpwm_unit, MCPWM_TIMER_2);
  delay(1);

  mcpwm_sync_config_t sync_conf = {
    .sync_sig = MCPWM_SELECT_TIMER0_SYNC,
    .timer_val = 0,
    .count_direction = MCPWM_TIMER_DIRECTION_UP
  };
  mcpwm_sync_configure(mcpwm_unit, MCPWM_TIMER_0, &sync_conf);
  mcpwm_sync_configure(mcpwm_unit, MCPWM_TIMER_1, &sync_conf);
  mcpwm_sync_configure(mcpwm_unit, MCPWM_TIMER_2, &sync_conf);

  // Enable sync event for all timers to be the TEZ of timer0
  mcpwm_set_timer_sync_output(mcpwm_unit, MCPWM_TIMER_0, MCPWM_SWSYNC_SOURCE_TEZ);
}

void config6PWM() {
  pwm_frequency = 5000;
  voltage_power_supply = 12;
  voltage_limit = 12;
  dead_zone = 0.05f;

  _configureTimerFrequency(pwm_frequency, &MCPWM0, MCPWM_UNIT_0, dead_zone);
}

void motor_init() {
  // PWM pins
  pinMode(PIN_UH, OUTPUT);  // AH
  pinMode(PIN_UL, OUTPUT);  // AL
  pinMode(PIN_VH, OUTPUT);  // BH
  pinMode(PIN_VL, OUTPUT);  // BL
  pinMode(PIN_WH, OUTPUT);  // CH
  pinMode(PIN_WL, OUTPUT);  // CL

  #if(PWM_DUTY_CYCLE != 0)
    analogWriteFrequency(PIN_UH, 5000);
    analogWriteFrequency(PIN_VH, 5000);
    analogWriteFrequency(PIN_WH, 5000);
  #endif

  #if USD_MCPWM_TMC6300

  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PIN_UH);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, PIN_VH);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, PIN_WH);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, PIN_UL);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, PIN_VL);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, PIN_WL);
  config6PWM();

  #endif
}

void writeDutyCycle6PWM(float dc_a, float dc_b, float dc_c) {
  // se the PWM on the slot timers
  // transform duty cycle from [0,1] to [0,100.0]
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, dc_a * 100.0);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, dc_a * 100.0);

  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, dc_b * 100.0);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, dc_b * 100.0);

  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, dc_c * 100.0);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B, dc_c * 100.0);
}

void setPwm(float Ua, float Ub, float Uc) {
  // limit the voltage in driver
  Ua = _constrain(Ua, 0, voltage_limit);
  Ub = _constrain(Ub, 0, voltage_limit);
  Uc = _constrain(Uc, 0, voltage_limit);
  // calculate duty cycle
  // limited in [0,1]
  dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
  dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
  dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);
  // hardware specific writing
  // hardware specific function - depending on driver and mcu
  writeDutyCycle6PWM(dc_a, dc_b, dc_c);
}

void set_au_bd(void) { 
  #if USD_MCPWM_TMC6300
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A,  100.0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, 100.0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B, 0);

    mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A);

    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, MCPWM_DUTY_MODE_0);
  #else

    #if PWM_DUTY_CYCLE == 0
    digitalWrite(PIN_UH, HIGH);
    digitalWrite(PIN_UL, LOW);
    digitalWrite(PIN_VH, LOW);
    digitalWrite(PIN_VL, HIGH);
    digitalWrite(PIN_WH, LOW);
    digitalWrite(PIN_WL, LOW);
    #else
    analogWrite(PIN_UH, PWM_DUTY_CYCLE);
    digitalWrite(PIN_UL, LOW);
    analogWrite(PIN_VH, 0);
    digitalWrite(PIN_VL, HIGH);
    analogWrite(PIN_WH, 0);
    digitalWrite(PIN_WL, LOW);
    #endif

  #endif
}

void set_au_cd(void) {
  #if USD_MCPWM_TMC6300
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, dc_a * 100.0);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, 0);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, 0);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B, dc_c * 100.0);
  #else

    #if PWM_DUTY_CYCLE == 0
    digitalWrite(PIN_UH, HIGH);
    digitalWrite(PIN_UL, LOW);
    digitalWrite(PIN_VH, LOW);
    digitalWrite(PIN_VL, LOW);
    digitalWrite(PIN_WH, LOW);
    digitalWrite(PIN_WL, HIGH);
    #else
    analogWrite(PIN_UH, PWM_DUTY_CYCLE);
    digitalWrite(PIN_UL, LOW);
    analogWrite(PIN_VH, 0);
    digitalWrite(PIN_VL, LOW);
    analogWrite(PIN_WH, 0);
    digitalWrite(PIN_WL, HIGH);
    #endif

  #endif
}

void set_bu_cd(void) {
  #if USD_MCPWM_TMC6300
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, dc_b * 100.0);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, 0);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, 0);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B, dc_c * 100.0);
  #else

    #if PWM_DUTY_CYCLE == 0
    digitalWrite(PIN_UH, LOW);
    digitalWrite(PIN_UL, LOW);
    digitalWrite(PIN_VH, HIGH);
    digitalWrite(PIN_VL, LOW);
    digitalWrite(PIN_WH, LOW);
    digitalWrite(PIN_WL, HIGH);
    #else
    analogWrite(PIN_UH, 0);
    digitalWrite(PIN_UL, LOW);
    analogWrite(PIN_VH, PWM_DUTY_CYCLE);
    digitalWrite(PIN_VL, LOW);
    analogWrite(PIN_WH, 0);
    digitalWrite(PIN_WL, HIGH);
    #endif
    
  #endif
}

void set_bu_ad(void) {
  #if USD_MCPWM_TMC6300
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, dc_a * 100.0);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, dc_b * 100.0);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, 0);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, 0);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B, 0);
  #else

    #if PWM_DUTY_CYCLE == 0
    digitalWrite(PIN_UH, LOW);
    digitalWrite(PIN_UL, HIGH);
    digitalWrite(PIN_VH, HIGH);
    digitalWrite(PIN_VL, LOW);
    digitalWrite(PIN_WH, LOW);
    digitalWrite(PIN_WL, LOW);
    #else
    analogWrite(PIN_UH, 0);
    digitalWrite(PIN_UL, HIGH);
    analogWrite(PIN_VH, PWM_DUTY_CYCLE);
    digitalWrite(PIN_VL, LOW);
    analogWrite(PIN_WH, 0);
    digitalWrite(PIN_WL, LOW);
    #endif
    
  #endif
}

void set_cu_ad(void) {
  #if USD_MCPWM_TMC6300
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, dc_a * 100.0);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, 0);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, dc_c * 100.0);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B, 0);
  #else

    #if PWM_DUTY_CYCLE == 0
    digitalWrite(PIN_UH, LOW);
    digitalWrite(PIN_UL, HIGH);
    digitalWrite(PIN_VH, LOW);
    digitalWrite(PIN_VL, LOW);
    // digitalWrite(PIN_WH, HIGH);
    digitalWrite(PIN_WL, LOW);
    #else
    analogWrite(PIN_UH, 0);
    digitalWrite(PIN_UL, HIGH);
    analogWrite(PIN_VH, 0);
    digitalWrite(PIN_VL, LOW);
    analogWrite(PIN_WH, PWM_DUTY_CYCLE);
    digitalWrite(PIN_WL, LOW);
    #endif
    
  #endif
}

void set_cu_bd(void) {
  #if USD_MCPWM_TMC6300
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, dc_b * 100.0);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, dc_c * 100.0);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B, 0);
  #else
    #if PWM_DUTY_CYCLE == 0
    digitalWrite(PIN_UH, LOW);
    digitalWrite(PIN_UL, LOW);
    digitalWrite(PIN_VH, LOW);
    digitalWrite(PIN_VL, HIGH);
    digitalWrite(PIN_WH, HIGH);
    digitalWrite(PIN_WL, LOW);
    #else
    analogWrite(PIN_UH, LOW);
    digitalWrite(PIN_UL, LOW);
    analogWrite(PIN_VH, LOW);
    digitalWrite(PIN_VL, HIGH);
    analogWrite(PIN_WH, PWM_DUTY_CYCLE);
    digitalWrite(PIN_WL, LOW);
    #endif
    
  #endif
}


void setup() {
  Serial.begin(115200);

  delay(1000);
  Serial.println("3");
  delay(1000);
  Serial.println("2");
  delay(1000);
  Serial.println("1");
  delay(1000);

  encoder.init();

  motor_init();

  Serial.println("Motor ready!");
  Serial.println("Set target velocity [rad/s]");

  set_au_bd();
}

// int i = 0;
// unsigned long previousMillis = 0;

void loop() {

  // unsigned long currentMillis = millis();  //读取当前时间

  // if (currentMillis - previousMillis >= 3) {
  //   switch (i) {
  //     case 0: set_au_bd(); i = 1; break;
  //     case 1: set_au_cd(); i = 2; break;
  //     case 2: set_bu_cd(); i = 3; break;
  //     case 3: set_bu_ad(); i = 4; break;
  //     case 4: set_cu_ad(); i = 5; break;
  //     case 5: set_cu_bd(); i = 0; break;
  //   }

  //   Serial.print(i);
  //   Serial.print("  ");
  //   Serial.println(encoder.getSensorAngle());
  //   previousMillis = currentMillis;  //更新时间记录
  // }
}
