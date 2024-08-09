
#include "MTC6300.h"
#include "Arduino.h"

#define BLDC_MCPWM_OP_INDEX_U 0
#define BLDC_MCPWM_OP_INDEX_V 1
#define BLDC_MCPWM_OP_INDEX_W 2
#define BLDC_MCPWM_GEN_INDEX_HIGH 0
#define BLDC_MCPWM_GEN_INDEX_LOW 1

MTC6300MotorDriver::MTC6300MotorDriver(int Uh, int Ul, int Vh, int Vl, int Wh, int Wl, int fault_pin)
  : pwmU_h(Uh), pwmU_l(Ul), pwmV_h(Vh), pwmV_l(Vl), pwmW_h(Wh), pwmW_l(Wl), drv_fault_pin(fault_pin) {}

void MTC6300MotorDriver::init() {
  Serial.println("1.Create MCPWM timer");
  mcpwm_timer_handle_t timer = NULL;
  mcpwm_timer_config_t timer_config = {
    .group_id = 0,
    .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
    .resolution_hz = mcpwm_timer_resolution_hz,
    .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    .period_ticks = mcpwm_period,
  };
  ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

  Serial.println("2.Create MCPWM operator");
  mcpwm_oper_handle_t operators[3];
  mcpwm_operator_config_t operator_config = {
    .group_id = 0,
  };
  for (int i = 0; i < 3; i++) {
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &operators[i]));
  }

  Serial.println("3.Connect operators to the same timer");
  for (int i = 0; i < 3; i++) {
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operators[i], timer));
  }

  Serial.println("4.Create comparators");

  mcpwm_comparator_config_t compare_config = { 0 };
  compare_config.flags.update_cmp_on_tez = true;
  for (int i = 0; i < 3; i++) {
    ESP_ERROR_CHECK(mcpwm_new_comparator(operators[i], &compare_config, &comparators[i]));
    // set compare value to 0, we will adjust the speed in a period timer callback
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparators[i], 0));
  }

  Serial.println("5.Create over current fault detector");
  mcpwm_fault_handle_t over_cur_fault = NULL;
  mcpwm_gpio_fault_config_t gpio_fault_config = { 0 };
  gpio_fault_config.gpio_num = BLDC_DRV_FAULT_GPIO;
  gpio_fault_config.group_id = 0;
  gpio_fault_config.flags.active_level = 1;  // low level means fault, refer to DRV8302 datasheet
  gpio_fault_config.flags.pull_up = true;    // internally pull up
  ESP_ERROR_CHECK(mcpwm_new_gpio_fault(&gpio_fault_config, &over_cur_fault));

  Serial.println("6.Set brake mode on the fault event");
  mcpwm_brake_config_t brake_config = { 0 };
  brake_config.brake_mode = MCPWM_OPER_BRAKE_MODE_CBC;
  brake_config.fault = over_cur_fault;
  brake_config.flags.cbc_recover_on_tez = true;
  for (int i = 0; i < 3; i++) {
    ESP_ERROR_CHECK(mcpwm_operator_set_brake_on_fault(operators[i], &brake_config));
  }

  Serial.println("7.Create PWM generators");

  mcpwm_generator_config_t gen_config = {};
  const int gen_gpios[3][2] = {
    { pwmU_h, pwmU_l },
    { pwmV_h, pwmV_l },
    { pwmW_h, pwmW_l },
  };
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 2; j++) {
      gen_config.gen_gpio_num = gen_gpios[i][j];
      ESP_ERROR_CHECK(mcpwm_new_generator(operators[i], &gen_config, &generators[i][j]));
    }
  }

  Serial.println("8.Set generator actions");
  // gen_high and gen_low output the same waveform after the following configuration
  // we will use the dead time module to add edge delay, also make gen_high and gen_low complementary
  for (int i = 0; i < 3; i++) {
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generators[i][BLDC_MCPWM_GEN_INDEX_HIGH],
                                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generators[i][BLDC_MCPWM_GEN_INDEX_HIGH],
                                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparators[i], MCPWM_GEN_ACTION_LOW)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_brake_event(generators[i][BLDC_MCPWM_GEN_INDEX_HIGH],
                                                              MCPWM_GEN_BRAKE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_OPER_BRAKE_MODE_CBC, MCPWM_GEN_ACTION_LOW)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_brake_event(generators[i][BLDC_MCPWM_GEN_INDEX_HIGH],
                                                              MCPWM_GEN_BRAKE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_OPER_BRAKE_MODE_CBC, MCPWM_GEN_ACTION_LOW)));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generators[i][BLDC_MCPWM_GEN_INDEX_LOW],
                                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generators[i][BLDC_MCPWM_GEN_INDEX_LOW],
                                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparators[i], MCPWM_GEN_ACTION_LOW)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_brake_event(generators[i][BLDC_MCPWM_GEN_INDEX_LOW],
                                                              MCPWM_GEN_BRAKE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_OPER_BRAKE_MODE_CBC, MCPWM_GEN_ACTION_LOW)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_brake_event(generators[i][BLDC_MCPWM_GEN_INDEX_LOW],
                                                              MCPWM_GEN_BRAKE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_OPER_BRAKE_MODE_CBC, MCPWM_GEN_ACTION_LOW)));
  }

  Serial.println("9.Setup deadtime");
  mcpwm_dead_time_config_t dt_config = {
    .posedge_delay_ticks = 5,
  };
  for (int i = 0; i < 3; i++) {
    ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(generators[i][BLDC_MCPWM_GEN_INDEX_HIGH], generators[i][BLDC_MCPWM_GEN_INDEX_HIGH], &dt_config));
  }
  mcpwm_dead_time_config_t dt_config1 = { 0 };
  dt_config1.negedge_delay_ticks = 5;
  dt_config1.flags.invert_output = true;
  for (int i = 0; i < 3; i++) {
    ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(generators[i][BLDC_MCPWM_GEN_INDEX_LOW], generators[i][BLDC_MCPWM_GEN_INDEX_LOW], &dt_config1));
  }

  Serial.println("10.Turn off all the gates");
  for (int i = 0; i < 3; i++) {
    ESP_ERROR_CHECK(mcpwm_generator_set_force_level(generators[i][BLDC_MCPWM_GEN_INDEX_HIGH], 0, true));
    // because gen_low is inverted by dead time module, so we need to set force level to 1
    ESP_ERROR_CHECK(mcpwm_generator_set_force_level(generators[i][BLDC_MCPWM_GEN_INDEX_LOW], 1, true));
  }

  Serial.println("Start the MCPWM timer");
  ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
  ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
}

uint32_t MTC6300MotorDriver::pwm_value_map(int32_t x, int32_t min_in, int32_t max_in, int32_t min_out, int32_t max_out)
{
  if(max_in >= min_in && x >= max_in) return max_out;
    if(max_in >= min_in && x <= min_in) return min_out;

    if(max_in <= min_in && x <= max_in) return max_out;
    if(max_in <= min_in && x >= min_in) return min_out;

    int32_t delta_in = max_in - min_in;
    int32_t delta_out = max_out - min_out;

    return ((x - min_in) * delta_out) / delta_in + min_out;
}

void MTC6300MotorDriver::setPwmValue(int U, int V, int W)
{
  uint32_t u_pwm = pwm_value_map(U, 0, 255, 0, mcpwm_period);
  uint32_t w_pwm = pwm_value_map(V, 0, 255, 0, mcpwm_period);
  uint32_t v_pwm = pwm_value_map(W, 0, 255, 0, mcpwm_period);

  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparators[BLDC_MCPWM_OP_INDEX_U], u_pwm));
  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparators[BLDC_MCPWM_OP_INDEX_V], w_pwm));
  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparators[BLDC_MCPWM_OP_INDEX_W], v_pwm));

  mcpwm_generator_set_force_level(generators[BLDC_MCPWM_OP_INDEX_U][BLDC_MCPWM_GEN_INDEX_HIGH], -1, true);
  mcpwm_generator_set_force_level(generators[BLDC_MCPWM_OP_INDEX_U][BLDC_MCPWM_GEN_INDEX_LOW], -1, true);
  mcpwm_generator_set_force_level(generators[BLDC_MCPWM_OP_INDEX_V][BLDC_MCPWM_GEN_INDEX_HIGH], -1, true);
  mcpwm_generator_set_force_level(generators[BLDC_MCPWM_OP_INDEX_V][BLDC_MCPWM_GEN_INDEX_LOW], -1, true);
  mcpwm_generator_set_force_level(generators[BLDC_MCPWM_OP_INDEX_W][BLDC_MCPWM_GEN_INDEX_HIGH], -1, true);
  mcpwm_generator_set_force_level(generators[BLDC_MCPWM_OP_INDEX_W][BLDC_MCPWM_GEN_INDEX_LOW], -1, true);
}
