#include "driver/mcpwm_prelude.h"
#include "driver/mcpwm_types.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#define TAG "BLDC"
#else
#include "esp_log.h"
static const char *TAG = "BLDC";
#endif

#define BLDC_MCPWM_TIMER_RESOLUTION_HZ 10000000  // 10MHz, 1 tick = 0.1us
#define BLDC_MCPWM_PERIOD 500                    // 50us, 20KHz
#define BLDC_SPIN_DIRECTION_CCW false            // define the spin direction
#define BLDC_SPEED_UPDATE_PERIOD_US 200000       // 200ms

#define BLDC_DRV_FAULT_GPIO 2
#define BLDC_PWM_UH_GPIO 6
#define BLDC_PWM_UL_GPIO 5
#define BLDC_PWM_VH_GPIO 1
#define BLDC_PWM_VL_GPIO 3
#define BLDC_PWM_WH_GPIO 0
#define BLDC_PWM_WL_GPIO 4
#define BLDC_MCPWM_OP_INDEX_U 0
#define BLDC_MCPWM_OP_INDEX_V 1
#define BLDC_MCPWM_OP_INDEX_W 2
#define BLDC_MCPWM_GEN_INDEX_HIGH 0
#define BLDC_MCPWM_GEN_INDEX_LOW 1

mcpwm_gen_handle_t generators[3][2] = {};
mcpwm_cmpr_handle_t comparators[3];

static void update_motor_speed_callback(void *arg) {
  static int step = 10;
  static int cur_speed = 20;
  if ((cur_speed + step) > 490 || (cur_speed + step) < 0) {
      step *= -1;
  }
  cur_speed += step;

  Serial.print("speed:");
  Serial.println(cur_speed);

  mcpwm_cmpr_handle_t *cmprs = (mcpwm_cmpr_handle_t *)arg;
  for (int i = 0; i < 3; i++) {
      ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmprs[i], cur_speed));
  }
}

void motor_init(void) {
  Serial.println("1.Create MCPWM timer");
  mcpwm_timer_handle_t timer = NULL;
  mcpwm_timer_config_t timer_config = {
    .group_id = 0,
    .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
    .resolution_hz = BLDC_MCPWM_TIMER_RESOLUTION_HZ,
    .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    .period_ticks = BLDC_MCPWM_PERIOD,
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
    { BLDC_PWM_UH_GPIO, BLDC_PWM_UL_GPIO },
    { BLDC_PWM_VH_GPIO, BLDC_PWM_VL_GPIO },
    { BLDC_PWM_WH_GPIO, BLDC_PWM_WL_GPIO },
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

// U+V-
static void bldc_set_phase_up_vm(mcpwm_gen_handle_t (*gens)[2]) {
  // U+ = PWM, U- = _PWM_
  mcpwm_generator_set_force_level(gens[BLDC_MCPWM_OP_INDEX_U][BLDC_MCPWM_GEN_INDEX_HIGH], -1, true);
  mcpwm_generator_set_force_level(gens[BLDC_MCPWM_OP_INDEX_U][BLDC_MCPWM_GEN_INDEX_LOW], -1, true);

  // V+ = 0, V- = 1  --[because gen_low is inverted by dead time]--> V+ = 0, V- = 0
  mcpwm_generator_set_force_level(gens[BLDC_MCPWM_OP_INDEX_V][BLDC_MCPWM_GEN_INDEX_HIGH], 0, true);
  mcpwm_generator_set_force_level(gens[BLDC_MCPWM_OP_INDEX_V][BLDC_MCPWM_GEN_INDEX_LOW], 0, true);

  // W+ = 0, W- = 0  --[because gen_low is inverted by dead time]--> W+ = 0, W- = 1
  mcpwm_generator_set_force_level(gens[BLDC_MCPWM_OP_INDEX_W][BLDC_MCPWM_GEN_INDEX_HIGH], 0, true);
  mcpwm_generator_set_force_level(gens[BLDC_MCPWM_OP_INDEX_W][BLDC_MCPWM_GEN_INDEX_LOW], 1, true);
}

// W+U-
static void bldc_set_phase_wp_um(mcpwm_gen_handle_t (*gens)[2]) {
  // U+ = 0, U- = 1  --[because gen_low is inverted by dead time]--> U+ = 0, U- = 0
  mcpwm_generator_set_force_level(gens[BLDC_MCPWM_OP_INDEX_U][BLDC_MCPWM_GEN_INDEX_HIGH], 0, true);
  mcpwm_generator_set_force_level(gens[BLDC_MCPWM_OP_INDEX_U][BLDC_MCPWM_GEN_INDEX_LOW], 0, true);

  // V+ = 0, V- = 0  --[because gen_low is inverted by dead time]--> V+ = 0, V- = 1
  mcpwm_generator_set_force_level(gens[BLDC_MCPWM_OP_INDEX_V][BLDC_MCPWM_GEN_INDEX_HIGH], 0, true);
  mcpwm_generator_set_force_level(gens[BLDC_MCPWM_OP_INDEX_V][BLDC_MCPWM_GEN_INDEX_LOW], 1, true);

  // W+ = PWM, W- = _PWM_
  mcpwm_generator_set_force_level(gens[BLDC_MCPWM_OP_INDEX_W][BLDC_MCPWM_GEN_INDEX_HIGH], -1, true);
  mcpwm_generator_set_force_level(gens[BLDC_MCPWM_OP_INDEX_W][BLDC_MCPWM_GEN_INDEX_LOW], -1, true);
}

// W+V-
static void bldc_set_phase_wp_vm(mcpwm_gen_handle_t (*gens)[2]) {
  // U+ = 0, U- = 0  --[because gen_low is inverted by dead time]--> U+ = 0, U- = 1
  mcpwm_generator_set_force_level(gens[BLDC_MCPWM_OP_INDEX_U][BLDC_MCPWM_GEN_INDEX_HIGH], 0, true);
  mcpwm_generator_set_force_level(gens[BLDC_MCPWM_OP_INDEX_U][BLDC_MCPWM_GEN_INDEX_LOW], 1, true);

  // V+ = 0, V- = 1  --[because gen_low is inverted by dead time]--> V+ = 0, V- = 0
  mcpwm_generator_set_force_level(gens[BLDC_MCPWM_OP_INDEX_V][BLDC_MCPWM_GEN_INDEX_HIGH], 0, true);
  mcpwm_generator_set_force_level(gens[BLDC_MCPWM_OP_INDEX_V][BLDC_MCPWM_GEN_INDEX_LOW], 0, true);

  // W+ = PWM, W- = _PWM_
  mcpwm_generator_set_force_level(gens[BLDC_MCPWM_OP_INDEX_W][BLDC_MCPWM_GEN_INDEX_HIGH], -1, true);
  mcpwm_generator_set_force_level(gens[BLDC_MCPWM_OP_INDEX_W][BLDC_MCPWM_GEN_INDEX_LOW], -1, true);
}

// V+U-
static void bldc_set_phase_vp_um(mcpwm_gen_handle_t (*gens)[2]) {
  // U+ = 0, U- = 1  --[because gen_low is inverted by dead time]--> U+ = 0, U- = 0
  mcpwm_generator_set_force_level(gens[BLDC_MCPWM_OP_INDEX_U][BLDC_MCPWM_GEN_INDEX_HIGH], 0, true);
  mcpwm_generator_set_force_level(gens[BLDC_MCPWM_OP_INDEX_U][BLDC_MCPWM_GEN_INDEX_LOW], 0, true);

  // V+ = PWM, V- = _PWM_
  mcpwm_generator_set_force_level(gens[BLDC_MCPWM_OP_INDEX_V][BLDC_MCPWM_GEN_INDEX_HIGH], -1, true);
  mcpwm_generator_set_force_level(gens[BLDC_MCPWM_OP_INDEX_V][BLDC_MCPWM_GEN_INDEX_LOW], -1, true);

  // W+ = 0, W- = 0  --[because gen_low is inverted by dead time]--> W+ = 0, W- = 1
  mcpwm_generator_set_force_level(gens[BLDC_MCPWM_OP_INDEX_W][BLDC_MCPWM_GEN_INDEX_HIGH], 0, true);
  mcpwm_generator_set_force_level(gens[BLDC_MCPWM_OP_INDEX_W][BLDC_MCPWM_GEN_INDEX_LOW], 1, true);
}

// V+W-
static void bldc_set_phase_vp_wm(mcpwm_gen_handle_t (*gens)[2]) {
  // U+ = 0, U- = 0  --[because gen_low is inverted by dead time]--> U+ = 0, U- = 1
  mcpwm_generator_set_force_level(gens[BLDC_MCPWM_OP_INDEX_U][BLDC_MCPWM_GEN_INDEX_HIGH], 0, true);
  mcpwm_generator_set_force_level(gens[BLDC_MCPWM_OP_INDEX_U][BLDC_MCPWM_GEN_INDEX_LOW], 1, true);

  // V+ = PWM, V- = _PWM_
  mcpwm_generator_set_force_level(gens[BLDC_MCPWM_OP_INDEX_V][BLDC_MCPWM_GEN_INDEX_HIGH], -1, true);
  mcpwm_generator_set_force_level(gens[BLDC_MCPWM_OP_INDEX_V][BLDC_MCPWM_GEN_INDEX_LOW], -1, true);

  // W+ = 0, W- = 1  --[because gen_low is inverted by dead time]--> W+ = 0, W- = 0
  mcpwm_generator_set_force_level(gens[BLDC_MCPWM_OP_INDEX_W][BLDC_MCPWM_GEN_INDEX_HIGH], 0, true);
  mcpwm_generator_set_force_level(gens[BLDC_MCPWM_OP_INDEX_W][BLDC_MCPWM_GEN_INDEX_LOW], 0, true);
}

// U+W- / A+C-
static void bldc_set_phase_up_wm(mcpwm_gen_handle_t (*gens)[2]) {
  // U+ = PWM, U- = _PWM_
  mcpwm_generator_set_force_level(gens[BLDC_MCPWM_OP_INDEX_U][BLDC_MCPWM_GEN_INDEX_HIGH], -1, true);
  mcpwm_generator_set_force_level(gens[BLDC_MCPWM_OP_INDEX_U][BLDC_MCPWM_GEN_INDEX_LOW], -1, true);

  // V+ = 0, V- = 0  --[because gen_low is inverted by dead time]--> V+ = 0, V- = 1
  mcpwm_generator_set_force_level(gens[BLDC_MCPWM_OP_INDEX_V][BLDC_MCPWM_GEN_INDEX_HIGH], 0, true);
  mcpwm_generator_set_force_level(gens[BLDC_MCPWM_OP_INDEX_V][BLDC_MCPWM_GEN_INDEX_LOW], 1, true);

  // W+ = 0, W- = 1  --[because gen_low is inverted by dead time]--> W+ = 0, W- = 0
  mcpwm_generator_set_force_level(gens[BLDC_MCPWM_OP_INDEX_W][BLDC_MCPWM_GEN_INDEX_HIGH], 0, true);
  mcpwm_generator_set_force_level(gens[BLDC_MCPWM_OP_INDEX_W][BLDC_MCPWM_GEN_INDEX_LOW], 0, true);
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

  motor_init();

  ESP_LOGI(TAG, "Start a timer to adjust motor speed periodically");
  esp_timer_handle_t periodic_timer = NULL;
  const esp_timer_create_args_t periodic_timer_args = {
    .callback = update_motor_speed_callback,
    .arg = comparators,
  };
  ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, BLDC_SPEED_UPDATE_PERIOD_US));
}

int i = 0;
unsigned long previousMillis = 0;
void loop() {
  // put your main code here, to run repeatedly:

  unsigned long currentMillis = millis();  //读取当前时间

  if (currentMillis - previousMillis >= 4) {
    switch (i) {
      case 0: bldc_set_phase_up_vm(generators); i = 1; break;
      case 1: bldc_set_phase_up_wm(generators); i = 2; break;
      case 2: bldc_set_phase_vp_wm(generators); i = 3; break;
      case 3: bldc_set_phase_vp_um(generators); i = 4; break;
      case 4: bldc_set_phase_wp_um(generators); i = 5; break;
      case 5: bldc_set_phase_wp_vm(generators); i = 0; break;
    }
    // Serial.println(i);
    previousMillis = currentMillis;  //更新时间记录
  }
}
