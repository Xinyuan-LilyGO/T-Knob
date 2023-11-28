
#include "driver/mcpwm_prelude.h"
#include "driver/mcpwm_types.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#define TAG ""
#else
#include "esp_log.h"
static const char *TAG = "BLDC";
#endif

#define BLDC_MCPWM_TIMER_RESOLUTION_HZ 10000000  // 10MHz, 1 tick = 0.1us
#define BLDC_MCPWM_PERIOD 500                    // 50us, 20KHz
#define BLDC_SPIN_DIRECTION_CCW false            // define the spin direction
#define BLDC_SPEED_UPDATE_PERIOD_US 200000       // 200ms
#define BLDC_INTERNAL_PRIORITY 2

#define PIN_DIAG 2
#define PIN_UH 6
#define PIN_UL 5
#define PIN_VH 1
#define PIN_VL 3
#define PIN_WH 0
#define PIN_WL 4

#define BLDC_MCPWM_OP_INDEX_U 0
#define BLDC_MCPWM_OP_INDEX_V 1
#define BLDC_MCPWM_OP_INDEX_W 2
#define BLDC_MCPWM_GEN_INDEX_HIGH 0
#define BLDC_MCPWM_GEN_INDEX_LOW 1

typedef void (*bldc_hall_phase_action_t)(mcpwm_gen_handle_t (*gens)[2]);

mcpwm_gen_handle_t generators[3][2] = {};

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

static bldc_hall_phase_action_t s_hall_actions[7];

static void update_motor_speed_callback(void *arg) {
  static int step = 20;
  static int cur_speed = 0;
  if ((cur_speed + step) > 300 || (cur_speed + step) < 0) {
    step *= -1;
  }
  cur_speed += step;

  mcpwm_cmpr_handle_t *cmprs = (mcpwm_cmpr_handle_t *)arg;
  for (int i = 0; i < 3; i++) {
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmprs[i], cur_speed));
  }
}

void motor_init(void) {
  mcpwm_timer_handle_t timer = NULL;
  mcpwm_timer_config_t timer_config;
  timer_config.group_id = 0;
  timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
  timer_config.resolution_hz = BLDC_MCPWM_TIMER_RESOLUTION_HZ;
  timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
  timer_config.period_ticks = BLDC_MCPWM_PERIOD;
  timer_config.intr_priority = BLDC_INTERNAL_PRIORITY;
  ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

  ESP_LOGI(TAG, "1. Create MCPWM operator");
  mcpwm_oper_handle_t operators[3];
  mcpwm_operator_config_t operator_config;
  operator_config.group_id = 0;
  operator_config.intr_priority = BLDC_INTERNAL_PRIORITY;
  for (int i = 0; i < 3; i++)
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &operators[i]));

  ESP_LOGI(TAG, "2. Connect operators to the same timer");
  for (int i = 0; i < 3; i++)
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operators[i], timer));

  ESP_LOGI(TAG, "3. Create comparators");
  mcpwm_cmpr_handle_t comparators[3];
  mcpwm_comparator_config_t compare_config;
  compare_config.flags.update_cmp_on_tez = true;
  compare_config.intr_priority = BLDC_INTERNAL_PRIORITY;
  for (int i = 0; i < 3; i++) {
    ESP_ERROR_CHECK(mcpwm_new_comparator(operators[i], &compare_config, &comparators[i]));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparators[i], 0));
  }

  ESP_LOGI(TAG, "4. Create over current fault detector");
  mcpwm_fault_handle_t over_cur_fault = NULL;
  mcpwm_gpio_fault_config_t gpio_fault_config;
  gpio_fault_config.gpio_num = PIN_DIAG;
  gpio_fault_config.group_id = 0;
  gpio_fault_config.flags.active_level = 1;  // high level means fault, refer to TMC6300 datasheet
  gpio_fault_config.flags.pull_down = true;  // internally pull down
  gpio_fault_config.intr_priority = BLDC_INTERNAL_PRIORITY;
  ESP_ERROR_CHECK(mcpwm_new_gpio_fault(&gpio_fault_config, &over_cur_fault));

  ESP_LOGI(TAG, "5. Set brake mode on the fault event");
  mcpwm_brake_config_t brake_config;
  brake_config.brake_mode = MCPWM_OPER_BRAKE_MODE_CBC;
  brake_config.fault = over_cur_fault;
  brake_config.flags.cbc_recover_on_tez = true;
  for (int i = 0; i < 3; i++)
    ESP_ERROR_CHECK(mcpwm_operator_set_brake_on_fault(operators[i], &brake_config));

  ESP_LOGI(TAG, "6. Create PWM generators");
  // mcpwm_gen_handle_t generators[3][2] = {};
  mcpwm_generator_config_t gen_config = {};
  const int gen_gpios[3][2] = {
    { PIN_UH, PIN_UL },
    { PIN_VH, PIN_VL },
    { PIN_WH, PIN_WL },
  };
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 2; j++) {
      gen_config.gen_gpio_num = gen_gpios[i][j];
      ESP_ERROR_CHECK(mcpwm_new_generator(operators[i], &gen_config, &generators[i][j]));
    }
  }

  ESP_LOGI(TAG, "7. Set generator actions");
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
  ESP_LOGI(TAG, "8. Setup deadtime");
  mcpwm_dead_time_config_t dt_config;
  dt_config.posedge_delay_ticks = 5;
  for (int i = 0; i < 3; i++)
    ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(generators[i][BLDC_MCPWM_GEN_INDEX_HIGH], generators[i][BLDC_MCPWM_GEN_INDEX_HIGH], &dt_config));

  dt_config.negedge_delay_ticks = 5,
  dt_config.flags.invert_output = true;
  // for (int i = 0; i < 3; i++)
  //   ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(generators[i][BLDC_MCPWM_GEN_INDEX_LOW], generators[i][BLDC_MCPWM_GEN_INDEX_LOW], &dt_config));


  ESP_LOGI(TAG, "9. Turn off all the gates");
  for (int i = 0; i < 3; i++) {
    ESP_ERROR_CHECK(mcpwm_generator_set_force_level(generators[i][BLDC_MCPWM_GEN_INDEX_HIGH], 0, true));
    // because gen_low is inverted by dead time module, so we need to set force level to 1
    ESP_ERROR_CHECK(mcpwm_generator_set_force_level(generators[i][BLDC_MCPWM_GEN_INDEX_LOW], 1, true));
  }

  // ESP_LOGI(TAG, "10. Create Hall sensor capture channels");
  // mcpwm_cap_timer_handle_t cap_timer = NULL;
  // mcpwm_capture_timer_config_t cap_timer_config;
  // cap_timer_config.group_id = 0;
  // cap_timer_config.clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT;
  // ESP_ERROR_CHECK(mcpwm_new_capture_timer(&cap_timer_config, &cap_timer));
  // mcpwm_cap_channel_handle_t cap_channels[3];
  // mcpwm_capture_channel_config_t cap_channel_config;
  // cap_channel_config.prescale = 1;
  // cap_channel_config.flags.pull_up = true;
  // cap_channel_config.flags.neg_edge = true;
  // cap_channel_config.flags.pos_edge = true;
  // const int cap_chan_gpios[3] = { HALL_CAP_U_GPIO, HALL_CAP_V_GPIO, HALL_CAP_W_GPIO };
  // for (int i = 0; i < 3; i++) {
  //   cap_channel_config.gpio_num = cap_chan_gpios[i];
  //   ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap_timer, &cap_channel_config, &cap_channels[i]));
  // }

  // ESP_LOGI(TAG, "11. Register event callback for capture channels");
  // TaskHandle_t task_to_notify = xTaskGetCurrentTaskHandle();
  // for (int i = 0; i < 3; i++) {
  //   mcpwm_capture_event_callbacks_t cbs;
  //   cbs.on_cap = bldc_hall_updated;
  //   ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(cap_channels[i], &cbs, task_to_notify));
  // }

  // ESP_LOGI(TAG, "12. Enable capture channels");
  // for (int i = 0; i < 3; i++)
  //   ESP_ERROR_CHECK(mcpwm_capture_channel_enable(cap_channels[i]));


  // ESP_LOGI(TAG, "13. Enable and start capture timer");
  // ESP_ERROR_CHECK(mcpwm_capture_timer_enable(cap_timer));
  // ESP_ERROR_CHECK(mcpwm_capture_timer_start(cap_timer));

  ESP_LOGI(TAG, "14. Start a timer to adjust motor speed periodically");
  esp_timer_handle_t periodic_timer = NULL;
  esp_timer_create_args_t periodic_timer_args;
  periodic_timer_args.callback = update_motor_speed_callback;
  periodic_timer_args.arg = comparators;
  ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, BLDC_SPEED_UPDATE_PERIOD_US));

  // ESP_LOGI(TAG, "15. Enable MOSFET gate");
  // gpio_set_level(BLDC_DRV_EN_GPIO, 1);

  ESP_LOGI(TAG, "16. Start the MCPWM timer");
  ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
  ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

  // uint32_t hall_sensor_value = 0;
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

  motor_init();

  // s_hall_actions[2] = bldc_set_phase_up_vm;
  // s_hall_actions[6] = bldc_set_phase_wp_vm;
  // s_hall_actions[4] = bldc_set_phase_wp_um;
  // s_hall_actions[5] = bldc_set_phase_vp_um;
  // s_hall_actions[1] = bldc_set_phase_vp_wm;
  // s_hall_actions[3] = bldc_set_phase_up_wm;
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("hello");
  // bldc_set_phase_up_wm(generators);
  // static int sec = 0;
  // sec++;
  // if(sec >= 1 && sec <= 6){
  //   s_hall_actions[sec](generators);
  // }

  // if(sec > 6){
  //   sec=0;
  // }

  delay(1000);
}
