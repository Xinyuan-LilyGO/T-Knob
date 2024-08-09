
#pragma once

#include "driver/mcpwm_prelude.h"
#include "driver/mcpwm_types.h"

#define BLDC_DRV_FAULT_GPIO 2
#define UH_PIN 6
#define UL_PIN 5
#define VH_PIN 1
#define VL_PIN 3
#define WH_PIN 0
#define WL_PIN 4

class MTC6300MotorDriver {
public:
  MTC6300MotorDriver(int Uh = UH_PIN, int Ul=UL_PIN, int Vh=VH_PIN, int Vl=VL_PIN, int Wh=WH_PIN, int Wl=WL_PIN, int fault_pin=BLDC_DRV_FAULT_GPIO);

  void init();

  uint32_t pwm_value_map(int32_t x, int32_t min_in, int32_t max_in, int32_t min_out, int32_t max_out);

  void setPwmValue(int U, int V, int W); // range: [0, 100]

  void bldc_set_phase_up_vm();
  void bldc_set_phase_wp_um();
  void bldc_set_phase_wp_vm();
  void bldc_set_phase_vp_um();
  void bldc_set_phase_vp_wm();
  void bldc_set_phase_up_wm();

  int pwmU_h, pwmU_l;  //!< phase U pwm pin number
  int pwmV_h, pwmV_l;  //!< phase V pwm pin number
  int pwmW_h, pwmW_l;  //!< phase W pwm pin number
  int drv_fault_pin;

private:
  uint32_t mcpwm_timer_resolution_hz = 10000000;  // 10MHz, 1 tick = 0.1us
  uint16_t mcpwm_period = 500;                    // 50us, 20KHz
  bool spin_direction_ccw = false;                // define the spin direction

  enum{
    BLDC_MCPWM_OP_INDEX_U = 0,
    BLDC_MCPWM_OP_INDEX_V = 1,
    BLDC_MCPWM_OP_INDEX_W = 2,
  };
  enum{
    BLDC_MCPWM_GEN_INDEX_HIGH = 0,
    BLDC_MCPWM_GEN_INDEX_LOW = 1,
  };

  mcpwm_gen_handle_t generators[3][2] = { 0 };
  mcpwm_cmpr_handle_t comparators[3] = { 0 };
};
