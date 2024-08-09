#include "MTC6300.h"
#include "MT6701.h"

MTC6300MotorDriver drive = MTC6300MotorDriver();
MT6701Sensor encode = MT6701Sensor();

static void update_motor_speed_callback(void *arg)
{
  drive.setPwmValue(100, 100, 100);
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
  ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 200000));
}

int i = 0;
unsigned long previousMillis = 0;
float prev_angle = 0.0f;
float curr_angle = 0.0f;
void loop() {
  unsigned long currentMillis = millis();  //读取当前时间

  if (currentMillis - previousMillis >= 4) {
    switch (i) {
      case 0: drive.bldc_set_phase_up_vm(); i = 1; break;
      case 1: drive.bldc_set_phase_up_wm(); i = 2; break;
      case 2: drive.bldc_set_phase_vp_wm(); i = 3; break;
      case 3: drive.bldc_set_phase_vp_um(); i = 4; break;
      case 4: drive.bldc_set_phase_wp_um(); i = 5; break;
      case 5: drive.bldc_set_phase_wp_vm(); i = 0; break;
    }

    curr_angle = encode.getSensorAngle();
    if (abs(curr_angle - prev_angle) > 0.05) {
      Serial.println(curr_angle);
      prev_angle = curr_angle;
    }
    // Serial.println(i);
    previousMillis = currentMillis;  //更新时间记录
  }
}

