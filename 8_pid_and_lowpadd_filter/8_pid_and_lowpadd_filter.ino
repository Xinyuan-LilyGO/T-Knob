
#include "FOCMotor.h"

#define BTN_RELEASE_STATE 1  // HIGH is release state
const int boot_btn = 9;

int GetBntState(int pin) {
  static int buttonState = BTN_RELEASE_STATE;
  static int lastButtonState = BTN_RELEASE_STATE;
  static int cnt = 0;

  buttonState = digitalRead(pin);

  if (buttonState != lastButtonState) {
    if (buttonState != BTN_RELEASE_STATE) {
      lastButtonState = buttonState;
      cnt++;
    }
    delay(20);
    lastButtonState = buttonState;
  }
  return cnt;
}

void setup() {
  Serial.begin(115200);

  pinMode(boot_btn, INPUT);

  FOCMotorInit(5);  // power supply voltage [5V]
  FOCMotorAlignSensor();
}

int mode = 0;
int mode_prev = 0;
void loop() {

  mode = GetBntState(boot_btn) % 5;
  if (mode != mode_prev) {
    printf("mode=%d\n", mode);
    mode_prev = mode;
  }

  switch (mode) {
    case 0:
      PIDSetAngleParam(0.056, 0, 0, 0);
      SetForceAngleLoop(getSerialDate());
      break;
    case 1:
      PIDSetVelocityParam(0.1, 0.2, 0.001, 0);
      PIDSetAngleParam(0.25, 0, 0, 0);
      SetVelocityAngleLoop(getSerialDate());
      break;
    case 2:
      PIDSetVelocityParam(0.005, 0.1, 0, 0);
      SetVelocityLoop(getSerialDate());
      break;
    case 3:
      SetTorque(getSerialDate());
      break;
    case 4:
      FOCSetTorque(getSerialDate(), 0);
      break;
    default:
      break;
  }
  serialReceiveUserCommand();
}
