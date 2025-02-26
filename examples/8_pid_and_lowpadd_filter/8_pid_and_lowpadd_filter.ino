
#include "FOCMotor.h"

#define BTN_RELEASE_STATE 1  // HIGH is release state
const int boot_btn = 9;
const int led1 = 23;
const int led2 = 22;
const int led3 = 21;
const int led4 = 20;
const int buzz = 18;

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
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  pinMode(led4, OUTPUT);
  pinMode(buzz, OUTPUT);

  digitalWrite(led1, LOW);
  digitalWrite(led2, LOW);
  digitalWrite(led3, LOW);
  digitalWrite(led4, LOW);
  analogWrite(buzz, 100);

  FOCMotorInit(5);  // power supply voltage [5V]
  FOCMotorAlignSensor();
}

int mode = 0;
int mode_prev = 0;
void loop() {

  mode = GetBntState(boot_btn) % 6;
  if (mode != mode_prev) {
    switch (mode) {
      case 0: printf("mode=%d, Velocity Angle Ctrl\n", mode); break;
      case 1: printf("mode=%d, Angle Ctrl\n", mode); break;
      case 2: printf("mode=%d, Velocity Ctrl\n", mode); break;
      case 3: printf("mode=%d, Velocity Torque Ctrl\n", mode); break;
      case 4: printf("mode=%d, Torque Ctrl\n", mode); break;
      case 5: printf("mode=%d, Led & Buzzer ON\n", mode); break;
      default: break;
    }
    mode_prev = mode;
  }

  switch (mode) {
    case 0:{
      digitalWrite(led1, HIGH);
      digitalWrite(led2, HIGH);
      digitalWrite(led3, HIGH);
      digitalWrite(led4, HIGH);
      analogWrite(buzz, 0);
      PIDSetVelocityParam(0.1, 0.2, 0.001, 0);
      PIDSetAngleParam(0.25, 0, 0, 0);
      SetVelocityAngleLoop(0);
    }
      break;
    case 1:
      PIDSetAngleParam(0.056, 0, 0, 0);
      SetForceAngleLoop(0);
      break;
    case 2:
      PIDSetVelocityParam(0.002, 0.1, 0, 0);
      SetVelocityLoop(2.5);
      break;
    case 3:
      SetTorque(2);
      break;
    case 4:
      FOCSetTorque(3, 0);
      break;
    case 5:
      digitalWrite(led1, LOW);
      digitalWrite(led2, LOW);
      digitalWrite(led3, LOW);
      digitalWrite(led4, LOW);
      analogWrite(buzz, 100);
      break;
    default:
      break;
  }
  serialReceiveUserCommand();
}
