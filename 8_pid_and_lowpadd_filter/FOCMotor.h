#pragma once

#include "MTC6300.h"
#include "MT6701.h"
#include "Arduino.h"
#include "PID.h"
#include "lowpass_filter.h"

#define _PI 3.14159265359f
#define _3PI_2 4.71238898038f  // 3PI/2
#define _2PI 6.28318530718f
#define _sign(a) (((a) < 0) ? -1 : ((a) > 0))
#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

// FOC motor
extern MTC6300Driver drive;
extern MT6701Sensor sensor;
void FOCMotorInit(float power_supply);
void FOCMotorAlignSensor();
int FOCGetSensorDirection();
float electricalAngle();
float _normalizeAngle(float angle);
void FOCSetTorque(float Uq, float angle_el);

// PID controller
extern PIDController velocity_pid;
extern PIDController angle_pid;
void PIDSetVelocityParam(float P,float I,float D,float ramp);
void PIDSetAngleParam(float P,float I,float D,float ramp);
float PIDGetVelocityOutput(float error);
float PIDGetAngleOutput(float error);

// Lowpass filter
extern LowPassFilter lowpass_filter;
float LFGetVelocity();

// Test Interface
void SetVelocityLoop(float targat);
void SetVelocityAngleLoop(float target);
void SetForceAngleLoop(float target);
void SetTorque(float target);

// Serial port
String serialReceiveUserCommand();
float getSerialDate();
