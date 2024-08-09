

#include "PID.h"
#include <Arduino.h>
#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

PIDController::PIDController(float P, float I, float D, float ramp, float limit)
  : P(P),
    I(I),
    D(D),
    output_ramp(ramp),
    limit(limit),
    error_prev(0.0f),
    output_prev(0.0f),
    integral_prev(0.0f) {
  timestamp_prev = micros();
}

// 函数调用运算符 () 可以被重载用于类的对象。当重载 () 时，您不是创造了一种新的调用函数的方式，相反地，这是创建一个可以传递任意数目参数的运算符函数。
float PIDController::operator()(float error) {
  // get current timestamp
  unsigned long timestamp_now = micros();
  // calculate the sample time from last call
  float Ts = (timestamp_now - timestamp_prev) * 1e-6f;
  // quick fix for strange cases (micros overflow + timestamp not defined)
  if (Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

  // PID controller
  float proportional = P * error;
  float integral = integral_prev + I * Ts * 0.5f * (error + error_prev);
  integral = _constrain(integral, -limit, limit);
  float derivative = D * (error - error_prev) / Ts;
  float output = proportional + integral + derivative;
  output = _constrain(output, -limit, limit);

  // To limit the change rate of PID
  if (output_ramp > 0) {
    float output_rate = (output - output_prev) / Ts;
    if (output_rate > output_ramp)
      output = output_prev + output_ramp * Ts;
    else if (output_rate > -output_ramp)
      output = output_prev - output_ramp * Ts;
  }

  // Save the current PID parameters
  integral_prev = integral;
  output_prev = output;
  error_prev = error;
  timestamp_prev = timestamp_now;

  // printf("%0.2f, %0.2f, %0.2f\n", output, error, limit);
  return output;
}