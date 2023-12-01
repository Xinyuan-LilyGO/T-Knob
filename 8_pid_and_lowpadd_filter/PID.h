#pragma once

class PIDController {
public:
  PIDController(float P, float I, float D, float ramp, float limit);
  ~PIDController() = default;

  float operator()(float error);  // 重载运算符 ()

  float P;
  float I;
  float D;
  float output_ramp;
  float limit;
protected:
  float error_prev;
  float output_prev;
  float integral_prev;
  unsigned long timestamp_prev;
};