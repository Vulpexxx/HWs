#ifndef _PID_HPP_
#define _PID_HPP_
#include <stdint.h>

class PID
{
public:
  PID(float kp, float ki, float kd);
  ~PID() = default;
  int16_t compute(float setpoint, float measured_value);

private:
  float kp_;
  float ki_;
  float kd_;
  float integral_;
  float previous_error_;
};

#endif
