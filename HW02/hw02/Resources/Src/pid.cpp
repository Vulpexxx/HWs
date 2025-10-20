/*
 * @Author: Vulpex 2267339737@qq.com
 * @Date: 2025-10-18 19:55:39
 * @LastEditors: Vulpex 2267339737@qq.com
 * @LastEditTime: 2025-10-20 14:23:57
 * @FilePath: \HW02\Resources\Src\pid.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 * @Author: Vulpex 2267339737@qq.com
 * @Date: 2025-10-18 19:55:39
 * @LastEditors: Vulpex 2267339737@qq.com
 * @LastEditTime: 2025-10-18 21:34:28
 * @FilePath: \HW02\Resources\Src\pid.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "pid.hpp"
#include <stdint.h>

PID::PID(float kp, float ki, float kd)
    : kp_(kp), ki_(ki), kd_(kd), integral_(0), previous_error_(0) {}

int16_t PID::compute(float setpoint, float measured_value){
  float error = setpoint - measured_value;

  // 处理环绕问题
  if (error > 4096) {
    error -= 8192;
  } else if (error < -4096){
    error += 8192;
  }

  integral_ += error;
  float derivative = error - previous_error_;
  float output = kp_ * error + ki_ * integral_ + kd_ * derivative;
  previous_error_ = error;
  return static_cast<int16_t>(output);
}
