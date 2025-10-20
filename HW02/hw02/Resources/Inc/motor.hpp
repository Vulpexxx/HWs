/*
 * @Author: Vulpex 2267339737@qq.com
 * @Date: 2025-10-18 19:45:17
 * @LastEditors: Vulpex 2267339737@qq.com
 * @LastEditTime: 2025-10-19 09:11:59
 * @FilePath: \HW02\Resources\Inc\motor.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _MOTOR_HPP_
#define _MOTOR_HPP_
#include <stdint.h>

class GM6020
{
public:
  uint16_t angle_;
  int16_t speed_;
  int16_t current_;
  uint32_t tx_id_;
  uint32_t rx_id_;
  GM6020() = default;
  GM6020(uint32_t tx_id, uint32_t rx_id);
  ~GM6020() = default;
  void encode(uint8_t *data);
  void decode(uint8_t *data);
  void setCurrent(int16_t &current) {
    current_ = current;
  }
};



#endif
