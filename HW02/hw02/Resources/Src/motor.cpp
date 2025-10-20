/*
 * @Author: Vulpex 2267339737@qq.com
 * @Date: 2025-10-18 19:45:25
 * @LastEditors: Vulpex 2267339737@qq.com
 * @LastEditTime: 2025-10-18 22:17:33
 * @FilePath: \HW02\Resources\Src\motor.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "motor.hpp"
#include <cstdint>
const int16_t MAX_CURRENT = 16384;
const int16_t MIN_CURRENT = -16384;

GM6020::GM6020(uint32_t tx_id, uint32_t rx_id)
{
  tx_id_ = tx_id;
  rx_id_ = rx_id;
  angle_ = 0;
  speed_ = 0;
  current_ = 0;
}

// 编码电机控制命令
void GM6020::encode(uint8_t *data)
{
  if(current_ > MAX_CURRENT) { current_ = MAX_CURRENT; }
  if(current_ < MIN_CURRENT) { current_ = MIN_CURRENT; }
  data[0] = current_ >> 8; // 电流高8位
  data[1] = current_; // 电流低8位
  data[2] = 0;
  data[3] = 0;
  data[4] = 0;
  data[5] = 0;
  data[6] = 0;
  data[7] = 0;
}

// 解码电机状态
void GM6020::decode(uint8_t *data)
{
  angle_ = ( data[0] << 8 ) + data[1]; //0~8191
  speed_ = ( data[2] << 8 ) + data[3]; //rpm
}

