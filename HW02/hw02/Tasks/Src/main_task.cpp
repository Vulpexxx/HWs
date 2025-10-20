/*
 * @Author: Vulpex 2267339737@qq.com
 * @Date: 2025-10-18 19:37:28
 * @LastEditors: Vulpex 2267339737@qq.com
 * @LastEditTime: 2025-10-20 15:56:06
 * @FilePath: \HW02\Tasks\Src\main_task.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
*******************************************************************************
* @file      ：<file>.c
* @brief     :
* @history   :
*  Version     Date            Author          Note
*  V0.9.0      yyyy-mm-dd      <author>        1. <note>
*******************************************************************************
* @attention :
*******************************************************************************
*  Copyright (c) 2024 Hello World Team，Zhejiang University.
*  All Rights Reserved.
*******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "stm32h7xx.h"
#include "main_task.hpp"
#include "math.h"
#include "tim.h"
#include "stdint.h"
#include "gpio.h"
#include "HW_fdcan.hpp"
#include "string.h"
#include "motor.hpp"
#include "pid.hpp"

/* Private macro -------------------------------------------------------------*/
#define TX_ID 0x1FE
#define RX_ID 0x205
#define KP 5.0f// 速度闭环用 20.0f
#define KI 0.0f
#define KD 0.0f
// 弧度 -> 编码器值 (GM6020范围 0~8191 对应 0~2PI)
#define RAD_TO_ENCODER(rad) ((rad) / (2.0f * M_PI) * 8192.0f)
// 每次设定点变化的最大增量,防止突变剧烈震动
#define RAMP_INCREMENT 5.0f
/* Private constants ---------------------------------------------------------*/
// 1. −5π/6 到 5π/6
const float POS_1A_RAD = -5.0f * M_PI / 6.0f;
const float POS_1B_RAD = 5.0f * M_PI / 6.0f;
// 2. π/3 到 2π/3
const float POS_2A_RAD = M_PI / 3.0f;
const float POS_2B_RAD = 2.0f * M_PI / 3.0f;
// 3. π/4 到 −π
const float POS_3A_RAD = M_PI / 4.0f;
const float POS_3B_RAD = -M_PI;
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t tick = 0;

float target_angle = 0;
float setpoint_angle = 0;
uint8_t stage = 0; // 当前运动状态:0-5

// int16_t target_speed = 0;// 速度控制用
GM6020 My_motor(TX_ID, RX_ID);
PID pid(KP, KI, KD);
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void MOTOR_Load(float target_angle);
// void MOTOR_Load(int16_t target_speed);// 速度控制用

void MainInit(void){
  FdcanFilterInit(&hfdcan1, FDCAN_FILTER_TO_RXFIFO0);
  HAL_FDCAN_Start(&hfdcan1);
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  HAL_TIM_Base_Start_IT(&htim6);
}

void MainTask(void){
  tick++;

  if(tick % 1000 == 0) { // 每1秒切换一次状态
    switch(stage) {
      case 0: target_angle = RAD_TO_ENCODER(POS_1A_RAD); break;
      case 1: target_angle = RAD_TO_ENCODER(POS_1B_RAD); break;
      case 2: target_angle = RAD_TO_ENCODER(POS_2A_RAD); break;
      case 3: target_angle = RAD_TO_ENCODER(POS_2B_RAD); break;
      case 4: target_angle = RAD_TO_ENCODER(POS_3A_RAD); break;
      case 5: target_angle = RAD_TO_ENCODER(POS_3B_RAD); break;
    }
    stage = (stage + 1) % 6; // 循环切换状态
  }
  // 确保目标角度在 [0, 8192) 范围内环绕
  while (target_angle < 0.0f) {
      target_angle += 8192.0f;
  }
  while (target_angle >= 8192.0f) {
      target_angle -= 8192.0f;
  }
  // 计算当前设定点与目标点的误差
  float ramp_error = target_angle - setpoint_angle;
  if (ramp_error > 4096.0f) {
    ramp_error -= 8192.0f;
  } else if (ramp_error < -4096.0f) {
    ramp_error += 8192.0f;
  }
  // 根据误差，让当前设定点向最终目标移动一小步
  if (ramp_error > RAMP_INCREMENT) {
    setpoint_angle += RAMP_INCREMENT;
  } else if (ramp_error < -RAMP_INCREMENT) {
    setpoint_angle -= RAMP_INCREMENT;
  } else {
    // 如果误差已经很小，就直接到达目标，防止抖动
    setpoint_angle = target_angle;
  }
  // 确保当前设定点也在 [0, 8192) 范围内环绕
  while (setpoint_angle >= 8192.0f) {
    setpoint_angle -= 8192.0f;
  }
  while (setpoint_angle < 0.0f) {
    setpoint_angle += 8192.0f;
  }
  
  MOTOR_Load(setpoint_angle);

  // target_speed = 200 * sin(tick / 1000.0f); // 速度控制用
  // MOTOR_Load(target_speed); // 速度控制用

}
  

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == (&htim6))
    {
      MainTask();
    }
}

// 速度控制用
// void MOTOR_Load(int16_t target_speed)
// {
//   int16_t output = 0;
//   uint8_t data[8] = {0};
//   output = pid.compute(target_speed, My_motor.speed_);
//   My_motor.setCurrent(output);
//   My_motor.encode(data);
//   FdcanSendMsg(&hfdcan1, data, My_motor.tx_id_, 8);
// }

// 位置控制（配套pid.compute()中有一段处理环绕问题的逻辑）
void MOTOR_Load(float target_angle)
{
  int16_t output = 0;
  uint8_t data[8] = {0};

  output = pid.compute(target_angle, My_motor.angle_);
  My_motor.setCurrent(output);
  My_motor.encode(data);
  FdcanSendMsg(&hfdcan1, data, My_motor.tx_id_, 8);
}