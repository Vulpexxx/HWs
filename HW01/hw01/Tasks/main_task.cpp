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
#include "stm32f103xb.h"
#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include "main_task.hpp"
#include "math.h"
#include "tim.h"
#include "stdint.h"
#include "gpio.h"
#include "usart.h"
#include "can.h"
#include "HW_can.hpp"
#include "string.h"

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t tick = 0;
//UART definitions
struct UartCommData
{
  uint32_t tick;
  float value;
};
UartCommData uart_data_tx,uart_data_rx;
uint8_t uart_rx_data[9];
uint8_t uart_tx_data[9];

//CAN definitions
struct CANCommData
{
  uint32_t tick;
  float value1;
  uint8_t value2;
  bool flag1;
  bool flag2;
  bool flag3;
  bool flag4;
};
CANCommData can_data_tx,can_data_rx;
extern uint8_t can_rx_data[8];
uint8_t can_tx_data[8];

/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void UART_Encode(uint8_t *data, UartCommData &uart_data);
void UART_Decode(uint8_t *data, UartCommData &uart_data);
void CAN_Encode(uint8_t *data, CANCommData &can_data);
void CAN_Decode(uint8_t *data, CANCommData &can_data);

//MAIN
void RobotInit(){
  uart_data_rx.tick = 0;
  uart_data_rx.value = 0;
  uart_data_tx.tick = 0;
  uart_data_tx.value = 0;
}

void MainInit(void){
  RobotInit();

  HAL_TIM_Base_Start_IT(&htim3);

  HAL_UART_Receive_DMA(&huart2, uart_rx_data, 9);

  CanFilter_Init(&hcan);
  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void MainTask(void){
  tick++;
  
  uart_data_tx.tick = tick;
  uart_data_tx.value = sin(tick * 0.001f);

  can_data_tx.tick = tick;
  can_data_tx.value1 = sin(tick * 0.001f);
  
    //不知道填什么，随便填的
  can_data_tx.value2 = 5;
  can_data_tx.flag1 = 1;
  can_data_tx.flag2 = 0;
  can_data_tx.flag3 = 1;
  can_data_tx.flag4 = 0;

  if(tick % 1000 == 0){
      HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
  }
  if(tick%100 == 0){
    UART_Encode(uart_tx_data, uart_data_tx);
    HAL_UART_Transmit_DMA(&huart1, uart_tx_data, 9);
  }

  CAN_Encode(can_tx_data, can_data_tx);
  CAN_Send_Msg(&hcan, can_tx_data, 0x100, 8);
  CAN_Decode(can_rx_data, can_data_rx);
}

/* Interrupt Handlers --------------------------------------------------------*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim3)
  {
    MainTask();
  }
}

uint32_t uart_receive_times = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart == &huart2)
  {
    UART_Decode(uart_rx_data, uart_data_rx);
    memset(uart_rx_data, 0, 9);
    HAL_UART_Receive_DMA(&huart2, uart_rx_data, 9);
    uart_receive_times++;
  }
}

//UART编解码
void UART_Encode(uint8_t *data, UartCommData &uart_data){
  data[0] = 0xAA;
  data[1] = 0xBB;
  data[2] = 0xCC;
  data[3] = uart_data.tick >> 24;
  data[4] = uart_data.tick >> 16;
  data[5] = uart_data.tick >> 8;
  data[6] = uart_data.tick;
  int16_t temp = uart_data.value * 30000;
  data[7] = temp >>8;
  data[8] = temp;
}

void UART_Decode(uint8_t *data, UartCommData &uart_data){
  int16_t temp = 0;
  uart_data.tick = 0;
  for(int i = 3; i < 7; i++){
    uart_data.tick += data[i] << (8*(6-i));
  }
  for(int i = 7; i < 9; i++){
    temp += data[i] << (8*(8-i));
  }

  uart_data.value = 1.0*temp/30000;
}

void CAN_Encode(uint8_t *data, CANCommData &can_data){
    data[0] = can_data.tick >> 24;
    data[1] = can_data.tick >> 16;
    data[2] = can_data.tick >> 8;
    data[3] = can_data.tick >> 0;
    int16_t temp = can_data.value1 * 30000;
    data[4] = temp >> 8;
    data[5] = temp >> 0;
    data[6] = can_data.value2;
    data[7] = (can_data.flag1 << 3) | (can_data.flag2 << 2) | (can_data.flag3 << 1) | (can_data.flag4 << 0);
}

void CAN_Decode(uint8_t *data, CANCommData &can_data){
    int16_t temp = 0;
    can_data.tick = 0;
    for(int i = 0; i < 4; i++){
        can_data.tick += data[i] << (8*(3-i));
    }
    for(int i = 4; i < 6; i++){
        temp += data[i] << (8*(5-i));
    }
    can_data.value1 = 1.0*temp/30000;
    can_data.value2 = data[6];
    can_data.flag1 = (data[7] >> 3) & 0x01;
    can_data.flag2 = (data[7] >> 2) & 0x01;
    can_data.flag3 = (data[7] >> 1) & 0x01;
    can_data.flag4 = (data[7] >> 0) & 0x01;
}