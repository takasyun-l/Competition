/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */

CAN_RxHeaderTypeDef CAN1_RxHander;             //
CAN_TxHeaderTypeDef CAN1_TxHander;             //
motor_measure_t motor_chassis[7];              //用于储存8个电机的传感器数据的结构体
uint8_t RxData[8] = {0};                       //
uint8_t TxData[8] = {0};                       //

int16_t InitPis = 0;

int16_t InitPosition[1000] = {0};

extern pid_speed* MOTOR3508_PID_SPEED;
extern pid_speed* MOTOR2006_PID_SPEED;
extern pid_position* MOTOR3508_PID_POSITION;
extern pid_position_speed* MOTOR3508_PID_POSITION_SPEED;
extern pid_position_speed* MOTOR6020_PID_POSITION_SPEED;

extern int PrePowerStatus;

int16_t findMode(int16_t arr[], int size) 
{
    // 为了方便处理，先将数组排序
	for (int i = 0; i < size - 1; i++) 
	{
		for (int j = 0; j < size - i - 1; j++) 
		{
			if (arr[j] > arr[j + 1]) 
				{
				// 交换元素
				int16_t temp = arr[j];
				arr[j] = arr[j + 1];
				arr[j + 1] = temp;
				}
		}
	}
	int maxCount = 0; // 记录当前众数的出现次数
	int currentCount = 1; // 记录当前数字的出现次数
	int mode = arr[0]; // 当前众数
	for(int i = 1; i < size; i++) 
	{
		if(arr[i] == arr[i - 1])
			{
				currentCount++;
			} 
			else 
				{
					if(currentCount > maxCount)
						{
							maxCount = currentCount;
							mode = arr[i - 1];
						}
						currentCount = 1;
				}
	}
	// 处理数组末尾的情况
	if(currentCount > maxCount) 
		{
			maxCount = currentCount;
			mode = arr[size - 1];
		}
		return mode;
}
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_10TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
	
	CAN_FilterTypeDef sFilterConfig;
	
	/* 配置CAN过滤器 */
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;
	
	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	
  /* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/*  */
 void CurrentControl(int16_t current1, int16_t current2, int16_t current3, int16_t current4)
{
	CAN1_TxHander.IDE = CAN_ID_STD;
	CAN1_TxHander.RTR = CAN_RTR_DATA;
	CAN1_TxHander.DLC = 0x08;
	CAN1_TxHander.StdId = 0x200;
	
	TxData[0] = current1 >> 8;
	TxData[1] = current1;
	TxData[2] = current2 >> 8;
	TxData[3] = current2;
	TxData[4] = current3 >> 8;
	TxData[5] = current3;
	TxData[6] = current4 >> 8;
	TxData[7] = current4;
	
	HAL_CAN_AddTxMessage(&hcan1, &CAN1_TxHander, TxData, 0);
}

void VoltageControl(int16_t voltage1, int16_t voltage2, int16_t voltage3, int16_t voltage4)
{
	CAN1_TxHander.IDE = CAN_ID_STD;
	CAN1_TxHander.RTR = CAN_RTR_DATA;
	CAN1_TxHander.DLC = 0x08;
	CAN1_TxHander.StdId = 0x1ff;
	
	TxData[0] = voltage1 >> 8;
	TxData[1] = voltage1;
	TxData[2] = voltage2 >> 8;
	TxData[3] = voltage2;
	TxData[4] = voltage3 >> 8;
	TxData[5] = voltage3;
	TxData[6] = voltage4 >> 8;
	TxData[7] = voltage4;
	
	HAL_CAN_AddTxMessage(&hcan1, &CAN1_TxHander, TxData, 0);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &CAN1_RxHander, RxData);
	static uint8_t i = 0;
	static int16_t OUT11 = 0, OUT12 = 0, OUT13 = 0, OUT14 = 0;
	//static int16_t OUT21 = 0, OUT22 = 0, OUT23 = 0, OUT24 = 0;
	    switch (CAN1_RxHander.StdId)
    {
        case CAN_3508_M1_ID:
        case CAN_3508_M2_ID:
        case CAN_3508_M3_ID:
        case CAN_3508_M4_ID:
        case CAN_YAW_MOTOR_ID:
        case CAN_PIT_MOTOR_ID:
        case CAN_TRIGGER_MOTOR_ID:
        {
            //get motor id
            i = CAN1_RxHander.StdId - CAN_3508_M1_ID;
            get_motor_measure(&motor_chassis[i], RxData);
            break;
        }

        default:
        {
            break;
        }
    }
		if(i == 0)
		{
			static _Bool Start = 0;
			static int Pos = 0;
			if(Start == 0)
			{
				InitPosition[Pos] = motor_chassis[i].angle;
				Pos++;
				if(Pos == 1000)
				{
					InitPis = findMode(InitPosition, 1000);
					Start = 1;
				}
			}
			
			if(Start == 1)
				OUT11 =PidCount_Pos_Spd(MOTOR3508_PID_POSITION_SPEED, motor_chassis[i], 3508, InitPis);
		}
		if(i == 1)
		{
			OUT12 = PidCount_Spd(MOTOR2006_PID_SPEED, motor_chassis[i]);
		}
//		if(i == 4)
//		{
//			OUT21 = PidCount_Pos_Spd(MOTOR6020_PID_POSITION_SPEED, motor_chassis[i], 6020);
//		}
		if(PrePowerStatus == 1)
			CurrentControl(OUT11, OUT12, OUT13, OUT14);
		//VoltageControl(OUT21, OUT22, OUT23, OUT24);
}

/* USER CODE END 1 */
