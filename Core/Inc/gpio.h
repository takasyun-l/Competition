/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
	
#define BUMP_A_CLOSE								HAL_GPIO_WritePin(Bump_A_GPIO_Port, Bump_A_Pin, GPIO_PIN_SET)
#define BUMP_A_OPEN									HAL_GPIO_WritePin(Bump_A_GPIO_Port, Bump_A_Pin, GPIO_PIN_RESET)
	
#define BUMP_B_CLOSE								HAL_GPIO_WritePin(Bump_B_GPIO_Port, Bump_B_Pin, GPIO_PIN_SET)
#define BUMP_B_OPEN									HAL_GPIO_WritePin(Bump_B_GPIO_Port, Bump_B_Pin, GPIO_PIN_RESET)
	
#define SensorStatus								HAL_GPIO_ReadPin(Sensor_GPIO_Port,Sensor_Pin)
	
#define	PrePower										HAL_GPIO_ReadPin(PrePower_GPIO_Port,PrePower_Pin)
/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

