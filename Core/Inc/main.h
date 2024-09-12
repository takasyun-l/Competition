/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
	
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* 3508接收数据格式 */
	typedef struct
{
    int16_t angle;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
	
	
    int16_t last_angle;
	  int16_t last_speed_rpm;
} motor_measure_t;

/* 电机ID */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x206,
    CAN_TRIGGER_MOTOR_ID = 0x207,
    CAN_GIMBAL_ALL_ID = 0x1FF,

} can_msg_id_e;
	
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SteerEngine_Pin GPIO_PIN_2
#define SteerEngine_GPIO_Port GPIOI
#define Bump_A_Pin GPIO_PIN_0
#define Bump_A_GPIO_Port GPIOI
#define Bump_B_Pin GPIO_PIN_12
#define Bump_B_GPIO_Port GPIOH
#define PrePower_Pin GPIO_PIN_2
#define PrePower_GPIO_Port GPIOB
#define Receive_Pin GPIO_PIN_1
#define Receive_GPIO_Port GPIOG
#define Sensor_Pin GPIO_PIN_13
#define Sensor_GPIO_Port GPIOD
#define YunTai_Pin GPIO_PIN_12
#define YunTai_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
			  (ptr)->last_speed_rpm = (ptr)->speed_rpm;                       \
        (ptr)->last_angle = (ptr)->angle;                               \
        (ptr)->angle = (uint16_t)((data)[0] << 8 | (data)[1]);          \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
