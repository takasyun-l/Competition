/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <tim.h>
#include <gpio.h>
#include <pid.h>
#include <usart.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern pid_speed* MOTOR3508_PID_SPEED;
extern pid_speed* MOTOR2006_PID_SPEED;

extern pid_position* MOTOR3508_PID_POSITION;
extern pid_position_speed* MOTOR3508_PID_POSITION_SPEED;

extern pid_position_speed* MOTOR6020_PID_POSITION_SPEED;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


#define	BOXUPOSITIONFIRST					15000						//��һ�����ӷ���λ���Ϸ�
#define	BOXPOSITIONFIRST					9780						//��һ�����ӷ���λ��

#define	BOXUPOSITIONSECOND				18000						//�ڶ������ӷ���λ���Ϸ�
#define	BOXPOSITIONSECOND					16333						//�ڶ������ӷ���λ��

#define BOXUPOSITIONTHIRD					24000						//���������ӷ���λ���Ϸ�
#define	BOXPOSITIONTHIRD					22869						//���������ӷ���λ��

#define	COCAUPOSITION							13309						//���ַ���λ���Ϸ�
#define COCAPOSITION							10325						//���ַ���λ��
#define COCALLPOSITION						8525						//���п��ַ���λ��

#define	INITPOSITION							8000						//3508��ʼλ��
#define	BOXGET										6000						//3508ȡ����λ��
#define COCAGET										5000						//3508ȡ����λ��

#define BOXGIVEFIRST							17052						//�����²�����λ��
#define BOXGIVEFIRSTUP						18000						//�����²�����λ���Ϸ�
#define	BOXGIVESECOND							24052						//�����ϴ�����λ��
#define	BOXGIVESECONDUP						26000						//�����ϴ�����λ���Ϸ�

#define COCAGIVE									23325						//���ÿ���λ��
#define	COCAGIVEUP								25000						//���ÿ���λ���Ϸ�

#define	BOX6020										1775						//6020��������λ��
#define	COCA6020									1165						//6020���ÿ���λ��
#define	COCALL6020								1117						//6020ȡ���п���λ��							
#define INIT6020									1393						//6020��ʼλ��

#define	COCAFIR6020								1440						//6020�ŵ�һ������
#define COCASEC6020								1393						//6020�ŵڶ�������
#define COCATHI6020								1340						//6020�ŵ���������
#define COCAALL6020								1393						//6020�������п���								

#define	SPEED2006FOR							2000						//2006ǰ���ٶ�
#define	SPEED2006MID							2000						//2006�ж��ٶ�
#define	SPEED2006BACK							4000						//2006����ٶ�

#define DJSPEED										30							//����ٶ�
#define	DJANGLEFIR								4								//����һλ��
#define DJANGLESEC								124							//���ֶ�λ��
#define	DJANGLETHI								244							//������λ��
#define DJANGLE										20							//����λ��												

#define	Mode											0								//ģʽ�л� 0һ��һ��ץ	1ץȫ��


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

int			SensorPos 			= 0;
int			Sensor					=	0;
int			SensorF					=	1;
char		ItemType[1]			= "A";
uint8_t	RecData[1]			=	"N";
int			CocaNum					= 0;
int			BoxNum					= 0;

int			steering_speed;
float		angle;
int			step_angle			=	0;
int			MODE						= Mode;
int			PrePowerStatus	=	0;
/* USER CODE END Variables */
/* Definitions for SupportLifting */
osThreadId_t SupportLiftingHandle;
const osThreadAttr_t SupportLifting_attributes = {
  .name = "SupportLifting",
  .stack_size = 4096 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Signal */
osThreadId_t SignalHandle;
const osThreadAttr_t Signal_attributes = {
  .name = "Signal",
  .stack_size = 4096 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Transmit */
osThreadId_t TransmitHandle;
const osThreadAttr_t Transmit_attributes = {
  .name = "Transmit",
  .stack_size = 1028 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void steering_angle(int steering_speed, float angle)
{
  angle = 6.5f * angle + 495.f;
  if(step_angle <= angle)
  {
    for(;step_angle < angle;)
    {
      step_angle += steering_speed;
      __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, step_angle);
      vTaskDelay(50);
    }
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, angle);
  }
  else if(step_angle > angle)
  {
    for(;step_angle > angle;)
    {
      step_angle -= steering_speed;
      __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, step_angle);
      vTaskDelay(50);
    }
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, angle);
  }
  else if(step_angle == angle)
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, angle);

}

/* USER CODE END FunctionPrototypes */

void Lifting(void *argument);
void SignalTransmission(void *argument);
void Printf(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of SupportLifting */
  SupportLiftingHandle = osThreadNew(Lifting, NULL, &SupportLifting_attributes);

  /* creation of Signal */
  SignalHandle = osThreadNew(SignalTransmission, NULL, &Signal_attributes);

  /* creation of Transmit */
  TransmitHandle = osThreadNew(Printf, NULL, &Transmit_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Lifting */
/**
  * @brief  Function implementing the SupportLifting thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Lifting */
void Lifting(void *argument)
{
  /* USER CODE BEGIN Lifting */
  /* Infinite loop */
  for(;;)
  {
		if(PrePower == 1)
		{
		static _Bool Init = 0;
		if(Init == 0)
		{
			MOTOR3508_PID_POSITION_SPEED->TarPos = 0;
			MOTOR2006_PID_SPEED->TarSpd = 0;
			__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, INIT6020);
			steering_angle(30, 3);
			BUMP_A_CLOSE;
			BUMP_B_CLOSE;
			vTaskDelay(500);
		}
		Init = 1;
		
		static _Bool FirPart = 0;
		if(FirPart == 0)
		{
			MOTOR2006_PID_SPEED->TarSpd = SPEED2006FOR;
			MOTOR3508_PID_POSITION_SPEED->TarPos = INITPOSITION;
		}
		FirPart = 1;
		
		if((SensorPos == 1 || SensorPos == 2 || SensorPos == 3 || SensorPos == 4 || SensorPos == 5 || SensorPos == 6) && Sensor == 1)
		{
			MOTOR2006_PID_SPEED->TarSpd = 0;
			vTaskDelay(100);
			if(ItemType[0] == 'B')
			{
				ItemType[0] = 'A';
				BUMP_A_OPEN;
				MOTOR3508_PID_POSITION_SPEED->TarPos = BOXGET;
				vTaskDelay(2000);
				if(BoxNum == 0)
					MOTOR3508_PID_POSITION_SPEED->TarPos = BOXUPOSITIONFIRST;
				else if(BoxNum == 1)
					MOTOR3508_PID_POSITION_SPEED->TarPos = BOXUPOSITIONSECOND;
				else if(BoxNum == 2)
					MOTOR3508_PID_POSITION_SPEED->TarPos = BOXUPOSITIONTHIRD;
				vTaskDelay(2000);
				__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, BOX6020);
        vTaskDelay(3000);
				if(BoxNum == 0)
				{
					BoxNum++;
					MOTOR3508_PID_POSITION_SPEED->TarPos = BOXPOSITIONFIRST;
					vTaskDelay(2000);
					BUMP_A_CLOSE;
					vTaskDelay(1500);
					MOTOR3508_PID_POSITION_SPEED->TarPos = BOXUPOSITIONFIRST;
				}
				else if(BoxNum == 1)
				{
					BoxNum++;
					MOTOR3508_PID_POSITION_SPEED->TarPos = BOXPOSITIONSECOND;
					vTaskDelay(2000);
					BUMP_A_CLOSE;
					vTaskDelay(1500);
					MOTOR3508_PID_POSITION_SPEED->TarPos = BOXUPOSITIONSECOND;
				}
				else if(BoxNum == 2)
				{
					BoxNum++;
					MOTOR3508_PID_POSITION_SPEED->TarPos = BOXPOSITIONTHIRD;
					vTaskDelay(2000);
					BUMP_A_CLOSE;
					vTaskDelay(1500);
					MOTOR3508_PID_POSITION_SPEED->TarPos = BOXUPOSITIONTHIRD;
				}
				vTaskDelay(1000);
				if(SensorPos == 6)
				{
					MOTOR2006_PID_SPEED->TarSpd = SPEED2006BACK;
					vTaskDelay(500);
				}
				else
				{
          __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, INIT6020);
          vTaskDelay(2000);
					MOTOR3508_PID_POSITION_SPEED->TarPos = INITPOSITION;
					MOTOR2006_PID_SPEED->TarSpd = SPEED2006MID;
				}
				Sensor = 0;
			}
			else if(ItemType[0] == 'C')
			{
				ItemType[0] = 'A';
				BUMP_A_OPEN;
				MOTOR3508_PID_POSITION_SPEED->TarPos = COCAGET;
				vTaskDelay(2000);
				MOTOR3508_PID_POSITION_SPEED->TarPos = COCAUPOSITION;
				__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, COCA6020);
				if(CocaNum == 0)
				{
					CocaNum++;
					steering_angle(DJSPEED, DJANGLEFIR);
				}
				else if(CocaNum == 1)
				{
					CocaNum++;
					steering_angle(DJSPEED, DJANGLESEC);
				}
				else if(CocaNum == 2)
				{
					CocaNum++;
					steering_angle(DJSPEED, DJANGLETHI);
				}
        vTaskDelay(2500);
				MOTOR3508_PID_POSITION_SPEED->TarPos = COCAPOSITION;
				vTaskDelay(500);
				BUMP_A_CLOSE;
				vTaskDelay(1500);
				MOTOR3508_PID_POSITION_SPEED->TarPos = COCAUPOSITION;
				if(SensorPos == 6)
				{
					MOTOR3508_PID_POSITION_SPEED->TarPos = BOXUPOSITIONTHIRD;
					MOTOR2006_PID_SPEED->TarSpd = SPEED2006BACK;
					__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, BOX6020);
				}
				else
				{
          __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, INIT6020);
          vTaskDelay(3000);
					MOTOR3508_PID_POSITION_SPEED->TarPos = INITPOSITION;
					MOTOR2006_PID_SPEED->TarSpd = SPEED2006MID;
				}
				Sensor = 0;
			}
		}
		
		if(SensorPos == 7 && Sensor == 1)
		{
			MOTOR2006_PID_SPEED->TarSpd = 0;
			vTaskDelay(500);
			for(;BoxNum >= 1;BoxNum--)
			{
				if(BoxNum == 2)
				{
					BUMP_A_OPEN;
					MOTOR3508_PID_POSITION_SPEED->TarPos = BOXPOSITIONTHIRD;
					vTaskDelay(1500);
					MOTOR3508_PID_POSITION_SPEED->TarPos = BOXUPOSITIONTHIRD;
					vTaskDelay(1500);
					__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, INIT6020);
					vTaskDelay(3000);
					MOTOR3508_PID_POSITION_SPEED->TarPos = BOXGIVEFIRST;												//A4ֽ�²�����λ��
					vTaskDelay(1500);
					BUMP_A_CLOSE;
					vTaskDelay(1500);
					MOTOR3508_PID_POSITION_SPEED->TarPos = BOXUPOSITIONSECOND;
					vTaskDelay(1500);
					__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, BOX6020);
          vTaskDelay(2000);
				}
				else if(BoxNum == 1)
				{
					BUMP_A_OPEN;
					MOTOR3508_PID_POSITION_SPEED->TarPos = BOXPOSITIONSECOND;
					vTaskDelay(1500);
					MOTOR3508_PID_POSITION_SPEED->TarPos = BOXGIVESECONDUP;
					vTaskDelay(1500);
					__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, INIT6020);
					vTaskDelay(3000);
					MOTOR3508_PID_POSITION_SPEED->TarPos = BOXGIVESECOND;												//A4ֽ�ϲ�����λ��
					vTaskDelay(1500);
					BUMP_A_CLOSE;
					vTaskDelay(1500);
					MOTOR3508_PID_POSITION_SPEED->TarPos = BOXGIVESECONDUP;
					vTaskDelay(1500);
					__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, BOX6020);
				}	
        vTaskDelay(2000);
			}
			MOTOR2006_PID_SPEED->TarSpd = SPEED2006BACK;
			
			Sensor = 0;
		}
		else if(SensorPos == 8 && Sensor == 1 && MODE == 0)
		{
			MOTOR2006_PID_SPEED->TarSpd = 0;
			vTaskDelay(500);
			{				
				BUMP_A_OPEN;
				MOTOR3508_PID_POSITION_SPEED->TarPos = BOXPOSITIONFIRST;
				vTaskDelay(1500);
				MOTOR3508_PID_POSITION_SPEED->TarPos = BOXGIVEFIRSTUP;
				vTaskDelay(1500);
				__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, INIT6020);
				vTaskDelay(3000);
				MOTOR3508_PID_POSITION_SPEED->TarPos = BOXGIVEFIRST;												//A4ֽ�²�����λ��
				vTaskDelay(1500);
				BUMP_A_CLOSE;
				vTaskDelay(1500);
				MOTOR3508_PID_POSITION_SPEED->TarPos = BOXGIVEFIRSTUP;
				vTaskDelay(1500);
				__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, COCA6020);
				steering_angle(DJSPEED, DJANGLEFIR);
				vTaskDelay(2000);
			}
			
			for(;CocaNum >= 0;CocaNum--)
			{
				if(CocaNum == 2)
				{
					BUMP_A_OPEN;
					MOTOR3508_PID_POSITION_SPEED->TarPos = COCAPOSITION;
					vTaskDelay(1500);
					MOTOR3508_PID_POSITION_SPEED->TarPos = COCAGIVEUP;
					vTaskDelay(1500);
					__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, COCAFIR6020);
					vTaskDelay(3000);
					MOTOR3508_PID_POSITION_SPEED->TarPos = COCAGIVE;
					vTaskDelay(1500);
					BUMP_A_CLOSE;
					vTaskDelay(2000);
					MOTOR3508_PID_POSITION_SPEED->TarPos = COCAGIVEUP;
					vTaskDelay(1500);
					__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, COCA6020);
					steering_angle(DJSPEED, DJANGLESEC);
				vTaskDelay(3000);
				}
				if(CocaNum == 1)
				{
					BUMP_A_OPEN;
					MOTOR3508_PID_POSITION_SPEED->TarPos = COCAPOSITION;
					vTaskDelay(1500);
					MOTOR3508_PID_POSITION_SPEED->TarPos = COCAGIVEUP;
					vTaskDelay(1500);
					__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, COCASEC6020);
					vTaskDelay(3000);
					MOTOR3508_PID_POSITION_SPEED->TarPos = COCAGIVE;
					vTaskDelay(1500);
					BUMP_A_CLOSE;
					vTaskDelay(2000);
					MOTOR3508_PID_POSITION_SPEED->TarPos = COCAGIVEUP;
					vTaskDelay(1500);
					__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, COCA6020);
					steering_angle(DJSPEED, DJANGLETHI);
				vTaskDelay(3000);
				}
				if(CocaNum == 0)
				{
					BUMP_A_OPEN;
					MOTOR3508_PID_POSITION_SPEED->TarPos = COCAPOSITION;
					vTaskDelay(1500);
					MOTOR3508_PID_POSITION_SPEED->TarPos = COCAGIVEUP;
					vTaskDelay(1500);
					__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, COCATHI6020);
					vTaskDelay(3000);
					MOTOR3508_PID_POSITION_SPEED->TarPos = COCAGIVE;
					vTaskDelay(1500);
					BUMP_A_CLOSE;
					vTaskDelay(2000);
					MOTOR3508_PID_POSITION_SPEED->TarPos = COCAGIVEUP;
					vTaskDelay(1500);
				}
			}		
			Sensor = 0;
		}
		else if(SensorPos == 8 && Sensor == 1 && MODE == 1)
		{
			MOTOR2006_PID_SPEED->TarSpd = 0;
			vTaskDelay(1000);
			{				
				BUMP_A_OPEN;
				MOTOR3508_PID_POSITION_SPEED->TarPos = BOXPOSITIONFIRST;
				vTaskDelay(1500);
				MOTOR3508_PID_POSITION_SPEED->TarPos = BOXGIVEFIRSTUP;
				vTaskDelay(1500);
				__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, INIT6020);
				vTaskDelay(3000);
				MOTOR3508_PID_POSITION_SPEED->TarPos = BOXGIVEFIRST;												//A4ֽ�²�����λ��
				vTaskDelay(1500);
				BUMP_A_CLOSE;
				vTaskDelay(1500);
				MOTOR3508_PID_POSITION_SPEED->TarPos = BOXGIVEFIRSTUP;
				vTaskDelay(1500);
				__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, COCALL6020);
				steering_angle(DJSPEED, DJANGLE);
				vTaskDelay(2000);
			}
			BUMP_B_OPEN;
			MOTOR3508_PID_POSITION_SPEED->TarPos = COCALLPOSITION;
			vTaskDelay(5000);
			MOTOR3508_PID_POSITION_SPEED->TarPos = COCAGIVEUP;
			vTaskDelay(2000);
			__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, COCAALL6020);
			vTaskDelay(5000);
			MOTOR3508_PID_POSITION_SPEED->TarPos = COCAGIVE;
			vTaskDelay(2000);
			BUMP_B_CLOSE;
			vTaskDelay(3000);
			MOTOR3508_PID_POSITION_SPEED->TarPos = COCAGIVEUP;
      
      Sensor = 0;
		}
	}
		osDelay(1);
  }
  /* USER CODE END Lifting */
}

/* USER CODE BEGIN Header_SignalTransmission */
/**
* @brief Function implementing the Signal thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SignalTransmission */
void SignalTransmission(void *argument)
{
  /* USER CODE BEGIN SignalTransmission */
  /* Infinite loop */
  for(;;)
  {
		RecData[0] = 'N';
		HAL_UART_Receive(&huart2, RecData, sizeof(RecData), 0xff);
		switch(RecData[0])
		{
			
			case '1':
			{
				ItemType[0] = 'B';
				break;
			}
			case '0':
			{
				ItemType[0] = 'C';
				break;
			}
			default:
				break;
		}

    osDelay(1);
  }
  /* USER CODE END SignalTransmission */
}

/* USER CODE BEGIN Header_Printf */
/**
* @brief Function implementing the Transmit thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Printf */
void Printf(void *argument)
{
  /* USER CODE BEGIN Printf */
  /* Infinite loop */
  for(;;)
  {
		if(PrePower == 1)
		{
			PrePowerStatus = 1;
		}
		
		if(Sensor == 0 && SensorF == 1)
		{
			if(SensorStatus == 1)
				{
					if(SensorStatus == 1)
						{
							SensorPos++;
							Sensor = 1;
              vTaskDelay(500);
              printf("1");
						}
				}
		}
		if(SensorStatus == 1)
		{
			SensorF = 0;
		}
		else if(SensorStatus == 0)
		{
			SensorF = 1;
		}
		
    osDelay(1);
  }
  /* USER CODE END Printf */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

