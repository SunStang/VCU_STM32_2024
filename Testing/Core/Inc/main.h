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
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define Regen_ADJ_Pin GPIO_PIN_0
#define Regen_ADJ_GPIO_Port GPIOA
#define Extra_Analog_Input_Pin GPIO_PIN_1
#define Extra_Analog_Input_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define Accelerator_Pin GPIO_PIN_4
#define Accelerator_GPIO_Port GPIOA
#define Extra_2_Input_Pin GPIO_PIN_2
#define Extra_2_Input_GPIO_Port GPIOB
#define Extra_1_Input_Pin GPIO_PIN_10
#define Extra_1_Input_GPIO_Port GPIOB
#define Hand_Brake_Pin GPIO_PIN_11
#define Hand_Brake_GPIO_Port GPIOB
#define Cruise_Control_Reset_Pin GPIO_PIN_12
#define Cruise_Control_Reset_GPIO_Port GPIOB
#define Cruise_Control_Set_Pin GPIO_PIN_13
#define Cruise_Control_Set_GPIO_Port GPIOB
#define Cruise_Control_EN_Pin GPIO_PIN_14
#define Cruise_Control_EN_GPIO_Port GPIOB
#define Neutral_Pin GPIO_PIN_15
#define Neutral_GPIO_Port GPIOB
#define Reverse_Pin GPIO_PIN_6
#define Reverse_GPIO_Port GPIOC
#define Drive_Pin GPIO_PIN_7
#define Drive_GPIO_Port GPIOC
#define Left_Turn_Pin GPIO_PIN_8
#define Left_Turn_GPIO_Port GPIOC
#define Right_Turn_Pin GPIO_PIN_9
#define Right_Turn_GPIO_Port GPIOC
#define Hazards_Pin GPIO_PIN_8
#define Hazards_GPIO_Port GPIOA
#define Foot_Brake_Pin GPIO_PIN_9
#define Foot_Brake_GPIO_Port GPIOA
#define Regen_EN_Pin GPIO_PIN_10
#define Regen_EN_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define Thermal_Shutoff_Pin GPIO_PIN_15
#define Thermal_Shutoff_GPIO_Port GPIOA
#define Left_Lights_Pin GPIO_PIN_10
#define Left_Lights_GPIO_Port GPIOC
#define Right_Lights_Pin GPIO_PIN_11
#define Right_Lights_GPIO_Port GPIOC
#define Brake_Lights_Pin GPIO_PIN_12
#define Brake_Lights_GPIO_Port GPIOC
#define DRL_Left_Pin GPIO_PIN_2
#define DRL_Left_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define DRL_Right_Pin GPIO_PIN_4
#define DRL_Right_GPIO_Port GPIOB
#define Extra_Output_Pin GPIO_PIN_5
#define Extra_Output_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
