/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f1xx_hal.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_usart.h"
#include "stm32f1xx.h"
#include "stm32f1xx_ll_gpio.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void limitSwitch1Trigger(void);
void limitSwitch2Trigger(void);
void limitSwitch3Trigger(void);
void drawWax();
void step(char axis, uint16_t numberSteps, uint16_t direction);
void step_update(char axis);
void homing_XY();
void homing_Z();
float PI(float setpoint, float current, int Kp, int Ki);
int limitActuation(float input, int min, int max);
void sendTemperature(int time, int temperature);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define WaxHeater_PWM_Pin LL_GPIO_PIN_6
#define WaxHeater_PWM_GPIO_Port GPIOA
#define Limit_SwitchX_Pin LL_GPIO_PIN_0
#define Limit_SwitchX_GPIO_Port GPIOB
#define Limit_SwitchX_EXTI_IRQn EXTI0_IRQn
#define LimitSwitchY_Pin LL_GPIO_PIN_1
#define LimitSwitchY_GPIO_Port GPIOB
#define LimitSwitchY_EXTI_IRQn EXTI1_IRQn
#define LimitSwitchZ_Pin LL_GPIO_PIN_15
#define LimitSwitchZ_GPIO_Port GPIOB
#define LimitSwitchZ_EXTI_IRQn EXTI15_10_IRQn
#define MotorY_STEP_Pin LL_GPIO_PIN_15
#define MotorY_STEP_GPIO_Port GPIOA
#define MotorZ_DIR_Pin LL_GPIO_PIN_3
#define MotorZ_DIR_GPIO_Port GPIOB
#define MotorX_DIR_Pin LL_GPIO_PIN_5
#define MotorX_DIR_GPIO_Port GPIOB
#define MotorZ_STEP_Pin LL_GPIO_PIN_6
#define MotorZ_STEP_GPIO_Port GPIOB
#define MotorY_DIR_Pin LL_GPIO_PIN_7
#define MotorY_DIR_GPIO_Port GPIOB
#define Timing_Pin LL_GPIO_PIN_9
#define Timing_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
void TIM1_UP_IRQHandler(void);
void TIM2_IRQHandler(void);
void TIM4_IRQHandler(void);
void HAL_ADC_ConvHalfCpltCallback (ADC_HandleTypeDef *hadc);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
