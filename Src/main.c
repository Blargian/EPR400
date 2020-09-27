/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "lwrb/lwrb.h"
#include "temperatureSensor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUF_LEN 2

/*
 * Function prototypes for UART and DMA management
 * */
void usart_rx_manage();
void usart_process_dma(const void* data, size_t len);
uint8_t usart_start_tx_dma_transfer(void);
void usart_send_string(const char* str);//

#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0])) //Array size
static uint8_t usart_rx_dma_buffer[64]; //Buffer which the DMA works with

/*Initialise ring buffer variables*/

/*For TX*/

static lwrb_t usart_tx_ringbuff;
static uint8_t usart_tx_ringbuff_data[128];
static size_t usart_tx_ringbuff_len;

/*For RX*/

static lwrb_t usart_rx_ringbuff;
static uint8_t usart_rx_ringbuff_data[128];
static size_t usart_rx_ringbuff_len;



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
uint16_t waxThermistor;
static uint16_t temperatureInDegrees;
char sendTemp [8];


/*For Limit Switches */
uint16_t limitSwitchState = 1;
uint16_t limitSwitch1_Trigger = 0;
uint16_t limitSwitch2_Trigger = 0;
uint16_t direction_x = 1; //Initially set 1 to home towards limit switch
uint16_t direction_y = 0; //Initially set 1 to home towards limit switch

uint16_t steps_x_target=0;
uint16_t steps_y_target=0;
uint16_t steps_z_target=0;

volatile int RELEASE = 1;

uint16_t steps_x = 0;
uint16_t steps_y = 0;
uint16_t steps_z = 0;

/*Program actions*/
uint16_t homing = 0;
uint16_t heater = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* Initialize the ringbuffers */
  lwrb_init(&usart_tx_ringbuff, usart_tx_ringbuff_data, sizeof(usart_tx_ringbuff_data));
  lwrb_init(&usart_rx_ringbuff, usart_rx_ringbuff_data, sizeof(usart_rx_ringbuff_data));

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  usart_send_string("READY");
  HAL_ADC_Start(&hadc1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	 if (HAL_ADC_PollForConversion(&hadc1, 1000000) == HAL_OK){
		 waxThermistor = HAL_ADC_GetValue(&hadc1);
		 temperatureInDegrees = read_temp(waxThermistor);
	 }

	 if(homing==1){
		 homing_XY();
		 //step('Y',3000,0);
		 homing=0;
		  }
	 if(heater==1){
		 HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	     heater = 0;
	 }

	 int currentTemperature = temperatureInDegrees;
	 int pi = PI(30,currentTemperature,50,0);
	 int actuate = limitActuation(pi,0,500);
	 htim3.Instance->CCR1 = actuate;
	 sendTemperature(currentTemperature);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);

   if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
  {
    Error_Handler();  
  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {
    
  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_16);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
    
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_4);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  
  }
  LL_SetSystemCoreClock(64000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();  
  };
  LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSRC_PCLK2_DIV_2);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 100;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_DISABLE_OCxPRELOAD(&htim1, TIM_CHANNEL_1);
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 200;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1280;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1280;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
  
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  /**USART1 GPIO Configuration  
  PA9   ------> USART1_TX
  PA10   ------> USART1_RX 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART1 DMA Init */
  
  /* USART1_RX Init */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_5, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MODE_CIRCULAR);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MDATAALIGN_BYTE);

  /* USART1_TX Init */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_4, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MDATAALIGN_BYTE);

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */
  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_5, (uint32_t)&USART1->DR);
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_5, (uint32_t)usart_rx_dma_buffer);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_5, ARRAY_LEN(usart_rx_dma_buffer));
  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_4, (uint32_t)&USART1->DR);

  /* Enable HT & TC interrupts for RX */
   LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_5);
   LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_5);

  /* Enable HT & TC interrupts for TX */
   LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_4);

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
  /* USER CODE BEGIN USART1_Init 2 */
  LL_USART_EnableDMAReq_RX(USART1);
  LL_USART_EnableDMAReq_TX(USART1);
  LL_USART_EnableIT_IDLE(USART1);



  /* USER CODE END USART1_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, MotorZ_DIR_Pin|MotorY_DIR_Pin|MotorX_DIR_Pin);

  /**/
  LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTB, LL_GPIO_AF_EXTI_LINE0);

  /**/
  LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTB, LL_GPIO_AF_EXTI_LINE1);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_0;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_1;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  LL_GPIO_SetPinPull(Limit_SwitchX_GPIO_Port, Limit_SwitchX_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_1, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinMode(Limit_SwitchX_GPIO_Port, Limit_SwitchX_Pin, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_1, LL_GPIO_MODE_INPUT);

  /**/
  GPIO_InitStruct.Pin = MotorZ_DIR_Pin|MotorY_DIR_Pin|MotorX_DIR_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  NVIC_SetPriority(EXTI0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(EXTI0_IRQn);
  NVIC_SetPriority(EXTI1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */

/*
 * This function is used to write a string to the TX buffer and then calls the
 * start DMA transfer function. credit: MaJerle
 * */

void
usart_send_string(const char* str) {
    lwrb_write(&usart_tx_ringbuff, str, strlen(str));   /* Write data to TX buffer for loopback */
    usart_start_tx_dma_transfer();              /* Then try to start transfer */
}

/*
 * This function is used to write a string to start the DMA transfer
 * Called by usart_send_string
 * credit: MaJerle
 * */

uint8_t usart_start_tx_dma_transfer(void){
	uint32_t old_primask;
	uint8_t started = 0;

	old_primask = __get_PRIMASK();
	__disable_irq();

	if(usart_tx_ringbuff_len ==0){
		usart_tx_ringbuff_len = lwrb_get_linear_block_read_length(&usart_tx_ringbuff);
		if(usart_tx_ringbuff_len>0){
			LL_DMA_DisableChannel(DMA1,LL_DMA_CHANNEL_4);

			/* Clear all flags */
			LL_DMA_ClearFlag_TC2(DMA1);
			LL_DMA_ClearFlag_HT2(DMA1);
			LL_DMA_ClearFlag_GI2(DMA1);
			LL_DMA_ClearFlag_TE2(DMA1);

			/* Start DMA transfer */
			LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_4, usart_tx_ringbuff_len);
			LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_4, (uint32_t)lwrb_get_linear_block_read_address(&usart_tx_ringbuff));

			/* Start new transfer */
			LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_4);
			started = 1;

		}
	}

	__set_PRIMASK(old_primask);
	return started;
}


/*This function copies data available for write from the DMA buffer to
 *a working buffer for use within the program. Uses the lwrb library
 *for ring buffer support.
 */

void usart_process_data(const void* data, size_t len){
	lwrb_write(&usart_rx_ringbuff, data, len);  /* Write data to RX buffer for editing */
}

/*This function handles the various cases for DMA read/write operation
 *to determine which data to transfer from the DMA buffer to some
 *working buffer. See stm32-usart-uart-dma-rx-tx by MaJerle.
 * */
void usart_rx_manage(){
	static size_t old_position;
	size_t position;

	/*Calculate the current position in the buffer*/
	position = ARRAY_LEN(usart_rx_dma_buffer) - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_5);
	/*Check if there has been a change*/
	if(old_position != position){

		/*"Linear mode" where we will subtract pointers*/

		if(position > old_position){
			usart_process_data(&usart_rx_dma_buffer[old_position], position - old_position);
		} else {
		/*"Overflow mode" where we will subtract pointers*/

			/*process to the end of the buffer*/
			usart_process_data(&usart_rx_dma_buffer[old_position], ARRAY_LEN(usart_rx_dma_buffer) - old_position);

			if (position > 0){
				usart_process_data(&usart_rx_dma_buffer[0], position);
			}
				}

		old_position = position; /*Save the current position as the old one for next run*/

		/*Check for if the end of the buffer is reached and if so reset*/
		if(old_position == ARRAY_LEN(usart_rx_dma_buffer)){
			old_position=0;
		}
	}
}
//Interrupt Handlers

void DMA1_Channel4_IRQHandler(void) {
	/* Check transfer-complete interrupt */
	if (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_CHANNEL_4) && LL_DMA_IsActiveFlag_TC4(DMA1)) {
		LL_DMA_ClearFlag_TC4(DMA1);             /* Clear transfer complete flag */
	    lwrb_skip(&usart_tx_ringbuff, usart_tx_ringbuff_len);/* Skip buffer, it has been successfully sent out */
	    usart_tx_ringbuff_len = 0;           /* Reset data length */
	    usart_start_tx_dma_transfer();          /* Start new transfer */
	    }
}

void DMA1_Channel5_IRQHandler(void)
{

	/* Check if the Half transfer flag is enabled and raised */
	if(LL_DMA_IsEnabledIT_HT(DMA1,LL_DMA_CHANNEL_5) && LL_DMA_IsActiveFlag_HT5(DMA1)){
		LL_DMA_ClearFlag_HT5(DMA1); /* Clear the Half-Transfer complete flag */
		usart_rx_manage();	/* call */
	}

	/*Check if the Transfer Complete flag is enabled and raised */
	if(LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_CHANNEL_5) && LL_DMA_IsActiveFlag_TC5(DMA1)){
		LL_DMA_ClearFlag_HT5(DMA1); /* Clear the Transfer-COmplete flag */
		usart_rx_manage();
	}

}

void USART1_IRQHandler(void)
{
  /*Check if the Idle flag is enabled and is raised*/
	if(LL_USART_IsEnabledIT_IDLE(USART1) && LL_USART_IsActiveFlag_IDLE(USART1)){
		LL_USART_ClearFlag_IDLE(USART1);      /* Clear IDLE line flag */
		usart_rx_manage();	/* Call check function */
	}

}

// Thermistor

//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
//	//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
//	waxThermistor=adc_buf[0];
//	temperatureInDegrees = read_temp(waxThermistor);
//	HAL_ADC_Start(&hadc1);
//}

/*Interrupt Routines for counting Steps*/

void TIM1_UP_IRQHandler(void)
{
  if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE))
  {
    if (__HAL_TIM_GET_IT_SOURCE(&htim1, TIM_IT_UPDATE))
    {
    	step_update('X');
    	__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
        __HAL_TIM_CLEAR_FLAG(&htim1,TIM_FLAG_CC1 );
        __HAL_TIM_CLEAR_FLAG(&htim1,TIM_FLAG_CC2 );
        __HAL_TIM_CLEAR_FLAG(&htim1,TIM_FLAG_CC3 );
        __HAL_TIM_CLEAR_FLAG(&htim1,TIM_FLAG_CC4 );
      }
    }
  }


void TIM2_IRQHandler(void)
{
	if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE))
	  {
	    if (__HAL_TIM_GET_IT_SOURCE(&htim2, TIM_IT_UPDATE))
	    {
	    	step_update('Y');
	    	__HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
	        __HAL_TIM_CLEAR_FLAG(&htim2,TIM_FLAG_CC1 );
	        __HAL_TIM_CLEAR_FLAG(&htim2,TIM_FLAG_CC2 );
	        __HAL_TIM_CLEAR_FLAG(&htim2,TIM_FLAG_CC3 );
	        __HAL_TIM_CLEAR_FLAG(&htim2,TIM_FLAG_CC4 );
	      }
	    }
}

void TIM4_IRQHandler(void)
{
	if (__HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_UPDATE))
	  {
	    if (__HAL_TIM_GET_IT_SOURCE(&htim4, TIM_IT_UPDATE))
	    {
	    	step_update('Z');
	    	__HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
	        __HAL_TIM_CLEAR_FLAG(&htim4,TIM_FLAG_CC1);
	        __HAL_TIM_CLEAR_FLAG(&htim4,TIM_FLAG_CC2);
	        __HAL_TIM_CLEAR_FLAG(&htim4,TIM_FLAG_CC3);
	        __HAL_TIM_CLEAR_FLAG(&htim4,TIM_FLAG_CC4);
	      }
	    }
}

void step(char axis, uint16_t numberSteps, uint16_t direction){

	//If Release is false hang here. Release is set to false after starting the PWM to ensure that further calls can't be made before
	// steps complete.

	while(RELEASE!=1){
		{}
	}

	switch(axis){
	case 'X':
		RELEASE=0;
		steps_x_target = numberSteps;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, direction);
		__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE );
		HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
		break;
	case 'Y':
		RELEASE=0;
		steps_y_target = numberSteps;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, direction);
		__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE );
		HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
		break;

	case 'Z':
		RELEASE=0;
		steps_z_target = numberSteps;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, direction);
		__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE );
		HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_1);
		break;
	}
}

void step_update(char axis){
	switch(axis){
		case 'X':
			steps_x++;
			if(steps_x==(2*steps_x_target)){
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
				steps_x_target=0;
				steps_x=0;
				RELEASE=1;
			}
			break;
		case 'Y':
			steps_y++;
			if(steps_y==(2*steps_y_target)){
				HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
				steps_y_target=0;
				steps_y=0;
				RELEASE=1;
					}
			break;

		case 'Z':
			steps_z++;
			if(steps_z==(2*steps_z_target)){
				HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
				steps_z_target=0;
				steps_z=0;
				RELEASE=1;
					}
			break;

		}
}

void homing_XY(){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, direction_x);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

// Handling of interrupts for limit switches

void limitSwitch1Trigger(void){
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	direction_x = 0;
	RELEASE = 1;
	step('X',200,direction_x);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, direction_y);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

void limitSwitch2Trigger(void){
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	direction_y = 1;
	RELEASE = 1;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, direction_y);
	step('Y',200,direction_y);

}

int PI(int setpoint, int current, int Kp, int Ki){

	if(current==0){
			return 0;
		}

	int err = setpoint - current;
	int P = Kp*err;
	return P;
}

int limitActuation(int input, int min, int max){
	if(input<min) return min;
	if(input>max) return max;
	return input;
}

void sendTemperature(int temperature){
	itoa(temperature,sendTemp,10);
	strcat(sendTemp,"\n");
	usart_send_string(sendTemp);
	sendTemp[0] = '\0';
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
