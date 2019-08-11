/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include <stdio.h>
#include "string.h"
#include "IR-remote.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

uint16_t x1 = 4;
uint8_t Rdata[50]; //an array for the data that the arm is going to receive

//#define StartTime1NEC 		9000
//#define StartTime2NEC 		4500
//#define HighTimeNEC 			560
//#define ZeroLowTimeNEC		560
//#define OneLowTimeNEC			1690

//#define StartTime1SAM 		4500
//#define StartTime2SAM 		4500
//#define HighTimeSAM 			560
//#define ZeroLowTimeSAM		560
//#define OneLowTimeSAM			1690

//#define StartTime1Sony 		2400
//#define StartTime2Sony 		600
//#define LowTimeSony 			600
//#define ZeroHighTimeSony	600
//#define OneHighTimeSony		1200

#define StartTime1Panasonic 		3456
#define StartTime2Panasonic 		1728
#define HighTimePanasonic 			432
#define ZeroLowTimePanasonic		432
#define OneLowTimePanasonic			1296

#define mask32		0x80000000
#define mask16		0x8000


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
//void SendNEC(const uint32_t data );
//void SendStartNEC(void);
//void SendDataNEC(const uint32_t data);
//void SendEndNEC(void);
//void SendHigh(void);
//void SendOne(void);
//void SendZero(void);
//void uDelay(const uint16_t);
//void SendSony(const uint16_t data );
//void SendStartSony(void);
//void SendDataSony(const uint16_t data);
//void SendOneSony(void);
//void SendZeroSony(void);
//void SendSAM(const uint32_t data );
//void SendStartSAM(void);
//void SendDataSAM(const uint32_t data);
//void SendHighSAM(void);
//void SendOneSAM(void);
//void SendZeroSAM(void);
//void SendEndSAM(void);
void SendPAN(const uint32_t data );
void SendStartPAN(void);
void SendDataPAN(const uint32_t data);
void SendEndPAN(void);
void SendHighPAN(void);
void SendOnePAN(void);
void SendZeroPAN(void);


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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_Base_Start(&htim3);
	HAL_UART_Receive_IT(&huart1,Rdata,1);// activating an interrupt for reading 1 byte from the laptop
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, x1);
		
		//SendNEC(0x20DF10EF);
		//SendSAM(0xB24D5FA0);
		HAL_Delay(50);

		//SendSony(0x20DF);
		
		//HAL_Delay(500);
		
		
  }
	
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
	{
		if(Rdata[0]=='n')
		{
			//SendPAN(0xE0E040BF);
			SendNEC(0x20DF10EF, htim1, x1);
//			SendNEC(0xFFFFFFFF);
		}
		else if(Rdata[0]=='s')
		{
			SendSony(0x20DF, htim1, x1);
			//SendSAM(0xE0E0E01F);
		}
		else if(Rdata[0]=='m')
		{
			SendSAM(0xE0E0D02F, htim1, x1);
			//SendSAM(0xB24D5FA0);
			//SendNEC(0x20DF10EF);
		}
		HAL_UART_Receive_IT(&huart1,Rdata,1);
	}

  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim1.Init.Prescaler = 13;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 14;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/************* Roozbeh & Nila Code *************************/




//void SendHigh(void)
//{
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, x1);
//	uDelay(HighTimeNEC);
//	
//}

//void SendOne(void)
//{
//	SendHigh();
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//	uDelay(OneLowTimeNEC);
//}

//void SendZero(void)
//{
//	SendHigh();
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//	uDelay(ZeroLowTimeNEC);
//}

//void SendStartNEC(void)
//{
//	// 9 ms High
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, x1);
//	uDelay(StartTime1NEC);
//	// 4.5 ms Low
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//	uDelay(StartTime2NEC);
//}

//void SendDataNEC(const uint32_t data)
//{
//	uint32_t mask = mask32;
//	
//	for(uint16_t i=0; i<32; i++)
//	{
//		if (data & mask)
//			SendOne();
//		else 
//			SendZero();
//		mask = mask >> 1;
//	}
//	
//	
//}

//void SendEndNEC(void)
//{
//	SendHigh();
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//}

//void SendNEC(const uint32_t data )
//{
//	SendStartNEC();
//	SendDataNEC(data);
//	SendEndNEC();
//}

//void uDelay(const uint16_t d)
//{
//	TIM3->CNT = 0;
//	while(TIM3->CNT < d);
//}

/***************** Sony ******************/
//void SendSony(const uint16_t data)
//{
//	SendStartSony();
//	SendDataSony(data);
//}

//void SendStartSony(void)
//{
//	// 9 ms High
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, x1);
//	uDelay(StartTime1Sony);
//	// 4.5 ms Low
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//	uDelay(StartTime2Sony);
//}

//void SendDataSony(const uint16_t data)
//{
//	uint16_t mask = mask16;
//	
//	for(uint16_t i=0; i<16; i++)
//	{
//		if (data & mask)
//			SendOneSony();
//		else 
//			SendZeroSony();
//		mask = mask >> 1;
//	}
//}

//void SendOneSony(void)
//{
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, x1);
//	uDelay(OneHighTimeSony);
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//	uDelay(LowTimeSony);
//}

//void SendZeroSony(void)
//{
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, x1);
//	uDelay(ZeroHighTimeSony);
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//	uDelay(LowTimeSony);
//}

/**************SAMSUNG***********/
//void SendHighSAM(void)
//{
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, x1);
//	uDelay(HighTimeSAM);
//	
//}

//void SendOneSAM(void)
//{
//	SendHighSAM();
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//	uDelay(OneLowTimeSAM);
//}

//void SendZeroSAM(void)
//{
//	SendHighSAM();
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//	uDelay(ZeroLowTimeSAM);
//}

//void SendStartSAM(void)
//{
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, x1);
//	uDelay(StartTime1SAM);
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//	uDelay(StartTime2SAM);
//}

//void SendDataSAM(const uint32_t data)
//{
//	uint32_t mask = mask32;
//	
//	for(uint16_t i=0; i<32; i++)
//	{
//		if (data & mask)
//			SendOneSAM();
//		else 
//			SendZeroSAM();
//		mask = mask >> 1;
//	}
//}


//void SendSAM(const uint32_t data )
//{
//	SendStartSAM();
//	SendDataSAM(data);
//	SendEndSAM();
//}

//void SendEndSAM(void)
//{
//	SendZeroSAM();
//}

/**********Panasonic************/

void SendHighPAN(void)
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, x1);
	uDelay(HighTimePanasonic);
	
}

void SendOnePAN(void)
{
	SendHighPAN();
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	uDelay(OneLowTimePanasonic);
}

void SendZeroPAN(void)
{
	SendHighPAN();
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	uDelay(ZeroLowTimePanasonic);
}

void SendStartPAN(void)
{
	// 9 ms High
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, x1);
	uDelay(StartTime1Panasonic);
	// 4.5 ms Low
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	uDelay(StartTime2Panasonic);
}

void SendDataPAN(const uint32_t data)
{
	uint32_t mask = mask32;
	
	for(uint16_t i=0; i<32; i++)
	{
		if (data & mask)
			SendOnePAN();
		else 
			SendZeroPAN();
		mask = mask >> 1;
	}
	
	
}

void SendEndPAN(void)
{
	SendHighPAN();
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
}

void SendPAN(const uint32_t data )
{
	SendStartPAN();
	SendDataPAN(data);
	SendEndPAN();
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
