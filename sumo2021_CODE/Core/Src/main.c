/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#define speed_fast 100
#define speed_slow 60
#define speed_backward 100
#define speed_turn 60
#define speed_turn_1_wheel 100

#define L_positive 1 //do line
#define L_negative 0

#define D_positive 1 //do duong
#define D_negative 0

#define B_positive 1 //button
#define B_negative 0
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t d[3] = {1}; //CAM BIEN DO LINE, TICH CUC CAO
uint8_t u[5] = {1}; //CAM BIEN DO DUONG, TICH CUC CAO
uint8_t i;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//do duong
//			---2-3---
//			1       4
//      ----0----

//do line
//			1-------2
//			|       |
//      0-------0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
											//CODE DI CHUYEN
void stop()
{
	htim3.Instance->CCR1 = 0;
	htim3.Instance->CCR2 = 0;
	htim4.Instance->CCR1 = 0;
	htim4.Instance->CCR2 = 0;
}
void fast_forward()
{
	htim3.Instance->CCR2 = 100;
	htim3.Instance->CCR1 = 0;
	htim4.Instance->CCR1 = 92;
	htim4.Instance->CCR2 = 0;
}

void slow_forward()
{
	htim3.Instance->CCR2 = 80;
	htim3.Instance->CCR1 = 0;
	htim4.Instance->CCR1 = 50;
	htim4.Instance->CCR2 = 0;
}

void backward()
{
	htim3.Instance->CCR1 = 70;
	htim3.Instance->CCR2 = 0;
	htim4.Instance->CCR2 = 60;
	htim4.Instance->CCR1 = 0;
}

void turn_R()
{
	htim3.Instance->CCR1 = 70;
	htim3.Instance->CCR2 = 0;
	htim4.Instance->CCR1 = 60;
	htim4.Instance->CCR2 = 0;
}

void turn_L()
{
	htim3.Instance->CCR2 = 70;
	htim3.Instance->CCR1 = 0;
	htim4.Instance->CCR2 = 60;
	htim4.Instance->CCR1 = 0;
}

void turn_R_1()
{
	htim3.Instance->CCR1 = 100;
	htim3.Instance->CCR2 = 0;
	htim4.Instance->CCR1 = 50;
	htim4.Instance->CCR2 = 0;
}

void turn_L_1()
{
	htim3.Instance->CCR2 = 90;
	htim3.Instance->CCR1 = 0;
	htim4.Instance->CCR2 = 60;
	htim4.Instance->CCR1 = 0;
}

void turn_L_1_wheel()
{
	htim3.Instance->CCR2 = speed_turn_1_wheel;
	htim3.Instance->CCR1 = 0;
	htim4.Instance->CCR1 = 0;
	htim4.Instance->CCR2 = 0;
}

void turn_R_1_wheel()
{
	htim3.Instance->CCR1 = 0;
	htim3.Instance->CCR2 = 0;
	htim4.Instance->CCR2 = 0;
	htim4.Instance->CCR1 = speed_turn_1_wheel;
}

										//CODE BAT LINE

int catch_line()
{
	for(i=0; i<3; i++)
	{
		if(d[i] == 0)
			return 1;
	}
	return 0;
}

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
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim2); 									//KHOI TAO TIMER
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);				//KHOI TAO CAC CHAN PWM CHO MOTOR DC
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);				//timer 3 : banh phai (2-1)
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);			  //timer 4 : banh trai (1-2)
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//		if(d[0] == 0)
//		{
//			fast_forward();
//		}
		if(d[1] == 0) //queo phai 120 do
		{
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
//			stop();
//			HAL_Delay(10);
			backward();
			HAL_Delay(120);
			turn_R(); 
			HAL_Delay(400);
		}
		if(d[2] == 0) //queo trai 120 do
		{
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
//			stop();
//			HAL_Delay(10);
			backward();
			HAL_Delay(120);
			turn_L(); 
			HAL_Delay(400);
		}
		if(d[1] == 1 && d[2] == 1 && d[0] == 1)
		{
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
			slow_forward();
		}
		if(d[1] == 1 && d[2] == 1 && d[0] == 0)
		{
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
			fast_forward();
		}
		if(!catch_line())
		{
			if((u[2] == 0) && (u[3] == 0))
			{
				fast_forward();
			}
			else
			{
				if(u[1] == 0 && u[2] == 1 && u[3] == 1)
				{
//					while((u[2] == 1) && (u[3] == 1))
//					{
//						turn_L();
//					}
					turn_L();
					HAL_Delay(420);
				}
				if(u[4] == 0 && u[3] == 1 && u[2] == 1)
				{
//					while((u[2] == 1) && (u[3] == 1))
//					{
//						turn_R();
//					}
					turn_R();
					HAL_Delay(400);
				}
				if(u[2] == 0 && u[3] == 1)
				{
					turn_L();
					HAL_Delay(80);
				}
				if(u[2] == 1 && u[3] == 0)
				{
					turn_R();
					HAL_Delay(90);
				}
				if(u[2] == 1 && u[3] == 1 && u[1] == 1 && u[4] == 1)
				{
					slow_forward();
				}
				if(u[0] == 1)
				{
					backward();
					HAL_Delay(200);
				}
			}	
		}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7200;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim3.Init.Prescaler = 8;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 8;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA3 PA4
                           PA9 PA10 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIM_PeriodElapsedCallback could be implemented in the user file
   */
	
	d[0] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);
	d[1] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11);
	d[2] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);
	
	u[0] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
	u[1] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
	u[2] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
	u[3] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
	u[4] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
	
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
