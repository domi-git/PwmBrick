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
#include "uart.h"
#include "term.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Battery-Voltage Measurement: Convert ADC-Digits to Voltage */
#define  DIG_VOLT  657u / 4095u

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
uint32_t state = 0u;					// State of the System

uint32_t tickTmp = 0u;				// Systick-Value from Last Input readings
uint32_t termTick = 0u;       // Number of 100ms ticks between since the last call of CycleTerm

uint32_t chgTick = 0u;				// Number of 100ms ticks the Up-Button is pressed
uint32_t chgModeChange = 0u;	// Remember, if charger mode was changed before

uint32_t upTmp = 0u;					// Last Position of Up-Button
uint32_t downTmp = 0u;				// Last Position of Down-Button

uint32_t flashCnt = 0u;				// Current number of LED-flashes within the blink-code

uint32_t usartEnable = 0u;    // Is USART1 started?
uint32_t usartDelay = 0u;     // Current Delay if USART isn't started yet (in multiples of 100ms)

uint32_t adcVal = 0u;					// Raw ADC-Value for Battery-Voltage (12-Bit)
uint32_t batVolt = 0u;        // Battery-Voltage from ADC in 0.01V
uint32_t recharge = 0u;       // 1=recharge state (Battery Voltage too low), 0=no recharge state


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC_Init(void);
/* USER CODE BEGIN PFP */
static void InitUsart1Tx(void);
static void InitPWM(void);
static void CycleUpBtn(void);
static void CycleDownBtn(void);
static void CycleLedDisplay(void);
static void CycleAdc(void);
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
  MX_ADC_Init();
  /* USER CODE BEGIN 2 */

  /* Initialize ADC-Periphal -> Channel 5 */
  //InitAdc();

  /* Additional Init-Code for the PWM with Deadtime */
  InitPWM();

  /* Start 10V-Regulator (For Mosfet Driver) */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* Infinite loop */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  	/*  Loop every 100ms */
  	if (HAL_GetTick() > tickTmp + 100u)
		{
			tickTmp = HAL_GetTick();

			/* Up-Button Handling */
			CycleUpBtn();

			/* Down-Button handling */
			CycleDownBtn();

			/* ADC-Handling (Get Battery-Voltage) */
			CycleAdc();

			/* Handle State-Changes for Battery-Undervoltage */
			if (batVolt > 0u)  /* Only check, if there has been a battery Voltage Measurement */
			{
				/* Voltage too low? => recharge state */
				if (recharge == 0u && batVolt < 340)
				{
					recharge = 1u;
				}
				/* If recharge state and battery voltage high enough => quit recharge state */
				else if (recharge == 1u && batVolt > 370)
				{
					recharge = 0u;
					state = 0u;
				}
			}

			/* No PWM in recharge state and 10x-Blink-Code */
			if (recharge == 0u)
			{
				/* Handle PWM-Duty */
				TIM1->CCR2 = state*595u;
				TIM1->CCR3 = state*595u;
			}
			else
			{
				TIM1->CCR2 = 0u;
				TIM1->CCR3 = 0u;
				state = 10u;
			}

			/* Enable USART after 10 seconds (no more programming possible, because SWCLK-Pin is used!) */
			if (usartEnable == 0u)
			{
				if (usartDelay < 100u)
				{
					usartDelay++;
				}
				else
				{
					usartEnable = 1u;
					InitUsart1Tx();
					InitUart();
				}
			}
			else
			{
				termTick++;
				if (termTick >= 2u)
				{
					termTick = 0u;
					CycleTerm();
				}
			}

	  	/* Display state as LED-blinking code */
			CycleLedDisplay();

		} /* 100ms */
  } /* while */

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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4000;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Green_Pin|Charge_Enable_Pin|Start_10V_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Green_Pin Charge_Enable_Pin Start_10V_Pin */
  GPIO_InitStruct.Pin = LED_Green_Pin|Charge_Enable_Pin|Start_10V_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_USART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Switch_Plus_Pin Switch_Minus_Pin */
  GPIO_InitStruct.Pin = Switch_Plus_Pin|Switch_Minus_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/*----------------------------------------------------------------------------*/
static void InitUsart1Tx(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_USART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/*----------------------------------------------------------------------------*/
static void InitPWM(void)
{
  /* Komplementären Ausgang von Channel 3 und Channel 2 aktivieren, wird von der Library nicht gemacht! */
  /* Der negative Ausgang von Channel 2 muss auch aktiviert werden, damit die Totzeit dort wirksam wird */
  TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_3, TIM_CCxN_ENABLE);
  TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_2, TIM_CCxN_ENABLE);

  /* Konfiguration der Verriegelungszeit nochmal durchführen. Code-Generierung kann nur Deadtime = 0 generieren... */
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 5;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig);

  /* Start PWM */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
}

/*----------------------------------------------------------------------------*/
static void CycleUpBtn (void)
{
	// Up-Button handling
	if (upTmp == 0u)
	{
		// If Up-Button wasn't pressed before and is pressed now => change upTmp
		if (HAL_GPIO_ReadPin(GPIOA, Switch_Plus_Pin) == GPIO_PIN_RESET)
		{
			upTmp = 1u;
		}
	}
	else  // Button was pressed before
	{
		// If Up-Button was pressed and is'nt pressed now => change state
		if (HAL_GPIO_ReadPin(GPIOA, Switch_Plus_Pin) == GPIO_PIN_SET)
		{
			if (chgModeChange == 0u)  // Only handle state changes when charge mode is not changed
			{
				// If Up-Button is now pressed and wasn't pressed before => next state and remember it is pressed
				upTmp = 0u;
				// Next state and overflow handling
				if (state <= 3u)
				{
					state++;
				}
				else
				{
					state = 4u;
				}
			}
			else
			{
				upTmp = 0u;
				chgModeChange = 0u;
			}
		}
		// Up-Button is still pressed
		else
		{
			chgTick++;
			// If Up-Button is pressed for 2 seconds => Toggle Charge mode
			if (chgTick > 20u)
			{
				chgTick = 0u;
				chgModeChange = 1u;
				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
			}
		}
	}
}

/*----------------------------------------------------------------------------*/
static void CycleDownBtn (void)
{
	if (downTmp == 0u)
	{
		// If Down-Button is now pressed and wasn't pressed before => next state and remember it is pressed
		if (HAL_GPIO_ReadPin(GPIOA, Switch_Minus_Pin) == GPIO_PIN_RESET)
		{
			downTmp = 1u;
			// Next state and overflow handling
			if (state >= 1u)
			{
				state--;
			}
			else
			{
				state = 0u;
			}
		}
	}
	else
	{
		// If Down-Button is not pressed and was pressed before => just remember it isn't pressed
		if (HAL_GPIO_ReadPin(GPIOA, Switch_Plus_Pin) == GPIO_PIN_SET)
		{
			downTmp = 0u;
		}
	}
}

/*----------------------------------------------------------------------------*/
static void CycleLedDisplay (void)
{
	if (state == 0u)
	{
		flashCnt = 0u;
		HAL_GPIO_WritePin(GPIOA, LED_Green_Pin, GPIO_PIN_RESET);
	}
	else
	{
		if (flashCnt >= 100u)  // long off-interval
		{
			flashCnt++;
			if (flashCnt > 105u) { flashCnt = 0u; }
		}
		else  // Blink-code
		{
			flashCnt++;
			HAL_GPIO_TogglePin(GPIOA, LED_Green_Pin);
			if (flashCnt >= 2u*state)
			{
				flashCnt = 100u;  // Start long off-interval
			}
		}
	} // LED-blinking code
}

/*----------------------------------------------------------------------------*/
static void CycleAdc (void)
{
	HAL_ADC_Start(&hadc);
	HAL_ADC_PollForConversion(&hadc, 5u);
	adcVal = HAL_ADC_GetValue(&hadc);
	HAL_ADC_Stop(&hadc);
	batVolt = adcVal * DIG_VOLT;
}

/*----------------------------------------------------------------------------*/

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
