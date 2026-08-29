/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
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
#include "usbd_cdc_interface.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* IWDG key values (RM0008 section 21.3.2, IWDG_KR register) */
#define IWDG_KEY_UNLOCK  0x5555U   /* enable write access to PR/RLR */
#define IWDG_KEY_RELOAD  0xAAAAU   /* reload the counter (feed dog) */
#define IWDG_KEY_START   0xCCCCU   /* start the watchdog */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_IWDG_Init(void);
static void MX_USART3_UART_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* Freeze IWDG while the debugger halts the core (otherwise the watchdog
   * resets the chip during breakpoints). */
  __HAL_DBGMCU_FREEZE_IWDG();

  /* Start the independent watchdog (~2 s timeout) BEFORE enabling PWM, so a
   * hang anywhere in the remaining init is caught. Must be refreshed in the
   * main loop; a runaway firmware resets the MCU and the arm stops. */
  MX_IWDG_Init();

  /* PWM channels are NOT started here. Each channel is started lazily by
   * Servo_SetAngle on its first M command, so the PC-side soft start
   * (M1..M6, 200 ms apart) genuinely sequences the inrush: at boot the
   * servo rail draws 0 A instead of ~15 A from six servos snapping to
   * neutral simultaneously. */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* Process incoming USART3 commands */
    CDC_ProcessRx();
    /* Gradual servo ramp (H command smooth movement). */
    Servo_RampStep();
    /* No-command timeout: hold position if the PC went silent. */
    CDC_TimeoutCheck();
    /* Feed the watchdog. */
    IWDG->KR = IWDG_KEY_RELOAD;
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USB clock config removed — project uses USART3 + CH340. */
}

/**
  * @brief TIM1 Initialization Function (servos 1-3 on PA8/PA9/PA10)
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = SERVO_HOME_PULSE;  /* 1166 us = 90 deg on 270-deg servos */
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim1);
}

/**
  * @brief TIM2 Initialization Function (servos 4-6 on PA0/PA1/PA2)
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = SERVO_HOME_PULSE;  /* 1166 us = 90 deg on 270-deg servos */
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim2);
}

/**
  * @brief IWDG Initialization Function (~2 s timeout), register level.
  *        LSI ~40 kHz / 128 = 312.5 Hz -> 625 ticks = 2.0 s.
  *        Note: LSI is 30..60 kHz, so the real timeout is 1.3..2.7 s.
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{
  /* Unlock write access to PR/RLR */
  IWDG->KR = IWDG_KEY_UNLOCK;
  /* Prescaler 128 (IWDG_PR_PR_2 = 0x4) */
  IWDG->PR = IWDG_PR_PR_2;
  /* Reload value 625 -> ~2.0 s */
  IWDG->RLR = 625U;
  /* Start the watchdog */
  IWDG->KR = IWDG_KEY_START;
}

/**
  * @brief GPIO Initialization Function (IR sensors on PB12/PB13)
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* IR sensor inputs (active low) */
  GPIO_InitStruct.Pin = IR1_GPIO_PIN | IR2_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
  * @brief USART3 Initialization (serial command bridge) - register level.
  *        PB10 = TX (AF_PP), PB11 = RX (input float), 115200 8N1.
  *        PCLK1 = 36 MHz -> USARTDIV = 36e6/(16*115200) = 19.53 -> BRR = 0x138.
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* USART3 GPIO: PB10 = TX, PB11 = RX */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USART3 clock + 115200 8N1 + RX interrupt */
  __HAL_RCC_USART3_CLK_ENABLE();
  USART3->BRR = 0x138U;
  USART3->CR2 = 0U;
  USART3->CR3 = 0U;
  USART3->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;

  /* NVIC */
  HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
}

/* USER CODE BEGIN 4 */

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

#ifdef USE_FULL_ASSERT
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
