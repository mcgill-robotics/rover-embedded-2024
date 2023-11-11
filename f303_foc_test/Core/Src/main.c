/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define _HIGH_IMPEDANCE 0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
/**
 * https://github.com/simplefoc/Arduino-FOC/blob/master/src/BLDCMotor.cpp
 */
int trap_120_map[6][3] = {
    {_HIGH_IMPEDANCE, 1, -1},
    {-1, 1, _HIGH_IMPEDANCE},
    {-1, _HIGH_IMPEDANCE, 1},
    {_HIGH_IMPEDANCE, -1, 1},
    {1, -1, _HIGH_IMPEDANCE},
    {1, _HIGH_IMPEDANCE, -1}};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct hal_gpio_pin
{
  GPIO_TypeDef *port;
  uint16_t pin;
} hal_gpio_pin_t;
typedef struct motor_phase
{
  hal_gpio_pin_t pin_high;
  hal_gpio_pin_t pin_low;
} motor_phase_t;

/**
 * @brief Commutate a motor phase, L6398 style
 */
void commutate_phase(motor_phase_t motor_phase, int state)
{
  switch (state)
  {
  case 1:
    HAL_GPIO_WritePin(motor_phase.pin_high.port, motor_phase.pin_high.pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor_phase.pin_low.port, motor_phase.pin_low.pin, GPIO_PIN_SET);
    break;
  case -1:
    HAL_GPIO_WritePin(motor_phase.pin_high.port, motor_phase.pin_high.pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor_phase.pin_low.port, motor_phase.pin_low.pin, GPIO_PIN_RESET);
    break;
  case _HIGH_IMPEDANCE:
    HAL_GPIO_WritePin(motor_phase.pin_high.port, motor_phase.pin_high.pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor_phase.pin_low.port, motor_phase.pin_low.pin, GPIO_PIN_SET);
    break;
  }
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */
  motor_phase_t phase_A = {
      .pin_high = {.port = GPIOA, .pin = GPIO_PIN_8},
      .pin_low = {.port = GPIOA, .pin = GPIO_PIN_11}};
  motor_phase_t phase_B = {
      .pin_high = {.port = GPIOA, .pin = GPIO_PIN_9},
      .pin_low = {.port = GPIOA, .pin = GPIO_PIN_12}};
  motor_phase_t phase_C = {
      .pin_high = {.port = GPIOA, .pin = GPIO_PIN_10},
      .pin_low = {.port = GPIOB, .pin = GPIO_PIN_1}};
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
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    //    HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, 1);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 800);
    HAL_Delay(2000);
    HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, 0);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 400);
    HAL_Delay(2000);
    //    for (int i = 0; i < 6; i++)
    //    {
    //      commutate_phase(phase_A, trap_120_map[i][0]);
    //      commutate_phase(phase_B, trap_120_map[i][1]);
    //      commutate_phase(phase_C, trap_120_map[i][2]);
    //      HAL_Delay(500);
    //    }

    // HOT 5Amps
    //    commutate_phase(phase_A, 1);
    //    commutate_phase(phase_B, -1);
    //    commutate_phase(phase_C, 0);

    // HOT 5Amps
    //    commutate_phase(phase_A, 0);
    //    commutate_phase(phase_B, 1);
    //    commutate_phase(phase_C, -1);

    // HOT 5Amps
    // commutate_phase(phase_A, 1);
    // commutate_phase(phase_B, 0);
    // commutate_phase(phase_C, -1);

    // MANUAL test
    // HAL_GPIO_WritePin(A_H_GPIO_Port, A_H_Pin, GPIO_PIN_SET);
    // HAL_GPIO_WritePin(A_L_GPIO_Port, A_L_Pin, GPIO_PIN_SET);
    // HAL_Delay(1000);

    // No reaction
    // HAL_GPIO_WritePin(A_H_GPIO_Port, A_H_Pin, GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(A_L_GPIO_Port, A_L_Pin, GPIO_PIN_RESET);

    // HOT 5AMP
    // HAL_GPIO_WritePin(A_H_GPIO_Port, A_H_Pin, GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(A_L_GPIO_Port, A_L_Pin, GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(A_H_GPIO_Port, B_H_Pin, GPIO_PIN_SET);
    // HAL_GPIO_WritePin(A_L_GPIO_Port, B_L_Pin, GPIO_PIN_SET);
    // HAL_Delay(1000);

    // HAL_GPIO_WritePin(A_H_GPIO_Port, A_H_Pin, GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(A_L_GPIO_Port, A_L_Pin, GPIO_PIN_RESET);
    // HAL_Delay(1000);
    // HAL_GPIO_WritePin(A_H_GPIO_Port, A_H_Pin, GPIO_PIN_SET);
    // HAL_GPIO_WritePin(A_L_GPIO_Port, A_L_Pin, GPIO_PIN_SET);
    // HAL_Delay(1000);

    // HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
    // HAL_Delay(500);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 8 - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
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
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
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
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, C_L_Pin | RED_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, B_H_Pin | C_H_Pin | B_L_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : C_L_Pin RED_LED_Pin */
  GPIO_InitStruct.Pin = C_L_Pin | RED_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : B_H_Pin C_H_Pin B_L_Pin */
  GPIO_InitStruct.Pin = B_H_Pin | C_H_Pin | B_L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
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
