/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define GM6020_CONTROL_PERIOD_MS          5U
#define GM6020_FEEDBACK_TIMEOUT_MS        100U
#define GM6020_SPEED_TEST_START_DELAY_MS  3000U
#define GM6020_SPEED_TEST_DURATION_MS     5000U
#define GM6020_SPEED_TARGET_RPM           10
#define GM6020_SPEED_KP                   100.0f
#define GM6020_CONTROL_OUTPUT_LIMIT       1500

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

FDCAN_HandleTypeDef hfdcan2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

typedef enum
{
  GM6020_CONTROL_WAITING = 0,
  GM6020_CONTROL_RUNNING,
  GM6020_CONTROL_COMPLETE,
  GM6020_CONTROL_FEEDBACK_FAULT,
  GM6020_CONTROL_TX_FAULT
} GM6020_ControlState;

/* GM6020 raw feedback values for CubeIDE Live Expressions / Watch. */
volatile uint8_t gm6020_rx_data[8] = {0};
volatile uint32_t gm6020_rx_count = 0;
volatile uint32_t gm6020_last_rx_tick = 0;
volatile uint32_t fdcan2_last_rx_id = 0;
volatile uint32_t fdcan2_rx_error_count = 0;
volatile uint32_t fdcan2_tx_error_count = 0;

static uint32_t vofa_last_tx_tick = 0;
static char vofa_tx_buffer[64];
static GM6020_ControlState gm6020_control_state = GM6020_CONTROL_WAITING;
static uint32_t gm6020_last_control_tick = 0;
static uint32_t gm6020_speed_test_start_tick = 0;
static int16_t gm6020_speed_target_rpm = 0;
static int16_t gm6020_speed_feedback_rpm = 0;
static int16_t gm6020_speed_error_rpm = 0;
static int16_t gm6020_control_output = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
HAL_StatusTypeDef GM6020_SendCurrent(int16_t iq1,
                                    int16_t iq2,
                                    int16_t iq3,
                                    int16_t iq4);
int16_t GM6020_LimitOutput(int32_t output);
void GM6020_RunSpeedPControl(uint32_t now);
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_FDCAN2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  FDCAN_FilterTypeDef sFilter;

  sFilter.IdType = FDCAN_STANDARD_ID;
  sFilter.FilterIndex = 0;
  sFilter.FilterType = FDCAN_FILTER_MASK;
  sFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;

  /* 接收所有标准ID */
  sFilter.FilterID1 = 0x000;
  sFilter.FilterID2 = 0x000;

  if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilter) != HAL_OK)
  {
      Error_Handler();
  }
  if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
  {
      Error_Handler();
  }
  if (HAL_FDCAN_ActivateNotification(
          &hfdcan2,
          FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
          0) != HAL_OK)
  {
      Error_Handler();
  }

  gm6020_last_control_tick = HAL_GetTick();
  vofa_last_tx_tick = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uint32_t now;

	  now = HAL_GetTick();

	  if ((now - gm6020_last_control_tick) >= GM6020_CONTROL_PERIOD_MS)
	  {
		  gm6020_last_control_tick += GM6020_CONTROL_PERIOD_MS;

		  if ((now - gm6020_last_control_tick) >= GM6020_CONTROL_PERIOD_MS)
		  {
			  gm6020_last_control_tick = now;
		  }

		  GM6020_RunSpeedPControl(now);
	  }

	  if ((now - vofa_last_tx_tick) >= 50U)
	  {
		  int tx_length;

		  vofa_last_tx_tick = now;

		  tx_length = snprintf(vofa_tx_buffer,
							 sizeof(vofa_tx_buffer),
							 "speed_p:%d,%d,%d,%d,%u\r\n",
							 (int)gm6020_speed_target_rpm,
							 (int)gm6020_speed_feedback_rpm,
							 (int)gm6020_speed_error_rpm,
							 (int)gm6020_control_output,
							 (unsigned int)gm6020_control_state);

		  if ((tx_length > 0) &&
			  ((size_t)tx_length < sizeof(vofa_tx_buffer)))
		  {
			  (void)HAL_UART_Transmit(&huart1,
							 (uint8_t *)vofa_tx_buffer,
							 (uint16_t)tx_length,
							 20U);
		  }
	  }
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 32;
  RCC_OscInitStruct.PLL.PLLN = 120;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 6;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = ENABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 4;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 6;
  hfdcan2.Init.NominalTimeSeg2 = 3;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 1;
  hfdcan2.Init.DataTimeSeg2 = 1;
  hfdcan2.Init.MessageRAMOffset = 0;
  hfdcan2.Init.StdFiltersNbr = 1;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.RxFifo0ElmtsNbr = 8;
  hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxFifo1ElmtsNbr = 0;
  hfdcan2.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxBuffersNbr = 0;
  hfdcan2.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.TxEventsNbr = 0;
  hfdcan2.Init.TxBuffersNbr = 0;
  hfdcan2.Init.TxFifoQueueElmtsNbr = 8;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan2.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    uint32_t i;

    if ((hfdcan->Instance != FDCAN2) ||
        ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U))
    {
        return;
    }

    if (HAL_FDCAN_GetRxMessage(hfdcan,
                               FDCAN_RX_FIFO0,
                               &rx_header,
                               rx_data) != HAL_OK)
    {
        fdcan2_rx_error_count++;
        return;
    }

    fdcan2_last_rx_id = rx_header.Identifier;

    if ((rx_header.IdType == FDCAN_STANDARD_ID) &&
        (rx_header.Identifier == 0x205U))
    {
        for (i = 0; i < 8U; i++)
        {
            gm6020_rx_data[i] = rx_data[i];
        }

        gm6020_rx_count++;
        gm6020_last_rx_tick = HAL_GetTick();
    }
}

void GM6020_RunSpeedPControl(uint32_t now)
{
    uint8_t feedback[8];
    uint32_t feedback_count;
    uint32_t feedback_tick;
    uint8_t feedback_fresh;
    int32_t p_output;
    uint32_t i;

    __disable_irq();
    for (i = 0; i < 8U; i++)
    {
        feedback[i] = gm6020_rx_data[i];
    }
    feedback_count = gm6020_rx_count;
    feedback_tick = gm6020_last_rx_tick;
    __enable_irq();

    gm6020_speed_feedback_rpm =
        (int16_t)(((uint16_t)feedback[2] << 8) | feedback[3]);
    feedback_fresh = ((feedback_count > 0U) &&
                      ((now - feedback_tick) <= GM6020_FEEDBACK_TIMEOUT_MS)) ? 1U : 0U;

    gm6020_speed_target_rpm = 0;
    gm6020_speed_error_rpm =
        gm6020_speed_target_rpm - gm6020_speed_feedback_rpm;
    gm6020_control_output = 0;

    switch (gm6020_control_state)
    {
        case GM6020_CONTROL_WAITING:
            if ((now >= GM6020_SPEED_TEST_START_DELAY_MS) &&
                (feedback_fresh != 0U))
            {
                gm6020_control_state = GM6020_CONTROL_RUNNING;
                gm6020_speed_test_start_tick = now;
            }
            break;

        case GM6020_CONTROL_RUNNING:
            if (feedback_fresh == 0U)
            {
                gm6020_control_state = GM6020_CONTROL_FEEDBACK_FAULT;
            }
            else if ((now - gm6020_speed_test_start_tick) >=
                     GM6020_SPEED_TEST_DURATION_MS)
            {
                gm6020_control_state = GM6020_CONTROL_COMPLETE;
            }
            else
            {
                gm6020_speed_target_rpm = GM6020_SPEED_TARGET_RPM;
                gm6020_speed_error_rpm =
                    gm6020_speed_target_rpm - gm6020_speed_feedback_rpm;
                p_output = (int32_t)(GM6020_SPEED_KP *
                                     (float)gm6020_speed_error_rpm);
                gm6020_control_output = GM6020_LimitOutput(p_output);
            }
            break;

        case GM6020_CONTROL_COMPLETE:
        case GM6020_CONTROL_FEEDBACK_FAULT:
        case GM6020_CONTROL_TX_FAULT:
        default:
            break;
    }

    if (GM6020_SendCurrent(gm6020_control_output, 0, 0, 0) != HAL_OK)
    {
        gm6020_control_output = 0;

        if (gm6020_control_state == GM6020_CONTROL_RUNNING)
        {
            gm6020_control_state = GM6020_CONTROL_TX_FAULT;
        }
    }
}

int16_t GM6020_LimitOutput(int32_t output)
{
    if (output > GM6020_CONTROL_OUTPUT_LIMIT)
    {
        output = GM6020_CONTROL_OUTPUT_LIMIT;
    }
    else if (output < -GM6020_CONTROL_OUTPUT_LIMIT)
    {
        output = -GM6020_CONTROL_OUTPUT_LIMIT;
    }

    return (int16_t)output;
}

HAL_StatusTypeDef GM6020_SendCurrent(int16_t iq1,
                                    int16_t iq2,
                                    int16_t iq3,
                                    int16_t iq4)
{
    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    HAL_StatusTypeDef status;

    TxHeader.Identifier = 0x1FF;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    TxData[0] = iq1 >> 8;
    TxData[1] = iq1;

    TxData[2] = iq2 >> 8;
    TxData[3] = iq2;

    TxData[4] = iq3 >> 8;
    TxData[5] = iq3;

    TxData[6] = iq4 >> 8;
    TxData[7] = iq4;

    status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2,
                                          &TxHeader,
                                          TxData);
    if (status != HAL_OK)
    {
        fdcan2_tx_error_count++;
    }

    return status;
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
