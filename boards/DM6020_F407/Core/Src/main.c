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
#define GM6020_SPEED_KI                   40.0f
#define GM6020_SPEED_INTEGRAL_LIMIT       1000.0f
#define GM6020_CONTROL_OUTPUT_LIMIT       1500
#define CAN1_LOOPBACK_TEST_ID              0x321U
#define CAN1_LOOPBACK_TEST_TIMEOUT_MS      20U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

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

/* GM6020 raw feedback and CAN diagnostics for Live Expressions / Watch. */
volatile uint8_t gm6020_rx_data[8] = {0};
volatile uint32_t gm6020_rx_count = 0;
volatile uint32_t gm6020_last_rx_tick = 0;
volatile uint32_t can1_last_rx_id = 0;
volatile uint32_t can1_rx_error_count = 0;
volatile uint32_t can1_tx_error_count = 0;
volatile uint8_t can1_loopback_test_pass = 0;
volatile uint32_t can1_loopback_test_error = 0;
volatile uint32_t can1_esr_snapshot = 0;
volatile uint32_t can1_tsr_snapshot = 0;

static uint32_t vofa_last_tx_tick = 0;
static char vofa_tx_buffer[64];
static GM6020_ControlState gm6020_control_state = GM6020_CONTROL_WAITING;
static uint32_t gm6020_last_control_tick = 0;
static uint32_t gm6020_speed_test_start_tick = 0;
static int16_t gm6020_speed_target_rpm = 0;
static int16_t gm6020_speed_feedback_rpm = 0;
static int16_t gm6020_speed_error_rpm = 0;
static int32_t gm6020_speed_p_output = 0;
static int32_t gm6020_speed_i_output = 0;
static float gm6020_speed_integral = 0.0f;
static int16_t gm6020_control_output = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

HAL_StatusTypeDef GM6020_SendCurrent(int16_t iq1,
                                    int16_t iq2,
                                    int16_t iq3,
                                    int16_t iq4);
int16_t GM6020_LimitOutput(int32_t output);
void GM6020_ResetSpeedPI(void);
void GM6020_RunSpeedPIControl(uint32_t now);
static HAL_StatusTypeDef CAN1_ConfigAcceptAllFilter(void);
static uint8_t CAN1_RunLoopbackSelfTest(void);

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
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* Verify the bxCAN core and bit-timing path without using the external
     transceiver, then automatically restore normal bus operation. */
  can1_loopback_test_pass = CAN1_RunLoopbackSelfTest();

  if (CAN1_ConfigAcceptAllFilter() != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_CAN_ActivateNotification(&hcan1,
                                   CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
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
    can1_esr_snapshot = CAN1->ESR;
    can1_tsr_snapshot = CAN1->TSR;

    if ((now - gm6020_last_control_tick) >= GM6020_CONTROL_PERIOD_MS)
    {
      gm6020_last_control_tick += GM6020_CONTROL_PERIOD_MS;

      if ((now - gm6020_last_control_tick) >= GM6020_CONTROL_PERIOD_MS)
      {
        gm6020_last_control_tick = now;
      }

      GM6020_RunSpeedPIControl(now);
    }

    if ((now - vofa_last_tx_tick) >= 50U)
    {
      int tx_length;

      vofa_last_tx_tick = now;

      tx_length = snprintf(vofa_tx_buffer,
                           sizeof(vofa_tx_buffer),
                           "speed_pi:%d,%d,%d,%ld,%ld,%d\r\n",
                           (int)gm6020_speed_target_rpm,
                           (int)gm6020_speed_feedback_rpm,
                           (int)gm6020_speed_error_rpm,
                           (long)gm6020_speed_p_output,
                           (long)gm6020_speed_i_output,
                           (int)gm6020_control_output);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static HAL_StatusTypeDef CAN1_ConfigAcceptAllFilter(void)
{
  CAN_FilterTypeDef filter = {0};

  /* Keep the original behavior: accept all CAN frames into FIFO0, then
     select GM6020 feedback ID 0x205 in the receive callback. */
  filter.FilterBank = 0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0;
  filter.FilterIdLow = 0;
  filter.FilterMaskIdHigh = 0;
  filter.FilterMaskIdLow = 0;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14;

  return HAL_CAN_ConfigFilter(&hcan1, &filter);
}

static uint8_t CAN1_RunLoopbackSelfTest(void)
{
  CAN_TxHeaderTypeDef tx_header = {0};
  CAN_RxHeaderTypeDef rx_header;
  uint8_t tx_data[8] = {0xA5U, 0x5AU, 0x11U, 0x22U,
                        0x33U, 0x44U, 0x55U, 0x66U};
  uint8_t rx_data[8] = {0};
  uint32_t tx_mailbox;
  uint32_t start_tick;
  uint32_t i;
  uint8_t passed = 0U;

  can1_loopback_test_error = 0U;

  if (HAL_CAN_DeInit(&hcan1) != HAL_OK)
  {
    can1_loopback_test_error = 1U;
    goto restore_normal_mode;
  }

  hcan1.Init.Mode = CAN_MODE_LOOPBACK;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    can1_loopback_test_error = 2U;
    goto restore_normal_mode;
  }
  if (CAN1_ConfigAcceptAllFilter() != HAL_OK)
  {
    can1_loopback_test_error = 3U;
    goto restore_normal_mode;
  }
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    can1_loopback_test_error = 4U;
    goto restore_normal_mode;
  }

  tx_header.StdId = CAN1_LOOPBACK_TEST_ID;
  tx_header.ExtId = 0U;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = 8U;
  tx_header.TransmitGlobalTime = DISABLE;

  if (HAL_CAN_AddTxMessage(&hcan1,
                           &tx_header,
                           tx_data,
                           &tx_mailbox) != HAL_OK)
  {
    can1_loopback_test_error = 5U;
    goto restore_normal_mode;
  }

  start_tick = HAL_GetTick();
  while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) == 0U)
  {
    if ((HAL_GetTick() - start_tick) >= CAN1_LOOPBACK_TEST_TIMEOUT_MS)
    {
      can1_loopback_test_error = 6U;
      goto restore_normal_mode;
    }
  }

  if (HAL_CAN_GetRxMessage(&hcan1,
                           CAN_RX_FIFO0,
                           &rx_header,
                           rx_data) != HAL_OK)
  {
    can1_loopback_test_error = 7U;
    goto restore_normal_mode;
  }

  if ((rx_header.IDE != CAN_ID_STD) ||
      (rx_header.RTR != CAN_RTR_DATA) ||
      (rx_header.StdId != CAN1_LOOPBACK_TEST_ID) ||
      (rx_header.DLC != 8U))
  {
    can1_loopback_test_error = 8U;
    goto restore_normal_mode;
  }

  for (i = 0; i < 8U; i++)
  {
    if (rx_data[i] != tx_data[i])
    {
      can1_loopback_test_error = 9U;
      goto restore_normal_mode;
    }
  }

  passed = 1U;

restore_normal_mode:
  (void)HAL_CAN_Stop(&hcan1);
  (void)HAL_CAN_DeInit(&hcan1);
  hcan1.Init.Mode = CAN_MODE_NORMAL;

  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    can1_loopback_test_error = 10U;
    return 0U;
  }

  return passed;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];
  uint32_t i;

  if (hcan->Instance != CAN1)
  {
    return;
  }

  if (HAL_CAN_GetRxMessage(hcan,
                           CAN_RX_FIFO0,
                           &rx_header,
                           rx_data) != HAL_OK)
  {
    can1_rx_error_count++;
    return;
  }

  if (rx_header.IDE == CAN_ID_STD)
  {
    can1_last_rx_id = rx_header.StdId;
  }

  if ((rx_header.IDE == CAN_ID_STD) &&
      (rx_header.RTR == CAN_RTR_DATA) &&
      (rx_header.StdId == 0x205U) &&
      (rx_header.DLC == 8U))
  {
    for (i = 0; i < 8U; i++)
    {
      gm6020_rx_data[i] = rx_data[i];
    }

    gm6020_rx_count++;
    gm6020_last_rx_tick = HAL_GetTick();
  }
}

void GM6020_ResetSpeedPI(void)
{
  gm6020_speed_integral = 0.0f;
  gm6020_speed_i_output = 0;
}

void GM6020_RunSpeedPIControl(uint32_t now)
{
  uint8_t feedback[8];
  uint32_t feedback_count;
  uint32_t feedback_tick;
  uint8_t feedback_fresh;
  float integral_candidate;
  float unsaturated_output;
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
  gm6020_speed_p_output = 0;
  gm6020_control_output = 0;

  switch (gm6020_control_state)
  {
    case GM6020_CONTROL_WAITING:
      GM6020_ResetSpeedPI();

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
        GM6020_ResetSpeedPI();
      }
      else if ((now - gm6020_speed_test_start_tick) >=
               GM6020_SPEED_TEST_DURATION_MS)
      {
        gm6020_control_state = GM6020_CONTROL_COMPLETE;
        GM6020_ResetSpeedPI();
      }
      else
      {
        gm6020_speed_target_rpm = GM6020_SPEED_TARGET_RPM;
        gm6020_speed_error_rpm =
            gm6020_speed_target_rpm - gm6020_speed_feedback_rpm;
        gm6020_speed_p_output =
            (int32_t)(GM6020_SPEED_KP *
                      (float)gm6020_speed_error_rpm);

        integral_candidate =
            gm6020_speed_integral +
            (GM6020_SPEED_KI *
             (float)gm6020_speed_error_rpm *
             ((float)GM6020_CONTROL_PERIOD_MS / 1000.0f));

        if (integral_candidate > GM6020_SPEED_INTEGRAL_LIMIT)
        {
          integral_candidate = GM6020_SPEED_INTEGRAL_LIMIT;
        }
        else if (integral_candidate < -GM6020_SPEED_INTEGRAL_LIMIT)
        {
          integral_candidate = -GM6020_SPEED_INTEGRAL_LIMIT;
        }

        unsaturated_output =
            (float)gm6020_speed_p_output + integral_candidate;

        /* Do not integrate farther into output saturation, but allow the
           integral term to unwind when the error changes direction. */
        if (!(((unsaturated_output >
                (float)GM6020_CONTROL_OUTPUT_LIMIT) &&
               (gm6020_speed_error_rpm > 0)) ||
              ((unsaturated_output <
                -(float)GM6020_CONTROL_OUTPUT_LIMIT) &&
               (gm6020_speed_error_rpm < 0))))
        {
          gm6020_speed_integral = integral_candidate;
        }

        gm6020_speed_i_output = (int32_t)gm6020_speed_integral;
        gm6020_control_output = GM6020_LimitOutput(
            gm6020_speed_p_output + gm6020_speed_i_output);
      }
      break;

    case GM6020_CONTROL_COMPLETE:
    case GM6020_CONTROL_FEEDBACK_FAULT:
    case GM6020_CONTROL_TX_FAULT:
    default:
      GM6020_ResetSpeedPI();
      break;
  }

  if (GM6020_SendCurrent(gm6020_control_output, 0, 0, 0) != HAL_OK)
  {
    gm6020_control_output = 0;
    GM6020_ResetSpeedPI();

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
  CAN_TxHeaderTypeDef tx_header = {0};
  uint8_t tx_data[8];
  uint32_t tx_mailbox;
  HAL_StatusTypeDef status;

  tx_header.StdId = 0x1FFU;
  tx_header.ExtId = 0U;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = 8U;
  tx_header.TransmitGlobalTime = DISABLE;

  tx_data[0] = (uint8_t)((uint16_t)iq1 >> 8);
  tx_data[1] = (uint8_t)iq1;
  tx_data[2] = (uint8_t)((uint16_t)iq2 >> 8);
  tx_data[3] = (uint8_t)iq2;
  tx_data[4] = (uint8_t)((uint16_t)iq3 >> 8);
  tx_data[5] = (uint8_t)iq3;
  tx_data[6] = (uint8_t)((uint16_t)iq4 >> 8);
  tx_data[7] = (uint8_t)iq4;

  status = HAL_CAN_AddTxMessage(&hcan1,
                                &tx_header,
                                tx_data,
                                &tx_mailbox);
  if (status != HAL_OK)
  {
    can1_tx_error_count++;
  }

  return status;
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
