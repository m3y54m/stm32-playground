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
#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADXL345_ADDR 0x53 // Accelerometer
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// 1g offset of earth gravitational acceleration
float accel_g_force_offset = 1.0f;
// Threshold for acceleration which indicates a shock or crash
const float shock_threshold = 4.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
int _write(int fd, char *ptr, int len)
{
  // Redirect printf() to UART Instance
  HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
  return len;
}
void delay_ms(uint32_t ms);
uint64_t millis(void);
void accel_write_reg(uint8_t addr, uint8_t val);
void accel_read_reg(uint8_t addr, uint8_t *read_buffer, uint8_t read_size);
void accel_init(void);
void accel_get_sensor_data(float *values);
void accel_calibrate_g_force(void);
float accel_get_g_force(void);
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  accel_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // uint32_t ms = millis();
    // char buffer[128];
    // sprintf(buffer, "Tick Timer: %ld\r\n", ms);
    // // Sending string to UART1
    // HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), 10);

    float g = accel_get_g_force();
    int q = (int)(g * 10000) / 10000;
    int r = (int)(g * 10000) - q * 10000;
    // printf("G-Force: %.4fg\r\n", g);
    printf("G-Force: %d.%dg\r\n", q, r);

    // // Blink LED_Pin (PC13)
    // HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    // HAL_Delay(1950);
    // HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    // HAL_Delay(50);
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hadc1.Init.ContinuousConvMode = DISABLE;
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
  sConfig.Channel = ADC_CHANNEL_9;
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
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
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
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_RTS;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
void delay_ms(uint32_t ms)
{
  HAL_Delay(ms);
}

uint64_t millis(void)
{
  return (uint64_t)HAL_GetTick();
}

void accel_write_reg(uint8_t addr, uint8_t val)
{
  uint8_t slave_addr = ADXL345_ADDR;
  slave_addr <<= 1;
  HAL_I2C_Mem_Write(&hi2c1, slave_addr, addr, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}

void accel_read_reg(uint8_t addr, uint8_t *read_buffer, uint8_t read_size)
{
  uint8_t slave_addr = ADXL345_ADDR;
  slave_addr <<= 1;
  HAL_I2C_Mem_Read(&hi2c1, slave_addr, addr, I2C_MEMADD_SIZE_8BIT, read_buffer, read_size, 100);
}

void accel_init(void)
{
  // Access to POWER_CTL register - 0x2D
  // Write 0x08 -> 00001000 ( D3=1 for Measuring mode )
  accel_write_reg(0x2D, 0x08);

  // Access to DATA_FORMAT register - 0x31
  // Write 0x0B -> 00001011 ( Bit D3=1 for FULL_RES, D1,D0=11 for Range -> +-16g )
  accel_write_reg(0x31, 0x0B);

  // Access to BW_RATE register - 0x2C
  // Write 0x09 -> 00001001 ( D3,D2,D1,D0=1001 for { Data Rate -> 50Hz , BW -> 25Hz , IDD -> 45uA } )
  accel_write_reg(0x2C, 0x09);

  delay_ms(10);

  // Update `accel_g_force_offset`
  accel_calibrate_g_force();
}

void accel_get_sensor_data(float *values)
{
  int16_t sensor_data[3];
  uint8_t read_buffer[6];

  // Access to DATA registers - 0x32 ~ 0x37 (6 registers)
  // Read 6 registers total, each axis value is stored in 2 registers
  accel_read_reg(0x32, read_buffer, 6);

  sensor_data[0] = (((int16_t)read_buffer[1]) << 8) | read_buffer[0];
  sensor_data[1] = (((int16_t)read_buffer[3]) << 8) | read_buffer[2];
  sensor_data[2] = (((int16_t)read_buffer[5]) << 8) | read_buffer[4];

  // printf("X: %d, Y: %d, Z: %d", sensor_data[0], sensor_data[1], sensor_data[2]);
  // printf("\r\n");

  // For all g-ranges in full resolution mode, sensitivity is 256
  // All values are in g units
  // To convert them to m/s^2 just multiply them by 9.80665
  values[0] = (float)sensor_data[0] / 256; // X-Axis
  values[1] = (float)sensor_data[1] / 256; // Y-Axis
  values[2] = (float)sensor_data[2] / 256; // Z-Axis
}

void accel_calibrate_g_force(void)
{
  float values[3];
  const uint16_t READINGS_COUNT = 100;
  float values_sum[3] = {0.0f};

  // Read and sum-up readings several times to reduce noise
  for (uint8_t i = 0; i < READINGS_COUNT; i++)
  {
    // Get g-force in all 3-axis
    accel_get_sensor_data(values);

    values_sum[0] += values[0];
    values_sum[1] += values[1];
    values_sum[2] += values[2];
  }
  // Calculate the average of all readings
  values[0] = values_sum[0] / READINGS_COUNT;
  values[1] = values_sum[1] / READINGS_COUNT;
  values[2] = values_sum[2] / READINGS_COUNT;

  // Calculate magnitude of g-force and update offset
  accel_g_force_offset = (float)sqrt(pow(values[0], 2) + pow(values[1], 2) + pow(values[2], 2));

  // printf("Magnitude of acceleration at rest: %.4fg\r\n", accel_g_force_offset);
  int q = (int)(accel_g_force_offset * 10000) / 10000;
  int r = (int)(accel_g_force_offset * 10000) - q * 10000;
  printf("Magnitude of acceleration at rest: %d.%dg\r\n", q, r);
}

float accel_get_g_force(void)
{
  float values[3];
  // Get g-force in all 3-axis
  accel_get_sensor_data(values);
  // Calculate magnitude of g-force (removed 1g offset of earth gravitational acceleration)
  return (float)fabs((float)sqrt(pow(values[0], 2) + pow(values[1], 2) + pow(values[2], 2)) - accel_g_force_offset);
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
