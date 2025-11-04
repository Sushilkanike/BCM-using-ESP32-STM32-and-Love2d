/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "i2c-lcd.h"
#include "mpu6050.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "mcp2515.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_ADC_CHANNELS 1

// --- MPU6050 Health Check Defines ---
#define MPU6050_I2C_ADDR (0x68 << 1) // MPU6050 7-bit address, left-shifted for HAL
#define MPU6050_WHO_AM_I_REG 0x75    // WHO_AM_I register address
#define MPU6050_WHO_AM_I_VAL 0x68    // Expected value from WHO_AM_I register

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
// HAL will populate this buffer with ADC results:
uint16_t pot_value;

MPU6050_Data mpu_data;


volatile uint8_t can_buttons[6] = {0, 0, 0, 0, 0, 0};
// Variables for button de-bouncing
#define DEBOUNCE_TIME_MS 200 // 200ms de-bounce delay
static volatile uint32_t last_press_time[6] = {0};

CAN_TxHeaderTypeDef can_tx_header;
uint8_t can_tx_data[8]; //8-byte payload

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

void IMU_Init(void){
	MPU6050_Init(&hi2c1);

	uint8_t who_am_i_val = 0;
	HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(&hi2c1, MPU6050_I2C_ADDR,
											 MPU6050_WHO_AM_I_REG, 1,
											 &who_am_i_val, 1, HAL_MAX_DELAY);
	lcd_clear();
	if (ret != HAL_OK)
	{
		// I2C communication failed (device did not ACK)
		lcd_send_string("I2C Comm Failed");
		while(1);
	}

	// 2. Check if the device is the correct one
	if (who_am_i_val != MPU6050_WHO_AM_I_VAL)
	{
		// Got a response, but it's not the MPU6050
		lcd_send_string("MPU6050 not found");
		while(1);
	}

	lcd_send_string("MPU6050 OK");
	HAL_Delay(1000);
}

void MCP_Init(void){

	  // --- Initialize MCP2515 ---
	  lcd_clear();
	  //8MHz crystal on MCP2515 board and 500kbps bus speed
	   if (MCP2515_Init(&hspi1, SPI_CS_GPIO_Port, SPI_CS_Pin, 8, 500) == MCP2515_OK)
	   {
	       lcd_put_cur(0, 0);
	       lcd_send_string("MCP2515 OK      ");
	   }
	   else
	   {
	       lcd_put_cur(0, 0);
	       lcd_send_string("MCP2515 FAIL    ");
	       while (1); // Halt on error
	   }
	   HAL_Delay(1000);

}

void get_IMU(void){
	MPU6050_Read_All(&hi2c1, &mpu_data);
}

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  HAL_NVIC_DisableIRQ(DMA2_Stream0_IRQn);

  lcd_init(&hi2c1);
  lcd_put_cur(0, 0); // Set cursor to first row, first column
  lcd_send_string("Initializing...");

  IMU_Init();

  MCP_Init();
  MCP_WriteByte(MCP_CANCTRL, 0x00); // NORMAL mode
  while( (MCP_ReadByte(MCP_CANSTAT) & 0xE0) != 0x00 );
  
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&pot_value, NUM_ADC_CHANNELS);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  lcd_clear();
  while (1)
  {
	  // --- 1. Get All Sensor Data ---
	        get_IMU(); // Updates global 'mpu_data'
	                   // Button states are updated by EXTI callback in 'can_buttons'

	        // --- 2. Prepare and Send POT/Button Packet (CAN ID 0x100) ---

	        // Map pot to 0-100, then scale to 0-255 for one byte
	        uint8_t mapped_pot = ((int32_t)pot_value * 255) / 4095;

	        // Pack 6 buttons into one byte (Bit 0 = Btn 0, Bit 1 = Btn 1, etc.)
	        uint8_t button_byte = 0;
	        for (int i = 0; i < 6; i++)
	        {
	            if (can_buttons[i])
	            {
	                button_byte |= (1 << i);
	            }
	        }

	        can_tx_header.StdId = 0x100;     // Standard CAN ID
	        can_tx_header.IDE = CAN_ID_STD;  // Standard ID type
	        can_tx_header.RTR = CAN_RTR_DATA;// Data frame
	        can_tx_header.DLC = 2;           // 2 bytes of data
	        can_tx_data[0] = mapped_pot;     // Data byte 0
	        can_tx_data[1] = button_byte;    // Data byte 1

	        MCP2515_SendMessage(&can_tx_header, can_tx_data);

	        HAL_Delay(10); // Short delay between packets

	        // --- 3. Prepare and Send IMU Accel Packet (CAN ID 0x101) ---

	        // Convert floats to int16_t (multiplied by 100 for 2 decimal places)
	        // e.g., 1.23g becomes 123
	        int16_t ax = (int16_t)(mpu_data.Ax * 100.0f);
	        int16_t ay = (int16_t)(mpu_data.Ay * 100.0f);
	        int16_t az = (int16_t)(mpu_data.Az * 100.0f);

	        can_tx_header.StdId = 0x101;
	        can_tx_header.DLC = 6; // 6 bytes (2 bytes per axis)

	        // Pack int16_t into two uint8_t (Big-Endian)
	        can_tx_data[0] = (ax >> 8) & 0xFF;   // Accel X High Byte
	        can_tx_data[1] = ax & 0xFF;         // Accel X Low Byte
	        can_tx_data[2] = (ay >> 8) & 0xFF;   // Accel Y High Byte
	        can_tx_data[3] = ay & 0xFF;         // Accel Y Low Byte
	        can_tx_data[4] = (az >> 8) & 0xFF;   // Accel Z High Byte
	        can_tx_data[5] = az & 0xFF;         // Accel Z Low Byte

	        MCP2515_SendMessage(&can_tx_header, can_tx_data);


	        // --- 4. Update LCD Display ---
	        char lcd_buf_row0[20];
	        char lcd_buf_row1[20];

	        // We can use the string buffers for the LCD
	        sprintf(lcd_buf_row0, "P:%-3d B:%02X", mapped_pot, button_byte);
	        sprintf(lcd_buf_row1, "X:%.1f Y:%.1f Z:%0.1f", mpu_data.Ax, mpu_data.Ay, mpu_data.Az);

	        lcd_put_cur(0, 0);
	        lcd_send_string(lcd_buf_row0);
	        lcd_put_cur(1, 0);
	        lcd_send_string(lcd_buf_row1);

	        HAL_Delay(90); // Total loop time will be ~100ms
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_1_Pin */
  GPIO_InitStruct.Pin = LED_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LED_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_2_Pin */
  GPIO_InitStruct.Pin = LED_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_CS_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB10 PB3
                           PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  uint32_t current_time = HAL_GetTick(); // Get current time in milliseconds

  switch (GPIO_Pin)
  {
    case GPIO_PIN_0:
      // Check if debounce time has passed for PB0
      if (current_time - last_press_time[0] > DEBOUNCE_TIME_MS)
      {
        can_buttons[0] ^= 1; // Toggle the value (0->1 or 1->0)
        last_press_time[0] = current_time; // Update the last press time
      }
      break;

    case GPIO_PIN_1:
      // Check if debounce time has passed for PB1
      if (current_time - last_press_time[1] > DEBOUNCE_TIME_MS)
      {
        can_buttons[1] ^= 1;
        last_press_time[1] = current_time;
      }
      break;

    case GPIO_PIN_10:
      // Check if debounce time has passed for PB2
      if (current_time - last_press_time[2] > DEBOUNCE_TIME_MS)
      {
        can_buttons[2] ^= 1;
        last_press_time[2] = current_time;
      }
      break;

    case GPIO_PIN_3:
      // Check if debounce time has passed for PB3
      if (current_time - last_press_time[3] > DEBOUNCE_TIME_MS)
      {
        can_buttons[3] ^= 1;
        last_press_time[3] = current_time;
      }
      break;

    case GPIO_PIN_4:
      // Check if debounce time has passed for PB4
      if (current_time - last_press_time[4] > DEBOUNCE_TIME_MS)
      {
        can_buttons[4] ^= 1;
        last_press_time[4] = current_time;
      }
      break;

    case GPIO_PIN_5:
      // Check if debounce time has passed for PB5
      if (current_time - last_press_time[5] > DEBOUNCE_TIME_MS)
      {
        can_buttons[5] ^= 1;
        last_press_time[5] = current_time;
      }
      break;

    default:
      // Unhandled interrupt
      __NOP(); // No Operation
      break;
  }
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

