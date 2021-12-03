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
#include "MAX30105.h"
#include <string.h>
#include <stdio.h>

#include "hr_processing.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define I2C_BUF_LENGTH 288 // bytes for the max number of samples we might get
#define MAX30105_ADDRESS          (0x57 << 1) //7-bit I2C Address, shifted left once
#define PPG_BUF_SIZE 50
#define AUDIO_BUF_SIZE 1000
#define I2S_FIFO_SIZE 64
static const uint8_t MAX30105_FIFODATA =		0x07;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;

I2S_HandleTypeDef hi2s1;
DMA_HandleTypeDef hdma_spi1_rx;

TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t i2c_buf[I2C_BUF_LENGTH];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2S1_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM10_Init(void);
/* USER CODE BEGIN PFP */
void DMATransferComplete(DMA_HandleTypeDef *DmaHandle);
static void argInit_200x1_real_T(double result[200]);
static double argInit_real_T(void);
static void main_hr_processing(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t data_in[I2S_FIFO_SIZE];
MAX30105 *hr_sens;
uint32_t *red_buf;
uint32_t *ir_buf;
int32_t *audio_buf;
uint16_t volatile ppg_buf_index = 0;
uint16_t volatile audio_buf_index = 0;
/* Function Definitions */
/*
 * Arguments    : double result[200]
 * Return Type  : void
 */
static void argInit_200x1_real_T(double result[200])
{
  int idx0;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 200; idx0++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx0] = argInit_real_T();
  }
}

/*
 * Arguments    : void
 * Return Type  : double
 */
static double argInit_real_T(void)
{
  return 0.0;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
static void main_hr_processing(void)
{
  double ppg_data_tmp[200];
  double heart_rate;
  /* Initialize function 'hr_processing' input arguments. */
  /* Initialize function input argument 'ppg_data'. */
  argInit_200x1_real_T(ppg_data_tmp);
  /* Initialize function input argument 'mic_data'. */
  /* Call the entry-point 'hr_processing'. */
  heart_rate = hr_processing(ppg_data_tmp, ppg_data_tmp);
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
  main_hr_processing();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2S1_Init();
  MX_TIM11_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
  //buffers for "chunk" of data given to algo
  red_buf = (uint32_t*)calloc(PPG_BUF_SIZE, 4);
  ir_buf = (uint32_t*)calloc(PPG_BUF_SIZE, 4);
  audio_buf = (int32_t*)calloc(AUDIO_BUF_SIZE, 4);

  hr_sens = new MAX30105();
  hr_sens->begin(hi2c1);
  hr_sens->setup(0x1D, 4, 2, 200, 215, 8192);
  // Start timer
  HAL_TIM_Base_Start_IT(&htim11);
  HAL_I2S_Receive_DMA(&hi2s1, data_in, I2S_FIFO_SIZE/2);
  //HAL_TIM_Base_Start_IT(&htim10);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (audio_buf_index == AUDIO_BUF_SIZE) {
		  int i = ppg_buf_index;
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief I2S1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S1_Init(void)
{

  /* USER CODE BEGIN I2S1_Init 0 */

  /* USER CODE END I2S1_Init 0 */

  /* USER CODE BEGIN I2S1_Init 1 */

  /* USER CODE END I2S1_Init 1 */
  hi2s1.Instance = SPI1;
  hi2s1.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s1.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s1.Init.DataFormat = I2S_DATAFORMAT_24B;
  hi2s1.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s1.Init.AudioFreq = I2S_AUDIOFREQ_32K;
  hi2s1.Init.CPOL = I2S_CPOL_LOW;
  hi2s1.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s1.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S1_Init 2 */

  /* USER CODE END I2S1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 100000;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 1000;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 100000;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 19;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

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
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/* USER CODE BEGIN 4 */
// Callback: timer has rolled over
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback and toggle LED
  if (htim == &htim11 )
  {
	  //Read register FIDO_DATA in (3-byte * number of active LED) chunks
	  //Until FIFO_RD_PTR = FIFO_WR_PTR
	  if (hi2c1.State != HAL_I2C_STATE_READY) { return; }
	  auto readPointer = hr_sens->getReadPointer();
	  auto writePointer = hr_sens->getWritePointer();

	  int numberOfSamples = 0;

	  //Do we have new data?
	  if (readPointer != writePointer)
	  {
	    //Calculate the number of readings we need to get from sensor
	    numberOfSamples = writePointer - readPointer;
	    if (numberOfSamples < 0) numberOfSamples += 32; //Wrap condition

	    //We now have the number of readings, now calc bytes to read
	    //For this example we are just doing Red and IR (3 bytes each)
	    int bytesLeftToRead = numberOfSamples * hr_sens->getActiveLEDs() * 3;

	    //Get ready to read a burst of data from the FIFO register

	    // Set the register to read from
		auto ret = HAL_I2C_Mem_Read_DMA(&hi2c1, MAX30105_ADDRESS, MAX30105_FIFODATA, 1, i2c_buf, bytesLeftToRead);
		ret; // for debugging
	  }

	  return; //Let the world know how much new data we found
  }

  if (htim == &htim10 )
  {
	if (audio_buf_index == 0 || ppg_buf_index == 0) { return; }
	uint8_t uart_buf[32];
	HAL_I2S_DMAPause(&hi2s1);
	HAL_TIM_Base_Stop_IT(&htim11);
	for (int i = 0; i < AUDIO_BUF_SIZE; i += 1) {
		sprintf((char*)uart_buf,"%li\r\n", audio_buf[i]);
		HAL_UART_Transmit(&huart2, uart_buf, strlen((char*)uart_buf), 1);
	}
	for (int i = 0; i < PPG_BUF_SIZE; i += 1) {
		sprintf((char*)uart_buf,"RED VAL:%li\r\n", red_buf[i]);
		HAL_UART_Transmit(&huart2, uart_buf, strlen((char*)uart_buf), 1);
		sprintf((char*)uart_buf,"IR VAL:%li\r\n", ir_buf[i]);
		HAL_UART_Transmit(&huart2, uart_buf, strlen((char*)uart_buf), 1);
	}

	audio_buf_index = 0;
	ppg_buf_index = 0;
	HAL_I2S_DMAResume(&hi2s1);
	HAL_TIM_Base_Start_IT(&htim11);
  }

}
// HAL_I2C_MasterRxCpltCallback

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef * hi2c)
{
		uint16_t bytesLeftToRead = hi2c->XferSize;

		uint8_t activeLEDs = hr_sens->getActiveLEDs();
		int rec_buf_pos = 0; // track position in rec_buf
		uint8_t uart_buf[32];
		while (bytesLeftToRead > 0)
		{
			if (ppg_buf_index >= PPG_BUF_SIZE) { return; }
			uint8_t temp[sizeof(uint32_t)]; //Array of 4 bytes that we will convert into long
			uint32_t tempLong;


			//Burst read three bytes - RED
			temp[3] = 0;
			temp[2] = i2c_buf[rec_buf_pos++];
			temp[1] = i2c_buf[rec_buf_pos++];
			temp[0] = i2c_buf[rec_buf_pos++];

			//Convert array to long
			memcpy(&tempLong, temp, sizeof(tempLong));

			tempLong &= 0x3FFFF; //Zero out all but 18 bits

			red_buf[ppg_buf_index] = tempLong; //Store this reading into the sense array
			//sprintf((char*)uart_buf,"RED VAL:%lu\r\n", red_val);
			//HAL_UART_Transmit(&huart2, uart_buf, strlen((char*)uart_buf), 1);
			// TODO add the value to the red buffer

			if (activeLEDs > 1)
			{
			  //Burst read three more bytes - IR
			  temp[3] = 0;
			  temp[2] = i2c_buf[rec_buf_pos++];
			  temp[1] = i2c_buf[rec_buf_pos++];
			  temp[0] = i2c_buf[rec_buf_pos++];

			  //Convert array to long
			  memcpy(&tempLong, temp, sizeof(tempLong));

			  tempLong &= 0x3FFFF; //Zero out all but 18 bits

			  ir_buf[ppg_buf_index] = tempLong;
			  //sprintf((char*)uart_buf,"IR VAL:%lu\r\n", ir_val);
			  //HAL_UART_Transmit(&huart2, uart_buf, strlen((char*)uart_buf), 1);
			  // TODO add the value to the ir buffer
			}

			if (activeLEDs > 2)
			{
			  //Burst read three more bytes - Green
			  temp[3] = 0;
			  temp[2] = i2c_buf[rec_buf_pos++];
			  temp[1] = i2c_buf[rec_buf_pos++];
			  temp[0] = i2c_buf[rec_buf_pos++];


			  //Convert array to long
			  memcpy(&tempLong, temp, sizeof(tempLong));

			  tempLong &= 0x3FFFF; //Zero out all but 18 bits

			  uint32_t green_val = tempLong;
			  //sprintf((char*)uart_buf,"GREEN VAL:%lu\r\n", green_val);
			  //HAL_UART_Transmit(&huart2, uart_buf, strlen((char*)uart_buf), 1);
			}
			ppg_buf_index++;
			bytesLeftToRead -= activeLEDs * 3;

		} //End while (bytesLeftToRead > 0)
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
	if (audio_buf_index < AUDIO_BUF_SIZE) {
		int32_t audio1 = (int32_t)(data_in[0] << 16 | data_in[1]);
		audio1 = audio1 >> 14;
		audio_buf[audio_buf_index] = audio1;
		audio_buf_index++;
	}
	else {
		int i = ppg_buf_index;
	}
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  //HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

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
