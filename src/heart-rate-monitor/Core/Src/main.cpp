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
#include "arm_math.h"
#include "arm_const_structs.h"
#include <cmath>
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
static const uint8_t SAMPLE_WINDOW = 8; // seconds

static const uint32_t PPG_SAMPLING_FREQUENCY = 400;
static const uint32_t PPG_AVG_RATE = 8;
static const uint32_t PPG_SAMPLES_PER_SECOND = PPG_SAMPLING_FREQUENCY/PPG_AVG_RATE;

static const uint32_t AUDIO_FS_BEFORE_AVG = 1000;
static const uint32_t AUDIO_AVG_RATE = 2;
static const uint32_t AUDIO_SAMPLES_PER_SECOND = AUDIO_FS_BEFORE_AVG / AUDIO_AVG_RATE;
static const uint32_t PPG_BUF_SIZE = (PPG_SAMPLES_PER_SECOND * SAMPLE_WINDOW);
static const uint32_t AUDIO_BUF_SIZE  = (AUDIO_SAMPLES_PER_SECOND * SAMPLE_WINDOW);

#define I2S_FIFO_SIZE 1024 // buffer for the dma
static const uint8_t MAX30105_FIFODATA =		0x07;

static const uint16_t AUDIO_FFT_LEN = 4096;
static const uint16_t PPG_FFT_LEN = 512;

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
uint16_t data_in[I2S_FIFO_SIZE];
MAX30105 *hr_sens;
float *red_buf;
float *ir_buf;
float *audio_buf;
uint16_t volatile ppg_buf_index = 0;
uint16_t volatile audio_buf_index = 0;
// Signal from timer that we should queue an I2C transfer from PPG
bool volatile queueI2cTransfer = false;
// Signal to timer that I2C transfer complete, OK to start
// requesting data again.
bool volatile waitingForI2c = false;
// Indicate the first time calculating BPM, so we ignore history
bool firstBpm = true;
uint16_t heartbeat = 0;

arm_rfft_fast_instance_f32 varInstCfftF32;
arm_rfft_fast_instance_f32 ppgFftInst;
uint32_t volatile start_tick;
uint32_t volatile end_tick;
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
void StartPpgRequest();
void ProcessData();

/* Function Declarations */
static void argInit_10000x1_int32_T(int result[10000]);
static void argInit_500x1_uint32_T(unsigned int result[500]);
static void argInit_d500x1_uint32_T(unsigned int result_data[],
                                    int *result_size);
static int argInit_int32_T(void);
static double argInit_real_T(void);
static unsigned short argInit_uint16_T(void);
static unsigned int argInit_uint32_T(void);
static void main_hr_processing(void);

/*
 * Returns the BPM calculated for the data in data_buf.
 */
float fft(arm_rfft_fast_instance_f32 * fftInst, float *data_buf, uint32_t data_len, uint32_t fft_len, uint32_t fs)
{
	arm_status status;
	float32_t maxValue;
	uint32_t maxIndex;
	status = ARM_MATH_SUCCESS;

	// Fill an 2^n array with 0s for 0 padding
	float32_t fft_array[fft_len];
	arm_fill_f32(0, &fft_array[data_len], fft_len-data_len);
	float32_t mean;
	// Get the DC component as the mean of the audio sample
	arm_mean_f32(data_buf, data_len, &mean);
	// Subtract the mean from the audio buf data.
	arm_fill_f32(mean, fft_array, data_len);
	arm_sub_f32(data_buf, fft_array, fft_array, data_len);

	float32_t fft_output[fft_len];
	uint8_t ifftFlag = 0; // 0=FFT, 1=InverseFFT
	/* Process the data through the RFFT/RIFFT module */
	arm_rfft_fast_f32 (fftInst, fft_array, fft_output, ifftFlag);
	/* Process the data through the Complex Magnitude Module for
	calculating the magnitude at each bin */
	arm_cmplx_mag_f32(fft_output, fft_array, fft_len/2);

	float hz_per_bin = ((float) fs) / fft_len;
	uint32_t min_bin = std::floor(1.0 / hz_per_bin);
	uint32_t max_bin = std::ceil(4.0 / hz_per_bin);

	/* Calculates maxValue and returns corresponding BIN value */
	arm_max_f32(&fft_array[min_bin], max_bin - min_bin, &maxValue, &maxIndex);

	// Calculate the BPM.
	// We add maxIndex and min_bin because the index is found in
	// a subset of the larger array between (min_bin, max_bin). So
	// we add the min_bin to account for this offset.
	// Multiply by hz_per_bin to get frequency.
	// Multiply by 60.0 BPM / Hz
	auto bpm = (maxIndex + min_bin) * hz_per_bin * 60.0;

	return bpm;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
static void main_hr_processing(void)
{

  auto audio_bpm = fft(&varInstCfftF32, audio_buf, AUDIO_BUF_SIZE, AUDIO_FFT_LEN, AUDIO_SAMPLES_PER_SECOND);
  auto ir_bpm = fft(&ppgFftInst, ir_buf, PPG_BUF_SIZE, PPG_FFT_LEN, PPG_SAMPLES_PER_SECOND);
  auto red_bpm = fft(&ppgFftInst, red_buf, PPG_BUF_SIZE, PPG_FFT_LEN, PPG_SAMPLES_PER_SECOND);;

  uint8_t uart_buf[64];
  if (false) {
	  sprintf((char*)uart_buf,"red bpm:%.0f\r\n", red_bpm);
	  HAL_UART_Transmit(&huart2, uart_buf, strlen((char*)uart_buf), 1);
	  sprintf((char*)uart_buf,"ir bpm:%.0f\r\n", ir_bpm);
	  HAL_UART_Transmit(&huart2, uart_buf, strlen((char*)uart_buf), 1);
	  sprintf((char*)uart_buf,"audio bpm:%.0f\r\n", audio_bpm);
	  HAL_UART_Transmit(&huart2, uart_buf, strlen((char*)uart_buf), 1);
  }

  const float ppg_weight = 0.98;
  const float audio_weight = 0.02;

  if (firstBpm) {
	  heartbeat = std::round(ppg_weight * ((ir_bpm + red_bpm) / 2.0) + audio_weight * audio_bpm);
	  firstBpm = false;
  } else {
	  heartbeat = std::round(ppg_weight * ((ir_bpm + red_bpm + heartbeat) / 3.0) + audio_weight * audio_bpm);
  }

  sprintf((char*)uart_buf,"bpm:%u\r\n", heartbeat);
  HAL_UART_Transmit(&huart2, uart_buf, strlen((char*)uart_buf), 1);

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
  MX_USART2_UART_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2S1_Init();
  MX_TIM11_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
  //buffers for "chunk" of data given to algo
  red_buf = (float*)calloc(PPG_BUF_SIZE, 4);
  ir_buf = (float*)calloc(PPG_BUF_SIZE, 4);
  audio_buf = (float*)calloc(AUDIO_BUF_SIZE, 4);

  hr_sens = new MAX30105();
  hr_sens->begin(hi2c1);
  hr_sens->setup(0x1D, PPG_AVG_RATE, 2, PPG_SAMPLING_FREQUENCY, 215, 8192);

  arm_status status;
  status=arm_rfft_fast_init_f32(&varInstCfftF32, AUDIO_FFT_LEN);
  status=arm_rfft_fast_init_f32(&ppgFftInst, PPG_FFT_LEN);

  // Start timer
  HAL_TIM_Base_Start_IT(&htim11);
  HAL_I2S_Receive_DMA(&hi2s1, data_in, I2S_FIFO_SIZE/2);
  // TODO RV: Remove?
  // HAL_TIM_Base_Start_IT(&htim10);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (ppg_buf_index >= PPG_BUF_SIZE) {
		  ProcessData();
	  }

	  if (queueI2cTransfer)
	  {
		  // Gets the head and tail pointers
		  // then schedules a nonblocking transfer of data
		  StartPpgRequest();
		  queueI2cTransfer = false;
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
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 1, 2);
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

// StartPpgRequest and HAL_I2C_MemRxCpltCallback based on Sparkfun library, in Sensors/
// Their copyright notice is reproduced below:
/***************************************************
 This is a library written for the Maxim MAX30105 Optical Smoke Detector
 It should also work with the MAX30102. However, the MAX30102 does not have a Green LED.
 These sensors use I2C to communicate, as well as a single (optional)
 interrupt line that is not currently supported in this driver.

 Written by Peter Jansen and Nathan Seidle (SparkFun)
 BSD license, all text above must be included in any redistribution.
 *****************************************************/

// Gets the number of samples to read and queues a non-blocking I2C transfer.
void StartPpgRequest()
{
	  //Read register FIDO_DATA in (3-byte * number of active LED) chunks
	  //Until FIFO_RD_PTR = FIFO_WR_PTR
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
		waitingForI2c = true;
	  }

	  return; //Let the world know how much new data we found
}

// Call this function when we're ready to parse the data in the
// gloabl buffers.
void ProcessData()
{
//	if (audio_buf_index == 0 || ppg_buf_index == 0) { return; }
	uint8_t uart_buf[32];

	// Capture the indices before outside forces can change them
	auto local_audio_idx = audio_buf_index;
	auto local_ppg_idx = ppg_buf_index;
	main_hr_processing();

	// Disable the data collection interrupts
//	HAL_I2S_DMAStop(&hi2s1);
//	HAL_TIM_Base_Stop_IT(&htim11);
	// Log to console all the updated audio values
	if(false)
	{
		for (int i = 0; i < local_audio_idx; i += 1) {
			sprintf((char*)uart_buf,"%.0f\r\n", audio_buf[i]);
			HAL_UART_Transmit(&huart2, uart_buf, strlen((char*)uart_buf), 1);
		}
		// Log to console all the updated ppg values
		for (int i = 0; i < local_ppg_idx; i += 1) {
			sprintf((char*)uart_buf,"RED VAL:%.0f\r\n", red_buf[i]);
			HAL_UART_Transmit(&huart2, uart_buf, strlen((char*)uart_buf), 1);
			sprintf((char*)uart_buf,"IR VAL:%.0f\r\n", ir_buf[i]);
			HAL_UART_Transmit(&huart2, uart_buf, strlen((char*)uart_buf), 1);
		}
	}

	// Reset the indices and re-enable the data collect interrupts
	audio_buf_index = 0;
	ppg_buf_index = 0;
//	HAL_I2S_Receive_DMA(&hi2s1, data_in, I2S_FIFO_SIZE/2);
//	HAL_TIM_Base_Start_IT(&htim11);
}

// Callback: timer has rolled over
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback and toggle LED
  if (htim == &htim11 )
  {
	  // if the i2c bus is still busy, don't queue another transfer
	  if (hi2c1.State != HAL_I2C_STATE_READY || queueI2cTransfer || waitingForI2c) { return; }
	  // Set this true to tell the main while loop that the timer has elapsed to
	  // look for new samples on I2C.
	  queueI2cTransfer = true;
  }

  // TODO RV: remove?
  if (htim == &htim10 )
  {
	  ProcessData();
  }
}


void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef * hi2c)
{
	uint16_t bytesLeftToRead = hi2c->XferSize;

	uint8_t activeLEDs = hr_sens->getActiveLEDs();
	int rec_buf_pos = 0; // track position in rec_buf
	while (bytesLeftToRead > 0)
	{
		if (ppg_buf_index >= PPG_BUF_SIZE) {
			return;
		}
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

		red_buf[ppg_buf_index] = (float)tempLong; //Store this reading into the sense array

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

		  ir_buf[ppg_buf_index] = (float) tempLong;
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
		}
		ppg_buf_index++;
		bytesLeftToRead -= activeLEDs * 3;

	} //End while (bytesLeftToRead > 0)
	// Ready to queue another transfer
	waitingForI2c = false;
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
	const uint8_t samplesToSkip = 32; // only pick 1 of 16 samples
	const uint8_t bytesBetweenSamples = 4; // each sample is 2 bytes, have L and R in buffer but only want L
	const uint16_t for_increment = samplesToSkip * bytesBetweenSamples * AUDIO_AVG_RATE;
	const uint8_t shift_amt = 1; // log2(AUDIO_AVG_RATE)
	for (uint16_t i = 0; i < I2S_FIFO_SIZE; i += for_increment)
	{
		// we've filled the audio buffer
		if (audio_buf_index >= AUDIO_BUF_SIZE) { return; }

		int32_t audio_avg = 0;
		for (uint16_t j = 0; j < AUDIO_AVG_RATE; j++)
		{
			// Parse the audio data
			int32_t temp = \
					(int32_t)(data_in[i + (j*samplesToSkip*bytesBetweenSamples)] << 16 \
							| data_in[i+1 + (j*samplesToSkip*bytesBetweenSamples)]);
			// perform the req'd audio shift (14) and divide by our averaging shift_amt
			temp = temp >> (14 + shift_amt);
			audio_avg += temp;
		}

		// Add to buffer and increment
		audio_buf[audio_buf_index] = (float) audio_avg;
		audio_buf_index++;
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
