/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <arm_math.h>

#define RTOS 2 				// 1-FreeRTOS, 2-Keil RTX 5
#define TEST_SELECTION 2	// 0-BOOT_TEST, 1-INTERRUPT_NO_LOAD, 2-INTERRUPT_LOAD, 3-START_TASK_FROM_ISR_NO_LOAD, 4-START_TASK_FROM_ISR_LOAD, 5-TASK_SWITCH_TIME
#define DISPLAY_TYPE 2 		// 0-off, 1-display 0_95in, 2-display 0_96in

#if (TEST_SELECTION == 0)	//BOOT_TEST
	#define BLINK_LD2 0
	#define TEMP 0
	#define FFT 0
	#define DISPLAY 0
	#define QUEUES 0
	#define TASK_SWITCH_TIME 0
#elif (TEST_SELECTION == 1)	//INTERRUPT_NO_LOAD
	#define BLINK_LD2 1
	#define TEMP 0
	#define FFT 0
	#define DISPLAY 0
	#define QUEUES 0
	#define TASK_SWITCH_TIME 0	//INTERRUPT_LOAD
#elif (TEST_SELECTION == 2)
	#define BLINK_LD2 1
	#define TEMP 1
	#define FFT 1
	#define DISPLAY DISPLAY_TYPE
	#define QUEUES 1
	#define TASK_SWITCH_TIME 0
#elif (TEST_SELECTION == 3)		//START_TASK_FROM_ISR_NO_LOAD
	#define BLINK_LD2 0
	#define TEMP 0
	#define FFT 0
	#define DISPLAY 0
	#define QUEUES 0
	#define TASK_SWITCH_TIME 0
	#define START_TASK_FROM_ISR 1
#elif (TEST_SELECTION == 4)		//START_TASK_FROM_ISR_LOAD
	#define BLINK_LD2 1
	#define TEMP 1
	#define FFT 1
	#define DISPLAY DISPLAY_TYPE
	#define QUEUES 1
	#define TASK_SWITCH_TIME 0
	#define START_TASK_FROM_ISR 1
#elif (TEST_SELECTION == 5)		//TASK_SWITCH_TIME
	#define BLINK_LD2 0
	#define TEMP 0
	#define FFT 0
	#define DISPLAY 0
	#define QUEUES 0
	#define TASK_SWITCH_TIME 1
#endif

#if (RTOS == 1)
#include <queue.h>
#include "OLED/test.h"
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLE_BUFFER_LENGTH        4096
#define SAMPLE_BUFFER_LENGTH_HALF   (SAMPLE_BUFFER_LENGTH/2)
#define SAMPLING_RATE               48000

float fft_input[SAMPLE_BUFFER_LENGTH];
float fft_output[SAMPLE_BUFFER_LENGTH];
float fft_power[SAMPLE_BUFFER_LENGTH_HALF];

uint8_t ifftFlag = 0;
float frequency_resolution = (float) SAMPLING_RATE
		/ (float) SAMPLE_BUFFER_LENGTH;

//event flags for keil
osEventFlagsId_t evt_id;
#define FLAGS_MSK1 0x00000001U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;

SPI_HandleTypeDef hspi3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ADC */
osThreadId_t ADCHandle;
const osThreadAttr_t ADC_attributes = {
  .name = "ADC",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Display */
osThreadId_t DisplayHandle;
const osThreadAttr_t Display_attributes = {
  .name = "Display",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for FFT */
osThreadId_t FFTHandle;
const osThreadAttr_t FFT_attributes = {
  .name = "FFT",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for interruptTask */
osThreadId_t interruptTaskHandle;
const osThreadAttr_t interruptTask_attributes = {
  .name = "interruptTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for adcQueue */
osMessageQueueId_t adcQueueHandle;
const osMessageQueueAttr_t adcQueue_attributes = {
  .name = "adcQueue"
};
/* Definitions for fftQueue */
osMessageQueueId_t fftQueueHandle;
const osMessageQueueAttr_t fftQueue_attributes = {
  .name = "fftQueue"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC3_Init(void);
static void MX_SPI3_Init(void);
void StartDefaultTask(void *argument);
void StartADC(void *argument);
void StartDisplay(void *argument);
void StartFFT(void *argument);
void StartInterruptTask(void *argument);

/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == B1_Pin) {
#if ( START_TASK_FROM_ISR == 1)
#if (RTOS == 1)
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		vTaskNotifyGiveFromISR(interruptTaskHandle, &xHigherPriorityTaskWoken);
	    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
#elif (RTOS == 2)
		osEventFlagsSet(evt_id, FLAGS_MSK1);
#endif
#else
		HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin,
				HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin));
#endif
	}
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Calibration for temperature
#define TS_CAL1 *((uint16_t*) 0x1FF1E820)
#define TS_CAL2 *((uint16_t*) 0x1FF1E820)
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
#if (RTOS == 2)
	evt_id = osEventFlagsNew(NULL);
	if (evt_id == NULL) {
	  ; // Event Flags object not created, handle failure
	}
#endif
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
  MX_ADC3_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
#if (DISPLAY == 1)
	OLED_0in95_rgb_test();
#elif (DISPLAY == 2)
	OLED_0in96_test();
#endif

#if (BOOT_TEST == 1)
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
#endif
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
#if (QUEUES == 1)
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of adcQueue */
  adcQueueHandle = osMessageQueueNew (2, sizeof(uint16_t), &adcQueue_attributes);

  /* creation of fftQueue */
  fftQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &fftQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
#endif
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ADC */
  ADCHandle = osThreadNew(StartADC, NULL, &ADC_attributes);

  /* creation of Display */
  DisplayHandle = osThreadNew(StartDisplay, NULL, &Display_attributes);

  /* creation of FFT */
  FFTHandle = osThreadNew(StartFFT, NULL, &FFT_attributes);

  /* creation of interruptTask */
  interruptTaskHandle = osThreadNew(StartInterruptTask, NULL, &interruptTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
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

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
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
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV8;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.Resolution = ADC_RESOLUTION_16B;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x0;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OLED_RST_GPIO_Port, OLED_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|OLED_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OLED_RST_Pin */
  GPIO_InitStruct.Pin = OLED_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(OLED_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC1 PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OLED_CS_Pin */
  GPIO_InitStruct.Pin = OLED_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(OLED_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin OLED_DC_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|OLED_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : STLINK_RX_Pin STLINK_TX_Pin */
  GPIO_InitStruct.Pin = STLINK_RX_Pin|STLINK_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	/* Infinite loop */

	for (;;) {
#if (TASK_SWITCH_TIME == 1)
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, SET);
#if (RTOS == 1)
		taskYIELD();
#elif (RTOS == 2)
		osThreadYield();
#endif
#endif

#if (BOOT_TEST == 1)
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		vTaskDelete(NULL);
#endif

#if (BLINK_LD2 == 1)
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
#endif
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartADC */
/**
 * @brief Function implementing the ADC thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartADC */
void StartADC(void *argument)
{
  /* USER CODE BEGIN StartADC */
#if (TEMP == 1)
	HAL_ADC_Start(&hadc3);
	uint16_t PomiarADC;
	/* Infinite loop */
	for (;;) {
		if (HAL_ADC_PollForConversion(&hadc3, 10) == HAL_OK) {
			PomiarADC = HAL_ADC_GetValue(&hadc3);
			HAL_ADC_Start(&hadc3);
//		int temperatura = (110-30)/(TS_CAL2-TS_CAL1)*(PomiarADC-TS_CAL1)+30;
#if (RTOS == 1)
			xQueueSend(adcQueueHandle, &PomiarADC, 0);
#elif (RTOS == 2)
			osMessageQueuePut(adcQueueHandle, &PomiarADC, 0, 0);
#endif
		}
	}
#else
#if (RTOS == 1)
	vTaskDelete(NULL);
#elif (RTOS == 2)
	osThreadExit();
#endif
#endif
  /* USER CODE END StartADC */
}

/* USER CODE BEGIN Header_StartDisplay */
/**
 * @brief Function implementing the Display thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDisplay */
void StartDisplay(void *argument)
{
  /* USER CODE BEGIN StartDisplay */
#if (DISPLAY != 0)
	uint16_t OdczytADC;
	/* Infinite loop */
	for (;;) {
#if (RTOS == 1)
		xQueueReceive(fftQueueHandle, &OdczytADC, portMAX_DELAY);	//block until data recieved
//		xQueueReceive(adcQueueHandle, &OdczytADC, 0);	//no block
#elif (RTOS == 2)
		osMessageQueueGet(fftQueueHandle, &OdczytADC, 0, osWaitForever);	//block until data recieved
#endif

#if (DISPLAY == 1)
		OLED_0in95_rgb_print_num(OdczytADC);
#elif (DISPLAY == 2)
		OLED_0in96_print_num(OdczytADC);
#endif
	}
#else
#if (RTOS == 1)
	vTaskDelete(NULL);
#elif (RTOS == 2)
	osThreadExit();
#endif
#endif
  /* USER CODE END StartDisplay */
}

/* USER CODE BEGIN Header_StartFFT */
/**
 * @brief Function implementing the FFT thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartFFT */
void StartFFT(void *argument)
{
  /* USER CODE BEGIN StartFFT */
	/* Infinite loop */
#if (FFT == 1)
	uint16_t OdczytADC;
#if (RTOS == 1)
		taskENTER_CRITICAL();
#elif (RTOS == 2)
		osKernelLock();
#endif
	arm_rfft_fast_instance_f32 fft;
	arm_rfft_fast_init_f32(&fft, SAMPLE_BUFFER_LENGTH);
#if (RTOS == 1)
		taskEXIT_CRITICAL();
#elif (RTOS == 2)
		osKernelUnlock();
#endif
	for (;;) {
#if (RTOS == 1)
		xQueueReceive(adcQueueHandle, &OdczytADC, portMAX_DELAY);	//block until data recieved
#elif (RTOS == 2)
		osMessageQueueGet(adcQueueHandle, &OdczytADC, 0, osWaitForever);	//block until data recieved
#endif
		/* write signal to array */
		for (int i = 0; i < SAMPLE_BUFFER_LENGTH; i++) {
			float r = (float) i / (float) SAMPLING_RATE;
			r *= 3.14159265359 * 2;
			r *= 880; // frequency in Hz
			float s = sin(r) + sin(r * 4) * 0.5 + sin(r * 3) * 0.25;
			fft_input[i] = s;
		}

		/* analyze signal */
#if (RTOS == 1)
		taskENTER_CRITICAL();
#elif (RTOS == 2)
		osKernelLock();
#endif
		arm_rfft_fast_f32(&fft, fft_input, fft_output, ifftFlag);
		arm_cmplx_mag_f32(fft_output, fft_power, SAMPLE_BUFFER_LENGTH_HALF);
#if (RTOS == 1)
		taskEXIT_CRITICAL();
#elif (RTOS == 2)
		osKernelUnlock();
#endif

		/* find dominant frequency */
		float32_t maxValue;
		uint32_t maxIndex;
		arm_max_f32(fft_power, SAMPLE_BUFFER_LENGTH_HALF, &maxValue, &maxIndex);
#if (RTOS == 1)
			xQueueSend(fftQueueHandle, &OdczytADC, portMAX_DELAY);
#elif (RTOS == 2)
			osMessageQueuePut(fftQueueHandle, &OdczytADC, 0, osWaitForever);
#endif
	}
#elif (TASK_SWITCH_TIME == 1)
	for(;;) {
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, RESET);
#if (RTOS == 1)
		taskYIELD();
#elif (RTOS == 2)
		osThreadYield();
#endif
	}
#else
#if (RTOS == 1)
	vTaskDelete(NULL);
#elif (RTOS == 2)
	osThreadExit();
#endif
#endif
  /* USER CODE END StartFFT */
}

/* USER CODE BEGIN Header_StartInterruptTask */
/**
* @brief Function implementing the interruptTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartInterruptTask */
void StartInterruptTask(void *argument)
{
  /* USER CODE BEGIN StartInterruptTask */
#if ( START_TASK_FROM_ISR == 1)
	for(;;){
#if (RTOS == 1)
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
#elif (RTOS == 2)
		osEventFlagsWait(evt_id, FLAGS_MSK1, osFlagsWaitAny, osWaitForever);
#endif
		HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
	}
#else
#if (RTOS == 1)
	vTaskDelete(NULL);
#elif (RTOS == 2)
	osThreadExit();
#endif
#endif
  /* USER CODE END StartInterruptTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
	while (1) {
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
