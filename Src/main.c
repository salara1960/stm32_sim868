/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "bmp280.h"
#include "bh1750.h"

//const char *ver = "ver 1.0rc1";//04.05.2019 - first working release
//const char *ver = "ver 1.1rc1";//05.05.2019 - add virtual com port (usb device CDC)
//const char *ver = "ver 1.1rc2";//06.05.2019 - remove USART3, now log_defice - virtual serial port on usb CDC
//const char *ver = "ver 1.2rc1";//08.05.2019 - connect sim868 to board (recv. from UART4, UART5 in interrupt_callback_function)
//const char *ver = "ver 1.2rc2";//09.05.2019 - add USART3 (aka log), remove Virtual Com Port on usb device
//const char *ver = "ver 1.2rc3";//11.05.2019 - add QUEUE for at command
//const char *ver = "ver 1.2rc4";//12.05.2019 - make QUEUE functions universal (for all queues), add GSM status (On/Off - Pin PA2)
//const char *ver = "ver 1.2rc5";//13.05.2019 - minor changes : recv. msg from gsm_command_uart (speed=9600 for uart4), REMOVE RTC
//const char *ver = "ver 1.3rc1";//15.05.2019 - minor changes : set speed=38400 (AT+CGNSCMD=0,"$PMTK251,38400*27") for uart5 (GPS), add parser for AT+CGNSINF command
//const char *ver = "ver 1.4rc1";//16.05.2019 - major changes : add send auto at_commands, add new timer with period 500 ms
//const char *ver = "ver 1.4rc2";//16.05.2019 - minor changes : change TIM2 period from 500 ms to 250 ms
//const char *ver = "ver 1.4rc3";//17.05.2019 - minor changes++
const char *ver = "ver 1.5rc1";//17.05.2019 - major changes : add json library !!!



/*
post-build steps command:
arm-none-eabi-objcopy -O binary "${BuildArtifactFileBaseName}.elf" "${BuildArtifactFileBaseName}.bin" && ls -la | grep "${BuildArtifactFileBaseName}.*"
*/

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_usart3_tx;

osThreadId_t defTaskHandle;
osThreadId_t sensTaskHandle;
osThreadId_t atTaskHandle;
osThreadId_t gpsTaskHandle;
osMessageQueueId_t mailQueueHandle;
osSemaphoreId_t binSemHandle;
/* USER CODE BEGIN PV */

volatile static uint32_t secCounter = 0;
volatile static uint64_t HalfSecCounter = 0;

HAL_StatusTypeDef i2cError = HAL_OK;
const uint32_t min_wait_ms = 150;
const uint32_t max_wait_ms = 1000;

I2C_HandleTypeDef *portBMP;
I2C_HandleTypeDef *portSSD;
I2C_HandleTypeDef *portBH;

static const char *_extDate = "DATE:";
#ifdef WITH_RTC
	bool evt_clear = false;
	static bool setDate = false;
	volatile uint32_t extDate = 0;
#endif

static char RxBuf[MAX_UART_BUF];
volatile uint8_t rx_uk;
uint8_t lRxByte;

static const char *_extRMC = "$GNRMC";
static char GpsRxBuf[MAX_UART_BUF];
volatile uint8_t gps_rx_uk;
uint8_t gRxByte;
uint32_t rmcCounter = 0;
uint8_t rmc5 = 6;
char msgNMEA[MAX_UART_BUF] = {0};
static s_msg_t q_gps;
const char *nameValid[] = {"Invaid", "Valid"};
const char *nameNS[] = {"North" , "South"};
const char *nameEW[] = {"East" , "West"};
static bool onGNS = false;

static char AtRxBuf[MAX_UART_BUF];
volatile uint8_t at_rx_uk;
uint8_t aRxByte;
uint8_t adone = 0;
uint32_t atCounter = 0;
char msgAT[MAX_UART_BUF] = {0};
static s_msg_t q_at;
static bool evt_gsm = false;
static bool OnOffEn = false;
static bool onGSM = false;

static int8_t cmdsInd = -1;
volatile bool cmdsDone = true;
char msgCMD[MAX_UART_BUF] = {0};
static s_msg_t q_cmd;
const uint8_t cmdsMax = 10;
const char *cmds[] = {
	"AT\r\n",
	"AT+CMEE=0\r\n",
	"AT+GMR\r\n",
	"AT+GSN\r\n",
	"AT+CCLK?\r\n",
	//"AT+CGNSCMD=0,\"$PMTK251,38400*27\"\r\n",
	//AT+CGNSCMD=0,"$PMTK251,115200*1F"
	"AT+CGNSPWR=1\r\n",
	"AT+CGNSPWR?\r\n",
	"AT+CSQ\r\n",
	"AT+CREG?\r\n",
	"AT+CGNSINF\r\n"
};

uint8_t *adrByte = NULL;
volatile s_flags flags = {0, 0, 0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefTask(void *argument); // for v2
void StartSensTask(void *argument); // for v2
void StartAtTask(void *argument); // for v2
void StartGpsTask(void *argument); // for v2

/* USER CODE BEGIN PFP */

void Report(bool addTime, const char *fmt, ...);
void errLedOn(const char *from);
void gsmONOFF();
void ClearRxBuf();
void initQ(s_msg_t *q);
int8_t putQ(char *adr, s_msg_t *q);
int8_t getQ(char *adr, s_msg_t *q);

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
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

#ifdef GSM_KEY_Pin
  HAL_GPIO_WritePin(GSM_KEY_GPIO_Port, GSM_KEY_Pin, GPIO_PIN_SET);//1
#endif

  ClearRxBuf();

  portBMP = &hi2c1;
  portSSD = &hi2c1;
  portBH  = &hi2c1;


#ifdef GSM_KEY_Pin
  HAL_GPIO_WritePin(GSM_KEY_GPIO_Port, GSM_KEY_Pin, GPIO_PIN_SET);
#endif
  HAL_GPIO_WritePin(GPIO_PortD, LED_GREEN_Pin | LED_BLUE_Pin | LED_RED_Pin | LED_ORANGE_Pin, GPIO_PIN_RESET);

  ssd1306_on(true);//screen ON
    if (!i2cError) {
  	  ssd1306_init();//screen INIT
  	  if (!i2cError) {
  		  ssd1306_pattern();//set any params for screen
  		  if (!i2cError) {
  			  //ssd1306_invert();//set inverse color mode
  			  //if (!i2cError)
  				  ssd1306_clear();//clear screen
  		  }
        }
    }

    bh1750_off();

    // start timer2 + interrupt
    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  osKernelInitialize(); // Initialize CMSIS-RTOS

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of binSem */
  const osSemaphoreAttr_t binSem_attributes = {
    .name = "binSem"
  };
  binSemHandle = osSemaphoreNew(1, 1, &binSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of mailQueue */
  const osMessageQueueAttr_t mailQueue_attributes = {
    .name = "mailQueue"
  };
  mailQueueHandle = osMessageQueueNew (4, sizeof(result_t), &mailQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */

  	  HAL_Delay(1000);
  	  ClearRxBuf();
  	  HAL_UART_Receive_IT(&huart3, (uint8_t *)&lRxByte, 1);//LOG

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defTask */
  const osThreadAttr_t defTask_attributes = {
    .name = "defTask",
    .priority = (osPriority_t) osPriorityAboveNormal,
    .stack_size = 1024
  };
  defTaskHandle = osThreadNew(StartDefTask, NULL, &defTask_attributes);

  /* definition and creation of sensTask */
  const osThreadAttr_t sensTask_attributes = {
    .name = "sensTask",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 512
  };
  sensTaskHandle = osThreadNew(StartSensTask, NULL, &sensTask_attributes);

  /* definition and creation of atTask */
  const osThreadAttr_t atTask_attributes = {
    .name = "atTask",
    .priority = (osPriority_t) osPriorityHigh,
    .stack_size = 1536
  };
  atTaskHandle = osThreadNew(StartAtTask, NULL, &atTask_attributes);

  /* definition and creation of gpsTask */
  const osThreadAttr_t gpsTask_attributes = {
    .name = "gpsTask",
    .priority = (osPriority_t) osPriorityNormal2,
    .stack_size = 1024
  };
  gpsTaskHandle = osThreadNew(StartGpsTask, NULL, &gpsTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  	  LOOP_FOREVER()

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

	secCounter     = 0; //1 sec counter (uint32_t)
	HalfSecCounter = 0; // 0.5 sec counter (uint64_t)

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  // htim2.Init.Prescaler = 41999; - 500ms
  // htim2.Init.Period = 124; - 500ms / 2 = 250 ms
  // 4 interrupt for one seconda period

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 41999;//20999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 124;//249;//499;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

#ifdef GSM_KEY_Pin
	HAL_GPIO_WritePin(GPIO_PortD, GSM_KEY_Pin, GPIO_PIN_RESET);//set 1
#endif

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;//38400;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GSM_KEY_GPIO_Port, GSM_KEY_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED_GREEN_Pin|LED_ORANGE_Pin|LED_RED_Pin|LED_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : GSM_STAT_Pin */
  GPIO_InitStruct.Pin = GSM_STAT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GSM_STAT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GSM_KEY_Pin */
  GPIO_InitStruct.Pin = GSM_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GSM_KEY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_GREEN_Pin LED_ORANGE_Pin LED_RED_Pin LED_BLUE_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|LED_ORANGE_Pin|LED_RED_Pin|LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

//------------------------------------------------------------------------------------------

uint8_t hextobin(char st, char ml)
{
const char hex[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
uint8_t a, b, c, i;

    for	(i = 0; i < 16; i++) { if (st == hex[i]) { b = i; break; } else b = 255; }
    for	(i = 0; i < 16; i++) { if (ml == hex[i]) { c = i; break; } else c = 255; }
    if ((b == 255) || (c == 255)) a = 255; else { b = b << 4;   a = b | c; }

    return a;
}
//------------------------------------------------------------------------------------------
void gsmONOFF()
{
	Report(true, "[%s] GSM_KEY set to 0\r\n", __func__);
	//HAL_GPIO_WritePin(GSM_KEY_GPIO_Port, GSM_KEY_Pin, GPIO_PIN_SET);//set 1
	//HAL_Delay(100);
	HAL_GPIO_WritePin(GSM_KEY_GPIO_Port, GSM_KEY_Pin, GPIO_PIN_RESET);//set 0
	HAL_Delay(1350);
	HAL_GPIO_WritePin(GSM_KEY_GPIO_Port, GSM_KEY_Pin, GPIO_PIN_SET);//set 1
	Report(true, "[%s] GSM_KEY set to 1\r\n", __func__);
}
//-----------------------------------------------------------------------------
void ClearRxBuf()
{
	adone = 0;
	rx_uk = 0;
    memset(RxBuf, 0, MAX_UART_BUF);
}
//-----------------------------------------------------------------------------

void set_Date(time_t epoch)
{
#ifdef WITH_RTC

RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;
struct tm ts;

	gmtime_r(&epoch, &ts);

	sDate.WeekDay = ts.tm_wday;
	sDate.Month   = ts.tm_mon + 1;
	sDate.Date    = ts.tm_mday;
	sDate.Year    = ts.tm_year;
	if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) errLedOn(__func__);
	else {
		sTime.Hours   = ts.tm_hour;
		sTime.Minutes = ts.tm_min;
		sTime.Seconds = ts.tm_sec;
		if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) errLedOn(__func__);
		else {
			setDate = true;
		}
	}

#endif
}
//-----------------------------------------------------------------------------
int sec_to_str_time(uint32_t sec, char *stx)
{
	uint32_t day = sec / (60 * 60 * 24);
	sec %= (60 * 60 * 24);
    uint32_t hour = sec / (60 * 60);
    sec %= (60 * 60);
    uint32_t min = sec / (60);
    sec %= 60;

    return (sprintf(stx, "%03lu.%02lu:%02lu:%02lu", day, hour, min, sec));
}
//-----------------------------------------------------------------------------
uint32_t get_secCounter()
{
	return secCounter;
}
//-----------------------------------------------------------------------------
void inc_secCounter()
{
	secCounter++;
}
//-----------------------------------------------------------------------------
uint64_t get_hsCounter()
{
	return HalfSecCounter;
}
//-----------------------------------------------------------------------------
void inc_hsCounter()
{
	HalfSecCounter++;
}
//------------------------------------------------------------------------------------------
uint32_t get_tmr(uint32_t sec)
{
	return (get_secCounter() + sec);
}
//------------------------------------------------------------------------------------------
bool check_tmr(uint32_t sec)
{
	return (get_secCounter() >= sec ? true : false);
}
//------------------------------------------------------------------------------------------
uint64_t get_hstmr(uint64_t hs)
{
	return (get_hsCounter() + hs);
}
//------------------------------------------------------------------------------------------
bool check_hstmr(uint64_t hs)
{
	return (get_hsCounter() >= hs ? true : false);
}
//------------------------------------------------------------------------------------------
// set LED_ERROR when error on and send message to UART1 (in from != NULL)
//     from - name of function where error location
void errLedOn(const char *from)
{
	HAL_GPIO_WritePin(GPIOD, LED_ERROR, GPIO_PIN_SET);//LED ON
	HAL_Delay(250);
	HAL_GPIO_WritePin(GPIOD, LED_ERROR, GPIO_PIN_RESET);//LED OFF
	HAL_Delay(250);
	HAL_GPIO_WritePin(GPIOD, LED_ERROR, GPIO_PIN_SET);//LED ON

	if (from) Report(true, "Error in '%s'\r\n", from);

}
//------------------------------------------------------------------------------------------
// set LED_ERROR when error on and send message to UART1 (in from != NULL)
//     act - action : true - ON, false - OFF
void Leds(bool act, uint16_t Pin)
{
	if (act) {
		HAL_GPIO_WritePin(GPIOD, Pin, GPIO_PIN_SET);//LED ON
		HAL_Delay(250);
		HAL_GPIO_WritePin(GPIOD, Pin, GPIO_PIN_RESET);//LED OFF
		HAL_Delay(250);
		HAL_GPIO_WritePin(GPIOD, Pin, GPIO_PIN_SET);//LED ON
	} else {
		HAL_GPIO_WritePin(GPIOD, Pin, GPIO_PIN_RESET);//LED OFF
	}
}
//----------------------------------------------------------------------------------------
int sec_to_string(uint32_t sec, char *stx, bool log)
{
int ret = 0;

#ifndef WITH_RTC
	uint32_t day = sec / (60 * 60 * 24);
	sec %= (60 * 60 * 24);
	uint32_t hour = sec / (60 * 60);
	sec %= (60 * 60);
	uint32_t min = sec / (60);
	sec %= 60;
	ret = sprintf(stx, "%03lu.%02lu:%02lu:%02lu", day, hour, min, sec);
#else
	if (!setDate) {//no valid date in RTC
		uint32_t day = sec / (60 * 60 * 24);
		sec %= (60 * 60 * 24);
		uint32_t hour = sec / (60 * 60);
		sec %= (60 * 60);
		uint32_t min = sec / (60);
		sec %= 60;
		ret = sprintf(stx, "%03lu.%02lu:%02lu:%02lu", day, hour, min, sec);
	} else {//in RTC valid date (epoch time)
		RTC_TimeTypeDef sTime;
		RTC_DateTypeDef sDate;
		if (HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) errLedOn(__func__);
		else {
			if (HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) errLedOn(__func__);
			else {
				if (log)
					ret = sprintf(stx, "%02u.%02u.%04u %02u:%02u:%02u",
							sDate.Date, sDate.Month, (uint16_t)(sDate.Year+1900),
							sTime.Hours, sTime.Minutes, sTime.Seconds);
				else
					ret = sprintf(stx, "%02u.%02u %02u:%02u:%02u",
							sDate.Date, sDate.Month,
							sTime.Hours, sTime.Minutes, sTime.Seconds);
			}
		}
	}
#endif
	if (log) {
		strcat(stx, " | ");
		ret += 3;
	}

    return ret;
}
//----------------------------------------------------------------------------------------
void Report(bool addTime, const char *fmt, ...)
{
HAL_StatusTypeDef er = HAL_OK;
size_t len = 256;

	char *buff = (char *)pvPortMalloc(len);//char *buff = (char *)pvPortMalloc(len); //vPortFree(buff);
	if (buff) {
		buff[0] = 0;
		int dl = 0, sz;
		va_list args;

		if (addTime) dl = sec_to_string(get_secCounter(), buff, true);
		sz = dl;
		va_start(args, fmt);
		sz += vsnprintf(buff + dl, len - dl, fmt, args);
		//
		if (osSemaphoreAcquire(binSemHandle, 2000) == osOK) {
			HAL_UART_Transmit_DMA(&huart3, (uint8_t *)buff, sz);
			while (HAL_UART_GetState(&huart3) != HAL_UART_STATE_READY) {
				if (HAL_UART_GetState(&huart3) == HAL_UART_STATE_BUSY_RX) break;
				osDelay(1);
			}
			osSemaphoreRelease(binSemHandle);
		} else er = HAL_ERROR;
		//
		va_end(args);

		vPortFree(buff);
	} else er = HAL_ERROR;

	if (er != HAL_OK) Leds(true, LED_RED_Pin);
}
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void initQ(s_msg_t *q)
{
	q->put = q->get = 0;
	for (uint8_t i = 0; i < MAX_QMSG; i++) {
		q->msg[i].id = i;
		q->msg[i].adr = NULL;
	}
}
//-----------------------------------------------------------------------------
int8_t putQ(char *adr, s_msg_t *q)
{
int8_t ret = -1;

	if (q->msg[q->put].adr == NULL) {
		ret = q->msg[q->put].id;
		q->msg[q->put].adr = adr;
		q->put++; if (q->put >= MAX_QMSG) q->put = 0;
	}

	return ret;
}
//-----------------------------------------------------------------------------
int8_t getQ(char *dat, s_msg_t *q)
{
int8_t ret = -1;

	if (q->msg[q->get].adr != NULL) {
		ret = q->msg[q->get].id;
		int len = strlen(q->msg[q->get].adr) & 0xff;
		memcpy(dat, q->msg[q->get].adr, len);
		*(dat + len) = '\0';
		free(q->msg[q->get].adr);
		q->msg[q->get].adr = NULL;
		q->get++; if (q->get >= MAX_QMSG) q->get = 0;
	}

	return ret;
}
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void getNMEA()
{
	GpsRxBuf[gps_rx_uk++] = (char)gRxByte;
	if (gRxByte == 0x0a) {//end of line
		char *uk = strstr(GpsRxBuf, _extRMC);//const char *_extRMC = "$GNRMC";
		if (uk) {
			rmc5++;
			if (rmc5 >= wait_sensor_def) {
				rmc5 = 0;
				int len = strlen(GpsRxBuf);
				char *buff = (char *)calloc(1, len + 1);
				if (buff) {
					memcpy(buff, GpsRxBuf, len);
					int8_t sta = putQ(buff, &q_gps);
					if (sta < 0) free(buff);//error !
					        else HAL_GPIO_WritePin(GPIO_PortD, LED_BLUE_Pin, GPIO_PIN_SET);//add to q_gps OK

					ssd1306_text_xy(GpsRxBuf, ssd1306_calcx(sprintf(GpsRxBuf, "RMC:%lu %d", ++rmcCounter, sta)), 3);
				}
			} else HAL_GPIO_WritePin(GPIO_PortD, LED_BLUE_Pin, GPIO_PIN_RESET);
		}
		gps_rx_uk = 0;
		memset(GpsRxBuf, 0, MAX_UART_BUF);
	}
}
//------------------------------------------------------------------------------------------
void getAT()
{
	if ((aRxByte > 0x0d) && (aRxByte < 0x80)) {
		if (aRxByte >= 0x20) adone = 1;
		if (adone) AtRxBuf[at_rx_uk++] = (char)aRxByte;
	}
	if (at_rx_uk > 0) {
		if ( ( (aRxByte == 0x0a) || (aRxByte == 0x3e) ) && adone) {
			if (aRxByte != 0x3e) strcat(AtRxBuf, "\r\n");
			int len = strlen(AtRxBuf);
			char *buff = (char *)calloc(1, len + 1);
			if (buff) {
				memcpy(buff, AtRxBuf, len);
				if (putQ(buff, &q_at) < 0) free(buff);
				if (strstr(AtRxBuf, "+CGNSINF:")) {
					char *buff2 = (char *)calloc(1, len + 1);
					if (buff2) {
						memcpy(buff2, AtRxBuf, len);
						if (putQ(buff2, &q_gps) < 0) free(buff2);
					}
				}
			}

			ssd1306_text_xy(AtRxBuf, ssd1306_calcx(sprintf(AtRxBuf, "MSG: %lu", ++atCounter)), 4);

			at_rx_uk = adone = 0;
			memset(AtRxBuf, 0, MAX_UART_BUF);
		}
	}

}
//------------------------------------------------------------------------------------------
void LogData()
{
	RxBuf[rx_uk++] = (char)lRxByte;
	if (lRxByte == 0x0d) {//end of line
		bool priz = false;
		char *uk = strstr(RxBuf, _extDate);//const char *_extDate = "DATE:";
		if (uk) {
#ifdef WITH_RTC
			uk += strlen(_extDate);
			if (*uk != '?') {
				if (strlen(uk) < 10) setDate = false;
				else {
					extDate = atoi(uk);
					set_Date((time_t)extDate);
				}
			} else setDate = true;
			evt_clear = true;
#endif
		} else {
			if (strstr(RxBuf, "GSM:")) {
				evt_gsm = true; priz = true;
			} else {
				if (strstr(RxBuf, "GPS:")) {
					flags.gps_log_show = ~flags.gps_log_show; priz = true;
				} else {
					if (strstr(RxBuf, "I2C:")) {
						flags.i2c_log_show = ~flags.i2c_log_show; priz = true;
					} else {
						if (strstr(RxBuf, "RESTART:")) {
							ssd1306_clear(); NVIC_SystemReset();
						} else {
							if (strstr(RxBuf, "AT")) {
								strcat(RxBuf, "\n"); priz = true;
							} else {
								if ((uk = strchr(RxBuf, '\r')) != NULL) *uk = '\0';
							}
							int len = strlen(RxBuf);
							char *buff = (char *)calloc(1, len + 1);
							if (buff) {
								memcpy(buff, RxBuf, len);
								if (putQ(buff, &q_cmd) < 0) free(buff);
							}
						}
					}
				}
			}

			if (priz) {
				ssd1306_clear_line(5);
				char *uke = strchr(RxBuf, '\r'); if (uke) *uke = '\0';
				if (strlen(RxBuf) > 16) RxBuf[16] = 0;
				ssd1306_text_xy(RxBuf, ssd1306_calcx(strlen(RxBuf)), 5);
			}
		}
		ClearRxBuf();
	}
}
//------------------------------------------------------------------------------------------
//  callback function when recv. data from all uart's
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	adrByte = NULL;
	if (huart->Instance == UART4 ) {//at_commands
		getAT();
		adrByte = &aRxByte;
	} else if (huart->Instance == UART5) {//gps_nmea_messages
		getNMEA();
		adrByte = &gRxByte;
	} else if (huart->Instance == USART3 ) {//log
		LogData();
		adrByte = &lRxByte;
	}
	if (adrByte) HAL_UART_Receive_IT(huart, adrByte, 1);
}
//------------------------------------------------------------------------------------------
void fConv(float in, conv_t *ic)
{
	float dro = (in - (uint16_t)in) * 100;
	ic->cel = (uint16_t)in;
	ic->dro = (uint16_t)dro;
}
//------------------------------------------------------------------------------------------
void mkMsgData(result_t *from, iresult_t *to)
{
conv_t tmp;

	to->chip = from->chip;
	fConv(from->humi, &tmp); to->humi.cel = tmp.cel; to->humi.dro = tmp.dro;
	fConv(from->lux, &tmp);  to->lux.cel  = tmp.cel; to->lux.dro  = tmp.dro;
	fConv(from->pres, &tmp); to->pres.cel = tmp.cel; to->pres.dro = tmp.dro;
	fConv(from->temp, &tmp); to->temp.cel = tmp.cel; to->temp.dro = tmp.dro;
}
//------------------------------------------------------------------------------------------
void AtParamInit()
{
	at_rx_uk = 0;
	memset(AtRxBuf, 0, MAX_UART_BUF);
	cmdsInd = -1;
	initQ(&q_at);
	initQ(&q_cmd);
	HAL_UART_Receive_IT(&huart4, (uint8_t *)&aRxByte, 1);//AT
	OnOffEn = true;
}
//------------------------------------------------------------------------------------------
void GpsParamInit()
{
	gps_rx_uk = 0;
	memset(GpsRxBuf, 0, MAX_UART_BUF);
	initQ(&q_gps);
	HAL_UART_Receive_IT(&huart5, (uint8_t *)&gRxByte, 1);//GPS
}
//------------------------------------------------------------------------------------------
int8_t parse_gps(char *in, s_gps_t *data)
{
//$--RMC,hhmmss.sss,x,llll.lll,a,yyyyy.yyy,a,x.x,u.u,xxxxxx,,,v*hh<CR><LF>
//$GNRMC,001805.868,V,,,,,0.00,0.00,060180,,,N*56

	char *uk = NULL, *uks = NULL, *uke = NULL, *porog = in + strlen(in);
	uks = strstr(in, "$GNRMC,");
	if (!uks) return 1;
	uks += 7;
	porog += 7;
	char tmp[32];
	uint8_t cnt = 0, len = 0;
	memset((uint8_t *)data, 0, sizeof(s_gps_t));
	while (1) {
		uke = strchr(uks, ',');
		if (!uke) uke = strchr(uks, '\r');
		if (uke) {//hhmmss.sss
			if (uke <= porog) {
				cnt++;
				len = (uke - uks);
				if (len > 0) {
					memset(tmp, 0, 32);
					memcpy(tmp, uks, len);
					switch (cnt) {
						case 1://time
							uk = strchr(tmp, '.');
							if (uk) {
								if (strlen(tmp) == 10) {
									data->ms = atoi(uk + 1);   *uk    = '\0';
									data->sec = atoi(&tmp[4]); tmp[4] = '\0';
									data->min = atoi(&tmp[2]); tmp[2] = '\0';
									data->hour = atoi(tmp);
								}
							}
							break;
						case 2://good
							if (tmp[0] == 'A') data->good = true;
							break;
						case 3://latitude
							data->latitude = (float)atof(tmp);
							break;
						case 4://latc
							if (tmp[0] == 'S') data->ns = true;
							break;
						case 5://longitude
							data->longitude = (float)atof(tmp);
							break;
						case 6://lonc
							if (tmp[0] == 'W') data->ew = true;
							break;
						case 7://speed
							data->speed = (float)atof(tmp);
							break;
						case 8://dir
							data->dir = (float)atof(tmp);
							break;
						case 9://UTC date
							if (strlen(tmp) == 6) {
								data->year = atoi(&tmp[4]); tmp[4] = '\0';
								data->mon  = atoi(&tmp[2]); tmp[2] = '\0';
								data->day = atoi(tmp);
							}
							break;
						case 12://mode+crc
							data->mode = tmp[0];
							uk = strchr(tmp, '*');
							if (uk) data->crc = hextobin(*(uk + 1), *(uk + 2));
							break;
					}
				}
				uks = uke + 1;
			} else break;
		} else break;
	}
	return 0;
}
//------------------------------------------------------------------------------------------
int8_t parse_inf(char *in, s_inf_t *inf)
{
//AT+CGNSINF
//+CGNSINF: 1,0,19800106002148.000,,,,0.00,0.0,0,,,,,,0,0,,,,,
//OK
	char *uk = NULL, *uks = NULL, *uke = NULL, *porog = in + strlen(in);
	uks = strstr(in, "+CGNSINF: ");
	if (!uks) return 1;
	uks += 10;
	porog += 10;
	char tmp[32];
	uint8_t cnt = 0, len = 0;
	memset((uint8_t *)inf, 0, sizeof(s_inf_t));
	while (1) {
		uke = strchr(uks, ',');
		if (!uke) uke = strchr(uks, '\r');
		if (uke) {//1,0,1980 01 06 00 21 48.000,,,,0.00,0.0,0,,,,,,0,0,,,,,
			if (uke <= porog) {
				cnt++;
				len = (uke - uks);
				if (len > 0) {
					memset(tmp, 0, 32);
					memcpy(tmp, uks, len);
					switch (cnt) {
						case 1://run
							inf->run = tmp[0] - 0x30;
							break;
						case 2://status
							inf->status = tmp[0] - 0x30;
							break;
						case 3://utp
							uk = strchr(tmp, '.');
							if (uk) {
								if (strlen(tmp) == 18) {
									inf->utc.ms   = atoi(uk + 1); *uk = '\0';
									inf->utc.sec  = atoi(&tmp[12]); tmp[12] = '\0';
									inf->utc.min  = atoi(&tmp[10]); tmp[10] = '\0';
									inf->utc.hour = atoi(&tmp[8]);  tmp[8]  = '\0';
									inf->utc.day  = atoi(&tmp[6]);  tmp[6]  = '\0';
									inf->utc.mon  = atoi(&tmp[4]);  tmp[4]  = '\0';
									inf->utc.year = atoi(&tmp[0]);
								}
							}
							break;
						case 4://latitude
							inf->latitude = (float)atof(tmp);
							break;
						case 5://longitude
							inf->longitude = (float)atof(tmp);
							break;
						case 6://altitude
							inf->altitude = atoi(tmp);
							break;
						case 7://speed
							inf->speed = (float)atof(tmp);
							break;
						case 8://dir
							inf->dir = (float)atof(tmp);
							break;
						case 9://mode
							inf->mode = tmp[0] - 0x30;
							break;
						case 11://float HDOP;
							inf->HDOP = (float)atof(tmp);
							break;
						case 12://float PDOP
							inf->PDOP = (float)atof(tmp);
							break;
						case 13://float VDOP
							inf->VDOP = (float)atof(tmp);
							break;
						case 15://uint8_t GPSsatV
							inf->GPSsatV = atoi(tmp);
							break;
						case 16://uint8_t GNSSsatU
							inf->GNSSsatU = atoi(tmp);
							break;
						case 17://uint8_t GLONASSsatV
							inf->GLONASSsatV = atoi(tmp);
							break;
						case 19://uint8_t dBHz
							inf->dBHz = atoi(tmp);
							break;
						case 20://float HPA
							inf->HPA = (float)atof(tmp);
							break;
						case 21://float VPA
							inf->VPA = (float)atof(tmp);
							break;
					}
				}
				uks = uke + 1;
			} else break;
		} else break;
	}
	return 0;
}

//------------------------------------------------------------------------------------------

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefTask */
/**
  * @brief  Function implementing the defTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefTask */
void StartDefTask(void *argument)
{

  /* USER CODE BEGIN 5 */

  	uint8_t row;
  	char toScreen[SCREEN_SIZE];
  	result_t levt;
  	iresult_t evt;
  	uint8_t show;

  /* Infinite loop */

  	while (1)   {
  		/**/
  		if (mailQueueHandle) {
  			if (osMessageQueueGet(mailQueueHandle, (void *)&levt, NULL, 100) == osOK) {
  				mkMsgData(&levt, &evt);
  				show = flags.i2c_log_show;
  				row = 6;
  				sprintf(toScreen, "mmHg : %u.%02u\nDegC : %u.%02u", evt.pres.cel, evt.pres.dro, evt.temp.cel, evt.temp.dro);
  				switch (evt.chip) {
  					case BMP280_SENSOR :
  						sprintf(toScreen, "mmHg : %u.%02u\nDegC : %u.%02u", evt.pres.cel, evt.pres.dro, evt.temp.cel, evt.temp.dro);
  						if (show) Report(true, "BMP280: Press=%u.%02u mmHg, Temp=%u.%02u DegC; BH1750: Lux=%u.%02u lx\r\n",
  										evt.pres.cel, evt.pres.dro, evt.temp.cel, evt.temp.dro, evt.lux.cel, evt.lux.dro);
  				  	break;
  					case BME280_SENSOR :
  						row = 5;
  						sprintf(toScreen, "mmHg : %u.%02u\nDegC : %u.%02u\nHumi: %u.%02u %%rH",
  										evt.pres.cel, evt.pres.dro, evt.temp.cel, evt.temp.dro, evt.humi.cel, evt.humi.dro);
  				  		if (show) Report(true, "BME280: Press=%u.%02u mmHg, Temp=%u.%02u DegC Humidity=%u.%02u %%rH; BH1750: Lux=%u.%02u lx\r\n",
  				  						evt.pres.cel, evt.pres.dro, evt.temp.cel, evt.temp.dro, evt.humi.cel, evt.humi.dro, evt.lux.cel, evt.lux.dro);
  				  	break;
  				  		default : {
  				  			sprintf(toScreen, "\n\n");
  				  			if (show) Report(true, "Unknown chip; BH1750: Lux=%u.%02u lx\r\n", evt.lux.cel, evt.lux.dro);
  				  		}
  				}
  				//
  				if (i2cError == HAL_OK) {
  					sprintf(toScreen+strlen(toScreen), "\nLux  : %u.%02u", evt.lux.cel, evt.lux.dro);
  					ssd1306_text_xy(toScreen, 1, row);//send string to screen
  				}
  				//
  			}
  		}
		/**/
  		osDelay(100);
  	}
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartSensTask */
/**
* @brief Function implementing the sensTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensTask */
void StartSensTask(void *argument)
{
  /* USER CODE BEGIN StartSensTask */

	uint8_t data_rdx[DATA_LENGTH] = {0};
	uint8_t reg_id   = 0;
	uint8_t reg_stat = 0;
	uint8_t reg_mode = 0;
	uint8_t reg_conf = 0;
	size_t d_size    = 1;
	int32_t temp, pres, humi = 0;

	result_t sens;

	uint16_t lux = 0;
	float lx = -1.0;

  /* Infinite loop */

	uint32_t wait_sensor = get_tmr(6);

	bh1750_on_mode();

	while (1) {
		if (check_tmr(wait_sensor)) {
			wait_sensor = get_tmr(wait_sensor_def);
			//--------------------  BH1750  ---------------------------------
			lx = -1.0;
			if (bh1750_proc(&lux) == HAL_OK) lx = lux / 1.2;
			sens.lux = lx;
			//--------------------  BMP280  ---------------------------------
			if (i2c_master_reset_sensor(&reg_id) != HAL_OK) {
				wait_sensor = get_tmr(2);
				continue;
			}
			switch (reg_id) {
				case BMP280_SENSOR : d_size = 6; break;
				case BME280_SENSOR : d_size = 8; break;
				default : {
					wait_sensor = get_tmr(2);
					continue;
				}
			}
			if (i2c_master_test_sensor(&reg_stat, &reg_mode, &reg_conf, reg_id) != HAL_OK) {
				wait_sensor = get_tmr(2);
				continue;
			}
			reg_stat &= 0x0f;
			memset(data_rdx, 0, DATA_LENGTH);
			if (i2c_master_read_sensor(BMP280_REG_PRESSURE, data_rdx, d_size) == HAL_OK) {
				if (bmp280_readCalibrationData(reg_id) == HAL_OK) {
					pres = temp = 0;
					pres = (data_rdx[0] << 12) | (data_rdx[1] << 4) | (data_rdx[2] >> 4);
					temp = (data_rdx[3] << 12) | (data_rdx[4] << 4) | (data_rdx[5] >> 4);
					if (reg_id == BME280_SENSOR) humi = (data_rdx[6] << 8) | data_rdx[7];
					bmp280_CalcAll(&sens, reg_id, temp, pres, humi);
					//
					if (mailQueueHandle) osMessageQueuePut(mailQueueHandle, (void *)&sens, 0, 10);
					//
				}
			}
		}
		//
		osDelay(100);
	}
  /* USER CODE END StartSensTask */
}

/* USER CODE BEGIN Header_StartAtTask */
/**
* @brief Function implementing the atTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAtTask */
void StartAtTask(void *argument)
{
  /* USER CODE BEGIN StartAtTask */

	AtParamInit();
	gsmONOFF();

	char *uki = msgCMD;
	const char *buff = NULL;
	bool cmdsReady = false;
	uint8_t counter = 0;
	uint32_t wait_ack = 0;
	uint64_t new_cmds = 0;

	/**/
	char toScr[SCREEN_SIZE];
	size_t fmem = xPortGetFreeHeapSize();
	ssd1306_text_xy(toScr, ssd1306_calcx(sprintf(toScr, "f:%u on:%u.%u", fmem, onGSM, onGNS)), 2);
	/**/

	/* Infinite loop */
	while (1) {

		if (getQ(msgAT, &q_at) >= 0) {
			if (strstr(msgAT, "NORMAL POWER DOWN")) {
				onGSM = false;
				if (!wait_ack) wait_ack = get_tmr(3);
			} else if (strlen(msgAT)) {
				onGSM = true;
				if (strstr(msgAT, "RDY")) counter++;
				else if (strstr(msgAT, "+CFUN:")) counter++;
				else if (strstr(msgAT, "+CPIN: NOT INSERTED")) counter = 5;
				else if (strstr(msgAT, "+CPIN: READY")) counter++;
				else if (strstr(msgAT, "Call Ready")) counter++;
				else if (strstr(msgAT, "SMS Ready")) counter++;
				else if (strstr(msgAT, "ERROR") ||
							strstr(msgAT, "OK") ||
								strchr(msgAT, '>') ||
									strstr(msgAT, "+BTSCAN: 1")) {// ||
										//strstr(msgAT, "+CGNSINF:")) {
					wait_ack = 0;
					cmdsDone = true;
					if (cmdsInd >= 0) {
						cmdsInd++;
						new_cmds = get_hstmr(1);//250 ms
					}
				} else if ((uki = strstr(msgAT, "+CGNSPWR: ")) != NULL) {
					if (*(uki + 10) == '1') onGNS = true;
									   else onGNS = false;
				}

			}
			if (counter >= 5) {
				counter = 0;
				cmdsInd = 0;
				cmdsDone = true;
				new_cmds = get_hstmr(8);//3 sec
			}
			Report(false, msgAT);
			/**/
			//if (fmem != xPortGetFreeHeapSize()) {
				fmem = xPortGetFreeHeapSize();
				ssd1306_text_xy(toScr, ssd1306_calcx(sprintf(toScr, "f:%u on:%u.%u", fmem, onGSM, onGNS)), 2);
			//}
			/**/

		}

		if (cmdsInd >= 0) {
			if (cmdsInd >= cmdsMax) {
				cmdsInd = -1;
				flags.gps_log_show = 1;
				flags.i2c_log_show = 1;
			} else {
				if (cmdsDone) {
					if (check_hstmr(new_cmds)) {
						cmdsReady = true;
						buff = &cmds[cmdsInd][0];
						Report(false, "%.*s", strlen(cmds[cmdsInd]), cmds[cmdsInd]);
					}
				}
			}
		}

		if (cmdsDone) {
			if (getQ(msgCMD, &q_cmd) >= 0) {//send at command to sim868
				cmdsReady = true;
				buff = msgCMD;
				//Report(false, msgCMD);
			}
		}

		if (cmdsReady && cmdsDone) {
			cmdsReady = false;
			HAL_UART_Transmit_DMA(&huart4, (uint8_t *)buff, strlen(buff));
			while (HAL_UART_GetState(&huart4) != HAL_UART_STATE_READY) {
				if (HAL_UART_GetState(&huart4) == HAL_UART_STATE_BUSY_RX) break;
				osDelay(1);
			}
			cmdsDone = false;
			if (strstr(buff, "AT+BTSPPSEND="))
				wait_ack = get_tmr(30);//WAIT ACK 30 SEC
			else
				wait_ack = get_tmr(10);//WAIT ACK 10 SEC
		}


		if (wait_ack) {
			if (check_tmr(wait_ack)) {
				evt_gsm = true;
				wait_ack = 0;
			}
		}

		if (evt_gsm) {
			evt_gsm = false;
			AtParamInit();
			flags.gps_log_show = flags.i2c_log_show = 0;
			gsmONOFF();
		}

		osDelay(2);
	}

  /* USER CODE END StartAtTask */
}

/* USER CODE BEGIN Header_StartGpsTask */
/**
* @brief Function implementing the gpsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGpsTask */
void StartGpsTask(void *argument)
{
  /* USER CODE BEGIN StartGpsTask */

	GpsParamInit();
	s_gps_t one;
	s_inf_t inf;
	bool ns = false;
	bool ew = false;

	/*
	AT+CGNSCMD=0,"$PMTK251,115200*1F"
	AT+CGNSCMD=0,"$PMTK251,38400*27"
	AT+CGNSCMD=0,"$PMTK251,19200*22"
	AT+CGNSCMD=0,"$PMTK251,9600*17"
	AT+CGNSCMD=0,"$PMTK251,0*28"
	*/
	//char *buff = (char *)pvPortMalloc(len); //vPortFree(buff);
	jfes_config_t jconf = {
		.jfes_malloc = (jfes_malloc_t)pvPortMalloc,//malloc
		.jfes_free = vPortFree//free
	};

	jfes_value_t *obj;
	jfes_size_t stx_size = MAX_UART_BUF - 3;

  /* Infinite loop */

	while (1) {

		if (getQ(msgNMEA, &q_gps) >= 0) {
			if (flags.gps_log_show) {
				Report(true, msgNMEA);
				if (!parse_gps(msgNMEA, &one)) {
					obj = jfes_create_object_value(&jconf);
					if (obj) {
						sprintf(msgNMEA, "%02u.%02u.%02u %02u:%02u:%02u.%03u",
								one.day, one.mon, one.year, one.hour, one.min, one.sec, one.ms);
						jfes_set_object_property(&jconf, obj, jfes_create_string_value(&jconf, msgNMEA, 0), "UTC", 0);
						jfes_set_object_property(&jconf, obj, jfes_create_string_value(&jconf, nameValid[one.good], 0), "Data", 0);
						if (one.ns) one.latitude *= -1.0;
						jfes_set_object_property(&jconf, obj, jfes_create_double_value(&jconf, (double)one.latitude), "Latitude", 0);
						if (one.ew) one.longitude *= -1.0;
						jfes_set_object_property(&jconf, obj, jfes_create_double_value(&jconf, (double)one.longitude), "Longitude", 0);
						jfes_set_object_property(&jconf, obj, jfes_create_double_value(&jconf, (double)one.speed), "Speed", 0);
						jfes_set_object_property(&jconf, obj, jfes_create_double_value(&jconf, (double)one.dir), "Dir", 0);
						sprintf(msgNMEA, "%c", one.mode);
						jfes_set_object_property(&jconf, obj, jfes_create_string_value(&jconf, msgNMEA, 0), "Mode", 0);
						sprintf(msgNMEA, "%02X", one.crc);
						jfes_set_object_property(&jconf, obj, jfes_create_string_value(&jconf, msgNMEA, 0), "CRC", 0);

						memset(msgNMEA, 0, MAX_UART_BUF);
						jfes_value_to_string(obj, msgNMEA, &stx_size, 1);
						strcat(msgNMEA, "\r\n");

						Report(false, msgNMEA);

						jfes_free_value(&jconf, obj);
					}/* else
						Report(false,
						"\tUTC=%02u.%02u.%02u %02u:%02u:%02u.%03u '%s data'\r\n\tLatitude=%.6f^ (%s)\r\n\tLongitude=%.6f^ (%s)\r\n\t"
						"Speed=%.2f km/h\r\n\tDir=%.2f^\r\n\tMode=%c\r\n\tCRC=%02X\r\n",
						one.day, one.mon, one.year,
						one.hour, one.min, one.sec, one.ms,
						nameValid[one.good],
						one.latitude, nameNS[one.ns],
						one.longitude, nameEW[one.ew],
						one.speed, one.dir,
						one.mode, one.crc);*/
				}
			}
			if (!parse_inf(msgNMEA, &inf)) {
				if (inf.latitude < 0) ns = true; else ns = false;
				if (inf.longitude < 0) ew = true; else ew = false;
				Report(false,
						"\tUTC=%02u.%02u.%04u %02u:%02u:%02u.%03u rs:%u/%u\r\n\tLatitude=%f^ (%s)\r\n\tLongitude=%f^ (%s)\r\n\tAltitude=%d m\r\n\tSpeed=%.2f km/h\r\n\tDir=%.2f^\r\n\tMode=%u\r\n\t"
						"HDOP=%.1f PDOP=%.1f VDOP=%.1f\r\n\tSat:%u/%u/%u\r\n\tdBHz=%u\r\n\tHPA=%.1f VPA=%.1f\r\n",
						inf.utc.day, inf.utc.mon, inf.utc.year, inf.utc.hour, inf.utc.min, inf.utc.sec, inf.utc.ms,
						inf.run, inf.status,
						inf.latitude, nameNS[ns], inf.longitude, nameEW[ew], inf.altitude,
						inf.speed, inf.dir, inf.mode,
						inf.HDOP, inf.PDOP, inf.VDOP,
						inf.GPSsatV, inf.GNSSsatU, inf.GLONASSsatV,
						inf.dBHz, inf.HPA, inf.VPA);
			}

		}

		osDelay(50);

	}

  /* USER CODE END StartGpsTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  else
  if (htim->Instance == TIM2) {//interrupt every 250 ms
	  if ((get_hsCounter() & 3) == 3) {//seconda
		  inc_secCounter();
		  HAL_GPIO_TogglePin(GPIOD, LED_GREEN_Pin);

		  if (!i2cError) {
#ifdef WITH_RTC
			  if (evt_clear) {
				  evt_clear = false;
				  ssd1306_clear_line(1);
			  }
#endif
			  char scrBuf[32];
			  ssd1306_text_xy(scrBuf, ssd1306_calcx(sec_to_string(get_secCounter(), scrBuf, false)), 1);
		  }
		  if (OnOffEn) {
			  if (!HAL_GPIO_ReadPin(GSM_STAT_GPIO_Port, GSM_STAT_Pin))
				  HAL_GPIO_WritePin(GPIO_PortD, LED_ORANGE_Pin, GPIO_PIN_SET);//gsm is on
			  else
				  HAL_GPIO_WritePin(GPIO_PortD, LED_ORANGE_Pin, GPIO_PIN_RESET);//gsm is off
		  }
	  }
	  inc_hsCounter();

  }

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
