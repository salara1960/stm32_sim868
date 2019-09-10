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

#include "libs.h"
#include "bh1750.h"

#ifdef SET_SMS
	#include "sms.h"
#endif

#ifdef SET_W25FLASH
	#include "w25.h"
#endif

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
//const char *ver = "ver 1.5rc1";//17.05.2019 - major changes : add json library !!!
//const char *ver = "ver 1.6rc1";//18.05.2019 - minor changes : for USART3 set speed=500000
								 // in StartGpsTask make json report for +CGNSINF: ......
								 // set MAX_UART_BUF = 400 bytes
//const char *ver = "ver 1.6rc2";//19.05.2019 - minor changes : add set DateTime (Unix epoch time), for example DATE:1558262631
//const char *ver = "ver 1.6rc3";//19.05.2019 - minor changes : usart2 instead of uart4 (GPS port)
//const char *ver = "ver 1.7rc1";//20.05.2019 - major changes : add SD Card (SPI1) !!! (first step : list of root file system)
//const char *ver = "ver 1.7rc2";//20.05.2019 - minor changes : all files content print <- second step.
//const char *ver = "ver 1.7rc3";//21.05.2019 - minor changes : write log file 'sensors.txt' to SD Card <- step 3.
//const char *ver = "ver 1.7rc4";//22.05.2019 - minor changes : in support SD card, data from sensors in json format <- step 4.
//const char *ver = "ver 1.7rc5";//22.05.2019 - minor changes : support next command : 'DATE:','MOUNT:','UMNT:','LIST:','RESTART:','STOP','GPS:','GSM:','I2C:' <- step 5.
//const char *ver = "ver 1.8rc1";//23.05.2019 - major changes : set UART4_TX from PA0 to PC10 (PA0 now for command 'STOP')
//const char *ver = "ver 1.9rc1";//24.05.2019 - major changes : add RTC (rtcclk = 320KHz)
//const char *ver = "ver 2.0rc1";//25.05.2019 - major changes : add spi OLED SSD1306 (SPI3), remove i2c OLED SSD1306 (i2c1)
//const char *ver = "ver 2.0rc2";//27.05.2019 - minor changes : add DevID (imei) to json object
//const char *ver = "ver 2.1rc1";//31.05.2019 - major changes : remove SD card from project (it's new branch 'withoutSD')
//const char *ver = "ver 2.1rc2";//31.05.2019 - minor changes : edit without RTC mode
//const char *ver = "ver 2.2rc1";//03.06.2019 - major changes : create gprs connection and send sensor's type json messages
//const char *ver = "ver 2.2rc2";//03.06.2019 - minor changes : make functions for make json_string for data
//const char *ver = "ver 2.2rc3";//04.06.2019 - minor changes : edit gprs connection mode (add commands CON: DIS:)
//const char *ver = "ver 2.2rc4";//05.06.2019 - minor changes : add commands :SRV=srv_adr:srv_port (for example : SRV:127.0.0.1:9000)
//const char *ver = "ver 2.2rc5";//05.06.2019 - minor changes : add GPS json_string to GPRS_queue (for sending to tcp_server)
//const char *ver = "ver 2.2rc6";//05.06.2019 - minor changes : add function  toDisplay(...);
//const char *ver = "ver 2.3rc1";//06.06.2019 - major changes : remove GpsTask, all gps support now moved to AtTask
//const char *ver = "ver 2.3rc2";//07.06.2019 - minor changes : add new at commands to 'play_list'
//const char *ver = "ver 2.3rc3";//09.06.2019 - minor changes in toDisplay(...) function
//const char *ver = "ver 2.3rc4";//10.06.2019 - minor changes+
//const char *ver = "ver 2.4rc1";//10.06.2019 - minor changes++
//const char *ver = "ver 2.5rc1";//11.06.2019 - major changes in AtTask
//const char *ver = "ver 2.5rc2";//11.06.2019 - minor changes+ in AtTask
//const char *ver = "ver 2.5rc3";//13.06.2019 - minor changes : edit screen font (0x14, 0x15)
//const char *ver = "ver 2.5rc4";//01.07.2019 - minor changes : add sim number, set one config struct for jfes
//const char *ver = "ver 2.6rc1";//02.07.2019 - major changes : remove jfes library (reason - memory leak in library)
//const char *ver = "ver 2.7rc1";//02.07.2019 - major changes : unused gps serial port(USART2), used AT+CGNSINF command for getting gps data
//const char *ver = "ver 2.7rc2";//03.07.2019 - major changes : remove USART2 and GSM_STATUS_pin (PA2) from project
//const char *ver = "ver 2.7rc3";//03.07.2019 - minor changes+++
//const char *ver = "ver 2.8rc1";//04.07.2019 - major changes : add JFES library (in #ifdef mode), fixed memory leak bug
//const char *ver = "ver 2.8rc2";//05.07.2019 - minor changes++++
//const char *ver = "ver 2.8rc3";//13.07.2019 - minor changes+++++
//const char *ver = "ver 2.9rc1";//15.07.2019 - major changes : add VIO -> GSM_STATUS pin (PA2) - pin42 module sim868
//const char *ver = "ver 3.0rc1";//16.07.2019 - major changes : add sms support - step 1 ('+CMT:' mode)
//const char *ver = "ver 3.0rc2";//17.07.2019 - minor changes : '+CMT:' continue - support Data coding: GSM7bit
//const char *ver = "ver 3.0rc3";//20.07.2019 - minor changes : sms mode edit continue
//const char *ver = "ver 3.1rc1";//21.07.2019 - minor changes : fixed bug in callback function (at_commands port of sim868)
//const char *ver = "ver 3.1rc2";//22.07.2019 - minor changes : some changes in sms parser + change command format - :CMD (CMD=OFF,ON,...)
//const char *ver = "ver 3.1rc3";//23.07.2019 - minor changes : add : log_show flag, RSSI to json, new commands - :LOG? , :LOG
//const char *ver = "ver 3.1rc4";//23.07.2019 - minor changes : show RSSI in dBm
//const char *ver = "ver 3.2rc1";//30.07.2019 - minor changes : add SMS_CONCAT mode - step 1
//const char *ver = "ver 3.2rc2";//31.07.2019 - minor changes : SMS_CONCAT mode done !
//const char *ver = "ver 3.2rc3";//31.07.2019 - minor changes : fixed bugs in calc. sms len
//const char *ver = "ver 3.2rc4";//01.08.2019 - minor changes :  implemented sending to tcp-server received SMS
//const char *ver = "ver 3.2rc5";//01.08.2019 - minor changes :  add queue for SMS
//const char *ver = "ver 3.2rc6";//02.08.2019 - minor changes : add static body mode in SMS queue, remove link option -specs=nosys.specs ! <- now calloc working !!!
//const char *ver = "ver 3.3rc1";//03.08.2019 - minor changes : set HEAP_SIZE up to 32K, use JFES library, fixed memory leak bug
//const char *ver = "ver 3.3rc2";//08.08.2019 - minor changes : check sms - in sms command present ?
//const char *ver = "ver 3.4rc1";//08.08.2019 - minor changes : remove inf_data's queue, put inf_data to sms queue, support cmd via sms
//const char *ver = "ver 3.4rc2";//09.08.2019 - minor changes : unused variables removed, add semaphore 'msgSem' for queue 'recq'
//const char *ver = "ver 3.5rc1";//10.08.2019 - minor changes : move sms functions to files (source and header)
//const char *ver = "ver 3.5rc2";//10.08.2019 - minor changes : edit gsmONOFF(ms) - ON/OFF function for sim868
//const char *ver = "ver 3.6rc1";//14.08.2019 - minor changes : move any general functions to files (source and header)
//const char *ver = "ver 3.6rc2";//15.08.2019 - minor changes : testing with tcp-server (srv with kernel timer device driver)
//const char *ver = "ver 3.7rc1";//17.08.2019 - minor changes for sms recv. : convert ucs2 to utf8 done !!!
//const char *ver = "ver 3.8rc1";//09.09.2019 - major changes : add data flash chip W25Q64 (used SPI1 and PA4-CS_CHIP pin ) - first step
const char *ver = "ver 3.8rc2";//10.09.2019 - major changes : read data flash chip ID - next step


/*
post-build steps command:
arm-none-eabi-objcopy -O binary "${BuildArtifactFileBaseName}.elf" "${BuildArtifactFileBaseName}.bin" && ls -la | grep "${BuildArtifactFileBaseName}.*"

LINK:
-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -u_printf_float -T"../STM32F407VGTx_FLASH.ld" -Wl,-Map=output.map -Wl,--gc-sections -lm

-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -u_printf_float -specs=nosys.specs -T"../STM32F407VGTx_FLASH.ld" -Wl,-Map=output.map -Wl,--gc-sections -lm

-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -specs=nosys.specs -specs=nano.specs -u_printf_float -T"../STM32F407VGTx_FLASH.ld" -Wl,-Map=output.map -Wl,--gc-sections -lm
-specs=nano.specs
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

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi3_tx;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_usart3_tx;

osThreadId_t defTaskHandle;
osThreadId_t sensTaskHandle;
osThreadId_t atTaskHandle;
osMessageQueueId_t mailQueueHandle;
osSemaphoreId_t binSemHandle;
/* USER CODE BEGIN PV */

osSemaphoreId_t msgSem;

uint32_t infCounter = 0;

const uint32_t ModuleOFF = 1650;
const uint32_t ModuleON  = 1150;

const char *dev_name = "STM32_SIM868";
char devID[size_imei + 1] = {0};//imei of gsm module

const uint32_t min_wait_ms = 150;
const uint32_t max_wait_ms = 1000;
HAL_StatusTypeDef i2cError = HAL_OK;
I2C_HandleTypeDef *portBMP;
#ifdef SET_OLED_I2C
	I2C_HandleTypeDef *portSSD;
#endif
I2C_HandleTypeDef *portBH;//hi2c1;
UART_HandleTypeDef *portAT;//huart4;
UART_HandleTypeDef *portLOG;//huart3;
#ifdef SET_OLED_SPI
	SPI_HandleTypeDef *portOLED;//hspi3;
#endif

#ifdef SET_W25FLASH
SPI_HandleTypeDef *portFLASH = NULL;//hspi1;
#endif

static const char *_extDate = ":DATE=";
bool evt_clear = false;
bool setDate = false;
#ifndef SET_RTC_TMR
	static bool epochSet = false;
#endif

static char DefBuf[MAX_UART_BUF];

static char RxBuf[MAX_UART_BUF];
volatile uint16_t rx_uk;
uint8_t lRxByte;

volatile uint8_t rmc5 = wait_sensor_def - 10;//12;//8;//6;
static s_msg_t q_gps;

bool onGNS = false;
const char *strOnOff[] = {"OFF", "ON"};

char AtRxBuf[MAX_UART_BUF];
volatile uint16_t at_rx_uk;
uint8_t aRxByte;
uint8_t adone = 0;
char msgAT[MAX_UART_BUF] = {0};
s_msg_t q_at;
uint8_t evt_gsm = 0;
bool ackYes = false;

int8_t cmdsInd = -1;
volatile bool cmdsDone = true;
char msgCMD[MAX_UART_BUF] = {0};
s_msg_t q_cmd;
const uint8_t cmdsMax = 11;
const char *cmds[] = {
	"AT\r\n",
	"AT+CMEE=0\r\n",
	"AT+GMR\r\n",//get version of FW
	"AT+GSN\r\n",//get IMEI
	"AT+CNMI=1,2,0,1,0\r\n",
	"AT+SCLASS0=0\r\n",
	"AT+CMGF=0\r\n",
	"AT+CGNSPWR=1\r\n",// power for GPS/GLONASS ON
	"AT+CGNSPWR?\r\n",//check power for GPS/GLONASS status
	"AT+CREG?\r\n",
	"AT+CSQ\r\n"//get RSSI
};

const char *sim_num = "+79062100000";
const char *srv_adr_def = "aaa.bbb.ccc.ddd";
const uint16_t srv_port_def = 9192;
char srv_adr[16] = {0};
uint16_t srv_port;

const char *gprsDISCONNECT = "AT+CIPCLOSE\r\n";
char msgGPRS[MAX_UART_BUF] = {0};
const char *conStatus[] = {"Discon.", "Connect"};

volatile bool LoopAll = true;
uint8_t *adrByte = NULL;
volatile s_flags flags = {0};
volatile s_gprs_stat gprs_stat = {0};
s_gsm_stat gsm_stat = {0};
const char *gpsINF = "AT+CGNSINF\r\n";
const char *atCSQ = "AT+CSQ\r\n";
bool con_dis = 0;

static s_recq_t recq;

#ifdef SET_JFES
	jfes_config_t conf;
	jfes_config_t *jconf = NULL;
#endif


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_UART4_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI3_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
void StartDefTask(void *argument); // for v2
void StartSensTask(void *argument); // for v2
void StartAtTask(void *argument); // for v2

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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_UART4_Init();
  MX_USART3_UART_Init();
  MX_SPI3_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  portBMP = &hi2c1;//BMP280
#ifdef SET_OLED_I2C
  portSSD = &hi2c1;//I2C1 - OLED ssd1306

  i2c_ssd1306_on(true);//screen ON
  if (!i2cError) {
	  i2c_ssd1306_init();//screen INIT
	  if (!i2cError) {
		  i2c_ssd1306_pattern();//set any params for screen
		  if (!i2cError) i2c_ssd1306_clear();//clear screen
	  }
  }
#endif
  portBH  = &hi2c1;//BH1750
  portAT  = &huart4;//AT
  portLOG = &huart3;//LOG
#ifdef SET_OLED_SPI
  portOLED = &hspi3;//SPI3 - OLED SSD1306
  spi_ssd1306_Reset();
  spi_ssd1306_on(true);//screen ON
  spi_ssd1306_init();//screen INIT
  spi_ssd1306_pattern();//set any params for screen
  spi_ssd1306_clear();//clear screen
#endif

#ifdef SET_W25FLASH
  portFLASH = &hspi1;//SPI1 - W25Q64 data flash chip
#endif

    bh1750_off();

    // start timer2 + interrupt
    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_Base_Start_IT(&htim2);

    strcpy(srv_adr, srv_adr_def);
    srv_port = srv_port_def;

#ifdef SET_JFES
    conf.jfes_malloc = (jfes_malloc_t)getMem;//pvPortMalloc,
    conf.jfes_free = freeMem;//vPortFree;
    jconf = &conf;
#endif


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

  const osSemaphoreAttr_t msgSem_attributes = {
  		.name = "msgSem"
  };
  msgSem = osSemaphoreNew(1, 1, &msgSem_attributes);


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
  	  /*
  	  const osMessageQueueAttr_t mqData_attributes = {
  		.name = "mqAllData"
  	  };
  	  mqData = osMessageQueueNew(MAX_QMSG, sizeof(s_data_t), &mqData_attributes);
	  */
  	  HAL_Delay(1500);
  	  rx_uk = 0;
  	  memset(RxBuf, 0, MAX_UART_BUF);
  	  HAL_UART_Receive_IT(portLOG, (uint8_t *)&lRxByte, 1);//LOG

  	  initRECQ(&recq);//char *rc = (char *)pvPortMalloc(len); vPortFree(rc);
  					  //char *rc = (char *)calloc(1, len); free(rc);

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defTask */
  const osThreadAttr_t defTask_attributes = {
    .name = "defTask",
    .priority = (osPriority_t) osPriorityAboveNormal,
    .stack_size = 2048
  };
  defTaskHandle = osThreadNew(StartDefTask, NULL, &defTask_attributes);

  /* definition and creation of sensTask */
  const osThreadAttr_t sensTask_attributes = {
    .name = "sensTask",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 1024
  };
  sensTaskHandle = osThreadNew(StartSensTask, NULL, &sensTask_attributes);

  /* definition and creation of atTask */
  const osThreadAttr_t atTask_attributes = {
    .name = "atTask",
    .priority = (osPriority_t) osPriorityHigh,
    .stack_size = 6144
  };
  atTaskHandle = osThreadNew(StartAtTask, NULL, &atTask_attributes);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_HSE_DIV25;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 2499;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
    
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date 
  */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

	ClearMyTicks();

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  // htim2.Init.Prescaler = 41999; - 500ms
  // htim2.Init.Period = 124; - 500ms / 2 = 250 ms
  // 4 interrupt for one seconda period

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 41999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 124;
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
//			AT port
  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

//#ifdef GSM_KEY_Pin
//	HAL_GPIO_WritePin(GPIO_PortD, GSM_KEY_Pin, GPIO_PIN_RESET);//set 1
//#endif

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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */
//			LOG port
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
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  HAL_GPIO_WritePin(GPIOA, GSM_KEY_Pin|W25_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED_GREEN_Pin|LED_ORANGE_Pin|LED_RED_Pin|LED_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OLED_RST_GPIO_Port, OLED_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OLED_CS_Pin|OLED_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : USER_IN_Pin GSM_STATUS_Pin */
  GPIO_InitStruct.Pin = USER_IN_Pin|GSM_STATUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : GSM_KEY_Pin W25_CS_Pin */
  GPIO_InitStruct.Pin = GSM_KEY_Pin|W25_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_GREEN_Pin LED_ORANGE_Pin LED_RED_Pin LED_BLUE_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|LED_ORANGE_Pin|LED_RED_Pin|LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OLED_RST_Pin */
  GPIO_InitStruct.Pin = OLED_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OLED_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OLED_CS_Pin OLED_DC_Pin */
  GPIO_InitStruct.Pin = OLED_CS_Pin|OLED_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

//******************************************************************************************
//******************************************************************************************
//******************************************************************************************

void getAT()
{
//  0x0d 0x0a ....... 0x0d 0x0a
	if ((aRxByte > 0x0d) && (aRxByte < 0x80)) {
		if (aRxByte >= 0x20) adone = 1;
		if (adone) AtRxBuf[at_rx_uk++] = (char)aRxByte;
	}

	if (adone) {
		if ( (aRxByte == 0x0a) || (aRxByte == 0x3e) ) {// '\n' || '>'
			if (aRxByte != 0x3e) strcat(AtRxBuf, "\r\n");//0x0D 0x0A
			if (LoopAll) {//-----------------------------------------------------
				int len = strlen(AtRxBuf);
				char *buff = (char *)calloc(1, len + 1);
				if (buff) {
					int8_t sta;
					if (strlen(buff) != (len + 1)) {
						HAL_GPIO_WritePin(GPIOD, LED_ERROR, GPIO_PIN_RESET);//LED OFF
						memcpy(buff, AtRxBuf, len);
						if (strstr(AtRxBuf, "+CGNSINF:")) {
							sta = putQ(buff, &q_gps);
						} else {
							sta = putQ(buff, &q_at);
						}
					} else {
						sta = -1;
						HAL_GPIO_WritePin(GPIOD, LED_ERROR, GPIO_PIN_SET);//LED ON
					}
					if (sta < 0) free(buff);
				}
			}//------------------------------------------------------------------

			at_rx_uk = 0;
			adone = 0;
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
		char *uk = strstr(RxBuf, _extDate);//const char *_extDate = ":DATE=";
		if (uk) {
			uk += strlen(_extDate);
			if (*uk != '?') {
				if (strlen(uk) < 10) setDate = false;
				else {
#ifdef SET_RTC_TMR
					set_Date((uint32_t)atoi(uk));
#else
					set_extDate((uint32_t)atoi(uk));
					epochSet = true;
					setDate = true;
#endif
				}
			} else setDate = true;
			evt_clear = true;
		} else {
			if (strstr(RxBuf, ":INF")) {
				flags.inf = 1; priz = 1;
			} else if (strstr(RxBuf, ":GET")) {
				flags.msg_begin = 1;
				priz = true;
			} else if ((uk = strstr(RxBuf, ":SRV=")) != NULL) {
				if (strlen(RxBuf) > 7) {
					getAdrPort(uk + 5);
					*(uk + 5) = '\0';
				}
				flags.srv = 1; priz = true;
			} else if (strstr(RxBuf, ":CON")) {
				flags.connect = 1;
				priz = true;
				con_dis = true;
			} else if (strstr(RxBuf, ":DIS")) {
				flags.disconnect = 1;
				priz = true;
				con_dis = false;
			} else if (strstr(RxBuf, ":VIO")) {
				flags.vio = 1;
				priz = true;
			} else if (strstr(RxBuf, ":ON")) {
				evt_gsm = 1;
				priz = true;
			} else if (strstr(RxBuf, ":OFF")) {
				evt_gsm = 2;
				priz = true;
			} else if ((uk = strstr(RxBuf, ":LOG")) != NULL) {//if (strstr(RxBuf, ":LOG?")) {
				priz = true;
				flags.log_flag = 1;
				if (*(uk + 4) != '?') flags.log_show = ~flags.log_show;
			} else if (strstr(RxBuf, ":RESTART")) {
				if (LoopAll) {
					flags.restart = 1;
				} else {
#ifdef SET_OLED_I2C
					i2c_ssd1306_clear();
#endif
#ifdef SET_OLED_SPI
					spi_ssd1306_clear();
#endif
					NVIC_SystemReset();
				}
			} else if (strstr(RxBuf, ":STOP")) {
				flags.stop = 1;
			} else if (strstr(RxBuf, ":CSQ")) {
				flags.csq = 1;
				priz = true;
		    } else {
				if (strstr(RxBuf, "AT")) {
					strcat(RxBuf, "\n"); priz = true;
				} else {
					if ((uk = strchr(RxBuf, '\r')) != NULL) *uk = '\0';
				}
				if (LoopAll && (RxBuf[0] != ':')) {
					int len = strlen(RxBuf);
					char *buff = (char *)calloc(1, len + 1);
					if (buff) {
						memcpy(buff, RxBuf, len);
						if (putQ(buff, &q_cmd) < 0) free(buff);
					}
				}
			}

			if (priz) {

#if defined(SET_OLED_I2C) || defined(SET_OLED_SPI)
	#ifdef SET_OLED_I2C
				i2c_ssd1306_clear_line(4);
	#endif
	#ifdef SET_OLED_SPI
				spi_ssd1306_clear_line(4);
	#endif
				char *uke = strchr(RxBuf, '\r'); if (uke) *uke = '\0';
				if (strlen(RxBuf) > 16) RxBuf[16] = 0;
				int l = strlen(RxBuf);
	#ifdef SET_OLED_I2C
				i2c_ssd1306_text_xy(RxBuf, ssd1306_calcx(l), 4);
	#endif
	#ifdef SET_OLED_SPI
				spi_ssd1306_text_xy(RxBuf, ssd1306_calcx(l), 4);
	#endif
#endif

			}//if (priz)
		}
		rx_uk = 0;
		memset(RxBuf, 0, MAX_UART_BUF);
	}
}
//------------------------------------------------------------------------------------------
//  callback function when recv. data from all uart's
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	adrByte = NULL;

	if (huart->Instance == UART4) {// portAT - at_commands
		getAT();
		adrByte = &aRxByte;
	} else if (huart->Instance == USART3 ) {// portLOG - log
		LogData();
		adrByte = &lRxByte;
	}

	if (adrByte) HAL_UART_Receive_IT(huart, adrByte, 1);
}

//-----------------------------------------------------------------------------------------


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

	osDelay(800);//1200

	initQ(&q_gps);

	s_inf_t inf;
	result_t levt;
	s_data_t iData;

	char toScreen[SCREEN_SIZE];
	uint8_t row;
	uint8_t show;
	uint8_t new = 0;
	bool stp = true;

#ifdef SET_W25FLASH
	W25qxx_Init();
#endif

	/* Infinite loop */

  	while (LoopAll)   {
  		//--------------------------------------------------------------------------
  		if (flags.restart) {
  			flags.restart = 0;
  			if (gprs_stat.connect) flags.disconnect = 1;
  			Report(true, "Restart ARM !\r\n");
  			osDelay(1200);
  			NVIC_SystemReset();
  			break;
  		}
  		//
  		if (flags.stop) {
  			if (gprs_stat.connect) flags.disconnect = 1;
  			sprintf(toScreen, "Stop All");
  			Report(true, "%s!\r\n", toScreen);
  			toDisplay((const char *)toScreen, 0, 5, false);
  			osDelay(1200);//1500
  			if (getVIO()) {
  				gsmONOFF(ModuleOFF);
  				uint8_t ct = 8;
  				while (getVIO() && ct--) {
  					gsmONOFF(ModuleOFF);
  					HAL_Delay(4000);
  				}
  			}
  			LoopAll = false;
  			flags.stop = 0;
  			break;
  		}
  		//
  		//--------------------------------------------------------------------------
  		//						get data from sensors_queue
  		//
  		if (osMessageQueueGet(mailQueueHandle, (void *)&levt, NULL, 50) == osOK) {
  			memcpy((uint8_t *)&iData.sens, (uint8_t *)&levt, sizeof(result_t));
  			new |= 1;
  			row = 6;
  			show = flags.log_show;
  			sprintf(toScreen, "mmHg : %.2f\nDegC : %.2f", levt.pres, levt.temp);
  			switch (levt.chip) {
  				case BMP280_SENSOR :
  					sprintf(toScreen, "mmHg : %.2f\nDegC : %.2f", levt.pres, levt.temp);
  					if (show) sprintf(DefBuf, "BMP280: mmHg=%.2f, DegC=%.2f; BH1750: Lx=%.2f\r\n",
  										levt.pres, levt.temp, levt.lux);
  				break;
  				case BME280_SENSOR :
  					row = 5;
  					sprintf(toScreen, "mmHg : %.2f\nDegC : %.2f\nHumi: %.2f %%rH", levt.pres, levt.temp, levt.humi);
  					if (show) sprintf(DefBuf, "BME280: mmHg=%.2f, DegC=%.2f %%rH=%.2f; BH1750: Lx=%.2f\r\n",
  				  						levt.pres, levt.temp, levt.humi, levt.lux);
  				break;
  				default : {
  						sprintf(toScreen, "\n\n");
  						if (show) sprintf(DefBuf, "Unknown chip; BH1750: Lx=%.2f\r\n", levt.lux);
  				}
  			}
  			//
#if defined(SET_OLED_I2C) || defined(SET_OLED_SPI)
  			sprintf(toScreen+strlen(toScreen), "\nLux  : %.2f", levt.lux);
	#ifdef SET_OLED_I2C
  			if (!i2cError) i2c_ssd1306_text_xy(toScreen, 1, row);
	#endif
	#ifdef SET_OLED_SPI
  			spi_ssd1306_text_xy(toScreen, 1, row);
	#endif
#endif
  			if (show) Report(true, DefBuf);
  		}
  		//--------------------------------------------------------------------------
  		//
  		osDelay(100);
  		//
  		//-------------------------------------------------------------------------
  		//						get data from gps_queue
  		//
  		if (!flags.auto_cmd) {
  			if (getQ(DefBuf, &q_gps) >= 0) {
  				if (flags.log_show) stp = true; else stp = false;
  				Report(stp, DefBuf);
  				if (!parse_inf(DefBuf, &inf)) {
  					flags.msg_end = 1;
  					memcpy((uint8_t *)&iData.inf, (uint8_t *)&inf, sizeof(s_inf_t));
  					new |= 2;
  				}
  			}
  		}
  		//--------------------------------------------------------------------------
  		if (new == 3) {
  			new = 0;
  			if (!makeInfString(&iData, DefBuf, sizeof(DefBuf) - 3)) addRECQ(DefBuf, &recq);
  		}
  		//--------------------------------------------------------------------------

  		osDelay(100);

  	}

  	osDelay(100);

  	exit(0);

  	//LOOP_FOREVER();

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

	size_t d_size = 1;
	int32_t temp, pres, humi = 0;
	uint8_t data_rdx[DATA_LENGTH] = {0};
	uint8_t reg_id = 0, reg_stat = 0, reg_mode = 0, reg_conf = 0;

	result_t sens;

	uint16_t lux = 0;
	float lx = -1.0;

  /* Infinite loop */

	flags.msg_end = 0;

	uint32_t wait_sensor = get_tmr(wait_sensor_def);

	bh1750_on_mode();

	while (LoopAll) {

		if (flags.msg_end) {
			flags.msg_end = 0;
			wait_sensor = 0;
		}

		if (check_tmr(wait_sensor) && !flags.auto_cmd) {
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
					osMessageQueuePut(mailQueueHandle, (void *)&sens, 0, 10);
					//
				}
			}
		}
		//
		osDelay(100);
	}

	osDelay(100);

	exit(0);
	//LOOP_FOREVER();

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

	AtParamInit(true);

	HAL_Delay(1200);

	char *uki = msgCMD;
	const char *buff = NULL;
	uint8_t counter = 0, cnt = 0, max_repeat = 8;
	uint32_t wait_ack = 0, wait_ack_cli = 0;
	uint64_t new_cmds = 0, min_ms = 1, max_ms = 6, tms;
	char *uk = NULL, *uks = NULL, *uke = NULL;
	bool repeat = false;

	flags.imei_flag = flags.inf       = 0;
	flags.sms       = flags.auto_cmd  = 0;
	flags.log_flag  = 0;

	memset((uint8_t *)&gprs_stat, 0, sizeof(s_gprs_stat));
	gprs_stat.send_ok = gprs_stat.next_send = 1;

	bool yes = false;
	msgGPRS[0] = 0;
	char cmd[64];
	int dl, i, j, k;

#ifdef SET_SMS
	char fromNum[lenFrom] = {0};
	uint8_t abcd[5] = {0};
	smsTMP[0] = 0;
	uint16_t sms_num;
	uint16_t sms_len;
	uint8_t sms_total;
	int8_t nrec = -1;
	s_udhi_t reco;
	uint32_t wait_sms = 0;
	InitSMSList();
#endif

	char toScr[SCREEN_SIZE];

	tms = min_ms;

#if defined(SET_OLED_I2C) || defined(SET_OLED_SPI)
	sprintf(toScr, "%s", srv_adr);
	toDisplay((const char *)toScr, 0, 2, false);
#endif

	uint8_t ctn = 8, faza = 0, rx_faza = 0;
	uint32_t tmps = ModuleON;

	bool vios = getVIO();
	if (!vios) {
		while (!getVIO()) { gsmONOFF(tmps); HAL_Delay(2000); }
	} else {
		flags.auto_cmd = 1;
		faza = 4;
		counter = 0;
		cmdsInd = 0;
		cmdsDone = true;
		new_cmds = 0;
	}
	bool prf = true;
	uint32_t wait_csq = 0;
	uint32_t cmdDelay = 0;
	uint16_t snd_len = 0;
	const uint32_t waitOff = 4000;
	const uint32_t waitOn = 2000;
	uint32_t wait_OnOff;

	wait_ack = get_tmr(2);


	/* Infinite loop */

	while (LoopAll) {

		//-----------------------------------------------------------
		if (evt_gsm) {
			tmps = 0;
			if (evt_gsm == 2) {
				tmps = ModuleOFF;//OFF sim868
				wait_OnOff = waitOff;
			} else {
				tmps = ModuleON;//ON sim868
				wait_OnOff = waitOn;
			}
			while (ctn--) {
				vios = getVIO();
				if (evt_gsm == 2) {//OFF
					if (!vios) break;
				} else {//ON
					if (vios) break;
				}
				gsmONOFF(tmps);//   ```|_____|``` - make pulse duration tmps
				HAL_Delay(wait_OnOff);
			};

			AtParamInit(false);
			gprs_stat.init    = 0;
			gprs_stat.connect = 0;
			flags.imei_flag   = 0;
			flags.auto_cmd    = flags.inf = 0;
			ctn = 8;
			evt_gsm = 0;
			faza = 0;
			prf = true;
		}
		//-----------------------------------------------------------

		switch (faza) {
			case 0:
				if (check_tmr(wait_ack)) {
					wait_ack = 0;
					if (flags.auto_cmd) {
						if (cmdsInd >= 0) {
							if (cmdsInd >= cmdsMax) {
								cmdsInd = -1;
								flags.auto_cmd = 0;
								flags.log_show = 1;
							} else {
								if (cmdsDone) {
									if (check_hstmr(new_cmds)) {
										buff = &cmds[cmdsInd][0];
										Report(false, "%.*s", strlen(cmds[cmdsInd]), cmds[cmdsInd]);
										faza = 1;
									}
								}
							}
						}
					} else {
						if (cmdsDone && !flags.auto_cmd) {
							if (getQ(msgCMD, &q_cmd) >= 0) {//get at_command from queue for sending to sim868
								buff = &msgCMD[0];
								faza = 1;
							} else faza = 3;
						}
					}
				}
			break;
			case 1://send at_command to sim868
				HAL_UART_Transmit_DMA(portAT, (uint8_t *)buff, strlen(buff));
				while (HAL_UART_GetState(portAT) != HAL_UART_STATE_READY) {
					if (HAL_UART_GetState(portAT) == HAL_UART_STATE_BUSY_RX) break;
					osDelay(1);
				}
				cmdsDone = false;
				wait_ack = get_tmr(15);
				if ( (strstr(buff, "AT+BTSPPSEND=")) ||
						(strstr(buff, "AT+CIICR")) ||
							(strstr(buff, "AT+CGACT=")) ||
								(strstr(buff, "AT+COPS")) ) {
					wait_ack = get_tmr(45);//WAIT ACK 45 SEC
				} else {
					if (!cmdsInd) wait_ack = get_tmr(5);
				}
				if (strstr(buff, "AT+GSN")) flags.imei_flag = 1;
				else
				if (strstr(buff, "AT+CIFSR")) flags.local_ip_flag = 1;
				else
				if (strstr(buff, "AT+CIPSEND=")) gprs_stat.send_ok = 0;

				if (cmdsInd == -1) wait_ack = 0;

				faza = 2;
			break;
			case 2:
				if (wait_ack) {
					if (check_tmr(wait_ack)) {
						wait_ack = 0;
						cmdsDone = true;
						if (getVIO()) {
							if (cmdsInd >= 0) {
								cmdsInd++;
								if (cmdsInd >= cmdsMax) {
									cmdsInd = -1;
								} else {
									new_cmds = get_hstmr(min_ms);//250 ms
								}
							} else evt_gsm = 2;//OFF
						} else evt_gsm = 1;//ON
					}
				}
				if (evt_gsm || flags.auto_cmd) faza = 0;
										  else faza = 3;
			break;
			case 3:
				yes = false;
				//
				if (flags.connect) {
					if (!gprs_stat.connect) {
						if (cmdsDone) {
							flags.connect = 0;
							yes = true;
							sprintf(cmd, "AT+CIPSTART=\"TCP\",\"%s\",%u\r\n", srv_adr, srv_port);
							buff = cmd;
							gprs_stat.try_connect = 1;
						}
					} else flags.connect = 0;
				} else if (flags.disconnect) {
					if (gprs_stat.connect) {
						if (cmdsDone) {
							flags.disconnect = 0;
							yes = true;
							buff = gprsDISCONNECT;//"AT+CIPCLOSE\r\n"
							gprs_stat.try_disconnect = 1;
						}
					} else flags.disconnect = 0;
				}
				//
				if (!yes) {
					if ((nrec = getRECQ(msgGPRS, &recq)) >= 0) {
						snd_len = strlen(msgGPRS);
						Report(true, "[getRECQ] : get record from queue OK (id=%d len=%u)\r\n", nrec , snd_len);
						if (gprs_stat.connect && gprs_stat.send_ok && cmdsDone) {
							sprintf(cmd, "AT+CIPSEND=%d\r\n", snd_len);
							yes = true;
							buff = cmd;
							gprs_stat.next_send = 0;
						} else if (flags.log_show) Report(true, msgGPRS);
					}
				}
				//
				if (gprs_stat.prompt && gprs_stat.connect) {
					yes = true;
					buff = msgGPRS;
					gprs_stat.prompt = 0;
				}
				if (!yes) {
					if (flags.inf) {
						if (cmdsDone && check_tmr(cmdDelay)) {
							flags.inf = 0;
							if (getVIO() && (onGNS)) {
								yes = true;
								buff = gpsINF;//AT+CGNSINF
								rmc5 = 0;
								wait_csq = get_tmr(wait_csq_def);
							}
						}
					} else if (flags.csq) {
						if (cmdsDone && getVIO()) {
							flags.csq = 0;
							yes = true;
							buff = atCSQ;//AT+CSQ
						}
					}
				}
				if (yes) {
					Report(false, "%s", buff);
					faza = 1;
					break;
				}
				faza = 0;
			break;
			case 4:
				if (check_tmr(wait_ack)) {
					wait_ack = 0;
					faza = 0;
					if (!getVIO()) evt_gsm = 1;//ON
				}
			break;

		}//switch (faza)

		//-----------------------------------------------------------------------------------
		//							rx_data_from_module
		//
		switch (rx_faza) {
			case 0:
				if (getQ(msgAT, &q_at) < 0) break;

				if (strstr(msgAT, "NORMAL POWER DOWN")) {
					if (!flags.stop) {
						HAL_GPIO_WritePin(GPIO_PortD, LED_BLUE_Pin, GPIO_PIN_RESET);
						gprs_stat.init = gprs_stat.connect = 0;
						wait_ack = get_tmr(4);
						faza = 4;
						prf = true;
					}
				} else if (strlen(msgAT)) {
					//
					if (wait_ack_cli) wait_ack_cli = 0;
					//
					ackYes = 1;
					if (flags.local_ip_flag) {
						flags.local_ip_flag = 0;
						gprs_stat.init = 1;
						wait_ack = 0;
						cmdsDone = true;
						if (cmdsInd >= 0) {
							cmdsInd++;
							new_cmds = get_hstmr(min_ms);
						}
					} else if (flags.imei_flag) {
						flags.imei_flag = 0;
						if (strlen(msgAT) >= size_imei) {
							strncpy(devID, msgAT, size_imei);
							devID[size_imei] = '\0';
						}
					} else if (strstr(msgAT, "+CREG:")) {//+CREG: 0,2
						cnt++;
						if (cnt < max_repeat) {
							if ((uk = strchr(msgAT, ',')) != NULL) {
								if (*(uk + 1) != '1') repeat = true;
												 else repeat = false;
							}
						} else {
							cnt = 0;
							repeat = false;
						}
						if (repeat) tms = max_ms;
					} else if (strstr(msgAT, "SMS Ready")) {
						counter = 1;
					} else if (strstr(msgAT, "+CPIN: NOT INSERTED")) {
						counter = 1;
					} else if ((uki = strstr(msgAT, "+CSQ: ")) != NULL) {//+CSQ: 15,0
						uki += 6;
						if ((uk = strchr(uki, ',')) != NULL) {
							dl = uk - uki;
							if (dl > 2) dl = 2;
							memcpy(cmd, uki, dl);
							cmd[dl] = 0;
							gsm_stat.rssi = atoi(cmd);
#if defined(SET_OLED_I2C) || defined(SET_OLED_SPI)
							sprintf(toScr, "RSSI : %d dBm  ", dBmRSSI[gsm_stat.rssi&0x1f]);//gsm_stat.rssi);
							toDisplay((const char *)toScr, 1, 5, true);
#endif
							if (prf) {
								prf = false;
								flags.inf = 1;
								cmdDelay = get_tmr(5);
							} else {
								wait_csq = get_tmr(wait_csq_def);
							}
						}
					} else if ((uki = strstr(msgAT, "+CGNSPWR: ")) != NULL) {
						if (*(uki + 10) == '1') onGNS = true;
										   else onGNS = false;
					} else if ((uki = strstr(msgAT, "+CGATT: ")) != NULL) {
						if (*(uki + 8) == '1') gprs_stat.cgatt_on = 1;
						else gprs_stat.cgatt_on = 0;
					} else if (strstr(msgAT, "ERROR") ||
								strstr(msgAT, "OK") ||
									strstr(msgAT, "CLOSE") ||
										strchr(msgAT, '>') ||
											strstr(msgAT, "+BTSCAN: 1")) {
						if (strstr(msgAT, "SEND OK")) {
							gprs_stat.send_ok = 1;
							gprs_stat.next_send = 1;
							wait_ack_cli = get_tmr(wait_ack_cli_sec);
						} else if (gprs_stat.connect) {
							if (strstr(msgAT, "CLOSE")) {
								gprs_stat.connect = 0;
								con_dis = false;
								gprs_stat.send_ok = gprs_stat.next_send = 1;
								Report(true, "--- DISCONNECTED ---\r\n");
								if (gprs_stat.try_disconnect) gprs_stat.try_disconnect = 0;
								HAL_GPIO_WritePin(GPIO_PortD, LED_BLUE_Pin, GPIO_PIN_RESET);
								wait_ack_cli = 0;
							} else if (strstr(msgAT, "ERROR")) {
								gprs_stat.try_disconnect = 1;
								wait_ack_cli = 0;
							}
						}
						if (strchr(msgAT, '>')) gprs_stat.prompt = 1;
										   else gprs_stat.prompt = 0;
						if (gprs_stat.try_connect) {
							if (strstr(msgAT, "CONNECT")) {
								gprs_stat.connect = 1;
								gprs_stat.send_ok = 1;
								Report(true, "+++ CONNECTED +++\r\n");
								gprs_stat.try_connect = 0;
								flags.msg_begin = 1;
								con_dis = true;
								HAL_GPIO_WritePin(GPIO_PortD, LED_BLUE_Pin, GPIO_PIN_SET);
							}
						}

						wait_ack = 0;
						cmdsDone = true;
						if (cmdsInd >= 0) {
							if (!repeat) {
								cmdsInd++;
								if (gprs_stat.cgatt_on) {
									cmdsInd++;
									gprs_stat.cgatt_on = 0;
								}
								tms = min_ms;
							}
							new_cmds = get_hstmr(tms);
						}
					}//"ERROR" || "OK" || "CLOSE" || '>' || "+BTSCAN: 1" -> next cmd enable (cmdsDone = true; wait_ack = 0;)

					//
					if (strstr(msgAT, "ALREADY CONNECT")) {
						gprs_stat.connect = 1;
						gprs_stat.send_ok = 1;
						gprs_stat.try_connect = 0;
						flags.msg_begin = 1;
						con_dis = true;
						HAL_GPIO_WritePin(GPIO_PortD, LED_BLUE_Pin, GPIO_PIN_SET);
					}
					//
#ifdef SET_SMS
					if ( strstr(msgAT, "+CMT: ") || strstr(msgAT, "+SCLASS0: ") ) {
						flags.sms = 1;
						memset(SMS_text, 0, SMS_BUF_LEN);
						k = strlen(msgAT);
						if (k > SMS_BUF_LEN - 1) k = SMS_BUF_LEN - 1;
						strncpy(SMS_text, msgAT, k);
					} else {
						if (flags.sms) {
							flags.sms = 0;
							j = strlen(SMS_text);
							i = strlen(msgAT);
							if ((j + i) < SMS_BUF_LEN) {
								strcat(&SMS_text[j], msgAT);
								if (i) Report(false, msgAT);
								memset(abcd, 0, sizeof(abcd));
								memset(fromNum, 0, sizeof(fromNum));
								sms_num = 0;
								sms_len = conv_ucs2_text((uint8_t *)SMS_text, fromNum, abcd, 0);
								if (sms_len > 0) {
									Report(true, "[SMS] len=%u udhi=[%02X%02X%02X%02X%02X] from='%s' body:\r\n%.*s\r\n",
											sms_len, abcd[0], abcd[1], abcd[2], abcd[3], abcd[4], fromNum, sms_len, SMS_text);
									//
									if ((abcd[0] == 1) && abcd[3]) {//with_UDHI and total > 0
										memset((uint8_t *)&reco, 0, sizeof(s_udhi_t));
										memcpy((uint8_t *)&reco, abcd, sizeof(abcd));
										if (reco.total <= maxSMSPart) {
											if (sms_len >= MaxBodyLen) sms_len = MaxBodyLen - 1;
											memcpy(reco.txt, SMS_text, sms_len);
											reco.len = sms_len;
											if (PutSMSList(&reco) != 255) {
												if (!wait_sms) wait_sms = get_tmr(wait_sms_time);//set timer for wait all patrs recv.
												if (LookAllPart(reco.total) == reco.total) {//all parts are present -> concat begin
													*SMS_text = '\0';
													if (ConcatSMS(SMS_text, reco.total, &sms_num, &sms_len) == reco.total) {
														Report(true, "[SMS] Concat message #%u (len=%u parts=%u) done:\r\n%.*s\r\n",
																     sms_num, sms_len, reco.total, sms_len, SMS_text);
														//----------------  check : command present in sms ?   ------------------------
														checkSMS(SMS_text, fromNum);
														//-----------------------------------------------------------------------------
														*smsTMP = '\0';
														if (!makeSMSString(SMS_text, &sms_len, fromNum, sms_num, smsTMP, sizeof(smsTMP) - 1)) {
															//
															if (addRECQ(smsTMP, &recq) < 0) {
																if (flags.log_show) Report(true, smsTMP);
															}
															//
														}
													}
													InitSMSList();
													wait_sms = 0;
												}
											}
										}
									} else {//without_UDHI
										//----------------  check : command present in sms ?   ------------------------
										checkSMS(SMS_text, fromNum);
										//-----------------------------------------------------------------------------
										*smsTMP = '\0';
										if (!makeSMSString(SMS_text, &sms_len, fromNum, sms_num, smsTMP, sizeof(smsTMP) - 1)) {
											if (addRECQ(smsTMP, &recq) < 0) {
												if (flags.log_show) Report(true, smsTMP);
											}
										}
									}
									//
								}
								*msgAT = '\0';
							}
						}
					}
					//
					if ((uks = strstr(msgAT, "+CUSD: 0, \"")) != NULL) {
						Report(false, msgAT);
						uks += 11;//uk to begin ucs2 string
						uke = strstr(uks, "\", 72");
						if (uke) {
							*uke = '\0';
							if (ucs2_to_utf8(uks, NULL, (uint8_t *)SMS_text)) Report(false, "%s\r\n", SMS_text);
						}
						*msgAT = '\0';
					}
					//
#endif
					//
				}

				//---------------------------------------------------------------------
				if (strlen(msgAT)) Report(false, msgAT);
				//---------------------------------------------------------------------

				if (counter) {
					counter = 0;
					cmdsInd = 0;
					cmdsDone = true;
					new_cmds = get_hstmr(min_ms);
					flags.auto_cmd = 1;
					prf = true;
				}
			break;
		}//switch (rx_faza)

		if (wait_ack_cli) {//check timeout for wait ack from tcpServer
			if (check_tmr(wait_ack_cli)) {
				wait_ack_cli = 0;
				flags.disconnect = 1;
				con_dis = false;
			}
		}

		if (wait_csq && !flags.auto_cmd) {
			if (check_tmr(wait_csq)) {
				flags.csq = 1;
				wait_csq = 0;
			}
		}

#ifdef SET_SMS
		//-------------------------   concat sms parts by timer   ------------------------
		if (wait_sms) {
			if (check_tmr(wait_sms)) {
				wait_sms = 0;
				sms_total = getSMSTotalCounter();
				if (sms_total) {
					*SMS_text = '\0';   sms_num = 0;
					if (ConcatSMS(SMS_text, sms_total, &sms_num, &sms_len) == sms_total) {
						Report(true, "[SMS] Concat message #%u with len %u by timeout:\r\n%.*s\r\n",
								     sms_num, sms_len, sms_len, SMS_text);
						//----------------  check : command present in sms ?   ------------------------
						checkSMS(SMS_text, fromNum);
						//-----------------------------------------------------------------------------
						*smsTMP = '\0';
						if (!makeSMSString(SMS_text, &sms_len, fromNum, sms_num, smsTMP, sizeof(smsTMP) - 1)) {
							if (addRECQ(smsTMP, &recq) < 0) {
								if (flags.log_show) Report(true, smsTMP);
							}
						}
					}
				}
				InitSMSList();
			}
		}
		//--------------------------------------------------------------------------------
#endif

		if (flags.srv) {
			flags.srv = 0;
			Report(true, "CURRENT SERVER : %s:%u\r\n", srv_adr, srv_port);
#if defined(SET_OLED_I2C) || defined(SET_OLED_SPI)
			sprintf(toScr, "%s", srv_adr);
			toDisplay((const char *)toScr, 0, 2, true);
#endif
		}

		if (flags.vio) {
			flags.vio = 0;
			Report(true, "GSM VIO is %u\r\n", getVIO());
		}

		if (flags.log_flag) {
			flags.log_flag = 0;
			Report(true, "LOG now is %s\r\n", strOnOff[flags.log_show]);
		}

		osDelay(1);

	}

	osDelay(50);
	exit(0);
	//LOOP_FOREVER();

  /* USER CODE END StartAtTask */
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
		  //------------------------------------------------------------------------------------------
		  inc_secCounter();
		  HAL_GPIO_TogglePin(GPIOD, LED_GREEN_Pin);

		  if (evt_clear) {
			  evt_clear = false;
#ifdef SET_OLED_I2C
			  if (!i2cError) i2c_ssd1306_clear_line(1);
#endif
#ifdef SET_OLED_SPI
			  spi_ssd1306_clear_line(1);
#endif
		  }

		  char scrBuf[32];
#ifndef SET_RTC_TMR
		  if (epochSet) inc_extDate();
		  uint32_t ep;
		  if (setDate) ep = get_extDate();
		   	  	  else ep = get_secCounter();
	#if defined(SET_OLED_I2C) || defined(SET_OLED_SPI)
		  sec_to_string(ep, scrBuf, false);
		  toDisplay((const char *)scrBuf, 0, 1, false);
	#endif
#else
	#if defined(SET_OLED_I2C) || defined(SET_OLED_SPI)
		  sec_to_string(get_secCounter(), scrBuf, false);
		  toDisplay((const char *)scrBuf, 0, 1, false);

	#endif
#endif
		  //------------------------------------------------------------------------------------------
		  if (HAL_GPIO_ReadPin(USER_IN_GPIO_Port, USER_IN_Pin) == GPIO_PIN_SET) {//user key is pressed
			  if (!con_dis) {
				  if (!flags.connect) flags.connect = 1;
				  con_dis = true;
			  } else {
				  if (!flags.disconnect) flags.disconnect = 1;
				  con_dis = false;
			  }
		  }
		  //------------------------------------------------------------------------------------------
		  if (getVIO()) HAL_GPIO_WritePin(GPIO_PortD, LED_ORANGE_Pin, GPIO_PIN_SET);//gsm is on
		  		   else HAL_GPIO_WritePin(GPIO_PortD, LED_ORANGE_Pin, GPIO_PIN_RESET);//gsm is off
		  //------------------------------------------------------------------------------------------
#if defined(SET_OLED_I2C) || defined(SET_OLED_SPI)
		  char pic = 0x14;
		  if (gprs_stat.connect) pic = 0x15;
		  sprintf(scrBuf, "%c %s", pic, conStatus[gprs_stat.connect]);
		  toDisplay((const char *)scrBuf, 0, 3, true);
#endif
		  //------------------------------------------------------------------------------------------
		  rmc5++;
		  if (flags.msg_begin) {
			  flags.msg_begin = 0;
			  rmc5 = wait_gps_def;
		  }
		  if (rmc5 >= wait_gps_def) {
			  if (gprs_stat.next_send && !flags.auto_cmd) {
				  rmc5 = 0;
				  flags.inf = 1;
			  }
		  }
		  //------------------------------------------------------------------------------------------
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
