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
#include "bmp280.h"
#include "bh1750.h"

#if defined(SET_OLED_I2C) || defined(SET_OLED_SPI)
	#include "ssd1306.h"
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
const char *ver = "ver 3.3rc1";//03.08.2019 - minor changes : set HEAP_SIZE up to 32K, use JFES library, fixed memory leak bug


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

const uint32_t ModuleOFF = 1750;
const uint32_t ModuleON  = 1150;

const char *dev_name = "STM32_SIM868";
char devID[size_imei + 1] = {0};//imei of gsm module

volatile static uint32_t secCounter = 0;//period 1s
volatile static uint64_t HalfSecCounter = 0;//period 250ms

const uint32_t min_wait_ms = 150;
const uint32_t max_wait_ms = 1000;
HAL_StatusTypeDef i2cError = HAL_OK;
I2C_HandleTypeDef *portBMP;
#ifdef SET_OLED_I2C
	I2C_HandleTypeDef *portSSD;
#endif
I2C_HandleTypeDef *portBH;
UART_HandleTypeDef *portAT;//huart4;
UART_HandleTypeDef *portLOG;//huart3;
#ifdef SET_OLED_SPI
	SPI_HandleTypeDef *portOLED;//hspi3;
#endif

static const char *_extDate = ":DATE=";
bool evt_clear = false;
static bool setDate = false;
#ifndef SET_RTC_TMR
	static bool epochSet = false;
	volatile static uint32_t extDate = 0;
#endif

static char DefBuf[MAX_UART_BUF];

static char RxBuf[MAX_UART_BUF];
volatile uint16_t rx_uk;
uint8_t lRxByte;

volatile uint8_t rmc5 = 12;//8;//6;
char msgNMEA[MAX_UART_BUF] = {0};
static s_msg_t q_gps;
const char *nameValid[] = {"Invaid", "Valid"};
const char *nameNS[] = {"North" , "South"};
const char *nameEW[] = {"East" , "West"};
static bool onGNS = false;

static char AtRxBuf[MAX_UART_BUF];
volatile uint16_t at_rx_uk;
uint8_t aRxByte;
uint8_t adone = 0;
char msgAT[MAX_UART_BUF] = {0};
static s_msg_t q_at;
static uint8_t evt_gsm = 0;
static bool ackYes = 0;

static int8_t cmdsInd = -1;
volatile bool cmdsDone = true;
char msgCMD[MAX_UART_BUF] = {0};
static s_msg_t q_cmd;
const uint8_t cmdsMax = 11;//19;//21;
const char *cmds[] = {
	"AT\r\n",
	"AT+CMEE=0\r\n",
	"AT+GMR\r\n",//get version of FW
	"AT+GSN\r\n",//get IMEI
	"AT+CNMI=1,2,0,1,0\r\n",
	"AT+SCLASS0=0\r\n",
//	"AT+CPMS=\"SM\",\"SM\",\"SM\"\r\n",
	"AT+CMGF=0\r\n",//;+CLIP=1\r\n",
//	"AT+CIMI\r\n",//get IMCI
//	"AT+CMGF=1\r\n",//text mode
//	"AT+CSCS=\"IRA\"\r\n",
	"AT+CGNSPWR=1\r\n",// power for GPS/GLONASS ON
	"AT+CGNSPWR?\r\n",//check power for GPS/GLONASS status
//	"AT+CCLK?\r\n",//get date/time
	"AT+CREG?\r\n",
	"AT+CSQ\r\n"//get RSSI
//	"AT+CENG=1\r\n",//Switch on engineering mode
/*	"AT+CGDCONT=1,\"IP\",\"internet.beeline.ru\"\r\n",
	"AT+CSTT=\"internet.beeline.ru\",\"beeline\",\"beeline\"\r\n",
	"AT+CGACT=1,1\r\n",
	"AT+CIICR\r\n",
	"AT+CGATT?\r\n",
	"AT+CGATT=1\r\n",
	"AT+CIFSR\r\n"*/
//	"AT+CENG?\r\n"
};

const char *sim_num = "+79062100000";
const char *srv_adr_def = "aaa.bbb.ccc.ddd";
const uint16_t srv_port_def = 9192;
static char srv_adr[16] = {0};
static uint16_t srv_port;

const char *gprsDISCONNECT = "AT+CIPCLOSE\r\n";
char msgGPRS[MAX_UART_BUF] = {0};
const char *conStatus[] = {"Discon.", "Connect"};

volatile static bool LoopAll = true;
uint8_t *adrByte = NULL;
volatile s_flags flags = {0};
volatile s_gprs_stat gprs_stat = {0};
osMessageQueueId_t mqData;
uint32_t infCounter = 0;
s_gsm_stat gsm_stat = {0};
const char *gpsINF = "AT+CGNSINF\r\n";
const char *atCSQ = "AT+CSQ\r\n";
static bool con_dis = 0;
const char *logStat[] = {"OFF", "ON"};

const uint8_t maxItems = 27;//30;
const char *Items[] = {
	"SeqNum",
	"MsgType",
	"DevID",
	"DevName",
	"SimNumber",
	"DevTime",
	"FreeMem",
	"EpochTime",
	"UTC",
//	"Run",
	"Status",
	"Latitude",
	"Longitude",
	"Altitude",
	"Speed",
	"Dir",
//	"Mode",
	"HDOP",
	"PDOP",
	"VDOP",
	"SatGPSV",
	"SatGNSSU",
	"SatGLONASSV",
	"dBHz",
//	"HPA",
//	"VPA",
	"Rssi",
	"Press",
	"Temp",
	"Lux",
	"Humi"
};

const int8_t dBmRSSI[max_rssi] = {
	-113,-111,-109,-107,-105,-103,-101,-99,
	-97 ,-95 ,-93 ,-91 ,-89 ,-87 ,-85 ,-83,
	-81 ,-79 ,-77 ,-75 ,-73 ,-71 ,-69 ,-67,
	-65 ,-63 ,-61 ,-59 ,-57 ,-55 ,-53 ,-51
};


#ifdef SET_JFES
	jfes_config_t conf;
	jfes_config_t *jconf = NULL;
#endif


#ifdef SET_SMS

	static s_udhi_t sms_rec[maxSMSPart];
	static s_smsq_t smsq;

	const char *smsType[max_smsType] = {
		"+CMT: ",
		"+SCLASS0: ",
		"+CMGR: "
	};

	const char *tp[4] = {//тип кодирования
		"GSM-7bit",
		"GSM-8bit",
		"UCS2",
		"???"
	};
	const char *type_name_a[9] = {//тип номера
		"unknown",
		"International",
		"National",
		"Network",
		"Subscriber",
		"Alphanumeric",
		"Abbreviated",
		"reserved",
		"???"
	};

	char alphabet[] = "@ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789 !\"#$%&'()*+,-./:;<=>?АБВГДЕЁЖЗИЙКЛМНОПРСТУФХЦЧШЩЪЫЬЭЮЯабвгдеёжзийклмнопрстуфхцчшщъыьэюя\n\r`эijIЭ";
	uint16_t cod_PDU[] = {
		0x0040,0x0041,0x0042,0x0043,0x0044,0x0045,0x0046,0x0047,0x0048,0x0049,0x004A,0x004B,0x004C,0x004D,0x004E,0x004F,
		0x0050,0x0051,0x0052,0x0053,0x0054,0x0055,0x0056,0x0057,0x0058,0x0059,0x005A,0x0061,0x0062,0x0063,0x0064,0x0065,
		0x0066,0x0067,0x0068,0x0069,0x006A,0x006B,0x006C,0x006D,0x006E,0x006F,0x0070,0x0071,0x0072,0x0073,0x0074,0x0075,
		0x0076,0x0077,0x0078,0x0079,0x007A,0x0030,0x0031,0x0032,0x0033,0x0034,0x0035,0x0036,0x0037,0x0038,0x0039,0x0020,
		0x0021,0x0022,0x0023,0x0024,0x0025,0x0026,0x0027,0x0028,0x0029,0x002A,0x002B,0x002C,0x002D,0x002E,0x002F,0x003A,
		0x003B,0x003C,0x003D,0x003E,0x003F,
		0x0410,0x0411,0x0412,0x0413,0x0414,0x0415,0x0401,0x0416,0x0417,0x0418,0x0419,0x041A,0x041B,0x041C,0x041D,0x041E,
		0x041F,0x0420,0x0421,0x0422,0x0423,0x0424,0x0425,0x0426,0x0427,0x0428,0x0429,0x042A,0x042B,0x042C,0x042D,0x042E,
		0x042F,0x0430,0x0431,0x0432,0x0433,0x0434,0x0435,0x00B8,0x0436,0x0437,0x0438,0x0439,0x043A,0x043B,0x043C,0x043D,
		0x043E,0x043F,0x0440,0x0441,0x0442,0x0443,0x0444,0x0445,0x0446,0x0447,0x0448,0x0449,0x044A,0x044B,0x044C,0x044D,
		0x044E,0x044F,0x000A,0x000D,0x0060,0x0454,0x0456,0x0457,0x0406,0x0404
	};

	const char *eolin = "\r\n";
	int TSINPART = 0;//from, date/time are present in part 1 sms only, if sms without udhi -> from, date/time not present
	char SMS_text[SMS_BUF_LEN];

	char smsGPRS[MAX_UART_BUF];

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
void StartDefTask(void *argument); // for v2
void StartSensTask(void *argument); // for v2
void StartAtTask(void *argument); // for v2

/* USER CODE BEGIN PFP */

void Report(bool addTime, const char *fmt, ...);
void errLedOn(const char *from);
void gsmONOFF(const uint32_t tw);
void initQ(s_msg_t *q);
void clearQ(s_msg_t *q);
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
  MX_USART3_UART_Init();
  MX_SPI3_Init();
  MX_RTC_Init();
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

    bh1750_off();

    // start timer2 + interrupt
    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_Base_Start_IT(&htim2);

    strcpy(srv_adr, srv_adr_def);
    srv_port = srv_port_def;

#ifdef SET_JFES
    conf.jfes_malloc = (jfes_malloc_t)malloc;//pvPortMalloc,
    conf.jfes_free = free;//vPortFree;
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

  	  const osMessageQueueAttr_t mqData_attributes = {
  		.name = "mqAllData"
  	  };
  	  mqData = osMessageQueueNew(MAX_QMSG, sizeof(s_data_t), &mqData_attributes);

  	  HAL_Delay(1500);
  	  rx_uk = 0;
  	  memset(RxBuf, 0, MAX_UART_BUF);
  	  HAL_UART_Receive_IT(portLOG, (uint8_t *)&lRxByte, 1);//LOG

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

	secCounter     = 0; // 1 sec counter (uint32_t)
	HalfSecCounter = 0; // 250 ms counter (uint64_t)
#ifndef SET_RTC_TMR
	extDate        = 0; // 1 sec counter (uint32_t)
#endif
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
  HAL_GPIO_WritePin(GSM_KEY_GPIO_Port, GSM_KEY_Pin, GPIO_PIN_RESET);

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

//------------------------------------------------------------------------------------------
bool getVIO()
{
	return ((bool)HAL_GPIO_ReadPin(GSM_STATUS_GPIO_Port, GSM_STATUS_Pin));
}
//-----------------------------------------------------------------------------------------
#if defined(SET_OLED_I2C) || defined(SET_OLED_SPI)
uint8_t ssd1306_calcx(int len)
{
uint8_t ret = 0;

    if ( (len > 0) && (len <= 16) ) ret = ((16 - len) >> 1) + 1;

    return ret;
}
#endif
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
void gsmONOFF(const uint32_t twait)
{
bool vio = getVIO();
uint32_t tik = twait / 10, cnt = 10;

	flags.log_show   = 0;
	onGNS = false;

	Report(true, "GSM_KEY set to 0 (vio=%u)\r\n", vio);
	HAL_GPIO_WritePin(GSM_KEY_GPIO_Port, GSM_KEY_Pin, GPIO_PIN_RESET);//set 0

	while ((vio == getVIO()) && cnt--) HAL_Delay(tik);

	HAL_GPIO_WritePin(GSM_KEY_GPIO_Port, GSM_KEY_Pin, GPIO_PIN_SET);//set 1
	Report(true, "GSM_KEY set to 1 (vio=%u)\r\n", getVIO());

	ackYes = 0;
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

#ifndef SET_RTC_TMR
	//-----------------------------------------------------------------------------
	uint32_t get_extDate()
	{
		return extDate;
	}
	//-----------------------------------------------------------------------------
	void inc_extDate()
	{
		extDate++;
	}
	//-----------------------------------------------------------------------------
	void set_extDate(uint32_t ep)
	{
		extDate = ep;
	}
	//-----------------------------------------------------------------------------
#endif

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
		HAL_GPIO_WritePin(GPIO_PortD, Pin, GPIO_PIN_SET);//LED ON
		HAL_Delay(250);
		HAL_GPIO_WritePin(GPIO_PortD, Pin, GPIO_PIN_RESET);//LED OFF
		HAL_Delay(250);
		HAL_GPIO_WritePin(GPIO_PortD, Pin, GPIO_PIN_SET);//LED ON
	} else {
		HAL_GPIO_WritePin(GPIO_PortD, Pin, GPIO_PIN_RESET);//LED OFF
	}
}
//----------------------------------------------------------------------------------------
int sec_to_string(uint32_t sec, char *stx, bool log)
{
int ret = 0;

	if (!setDate) {//no valid date in RTC
		uint32_t s = sec;
		uint32_t day = s / (60 * 60 * 24);
		s %= (60 * 60 * 24);
		uint32_t hour = s / (60 * 60);
		s %= (60 * 60);
		uint32_t min = s / (60);
		s %= 60;
		ret = sprintf(stx, "%03lu.%02lu:%02lu:%02lu", day, hour, min, s);
	} else {//in RTC valid date (epoch time)
#ifndef SET_RTC_TMR
 		struct tm ts;
		time_t ep = (time_t)sec;
		if (gmtime_r(&ep, &ts) != NULL) {
			if (log)
				ret = sprintf(stx, "%02d.%02d.%04d %02d:%02d:%02d",
							ts.tm_mday, ts.tm_mon + 1, ts.tm_year + 1900, ts.tm_hour, ts.tm_min, ts.tm_sec);
			else
				ret = sprintf(stx, "%02d.%02d %02d:%02d:%02d",
							ts.tm_mday, ts.tm_mon + 1, ts.tm_hour, ts.tm_min, ts.tm_sec);
		}
#else
		RTC_TimeTypeDef sTime;
		RTC_DateTypeDef sDate;
		if (HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN)) errLedOn(NULL);
		else {
			if (HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN)) errLedOn(NULL);
			else {
				if (log) ret = sprintf(stx, "%02u.%02u.%04u %02u:%02u:%02u",
						sDate.Date, sDate.Month, sDate.Year + 1900,
						sTime.Hours, sTime.Minutes, sTime.Seconds);
				else
					ret = sprintf(stx, "%02u.%02u %02u:%02u:%02u",
										sDate.Date, sDate.Month,
										sTime.Hours, sTime.Minutes, sTime.Seconds);
			}
		}
#endif
	}

	if (log) {
		strcat(stx, " | ");
		ret += 3;
	}

    return ret;
}
//-----------------------------------------------------------------------------
#ifdef SET_RTC_TMR
void set_Date(uint32_t epoch)
{
struct tm ts;
time_t ep = epoch;

	if (!gmtime_r(&ep, &ts)) {
		errLedOn(NULL);
		return;
	}

	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;
	sDate.WeekDay = ts.tm_wday;
	sDate.Month   = ts.tm_mon + 1;
	sDate.Date    = ts.tm_mday;
	sDate.Year    = ts.tm_year;
	sTime.Hours   = ts.tm_hour;
	sTime.Minutes = ts.tm_min;
	sTime.Seconds = ts.tm_sec;

	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN)) errLedOn(NULL);
	else {
		if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN)) errLedOn(NULL);
		else setDate = true;
	}
}
//----------------------------------------------------------------------------------------
uint32_t getSecRTC(RTC_HandleTypeDef *hrtc)
{
time_t ep = 0;
RTC_DateTypeDef sDate;

	if (HAL_RTC_GetDate(hrtc, &sDate, RTC_FORMAT_BIN)) errLedOn(NULL);
	else {
		RTC_TimeTypeDef sTime;
		if (HAL_RTC_GetTime(hrtc, &sTime, RTC_FORMAT_BIN)) errLedOn(NULL);
		else {
			struct tm ts;
			ts.tm_sec = sTime.Seconds;
			ts.tm_min = sTime.Minutes;
			ts.tm_hour = sTime.Hours;
			ts.tm_mday = sDate.Date;
			ts.tm_mon = sDate.Month - 1;
			ts.tm_year = sDate.Year;
			ts.tm_wday = sDate.WeekDay;
			ep = mktime(&ts);
		}
	}

	return ep;
}
#endif
//----------------------------------------------------------------------------------------
void Report(bool addTime, const char *fmt, ...)
{
HAL_StatusTypeDef er = HAL_OK;
size_t len = MAX_UART_BUF;

#ifndef SET_STATIC_MEM_LOG
	char *buff = (char *)pvPortMalloc(len);//char *buff = (char *)calloc(1, len);
	if (buff) {
#else
	char buff[MAX_UART_BUF];
#endif
		int dl = 0, sz;
		va_list args;

		if (addTime) {

#ifdef SET_RTC_TMR
			dl = sec_to_string(get_secCounter(), buff, true);
#else
			uint32_t ep;
			if (setDate) ep = get_extDate();
					else ep = get_secCounter();
			dl = sec_to_string(ep, buff, true);
#endif
		}
		sz = dl;
		va_start(args, fmt);
		sz += vsnprintf(buff + dl, len - dl, fmt, args);
		//
		if (osSemaphoreAcquire(binSemHandle, 2000) == osOK) {
			HAL_UART_Transmit_DMA(portLOG, (uint8_t *)buff, sz);
			while (HAL_UART_GetState(portLOG) != HAL_UART_STATE_READY) {
				if (HAL_UART_GetState(portLOG) == HAL_UART_STATE_BUSY_RX) break;
				osDelay(1);
			}
			osSemaphoreRelease(binSemHandle);
		} else er = HAL_ERROR;
		//
		va_end(args);
#ifndef SET_STATIC_MEM_LOG
		vPortFree(buff);//free(buff);
	} else er = HAL_ERROR;
#endif

	if (er != HAL_OK) Leds(true, LED_ERROR);
}
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
void clearQ(s_msg_t *q)
{
	q->put = q->get = 0;
	for (uint8_t i = 0; i < MAX_QMSG; i++) {
		q->msg[i].id = i;
		if (q->msg[i].adr) free(q->msg[i].adr);
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
		int len = strlen(q->msg[q->get].adr);
		memcpy(dat, q->msg[q->get].adr, len);
		*(dat + len) = '\0';
		free(q->msg[q->get].adr);
		q->msg[q->get].adr = NULL;
		q->get++; if (q->get >= MAX_QMSG) q->get = 0;
	}

	return ret;
}
//------------------------------------------------------------------------------
void getAdrPort(char *uk)
{
int i31;

	char *uki = strchr(uk, ':');
	if (uki) {
		i31 = atoi(uki + 1);
		if ((i31 > 0) && (i31 <= 65530)) srv_port = i31;
		i31 = uki - uk;
	} else {
		i31 = strlen(uk);
	}

	if (i31 > 0) {
		if (i31 >= sizeof(srv_adr)) i31 = sizeof(srv_adr) - 1;
		memcpy(srv_adr, uk, i31);
		*(srv_adr + i31) = 0;
	}
}
//------------------------------------------------------------------------------------------
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
				//
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
//------------------------------------------------------------------------------------------
void AtParamInit(bool how)
{
	at_rx_uk = 0;
	memset(AtRxBuf, 0, MAX_UART_BUF);
	cmdsInd = -1;
	if (how) {//init queue
		initQ(&q_at);
		initQ(&q_cmd);
	} else {//clear queue
		clearQ(&q_at);
		clearQ(&q_cmd);
	}

	HAL_UART_Receive_IT(portAT, (uint8_t *)&aRxByte, 1);//AT
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
int8_t makeInfString(const s_data_t *data, char *buf, int max_len_buf)
{
int8_t ret = -1;

	if (!data || !buf || (max_len_buf < 4)) return ret;

#ifdef SET_JFES
    uint8_t i = 0;
    jfes_value_t *obj = jfes_create_object_value(jconf);
    if (obj) {
    	jfes_set_object_property(jconf, obj, jfes_create_integer_value(jconf, ++infCounter), Items[i++], 0);//"SeqNum"
    	jfes_set_object_property(jconf, obj, jfes_create_string_value(jconf, "+CGNSINF", 0), Items[i++], 0);//"MsgType"
    	if (strlen(devID)) jfes_set_object_property(jconf, obj, jfes_create_string_value(jconf, devID, 0), Items[i++], 0);//"DevID"
    	jfes_set_object_property(jconf, obj, jfes_create_string_value(jconf, dev_name, 0), Items[i++], 0);//"DevName"
    	jfes_set_object_property(jconf, obj, jfes_create_string_value(jconf, sim_num, 0), Items[i++], 0);//"SimNumber"
    	jfes_set_object_property(jconf, obj, jfes_create_integer_value(jconf, get_secCounter()), Items[i++], 0);//"DevTime"
    	jfes_set_object_property(jconf, obj, jfes_create_integer_value(jconf, xPortGetFreeHeapSize()), Items[i++], 0);//"FreeMem"
    	if (setDate) {
#ifdef SET_RTC_TMR
            uint32_t ep = getSecRTC(&hrtc);
#else
            uint32_t ep = get_extDate();
#endif
            jfes_set_object_property(jconf, obj, jfes_create_integer_value(jconf, ep), Items[i++], 0);//"EpochTime"
        } else i++;
        char stx[64];
        int dl = sprintf(stx, "%02u.%02u.%04u %02u:%02u:%02u.%03u",
        		data->inf.utc.day, data->inf.utc.mon, data->inf.utc.year,
				data->inf.utc.hour, data->inf.utc.min, data->inf.utc.sec, data->inf.utc.ms);
        jfes_set_object_property(jconf, obj, jfes_create_string_value(jconf, stx, dl), Items[i++], 0);//"UTC"
//        jfes_set_object_property(jconf, obj, jfes_create_integer_value(jconf, data->inf.run), Items[i++], 0);//"Run"
        jfes_set_object_property(jconf, obj, jfes_create_string_value(jconf, nameValid[data->inf.status&1], 0), Items[i++], 0);//"Status"
        jfes_set_object_property(jconf, obj, jfes_create_double_value(jconf, (double)data->inf.latitude), Items[i++], 0);//"Latitude"
        jfes_set_object_property(jconf, obj, jfes_create_double_value(jconf, (double)data->inf.longitude), Items[i++], 0);//"Longitude"
        jfes_set_object_property(jconf, obj, jfes_create_integer_value(jconf, data->inf.altitude), Items[i++], 0);//"Altitude"
        jfes_set_object_property(jconf, obj, jfes_create_double_value(jconf, (double)data->inf.speed), Items[i++], 0);//"Speed"
        jfes_set_object_property(jconf, obj, jfes_create_double_value(jconf, (double)data->inf.dir), Items[i++], 0);//"Dir"
//        jfes_set_object_property(jconf, obj, jfes_create_integer_value(jconf, data->inf.mode), Items[i++], 0);//"Mode"
        i++;//jfes_set_object_property(jconf, obj, jfes_create_double_value(jconf, (double)data->inf.HDOP), Items[i++], 0);//"HDOP"
        i++;//jfes_set_object_property(jconf, obj, jfes_create_double_value(jconf, (double)data->inf.PDOP), Items[i++], 0);//"PDOP"
        i++;//jfes_set_object_property(jconf, obj, jfes_create_double_value(jconf, (double)data->inf.VDOP), Items[i++], 0);//"VDOP"
        jfes_set_object_property(jconf, obj, jfes_create_integer_value(jconf, data->inf.GPSsatV), Items[i++], 0);//"SatGPSV"
        jfes_set_object_property(jconf, obj, jfes_create_integer_value(jconf, data->inf.GNSSsatU), Items[i++], 0);//"SatGNSSU"
        jfes_set_object_property(jconf, obj, jfes_create_integer_value(jconf, data->inf.GLONASSsatV), Items[i++], 0);//"SatGLONASSV"
        i++;//jfes_set_object_property(jconf, obj, jfes_create_integer_value(jconf, data->inf.dBHz), Items[i++], 0);//"dBHz"
//        jfes_set_object_property(jconf, obj, jfes_create_double_value(jconf, (double)data->inf.HPA), Items[i++], 0);//"HPA"
//        jfes_set_object_property(jconf, obj, jfes_create_double_value(jconf, (double)data->inf.VPA), Items[i++], 0);//"VPA"
        jfes_set_object_property(jconf, obj, jfes_create_integer_value(jconf, dBmRSSI[gsm_stat.rssi&0x1f]), Items[i++], 0);//"RSSI"

        jfes_set_object_property(jconf, obj, jfes_create_double_value(jconf, (double)data->sens.pres), Items[i++], 0);//"Pres"
        jfes_set_object_property(jconf, obj, jfes_create_double_value(jconf, (double)data->sens.temp), Items[i++], 0);//"Temp"
        jfes_set_object_property(jconf, obj, jfes_create_double_value(jconf, (double)data->sens.lux), Items[i++], 0);//"Lux"

        if (data->sens.chip == BME280_SENSOR) {
        	jfes_set_object_property(jconf, obj, jfes_create_double_value(jconf, (double)data->sens.humi), Items[i], 0);//"Humi"
        }

        jfes_size_t stx_size  = (jfes_size_t)max_len_buf;
        jfes_value_to_string(obj, buf, &stx_size, 1);
        *(buf + stx_size) = '\0';

        jfes_free_value(jconf, obj);

        ret = 0;
    }

#else

	char tmp[64];
	int len = 4;

	strcpy(buf, "{\r\n");

	for (int i = 0; i < maxItems; i++) {
		tmp[0] = '\0';
		switch (i) {
			case 0://"SeqNum",
				len += sprintf(tmp, "\t\"%s\": %lu,\r\n", Items[i], ++infCounter);
				break;
			case 1://"MsgType",
				len += sprintf(tmp, "\t\"%s\": \"+CGNSINF\",\r\n", Items[i]);
				break;
			case 2://"DevID",
				len += sprintf(tmp, "\t\"%s\": \"%s\",\r\n", Items[i], devID);
				break;
			case 3://"DevName",
				len += sprintf(tmp, "\t\"%s\": \"%s\",\r\n", Items[i], dev_name);
				break;
			case 4://"SimNumber",
				len += sprintf(tmp, "\t\"%s\": \"%s\",\r\n", Items[i], sim_num);
				break;
			case 5://"DevTime",
				len += sprintf(tmp, "\t\"%s\": %lu,\r\n", Items[i], get_secCounter());
				break;
			case 6://"FreeMem",
				len += sprintf(tmp, "\t\"%s\": %u,\r\n", Items[i], xPortGetFreeHeapSize());
				break;
			case 7://"EpochTime",
				if (setDate) {
#ifdef SET_RTC_TMR
					uint32_t ep = getSecRTC(&hrtc);
#else
					uint32_t ep = get_extDate();
#endif
					len += sprintf(tmp, "\t\"%s\": %lu,\r\n", Items[i], ep);
				} else memset(tmp, 0, sizeof(tmp));
				break;
			case 8://"UTC",
				len += sprintf(tmp, "\t\"%s\": \"%02u.%02u.%02u %02u:%02u:%02u.%03u\",\r\n", Items[i],
        						data->inf.utc.day, data->inf.utc.mon, data->inf.utc.year,
								data->inf.utc.hour, data->inf.utc.min, data->inf.utc.sec, data->inf.utc.ms);
				break;
/*
			case 9://"Run",
        		len += sprintf(tmp, "\t\"%s\": %u,\r\n", Items[i], data->inf.run);
        		break;
*/
			case 9://"Status",
				len += sprintf(tmp, "\t\"%s\": \"%s\",\r\n", Items[i], nameValid[data->inf.status&1]);
				break;
			case 10://"Latitude",
				len += sprintf(tmp, "\t\"%s\": %f,\r\n", Items[i], data->inf.latitude);
				break;
			case 11://"Longitude",
				len += sprintf(tmp, "\t\"%s\": %f,\r\n", Items[i], data->inf.longitude);
				break;
			case 12://"Altitude",
				len += sprintf(tmp, "\t\"%s\": %d,\r\n", Items[i], data->inf.altitude);
				break;
			case 13://"Speed",
				len += sprintf(tmp, "\t\"%s\": %.2f,\r\n", Items[i], data->inf.speed);
				break;
			case 14://"Dir",
				len += sprintf(tmp, "\t\"%s\": %.2f,\r\n", Items[i], data->inf.dir);
				break;
/*
			case 15://"Mode",
        		len += sprintf(tmp, "\t\"%s\": %u,\r\n", Items[i], data->inf.mode);
        		break;
*/
			case 15://"HDOP",
				//len += sprintf(tmp, "\t\"%s\": %.2f,\r\n", Items[i], data->inf.HDOP);
				break;
			case 16://"PDOP",
				//len += sprintf(tmp, "\t\"%s\": %.2f,\r\n", Items[i], data->inf.PDOP);
				break;
			case 17://"VDOP",
				//len += sprintf(tmp, "\t\"%s\": %.2f,\r\n", Items[i], data->inf.VDOP);
				break;
			case 18://"SatGPSV",
				len += sprintf(tmp, "\t\"%s\": %u,\r\n", Items[i], data->inf.GPSsatV);
				break;
			case 19://"SatGNSSU",
				len += sprintf(tmp, "\t\"%s\": %u,\r\n", Items[i], data->inf.GNSSsatU);
				break;
			case 20://"SatGLONASSV",
				len += sprintf(tmp, "\t\"%s\": %u,\r\n", Items[i], data->inf.GLONASSsatV);
				break;
			case 21://"dBHz",
				//len += sprintf(tmp, "\t\"%s\": %u,\r\n", Items[i], data->inf.dBHz);
				break;
/*
			case 22://"HPA",
        		len += sprintf(tmp, "\t\"%s\": %.2f,\r\n", Items[i], data->inf.HPA);
				break;
			case 23://"VPA",
        		len += sprintf(tmp, "\t\"%s\": %.2f,\r\n", Items[i], data->inf.VPA);
        		break;
*/
			case 22://"RSSI"
				len += sprintf(tmp, "\t\"%s\": %d,\r\n", Items[i], dBmRSSI[gsm_stat.rssi&0x1f]);//gsm_stat.rssi);
				break;
			case 23://"Press",
				len += sprintf(tmp, "\t\"%s\": %.2f,\r\n", Items[i], data->sens.pres);
				break;
			case 24://"Temp",
				len += sprintf(tmp, "\t\"%s\": %.2f,\r\n", Items[i], data->sens.temp);
				break;
			case 25://"Lux",
				if (data->sens.chip == BME280_SENSOR) {
					len += sprintf(tmp, "\t\"%s\": %.2f,\r\n", Items[i], data->sens.lux);
				} else {
					len += sprintf(tmp, "\t\"%s\": %.2f\r\n", Items[i], data->sens.lux);
				}
				break;
			case 26://"Humi"
				if (data->sens.chip == BME280_SENSOR) {
					len += sprintf(tmp, "\t\"%s\": %.2f\r\n", Items[i], data->sens.humi);
				} else memset(tmp, 0, sizeof(tmp));
				break;
		}//switch (i)

		if (len < max_len_buf)
			sprintf(buf+strlen(buf), "%s", tmp);
		else
			break;

	}//for

	strcat(buf, "}");

	ret = 0;

#endif

	return ret;
}
//-----------------------------------------------------------------------------------------
void toDisplay(const char *st, uint8_t column, uint8_t line, bool clear)
{

#ifdef SET_OLED_I2C
	  if (!i2cError) {
		  if (clear) i2c_ssd1306_clear_line(line);
		  if (!column)
			  i2c_ssd1306_text_xy(st, ssd1306_calcx(strlen(st)), line);
		  else
			  i2c_ssd1306_text_xy(st, column, line);
	  }
#endif

#ifdef SET_OLED_SPI
	  if (clear) spi_ssd1306_clear_line(line);
	  if (!column)
		  spi_ssd1306_text_xy(st, ssd1306_calcx(strlen(st)), line);
	  else
		  spi_ssd1306_text_xy(st, column, line);
#endif

}
//------------------------------------------------------------------------------------

#ifdef SET_SMS

//-----------------------------------------------------------------------------
void *getMem(size_t len)
{
#ifdef SET_CALLOC_MEM
		return (calloc(1, len));
#else
	#ifdef SET_MALLOC_MEM
		return (malloc(len));
	#else
		return (pvPortMalloc(len));
	#endif
#endif
}
//-----------------------------------------------------------------------------
void freeMem(void *mem)
{
#if defined(SET_CALLOC_MEM) || defined(SET_MALLOC_MEM)
		free(mem);
#else
		vPortFree(mem);
#endif
}
//-----------------------------------------------------------------------------

void InitSMSList()
{
	for (uint8_t i = 0; i < maxSMSPart; i++) memset((uint8_t *)&sms_rec[i], 0, sizeof(s_udhi_t));
}
//-----------------------------------------------------------------------------
uint8_t PutSMSList(s_udhi_t *rec)
{
	if (!rec->part || rec->part > maxSMSPart) return 255;

	memcpy((uint8_t *)&sms_rec[rec->part - 1], (uint8_t *)rec, sizeof(s_udhi_t));

	return rec->part;
}
//-----------------------------------------------------------------------------
uint8_t LookAllPart(uint8_t total)
{
uint8_t ret = 0;

	if (total > maxSMSPart) return ret;

	for (uint8_t i = 0; i < total; i++) if (sms_rec[i].part) ret++;

	return ret;
}
//------------------------------------------------------------------------------
uint8_t ConcatSMS(char *buf, uint8_t total, uint16_t *sn, uint16_t *sl)
{

	if (total > maxSMSPart) return 0;

	uint16_t num = 0, len = 0, dl;
	uint8_t done = 0, i = 0;

	while (!done) {
		if (!num) {
			if (sms_rec[i].num) num = sms_rec[i].num;
		}
		if (!sms_rec[i].len) sms_rec[i].len = sprintf(sms_rec[i].txt, "--- part %u ---", i + 1);

		if ((len + sms_rec[i].len) > (SMS_BUF_LEN - 1)) {
			dl = SMS_BUF_LEN - 1 - len;
			done = 1;
		} else dl = sms_rec[i].len;

		memcpy(buf + len, sms_rec[i].txt, dl);
		len += dl;

		i++;
		if ((done) || (i >= total)) break;
	}
	*sn = num;
	*sl = len;

	return total;
}
//------------------------------------------------------------------------------
uint8_t getSMSTotalCounter()
{
uint8_t ret = 0;

	for (uint8_t i = 0; i < maxSMSPart; i++) {
		if (sms_rec[i].total) {
			ret = sms_rec[i].total;
			break;
		}
	}

	return ret;
}
//------------------------------------------------------------------------------
/*
uint16_t SwapBytes(uint16_t word)
{
	return ((word >> 8) | (word << 8));
}
*/
//------------------------------------------------------------------------------
int gsm7bit_to_text(int len_inbuff, uint8_t *inbuff, uint8_t *outbuff, int fl, uint8_t max_udl, uint8_t u_len)
{
int dl_ind = 0, i = 0, shift = 1, lb = max_udl;
uint8_t a, b, prev = 0;
uint8_t *ps1 = inbuff;
uint8_t *ps2 = outbuff;
uint8_t words[4] = {0};

	if (!inbuff || !outbuff || !len_inbuff) return dl_ind;

    if (u_len > 0) lb -= (u_len + 1);
    lb--;

    memcpy(words, ps1 - 2, 2);
    b = hextobin(words[0], words[1]);
    if ((b > 0) && (fl)) {
    	*(uint8_t *)ps2 = b >> 1;
    	ps2++;
    	dl_ind++;
    }

    while (1) {
    	memcpy(words, ps1, 2);
    	a = hextobin(words[0], words[1]);
    	ps1 += 2;
    	i += 2;
    	b = a;
    	a <<= (uint8_t)(shift - 1);
    	a = (a | prev) & 0x7f;
    	prev = (b >> (8 - shift));
    	*(uint8_t *)ps2 = a;
    	ps2++;
    	dl_ind++;
    	if (shift != 7) shift++;
    	else {
    		*(uint8_t *)ps2 = prev;
    		ps2++;
    		dl_ind++;
    		shift = 1;
    		prev = 0;
    	}
    	if ((dl_ind > lb) || (i >= SMS_BUF_LEN - 1)) break;
    }

    return (dl_ind);
}
//----------------------------------------------------------------------------------
int conv_ucs2_text(uint8_t *buffer_txt, char *fromik, uint8_t *udhi5, uint8_t prn)
{
int ret = 0;
int tt, tt1 = 0, tt_n, tt1_n, len, i = 0, j = 0, k = 0, shift, yes, it = 0, tzone, end_ind = 0;
char *ps1, *ps_begin, *ps3, *uk_start, *pss, *pss0, *ps_o, *ps_type, *ina2, *qik, *ps0;
char *uk_start7 = NULL;
char *ina = NULL;
char *ina0 = NULL;
char *ps_sta = NULL;
char *nachalo = NULL;
uint16_t dcs;
uint8_t a, a_n, b, b_n, c, prev, dl = 0, dl_ind, new_a;
uint8_t pdu_type = 0xff, type_num_a, len_num_a, user_data_len = 0;
uint8_t user_data_l = 0, udhi_len = 0, len_sca = 0, tp_mti = 0, tp_vpf = 0;
char words[5], chcs[12];
char words1[34], words2[34];
char stx[256];
int pdu, pack, ind_tp = 3, its_ok = 0, ofs = 0, with_udh = 0, flg = 0;
char udhi_str[32] = {0}, sca_str[32] = {0};
uint8_t udhi_4[5] = {0};
uint8_t buffer_temp[SMS_BUF_LEN] = {0};

	if ((ps0 = strstr((char *)buffer_txt, "\r\nOK\r\n")) != NULL) *(ps0) = '\0';

    len  =  strlen((char *)buffer_txt);

	if ((len < 7) || (len > 512)) {
		*buffer_txt = 0;
		return ret;
	} else {
		tt = pdu = 0;
		ps1 = (char *)&buffer_txt[0];
		ps0 = nachalo = ps1;
		ps_begin = ps_sta = ps1;
		memset(words, 0, sizeof(words));
		i = j = 0;
		ofs = 0;
		for (k = 0; k < max_smsType; k++) {
			ina0 = strstr(ps1, smsType[k]);////+CMT: ,,26 //+CMT: ,26 //+CMT: "",26 | "+CLASS0: " | "+CMGR: "
			if (ina0) {
				ina = ina0;
				ofs = strlen(smsType[k]);
				break;
			}
		}
		if (ina) {
			ina += ofs;
			ina2 = strstr(ina, "\r\n");
			if (ina2) {
				ina2 += 2;
				nachalo = ina2;//начало pdu !!!!!!!!!!!!!!!!!!!!!!!!!!
				if ((ina2 - ps1) < (len - ofs)) its_ok = 1;
				//---------------------------------------------------------
				k = 2;
				qik = strstr(ina, ",,");
				if (!qik) {
					qik = strstr(ina, "\",");
					if (!qik) {
						qik = strchr(ina, ',');
						if (qik) k = 1;
					} else k = 2;
				}
				if (qik) {
					qik += k;//указатель на начало длинны сообщения в байтах
					if ((ina2 - qik) > 1) i = ina2 - qik - 2;//!!!!!!!!!!!!!!! //1;//количество символов длинны самого pdu в байтах
					if ((i > 0) && (i < 4)) {
						memset(words, 0, sizeof(words));
						memcpy(words, qik, i);
						j = atoi(words);//собственно длинна pdu в байтах, указанная с сторке +CMGR: 1,,113
						j <<= 1;//собственно длинна pdu в символах в строке +CMGR: 1,,113
						k = strlen(ina2);//принято символов тела pdu
						if (k > j) {
							memset(words1, 0, sizeof(words1));
							memcpy(words1, ina2, 2);
							i = atoi(words1);//длинна номера sca
							i <<= 1;
							len_sca = i;
							ina2 += i + 2;
						}
					}
				}
				//---------------------------------------------------------
				ps1 = ps_begin = ina2;
				uk_start = ps1;
			}
		}

		if (!its_ok) {
			Report(false, "Sender number not found\r\n");
			uk_start7 = strstr(ps_begin, "00");
			if (uk_start7)  {
				if (uk_start7 == ps_sta) {
					its_ok = 1;
					nachalo = uk_start7 + 2;
					uk_start = nachalo;
				}
			}
		}


		if (its_ok) {
			memset(sca_str, 0, sizeof(sca_str));
			memset(words, 0, sizeof(words));
			if (nachalo) {
				memcpy(words, nachalo, 2);
				b = hextobin(words[0], words[1]);//sca_len
				if ((b > 0) && (b <= 7)) memcpy(sca_str, nachalo, (b + 1) << 1);
				if (!b) nachalo += 2;//1-й байт (длинна) = 0 -> номер смс-центра отсутствует
				   else nachalo += ((b + 1) << 1);//указатель на pdu_type
				memset(words, 0, sizeof(words));
				memcpy(words, nachalo, 2);
				pdu_type = hextobin(words[0], words[1]);//pdu_type
				if (pdu_type & 0x40) {
					with_udh = 1;
					memset(udhi_str, 0, sizeof(udhi_str));
					udhi_len = 0;
				}
				if (pdu_type & 1) tp_mti = 2; else tp_mti = 0;//есть MR или нет
				tp_vpf = ((pdu_type >> 3) & 3);//for submit sms
				uk_start = nachalo;
			}

			sprintf(stx, "CMGR/CMT_LEN=%s|%d, got_len=%d, PDU_TYPE=0x%02X", words, j, k, pdu_type);
			if (len_sca > 0) sprintf(stx+strlen(stx), ", SCA[%d]=%s,", len_sca, sca_str);
			if (with_udh) strcat(stx," With_UDHI");
					 else strcat(stx," Without_UDHI");
			//sprintf(stx+strlen(stx),"\r\nPDU:\r\n%s\r\n", uk_start);
			if (prn) Report(false, "%s\r\n", stx);

			ps1 = ps3 = uk_start;

			ps_type = uk_start + 4 + tp_mti;//указатель на тип номера
			memcpy(words, ps_type, 2);
			b = hextobin(words[0], words[1]);//тип номера
			b = (b & 0x70) >> 4; //нужны разряды bit6 bit5 bit4 - это тип номера A
			type_num_a = b;//тип номера А
			if (type_num_a > 8) type_num_a = 8;
			ps1 = ps1 + 2 + tp_mti;//указатель на кол-во символов в номере A - 0x0c
			memcpy(words, ps1, 2);
			a = hextobin(words[0], words[1]);
			if ((a & 1) == 1) a++;
			if (a > 32) a = 32;
			new_a = a;
			len_num_a = a;//длинна номера A !!!!!!!!!!!!!
			ps1 += 4;//указатель на начало номера A - 0x83
			memset(words1, 0, sizeof(words1));
			memcpy(words1, ps1, a);
			if (b != 5) {//надо переставить местами цифры номера
				j = 0;
				while (j < a) {
					c = words1[j];
					words1[j] = words1[j + 1];
					words1[j + 1] = c;
					j += 2;
				}
				if (words1[a - 1] == 'F') words1[a - 1] = ' ';
			} else {// номер закодирован в GSM-7bit - ПРИКИНЬ !  ВОТ УРОДЫ !!!
				j = 0; 	i = 0;
				memset(words2, 0, sizeof(words2));
				while (j < a) {
					words2[i] = hextobin(words1[j], words1[j + 1]);
					i++; j += 2;
				}
				//-------------- decoding num_a from GSM-7bit to KOI8-R -----------
				memset(words1, 0, sizeof(words1));
				tt_n = tt1_n = 0;
				dl_ind = prev = 0;
				dl = i;
				shift = 1;
				while (1) {
					a_n = words2[tt_n++];
					b_n = a_n;
					a_n <<= (uint8_t)(shift - 1);
					a_n = (a_n | prev) & 0x7f;
					words1[tt1_n++] = a_n;
					dl_ind++;
					prev = (b_n >> (8 - shift));
					if (shift != 7) {
						shift++;
					} else {
						words1[tt1_n++] = prev;
						dl_ind++;
						shift = 1;
						prev = 0;
					}
					if (dl_ind > dl) break;
				}
				//------------------------------------------------------------------------------------
				new_a = strlen(words1);
				a >>= 1;
			}

			end_ind = tt1 = 0;
			if (fromik) memset(fromik, 0, 32);    //макс-ая длинна номера А = 31
			if (b == 1) buffer_temp[tt1++] = '+'; //type_of_num_A = intern....
			ps_o = (char *)&buffer_temp[tt1];
			if (new_a > 0) {
				memcpy(ps_o, words1, new_a);
				tt1 += new_a;
				buffer_temp[tt1++] = 0x20;
			}
			memcpy(words, ps1 + len_num_a, 2);	  //p_id=hextobin(words[0], words[1]);
			pss = ps1 + len_num_a + 2;            //указатель на DCS

			memcpy(words, pss, 2);
			dcs = hextobin(words[0], words[1]);
			c = (dcs & 0x0c) >> 2;
			pdu = 0;//default
			switch (c) {
				case 0:
				case 1:
				case 3: pdu = 0; break;
				case 2: pdu = 1; break;
			}
			if (c > 2) c = 3;
			ind_tp = c;       //индекс типа кодировки, в которой получено сообщение bit3 bit2 в dcs
			if ((dcs & 0x20)) //dcs & 0x20 - признак компрессии, bit5 в dcs
				pack = 1;
			else
				pack = 0;

			k = strlen(words1);
			if (words1[k-1] == ' ') words1[k - 1] = '\0';
			if (fromik) {
				k = strlen(words1);
				if (k > lenFrom - 1) k = lenFrom - 1;
				memcpy(fromik, words1, k); //макс-ая длинна номера А = 31
			}

			sprintf(stx, "DCS=0x%02X, ENC[%d]=%s", dcs, ind_tp, tp[ind_tp]);
			if (new_a > 0) sprintf(stx+strlen(stx),", SENDER: type[%d]=%s number[%d]=%s",
													type_num_a, type_name_a[type_num_a], new_a, words1);
			if (pack) sprintf(stx+strlen(stx),", PACK=%d", pack);
			//
			//---------------------------------------------------
			//
			pss += 2; //указатель на начало области date/time
			uint8_t byte = 18;
			if (tp_vpf != 2) {
				memset(words1, 0, sizeof(words1));
				memcpy(words1, pss, 14); //14 символов для date/time
				j = 0;
				while (j < 14) {
					c = words1[j];
					words1[j] = words1[j + 1];
					words1[j + 1] = c;
					j += 2;
				}

				buffer_temp[tt1++] = 0x32; 	buffer_temp[tt1++] = 0x30; //20+год

				pss = &words1[0];	pss0 = (char *)&buffer_temp[tt1];	  memcpy(pss0, pss, 2);		tt1 += 2;   //год
				buffer_temp[tt1++] = '/';
				pss += 2; 		    pss0 = (char *)&buffer_temp[tt1];	  memcpy(pss0, pss, 2);		tt1 += 2;   //месяц
				buffer_temp[tt1++] = '/';
				pss += 2; 		    pss0 = (char *)&buffer_temp[tt1];     memcpy(pss0, pss, 2);		tt1 += 2;   //день
				buffer_temp[tt1++] = 0x20;

				pss += 2; 		    pss0 = (char *)&buffer_temp[tt1];     memcpy(pss0, pss, 2);		tt1 += 2;   //часы
				buffer_temp[tt1++] = ':';
				pss += 2; 		    pss0 = (char *)&buffer_temp[tt1];     memcpy(pss0, pss, 2);		tt1 += 2;   //минуты
				buffer_temp[tt1++] = ':';
				pss += 2; 		    pss0 = (char *)&buffer_temp[tt1];     memcpy(pss0, pss, 2);		tt1 += 2;   //секунды
				buffer_temp[tt1++] = '+';
				pss += 2;

				memset(chcs, 0, sizeof(chcs)); memcpy(chcs, pss, 2); tzone = atoi(chcs); tzone = (tzone * 15) / 60;
				memset(chcs, 0, sizeof(chcs)); sprintf(chcs, "%02d", tzone); pss = &chcs[0];
				pss0 = (char *)&buffer_temp[tt1]; memcpy(pss0, pss, 2); tt1 += 2;   //(+02)
			} else {
				pss += 2;
				byte = 6;
			}
			tt = (ps1 - ps0) + len_num_a + byte; //указатель на UDL - user data len
			memcpy(&buffer_temp[tt1], eolin, strlen(eolin)); tt1 += strlen(eolin);
			end_ind = tt1;
			//
			ps3 = (char *)&buffer_txt[tt];
			memset(words, 0, sizeof(words));
			memcpy(words, ps3, 2);
			dl = hextobin(words[0], words[1]); //длинна тела сообщения
			user_data_l = user_data_len = dl;
			sprintf(stx+strlen(stx),", UDL=%d[%s]\r\n", user_data_len, words);
			if (prn) Report(false, stx);

			tt += 2; //индекс на начало текста сообщения или на начало udhi
		}//if (its_ok)
		//
		//----------------------------------------------------------------
		//
		dl_ind = 0;
		if (pdu == 1) { //тело сообщения в UCS2

			if (with_udh) { //Это часть сообщения
				ps1 = (char *)&buffer_txt[tt];
				memset(words, 0, sizeof(words));
				memcpy(words, ps1, 2);
				udhi_len = hextobin(words[0], words[1]);
				if (udhi_len <= 32) {
					memcpy(udhi_str, ps1 + 2, udhi_len << 1);
					tt += ((udhi_len + 1) << 1); //NEW BODY POINTER (index)
					dl -= udhi_len + 1;
					user_data_len = dl;
				}
			}

			while (1) {
				ps1 = (char *)&buffer_txt[tt];
				memset(words, 0, sizeof(words));
				memcpy(words, ps1, 4);
				yes = 0;
				for (j = 0; j < cod_PDU_len; j++) {
					sprintf(chcs, "%04X", cod_PDU[j]);
					if (!strncmp(words, chcs, 4)) {
						yes = 1;
						it = j;
						break;
					}
				}
				if (yes == 1) buffer_temp[tt1++] = alphabet[it];
					     else buffer_temp[tt1++] = '.';
				tt += 4;   dl_ind++;
				if (((tt + 4) > len) || (dl_ind > dl)) break;
			}
		} else {//if (pdu == 1)
			switch (ind_tp) {
				case 0 ://7 bit encoding
					k = 0;
					if (with_udh) {
						ps1 = (char *)&buffer_txt[tt];
						memset(words, 0, sizeof(words));
						memcpy(words, ps1, 2);
						udhi_len = hextobin(words[0], words[1]);
						if (udhi_len <= 32) {
							memcpy(udhi_str, ps1 + 2, udhi_len << 1);
							tt += ((udhi_len + 1) << 1);//NEW BODY POINTER
							k = 7 - (udhi_len + 1);
							tt += k << 1;
							if (k > 0) flg = k; else flg = 0;
							dl -= udhi_len + 1;
							k = dl * 7;
							dl = k >> 3;
							if (k % 8) dl++;
							user_data_len = dl;
						}
					} else {
						flg = 0;
					}

					dl_ind = gsm7bit_to_text(dl, &buffer_txt[tt], &buffer_temp[end_ind], flg, user_data_l, udhi_len);

					tt1 += dl_ind;
				break;
				case 1://8 bit encoding
					if (with_udh) {
						ps1 = (char *)&buffer_txt[tt];
						memcpy(words, ps1, 2);
						udhi_len = hextobin(words[0], words[1]);
						if (udhi_len <= 32) {
							memcpy(udhi_str, ps1 + 2, udhi_len << 1);
							tt += ((udhi_len + 1) << 1);//NEW BODY POINTER
							dl -= udhi_len + 1;
							user_data_len = dl;
						}
					}

					while (1) {
						ps1 = (char *)&buffer_txt[tt];
						memcpy(words, ps1, 2);
						buffer_temp[tt1++] = hextobin(words[0], words[1]);
						tt += 2;
						dl_ind++;
						if ((tt >= len) || (dl_ind >= dl)) break;
					}
				break;
			}//switch (ind_tp)
		}//else -> 7 ! 8 bit encoding

		buffer_temp[tt1] = 0;

		if (with_udh) {
			if (udhi_len >= 5) {
				udhi_4[0] = 1; //tp
				a = hextobin(udhi_str[2], udhi_str[3]);
				b = (udhi_len << 1);
				if (a == 4) { //2 байта на номер смс
					b -= 8;
					udhi_4[1] = hextobin(udhi_str[b], udhi_str[b + 1]); //num_1
					b += 2;
				} else { //1 байт на номер смс
					udhi_4[1] = 0; //num_1
					b -= 6;
				}
				udhi_4[2] = hextobin(udhi_str[b],     udhi_str[b + 1]); //num_2
				udhi_4[3] = hextobin(udhi_str[b + 2], udhi_str[b + 3]); //total
				udhi_4[4] = hextobin(udhi_str[b + 4], udhi_str[b + 5]); //part
			}
		//}
			a = udhi_4[1];   udhi_4[1] = udhi_4[2];   udhi_4[2] = a;//swapbytes in sms_num
			memcpy(udhi5, udhi_4, 5);

		//if (with_udh) {
			if (TSINPART) {
				end_ind = 0;
			} else {
				if (udhi_4[4] < 2) end_ind = 0;
			}
		} else {
			if (TSINPART) end_ind = 0;
		}

		if (end_ind > SMS_BUF_LEN - 1) end_ind = 0;

		it = SMS_BUF_LEN - 1 - end_ind;

		//memset(buffer_txt, 0, SMS_BUF_LEN);
		memcpy(buffer_txt, &buffer_temp[end_ind], it);

		if (with_udh && prn) Report(false,"UDH(%d): [%s]\r\n", udhi_len, udhi_str);

		ret = tt1 - end_ind;//strlen((char *)buffer_txt);//dl_ind;

	}

 	return ret;
}
//-----------------------------------------------------------------------------------------
int ucs2_to_text(char *buf_in, uint8_t *buf_out)
{
	if (!buf_in) return 0;
	int len = strlen(buf_in);	if (!len) return 0;

	int dl_ind = 0, yes, it = 0, j, tt = 0, tt1 = 0;
	char words[5] = {0}, tmp[5];

    while (1) {
    	memcpy(words, &buf_in[tt], 4);
    	yes = 0;
    	for (j = 0; j < cod_PDU_len; j++) {
    		sprintf(tmp,"%04X", cod_PDU[j]);
    		if (!strncmp(words, tmp, 4)) {
    			yes = 1;
    			it = j;
    			break;
    		}
    	}
    	if (yes) {
    		buf_out[tt1++] = alphabet[it];
    	} else {
    		buf_out[tt1++] = '.';
    	}

    	tt += 4;
    	dl_ind++;
    	if (tt >= len) break;
    }

    buf_out[dl_ind] = '\0';

    return dl_ind;
}
//-----------------------------------------------------------------------------------------
int8_t makeSMSString(const char *body, uint16_t *blen, char *fnum, uint16_t snum, char *buf, int max_len_buf)
{
int8_t ret = -1;

#ifdef SET_JFES
    uint8_t i = 0;
    jfes_value_t *obj = jfes_create_object_value(jconf);
    if (obj) {
    	jfes_set_object_property(jconf, obj, jfes_create_integer_value(jconf, ++infCounter), Items[i++], 0);//"SeqNum"
    	jfes_set_object_property(jconf, obj, jfes_create_string_value(jconf, "SMS", 0), Items[i++], 0);//"MsgType"
    	if (strlen(devID)) jfes_set_object_property(jconf, obj, jfes_create_string_value(jconf, devID, 0), Items[i++], 0);//"DevID"
    	jfes_set_object_property(jconf, obj, jfes_create_string_value(jconf, dev_name, 0), Items[i++], 0);//"DevName"
    	jfes_set_object_property(jconf, obj, jfes_create_string_value(jconf, sim_num, 0), Items[i++], 0);//"SimNumber"
    	jfes_set_object_property(jconf, obj, jfes_create_integer_value(jconf, get_secCounter()), Items[i++], 0);//"DevTime"
    	jfes_set_object_property(jconf, obj, jfes_create_integer_value(jconf, xPortGetFreeHeapSize()), Items[i++], 0);//"FreeMem"
    	if (setDate) {
    	#ifdef SET_RTC_TMR
    	            uint32_t ep = getSecRTC(&hrtc);
    	#else
    	            uint32_t ep = get_extDate();
    	#endif
    	            jfes_set_object_property(jconf, obj, jfes_create_integer_value(jconf, ep), Items[i++], 0);//"EpochTime"
    	}
    	jfes_set_object_property(jconf, obj, jfes_create_integer_value(jconf, snum), "smsNumber", 0);
    	jfes_set_object_property(jconf, obj, jfes_create_string_value(jconf, fnum, 0), "fromNumber", 0);
    	jfes_set_object_property(jconf, obj, jfes_create_string_value(jconf, body, 0), "smsBody", 0);

        jfes_size_t stx_size  = (jfes_size_t)max_len_buf;
        jfes_value_to_string(obj, buf, &stx_size, 1);
        *(buf + stx_size) = '\0';
        strcat(buf, "\r\n");

        jfes_free_value(jconf, obj);

        ret = 0;
    }

#else
    uint16_t body_len = *blen;
    strcpy(buf, "{\r\n");
    int len = 4;

    sprintf(buf+strlen(buf), "\t\"SeqNum\": %lu,\r\n", ++infCounter);
    sprintf(buf+strlen(buf), "\t\"MsgType\": \"SMS\",\r\n");
    sprintf(buf+strlen(buf), "\t\"DevID\": \"%s\",\r\n", devID);
    sprintf(buf+strlen(buf), "\t\"DevName\": \"%s\",\r\n", dev_name);
    sprintf(buf+strlen(buf), "\t\"SimNumber\": \"%s\",\r\n", sim_num);
    sprintf(buf+strlen(buf), "\t\"DevTime\": %lu,\r\n", get_secCounter());
    sprintf(buf+strlen(buf), "\t\"FreeMem\": %u,\r\n",xPortGetFreeHeapSize());
    if (setDate) {
    #ifdef SET_RTC_TMR
    	uint32_t ep = getSecRTC(&hrtc);
	#else
    	uint32_t ep = get_extDate();
	#endif
    	sprintf(buf+strlen(buf), "\t\"EpochTime\": %lu,\r\n", ep);
    }
    sprintf(buf+strlen(buf), "\t\"smsNumber\": %u,\r\n", snum);
    sprintf(buf+strlen(buf), "\t\"fromNumber\": \"%s\",\r\n", fnum);
    len += strlen(buf);
    if ((len + body_len + 16) > max_len_buf) body_len = max_len_buf - len - 16;
    sprintf(buf+strlen(buf), "\t\"smsBody\": \"%.*s\"\r\n}\r\n", body_len, body);

    ret = 0;

#endif

    *blen = (uint16_t)strlen(buf);

    return ret;
}
//-----------------------------------------------------------------------------
void initSMSQ(s_smsq_t *sq)//s_smsq_t smsq;
{
	sq->put = sq->get = 0;
	for (uint8_t i = 0; i < MAX_QSMS; i++) {
		sq->sms[i].id = i;
#ifdef SET_SMSQ_STATIC
		sq->sms[i].adr[0] = '\0';
#else
		sq->sms[i].adr = NULL;
#endif
	}
}
//-----------------------------------------------------------------------------
void clearSMSQ(s_smsq_t *sq)
{
	sq->put = sq->get = 0;
	for (uint8_t i = 0; i < MAX_QSMS; i++) {
		sq->sms[i].id = i;
#ifdef SET_SMSQ_STATIC
		sq->sms[i].adr[0] = '\0';
#else
		freeMem(sq->sms[i].adr);
		sq->sms[i].adr = NULL;
#endif
	}
}
//-----------------------------------------------------------------------------
int8_t putSMSQ(char *adr, s_smsq_t *sq)
{
int8_t ret = -1;

#ifdef SET_SMSQ_STATIC
	if (!strlen(sq->sms[sq->put].adr)) {
		int len = strlen(adr);
		if (len >= SMS_BUF_LEN) len = SMS_BUF_LEN - 1;
		memcpy((char *)&sq->sms[sq->put].adr[0], adr, len);
		sq->sms[sq->put].adr[len] = '\0';
#else
	if (sq->sms[sq->put].adr == NULL) {
		sq->sms[sq->put].adr = adr;
#endif
		ret = sq->sms[sq->put].id;
		sq->put++; if (sq->put >= MAX_QSMS) sq->put = 0;
	}

	return ret;
}
//-----------------------------------------------------------------------------
int8_t getSMSQ(char *dat, s_smsq_t *sq)
{
int8_t ret = -1;
int len = 0;

#ifdef SET_SMSQ_STATIC
	len = strlen(sq->sms[sq->get].adr);
	if (len) {
		ret = sq->sms[sq->get].id;
		memcpy(dat, sq->sms[sq->get].adr, len);
		sq->sms[sq->get].adr[0] = '\0';
	}
#else
	if (sq->sms[sq->get].adr != NULL) {
		len = strlen(sq->sms[sq->get].adr);
		ret = sq->sms[sq->get].id;
		memcpy(dat, sq->sms[sq->get].adr, len);
		freeMem(sq->sms[sq->get].adr);
		sq->sms[sq->get].adr = NULL;
	}
#endif

	if (ret >= 0) {
		*(dat + len) = '\0';
		sq->get++;
		if (sq->get >= MAX_QSMS) sq->get = 0;
	}

	return ret;
}
//------------------------------------------------------------------------------------------
int8_t addToSMSQ(char *txt, uint16_t txt_len, s_smsq_t *sq)
{
int8_t nrec = -1;

#ifdef SET_SMSQ_STATIC
	if (txt_len >= SMS_BUF_LEN) {
		txt_len = SMS_BUF_LEN - 1;
		txt[txt_len] = '\0';
	}
	if ((nrec = putSMSQ(txt, sq)) >= 0) {
		Report(true, "[%s] : add sms_record OK (id=%d len=%d)\r\n", __func__, nrec, txt_len);
	} else {
		Report(true, "[%s] : add sms_record error (len=%d)\r\n", __func__, txt_len);
	}
#else
	int need_len = txt_len + 1;
	if (need_len <= MAX_UART_BUF) need_len = MAX_UART_BUF;
	char *rc = (char *)getMem((size_t)need_len);
	if (rc) {
		int got_len = strlen(rc);
		if (got_len >= need_len) {
			memcpy(rc, txt, txt_len);
			*(rc + txt_len) = '\0';
			if ((nrec = putSMSQ(rc, sq)) >= 0) {
				Report(true, "[%s] : add sms_record OK (id=%d len=%d/%d)\r\n", __func__, nrec, need_len, got_len);
		    } else {
				Report(true, "[%s] : add sms_record error (len=%d/%d)\r\n", __func__, need_len, got_len);
				freeMem(rc);
			}
		} else {
			Report(true, "[%s] : error memory size %d != %d\r\n", __func__, need_len, got_len);
			freeMem(rc);
		}
	} else Report(true, "[%s] : error get memory (len=%d)\r\n", __func__, need_len);
#endif

	return nrec;
}
//------------------------------------------------------------------------------------------

#endif

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

	osDelay(1200);

	//rmc5 = 1;
	initQ(&q_gps);

	s_inf_t inf;
	result_t levt;
	s_data_t iData;

	char toScreen[SCREEN_SIZE];
	uint8_t row;
	uint8_t show;
	uint8_t new = 0;
	bool stp = true;

	/* Infinite loop */

  	while (LoopAll)   {

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
  			if (getQ(msgNMEA, &q_gps) >= 0) {
  				if (flags.log_show) stp = true; else stp = false;
  				Report(stp, msgNMEA);
  				if (!parse_inf(msgNMEA, &inf)) {
  					flags.msg_end = 1;
  					memcpy((uint8_t *)&iData.inf, (uint8_t *)&inf, sizeof(s_inf_t));
  					new |= 2;
  				}
  			}
  		}
  		//--------------------------------------------------------------------------
  		//
  		if (flags.restart) {
  			flags.restart = 0;
  			if (gprs_stat.connect) flags.disconnect = 1;
  			Report(true, "Restart ARM !\r\n");
  			osDelay(1500);
  			NVIC_SystemReset();
  			break;
  		}
  		//
  		if (flags.stop) {
  			flags.stop = 0;
  			if (gprs_stat.connect) flags.disconnect = 1;
  			sprintf(toScreen, "Stop All");
  			Report(true, "%s!\r\n", toScreen);
  			toDisplay((const char *)toScreen, 0, 5, false);
  			osDelay(1500);
  			gsmONOFF(ModuleOFF);
  			uint8_t ct = 6;
  			while (getVIO()) {
  				gsmONOFF(ModuleOFF);
  				osDelay(1500);
  				ct--;
  				if (!ct) break;
  			}
  			LoopAll = false;
  			break;
  		}

  		//-------------------------------------------------------------------------
  		if (new == 3) {
  			new = 0;
  			osMessageQueuePut(mqData, (void *)&iData, 0, 20);
  		}
  		//-------------------------------------------------------------------------

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
	int8_t who = -1;
	msgGPRS[0] = 0;
	char cmd[64];
	int dl, i, j, k;
	uint16_t gprs_len;

#ifdef SET_SMS
	initSMSQ(&smsq);//char *rc = (char *)pvPortMalloc(len); vPortFree(rc);
					//char *rc = (char *)calloc(1, len); free(rc);
	char fromNum[lenFrom] = {0};
	uint8_t abcd[5] = {0};
	uint16_t sms_num;
	uint16_t sms_len;
	uint8_t sms_total;
	int8_t nrec;
	s_udhi_t reco;
	uint32_t wait_sms = 0;
	InitSMSList();
#endif

	char toScr[SCREEN_SIZE];

	tms = min_ms;

	s_data_t aData;

#if defined(SET_OLED_I2C) || defined(SET_OLED_SPI)
	sprintf(toScr, "%s", srv_adr);
	toDisplay((const char *)toScr, 0, 2, false);
#endif

	uint8_t ctn = 5, faza = 0, rx_faza = 0;
	uint32_t tmps = ModuleON;

	bool vios = getVIO();
	if (!vios) {
		gsmONOFF(tmps);
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
	wait_ack = get_tmr(2);


	/* Infinite loop */

	while (LoopAll) {

		//-----------------------------------------------------------
		if (evt_gsm) {
			if (evt_gsm == 2)
				tmps = ModuleOFF;//OFF sim868
			else
				tmps = ModuleON;//ON sim868
			vios = getVIO();
			while ((vios == getVIO()) && ctn--) {
				gsmONOFF(tmps);//   ```|_____|``` - make pulse duration tmps
				osDelay(1500);
			};

			AtParamInit(false);
			gprs_stat.init    = 0;
			gprs_stat.connect = 0;
			flags.imei_flag   = 0;
			flags.auto_cmd    = flags.inf = 0;
			ctn = 5;
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
				if (flags.connect) {
					if (cmdsDone) {
						flags.connect = 0;
						if (!gprs_stat.connect) {
							yes = true;
							sprintf(cmd, "AT+CIPSTART=\"TCP\",\"%s\",%u\r\n", srv_adr, srv_port);
							buff = cmd;
							gprs_stat.try_connect = 1;
						}
					}
				}
				if (flags.disconnect) {
					if (cmdsDone) {
						flags.disconnect = 0;
						if (gprs_stat.connect) {
							yes = true;
							buff = gprsDISCONNECT;//"AT+CIPCLOSE\r\n"
							gprs_stat.try_disconnect = 1;
						}
					}
				}
				if (osMessageQueueGet(mqData, (void *)&aData, NULL, 10) == osOK) {
					if (!makeInfString(&aData, msgGPRS, sizeof(msgGPRS) - 3)) {
						strcat(msgGPRS, "\r\n");
						gprs_len = strlen(msgGPRS);
						if (gprs_stat.connect && gprs_stat.send_ok && cmdsDone) {
							sprintf(cmd, "AT+CIPSEND=%d\r\n", gprs_len);
							yes = true;
							buff = cmd;
							who = 0;
							gprs_stat.next_send = 0;
						} else if (flags.log_show) Report(true, "%s(%u)\r\n", msgGPRS, gprs_len);
					}
				}
#ifdef SET_SMS
				else if ((nrec = getSMSQ(smsGPRS, &smsq)) >= 0) {
					sms_len = strlen(smsGPRS);
					Report(true, "[getSMSQ] : get sms_record OK (id=%d len=%u)\r\n", nrec , sms_len);
					if (gprs_stat.connect && gprs_stat.send_ok && cmdsDone) {
						sprintf(cmd, "AT+CIPSEND=%d\r\n", sms_len);
						yes = true;
						buff = cmd;
						who = 1;
						gprs_stat.next_send = 0;
					} else if (flags.log_show) Report(true, "%s(%u)\r\n", smsGPRS, sms_len);
				}
#endif
				if (gprs_stat.prompt && gprs_stat.connect) {
					yes = true;
#ifdef SET_SMS
					if (!who) buff = msgGPRS;
					else if (who == 1) buff = smsGPRS;
#else
					buff = msgGPRS;
#endif
					gprs_stat.prompt = 0;
					who = -1;
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
					gprs_stat.init = gprs_stat.connect = 0;
					wait_ack = get_tmr(4);
					faza = 4;
					prf = true;
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
														*msgGPRS = '\0';
														if (!makeSMSString(SMS_text, &sms_len, fromNum, sms_num, smsGPRS, sizeof(smsGPRS) - 1)) {
															//
															if (addToSMSQ(smsGPRS, sms_len, &smsq) < 0) {
																if (flags.log_show) Report(true, smsGPRS);
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
										*msgGPRS = '\0';
										if (!makeSMSString(SMS_text, &sms_len, fromNum, sms_num, smsGPRS, sizeof(smsGPRS) - 1)) {

											//
											if (addToSMSQ(smsGPRS, sms_len, &smsq) < 0) {
												if (flags.log_show) Report(true, smsGPRS);
											}
											//
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
							if (ucs2_to_text(uks, (uint8_t *)SMS_text)) Report(false, "%s\r\n", SMS_text);
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
						*msgGPRS = '\0';
						if (!makeSMSString(SMS_text, &sms_len, fromNum, sms_num, smsGPRS, sizeof(smsGPRS) - 1)) {
							//
							if (addToSMSQ(smsGPRS, sms_len, &smsq) < 0) {
								if (flags.log_show) Report(true, smsGPRS);
							}
							//
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
			Report(true, "LOG now is %s\r\n", logStat[flags.log_show]);
		}

		osDelay(1);

	}

	osDelay(100);
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
