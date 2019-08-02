/* USER CODE BEGIN Header */
//char *buff = (char *)pvPortMalloc(len); vPortFree(buff);
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <stdarg.h>
#include "def.h"

#ifdef SET_JFES
	#include "jfes.h"
#endif
//#include "stm32f4xx_hal_rtc.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */


#undef SET_STATIC_MEM_LOG	//use dynamic memmory


#define MAX_QMSG 8


#pragma pack(push,1)
typedef struct {
	uint8_t chip;
	float temp; // DegC
	float pres; // mmHg
	float humi; // %rH
	float lux;  // lux
} result_t;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct q_msg_t {
	int8_t id;
	char *adr;
} q_msg_t;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct s_msg_t {
	uint8_t put;
	uint8_t get;
	q_msg_t msg[MAX_QMSG];
} s_msg_t;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct {
	unsigned init:1;
	unsigned connect:1;
	unsigned try_connect:1;
	unsigned try_disconnect:1;
	unsigned prompt:1;
	unsigned try_send:1;
	unsigned cgatt_on:1;
	unsigned send_ok:1;
	unsigned next_send:1;
	unsigned unused:7;
} s_gprs_stat;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct {
	unsigned log_flag:1;
	unsigned log_show:1;
	unsigned csq:1;
	unsigned restart:1;
	unsigned stop:1;
	unsigned imei_flag:1;
	unsigned local_ip_flag:1;
	unsigned connect:1;
	unsigned disconnect:1;
	unsigned srv:1;
	unsigned msg_begin:1;
	unsigned msg_end:1;
	unsigned auto_cmd:1;
	unsigned inf:1;
	unsigned vio:1;
	unsigned sms:1;
} s_flags;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct {
	uint16_t year;
	uint8_t mon;
	uint8_t day;
	uint8_t hour;
	uint8_t min;
	uint8_t sec;
	uint16_t ms;
} s_utc_t;
#pragma pack(pop)

//+CGNSINF: 1,0,19800106002148.000,,,,0.00,0.0,0,,,,,,0,0,,,,,
#pragma pack(push,1)
typedef struct {
	uint8_t run;		//1
	uint8_t status;		//2
	s_utc_t utc;		//3
	float latitude;		//4
	float longitude;	//5
	int16_t altitude;	//6
	float speed;		//7
	float dir;			//8
	uint8_t mode;		//9
	uint8_t none1;		//10
	float HDOP;			//11
	float PDOP;			//12
	float VDOP;			//13
	uint8_t none2;		//14
	uint8_t GPSsatV;	//15
	uint8_t GNSSsatU;	//16
	uint8_t GLONASSsatV;//17
	uint8_t none3;		//18
	uint8_t dBHz;		//19
	float HPA;			//20
	float VPA;			//21
} s_inf_t;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct {
	result_t sens;
	s_inf_t inf;
} s_data_t;//allData;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct {
	uint8_t rssi;
} s_gsm_stat;
#pragma pack(pop)

//-------------------------------------------------------------------
#ifdef SET_SMS

#define SET_SMSQ_STATIC
//#define SET_CALLOC_MEM
//#define SET_MALLOC_MEM

#define MAX_QSMS 4
#define maxSMSPart 8
#define MaxBodyLen 113
#define SMS_BUF_LEN 560

#define cod_PDU_len 159 //137
#define lenFrom 32
#define wait_sms_time 60 * 3
#define max_smsType 3

#pragma pack(push,1)
typedef struct s_udhi_t {
	uint8_t tp;//0-обычная смс, 1-часть длинной смс, 255-квитанция
	uint16_t num;//индекс (номер) смс
	uint8_t total;//количество частей
	uint8_t part;//номер части
	uint16_t len;
	char txt[MaxBodyLen];//113//[MaxBodyLen];//440 bytes
} s_udhi_t;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct q_rec_t {
	int8_t id;
	#ifdef SET_SMSQ_STATIC
		char adr[SMS_BUF_LEN];
	#else
		char *adr;
	#endif
} q_rec_t;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct s_smsq_t {
	uint8_t put;
	uint8_t get;
	q_rec_t sms[MAX_QSMS];
} s_smsq_t;
#pragma pack(pop)

#endif
//-------------------------------------------------------------------

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

extern const uint32_t min_wait_ms;
extern const uint32_t max_wait_ms;
extern HAL_StatusTypeDef i2cError;
extern I2C_HandleTypeDef *portBMP;
#ifdef SET_OLED_I2C
	extern I2C_HandleTypeDef *portSSD;
#endif
extern I2C_HandleTypeDef *portBH;
extern UART_HandleTypeDef *portLOG;
extern SPI_HandleTypeDef *portSPI;
#ifdef SET_OLED_SPI
	extern SPI_HandleTypeDef *portOLED;
#endif

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */


#define wait_sensor_def 32
#define wait_gps_def wait_sensor_def - 2 //>> 1
#define wait_ack_cli_sec 10
#define size_imei 15
#define wait_csq_def wait_gps_def - 2
#define max_rssi 32

#ifdef SET_JFES
	#define MAX_UART_BUF 640//704//640//512//480//400//384//256
#else
	#define MAX_UART_BUF 600//512//480//400//384//256
#endif


/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
extern void Error_Handler(void);

/* USER CODE BEGIN EFP */

extern void Report(bool addTime, const char *fmt, ...);
extern void errLedOn(const char *from);
extern void Leds(bool act, uint16_t Pin);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USER_IN_Pin GPIO_PIN_0
#define USER_IN_GPIO_Port GPIOA
#define GSM_STATUS_Pin GPIO_PIN_2
#define GSM_STATUS_GPIO_Port GPIOA
#define GSM_KEY_Pin GPIO_PIN_3
#define GSM_KEY_GPIO_Port GPIOA
#define TX3_Pin GPIO_PIN_10
#define TX3_GPIO_Port GPIOB
#define RX3_Pin GPIO_PIN_11
#define RX3_GPIO_Port GPIOB
#define LED_GREEN_Pin GPIO_PIN_12
#define LED_GREEN_GPIO_Port GPIOD
#define LED_ORANGE_Pin GPIO_PIN_13
#define LED_ORANGE_GPIO_Port GPIOD
#define LED_RED_Pin GPIO_PIN_14
#define LED_RED_GPIO_Port GPIOD
#define LED_BLUE_Pin GPIO_PIN_15
#define LED_BLUE_GPIO_Port GPIOD
#define OLED_RST_Pin GPIO_PIN_11
#define OLED_RST_GPIO_Port GPIOC
#define OLED_MOSI_Pin GPIO_PIN_12
#define OLED_MOSI_GPIO_Port GPIOC
#define OLED_SCK_Pin GPIO_PIN_3
#define OLED_SCK_GPIO_Port GPIOB
#define OLED_CS_Pin GPIO_PIN_4
#define OLED_CS_GPIO_Port GPIOB
#define OLED_DC_Pin GPIO_PIN_6
#define OLED_DC_GPIO_Port GPIOB
#define SDA1_Pin GPIO_PIN_7
#define SDA1_GPIO_Port GPIOB
#define SCL1_Pin GPIO_PIN_8
#define SCL1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define SCREEN_SIZE 16*8
#define LED_ERROR LED_RED_Pin
#define GPIO_PortD GPIOD

#define LOOP_FOREVER() while(1) {}

#define _250ms 1
#define _500ms 2
#define _750ms 3
#define _1s (_250ms * 5)
#define _1_250s (_250ms * 5)
#define _1_500s (_250ms * 6)
#define _1_750s (_250ms * 7)
#define _2s (_250ms * 8)


#ifdef SET_OLED_SPI
	#define CS_OLED_SELECT() HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_RESET)
	#define CS_OLED_DESELECT() HAL_GPIO_WritePin(OLED_CS_GPIO_Port, OLED_CS_Pin, GPIO_PIN_SET)
#endif


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
