/* USER CODE BEGIN Header */
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
#include "jfes.h"

//#include "stm32f4xx_hal_rtc.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

#define SET_SD_CARD

#define MAX_QMSG 8

#pragma pack(push,1)
typedef struct {
	uint8_t chip;
	float temp;// DegC
	float pres;// mmHg
	float humi;// %rH
	float lux;// lux
} result_t;
typedef struct {
	uint16_t cel;
	uint16_t dro;
} conv_t;
typedef struct {
	uint8_t chip;
	conv_t temp;// DegC
	conv_t pres;// mmHg
	conv_t humi;// %rH
	conv_t lux;// lux
} iresult_t;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct q_msg_t {
	int8_t id;
	char *adr;
} q_msg_t;

typedef struct s_msg_t {
	uint8_t put;
	uint8_t get;
	q_msg_t msg[MAX_QMSG];
} s_msg_t;

typedef struct {
	uint8_t gps_log_show:1;
	uint8_t i2c_log_show:1;
	uint8_t sd_list:1;
	uint8_t sd_mount:1;
	uint8_t sd_umount:1;
	uint8_t restart:1;
	uint8_t stop:1;
	uint8_t none:1;
} s_flags;
#pragma pack(pop)

//$--RMC,hhmmss.sss,x,llll.lll,a,yyyyy.yyy,a,x.x,u.u,xxxxxx,,,v*hh<CR><LF>
//$GNRMC,001805.868,V,,,,,0.00,0.00,060180,,,N*56
#pragma pack(push,1)
typedef struct {
	uint8_t hour;	//hh
	uint8_t min;	//mm
	uint8_t sec;	//ss
	uint16_t ms;	//sss
	bool good;		//x = 'V' - invalid, 'A' - valid
	float latitude;	//llll.lll
	bool ns;		//a - ‘N’ = North; ‘S’ = South
	float longitude;//yyyy.yyy
	bool ew;		//a - ‘E’ = East; ‘W’ = West
	float speed;	//x.x - 0.00
	float dir;		//u.u - 0.00
	uint8_t day;	//xx - 06
	uint8_t mon;	//xx - 01
	uint8_t year;	//xx - 80
	char mode;		//v = ‘N’ = Data not valid, ‘A’ = Autonomous mode, ‘D’ = Differential mode, ‘E’ = Estimated (dead reckoning) mode
	uint8_t crc;
} s_gps_t;

typedef struct {
	uint16_t year;
	uint8_t mon;
	uint8_t day;
	uint8_t hour;
	uint8_t min;
	uint8_t sec;
	uint16_t ms;
} s_utc_t;
//+CGNSINF: 1,0,19800106002148.000,,,,0.00,0.0,0,,,,,,0,0,,,,,
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

/*
typedef enum {
	LOW_SPEED = 9600,
	HIGH_SPEED = 115200
} speed_t;
*/
#ifdef SET_SD_CARD
typedef struct {
	uint16_t day:5;//0..4 (1..31)
	uint16_t mon:4;//5..8 (1..12)
	uint16_t year:7;//9..15 (0..127) from 1980
} fat_date_t;

typedef struct {
	uint16_t sec:5;//0..4 (0..29)
	uint16_t min:6;//5..10 (0..59)
	uint16_t hour:5;//11..15 (0..23)
} fat_time_t;
#endif


/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

HAL_StatusTypeDef i2cError;
const uint32_t min_wait_ms;
const uint32_t max_wait_ms;

I2C_HandleTypeDef *portBMP;
I2C_HandleTypeDef *portSSD;
I2C_HandleTypeDef *portBH;

UART_HandleTypeDef *portLOG;
SPI_HandleTypeDef *portSPI;

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/*
#define BLACK_COLOR   "\x1b[30m"
#define RED_COLOR     "\x1b[31m"
#define GREEN_COLOR   "\x1b[32m"
#define YELLOW_COLOR  "\x1b[33m"
#define BLUE_COLOR    "\x1b[34m"
#define MAGENTA_COLOR "\x1b[35m"
#define CYAN_COLOR    "\x1b[36m"
#define WHITE_COLOR   "\x1b[37m"
*/
#define wait_sensor_def 10
#define MAX_UART_BUF 512//480//400//384//256
#define maxLogSize 65536

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

void Report(bool addTime, const char *fmt, ...);
void errLedOn(const char *from);
void Leds(bool act, uint16_t Pin);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USER_IN_Pin GPIO_PIN_0
#define USER_IN_GPIO_Port GPIOA
#define GSM_STAT_Pin GPIO_PIN_2
#define GSM_STAT_GPIO_Port GPIOA
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
#define SD_CS_Pin GPIO_PIN_5
#define SD_CS_GPIO_Port GPIOB
#define SDA1_Pin GPIO_PIN_7
#define SDA1_GPIO_Port GPIOB
#define SCL1_Pin GPIO_PIN_8
#define SCL1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define SCREEN_SIZE 16*8
#define LED_ERROR LED_RED_Pin
#define GPIO_PortD GPIOD
#define LOOP_FOREVER() while(1) {}

//#ifdef SET_SD_CARD
	#define SS_SD_SELECT() HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET)
	#define SS_SD_DESELECT() HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET)
	#define LD_ON HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_ERROR, GPIO_PIN_SET); //RED
	#define LD_OFF HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_ERROR, GPIO_PIN_RESET); //RED
//#endif


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
