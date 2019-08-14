#include "libs.h"

//------------------------------------------------------------------------------------

volatile static uint32_t secCounter = 0;//period 1s
volatile static uint64_t HalfSecCounter = 0;//period 250ms
#ifndef SET_RTC_TMR
	volatile uint32_t extDate = 0;
#endif

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

const char *nameValid[] = {"Invaid", "Valid"};
//const char *nameNS[] = {"North" , "South"};
//const char *nameEW[] = {"East" , "West"};

//------------------------------------------------------------------------------------

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
//------------------------------------------------------------------------------------
void ClearMyTicks()
{
	secCounter     = 0; // 1 sec counter (uint32_t)
	HalfSecCounter = 0; // 250 ms counter (uint64_t)
#ifndef SET_RTC_TMR
	extDate        = 0; // 1 sec counter (uint32_t)
#endif
}
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
//------------------------------------------------------------------------------------
void freeMem(void *mem)
{
#if defined(SET_CALLOC_MEM) || defined(SET_MALLOC_MEM)
		free(mem);
#else
		vPortFree(mem);
#endif
}
//------------------------------------------------------------------------------------------
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
//------------------------------------------------------------------------------------------
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
		//
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
/*
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
*/
//------------------------------------------------------------------------------------------
bool getVIO()
{
	return ((bool)HAL_GPIO_ReadPin(GSM_STATUS_GPIO_Port, GSM_STATUS_Pin));
}
//------------------------------------------------------------------------------------------
void gsmONOFF(const uint32_t twait)
{
bool wv, vio = getVIO();
int one = 50;
int cnt = twait / one;
uint32_t start_tik = HAL_GetTick();

	flags.log_show   = 0;
	onGNS = false;

	if (twait <= ModuleON) {
		wv = true;
		if (vio) goto done;
	} else {
		wv = false;
		if (!vio) goto done;
	}
	Report(true, "[%s] GSM_KEY set to 0 (vio=%u)\r\n", strOnOff[wv], getVIO());
	HAL_GPIO_WritePin(GSM_KEY_GPIO_Port, GSM_KEY_Pin, GPIO_PIN_RESET);//set 0
	start_tik = HAL_GetTick();

	while (1) {
		vio = getVIO();
		if (wv) {//wait ON - getVIO() = true
			if (vio) break;
		} else {//wait OFF - getVIO() = false
			if (!vio) break;
		}
		HAL_Delay(one);
		cnt--;   if (cnt <= 0) break;
	}

	HAL_GPIO_WritePin(GSM_KEY_GPIO_Port, GSM_KEY_Pin, GPIO_PIN_SET);//set 1

done:

	Report(true, "[%s] VIO now is %u (wait %u ms)\r\n", strOnOff[wv], getVIO(), HAL_GetTick() - start_tik);

	ackYes = 0;
}
//-----------------------------------------------------------------------------------------
#if defined(SET_OLED_I2C) || defined(SET_OLED_SPI)
uint8_t ssd1306_calcx(int len)
{
uint8_t ret = 0;

    if ( (len > 0) && (len <= 16) ) ret = ((16 - len) >> 1) + 1;

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

#endif /* defined(SET_OLED_I2C) || defined(SET_OLED_SPI) */

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
        strcat(buf, "\r\n");

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

	strcat(buf, "}\r\n");

	ret = 0;

#endif

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
//-----------------------------------------------------------------------------------------
void initRECQ(s_recq_t *q)//s_recq_t recq;
{
	if (osSemaphoreAcquire(msgSem, 500) == osOK) {
		q->put = q->get = 0;
		for (uint8_t i = 0; i < MAX_QREC; i++) {
			q->rec[i].id = i;
#ifdef SET_RECQ_STATIC
			q->rec[i].adr[0] = '\0';
#else
			q->rec[i].adr = NULL;
#endif
		}
		osSemaphoreRelease(msgSem);
	}
}
//-----------------------------------------------------------------------------
void clearRECQ(s_recq_t *q)
{
	if (osSemaphoreAcquire(msgSem, 500) == osOK) {
		q->put = q->get = 0;
		for (uint8_t i = 0; i < MAX_QREC; i++) {
			q->rec[i].id = i;
#ifdef SET_RECQ_STATIC
			q->rec[i].adr[0] = '\0';
#else
			freeMem(q->rec[i].adr);
			q->rec[i].adr = NULL;
#endif
		}
		osSemaphoreRelease(msgSem);
	}
}
//-----------------------------------------------------------------------------
int8_t putRECQ(char *adr, s_recq_t *q)
{
int8_t ret = -1;

	if (osSemaphoreAcquire(msgSem, 500) == osOK) {

#ifdef SET_RECQ_STATIC
		if (!strlen(q->rec[q->put].adr)) {
			int len = strlen(adr);
			if (len >= REC_BUF_LEN) len = REC_BUF_LEN - 1;
			memcpy((char *)&q->rec[q->put].adr[0], adr, len);
			q->rec[q->put].adr[len] = '\0';
#else
		if (q->rec[q->put].adr == NULL) {
			q->rec[q->put].adr = adr;
#endif
			ret = q->rec[q->put].id;
			q->put++;   if (q->put >= MAX_QREC) q->put = 0;
		}

		osSemaphoreRelease(msgSem);
	}

	return ret;
}
//-----------------------------------------------------------------------------
int8_t getRECQ(char *dat, s_recq_t *q)
{
int8_t ret = -1;
int len = 0;

	if (osSemaphoreAcquire(msgSem, 500) == osOK) {

#ifdef SET_RECQ_STATIC
		len = strlen(q->rec[q->get].adr);
		if (len) {
			ret = q->rec[q->get].id;
			memcpy(dat, q->rec[q->get].adr, len);
			q->rec[q->get].adr[0] = '\0';
		}
#else
		if (q->rec[q->get].adr != NULL) {
			len = strlen(q->rec[q->get].adr);
			ret = q->rec[q->get].id;
			memcpy(dat, q->rec[q->get].adr, len);
			freeMem(q->rec[q->get].adr);
			q->rec[q->get].adr = NULL;
		}
#endif

		if (ret >= 0) {
			*(dat + len) = '\0';
			q->get++;   if (q->get >= MAX_QREC) q->get = 0;
		}

		osSemaphoreRelease(msgSem);

	}

	return ret;
}
//------------------------------------------------------------------------------------------
int8_t addRECQ(char *txt, s_recq_t *q)
{
int8_t nrec = -1;
uint16_t txt_len = strlen(txt);

#ifdef SET_RECQ_STATIC
	if (txt_len >= REC_BUF_LEN) {
		txt_len = REC_BUF_LEN - 1;
		txt[txt_len] = '\0';
	}
	if ((nrec = putRECQ(txt, q)) >= 0) {
		Report(true, "[%s] : put record to queue OK (id=%d len=%d)\r\n", __func__, nrec, txt_len);
	} else {
		Report(true, "[%s] : put record to queue error (len=%d)\r\n", __func__, txt_len);
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
			if ((nrec = putRECQ(rc, q)) >= 0) {
				Report(true, "[%s] : put record to queue OK (id=%d len=%d/%d)\r\n", __func__, nrec, need_len, got_len);
		    } else {
				Report(true, "[%s] : put record to queue error (len=%d/%d)\r\n", __func__, need_len, got_len);
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


//******************************************************************************************
