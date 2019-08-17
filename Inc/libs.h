#ifndef __LIBS_H__
#define __LIBS_H__

#include "main.h"
#if defined(SET_OLED_I2C) || defined(SET_OLED_SPI)
	#include "ssd1306.h"
#endif
#include "bmp280.h"

//--------------------------------------------------------------------------

#define max_rssi 32

//--------------------------------------------------------------------------

extern const int8_t dBmRSSI[max_rssi];

//--------------------------------------------------------------------------

extern uint32_t get_secCounter();
extern void inc_secCounter();
extern uint64_t get_hsCounter();
extern void inc_hsCounter();

extern void ClearMyTicks();
extern void *getMem(size_t sz);
extern void freeMem(void *ptr);
extern bool getVIO();
extern void gsmONOFF(const uint32_t twait);
extern void Leds(bool act, uint16_t Pin);
extern void Report(bool addTime, const char *fmt, ...);
extern void errLedOn(const char *from);
extern int sec_to_string(uint32_t sec, char *stx, bool log);

#ifdef SET_RTC_TMR
	extern void set_Date(uint32_t epoch);
	extern uint32_t getSecRTC(RTC_HandleTypeDef *hrtc);
#else
	extern volatile uint32_t extDate;
	extern uint32_t get_extDate();
	extern void inc_extDate();
	extern void set_extDate(uint32_t ep);
#endif

extern uint32_t get_tmr(uint32_t sec);
extern bool check_tmr(uint32_t sec);
extern uint64_t get_hstmr(uint64_t hs);
extern bool check_hstmr(uint64_t hs);

#if defined(SET_OLED_I2C) || defined(SET_OLED_SPI)
	extern uint8_t ssd1306_calcx(int len);
	extern void toDisplay(const char *st, uint8_t column, uint8_t line, bool clear);
#endif

extern int8_t parse_inf(char *in, s_inf_t *inf);
extern int8_t makeInfString(const s_data_t *data, char *buf, int max_len_buf);
extern void getAdrPort(char *uk);
extern void initRECQ(s_recq_t *q);
extern void clearRECQ(s_recq_t *q);
extern int8_t putRECQ(char *adr, s_recq_t *q);
extern int8_t getRECQ(char *dat, s_recq_t *q);
extern int8_t addRECQ(char *txt, s_recq_t *q);
extern void initQ(s_msg_t *q);
extern void clearQ(s_msg_t *q);
extern int8_t putQ(char *adr, s_msg_t *q);
extern int8_t getQ(char *dat, s_msg_t *q);
extern void AtParamInit(bool how);

//--------------------------------------------------------------------------

#endif /* __LIBS_H__ */

