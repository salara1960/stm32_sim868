#ifndef __DEF_H
#define __DEF_H

#undef SET_JFES
#undef SET_RTC_TMR
#undef SET_OLED_I2C

#define SET_OLED_SPI
#define SET_SMS

#undef SET_STATIC_MEM_LOG	//use static/dynamic memmory

#ifdef SET_SMS
	#define SET_SMSQ_STATIC

	//#define SET_CALLOC_MEM
	//#define SET_MALLOC_MEM
#endif

#endif
