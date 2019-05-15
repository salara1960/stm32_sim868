/*
 * bh1750.h
 *
 *  Created on: Apr 23, 2019
 *      Author: alarm
 */

#ifndef BH1750_H_
#define BH1750_H_

#include "main.h"

/*  GY-302 (BH1750)  */
#define BH1750_ADDR            0x23 //!< slave address for BH1750 sensor (0x23 addr=L, 0x5C addr=H)
#define BH1750_POWER_DOWN      0x00
#define BH1750_POWER_ON        0x01
#define BH1750_RESET_VALUE     0x07
#define BH1750_CON_HRES_MODE   0x10 //120ms 1lx resolution
#define BH1750_CON_HRES_MODE2  0x11 //120ms 0.5lx resolution
#define BH1750_CON_LRES_MODE   0x13 //16ms 4lx resolution
#define BH1750_OT_HRES_MODE    0x20 //120ms 1lx resolution
#define BH1750_OT_HRES_MODE2   0x21 //120ms 0.5lx resolution
#define BH1750_OT_LRES_MODE    0x23 //16ms 4lx resolution

void bh1750_off();
void bh1750_on_mode();
HAL_StatusTypeDef bh1750_proc(uint16_t *lux);

#endif /* BH1750_H_ */
