/*
 * bh1750.c
 *
 *  Created on: Apr 23, 2019
 *      Author: alarm
 */
#include "bh1750.h"

//-----------------------------------------------------------------------------
void bh1750_off()
{
uint8_t dat = BH1750_POWER_DOWN;

    	HAL_I2C_Master_Transmit(portBH, BH1750_ADDR << 1, &dat, 1, min_wait_ms);
}
//-----------------------------------------------------------------------------
void bh1750_on_mode()
{
uint8_t dat[] = {BH1750_POWER_ON, BH1750_CON_HRES_MODE};

	if ((i2cError = HAL_I2C_Master_Transmit(portBH, BH1750_ADDR << 1, &dat[0], 1, min_wait_ms)) != HAL_OK) errLedOn(__func__);
	else
	if ((i2cError = HAL_I2C_Master_Transmit(portBH, BH1750_ADDR << 1, &dat[1], 1, min_wait_ms)) != HAL_OK) errLedOn(__func__);
}
//-----------------------------------------------------------------------------
HAL_StatusTypeDef bh1750_proc(uint16_t *lux)
{
	if (!i2cError) {
		uint8_t dat[] = {0,0};
		i2cError = HAL_I2C_Master_Receive(portBH, BH1750_ADDR << 1, dat, 2, min_wait_ms);
		if (i2cError == HAL_OK) {
			uint16_t lx = dat[0];
			lx <<= 8;
			lx |= dat[1];
			*lux = lx;
		} else errLedOn(__func__);
	}

    return i2cError;
}
//-----------------------------------------------------------------------------




