#include "ssd1306.h"

#ifdef SET_OLED_I2C
//------------------------------------------------------------------------

uint8_t invert = OLED_CMD_DISPLAY_NORMAL;

const uint32_t _mswait = 10;

//******************************************************************************************

//-----------------------------------------------------------------------------------------

void i2c_ssd1306_on(bool flag)
{
uint8_t dat[] = {OLED_CONTROL_BYTE_CMD_SINGLE, 0};

    if (flag) dat[1] = OLED_CMD_DISPLAY_ON;
    	 else dat[1] = OLED_CMD_DISPLAY_OFF;

    HAL_I2C_Master_Transmit(portSSD, OLED_I2C_ADDRESS, dat, sizeof(dat), min_wait_ms);
}
//-----------------------------------------------------------------------------------------
void i2c_ssd1306_init()
{
uint8_t dat[] = {
	OLED_CONTROL_BYTE_CMD_STREAM,//0x00
	OLED_CMD_SET_CHARGE_PUMP,    //0x8D
	0x14,
	OLED_CMD_SET_SEGMENT_REMAP,  //0xA1
	OLED_CMD_SET_COM_SCAN_MODE,  //0xC8
	OLED_CMD_SET_COLUMN_RANGE,   //0x21
	0x00,
	0x7F,
	OLED_CMD_SET_PAGE_RANGE,     //0x22
	0x00,
	0x07,
	OLED_CMD_DISPLAY_ON,         //0xAF
	invert
};

	HAL_I2C_Master_Transmit(portSSD, OLED_I2C_ADDRESS, dat, sizeof(dat), min_wait_ms);
}
//-----------------------------------------------------------------------------------------
void i2c_ssd1306_invert()
{
uint8_t dat[] = {OLED_CONTROL_BYTE_CMD_SINGLE, 0};

    if (invert == OLED_CMD_DISPLAY_INVERTED) invert = OLED_CMD_DISPLAY_NORMAL;
										else invert = OLED_CMD_DISPLAY_INVERTED;
    dat[1] = invert;

    HAL_I2C_Master_Transmit(portSSD, OLED_I2C_ADDRESS, dat, sizeof(dat), min_wait_ms);
}
//-----------------------------------------------------------------------------------------
void i2c_ssd1306_clear()
{
uint8_t i, dat[] = {OLED_CONTROL_BYTE_CMD_SINGLE, 0}, zero[129] = {0};

    zero[0] = OLED_CONTROL_BYTE_DATA_STREAM;

    for (i = 0; i < 8; i++) {
    	dat[1] = 0xB0 | i;
    	HAL_I2C_Master_Transmit(portSSD, OLED_I2C_ADDRESS, dat,    2, min_wait_ms);
    	HAL_I2C_Master_Transmit(portSSD, OLED_I2C_ADDRESS, zero, 129, max_wait_ms);
    }

}
//-----------------------------------------------------------------------------------------
void i2c_ssd1306_pattern()
{
uint8_t i, dat[] = {OLED_CONTROL_BYTE_CMD_SINGLE, 0};
uint8_t buf[129];

    buf[0] = OLED_CONTROL_BYTE_DATA_STREAM;
    for (i = 1; i < 129; i++) buf[i] = 0xFF >> (i % 8);
    for (i = 0; i < 8; i++) {
    	dat[1] = 0xB0 | i;
    	HAL_I2C_Master_Transmit(portSSD, OLED_I2C_ADDRESS, dat,   2, min_wait_ms);
    	HAL_I2C_Master_Transmit(portSSD, OLED_I2C_ADDRESS, buf, 129, max_wait_ms);
    }

}
//-----------------------------------------------------------------------------------------
void i2c_ssd1306_contrast(uint8_t value)//0xff or 0x00
{
uint8_t dat[] = {OLED_CONTROL_BYTE_CMD_STREAM, OLED_CMD_SET_CONTRAST, value};

	HAL_I2C_Master_Transmit(portSSD, OLED_I2C_ADDRESS, dat, sizeof(dat), min_wait_ms);
}
//-----------------------------------------------------------------------------------------
void i2c_ssd1306_clear_line(uint8_t cy)
{
uint8_t cif_zero[] = {OLED_CONTROL_BYTE_DATA_STREAM, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t first[] = {
	OLED_CONTROL_BYTE_CMD_STREAM,
	OLED_CMD_SET_COLUMN_RANGE,
	0,
	0x7f,
	OLED_CMD_SET_PAGE_RANGE,
	cy - 1,
	7
};

	if (HAL_I2C_Master_Transmit(portSSD, OLED_I2C_ADDRESS, first, sizeof(first), min_wait_ms) == HAL_OK) {
		for (uint8_t i = 0; i < 16; i++) {
			if (HAL_I2C_Master_Transmit(portSSD, OLED_I2C_ADDRESS, cif_zero, sizeof(cif_zero), min_wait_ms) != HAL_OK) break;
		}
	}

}
//-----------------------------------------------------------------------------------------
void i2c_ssd1306_text_xy(const char *stroka, uint8_t cx, uint8_t cy)
{
uint8_t i, lin = cy - 1, col = cx - 1;
int len = strlen(stroka);
uint8_t dat[] = {OLED_CONTROL_BYTE_CMD_STREAM, 0, 0x10, 0};
uint8_t cif[] = {OLED_CONTROL_BYTE_DATA_STREAM, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t first[] = {
	OLED_CONTROL_BYTE_CMD_STREAM,
	OLED_CMD_SET_COLUMN_RANGE,
	col << 3,
	0x7f,
	OLED_CMD_SET_PAGE_RANGE,
	lin,
	7
};

	if (HAL_I2C_Master_Transmit(portSSD, OLED_I2C_ADDRESS, first, sizeof(first), min_wait_ms) == HAL_OK) {
		for (i = 0; i < len; i++) {
			if (stroka[i] == '\n') {
				dat[3] = 0xB0 | ++lin;
				HAL_I2C_Master_Transmit(portSSD, OLED_I2C_ADDRESS, dat, sizeof(dat), min_wait_ms);
			} else {
				memcpy(&cif[1], &font8x8[(uint8_t)stroka[i]][0], 8);
				HAL_I2C_Master_Transmit(portSSD, OLED_I2C_ADDRESS, cif, sizeof(cif), min_wait_ms);
			}
		}
	}

}
//-----------------------------------------------------------------------------------------
void i2c_ssd1306_text(const char *stroka)
{
	if (stroka) {
		if (!i2cError) {
			ssd1306_text_xy(stroka, 1, 1);
		} else {
			errLedOn(__func__);
		}
	}
}
//******************************************************************************************
#endif


