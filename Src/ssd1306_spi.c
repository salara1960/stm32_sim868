

#include "ssd1306.h"

#ifdef SET_OLED_SPI

//------------------------------------------------------------------------

uint8_t spi_invert = OLED_CMD_DISPLAY_NORMAL;

//******************************************************************************************

void spi_ssd1306_Reset() {

	CS_OLED_DESELECT();
	HAL_GPIO_WritePin(OLED_RST_GPIO_Port, OLED_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(OLED_RST_GPIO_Port, OLED_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
}
//-----------------------------------------------------------------------------------------
void spi_ssd1306_WriteCmds(uint8_t *cmds, size_t sz)
{
	CS_OLED_SELECT();

    HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_RESET);

    HAL_SPI_Transmit(portOLED, cmds, sz, HAL_MAX_DELAY);

    CS_OLED_DESELECT();
}
//-----------------------------------------------------------------------------------------
// Send data
void spi_ssd1306_WriteData(const char *buf, size_t sz, bool with)
{
    CS_OLED_SELECT();

    HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_SET);

    if (with) {
    	uint16_t cnt = 0;
    	HAL_SPI_Transmit_DMA(portOLED, (uint8_t *)buf, sz);
    	while (HAL_SPI_GetState(portOLED) != HAL_SPI_STATE_READY) {
    		HAL_Delay(1);
    		cnt++;
    		if (!cnt) break;
    	}
    } else {
    	HAL_SPI_Transmit(portOLED, (uint8_t *)buf, sz, HAL_MAX_DELAY);
    }

    CS_OLED_DESELECT();
}
//-----------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------
void spi_ssd1306_on(bool flag)
{
uint8_t dat[] = {OLED_CONTROL_BYTE_CMD_SINGLE, 0};

    if (flag) dat[1] = OLED_CMD_DISPLAY_ON;
    	 else dat[1] = OLED_CMD_DISPLAY_OFF;

    spi_ssd1306_WriteCmds(dat, sizeof(dat));
}
//-----------------------------------------------------------------------------------------
void spi_ssd1306_init()
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
	spi_invert
};

	spi_ssd1306_WriteCmds(dat, sizeof(dat));
}
//-----------------------------------------------------------------------------------------
void spi_ssd1306_invert()
{
uint8_t dat[] = {OLED_CONTROL_BYTE_CMD_SINGLE, 0};

    if (spi_invert == OLED_CMD_DISPLAY_INVERTED) spi_invert = OLED_CMD_DISPLAY_NORMAL;
											else spi_invert = OLED_CMD_DISPLAY_INVERTED;
    dat[1] = spi_invert;

    spi_ssd1306_WriteCmds(dat, sizeof(dat));
}
//-----------------------------------------------------------------------------------------
void spi_ssd1306_clear()
{
uint8_t i, dat[] = {OLED_CONTROL_BYTE_CMD_SINGLE, 0}, zero[129] = {0};

    zero[0] = OLED_CONTROL_BYTE_DATA_STREAM;

    for (i = 0; i < 8; i++) {
    	dat[1] = 0xB0 | i;
    	spi_ssd1306_WriteCmds(dat, sizeof(dat));
    	spi_ssd1306_WriteData((const char *)zero, sizeof(zero), 0);
    }

}
//-----------------------------------------------------------------------------------------
void spi_ssd1306_pattern()
{
uint8_t i, dat[] = {OLED_CONTROL_BYTE_CMD_SINGLE, 0};
uint8_t buf[129];

    buf[0] = OLED_CONTROL_BYTE_DATA_STREAM;
    for (i = 1; i < 129; i++) buf[i] = 0xFF >> (i % 8);
    for (i = 0; i < 8; i++) {
    	dat[1] = 0xB0 | i;
    	spi_ssd1306_WriteCmds(dat, sizeof(dat));
    	spi_ssd1306_WriteData((const char *)buf, sizeof(buf), 0);
    }

}
//-----------------------------------------------------------------------------------------
void spi_ssd1306_contrast(uint8_t value)//0xff or 0x00
{
uint8_t dat[] = {OLED_CONTROL_BYTE_CMD_STREAM, OLED_CMD_SET_CONTRAST, value};

	spi_ssd1306_WriteCmds(dat, sizeof(dat));
}
//-----------------------------------------------------------------------------------------
void spi_ssd1306_clear_line(uint8_t cy)
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

	spi_ssd1306_WriteCmds(first, sizeof(first));
	for (uint8_t i = 0; i < 16; i++) {
		spi_ssd1306_WriteCmds(cif_zero, 1);
		spi_ssd1306_WriteData((const char *)&cif_zero[1], sizeof(cif_zero) - 1, 0);
	}

}
//-----------------------------------------------------------------------------------------
void spi_ssd1306_text_xy(const char *stroka, uint8_t cx, uint8_t cy)
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

	spi_ssd1306_WriteCmds(first, sizeof(first));
	for (i = 0; i < len; i++) {
		if (stroka[i] == '\n') {
			dat[3] = 0xB0 | ++lin;
			spi_ssd1306_WriteCmds(dat, sizeof(dat));
		} else {
			memcpy(&cif[1], &font8x8[(uint8_t)stroka[i]][0], 8);
			spi_ssd1306_WriteCmds(cif, 1);
			spi_ssd1306_WriteData((const char *)&cif[1], sizeof(cif) - 1, 0);
		}
	}


}
//-----------------------------------------------------------------------------------------
void spi_ssd1306_text(const char *stroka)
{
	if (stroka) {
		//if (!i2cError) {
			spi_ssd1306_text_xy(stroka, 1, 1);
		//} else {
		//	errLedOn(__func__);
		//}
	}
}
//******************************************************************************************

#endif

