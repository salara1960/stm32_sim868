#ifndef __SSD1306_H__
#define __SSD1306_H__

#include "main.h"
#include "font.h"

#if defined(SET_OLED_I2C) || defined(SET_OLED_SPI)

//------------------------------------------------------------------------

// Control byte
#define OLED_CONTROL_BYTE_CMD_SINGLE    0x80
#define OLED_CONTROL_BYTE_CMD_STREAM    0x00
#define OLED_CONTROL_BYTE_DATA_STREAM   0x40

// Fundamental commands (pg.28)
#define OLED_CMD_SET_CONTRAST           0x81    // follow with 0x7F
#define OLED_CMD_DISPLAY_RAM            0xA4
#define OLED_CMD_DISPLAY_ALLON          0xA5
#define OLED_CMD_DISPLAY_NORMAL         0xA6
#define OLED_CMD_DISPLAY_INVERTED       0xA7
#define OLED_CMD_DISPLAY_OFF            0xAE
#define OLED_CMD_DISPLAY_ON             0xAF

// Addressing Command Table (pg.30)
#define OLED_CMD_SET_MEMORY_ADDR_MODE   0x20    // follow with 0x00 = HORZ mode = Behave like a KS108 graphic LCD
#define OLED_CMD_SET_COLUMN_RANGE       0x21    // can be used only in HORZ/VERT mode - follow with 0x00 and 0x7F = COL127
#define OLED_CMD_SET_PAGE_RANGE         0x22    // can be used only in HORZ/VERT mode - follow with 0x00 and 0x07 = PAGE7

// Hardware Config (pg.31)
#define OLED_CMD_SET_DISPLAY_START_LINE 0x40
#define OLED_CMD_SET_SEGMENT_REMAP      0xA1
#define OLED_CMD_SET_MUX_RATIO          0xA8    // follow with 0x3F = 64 MUX
#define OLED_CMD_SET_COM_SCAN_MODE      0xC8
#define OLED_CMD_SET_DISPLAY_OFFSET     0xD3    // follow with 0x00
#define OLED_CMD_SET_COM_PIN_MAP        0xDA    // follow with 0x12
#define OLED_CMD_NOP                    0xE3    // NOP

// Timing and Driving Scheme (pg.32)
#define OLED_CMD_SET_DISPLAY_CLK_DIV    0xD5    // follow with 0x80
#define OLED_CMD_SET_PRECHARGE          0xD9    // follow with 0xF1
#define OLED_CMD_SET_VCOMH_DESELCT      0xDB    // follow with 0x30

// Charge Pump (pg.62)
#define OLED_CMD_SET_CHARGE_PUMP        0x8D    // follow with 0x14

//------------------------------------------------------------------------

#ifdef SET_OLED_I2C
	#define OLED_I2C_ADDRESS                0x3C << 1

	extern void i2c_ssd1306_on(bool flag);
	extern void i2c_ssd1306_init();
	extern void i2c_ssd1306_invert();
	extern void i2c_ssd1306_clear();
	extern void i2c_ssd1306_pattern();
	extern void i2c_ssd1306_contrast(uint8_t value);
	extern void i2c_ssd1306_clear_line(uint8_t cy);
	extern void i2c_ssd1306_text_xy(const char *stroka, uint8_t cx, uint8_t cy);
	extern void i2c_ssd1306_text(const char *stroka);
#endif

#ifdef SET_OLED_SPI
	extern void spi_ssd1306_Reset();
	extern void spi_ssd1306_WriteCmds(uint8_t *cmds, size_t sz);
	extern void spi_ssd1306_WriteData(const char *buf, size_t sz, bool with);
	extern void spi_ssd1306_on(bool flag);
	extern void spi_ssd1306_init();
	extern void spi_ssd1306_invert();
	extern void spi_ssd1306_clear();
	extern void spi_ssd1306_pattern();
	extern void spi_ssd1306_contrast(uint8_t value);
	extern void spi_ssd1306_clear_line(uint8_t cy);
	extern void spi_ssd1306_text_xy(const char *stroka, uint8_t cx, uint8_t cy);
	extern void spi_ssd1306_text(const char *stroka);
#endif

//------------------------------------------------------------------------

#endif

#endif /* __SSD1306_H__ */

