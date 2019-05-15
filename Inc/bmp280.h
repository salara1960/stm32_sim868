#ifndef __BMP280_H__
#define __BMP280_H__

#include "main.h"

/*  BMP280/BME280 registers  */
#define BMP280_ADDR            0x76 //!< slave address for BMP280 sensor
#define BMP280_SENSOR          0x58
#define BME280_SENSOR          0x60
#define BMP280_REG_TEMP_XLSB   0xFC // bits: 7-4
#define BMP280_REG_TEMP_LSB    0xFB
#define BMP280_REG_TEMP_MSB    0xFA
#define BMP280_REG_TEMP        (BMP280_REG_TEMP_MSB)
#define BMP280_REG_PRESS_XLSB  0xF9 // bits: 7-4
#define BMP280_REG_PRESS_LSB   0xF8
#define BMP280_REG_PRESS_MSB   0xF7
#define BMP280_REG_PRESSURE    (BMP280_REG_PRESS_MSB)
#define BMP280_REG_CONFIG      0xF5 // bits: 7-5 t_sb; 4-2 filter; 0 spi3w_en
#define BMP280_REG_CTRL        0xF4 // bits: 7-5 osrs_t; 4-2 osrs_p; 1-0 mode
#define BMP280_REG_STATUS      0xF3 // bits: 3 measuring; 0 im_update
#define BME280_REG_CTRL_HUM    0xF2 // bits: 2-0 osrs_h;
#define BMP280_REG_RESET       0xE0
#define BMP280_REG_ID          0xD0
#define BMP280_REG_CALIB       0x88
#define BMP280_NORMAL_MODE     0x3
#define BMP280_FORCED1_MODE    0x1
#define BMP280_FORCED2_MODE    0x2
#define BMP280_OSRS_T          0x20
#define BMP280_OSRS_P          0x04
#define BMP280_CONF_T_SB       0x40
#define BMP280_CONF_FILTER     0x00
#define BMP280_CONF_SPI3W      0x00
#define BME280_OSRS_H          0x01
#define BMP280_RESET_VALUE     0xB6

#define DATA_LENGTH            8//256        //!<Data buffer length for test buffer

typedef struct bmp280_calib_t {
	uint16_t dig_T1;
	int16_t  dig_T2;
	int16_t  dig_T3;
	uint16_t dig_P1;
	int16_t  dig_P2;
	int16_t  dig_P3;
	int16_t  dig_P4;
	int16_t  dig_P5;
	int16_t  dig_P6;
	int16_t  dig_P7;
	int16_t  dig_P8;
	int16_t  dig_P9;
	int8_t   dig_H1;
	int16_t  dig_H2;
	int8_t   dig_H3;
	int16_t  dig_H4;
	int16_t  dig_H5;
	int8_t   dig_H6;
} bmx280_calib_t;


HAL_StatusTypeDef i2c_master_read_sensor(uint8_t reg, uint8_t *data_rd, size_t size);
HAL_StatusTypeDef i2c_master_reset_sensor(uint8_t *chip_id);
HAL_StatusTypeDef i2c_master_test_sensor(uint8_t *stat, uint8_t *mode, uint8_t *conf, uint8_t chip_id);
HAL_StatusTypeDef bmp280_readCalibrationData(uint8_t chip_id);
void bmp280_CalcAll(result_t *ssen, int32_t chip_id, int32_t tp, int32_t pp, int32_t hh);

#endif /* __BMP280_H__ */

