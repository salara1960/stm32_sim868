#include "sd.h"

//--------------------------------------------------

sd_info_ptr sdinfo;
uint32_t byteswritten, bytesread;

//-----------------------------------------------
uint8_t SPIx_WriteRead(uint8_t Byte)
{
uint8_t rxbyte = 0;

	if (HAL_SPI_TransmitReceive(portSPI, &Byte, &rxbyte, 1, 500) != HAL_OK) Leds(true, LED_ERROR); //errLedOn(__func__);

	return rxbyte;
}
//-----------------------------------------------
void SPI_SendByte(uint8_t bt)
{
  SPIx_WriteRead(bt);
}
//-----------------------------------------------
uint8_t SPI_ReceiveByte(void)
{
  return (SPIx_WriteRead(0xFF));
}
//-----------------------------------------------
void SPI_Release(void)
{
  SPIx_WriteRead(0xFF);
}
//-----------------------------------------------
uint8_t SPI_wait_ready(void)
{
uint8_t res;
uint16_t cnt = 0;

	do {
		res = SPI_ReceiveByte();
		cnt++;
	} while ( (res != 0xFF) && (cnt < 0xFFFF) );

	if (cnt >= 0xFFFF) res = 1;

	return res;
}
//-----------------------------------------------
static uint8_t SD_cmd(uint8_t cmd, uint32_t arg)
{
uint8_t n = 1, res;

	// ACMD<n> is the command sequense of CMD55-CMD<n>
	if (cmd & 0x80) {
		cmd &= 0x7F;
		res = SD_cmd(CMD55, 0);
		if (res > 1) goto done;//return res;
	}
	// Select the card
	SS_SD_DESELECT();
	SPI_ReceiveByte();
	SS_SD_SELECT();
	SPI_ReceiveByte();

	// Send a command packet
	SPI_SendByte(cmd); // Start + Command index
	SPI_SendByte((uint8_t)(arg >> 24)); // Argument[31..24]
	SPI_SendByte((uint8_t)(arg >> 16)); // Argument[23..16]
	SPI_SendByte((uint8_t)(arg >> 8)); // Argument[15..8]
	SPI_SendByte((uint8_t)arg); // Argument[7..0]
	//n = 1; // Dummy CRC + Stop
	if (cmd == CMD0) n = 0x95; // Valid CRC for CMD0(0)
	else
	if (cmd == CMD8) n = 0x87; // Valid CRC for CMD8(0x1AA)
	SPI_SendByte(n);	
	// Receive a command response
	n = 10;//10; // Wait for a valid response in timeout of 10 attempts
	do {
		res = SPI_ReceiveByte();
		HAL_Delay(1);
	} while ((res & 0x80) && --n);

done:

//	Report(false, "\t[%s] cmd=0x%02X arg=%u res=0x%02X\r\n", __func__, cmd, arg, res);

  return res;
}
//-----------------------------------------------
void SD_PowerOn(void)
{
	//Timer1 = 0;
	//while(Timer1<2);
	HAL_Delay(20);
}
//-----------------------------------------------
uint8_t SD_Read_Block (uint8_t *buff, uint32_t lba)
{
	uint8_t result = SD_cmd (CMD17, lba);
	if (result) return 5;

	SPI_Release();

	uint16_t cnt = 0;
	do {
		result = SPI_ReceiveByte();
		cnt++;
	} while ( (result != 0xFE) && (cnt < 0xFFFF) );

	if (cnt >= 0xFFFF) return 5;
	for (cnt = 0; cnt < 512; cnt++) buff[cnt] = SPI_ReceiveByte();
	SPI_Release();
	SPI_Release();

	return 0;
}
//-----------------------------------------------
uint8_t SD_Write_Block (uint8_t *buff, uint32_t lba)
{
uint16_t cnt;

	uint8_t result = SD_cmd(CMD24, lba);
	if (result) return 6;

	SPI_Release();
	SPI_SendByte (0xFE);
	for (cnt = 0; cnt < 512; cnt++) SPI_SendByte(buff[cnt]);
	SPI_Release();
	SPI_Release();
	result = SPI_ReceiveByte();
	if ((result & 0x05) != 0x05) return 6;
	cnt = 0;
	do {
		result = SPI_ReceiveByte();
		cnt++;
	} while ( (result != 0xFF) && (cnt < 0xFFFF) );
	if (cnt >= 0xFFFF) return 6;

	return 0;
}
//-----------------------------------------------
uint8_t sd_ini(void)
{
uint8_t i, cmd, ret = 0, res = 0;//, cs = 0;
int16_t tmr;
//uint32_t temp;

	LD_OFF;

	sdinfo.type = 0;
	uint8_t ocr[4];
	//temp = portSPI->Init.BaudRatePrescaler;
	//portSPI->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128; //140.4 //156.25 kbbs
	//HAL_SPI_Init(portSPI);
	//SS_SD_DESELECT();
	//for (i = 0; i < 10; i++) SPI_Release();
	//portSPI->Init.BaudRatePrescaler = temp;
	//HAL_SPI_Init(portSPI);
	SS_SD_SELECT();
	res = SD_cmd(CMD0, 0);
	if (res == 1) {// Enter Idle state
		SPI_Release();
		if (SD_cmd(CMD8, 0x1AA) == 1) {// SDv2
			for (i = 0; i < 4; i++) ocr[i] = SPI_ReceiveByte();
//			Report(false, "\tOCR: 0x%02X 0x%02X 0x%02X 0x%02X\r\n", ocr[0], ocr[1], ocr[2], ocr[3]);
			// Get trailing return value of R7 resp
			if (ocr[2] == 0x01 && ocr[3] == 0xAA) {// The card can work at vdd range of 2.7-3.6V
				for (tmr = 12000; tmr && SD_cmd(ACMD41, 1UL << 30); tmr--); // Wait for leaving idle state (ACMD41 with HCS bit)
				if (tmr && SD_cmd(CMD58, 0) == 0) { // Check CCS bit in the OCR
					for (i = 0; i < 4; i++) ocr[i] = SPI_ReceiveByte();
//					Report(false, "\tOCR: 0x%02X 0x%02X 0x%02X 0x%02X\r\n", ocr[0], ocr[1], ocr[2], ocr[3]);
					sdinfo.type = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2; // SDv2 (HC or SC)
				}
			}
//			cs = 4;
		} else {//SDv1 or MMCv3
			if (SD_cmd(ACMD41, 0) <= 1) {
				sdinfo.type = CT_SD1;
				cmd = ACMD41; // SDv1
//				cs = 3;
			} else {
				sdinfo.type = CT_MMC;
				cmd = CMD1; // MMCv3
//				cs = 2;
			}
			for (tmr = 25000; tmr && SD_cmd(cmd, 0); tmr--) ; // Wait for leaving idle state
			if (!tmr || SD_cmd(CMD16, 512) != 0) // Set R/W block length to 512
			sdinfo.type = 0;
		}
	} else {
		ret = 1;
//		cs = 1;
	}
//	Report(false, "\t[%s] Type SD=0x%02X cs=%u res=%u ret=%u\r\n", __func__, sdinfo.type, cs, res, ret);

  return ret;
}
//-----------------------------------------------


