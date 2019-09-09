#include "w25.h"


#ifdef SET_W25FLASH


//------------------------------------------------------------------------------------------

w25qxx_t w25qxx;

//------------------------------------------------------------------------------------------
uint8_t W25qxx_Spi(uint8_t Data)
{
    uint8_t ret;
    HAL_SPI_TransmitReceive(portFLASH, &Data, &ret, 1, 10);
    return ret;
}
//------------------------------------------------------------------------------------------
uint32_t W25qxx_ReadID(void)
{
uint32_t Temp = 0, Temp0 = 0, Temp1 = 0, Temp2 = 0;

    W25_SELECT();//HAL_GPIO_WritePin(_W25QXX_CS_GPIO, _W25QXX_CS_PIN, GPIO_PIN_RESET);//set to 0
    W25qxx_Spi(0x9F);
    Temp0 = W25qxx_Spi(W25QXX_DUMMY_BYTE);
    Temp1 = W25qxx_Spi(W25QXX_DUMMY_BYTE);
    Temp2 = W25qxx_Spi(W25QXX_DUMMY_BYTE);
    W25_UNSELECT();//HAL_GPIO_WritePin(_W25QXX_CS_GPIO, _W25QXX_CS_PIN, GPIO_PIN_SET);
    Temp = (Temp0 << 16) | (Temp1 << 8) | Temp2;

    return Temp;
}
//------------------------------------------------------------------------------------------
void W25qxx_ReadUniqID(void)
{
    W25_SELECT();

    W25qxx_Spi(0x4B);

    for(uint8_t i = 0; i < 4; i++) W25qxx_Spi(W25QXX_DUMMY_BYTE);
    for(uint8_t i = 0; i < 8; i++) w25qxx.UniqID[i] = W25qxx_Spi(W25QXX_DUMMY_BYTE);

    W25_UNSELECT();
}
//------------------------------------------------------------------------------------------
void W25qxx_WriteEnable(void)
{
    W25_SELECT();

    W25qxx_Spi(0x06);

    W25_UNSELECT();

    W25qxx_Delay(1);
}
//------------------------------------------------------------------------------------------
void W25qxx_WriteDisable(void)
{
    W25_SELECT();

    W25qxx_Spi(0x04);

    W25_UNSELECT();

    W25qxx_Delay(1);
}
//------------------------------------------------------------------------------------------
uint8_t W25qxx_ReadStatusRegister(uint8_t SelectStatusReg)
{
uint8_t status = 0;

    W25_SELECT();

    switch (SelectStatusReg) {
        case 1:
            W25qxx_Spi(0x05);
            status = W25qxx_Spi(W25QXX_DUMMY_BYTE);
            w25qxx.StatusRegister1 = status;
        break;
        case 2:
            W25qxx_Spi(0x35);
            status = W25qxx_Spi(W25QXX_DUMMY_BYTE);
            w25qxx.StatusRegister2 = status;
        break;
        default : {
            W25qxx_Spi(0x15);
            status = W25qxx_Spi(W25QXX_DUMMY_BYTE);
            w25qxx.StatusRegister3 = status;
        }
    }

    W25_UNSELECT();

    return status;
}
//------------------------------------------------------------------------------------------
void W25qxx_WriteStatusRegister(uint8_t SelectStatusReg, uint8_t Data)
{
    W25_SELECT();

    switch (SelectStatusReg) {
        case 1 :
            W25qxx_Spi(0x01);
            w25qxx.StatusRegister1 = Data;
        break;
        case 2 :
            W25qxx_Spi(0x31);
            w25qxx.StatusRegister2 = Data;
        break;
        default : {
            W25qxx_Spi(0x11);
            w25qxx.StatusRegister3 = Data;
        }
    }

    W25qxx_Spi(Data);

    W25_UNSELECT();
}
//------------------------------------------------------------------------------------------
void W25qxx_WaitForWriteEnd(void)
{
    W25qxx_Delay(1);

    W25_SELECT();

    W25qxx_Spi(0x05);
    do
    {
        w25qxx.StatusRegister1 = W25qxx_Spi(W25QXX_DUMMY_BYTE);
        W25qxx_Delay(1);
    } while ((w25qxx.StatusRegister1 & 0x01) == 0x01);

    W25_UNSELECT();
}
//------------------------------------------------------------------------------------------
bool W25qxx_Init(void)
{
    w25qxx.Lock = 1;

    while (HAL_GetTick() < 100) W25qxx_Delay(1);

    uint32_t id = W25qxx_ReadID();
    #if (_W25QXX_DEBUG==1)
    Report(true, "w25qxx Init Begin...\r\nw25qxx ID:0x%X\r\n", id);
    #endif
    switch (id & 0x0000FFFF) {
        case 0x401A:// w25q512
            w25qxx.ID = W25Q512;
            w25qxx.BlockCount = 1024;
            #if (_W25QXX_DEBUG==1)
            Report(true, "w25qxx Chip: w25q512\r\n");
            #endif
        break;
        case 0x4019:// w25q256
            w25qxx.ID = W25Q256;
            w25qxx.BlockCount = 512;
            #if (_W25QXX_DEBUG==1)
            Report(true, "w25qxx Chip: w25q256\r\n");
            #endif
        break;
        case 0x4018:// w25q128
            w25qxx.ID = W25Q128;
            w25qxx.BlockCount = 256;
            #if (_W25QXX_DEBUG==1)
            Report(true, "w25qxx Chip: w25q128\r\n");
            #endif
        break;
        case 0x4017:// w25q64
            w25qxx.ID = W25Q64;
            w25qxx.BlockCount = 128;
            #if (_W25QXX_DEBUG==1)
            Report(true, "w25qxx Chip: w25q64\r\n");
            #endif
        break;
        case 0x4016://w25q32
            w25qxx.ID = W25Q32;
            w25qxx.BlockCount = 64;
            #if (_W25QXX_DEBUG==1)
            Report(true, "w25qxx Chip: w25q32\r\n");
            #endif
        break;
        case 0x4015://w25q16
            w25qxx.ID = W25Q16;
            w25qxx.BlockCount = 32;
            #if (_W25QXX_DEBUG==1)
            Report(true, "w25qxx Chip: w25q16\r\n");
            #endif
        break;
        case 0x4014://w25q80
            w25qxx.ID = W25Q80;
            w25qxx.BlockCount = 16;
            #if (_W25QXX_DEBUG==1)
            Report(true, "w25qxx Chip: w25q80\r\n");
            #endif
        break;
        case 0x4013://w25q40
            w25qxx.ID = W25Q40;
            w25qxx.BlockCount = 8;
            #if (_W25QXX_DEBUG==1)
            Report(true, "w25qxx Chip: w25q40\r\n");
            #endif
        break;
        case 0x4012://w25q20
            w25qxx.ID = W25Q20;
            w25qxx.BlockCount = 4;
            #if (_W25QXX_DEBUG==1)
            Report(true, "w25qxx Chip: w25q20\r\n");
            #endif
        break;
        case 0x4011://w25q10
            w25qxx.ID = W25Q10;
            w25qxx.BlockCount = 2;
            #if (_W25QXX_DEBUG==1)
            Report(true, "w25qxx Chip: w25q10\r\n");
            #endif
        break;
        default: {
            #if (_W25QXX_DEBUG==1)
            Report(true, "w25qxx Unknown ID\r\n");
            #endif

            w25qxx.Lock = 0;

            return false;
        }
    }
    w25qxx.PageSize = 256;
    w25qxx.SectorSize = 0x1000;
    w25qxx.SectorCount = w25qxx.BlockCount * 16;
    w25qxx.PageCount = (w25qxx.SectorCount * w25qxx.SectorSize) / w25qxx.PageSize;
    w25qxx.BlockSize = w25qxx.SectorSize * 16;
    w25qxx.CapacityInKiloByte = (w25qxx.SectorCount * w25qxx.SectorSize) / 1024;
    W25qxx_ReadUniqID();
    W25qxx_ReadStatusRegister(1);
    W25qxx_ReadStatusRegister(2);
    W25qxx_ReadStatusRegister(3);
    #if (_W25QXX_DEBUG==1)
    Report(true, "Chip info:\r\n"
    		     "w25qxx Page Size: %d Bytes\r\n"
                 "w25qxx Page Count: %d\r\n"
                 "w25qxx Sector Size: %d Bytes\r\n"
                 "w25qxx Sector Count: %d\r\n"
                 "w25qxx Block Size: %d Bytes\r\n"
                 "w25qxx Block Count: %d\r\n"
                 "w25qxx Capacity: %d KiloBytes\r\n"
                 "w25qxx Init Done\r\n",
                 w25qxx.PageSize,
                 w25qxx.PageCount,
                 w25qxx.SectorSize,
                 w25qxx.SectorCount,
                 w25qxx.BlockSize,
                 w25qxx.BlockCount,
                 w25qxx.CapacityInKiloByte);
    #endif

    w25qxx.Lock=0;

    return true;
}
//------------------------------------------------------------------------------------------
void W25qxx_EraseChip(void)
{
    while (w25qxx.Lock) W25qxx_Delay(1);

    w25qxx.Lock = 1;

    #if (_W25QXX_DEBUG==1)
    uint32_t StartTime = HAL_GetTick();
    Report(true, "%s Begin...\r\n", __func__);
    #endif
    W25qxx_WriteEnable();

    W25_SELECT();

    W25qxx_Spi(0xC7);

    W25_UNSELECT();

    W25qxx_WaitForWriteEnd();
    #if (_W25QXX_DEBUG==1)
    Report(true, "%s done after %u ms!\r\n", __func__, HAL_GetTick() - StartTime);
    #endif
    W25qxx_Delay(10);

    w25qxx.Lock = 0;
}
//------------------------------------------------------------------------------------------
void W25qxx_EraseSector(uint32_t SectorAddr)
{
    while (w25qxx.Lock) W25qxx_Delay(1);

    w25qxx.Lock = 1;

    #if (_W25QXX_DEBUG==1)
    uint32_t StartTime = HAL_GetTick();
    Report(true, "%s %u Begin...\r\n", __func__, SectorAddr);
    #endif
    W25qxx_WaitForWriteEnd();
    SectorAddr = SectorAddr * w25qxx.SectorSize;
    W25qxx_WriteEnable();

    W25_SELECT();
    W25qxx_Spi(0x20);
    if (w25qxx.ID >= W25Q256) W25qxx_Spi((SectorAddr & 0xFF000000) >> 24);
    W25qxx_Spi((SectorAddr & 0xFF0000) >> 16);
    W25qxx_Spi((SectorAddr & 0xFF00) >> 8);
    W25qxx_Spi(SectorAddr & 0xFF);
    W25_UNSELECT();

    W25qxx_WaitForWriteEnd();
    #if (_W25QXX_DEBUG==1)
    Report(true, "%s done after %u ms\r\n", __func__, HAL_GetTick() - StartTime);
    #endif
    W25qxx_Delay(1);

    w25qxx.Lock = 0;
}
//------------------------------------------------------------------------------------------
void W25qxx_EraseBlock(uint32_t BlockAddr)
{
    while (w25qxx.Lock) W25qxx_Delay(1);

    w25qxx.Lock = 1;

    #if (_W25QXX_DEBUG==1)
    Report(true, "%s %u Begin...\r\n", __func__, BlockAddr);
    W25qxx_Delay(100);
    uint32_t StartTime = HAL_GetTick();
    #endif
    W25qxx_WaitForWriteEnd();
    BlockAddr = BlockAddr * w25qxx.SectorSize * 16;
    W25qxx_WriteEnable();

    W25_SELECT();
    W25qxx_Spi(0xD8);
    if(w25qxx.ID >= W25Q256) W25qxx_Spi((BlockAddr & 0xFF000000) >> 24);
    W25qxx_Spi((BlockAddr & 0xFF0000) >> 16);
    W25qxx_Spi((BlockAddr & 0xFF00) >> 8);
    W25qxx_Spi(BlockAddr & 0xFF);
    W25_UNSELECT();

    W25qxx_WaitForWriteEnd();
    #if (_W25QXX_DEBUG==1)
    Report(true, "%s done after %u ms\r\n", __func__, HAL_GetTick() - StartTime);
    W25qxx_Delay(100);
    #endif
    W25qxx_Delay(1);

    w25qxx.Lock = 0;
}
//------------------------------------------------------------------------------------------
uint32_t W25qxx_PageToSector(uint32_t PageAddress)
{
    return ((PageAddress * w25qxx.PageSize) / w25qxx.SectorSize);
}
//------------------------------------------------------------------------------------------
uint32_t W25qxx_PageToBlock(uint32_t PageAddress)
{
    return ((PageAddress * w25qxx.PageSize) / w25qxx.BlockSize);
}
//------------------------------------------------------------------------------------------
uint32_t W25qxx_SectorToBlock(uint32_t SectorAddress)
{
    return ((SectorAddress * w25qxx.SectorSize) / w25qxx.BlockSize);
}
//------------------------------------------------------------------------------------------
uint32_t W25qxx_SectorToPage(uint32_t SectorAddress)
{
    return (SectorAddress * w25qxx.SectorSize) / w25qxx.PageSize;
}
//------------------------------------------------------------------------------------------
uint32_t W25qxx_BlockToPage(uint32_t BlockAddress)
{
    return (BlockAddress * w25qxx.BlockSize) / w25qxx.PageSize;
}
//------------------------------------------------------------------------------------------
bool W25qxx_IsEmptyPage(uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToCheck_up_to_PageSize)
{

    while(w25qxx.Lock) W25qxx_Delay(1);

    w25qxx.Lock = 1;

    if ( ((NumByteToCheck_up_to_PageSize + OffsetInByte) > w25qxx.PageSize) ||
            (!NumByteToCheck_up_to_PageSize) )
                        NumByteToCheck_up_to_PageSize = w25qxx.PageSize - OffsetInByte;
    #if (_W25QXX_DEBUG==1)
    Report(true, "w25qxx CheckPage:0x%X(%u), Offset:%u, Bytes:%u begin...\r\n",
                 Page_Address, Page_Address, OffsetInByte, NumByteToCheck_up_to_PageSize);
    W25qxx_Delay(100);
    uint32_t StartTime = HAL_GetTick();
    #endif
    uint8_t pBuffer[32];
    uint32_t i, WorkAddress;
    for (i = OffsetInByte; i < w25qxx.PageSize; i += sizeof(pBuffer)) {
        W25_SELECT();
        WorkAddress = (i + Page_Address * w25qxx.PageSize);
        W25qxx_Spi(0x0B);
        if (w25qxx.ID >= W25Q256) W25qxx_Spi((WorkAddress & 0xFF000000) >> 24);
        W25qxx_Spi((WorkAddress & 0xFF0000) >> 16);
        W25qxx_Spi((WorkAddress & 0xFF00) >> 8);
        W25qxx_Spi(WorkAddress & 0xFF);
        W25qxx_Spi(0);
        HAL_SPI_Receive(portFLASH, pBuffer, sizeof(pBuffer), 100);
        W25_UNSELECT();
        for (uint8_t x = 0; x < sizeof(pBuffer); x++) {
            if (pBuffer[x] != 0xFF) goto NOT_EMPTY;
        }
    }
    if ((w25qxx.PageSize + OffsetInByte) % sizeof(pBuffer) != 0) {
        i -= sizeof(pBuffer);
        for ( ; i < w25qxx.PageSize; i++) {
            W25_SELECT();
            WorkAddress = (i + Page_Address * w25qxx.PageSize);
            W25qxx_Spi(0x0B);
            if (w25qxx.ID >= W25Q256) W25qxx_Spi((WorkAddress & 0xFF000000) >> 24);
            W25qxx_Spi((WorkAddress & 0xFF0000) >> 16);
            W25qxx_Spi((WorkAddress & 0xFF00) >> 8);
            W25qxx_Spi(WorkAddress & 0xFF);
            W25qxx_Spi(0);
            HAL_SPI_Receive(portFLASH, pBuffer, 1, 100);
            W25_UNSELECT();
            if (pBuffer[0] != 0xFF) goto NOT_EMPTY;
        }
    }
    #if (_W25QXX_DEBUG==1)
    Report(true, "w25qxx CheckPage is Empty in %u ms\r\n", HAL_GetTick() - StartTime);
    W25qxx_Delay(100);
    #endif

    w25qxx.Lock = 0;

    return true;

NOT_EMPTY:
    #if (_W25QXX_DEBUG==1)
    Report(true, "w25qxx CheckPage is Not Empty in %u ms\r\n", HAL_GetTick() - StartTime);
    W25qxx_Delay(100);
    #endif

    w25qxx.Lock = 0;

    return false;
}
//------------------------------------------------------------------------------------------
bool W25qxx_IsEmptySector(uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToCheck_up_to_SectorSize)
{
    while (w25qxx.Lock) W25qxx_Delay(1);

    w25qxx.Lock = 1;

    if ( (NumByteToCheck_up_to_SectorSize > w25qxx.SectorSize) || (!NumByteToCheck_up_to_SectorSize) )
                NumByteToCheck_up_to_SectorSize = w25qxx.SectorSize;
    #if (_W25QXX_DEBUG==1)
    Report(true, "w25qxx CheckSector:0x%X(%u), Offset:%u, Bytes:%u begin...\r\n",
                 Sector_Address, Sector_Address, OffsetInByte, NumByteToCheck_up_to_SectorSize);
    W25qxx_Delay(100);
    uint32_t StartTime = HAL_GetTick();
    #endif
    uint8_t pBuffer[32];
    uint32_t i, WorkAddress;
    for ( i = OffsetInByte; i < w25qxx.SectorSize; i += sizeof(pBuffer)) {
        W25_SELECT();
        WorkAddress = (i + Sector_Address * w25qxx.SectorSize);
        W25qxx_Spi(0x0B);
        if (w25qxx.ID >= W25Q256) W25qxx_Spi((WorkAddress & 0xFF000000) >> 24);
        W25qxx_Spi((WorkAddress & 0xFF0000) >> 16);
        W25qxx_Spi((WorkAddress & 0xFF00) >> 8);
        W25qxx_Spi(WorkAddress & 0xFF);
        W25qxx_Spi(0);
        HAL_SPI_Receive(portFLASH, pBuffer, sizeof(pBuffer), 100);
        W25_UNSELECT();
        for (uint8_t x = 0; x < sizeof(pBuffer); x++) {
            if (pBuffer[x] != 0xFF) goto NOT_EMPTY;
        }
    }
    if ((w25qxx.SectorSize + OffsetInByte) % sizeof(pBuffer) != 0) {
        i -= sizeof(pBuffer);
        for( ; i < w25qxx.SectorSize; i++) {
            W25_SELECT();
            WorkAddress = (i + Sector_Address * w25qxx.SectorSize);
            W25qxx_Spi(0x0B);
            if (w25qxx.ID >= W25Q256) W25qxx_Spi((WorkAddress & 0xFF000000) >> 24);
            W25qxx_Spi((WorkAddress & 0xFF0000) >> 16);
            W25qxx_Spi((WorkAddress & 0xFF00) >> 8);
            W25qxx_Spi(WorkAddress & 0xFF);
            W25qxx_Spi(0);
            HAL_SPI_Receive(portFLASH, pBuffer, 1, 100);
            W25_UNSELECT();
            if (pBuffer[0] != 0xFF) goto NOT_EMPTY;
        }
    }

    #if (_W25QXX_DEBUG==1)
    Report(true, "w25qxx CheckSector is Empty in %u ms\r\n", HAL_GetTick() - StartTime);
    W25qxx_Delay(100);
    #endif

    w25qxx.Lock = 0;

    return true;

NOT_EMPTY:
    #if (_W25QXX_DEBUG==1)
    Report(true, "w25qxx CheckSector is Not Empty in %u ms\r\n", HAL_GetTick() - StartTime);
    W25qxx_Delay(100);
    #endif

    w25qxx.Lock = 0;

    return false;
}
//------------------------------------------------------------------------------------------
bool W25qxx_IsEmptyBlock(uint32_t Block_Address, uint32_t OffsetInByte, uint32_t NumByteToCheck_up_to_BlockSize)
{
    while (w25qxx.Lock) W25qxx_Delay(1);

    w25qxx.Lock = 1;

    if ( (NumByteToCheck_up_to_BlockSize > w25qxx.BlockSize) || !NumByteToCheck_up_to_BlockSize )
                          NumByteToCheck_up_to_BlockSize = w25qxx.BlockSize;
    #if (_W25QXX_DEBUG==1)
    Report(true, "w25qxx CheckBlock:0x%X(%u), Offset:%u, Bytes:%u begin...\r\n",
                 Block_Address, Block_Address, OffsetInByte, NumByteToCheck_up_to_BlockSize);
    W25qxx_Delay(100);
    uint32_t StartTime = HAL_GetTick();
    #endif
    uint8_t pBuffer[32];
    uint32_t i, WorkAddress;
    for (i = OffsetInByte; i < w25qxx.BlockSize; i += sizeof(pBuffer)) {
        W25_SELECT();
        WorkAddress = (i + Block_Address * w25qxx.BlockSize);
        W25qxx_Spi(0x0B);
        if (w25qxx.ID >= W25Q256) W25qxx_Spi((WorkAddress & 0xFF000000) >> 24);
        W25qxx_Spi((WorkAddress & 0xFF0000) >> 16);
        W25qxx_Spi((WorkAddress & 0xFF00) >> 8);
        W25qxx_Spi(WorkAddress & 0xFF);
        W25qxx_Spi(0);
        HAL_SPI_Receive(portFLASH, pBuffer, sizeof(pBuffer), 100);
        W25_UNSELECT();
        for (uint8_t x = 0; x < sizeof(pBuffer); x++) {
            if(pBuffer[x] != 0xFF) goto NOT_EMPTY;
        }
    }
    if ((w25qxx.BlockSize + OffsetInByte) % sizeof(pBuffer) != 0) {
        i -= sizeof(pBuffer);
        for ( ; i < w25qxx.BlockSize; i++) {
            W25_SELECT();
            WorkAddress = (i + Block_Address * w25qxx.BlockSize);
            W25qxx_Spi(0x0B);
            if (w25qxx.ID >= W25Q256) W25qxx_Spi((WorkAddress & 0xFF000000) >> 24);
            W25qxx_Spi((WorkAddress & 0xFF0000) >> 16);
            W25qxx_Spi((WorkAddress & 0xFF00) >> 8);
            W25qxx_Spi(WorkAddress & 0xFF);
            W25qxx_Spi(0);
            HAL_SPI_Receive(portFLASH, pBuffer, 1, 100);
            W25_UNSELECT();
            if (pBuffer[0] != 0xFF) goto NOT_EMPTY;
        }
    }
    #if (_W25QXX_DEBUG==1)
    Report(true, "w25qxx CheckBlock is Empty in %u ms\r\n", HAL_GetTick() - StartTime);
    W25qxx_Delay(100);
    #endif

    w25qxx.Lock = 0;

    return true;

NOT_EMPTY:
    #if (_W25QXX_DEBUG==1)
    Report(true, "w25qxx CheckBlock is Not Empty in %u ms\r\n", HAL_GetTick() - StartTime);
    W25qxx_Delay(100);
    #endif

    w25qxx.Lock = 0;

    return false;
}
//------------------------------------------------------------------------------------------
void W25qxx_WriteByte(uint8_t pBuffer, uint32_t WriteAddr_inBytes)
{
    while (w25qxx.Lock) W25qxx_Delay(1);

    w25qxx.Lock = 1;

    #if (_W25QXX_DEBUG==1)
    uint32_t StartTime = HAL_GetTick();
    Report(true, "%s 0x%02X at address %d begin...", __func__, pBuffer, WriteAddr_inBytes);
    #endif
    W25qxx_WaitForWriteEnd();
    W25qxx_WriteEnable();

    W25_SELECT();
    W25qxx_Spi(0x02);
    if (w25qxx.ID >= W25Q256) W25qxx_Spi((WriteAddr_inBytes & 0xFF000000) >> 24);
    W25qxx_Spi((WriteAddr_inBytes & 0xFF0000) >> 16);
    W25qxx_Spi((WriteAddr_inBytes & 0xFF00) >> 8);
    W25qxx_Spi(WriteAddr_inBytes & 0xFF);
    W25qxx_Spi(pBuffer);
    W25_UNSELECT();

    W25qxx_WaitForWriteEnd();
    #if (_W25QXX_DEBUG==1)
    Report(true, "%s done after %d ms\r\n", __func__, HAL_GetTick() - StartTime);
    #endif

    w25qxx.Lock = 0;
}
//------------------------------------------------------------------------------------------
void W25qxx_WritePage(uint8_t *pBuffer, uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_PageSize)
{
    while (w25qxx.Lock) W25qxx_Delay(1);

    w25qxx.Lock = 1;

    if ( ((NumByteToWrite_up_to_PageSize + OffsetInByte) > w25qxx.PageSize) || !NumByteToWrite_up_to_PageSize )
                NumByteToWrite_up_to_PageSize = w25qxx.PageSize - OffsetInByte;
    if ( (OffsetInByte + NumByteToWrite_up_to_PageSize) > w25qxx.PageSize )
                NumByteToWrite_up_to_PageSize = w25qxx.PageSize - OffsetInByte;
    #if (_W25QXX_DEBUG==1)
    Report(true, "%s WritePage:0x%X(%u), Offset:%u ,Writes %u Bytes, begin...\r\n",
                 __func__, Page_Address, Page_Address, OffsetInByte, NumByteToWrite_up_to_PageSize);
    W25qxx_Delay(100);
    uint32_t StartTime = HAL_GetTick();
    #endif
    W25qxx_WaitForWriteEnd();
    W25qxx_WriteEnable();

    W25_SELECT();
    W25qxx_Spi(0x02);
    Page_Address = (Page_Address * w25qxx.PageSize) + OffsetInByte;
    if (w25qxx.ID >= W25Q256) W25qxx_Spi((Page_Address & 0xFF000000) >> 24);
    W25qxx_Spi((Page_Address & 0xFF0000) >> 16);
    W25qxx_Spi((Page_Address & 0xFF00) >> 8);
    W25qxx_Spi(Page_Address&0xFF);
    HAL_SPI_Transmit(portFLASH, pBuffer, NumByteToWrite_up_to_PageSize, 100);
    W25_UNSELECT();
    W25qxx_WaitForWriteEnd();
    #if (_W25QXX_DEBUG==1)
    StartTime = HAL_GetTick() - StartTime;
    for (uint32_t i = 0; i < NumByteToWrite_up_to_PageSize ; i++) {
        if ( (i % 8 == 0) && (i > 2) ) {
            Report(false, "\r\n");
            W25qxx_Delay(10);
        }
        Report(false, "0x%02X,", pBuffer[i]);
    }
    Report(false, "\r\n");
    Report(true, "%s done after %u ms\r\n", __func__, StartTime);
    W25qxx_Delay(100);
    #endif
    W25qxx_Delay(1);

    w25qxx.Lock = 0;
}
//------------------------------------------------------------------------------------------
void W25qxx_WriteSector(uint8_t *pBuffer, uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_SectorSize)
{
    if ((NumByteToWrite_up_to_SectorSize > w25qxx.SectorSize) || !NumByteToWrite_up_to_SectorSize)
                NumByteToWrite_up_to_SectorSize = w25qxx.SectorSize;
    #if (_W25QXX_DEBUG==1)
    Report(true, "%s WriteSector:0x%X(%u), Offset:%u ,Write %u Bytes, begin...\r\n",
                 __func__, Sector_Address, Sector_Address, OffsetInByte, NumByteToWrite_up_to_SectorSize);
    W25qxx_Delay(100);
    #endif
    if (OffsetInByte >= w25qxx.SectorSize) {
        #if (_W25QXX_DEBUG==1)
        Report(true, "---w25qxx WriteSector Faild!\r\n");
        W25qxx_Delay(100);
        #endif

        return;

    }

    int32_t BytesToWrite;
    uint32_t LocalOffset, StartPage;
    if ((OffsetInByte + NumByteToWrite_up_to_SectorSize) > w25qxx.SectorSize)
        BytesToWrite = w25qxx.SectorSize - OffsetInByte;
    else
        BytesToWrite = NumByteToWrite_up_to_SectorSize;
    StartPage = W25qxx_SectorToPage(Sector_Address) + (OffsetInByte / w25qxx.PageSize);
    LocalOffset = OffsetInByte % w25qxx.PageSize;

    do
    {
        W25qxx_WritePage(pBuffer, StartPage, LocalOffset, BytesToWrite);
        StartPage++;
        BytesToWrite -= w25qxx.PageSize - LocalOffset;
        pBuffer += w25qxx.PageSize;
        LocalOffset = 0;
    } while(BytesToWrite > 0);
    #if (_W25QXX_DEBUG==1)
    printf("%s Done\r\n", __func__);
    W25qxx_Delay(100);
    #endif
}
//------------------------------------------------------------------------------------------
void W25qxx_WriteBlock(uint8_t *pBuffer, uint32_t Block_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_BlockSize)
{
    if ((NumByteToWrite_up_to_BlockSize > w25qxx.BlockSize) || !NumByteToWrite_up_to_BlockSize)
            NumByteToWrite_up_to_BlockSize = w25qxx.BlockSize;
    #if (_W25QXX_DEBUG==1)
    Report(true, "%s WriteBlock:0x%X(%u), Offset:%u ,Write %u Bytes, begin...\r\n",
                 __func__, Block_Address, Block_Address, OffsetInByte, NumByteToWrite_up_to_BlockSize);
    W25qxx_Delay(100);
    #endif
    if (OffsetInByte >= w25qxx.BlockSize) {
        #if (_W25QXX_DEBUG==1)
        Report(true, "%s Faild!\r\n", __func__);
        W25qxx_Delay(100);
        #endif

        return;
    }

    int32_t BytesToWrite;
    uint32_t LocalOffset, StartPage;
    if ((OffsetInByte + NumByteToWrite_up_to_BlockSize) > w25qxx.BlockSize)
        BytesToWrite = w25qxx.BlockSize - OffsetInByte;
    else
        BytesToWrite = NumByteToWrite_up_to_BlockSize;
    StartPage = W25qxx_BlockToPage(Block_Address) + (OffsetInByte / w25qxx.PageSize);
    LocalOffset = OffsetInByte % w25qxx.PageSize;
    do
    {
        W25qxx_WritePage(pBuffer, StartPage, LocalOffset, BytesToWrite);
        StartPage++;
        BytesToWrite -= w25qxx.PageSize - LocalOffset;
        pBuffer += w25qxx.PageSize;
        LocalOffset = 0;
    } while(BytesToWrite > 0);

    #if (_W25QXX_DEBUG==1)
    Report(true, "%s done\r\n", __func__);
    W25qxx_Delay(100);
    #endif
}
//------------------------------------------------------------------------------------------
void W25qxx_ReadByte(uint8_t *pBuffer, uint32_t Bytes_Address)
{
    while (w25qxx.Lock) W25qxx_Delay(1);

    w25qxx.Lock = 1;

    #if (_W25QXX_DEBUG==1)
    uint32_t StartTime = HAL_GetTick();
    Report(true, "%s at address %u begin...\r\n", __func__, Bytes_Address);
    #endif

    W25_SELECT();
    W25qxx_Spi(0x0B);
    if (w25qxx.ID >= W25Q256) W25qxx_Spi((Bytes_Address & 0xFF000000) >> 24);
    W25qxx_Spi((Bytes_Address & 0xFF0000) >> 16);
    W25qxx_Spi((Bytes_Address& 0xFF00) >> 8);
    W25qxx_Spi(Bytes_Address & 0xFF);
    W25qxx_Spi(0);
    *pBuffer = W25qxx_Spi(W25QXX_DUMMY_BYTE);
    W25_UNSELECT();

    #if (_W25QXX_DEBUG==1)
    Report(true, "%s 0x%02X done after %u ms\r\n", __func__, *pBuffer, HAL_GetTick() - StartTime);
    #endif

    w25qxx.Lock = 0;
}
//------------------------------------------------------------------------------------------
void W25qxx_ReadBytes(uint8_t *pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead)
{
    while (w25qxx.Lock) W25qxx_Delay(1);

    w25qxx.Lock = 1;

    #if (_W25QXX_DEBUG==1)
    uint32_t StartTime = HAL_GetTick();
    Report(true, "%s at Address:0x%X(%u), %u Bytes  begin...\r\n", __func__, ReadAddr, ReadAddr, NumByteToRead);
    #endif

    W25_SELECT();
    W25qxx_Spi(0x0B);
    if (w25qxx.ID >= W25Q256) W25qxx_Spi((ReadAddr & 0xFF000000) >> 24);
    W25qxx_Spi((ReadAddr & 0xFF0000) >> 16);
    W25qxx_Spi((ReadAddr& 0xFF00) >> 8);
    W25qxx_Spi(ReadAddr & 0xFF);
    W25qxx_Spi(0);
    HAL_SPI_Receive(portFLASH, pBuffer, NumByteToRead, 2000);
    W25_UNSELECT();

    #if (_W25QXX_DEBUG==1)
    StartTime = HAL_GetTick() - StartTime;
    for (uint32_t i = 0; i < NumByteToRead ; i++) {
        if ((i % 8 == 0) && (i > 2)) {
            Report(false, "\r\n");
            W25qxx_Delay(10);
        }
        Report(false, "0x%02X,", pBuffer[i]);
    }
    Report(false, "\r\n");
    Report(true, "%s done after %u ms\r\n", __func__, StartTime);
    W25qxx_Delay(100);
    #endif
    W25qxx_Delay(1);

    w25qxx.Lock = 0;
}
//------------------------------------------------------------------------------------------
void W25qxx_ReadPage(uint8_t *pBuffer, uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_PageSize)
{
    while (w25qxx.Lock) W25qxx_Delay(1);

    w25qxx.Lock = 1;

    if ((NumByteToRead_up_to_PageSize > w25qxx.PageSize) || !NumByteToRead_up_to_PageSize)
        NumByteToRead_up_to_PageSize = w25qxx.PageSize;
    if ((OffsetInByte + NumByteToRead_up_to_PageSize) > w25qxx.PageSize)
        NumByteToRead_up_to_PageSize = w25qxx.PageSize - OffsetInByte;
    #if (_W25QXX_DEBUG==1)
    Report(true, "%s:0x%X(%u), Offset:%u ,Read %u Bytes, begin...\r\n",
                 __func__, Page_Address, Page_Address, OffsetInByte, NumByteToRead_up_to_PageSize);
    W25qxx_Delay(100);
    uint32_t StartTime = HAL_GetTick();
    #endif
    Page_Address = Page_Address * w25qxx.PageSize + OffsetInByte;

    W25_SELECT();
    W25qxx_Spi(0x0B);
    if (w25qxx.ID >= W25Q256) W25qxx_Spi((Page_Address & 0xFF000000) >> 24);
    W25qxx_Spi((Page_Address & 0xFF0000) >> 16);
    W25qxx_Spi((Page_Address& 0xFF00) >> 8);
    W25qxx_Spi(Page_Address & 0xFF);
    W25qxx_Spi(0);
    HAL_SPI_Receive(portFLASH, pBuffer, NumByteToRead_up_to_PageSize, 100);
    W25_UNSELECT();

    #if (_W25QXX_DEBUG==1)
    StartTime = HAL_GetTick() - StartTime;
    for (uint32_t i = 0; i < NumByteToRead_up_to_PageSize ; i++) {
        if ((i % 8 == 0) && (i > 2)) {
            Report(false, "\r\n");
            W25qxx_Delay(10);
        }
        Report(false, "0x%02X,", pBuffer[i]);
    }
    Report(false, "\r\n");
    Report(true, "%s done after %u ms\r\n", __func__, StartTime);
    W25qxx_Delay(100);
    #endif
    W25qxx_Delay(1);

    w25qxx.Lock = 0;
}
//------------------------------------------------------------------------------------------
void W25qxx_ReadSector(uint8_t *pBuffer, uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_SectorSize)
{
    if ((NumByteToRead_up_to_SectorSize > w25qxx.SectorSize) || !NumByteToRead_up_to_SectorSize)
                NumByteToRead_up_to_SectorSize = w25qxx.SectorSize;
    #if (_W25QXX_DEBUG==1)
    Report(true, "%s:0x%X(%u), Offset:%u ,Read %u Bytes, begin...\r\n",
                 __func__, Sector_Address, Sector_Address, OffsetInByte, NumByteToRead_up_to_SectorSize);
    W25qxx_Delay(100);
    #endif
    if (OffsetInByte >= w25qxx.SectorSize) {
        #if (_W25QXX_DEBUG==1)
        Report(true, "---w25qxx ReadSector Faild!\r\n");
        W25qxx_Delay(100);
        #endif

        return;
    }

    int32_t BytesToRead;
    uint32_t LocalOffset, StartPage;
    if ((OffsetInByte + NumByteToRead_up_to_SectorSize) > w25qxx.SectorSize)
        BytesToRead = w25qxx.SectorSize - OffsetInByte;
    else
        BytesToRead = NumByteToRead_up_to_SectorSize;
    StartPage = W25qxx_SectorToPage(Sector_Address) + (OffsetInByte / w25qxx.PageSize);
    LocalOffset = OffsetInByte % w25qxx.PageSize;
    do
    {
        W25qxx_ReadPage(pBuffer, StartPage, LocalOffset, BytesToRead);
        StartPage++;
        BytesToRead -= w25qxx.PageSize - LocalOffset;
        pBuffer += w25qxx.PageSize;
        LocalOffset = 0;
    } while(BytesToRead > 0);

    #if (_W25QXX_DEBUG==1)
    Report(true, "%s done\r\n", __func__);
    W25qxx_Delay(100);
    #endif
}
//------------------------------------------------------------------------------------------
void W25qxx_ReadBlock(uint8_t *pBuffer, uint32_t Block_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_BlockSize)
{
    if ((NumByteToRead_up_to_BlockSize > w25qxx.BlockSize) || !NumByteToRead_up_to_BlockSize)
        NumByteToRead_up_to_BlockSize = w25qxx.BlockSize;
    #if (_W25QXX_DEBUG==1)
    Report(true, "%s:0x%X(%u), Offset:%u ,Read %u Bytes, begin...\r\n",
                 __func__, Block_Address, Block_Address, OffsetInByte, NumByteToRead_up_to_BlockSize);
    W25qxx_Delay(100);
    #endif
    if (OffsetInByte >= w25qxx.BlockSize) {
        #if (_W25QXX_DEBUG==1)
        printf("%s Faild!\r\n", __func__);
        W25qxx_Delay(100);
        #endif

        return;
    }

    int32_t BytesToRead;
    uint32_t LocalOffset, StartPage;
    if ((OffsetInByte + NumByteToRead_up_to_BlockSize) > w25qxx.BlockSize)
        BytesToRead = w25qxx.BlockSize - OffsetInByte;
    else
        BytesToRead = NumByteToRead_up_to_BlockSize;
    StartPage = W25qxx_BlockToPage(Block_Address) + (OffsetInByte / w25qxx.PageSize);
    LocalOffset = OffsetInByte % w25qxx.PageSize;
    do
    {
        W25qxx_ReadPage(pBuffer, StartPage, LocalOffset, BytesToRead);
        StartPage++;
        BytesToRead -= w25qxx.PageSize - LocalOffset;
        pBuffer += w25qxx.PageSize;
        LocalOffset = 0;
    } while(BytesToRead > 0);

    #if (_W25QXX_DEBUG==1)
    Report(true, "%s done\r\n", __func__);
    W25qxx_Delay(100);
    #endif
}
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------

#endif
