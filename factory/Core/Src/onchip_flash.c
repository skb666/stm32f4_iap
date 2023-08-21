#include "onchip_flash.h"

#include "main.h"

/**------------------------------------------
  * @brief  Gets the sector of a given address
  * @param  Address: Flash address
  * @retval The sector of a given address
  --------------------------------------------*/
uint8_t STMFLASH_GetFlashSector(uint32_t addr) {
  if (addr < ADDR_FLASH_SECTOR_1) {
    return FLASH_SECTOR_0;
  } else if (addr < ADDR_FLASH_SECTOR_2) {
    return FLASH_SECTOR_1;
  } else if (addr < ADDR_FLASH_SECTOR_3) {
    return FLASH_SECTOR_2;
  } else if (addr < ADDR_FLASH_SECTOR_4) {
    return FLASH_SECTOR_3;
  } else if (addr < ADDR_FLASH_SECTOR_5) {
    return FLASH_SECTOR_4;
  } else if (addr < ADDR_FLASH_SECTOR_6) {
    return FLASH_SECTOR_5;
  } else if (addr < ADDR_FLASH_SECTOR_7) {
    return FLASH_SECTOR_6;
  } else if (addr < ADDR_FLASH_SECTOR_8) {
    return FLASH_SECTOR_7;
  } else if (addr < ADDR_FLASH_SECTOR_9) {
    return FLASH_SECTOR_8;
  } else if (addr < ADDR_FLASH_SECTOR_10) {
    return FLASH_SECTOR_9;
  } else if (addr < ADDR_FLASH_SECTOR_11) {
    return FLASH_SECTOR_10;
  }
  return FLASH_SECTOR_11;
}

uint32_t STMFLASH_ReadWord(uint32_t faddr) {
  return *(__IO uint32_t *)faddr;
}

void STMFLASH_Write(uint32_t WriteAddr, uint32_t *pBuffer, uint32_t Num) {
  FLASH_EraseInitTypeDef FlashEraseInit;
  HAL_StatusTypeDef FlashStatus = HAL_OK;
  uint32_t SectorError = 0;
  uint32_t addrx = 0;
  uint32_t endaddr = 0;
  if (WriteAddr < STM32_FLASH_BASE || WriteAddr % 4) {
    return; // 非法地址
  }

  HAL_FLASH_Unlock();            // 解锁
  addrx = WriteAddr;             // 写入的起始地址
  endaddr = WriteAddr + Num * 4; // 写入的结束地址

  if (addrx <= STM32_FLASH_END) {
    while (addrx < endaddr) {
      if (STMFLASH_ReadWord(addrx) != 0XFFFFFFFF) {
        FlashEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;     // 擦除类型，扇区擦除
        FlashEraseInit.Sector = STMFLASH_GetFlashSector(addrx); // 要擦除的扇区
        FlashEraseInit.NbSectors = 1;                           // 一次只擦除一个扇区
        FlashEraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;    // 电压范围，VCC=2.7~3.6V之间!!
        if (HAL_FLASHEx_Erase(&FlashEraseInit, &SectorError) != HAL_OK) {
          break; // 发生错误了
        }
      } else
        addrx += 4;
      FLASH_WaitForLastOperation(FLASH_WAITETIME); // 等待上次操作完成
    }
  }
  FlashStatus = FLASH_WaitForLastOperation(FLASH_WAITETIME); // 等待上次操作完成
  if (FlashStatus == HAL_OK) {
    while (WriteAddr < endaddr) {                                                     // 写数据
      if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, WriteAddr, *pBuffer) != HAL_OK) { // 写入数据
        break;                                                                        // 写入异常
      }
      WriteAddr += 4;
      pBuffer++;
    }
  }
  HAL_FLASH_Lock(); // 上锁
}

void STMFLASH_Read(uint32_t ReadAddr, uint32_t *pBuffer, uint32_t size) {
  uint32_t i;
  for (i = 0; i < size; i++) {
    pBuffer[i] = STMFLASH_ReadWord(ReadAddr); // 读取4个字节.
    ReadAddr += 4;                            // 偏移4个字节.
  }
}