#include "w25qxx.h"

#include <stdio.h>

#include "main.h"

// 定义使用的芯片型号
static uint16_t W25QXX_TYPE = W25Q16;

/*
 * @brief CS使能控制函数
 *
 * @param a:0为低电平 表示有效
 *        a:其他值为高电平 表示无效
 */
__attribute__((weak)) void W25QXX_CS(uint8_t a) {
  if (a == 0) {
    LL_GPIO_ResetOutputPin(F_CS_GPIO_Port, F_CS_Pin);
  } else {
    LL_GPIO_SetOutputPin(F_CS_GPIO_Port, F_CS_Pin);
  }
}

/*
 * @brief   SPI1总线读写一个字节
 *
 * @param   TxData:写入的字节
 *
 * @return  读出的字节
 */
__attribute__((weak)) uint8_t SPI1_ReadWriteByte(uint8_t TxData) {
  uint8_t retry = 0;

  /* Check if Tx buffer is empty */
  while (!LL_SPI_IsActiveFlag_TXE(SPI1)) {
    retry++;
    if (retry > 200) return 0;
  }

  /* Write character in Data register.
  TXE flag is cleared by reading data in DR register */
  LL_SPI_TransmitData8(SPI1, TxData);
  retry = 0;

  /* Check if Rx buffer is not empty */
  while (!LL_SPI_IsActiveFlag_RXNE(SPI1)) {
    retry++;
    if (retry > 200) return 0;
  }

  /* received byte from SPI lines. */
  return LL_SPI_ReceiveData8(SPI1);
}

/*
 * @brief   读取芯片ID
 *
 * @note    高8位是厂商代号（本程序不判断厂商代号）、低8位是容量大小
 *            0XEF13型号为W25Q80
 *            0XEF14型号为W25Q16
 *            0XEF15型号为W25Q32
 *            0XEF16型号为W25Q64
 *            0XEF17型号为W25Q128
 *            0XEF18型号为W25Q256
 *
 * @return  读出的字节
 */
uint16_t W25QXX_ReadID(void) {
  uint16_t Temp = 0;

  W25QXX_CS(0);
  SPI1_ReadWriteByte(0x90);  // 发送读取ID命令
  SPI1_ReadWriteByte(0x00);
  SPI1_ReadWriteByte(0x00);
  SPI1_ReadWriteByte(0x00);
  Temp |= SPI1_ReadWriteByte(0xFF) << 8;
  Temp |= SPI1_ReadWriteByte(0xFF);
  W25QXX_CS(1);

  return Temp;
}

/*
 * @brief   读取W25QXX的状态寄存器
 *
 * @note    W25QXX一共有3个状态寄存器
 *            状态寄存器1：BIT7 6  5  4   3   2   1   0
 *                        SPR  RV TB BP2 BP1 BP0 WEL BUSY
 *                        SPR:            默认0,状态寄存器保护位,配合WP使用
 *                        TB,BP2,BP1,BP0: FLASH区域写保护设置
 *                        WEL:            写使能锁定
 *                        BUSY:           忙标记位(1,忙;0,空闲)
 *                        默认:           0x00
 *            状态寄存器2：BIT7 6   5   4   3   2   1  0
 *                        SUS  CMP LB3 LB2 LB1 (R) QE SRP1
 *            状态寄存器3：BIT7     6    5     4   3  2    1   0
 *                        HOLD/RST DRV1 DRV0 (R) (R) WPS (R) (R)
 *
 * @paramregno: 状态寄存器号。范:1~3
 *
 * @return      状态寄存器值
 */
uint8_t W25QXX_ReadSR(uint8_t regno) {
  uint8_t byte = 0, command = 0;
  switch (regno) {
    case 1:
      command = W25X_ReadStatusReg1;
      break;
    case 2:
      command = W25X_ReadStatusReg2;
      break;
    case 3:
      command = W25X_ReadStatusReg3;
      break;
    default:
      command = W25X_ReadStatusReg1;
      break;
  }
  W25QXX_CS(0);
  SPI1_ReadWriteByte(command);
  byte = SPI1_ReadWriteByte(0Xff);
  W25QXX_CS(1);

  return byte;
}

/*
 * @brief   写W25QXX状态寄存器
 *
 * @note    W25QXX一共有3个状态寄存器
 *            状态寄存器1：BIT7 6  5  4   3   2   1   0
 *                        SPR  RV TB BP2 BP1 BP0 WEL BUSY
 *                        SPR:            默认0,状态寄存器保护位,配合WP使用
 *                        TB,BP2,BP1,BP0: FLASH区域写保护设置
 *                        WEL:            写使能锁定
 *                        BUSY:           忙标记位(1,忙;0,空闲)
 *                        默认:           0x00
 *            状态寄存器2：BIT7 6   5   4   3   2   1  0
 *                        SUS  CMP LB3 LB2 LB1 (R) QE SRP1
 *            状态寄存器3：BIT7     6    5     4   3  2    1   0
 *                        HOLD/RST DRV1 DRV0 (R) (R) WPS (R) (R)
 *
 * @paramregno: 状态寄存器号。范:1~3
 *
 * @return      状态寄存器值
 */
void W25QXX_Write_SR(uint8_t regno, uint8_t sr) {
  uint8_t command = 0;
  switch (regno) {
    case 1:
      command = W25X_WriteStatusReg1;
      break;
    case 2:
      command = W25X_WriteStatusReg2;
      break;
    case 3:
      command = W25X_WriteStatusReg3;
      break;
    default:
      command = W25X_WriteStatusReg1;
      break;
  }
  W25QXX_CS(0);
  SPI1_ReadWriteByte(command);
  SPI1_ReadWriteByte(sr);
  W25QXX_CS(1);
}

/*
 * @brief W25QXX写使能 将WEL置位
 */
void W25QXX_Write_Enable(void) {
  W25QXX_CS(0);
  SPI1_ReadWriteByte(W25X_WriteEnable);
  W25QXX_CS(1);
}

/*
 * @brief W25QXX写禁止 将WEL清零
 */
void W25QXX_Write_Disable(void) {
  W25QXX_CS(0);
  SPI1_ReadWriteByte(W25X_WriteDisable);
  W25QXX_CS(1);
}

/*
 * @brief   初始化SPI FLASH的IO口
 *
 * @return  0：识别成功。1：识别失败
 */
uint8_t W25QXX_Init(void) {
  uint8_t temp;

  W25QXX_CS(1);

  W25QXX_TYPE = W25QXX_ReadID();
  // SPI FLASH为W25Q256时才用设置为4字节地址模式
  if (W25QXX_TYPE == W25Q256) {
    // 读取状态寄存器3，判断地址模式
    temp = W25QXX_ReadSR(3);
    // 如果不是4字节地址模式,则进入4字节地址模式
    if ((temp & 0x01) == 0) {
      W25QXX_CS(0);
      // 发送进入4字节地址模式指令
      SPI1_ReadWriteByte(W25X_Enable4ByteAddr);
      W25QXX_CS(1);
    }
  }
  if (W25QXX_TYPE == W25Q256 || W25QXX_TYPE == W25Q128 || W25QXX_TYPE == W25Q64 || W25QXX_TYPE == W25Q32 || W25QXX_TYPE == W25Q16 || W25QXX_TYPE == W25Q80) {
    printf("Deceive_ID is %X\n", W25QXX_TYPE);
    return 0;
  } else {
    printf("Read Deceive_ID fail\n");
    return 1;
  }
}

/*
 * @brief 读取SPI FLASH。
 *
 * @note  在指定地址开始读取指定长度的数据。
 *
 * @param pBuffer       数据存储区
 * @param ReadAddr      开始读取的地址(24bit)
 * @param NumByteToRead 要读取的字节数(最大65535)
 *
 */
void W25QXX_Read(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead) {
  uint16_t i;
  W25QXX_CS(0);
  SPI1_ReadWriteByte(W25X_ReadData);
  if (W25QXX_TYPE == W25Q256) {
    // 如果是W25Q256的话地址为4字节的，要发送最高8位
    SPI1_ReadWriteByte((uint8_t)((ReadAddr) >> 24));
  }
  SPI1_ReadWriteByte((uint8_t)((ReadAddr) >> 16));  // 发送24bit地址
  SPI1_ReadWriteByte((uint8_t)((ReadAddr) >> 8));
  SPI1_ReadWriteByte((uint8_t)ReadAddr);
  for (i = 0; i < NumByteToRead; i++) {
    pBuffer[i] = SPI1_ReadWriteByte(0XFF);  // 循环读数
  }
  W25QXX_CS(1);
}

/*
 * @brief 等待空闲
 */
void W25QXX_Wait_Busy(void) {
  while ((W25QXX_ReadSR(1) & 0x01) == 0x01) {
    ;
  }
}

/*
 * @brief SPI在一页(0~65535)内写入少于256个字节的数据
 *
 * @note  在指定地址开始写入最大256字节的数据
 *
 * @param pBuffer         数据存储区
 * @param WriteAddr       开始写入的地址(24bit)
 * @param NumByteToWrite  要写入的字节数(最大256),该数不应该超过该页的剩余字节数!!!
 *
 */
void W25QXX_Write_Page(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite) {
  uint16_t i;
  W25QXX_Write_Enable();
  W25QXX_CS(0);
  SPI1_ReadWriteByte(W25X_PageProgram);  // 发送写页命令
  if (W25QXX_TYPE == W25Q256) {          // 如果是W25Q256的话地址为4字节的，要发送最高8位
    SPI1_ReadWriteByte((uint8_t)((WriteAddr) >> 24));
  }
  SPI1_ReadWriteByte((uint8_t)((WriteAddr) >> 16));  // 发送24bit地址
  SPI1_ReadWriteByte((uint8_t)((WriteAddr) >> 8));
  SPI1_ReadWriteByte((uint8_t)WriteAddr);
  for (i = 0; i < NumByteToWrite; i++) {
    SPI1_ReadWriteByte(pBuffer[i]);
  }
  W25QXX_CS(1);
  W25QXX_Wait_Busy();
}

/*
 * @brief 无检验写SPI FLASH
 *
 * @note  必须确保所写的地址范围内的数据全部为0XFF,否则在非0XFF处写入的数据将失败!
 *        具有自动换页功能。在指定地址开始写入指定长度的数据,但是要确保地址不越界!
 *
 * @param pBuffer         数据存储区
 * @param WriteAddr       开始写入的地址(24bit)
 * @param NumByteToWrite  要写入的字节数(最大65535)
 *
 */
void W25QXX_Write_NoCheck(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite) {
  uint16_t pageremain;
  // 计算单页剩余的字节数
  pageremain = 256 - WriteAddr % 256;
  if (NumByteToWrite <= pageremain)
    pageremain = NumByteToWrite;  // 不大于256个字节
  while (1) {
    W25QXX_Write_Page(pBuffer, WriteAddr, pageremain);
    if (NumByteToWrite == pageremain) {
      break;
    } else {
      pBuffer += pageremain;
      WriteAddr += pageremain;
      NumByteToWrite -= pageremain;  // 减去已经写入了的字节数
      if (NumByteToWrite > 256) {
        pageremain = 256;  // 一次可以写入256个字节
      } else {
        pageremain = NumByteToWrite;  // 不够256个字节了
      }
    }
  }
}

/*
 * @brief 写SPI FLASH
 *
 * @note  在指定地址开始写入指定长度的数据。相比于上面的函数，该函数带擦除操作!
 *
 * @param pBuffer         数据存储区
 * @param WriteAddr       开始写入的地址(24bit)
 * @param NumByteToWrite  要写入的字节数(最大65535)
 *
 */
uint8_t W25QXX_BUFFER[4096];
void W25QXX_Write(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite) {
  uint32_t secpos;
  uint16_t secoff;
  uint16_t secremain;
  uint16_t i;
  uint8_t* W25QXX_BUF;
  W25QXX_BUF = W25QXX_BUFFER;
  secpos = WriteAddr / 4096;  // 扇区地址
  secoff = WriteAddr % 4096;  // 在扇区内的偏移
  secremain = 4096 - secoff;  // 扇区剩余空间大小
  if (NumByteToWrite <= secremain) {
    secremain = NumByteToWrite;  // 不大于4096个字节
  }
  while (1) {
    W25QXX_Read(W25QXX_BUF, secpos * 4096, 4096);  // 读出整个扇区的内容
    for (i = 0; i < secremain; i++) {              // 校验数据
      if (W25QXX_BUF[secoff + i] != 0XFF) {
        break;  // 需要擦除
      }
    }
    if (i < secremain) {                 // 需要擦除
      W25QXX_Erase_Sector(secpos);       // 擦除这个扇区
      for (i = 0; i < secremain; i++) {  // 复制
        W25QXX_BUF[i + secoff] = pBuffer[i];
      }
      W25QXX_Write_NoCheck(W25QXX_BUF, secpos * 4096, 4096);  // 写入整个扇区
    } else {
      W25QXX_Write_NoCheck(pBuffer, WriteAddr, secremain);  // 写已经擦除了的,直接写入扇区剩余区间.
    }
    if (NumByteToWrite == secremain) {
      break;                        // 写入结束了
    } else {                        // 写入未结束
      secpos++;                     // 扇区地址增1
      secoff = 0;                   // 偏移位置为0
      pBuffer += secremain;         // 指针偏移
      WriteAddr += secremain;       // 写地址偏移
      NumByteToWrite -= secremain;  // 字节数递减
      if (NumByteToWrite > 4096) {
        secremain = 4096;  // 下一个扇区还是写不完
      } else {
        secremain = NumByteToWrite;  // 下一个扇区可以写完了
      }
    }
  }
}

/*
 * @brief 擦除整个芯片
 *
 * @note  等待时间超长...
 *
 */
void W25QXX_Erase_Chip(void) {
  W25QXX_Write_Enable();
  W25QXX_Wait_Busy();
  W25QXX_CS(0);
  SPI1_ReadWriteByte(W25X_ChipErase);
  W25QXX_CS(1);
  W25QXX_Wait_Busy();
}

/*
 * @brief 擦除一个扇区
 *
 * @note  擦除一个扇区的最少时间:150ms
 *
 * @param Dst_Addr  扇区地址 根据实际容量设置
 *
 */
void W25QXX_Erase_Sector(uint32_t Dst_Addr) {
  Dst_Addr *= 4096;
  W25QXX_Write_Enable();
  W25QXX_Wait_Busy();
  W25QXX_CS(0);
  SPI1_ReadWriteByte(W25X_SectorErase);
  if (W25QXX_TYPE == W25Q256) {
    SPI1_ReadWriteByte((uint8_t)((Dst_Addr) >> 24));
  }
  SPI1_ReadWriteByte((uint8_t)((Dst_Addr) >> 16));
  SPI1_ReadWriteByte((uint8_t)((Dst_Addr) >> 8));
  SPI1_ReadWriteByte((uint8_t)Dst_Addr);
  W25QXX_CS(1);
  W25QXX_Wait_Busy();
}
