#ifndef __W25QXX_H__
#define __W25QXX_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 25系列FLASH芯片厂商与容量代号（厂商代号EF）
#define W25Q80 0XEF13
#define W25Q16 0XEF14
#define W25Q32 0XEF15
#define W25Q64 0XEF16
#define W25Q128 0XEF17
#define W25Q256 0XEF18
#define EX_FLASH_ADD 0x000000  // W25Q64的地址是24位宽

// ********************* 指令表 ************************* //
// 写使能 与 写禁止
#define W25X_WriteEnable 0x06
#define W25X_WriteDisable 0x04
// 读取状态寄存器123的命令
#define W25X_ReadStatusReg1 0x05
#define W25X_ReadStatusReg2 0x35
#define W25X_ReadStatusReg3 0x15
// 写状态寄存器123的命令
#define W25X_WriteStatusReg1 0x01
#define W25X_WriteStatusReg2 0x31
#define W25X_WriteStatusReg3 0x11
// 读取数据指令
#define W25X_ReadData 0x03
#define W25X_FastReadData 0x0B
#define W25X_FastReadDual 0x3B
#define W25X_PageProgram 0x02
#define W25X_BlockErase 0xD8
// 扇区擦除指令
#define W25X_SectorErase 0x20
// 片擦除命令
#define W25X_ChipErase 0xC7
#define W25X_PowerDown 0xB9
#define W25X_ReleasePowerDown 0xAB
#define W25X_DeviceID 0xAB
#define W25X_ManufactDeviceID 0x90
#define W25X_JedecDeviceID 0x9F
// 进入4字节地址模式指令
#define W25X_Enable4ByteAddr 0xB7
#define W25X_Exit4ByteAddr 0xE9

void W25QXX_CS(uint8_t a);                        // W25QXX片选引脚控制
uint8_t SPI1_ReadWriteByte(uint8_t TxData);       // SPI1总线底层读写
uint16_t W25QXX_ReadID(void);                     // 读取FLASH ID
uint8_t W25QXX_ReadSR(uint8_t regno);             // 读取状态寄存器
void W25QXX_Write_SR(uint8_t regno, uint8_t sr);  // 写状态寄存器
void W25QXX_Write_Enable(void);                   // 写使能
void W25QXX_Write_Disable(void);                  // 写保护
uint8_t W25QXX_Init(void);                        // 初始化W25QXX函数
void W25QXX_Wait_Busy(void);                      // 等待空闲
// 读取flash
void W25QXX_Read(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);
// 写入flash
void W25QXX_Write_Page(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void W25QXX_Write_NoCheck(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void W25QXX_Write(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
// 擦除flash
void W25QXX_Erase_Chip(void);                 // 整片擦除
void W25QXX_Erase_Sector(uint32_t Dst_Addr);  // 扇区擦除

#ifdef __cplusplus
}
#endif

#endif /* MYPROJECT_W25Q64_W25QXX_H_ */
