#ifndef __INTERFACE_H__
#define __INTERFACE_H__

#include <stdint.h>

#include "device.h"
#include "onchip_flash.h"

#define ADDR_BOOTLOADER ADDR_FLASH_SECTOR_0
#define ADDR_BOOT_PARAM ADDR_FLASH_SECTOR_2
#define ADDR_BOOT_PARAM_BAK ADDR_FLASH_SECTOR_3
#define ADDR_APP_FACTORY ADDR_FLASH_SECTOR_5
#define ADDR_APP_RUN ADDR_FLASH_SECTOR_6
#define ADDR_APP_APP1 ADDR_FLASH_SECTOR_8
#define ADDR_APP_APP2 ADDR_FLASH_SECTOR_10

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  STATUS_NORMAL = 0,   // APP能正常稳定运行
  STATUS_UPDATED,  // APP刚更新完成，等待测试启动
  STATUS_ERROR,    // APP错误，不能正常工作
} APP_STATUS;

typedef enum {
  BOOT_FACTORY = 0,
  BOOT_APP1,
  BOOT_APP2,
} APP_BOOT;

typedef enum {
  APP_NONE = 0,
  APP_APP1,
  APP_APP2,
} APP_RUN;

typedef struct {
  uint8_t app_boot;
  uint8_t app_run;
  uint8_t app_status[2];
  uint32_t crc_val;
} BOOT_PARAM;

void boot_param_read_check(BOOT_PARAM *pdata);
uint32_t select_boot_addr(BOOT_PARAM *param);
void start_boot_app(uint32_t boot_addr);
void boot_param_check_upgrade(void);
void iap_update(frame_parse_t *frame);
void boot_test(void);

#ifdef __cplusplus
}
#endif

#endif
