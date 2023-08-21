#include "interface.h"

#include "crc.h"
#include "device.h"
#include "main.h"

#define APP_DATALEN (2 * 128 * 1024)
#define BIN_BUF_DATALEN (8 * 1024)

typedef int (*pFunction)(void);
__IO uint32_t JumpAddress;
pFunction JumpToApplication;

const uint32_t boot_param_crcdatalen = sizeof(BOOT_PARAM) / 4 - 1;
const BOOT_PARAM boot_param_default = {
    .app_boot = BOOT_FACTORY,
    .app_run = APP_NONE,
    .app_status = {
        [0] = STATUS_ERROR,
        [1] = STATUS_ERROR,
    },
    .crc_val = 0x78afacac,
};

static uint32_t _CCM_DATA bin_buf[BIN_BUF_DATALEN];

static uint32_t param_crc_calc(const BOOT_PARAM *param) {
  uint32_t crc = 0;

  crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)param, boot_param_crcdatalen);

  return crc;
}

static void boot_param_update(uint32_t addr, BOOT_PARAM *param) {
  param->crc_val = param_crc_calc(param);
  STMFLASH_Write(addr, (uint32_t *)param, boot_param_crcdatalen + 1);
}

void boot_param_read_check(BOOT_PARAM *pdata) {
  BOOT_PARAM param, param_bak;

  STMFLASH_Read(ADDR_BOOT_PARAM, (uint32_t *)&param, boot_param_crcdatalen + 1);
  STMFLASH_Read(ADDR_BOOT_PARAM_BAK, (uint32_t *)&param_bak, boot_param_crcdatalen + 1);

  if (param_crc_calc(&param) == param.crc_val) {
    uart6_printf("boot param checked Ok\n");
    if (param_crc_calc(&param_bak) == param_bak.crc_val) {
      uart6_printf("boot param backup checked Ok\n");
      if (memcmp(&param, &param_bak, sizeof(BOOT_PARAM)) != 0) {
        uart6_printf("boot param main sector and backup sector data are different, update bakup sector data\n");
        STMFLASH_Write(ADDR_BOOT_PARAM_BAK, (uint32_t *)&param, boot_param_crcdatalen + 1);
      } else {
        uart6_printf("boot param main sector and backup sector data are the same\n");
      }
    } else {
      uart6_printf("boot param backup checked Fail, update backup sector data\n");
      STMFLASH_Write(ADDR_BOOT_PARAM_BAK, (uint32_t *)&param, boot_param_crcdatalen + 1);
    }
    memcpy(pdata, &param, sizeof(BOOT_PARAM));
  } else {
    uart6_printf("boot param checked Fail\n");
    if (param_crc_calc(&param_bak) == param_bak.crc_val) {
      uart6_printf("boot param backup checked Ok\n");
      uart6_printf("update main sector data\n");
      STMFLASH_Write(ADDR_BOOT_PARAM, (uint32_t *)&param_bak, boot_param_crcdatalen + 1);
      memcpy(pdata, &param_bak, sizeof(BOOT_PARAM));
    } else {
      uart6_printf("boot param backup checked Fail\n");
      uart6_printf("restore defaults\n");
      STMFLASH_Write(ADDR_BOOT_PARAM, (uint32_t *)&boot_param_default, boot_param_crcdatalen + 1);
      STMFLASH_Write(ADDR_BOOT_PARAM_BAK, (uint32_t *)&boot_param_default, boot_param_crcdatalen + 1);
      memcpy(pdata, &boot_param_default, sizeof(BOOT_PARAM));
    }
  }
}

static void load_app(BOOT_PARAM *param) {
  uint32_t read_count = 0;
  uint32_t load_addr = 0;

  if (param->app_boot == BOOT_APP1) {
    load_addr = ADDR_APP_APP1;
  } else if (param->app_boot == BOOT_APP2) {
    load_addr = ADDR_APP_APP2;
  }

  while (read_count < APP_DATALEN) {
    STMFLASH_Read(load_addr + read_count, bin_buf, BIN_BUF_DATALEN);
    STMFLASH_Write(ADDR_APP_RUN + read_count, bin_buf, BIN_BUF_DATALEN);
    read_count += BIN_BUF_DATALEN * 4;
  }

  param->app_run = param->app_boot;
}

uint32_t select_boot_addr(BOOT_PARAM *param) {
  uint32_t boot_addr = ADDR_APP_FACTORY;

  if (param->app_boot == BOOT_FACTORY) {
    boot_addr = ADDR_APP_FACTORY;
  } else if (param->app_boot == BOOT_APP1) {
    if (param->app_boot != param->app_run) {
      param->app_run = APP_APP1;
      param->app_status[0] = STATUS_UPDATED;
      boot_addr = ADDR_APP_RUN;
      load_app(param);
      boot_param_update(ADDR_BOOT_PARAM, param);
      boot_param_update(ADDR_BOOT_PARAM_BAK, param);
    } else {
      if (param->app_status[0] == STATUS_NORMAL) {
        boot_addr = ADDR_APP_RUN;
      } else {
        if (param->app_status[0] == STATUS_UPDATED) {
          param->app_status[0] = STATUS_ERROR;
        }
        if (param->app_status[1] == STATUS_NORMAL) {
          param->app_boot = BOOT_APP2;
          param->app_run = APP_APP2;
          boot_addr = ADDR_APP_RUN;
          load_app(param);
        } else {
          param->app_boot = BOOT_FACTORY;
          param->app_run = APP_NONE;
          boot_addr = ADDR_APP_FACTORY;
        }
        boot_param_update(ADDR_BOOT_PARAM, param);
        boot_param_update(ADDR_BOOT_PARAM_BAK, param);
      }
    }
  } else if (param->app_boot == BOOT_APP2) {
    if (param->app_boot != param->app_run) {
      param->app_run = APP_APP2;
      param->app_status[1] = STATUS_UPDATED;
      boot_addr = ADDR_APP_RUN;
      load_app(param);
      boot_param_update(ADDR_BOOT_PARAM, param);
      boot_param_update(ADDR_BOOT_PARAM_BAK, param);
    } else {
      if (param->app_status[1] == STATUS_NORMAL) {
        boot_addr = ADDR_APP_RUN;
      } else {
        if (param->app_status[1] == STATUS_UPDATED) {
          param->app_status[1] = STATUS_ERROR;
        }
        if (param->app_status[0] == STATUS_NORMAL) {
          param->app_boot = BOOT_APP1;
          param->app_run = APP_APP1;
          boot_addr = ADDR_APP_RUN;
          load_app(param);
        } else {
          param->app_boot = BOOT_FACTORY;
          param->app_run = APP_NONE;
          boot_addr = ADDR_APP_FACTORY;
        }
        boot_param_update(ADDR_BOOT_PARAM, param);
        boot_param_update(ADDR_BOOT_PARAM_BAK, param);
      }
    }
  }
  return boot_addr;
}

inline __attribute__((always_inline)) void start_boot_app(uint32_t boot_addr) {
  JumpAddress = *(__IO uint32_t *)(boot_addr + 4);
  JumpToApplication = (pFunction)JumpAddress;
  __set_MSP(*(__IO uint32_t *)(JumpAddress - 4));
  JumpToApplication();
}

void boot_test(void) {
  ;
}
