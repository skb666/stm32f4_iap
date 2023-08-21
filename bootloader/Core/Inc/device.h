#ifndef __DEVICE_H__
#define __DEVICE_H__

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ring_fifo.h"

#ifdef __cplusplus
extern "C" {
#endif

extern char print_buf[];

void uart6_config(void);
void uart6_dmarx_done_isr(void);
void uart6_dmarx_part_done_isr(void);
void uart6_dmatx_done_isr(void);

void uart6_tx_poll(void);
uint16_t uart6_read(uint8_t *buf, uint16_t size);
uint16_t uart6_write(const uint8_t *buf, uint16_t size);
void print_uart6_tx_rx(void);

#define uart6_printf(fmt, args...)                        \
  do {                                                    \
    sprintf((char *)print_buf, fmt, ##args);              \
    uart6_write((uint8_t *)print_buf, strlen(print_buf)); \
    LL_mDelay(2);                                         \
  } while (0)

#ifdef __cplusplus
}
#endif

#endif
