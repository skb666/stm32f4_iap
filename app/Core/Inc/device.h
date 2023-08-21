#ifndef __DEVICE_H__
#define __DEVICE_H__

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ring_fifo.h"

#define FRAME_DATA_LEN_MAX 1024

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  FRAME_TYPE_DATA = 0,
  FRAME_TYPE_BEGIN,
  FRAME_TYPE_END,
  FRAME_TYPE_MAX,
} FRAME_TYPE;

typedef struct {
  uint8_t status;
  uint8_t id;
  uint8_t byte_order;
  uint16_t length;
  uint16_t recv_size;
  uint8_t data[FRAME_DATA_LEN_MAX];
} frame_parse_t;

extern char print_buf[];

void uart6_config(void);
void uart6_dmarx_done_isr(void);
void uart6_dmarx_part_done_isr(void);
void uart6_dmatx_done_isr(void);

void uart6_tx_poll(void);
uint16_t uart6_read(uint8_t *buf, uint16_t size);
uint16_t uart6_write(const uint8_t *buf, uint16_t size);
void change_byte_order(uint8_t *addr, size_t size);
int8_t frame_parse_register(void (*func)(frame_parse_t *));
void uart6_frame_parse(void);
void print_frame(frame_parse_t *frame);
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
