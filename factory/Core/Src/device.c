#include "device.h"

#include <stdio.h>
#include <stdint.h>

#include "main.h"

#define UART6_TX_RING_SIZE 1024
#define UART6_RX_RING_SIZE 1024
#define UART6_DMATX_BUF_SIZE 256
#define UART6_DMARX_BUF_SIZE 256

#define FRAME_HEAD1 0x55
#define FRAME_HEAD2 0xAA

typedef enum {
  PARSE_STAT_HEAD1 = 0,
  PARSE_STAT_HEAD2,
  PARSE_STAT_ID,
  PARSE_STAT_LENGTH,
  PARSE_STAT_DATA,
} FRAME_PARSE_STAT;

typedef struct {
  volatile uint16_t status; /* 发送状态 */
  uint16_t last_dmarx_size; /* dma上一次接收数据大小 */
  uint32_t tx_count;
  uint32_t rx_count;
} uart_device_t;

char _CCM_DATA print_buf[512];

static ring_def(uint8_t _CCM_DATA, uart6_tx_ring, UART6_TX_RING_SIZE, 1);
static ring_def(uint8_t _CCM_DATA, uart6_rx_ring, UART6_RX_RING_SIZE, 1);
static uint8_t uart6_dmatx_buf[UART6_DMATX_BUF_SIZE];
static uint8_t uart6_dmarx_buf[UART6_DMARX_BUF_SIZE];

static uart_device_t uart6_dev = {0};
static frame_parse_t rx_frame = {
  .status = PARSE_STAT_HEAD1,
};

void uart6_config(void) {
  /* USART6_RX DMA */
  LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_1, LL_DMA_CHANNEL_5);
  LL_DMA_ConfigTransfer(DMA2, LL_DMA_STREAM_1,
      LL_DMA_DIRECTION_PERIPH_TO_MEMORY |
          LL_DMA_PRIORITY_HIGH |
          LL_DMA_MODE_CIRCULAR |
          LL_DMA_PERIPH_NOINCREMENT |
          LL_DMA_MEMORY_INCREMENT |
          LL_DMA_PDATAALIGN_BYTE |
          LL_DMA_MDATAALIGN_BYTE);
  LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_1,
      LL_USART_DMA_GetRegAddr(USART6),
      (uint32_t)uart6_dmarx_buf,
      LL_DMA_GetDataTransferDirection(DMA2, LL_DMA_STREAM_1));
  LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_1, UART6_DMARX_BUF_SIZE);

  /* USART6_TX DMA */
  LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_7, LL_DMA_CHANNEL_5);
  LL_DMA_ConfigTransfer(DMA2, LL_DMA_STREAM_7,
      LL_DMA_DIRECTION_MEMORY_TO_PERIPH |
          LL_DMA_PRIORITY_HIGH |
          LL_DMA_MODE_NORMAL |
          LL_DMA_PERIPH_NOINCREMENT |
          LL_DMA_MEMORY_INCREMENT |
          LL_DMA_PDATAALIGN_BYTE |
          LL_DMA_MDATAALIGN_BYTE);
  LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_7,
      (uint32_t)uart6_dmatx_buf,
      LL_USART_DMA_GetRegAddr(USART6),
      LL_DMA_GetDataTransferDirection(DMA2, LL_DMA_STREAM_7));

  LL_DMA_ClearFlag_DME1(DMA2);
  LL_DMA_ClearFlag_DME7(DMA2);
  LL_DMA_ClearFlag_HT1(DMA2);
  LL_DMA_ClearFlag_TC1(DMA2);
  LL_DMA_ClearFlag_TC7(DMA2);
  LL_DMA_ClearFlag_TE1(DMA2);
  LL_DMA_ClearFlag_TE7(DMA2);

  LL_DMA_EnableIT_HT(DMA2, LL_DMA_STREAM_1);
  LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_1);
  LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_7);

  LL_USART_EnableDMAReq_RX(USART6);
  LL_USART_EnableDMAReq_TX(USART6);
  LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_1);

  LL_USART_EnableIT_IDLE(USART6);
}

/**
 * @brief  串口dma接收完成中断处理
 * @param
 * @retval
 */
void uart6_dmarx_done_isr(void) {
  uint16_t recv_size;

  recv_size = UART6_DMARX_BUF_SIZE - uart6_dev.last_dmarx_size;

  __disable_irq();
  ring_push_mult(&uart6_rx_ring, &uart6_dmarx_buf[uart6_dev.last_dmarx_size], recv_size);
  __enable_irq();

  uart6_dev.rx_count += recv_size;
  uart6_dev.last_dmarx_size = 0;
}

/**
 * @brief  串口dma接收部分数据中断处理
 * @param
 * @retval
 */
void uart6_dmarx_part_done_isr(void) {
  uint16_t recv_total_size;
  uint16_t recv_size;

  recv_total_size = UART6_DMARX_BUF_SIZE - LL_DMA_GetDataLength(DMA2, LL_DMA_STREAM_1);
  recv_size = recv_total_size - uart6_dev.last_dmarx_size;

  __disable_irq();
  ring_push_mult(&uart6_rx_ring, &uart6_dmarx_buf[uart6_dev.last_dmarx_size], recv_size);
  __enable_irq();

  uart6_dev.rx_count += recv_size;
  uart6_dev.last_dmarx_size = recv_total_size;
}

/**
 * @brief  串口dma发送完成中断处理
 * @param
 * @retval
 */
void uart6_dmatx_done_isr(void) {
  uart6_dev.status = 0; /* DMA发送空闲 */
}

void uart6_tx_poll(void) {
  uint16_t size = 0;

  if (uart6_dev.status) {
    return;
  }

  if (ring_is_empty(&uart6_tx_ring)) {
    return;
  }

  __disable_irq();
  size = ring_pop_mult(&uart6_tx_ring, &uart6_dmatx_buf, UART6_DMATX_BUF_SIZE);
  __enable_irq();

  uart6_dev.status = 1;
  uart6_dev.tx_count += size;

  LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_7);
  LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_7, size);
  LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_7);
}

uint16_t uart6_read(uint8_t *buf, uint16_t size) {
  uint16_t ok = 0;

  if (buf == NULL) {
    return 0;
  }

  if (ring_is_empty(&uart6_rx_ring)) {
    return 0;
  }

  __disable_irq();
  ok = ring_pop_mult(&uart6_rx_ring, buf, size);
  __enable_irq();

  return ok;
}

uint16_t uart6_write(const uint8_t *buf, uint16_t size) {
  uint16_t ok = 0;

  if (buf == NULL) {
    return 0;
  }

  __disable_irq();
  ok = ring_push_mult(&uart6_tx_ring, buf, size);
  __enable_irq();

  return ok;
}

void change_byte_order(uint8_t *addr, size_t size) {
  uint8_t tmp;
  size_t i, imax = size / 2;

  for (i = 0; i < imax; ++i) {
    tmp = addr[i];
    addr[i] = addr[size - i - 1];
    addr[size - i - 1] = tmp;
  }
}

void uart6_frame_parse(void *func) {
  uint16_t size = 0;
  uint8_t rx;

  switch (rx_frame.status) {
    case PARSE_STAT_HEAD1: {
      size = uart6_read(&rx, 1);
      if (size && rx == FRAME_HEAD1) {
        rx_frame.status = PARSE_STAT_HEAD2;
      }
    } break;
    case PARSE_STAT_HEAD2: {
      size = uart6_read(&rx, 1);
      if (size) {
        if (rx == FRAME_HEAD2) {
          rx_frame.status = PARSE_STAT_ID;
        } else {
          rx_frame.status = PARSE_STAT_HEAD1;
        }
      }
    } break;
    case PARSE_STAT_ID: {
      size = uart6_read(&rx_frame.id, 1);
      if (size) {
        if (rx_frame.id == 0xff) {
          rx_frame.byte_order = 1;
          rx_frame.status= PARSE_STAT_HEAD1;
        } else if (rx_frame.id == 0xfe) {
          rx_frame.byte_order = 0;
          rx_frame.status = PARSE_STAT_HEAD1;
        } else if (rx_frame.id < FRAME_TYPE_MAX) {
          rx_frame.status = PARSE_STAT_LENGTH;
        } else {
          rx_frame.status= PARSE_STAT_HEAD1;
        }
      }
    } break;
    case PARSE_STAT_LENGTH: {
      size = uart6_read((uint8_t *)&rx_frame.length + rx_frame.recv_size, sizeof(rx_frame.length) - rx_frame.recv_size);
      if (size) {
        rx_frame.recv_size += size;
      }
      if (rx_frame.recv_size >= sizeof(rx_frame.length)) {
        if (rx_frame.byte_order) {
          change_byte_order((uint8_t *)&rx_frame.length, sizeof(rx_frame.length));
        }
        if (rx_frame.length > FRAME_DATA_LEN_MAX) {
          printf("frame length error!!! (%hu)\n", rx_frame.length);
          rx_frame.status = PARSE_STAT_HEAD1;
        } else {
          rx_frame.status = PARSE_STAT_DATA;
        }
        rx_frame.recv_size = 0;
      }
    } break;
    case PARSE_STAT_DATA: {
      size = uart6_read(rx_frame.data + rx_frame.recv_size, rx_frame.length - rx_frame.recv_size);
      if (size) {
        rx_frame.recv_size += size;
      }
      if (rx_frame.recv_size >= rx_frame.length) {
        rx_frame.status = PARSE_STAT_HEAD1;
        rx_frame.recv_size = 0;
        if (func) {
          ((void (*)(frame_parse_t *))func)(&rx_frame);
        }
      }
    } break;
    default: {
      printf("frame status error!!!\n");
    } break;
  }
}

void print_frame(frame_parse_t *frame) {
  printf("id: %hhu\n", frame->id);
  uart6_printf("length: %hu\n", frame->length);
  printf("data:");
  for (uint16_t i = 0; i < frame->length; ++i) {
    printf(" 0x%02x", frame->data[i]);
  }
  printf("\n");
}

void print_uart6_tx_rx(void) {
  printf("uart6_tx: %lu\n", uart6_dev.tx_count);
  printf("uart6_rx: %lu\n", uart6_dev.rx_count);
}
