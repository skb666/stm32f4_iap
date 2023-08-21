#ifndef __RING_FIFO_H__
#define __RING_FIFO_H__

#include <stdint.h>
#include <stdlib.h>

#define _CCM_DATA __attribute__((section(".ccmram.data")))

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  void *buffer;
  const uint16_t capacity;
  const uint16_t element_size;
  const uint16_t cover;
  int16_t head;
  int16_t tail;
  int16_t size;
} RING_FIFO;

#define ring_def(Type, BufName, Size, Cover)         \
  Type __##BufName##_data[Size];                     \
  RING_FIFO BufName = {                              \
      .buffer = __##BufName##_data,                  \
      .capacity = Size,                              \
      .element_size = sizeof(__##BufName##_data[0]), \
      .cover = Cover,                                \
      .head = 0,                                     \
      .tail = 0,                                     \
      .size = 0,                                     \
  }

int8_t ring_push(RING_FIFO *ring, const void *element);
int8_t ring_pop(RING_FIFO *ring, void *element);
uint16_t ring_push_mult(RING_FIFO *ring, const void *elements, uint16_t num);
uint16_t ring_pop_mult(RING_FIFO *ring, void *elements, uint16_t num);
void ring_reset(RING_FIFO *ring);
int8_t ring_is_empty(RING_FIFO *ring);
int8_t ring_is_full(RING_FIFO *ring);
int16_t ring_size(RING_FIFO *ring);
void print_ring(RING_FIFO *ring);

#ifdef __cplusplus
}
#endif

#endif
