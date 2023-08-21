#include "ring_fifo.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int8_t ring_push(RING_FIFO *ring, const void *element) {
  uint8_t *pbuf = NULL;

  if (ring == NULL || element == NULL) {
    return -1;
  }

  /* 已满且不支持覆盖 */
  if (ring->size >= ring->capacity && !ring->cover) {
    return -1;
  }

  pbuf = (uint8_t *)ring->buffer + ring->tail * ring->element_size;
  memcpy(pbuf, element, ring->element_size);
  ring->tail = (ring->tail + 1) % ring->capacity;

  /* 已满但支持覆盖 */
  if (ring->size >= ring->capacity && ring->cover) {
    ring->head = (ring->head + 1) % ring->capacity;
  } else {
    ring->size += 1;
  }

  return 0;
}

int8_t ring_pop(RING_FIFO *ring, void *element) {
  uint8_t *pbuf = NULL;

  if (ring == NULL || element == NULL || ring->size == 0) {
    return -1;
  }

  pbuf = (uint8_t *)ring->buffer + ring->head * ring->element_size;
  memcpy(element, pbuf, ring->element_size);
  ring->head = (ring->head + 1) % ring->capacity;

  ring->size -= 1;

  return 0;
}

uint16_t ring_push_mult(RING_FIFO *ring, const void *elements, uint16_t num) {
  uint8_t *inbuf = NULL;
  uint8_t *outbuf = NULL;
  uint16_t cnt = 0, remain = 0;
  uint16_t tmp = 0;

  if (ring == NULL || elements == NULL || num == 0) {
    return 0;
  }

  /* 已满且不支持覆盖 */
  if (ring->size >= ring->capacity && !ring->cover) {
    return 0;
  }

  if (ring->size + num <= ring->capacity) {
    /* 放得下则全放 */
    cnt = num;
  } else {
    /* 放到填满为止 */
    cnt = ring->capacity - ring->size;
  }

  /* 未满时放入，直到填满为止 */
  if (cnt) {
    inbuf = (uint8_t *)elements;
    outbuf = (uint8_t *)ring->buffer + ring->tail * ring->element_size;
    if (ring->tail + cnt <= ring->capacity) {
      /* 放的时候不会够到数组边界 */
      memcpy(outbuf, inbuf, cnt * ring->element_size);
    } else {
      /* 放的时候会够到数组边界，分两次 */
      tmp = ring->capacity - ring->tail;
      memcpy(outbuf, inbuf, tmp * ring->element_size);
      inbuf += tmp * ring->element_size;
      outbuf = (uint8_t *)ring->buffer;
      memcpy(outbuf, inbuf, (cnt - tmp) * ring->element_size);
    }
    ring->tail = (ring->tail + cnt) % ring->capacity;
    ring->size += cnt;
  }

  /* 支持覆盖 */
  if (ring->cover) {
    remain = num - cnt;
    if (remain == 0) {
      /* 没有东西要放了 */
      return cnt;
    } else if (remain < ring->capacity) {
      /* 在放满后还需要部分覆盖 */
      inbuf = (uint8_t *)elements + cnt * ring->element_size;
      outbuf = (uint8_t *)ring->buffer + ring->head * ring->element_size;
      if (ring->head + remain <= ring->capacity) {
        /* 覆盖时不会够到数组边界 */
        memcpy(outbuf, inbuf, remain * ring->element_size);
      } else {
        /* 覆盖时会够到数组边界，分两次 */
        tmp = ring->capacity - ring->head;
        memcpy(outbuf, inbuf, tmp * ring->element_size);
        inbuf += tmp * ring->element_size;
        outbuf = (uint8_t *)ring->buffer;
        memcpy(outbuf, inbuf, (remain - tmp) * ring->element_size);
      }
      ring->head = (ring->head + remain) % ring->capacity;
      ring->tail = ring->head;
    } else {
      /* 在放满后需要全覆盖 */
      inbuf = (uint8_t *)elements + (num - ring->capacity) * ring->element_size;
      outbuf = (uint8_t *)ring->buffer;
      memcpy(outbuf, inbuf, ring->capacity * ring->element_size);
      ring->head = 0;
      ring->tail = 0;
    }
    return num;
  }

  return cnt;
}

uint16_t ring_pop_mult(RING_FIFO *ring, void *elements, uint16_t num) {
  uint8_t *inbuf = NULL;
  uint8_t *outbuf = NULL;
  uint16_t cnt = 0;
  uint16_t tmp = 0;

  if (ring == NULL || elements == NULL || num == 0 || ring->size == 0) {
    return 0;
  }

  if (num <= ring->size) {
    /* 取的数量不超过有的数量 */
    cnt = num;
  } else {
    /* 取得数量过大时，有几个取几个 */
    cnt = ring->size;
  }

  inbuf = (uint8_t *)ring->buffer + ring->head * ring->element_size;
  outbuf = (uint8_t *)elements;
  if (ring->head + cnt <= ring->capacity) {
    /* 取的时候不会够到数组边界 */
    memcpy(outbuf, inbuf, cnt * ring->element_size);
  } else {
    /* 取的时候会够到数组边界，分两次 */
    tmp = ring->capacity - ring->head;
    memcpy(outbuf, inbuf, tmp * ring->element_size);
    inbuf = (uint8_t *)ring->buffer;
    outbuf += tmp * ring->element_size;
    memcpy(outbuf, inbuf, (cnt - tmp) * ring->element_size);
  }

  ring->head = (ring->head + cnt) % ring->capacity;
  ring->size -= cnt;

  return cnt;
}

void ring_reset(RING_FIFO *ring) {
  ring->head = 0;
  ring->tail = 0;
  ring->size = 0;
}

int8_t ring_is_empty(RING_FIFO *ring) {
  return (ring->size == 0);
}

int8_t ring_is_full(RING_FIFO *ring) {
  return (ring->size >= ring->capacity);
}

int16_t ring_size(RING_FIFO *ring) {
  return ring->size;
}

void print_ring(RING_FIFO *ring) {
  printf("========== ring ==========\n");
  printf("cover: %hu\n", ring->cover);
  printf("element_size: %hu\n", ring->element_size);
  printf("capacity: %hu\n", ring->capacity);
  printf("head: %hd\n", ring->head);
  printf("tail: %hd\n", ring->tail);
  printf("size: %hd\n", ring->size);
  printf("--------------------------\n");
}
