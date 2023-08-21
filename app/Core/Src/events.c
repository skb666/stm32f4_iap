#include "events.h"

#include <stdio.h>

#include "main.h"
#include "ring_fifo.h"
#include "device.h"

static const char *key_name[] = {KEY_ENUM(ENUM_STRING)};
static ring_def(EVENT _CCM_DATA, g_events, 100, 1);

int8_t event_put(EVENT *ev) {
  int8_t err = 0;

  __disable_irq();
  err = ring_push(&g_events, ev);
  __enable_irq();

  return err;
}

int8_t event_get(EVENT *ev) {
  int8_t err = 0;

  __disable_irq();
  err = ring_pop(&g_events, ev);
  __enable_irq();

  return err;
}

int16_t event_count() {
  return ring_size(&g_events);
}

int8_t event_empty() {
  return ring_is_empty(&g_events);
}

uint16_t event_poll() {
  int8_t err = 0;
  EVENT ev;
  uint16_t ok = 0;
  while (event_count()) {
    err = event_get(&ev);
    if (err) {
      printf("get event error!!!\n");
    } else {
      ok += 1;
    }
    switch (ev.type) {
      case EV_TIM: {
        tim_event_proc(&ev);
      } break;
      case EV_KEY: {
        key_event_proc(&ev);
      } break;
      default:
        break;
    }
  }
  return ok;
}

void tim_event_proc(EVENT *ev) {
  LL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
  LL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
}

void key_event_proc(EVENT *ev) {
  size_t id = (size_t)ev->data;
  switch (ev->sub_type) {
    case KE_PRESS: {
      printf("[%s]: PRESS\n", key_name[id]);
    } break;
    case KE_RELEASE: {
      printf("[%s]: RELEASE\n", key_name[id]);
    } break;
    case KE_LONG_PRESS: {
      printf("[%s]: LONG_PRESS\n", key_name[id]);
    } break;
    case KE_LONG_RELEASE: {
      printf("[%s]: LONG_RELEASE\n", key_name[id]);
    } break;
    default:
      break;
  }
}
