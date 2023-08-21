#ifndef __EVENTS_H__
#define __EVENTS_H__

#include <stdint.h>

#include "key.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  EV_TIM,
  EV_KEY,
  EV_NONE,
} EVENT_TYPE;

typedef struct {
  int type;
  int sub_type;
  void *data;
} EVENT;

int8_t event_put(EVENT *ev);
int8_t event_get(EVENT *ev);
int16_t event_count();
int8_t event_empty();

uint16_t event_poll();

void tim_event_proc(EVENT *ev);
void key_event_proc(EVENT *ev);

#ifdef __cplusplus
}
#endif

#endif
