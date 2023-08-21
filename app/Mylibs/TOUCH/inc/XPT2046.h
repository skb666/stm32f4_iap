#ifndef __XPT2046_H__
#define __XPT2046_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint16_t x;
  uint16_t y;
} POINT;

void XPT2046_SetFlag(uint8_t v);
uint8_t XPT2046_GetFlag();

void Touch_Init(uint8_t flag);
int Touch_Calibrate(uint8_t flag);
POINT *Read_2046(void);
POINT *Read_2046_2(void);
int8_t Get_touch_point(POINT *displayPtr, POINT *screenPtr);

void Palette_Init(void);
void Palette_draw_point(uint16_t x, uint16_t y);

#ifdef __cplusplus
}
#endif

#endif
