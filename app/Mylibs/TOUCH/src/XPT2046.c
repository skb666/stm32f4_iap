#include "XPT2046.h"

#include <stdio.h>

#include "ILI93xx.h"
#include "main.h"
#include "w25qxx.h"

#define CHX 0x90     // 通道X+的选择控制字
#define CHY 0xd0     // 通道Y+的选择控制字
#define THRESHOLD 2  // 差值门限

typedef struct {
  // sizeof(long double) = 8
  long double An, Bn, Cn, Dn, En, Fn, Divider;
} TOUCH_PARAM;

static volatile uint8_t touch_flag;
static uint8_t cal_flag = 0xcc;  // 触摸屏校正标志位
static long double cal_p[6] = {
    0.089694,
    -0.001004,
    -18.180215,
    0.001288,
    0.062815,
    -12.706554,
};  // 触摸屏校正系数

static POINT ScreenSample[4];
static POINT DisplaySample[4] = {
    {40, 35},
    {20, 200},
    {290, 200},
    {270, 35},
};
static TOUCH_PARAM touch_param;  // 用于保存校正系数

static inline void TP_CS(uint8_t in) {
  if (in) {
    LL_GPIO_SetOutputPin(T_CS_GPIO_Port, T_CS_Pin);
  } else {
    LL_GPIO_ResetOutputPin(T_CS_GPIO_Port, T_CS_Pin);
  }
}

static inline void TP_DCLK(uint8_t in) {
  if (in) {
    LL_GPIO_SetOutputPin(T_SCK_GPIO_Port, T_SCK_Pin);
  } else {
    LL_GPIO_ResetOutputPin(T_SCK_GPIO_Port, T_SCK_Pin);
  }
}

static inline void TP_DIN(uint8_t in) {
  if (in) {
    LL_GPIO_SetOutputPin(T_MOSI_GPIO_Port, T_MOSI_Pin);
  } else {
    LL_GPIO_ResetOutputPin(T_MOSI_GPIO_Port, T_MOSI_Pin);
  }
}

static inline uint32_t TP_DOUT() {
  return LL_GPIO_IsInputPinSet(T_MISO_GPIO_Port, T_MISO_Pin);
}

static inline uint32_t INT_IN_2046() {
  return LL_GPIO_IsInputPinSet(T_PEN_GPIO_Port, T_PEN_Pin);
}

void XPT2046_SetFlag(uint8_t v) {
  touch_flag = v;
}

uint8_t XPT2046_GetFlag() {
  return touch_flag;
}

/* us 延时 */
static void DelayUS(uint32_t cnt) {
  uint16_t i;
  for (i = 0; i < cnt; i++) {
    uint8_t us = 12;
    while (us--) {
      ;
    }
  }
}

void Touch_Init(uint8_t flag) {
  // 片选使能
  TP_CS(0);
  // 等待触摸屏校正
  while(Touch_Calibrate(flag) != 0);
}

/*
*********************************************************************************************************
*	函 数 名: XPT2046_WriteCMD
*	功能说明: 写命令
*	形    参：CHX 	0x90 	//通道Y+的选择控制字 CHY 	0xd0	//通道X+的选择控制字
*	返 回 值: 无
*********************************************************************************************************
*/
static void XPT2046_WriteCMD(uint8_t cmd) {
  uint8_t i, buf;
  // TP_CS(1);
  TP_DIN(0);
  TP_DCLK(0);
  // TP_CS(0);
  for (i = 0; i < 8; i++) {
    buf = (cmd >> (7 - i)) & 0x1;
    TP_DIN(buf);
    DelayUS(5);
    TP_DCLK(1);
    DelayUS(5);
    TP_DCLK(0);
  }
}

/*
*********************************************************************************************************
*	函 数 名: XPT2046_ReadCMD
*	功能说明: 选择一个模拟通道，启动ADC，并返回ADC采样结果
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static uint16_t XPT2046_ReadCMD(void) {
  uint16_t buf = 0, temp;
  uint8_t i;
  TP_DIN(0);
  TP_DCLK(1);
  for (i = 0; i < 12; i++) {
    TP_DCLK(0);
    temp = (TP_DOUT()) ? 1 : 0;
    buf |= (temp << (11 - i));
    TP_DCLK(1);
  }
  buf &= 0x0fff;

  return (buf);
}

/*
*********************************************************************************************************
*	函 数 名: TSC2046_ReadAdc
*	功能说明: 选择一个模拟通道，启动ADC，并返回ADC采样结果
*	形    参：_ucCh = 0x90 表示Y通道； 0xd0 表示X通道
*	返 回 值: 12位ADC值
*********************************************************************************************************
*/
uint16_t XPT2046_ReadAdc(uint8_t _ucCh) {
  // uint16_t usAdc;

  XPT2046_WriteCMD(_ucCh);

  return XPT2046_ReadCMD();
}

/*
 * 读取TP x y 的AD值(12bit，最大是4096)
 */
void Touch_GetAdXY(int *x, int *y) {
  int adx, ady;

  adx = XPT2046_ReadAdc(CHX);
  DelayUS(1);
  ady = XPT2046_ReadAdc(CHY);

  *x = adx;
  *y = ady;
}

/*
 * 校正触摸时画十字专用
 * x:0~300
 * y:0~230
 */
void DrawCross(uint16_t x, uint16_t y) {
  LCD_Fill(x, y, x + 20, y, RED);
  LCD_Fill(x + 10, y - 10, x + 10, y + 10, RED);
}

/******************************************************
 * 函数名：Read_2046
 * 描述  ：得到滤波之后的X Y
 * 输入  : 无
 * 输出  ：POINT结构体地址
 * 举例  ：无
 * 注意  ：速度相对比较慢
 *********************************************************/
POINT *Read_2046(void) {
  static POINT screen;
  int m0, m1, m2, TP_X[1], TP_Y[1], temp[3];
  uint8_t count = 0;

  /* 坐标X和Y进行9次采样*/
  int buffer[2][9] = {{0}, {0}};
  do {
    Touch_GetAdXY(TP_X, TP_Y);
    buffer[0][count] = TP_X[0];
    buffer[1][count] = TP_Y[0];
    count++;
  } /*用户点击触摸屏时即TP_INT_IN信号为低 并且 count<9*/
  while (!INT_IN_2046() && count < 9);

  /*如果触笔弹起*/
  if (INT_IN_2046() == 1) {
    /*中断标志复位*/
    touch_flag = 0;
  }

  /* 如果成功采样9次,进行滤波 */
  if (count == 9) {
    /* 为减少运算量,分别分3组取平均值 */
    temp[0] = (buffer[0][0] + buffer[0][1] + buffer[0][2]) / 3;
    temp[1] = (buffer[0][3] + buffer[0][4] + buffer[0][5]) / 3;
    temp[2] = (buffer[0][6] + buffer[0][7] + buffer[0][8]) / 3;

    /* 计算3组数据的差值 */
    m0 = temp[0] - temp[1];
    m1 = temp[1] - temp[2];
    m2 = temp[2] - temp[0];

    /* 对上述差值取绝对值 */
    m0 = m0 > 0 ? m0 : (-m0);
    m1 = m1 > 0 ? m1 : (-m1);
    m2 = m2 > 0 ? m2 : (-m2);

    /* 判断绝对差值是否都超过差值门限，如果这3个绝对差值都超过门限值，则判定这次采样点为野点,抛弃采样点，差值门限取为2 */
    if (m0 > THRESHOLD && m1 > THRESHOLD && m2 > THRESHOLD)
      return 0;

    /* 计算它们的平均值，同时赋值给screen */
    if (m0 < m1) {
      if (m2 < m0) {
        screen.x = (temp[0] + temp[2]) / 2;
      } else {
        screen.x = (temp[0] + temp[1]) / 2;
      }
    } else if (m2 < m1) {
      screen.x = (temp[0] + temp[2]) / 2;
    } else {
      screen.x = (temp[1] + temp[2]) / 2;
    }

    /* 同上 计算Y的平均值 */
    temp[0] = (buffer[1][0] + buffer[1][1] + buffer[1][2]) / 3;
    temp[1] = (buffer[1][3] + buffer[1][4] + buffer[1][5]) / 3;
    temp[2] = (buffer[1][6] + buffer[1][7] + buffer[1][8]) / 3;
    m0 = temp[0] - temp[1];
    m1 = temp[1] - temp[2];
    m2 = temp[2] - temp[0];
    m0 = m0 > 0 ? m0 : (-m0);
    m1 = m1 > 0 ? m1 : (-m1);
    m2 = m2 > 0 ? m2 : (-m2);
    if (m0 > THRESHOLD && m1 > THRESHOLD && m2 > THRESHOLD) {
      return 0;
    }

    if (m0 < m1) {
      if (m2 < m0) {
        screen.y = (temp[0] + temp[2]) / 2;
      } else {
        screen.y = (temp[0] + temp[1]) / 2;
      }
    } else if (m2 < m1) {
      screen.y = (temp[0] + temp[2]) / 2;
    } else {
      screen.y = (temp[1] + temp[2]) / 2;
    }

    return &screen;
  } else if (count > 1) {
    screen.x = buffer[0][0];
    screen.y = buffer[1][0];
    return &screen;
  }
  return 0;
}

/******************************************************
 * 函数名：Read_2046
 * 描述  ：得到简单滤波之后的X Y
 * 输入  : 无
 * 输出  ：POINT结构体地址
 * 举例  ：无
 * 注意  ：”画板应用实例"专用,不是很精准，但是速度比较快
 *********************************************************/
POINT *Read_2046_2(void) {
  static POINT screen2;
  int TP_X[1], TP_Y[1];
  uint8_t count = 0;
  int buffer[2][10] = {{0}, {0}}; /*坐标X和Y进行多次采样*/
  int min_x, max_x;
  int min_y, max_y;
  int i = 0;

  do { /* 循环采样10次 */
    Touch_GetAdXY(TP_X, TP_Y);
    buffer[0][count] = TP_X[0];
    buffer[1][count] = TP_Y[0];
    count++;
  } /*用户点击触摸屏时即TP_INT_IN信号为低 并且 count<10*/
  while (!INT_IN_2046() && count < 10);

  /*如果触笔弹起*/
  if (INT_IN_2046()) {
    /*中断标志复位*/
    touch_flag = 0;
  }

  /*如果成功采样10个样本*/
  if (count == 10) {
    max_x = min_x = buffer[0][0];
    max_y = min_y = buffer[1][0];
    for (i = 1; i < 10; i++) {
      if (buffer[0][i] < min_x) {
        min_x = buffer[0][i];
      } else if (buffer[0][i] > max_x) {
        max_x = buffer[0][i];
      }
    }

    for (i = 1; i < 10; i++) {
      if (buffer[1][i] < min_y) {
        min_y = buffer[1][i];
      } else if (buffer[1][i] > max_y) {
        max_y = buffer[1][i];
      }
    }
    /*去除最小值和最大值之后求平均值*/
    screen2.x = (buffer[0][0] + buffer[0][1] + buffer[0][2] + buffer[0][3] + buffer[0][4] + buffer[0][5] + buffer[0][6] + buffer[0][7] + buffer[0][8] + buffer[0][9] - min_x - max_x) >> 3;
    screen2.y = (buffer[1][0] + buffer[1][1] + buffer[1][2] + buffer[1][3] + buffer[1][4] + buffer[1][5] + buffer[1][6] + buffer[1][7] + buffer[1][8] + buffer[1][9] - min_y - max_y) >> 3;

    return &screen2;
  }
  return 0;
}

/******************************************************
 * 函数名：Cal_touch_para
 * 描述  ：计算出触摸屏到液晶屏坐标变换的转换函数的 K A B C D E F系数
 * 输入  : 无
 * 输出  ：返回1表示成功 0失败
 * 举例  ：无
 * 注意  ：只有在LCD和触摸屏间的误差角度非常小时,才能运用下面公式
 *********************************************************/
int8_t Cal_touch_para(POINT *displayPtr, POINT *screenPtr, TOUCH_PARAM *para) {
  int8_t retTHRESHOLD = 1;

  /* K＝(X0－X2) (Y1－Y2)－(X1－X2) (Y0－Y2) */
  para->Divider = ((screenPtr[0].x - screenPtr[2].x) * (screenPtr[1].y - screenPtr[2].y)) -
                  ((screenPtr[1].x - screenPtr[2].x) * (screenPtr[0].y - screenPtr[2].y));

  if (para->Divider == 0) {
    retTHRESHOLD = 0;
  } else {
    /* A＝((XD0－XD2) (Y1－Y2)－(XD1－XD2) (Y0－Y2))／K	*/
    para->An = ((displayPtr[0].x - displayPtr[2].x) * (screenPtr[1].y - screenPtr[2].y)) -
               ((displayPtr[1].x - displayPtr[2].x) * (screenPtr[0].y - screenPtr[2].y));

    /* B＝((X0－X2) (XD1－XD2)－(XD0－XD2) (X1－X2))／K	*/
    para->Bn = ((screenPtr[0].x - screenPtr[2].x) * (displayPtr[1].x - displayPtr[2].x)) -
               ((displayPtr[0].x - displayPtr[2].x) * (screenPtr[1].x - screenPtr[2].x));

    /* C＝(Y0(X2XD1－X1XD2)+Y1(X0XD2－X2XD0)+Y2(X1XD0－X0XD1))／K */
    para->Cn = (screenPtr[2].x * displayPtr[1].x - screenPtr[1].x * displayPtr[2].x) * screenPtr[0].y +
               (screenPtr[0].x * displayPtr[2].x - screenPtr[2].x * displayPtr[0].x) * screenPtr[1].y +
               (screenPtr[1].x * displayPtr[0].x - screenPtr[0].x * displayPtr[1].x) * screenPtr[2].y;

    /* D＝((YD0－YD2) (Y1－Y2)－(YD1－YD2) (Y0－Y2))／K	*/
    para->Dn = ((displayPtr[0].y - displayPtr[2].y) * (screenPtr[1].y - screenPtr[2].y)) -
               ((displayPtr[1].y - displayPtr[2].y) * (screenPtr[0].y - screenPtr[2].y));

    /* E＝((X0－X2) (YD1－YD2)－(YD0－YD2) (X1－X2))／K	*/
    para->En = ((screenPtr[0].x - screenPtr[2].x) * (displayPtr[1].y - displayPtr[2].y)) -
               ((displayPtr[0].y - displayPtr[2].y) * (screenPtr[1].x - screenPtr[2].x));

    /* F＝(Y0(X2YD1－X1YD2)+Y1(X0YD2－X2YD0)+Y2(X1YD0－X0YD1))／K */
    para->Fn = (screenPtr[2].x * displayPtr[1].y - screenPtr[1].x * displayPtr[2].y) * screenPtr[0].y +
               (screenPtr[0].x * displayPtr[2].y - screenPtr[2].x * displayPtr[0].y) * screenPtr[1].y +
               (screenPtr[1].x * displayPtr[0].y - screenPtr[0].x * displayPtr[1].y) * screenPtr[2].y;
  }
  return (retTHRESHOLD);
}

/******************************************************
 * 函数名：Touchl_Calibrate
 * 描述  : 触摸屏校正函数
 * 输入  : flag 为 1 时从 eeprom 获取参数
 * 输出  : 0   ---   校正成功
 *         1   ---   校正失败
 * 举例  : 无
 * 注意  : 无
 *********************************************************/
int Touch_Calibrate(uint8_t flag) {
  uint8_t i, k;
  uint16_t test_x = 0, test_y = 0;
  uint16_t gap_x = 0, gap_y = 0;
  POINT *Ptr;

  if (flag) {
    W25QXX_Read(&cal_flag, 0, 1);
    if( cal_flag == 0x55 ) {
      W25QXX_Read((void*)cal_p, 1, sizeof(cal_p));
      for (k = 0; k < 6; k++) {
        printf("rx = %Lf\n", cal_p[k]);
      }
      return 0;
    }
  }

  for (i = 0; i < 4; i++) {
    LCD_Clear(BLACK);
    POINT_COLOR = RED;
    BACK_COLOR = BLACK;
    LCD_ShowString(50, 110, 18 * 8, 16, 16, (uint8_t *)"Touch Calibrate...");
    LCD_ShowNum(90, 130, i + 1, 1, 32);

    /* 适当的延时很有必要 */
    LL_mDelay(500);
    DrawCross(DisplaySample[i].x, DisplaySample[i].y);  // 显示校正用的“十”字
    do {
      Ptr = Read_2046_2();       // 读取TSC2046数据到变量ptr
    } while (Ptr == (void *)0);  // 当ptr为空时表示没有触点被按下
    ScreenSample[i].x = Ptr->x;  // 把读取的原始数据存放到全局变量ScreenSample结构体
    ScreenSample[i].y = Ptr->y;
  }
  /* 用原始参数计算出 原始参数与坐标的转换系数。 */
  Cal_touch_para(&DisplaySample[0], &ScreenSample[0], &touch_param);

  /*取一个点计算X值*/
  test_x = ((touch_param.An * ScreenSample[3].x) +
               (touch_param.Bn * ScreenSample[3].y) +
               touch_param.Cn) /
           touch_param.Divider;

  /*取一个点计算Y值*/
  test_y = ((touch_param.Dn * ScreenSample[3].x) +
               (touch_param.En * ScreenSample[3].y) +
               touch_param.Fn) /
           touch_param.Divider;

  /* 实际坐标与计算坐标的差 */
  gap_x = (test_x > DisplaySample[3].x) ? (test_x - DisplaySample[3].x) : (DisplaySample[3].x - test_x);
  gap_x = (test_y > DisplaySample[3].y) ? (test_y - DisplaySample[3].y) : (DisplaySample[3].y - test_y);

  // LCD_Rectangle(0,0,320,240,CAL_BACKGROUND_COLOR);
  LCD_Clear(BLACK);

  /* 可以通过修改这两个值的大小来调整精度 */
  if ((gap_x > 10) || (gap_y > 10)) {
    POINT_COLOR = RED;
    BACK_COLOR = BLACK;
    LCD_ShowString(50, 100, 18 * 8, 16, 16, (uint8_t *)"Calibrate fail");
    LCD_ShowString(50, 120, 18 * 8, 16, 16, (uint8_t *)"try again");
    LL_mDelay(2000);
    return 1;
  }

  /* 校正系数为全局变量 */
  // aa1 = (touch_param.An*1.0)/touch_param.Divider;
  // bb1 = (touch_param.Bn*1.0)/touch_param.Divider;
  // cc1 = (touch_param.Cn*1.0)/touch_param.Divider;
  //
  // aa2 = (touch_param.Dn*1.0)/touch_param.Divider;
  // bb2 = (touch_param.En*1.0)/touch_param.Divider;
  // cc2 = (touch_param.Fn*1.0)/touch_param.Divider;

  cal_p[0] = (touch_param.An * 1.0) / touch_param.Divider;  // aa1
  cal_p[1] = (touch_param.Bn * 1.0) / touch_param.Divider;  // bb1
  cal_p[2] = (touch_param.Cn * 1.0) / touch_param.Divider;  // cc1

  cal_p[3] = (touch_param.Dn * 1.0) / touch_param.Divider;  // aa2
  cal_p[4] = (touch_param.En * 1.0) / touch_param.Divider;  // bb2
  cal_p[5] = (touch_param.Fn * 1.0) / touch_param.Divider;  // cc2

  {
    cal_flag = 0x55;
    W25QXX_Write(&cal_flag, 0, 1);
    W25QXX_Write((void *)cal_p, 1, sizeof(cal_p));
    for (k = 0; k < 6; k++) {
      printf("tx = %Lf\n", cal_p[k]);
    }
  }

  POINT_COLOR = RED;
  BACK_COLOR = BLACK;
  LCD_ShowString(50, 100, 18 * 8, 16, 16, (uint8_t *)"Calibrate Succed");
  LL_mDelay(1000);
  return 0;
}

/******************************************************
 * 函数名：Get_touch_point
 * 描述  ：通过 K A B C D E F 把通道X Y的值转换为液晶屏坐标
 * 输入  : 无
 * 输出  ：返回1表示成功 0失败
 * 举例  ：无
 * 注意  ：如果获取的触点信息有误，将返回DISABLE
 *********************************************************/
// long double linear=0 ;
// long double aa1=0,bb1=0,cc1=0,aa2=0,bb2=0,cc2=0;
int8_t Get_touch_point(POINT *displayPtr, POINT *screenPtr) {
  int8_t retTHRESHOLD = 1;

  if (screenPtr == 0) {
    /*如果获取的触点信息有误，则返回DISABLE*/
    retTHRESHOLD = 0;
  } else {
    // if( para->Divider != 0 )  /* 每次都要校正时 */
    if (touch_param.Divider != 1) { /* 校正系数写到FLASH时 */
      // displayPtr->x = ( (aa1 * screenPtr->x) + (bb1 * screenPtr->y) + cc1);
      // displayPtr->y = ((aa2 * screenPtr->x) + (bb2 * screenPtr->y) + cc2 );
      displayPtr->x = ((cal_p[0] * screenPtr->x) + (cal_p[1] * screenPtr->y) + cal_p[2]);
      displayPtr->y = ((cal_p[3] * screenPtr->x) + (cal_p[4] * screenPtr->y) + cal_p[5]);
    } else {
      retTHRESHOLD = 0;
    }
  }
  return (retTHRESHOLD);
}

/*
 * 画板初始化，用于取色用
 */
void Palette_Init(void) {
  /* 整屏清为白色 */
  LCD_Clear(WHITE);

  /* 画两条直线 */
  LCD_Fill(39, 0, 40, 30, BLACK);
  LCD_Fill(0, 29, 40, 30, BLACK);
  POINT_COLOR = RED;
  BACK_COLOR = WHITE;
  LCD_ShowString(7, 10, 3 * 8, 16, 16, (uint8_t *)"CLR");

  LCD_Fill(0, 30, 40, 60, GREEN);
  LCD_Fill(0, 60, 40, 90, BLUE);
  LCD_Fill(0, 90, 40, 120, BRED);
  LCD_Fill(0, 120, 40, 150, GRED);
  LCD_Fill(0, 150, 40, 180, GBLUE);
  LCD_Fill(0, 180, 40, 210, BLACK);
  LCD_Fill(0, 210, 40, 240, RED);

  LL_mDelay(500);
}

/******************************************************
 * 函数名：Palette_draw_point
 * 描述  ：在LCD指定位置画一个大点(包含四个小点)
 * 输入  : Xpos		--X方向位置
 *         Ypos		--Y方向位置
 * 输出  ：无
 * 举例  ：Palette_draw_point(100,100);
 * 注意  ：该函数是 "触摸画板应用实例" 专用函数
 *********************************************************/
void Palette_draw_point(uint16_t x, uint16_t y) {
  /* 画笔默认为黑色 */
  static uint16_t Pen_color = 0;
  uint16_t y_pos = y;

  /* 在画板内取色 */
  if (x < 40) {
    if (y > 30) {
      Pen_color = (y_pos < 60) ? GREEN : (y_pos < 90) ? BLUE
                                     : (y_pos < 120)  ? BRED
                                     : (y_pos < 150)  ? GRED
                                     : (y_pos < 180)  ? GBLUE
                                     : (y_pos < 210)  ? BLACK
                                     : (y_pos < 240)  ? RED
                                                      : BLUE;
    } else { /* 清屏 */
#if 1
      LCD_Fill(40, 0, 320, 240, WHITE);
#else
      LCD_Fill(40, 0, 320, 240, BLACK);
#endif
      return;
    }
  } else {
    LCD_Fast_DrawPoint(x, y, Pen_color);
    LCD_Fast_DrawPoint(x + 1, y, Pen_color);
    LCD_Fast_DrawPoint(x, y + 1, Pen_color);
    LCD_Fast_DrawPoint(x + 1, y + 1, Pen_color);
  }
}
