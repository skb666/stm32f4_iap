/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_iwdg.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum {
  FRAME_TYPE_DATA = 0,
  FRAME_TYPE_BEGIN,
  FRAME_TYPE_END,
  FRAME_TYPE_DEBUG,
  FRAME_TYPE_MAX,
} FRAME_TYPE;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define KEY1_Pin LL_GPIO_PIN_3
#define KEY1_GPIO_Port GPIOE
#define KEY0_Pin LL_GPIO_PIN_4
#define KEY0_GPIO_Port GPIOE
#define LED0_Pin LL_GPIO_PIN_9
#define LED0_GPIO_Port GPIOF
#define LED1_Pin LL_GPIO_PIN_10
#define LED1_GPIO_Port GPIOF
#define WK_UP_Pin LL_GPIO_PIN_0
#define WK_UP_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
#define ENUM_ITEM(ITEM) ITEM,
#define ENUM_STRING(ITEM) #ITEM,
#define KEY_ENUM(NAME) \
  NAME(WK_UP)          \
  NAME(KEY0)           \
  NAME(KEY1)           \
  NAME(KEY_NUM)

typedef enum {
  KEY_ENUM(ENUM_ITEM)
} KEY_NAME;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
