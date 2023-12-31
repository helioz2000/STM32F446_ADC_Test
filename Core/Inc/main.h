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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

// For display support
#include "fonts.h"
#include "z_displ_ILI9XXX.h"
#include "z_displ_ILI9XXX_test.h"
 /*
#include "z_touch_XPT2046.h"
#include "z_touch_XPT2046_test.h"
#include "z_touch_XPT2046_menu.h"
*/
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
int energy_totals_init(uint8_t reset);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define TOUCH_MISO_Pin GPIO_PIN_2
#define TOUCH_MISO_GPIO_Port GPIOC
#define DISPL_MOSI_Pin GPIO_PIN_3
#define DISPL_MOSI_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define DISPL_LED_Pin GPIO_PIN_6
#define DISPL_LED_GPIO_Port GPIOA
#define DISPL_DC_Pin GPIO_PIN_7
#define DISPL_DC_GPIO_Port GPIOA
#define DISPL_SCK_Pin GPIO_PIN_10
#define DISPL_SCK_GPIO_Port GPIOB
#define DISPL_CS_Pin GPIO_PIN_7
#define DISPL_CS_GPIO_Port GPIOC
#define TOUCH_CS_Pin GPIO_PIN_9
#define TOUCH_CS_GPIO_Port GPIOA
#define DEBUG_Pin GPIO_PIN_10
#define DEBUG_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define ESP01_EN_Pin GPIO_PIN_15
#define ESP01_EN_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define TOUCH_IRQ_Pin GPIO_PIN_4
#define TOUCH_IRQ_GPIO_Port GPIOB
#define TOUCH_IRQ_EXTI_IRQn EXTI4_IRQn
#define CLK_TUNE_Pin GPIO_PIN_5
#define CLK_TUNE_GPIO_Port GPIOB
#define DISPL_RST_Pin GPIO_PIN_6
#define DISPL_RST_GPIO_Port GPIOB
#define ESP01_RST_Pin GPIO_PIN_7
#define ESP01_RST_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
