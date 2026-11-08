/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TDC3_CS_Pin GPIO_PIN_2
#define TDC3_CS_GPIO_Port GPIOE
#define TDC4_CS_Pin GPIO_PIN_3
#define TDC4_CS_GPIO_Port GPIOE
#define TDC5_CS_Pin GPIO_PIN_4
#define TDC5_CS_GPIO_Port GPIOE
#define TDC6_CS_Pin GPIO_PIN_5
#define TDC6_CS_GPIO_Port GPIOE
#define TDC7_CS_Pin GPIO_PIN_6
#define TDC7_CS_GPIO_Port GPIOE
#define SD_DETECT_Pin GPIO_PIN_13
#define SD_DETECT_GPIO_Port GPIOC
#define MAC1_TX_Pin GPIO_PIN_0
#define MAC1_TX_GPIO_Port GPIOA
#define MAC1_RX_Pin GPIO_PIN_1
#define MAC1_RX_GPIO_Port GPIOA
#define DEBUG_TX_Pin GPIO_PIN_2
#define DEBUG_TX_GPIO_Port GPIOA
#define DEBUG_RX_Pin GPIO_PIN_3
#define DEBUG_RX_GPIO_Port GPIOA
#define IMU_INT1_Pin GPIO_PIN_0
#define IMU_INT1_GPIO_Port GPIOB
#define IMU_INT1_EXTI_IRQn EXTI0_IRQn
#define TDC8_CS_Pin GPIO_PIN_7
#define TDC8_CS_GPIO_Port GPIOE
#define TDC_EN_Pin GPIO_PIN_8
#define TDC_EN_GPIO_Port GPIOE
#define RTC1_CS_Pin GPIO_PIN_9
#define RTC1_CS_GPIO_Port GPIOE
#define RTC2_CS_Pin GPIO_PIN_10
#define RTC2_CS_GPIO_Port GPIOE
#define RTC3_CS_Pin GPIO_PIN_11
#define RTC3_CS_GPIO_Port GPIOE
#define RTC4_CS_Pin GPIO_PIN_12
#define RTC4_CS_GPIO_Port GPIOE
#define SDR_CS_Pin GPIO_PIN_13
#define SDR_CS_GPIO_Port GPIOE
#define IMU_CS_Pin GPIO_PIN_14
#define IMU_CS_GPIO_Port GPIOE
#define SD_CS_Pin GPIO_PIN_15
#define SD_CS_GPIO_Port GPIOE
#define CSAC2_TX_Pin GPIO_PIN_10
#define CSAC2_TX_GPIO_Port GPIOB
#define CSAC2_RX_Pin GPIO_PIN_11
#define CSAC2_RX_GPIO_Port GPIOB
#define TDC1_INTB_Pin GPIO_PIN_8
#define TDC1_INTB_GPIO_Port GPIOD
#define TDC1_INTB_EXTI_IRQn EXTI9_5_IRQn
#define TDC2_INTB_Pin GPIO_PIN_9
#define TDC2_INTB_GPIO_Port GPIOD
#define TDC2_INTB_EXTI_IRQn EXTI9_5_IRQn
#define TDC3_INTB_Pin GPIO_PIN_10
#define TDC3_INTB_GPIO_Port GPIOD
#define TDC3_INTB_EXTI_IRQn EXTI15_10_IRQn
#define TDC4_INTB_Pin GPIO_PIN_11
#define TDC4_INTB_GPIO_Port GPIOD
#define TDC4_INTB_EXTI_IRQn EXTI15_10_IRQn
#define TDC5_INTB_Pin GPIO_PIN_12
#define TDC5_INTB_GPIO_Port GPIOD
#define TDC5_INTB_EXTI_IRQn EXTI15_10_IRQn
#define TDC6_INTB_Pin GPIO_PIN_13
#define TDC6_INTB_GPIO_Port GPIOD
#define TDC6_INTB_EXTI_IRQn EXTI15_10_IRQn
#define TDC7_INTB_Pin GPIO_PIN_14
#define TDC7_INTB_GPIO_Port GPIOD
#define TDC7_INTB_EXTI_IRQn EXTI15_10_IRQn
#define TDC8_INTB_Pin GPIO_PIN_15
#define TDC8_INTB_GPIO_Port GPIOD
#define TDC8_INTB_EXTI_IRQn EXTI15_10_IRQn
#define MAC2_TX_Pin GPIO_PIN_6
#define MAC2_TX_GPIO_Port GPIOC
#define MAC2_RX_Pin GPIO_PIN_7
#define MAC2_RX_GPIO_Port GPIOC
#define CSAC1_TX_Pin GPIO_PIN_9
#define CSAC1_TX_GPIO_Port GPIOA
#define CSAC1_RX_Pin GPIO_PIN_10
#define CSAC1_RX_GPIO_Port GPIOA
#define GPS_TX_Pin GPIO_PIN_12
#define GPS_TX_GPIO_Port GPIOC
#define GPS_RX_Pin GPIO_PIN_2
#define GPS_RX_GPIO_Port GPIOD
#define TEMP_SCL_Pin GPIO_PIN_6
#define TEMP_SCL_GPIO_Port GPIOB
#define TEMP_SDA_Pin GPIO_PIN_7
#define TEMP_SDA_GPIO_Port GPIOB
#define TDC1_CS_Pin GPIO_PIN_0
#define TDC1_CS_GPIO_Port GPIOE
#define TDC2_CS_Pin GPIO_PIN_1
#define TDC2_CS_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
