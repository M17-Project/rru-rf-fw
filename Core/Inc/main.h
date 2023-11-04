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
#define FWD_SENSE_Pin GPIO_PIN_0
#define FWD_SENSE_GPIO_Port GPIOA
#define REF_SENSE_Pin GPIO_PIN_1
#define REF_SENSE_GPIO_Port GPIOA
#define TEMP_SENSE_Pin GPIO_PIN_2
#define TEMP_SENSE_GPIO_Port GPIOA
#define RF_PWR_SP_Pin GPIO_PIN_4
#define RF_PWR_SP_GPIO_Port GPIOA
#define DBG_TP1_Pin GPIO_PIN_1
#define DBG_TP1_GPIO_Port GPIOB
#define DBG_TP2_Pin GPIO_PIN_2
#define DBG_TP2_GPIO_Port GPIOB
#define DBG_TX_Pin GPIO_PIN_10
#define DBG_TX_GPIO_Port GPIOB
#define DBG_RX_Pin GPIO_PIN_11
#define DBG_RX_GPIO_Port GPIOB
#define RX_nCS_Pin GPIO_PIN_6
#define RX_nCS_GPIO_Port GPIOB
#define TX_nCS_Pin GPIO_PIN_7
#define TX_nCS_GPIO_Port GPIOB
#define PA_EN_Pin GPIO_PIN_8
#define PA_EN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
