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
#include "stm32g4xx_hal.h"

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
#define CFG3_Pin GPIO_PIN_4
#define CFG3_GPIO_Port GPIOA
#define CFG2_Pin GPIO_PIN_5
#define CFG2_GPIO_Port GPIOA
#define CFG1_Pin GPIO_PIN_6
#define CFG1_GPIO_Port GPIOA
#define CFG0_Pin GPIO_PIN_7
#define CFG0_GPIO_Port GPIOA
#define DRV_EN_Pin GPIO_PIN_5
#define DRV_EN_GPIO_Port GPIOC
#define CFG4_Pin GPIO_PIN_0
#define CFG4_GPIO_Port GPIOB
#define CFG5_Pin GPIO_PIN_1
#define CFG5_GPIO_Port GPIOB
#define CFG6_Pin GPIO_PIN_2
#define CFG6_GPIO_Port GPIOB
#define DIR_Pin GPIO_PIN_9
#define DIR_GPIO_Port GPIOA
#define SPI_MODE_Pin GPIO_PIN_11
#define SPI_MODE_GPIO_Port GPIOA
#define SD_MODE_Pin GPIO_PIN_12
#define SD_MODE_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
#define READ 0x01
#define WRITE 0x00
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
