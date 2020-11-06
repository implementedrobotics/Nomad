/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_cordic.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_crs.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_usart.h"
#include "stm32g4xx_ll_gpio.h"

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
#define ENC_A_Pin LL_GPIO_PIN_0
#define ENC_A_GPIO_Port GPIOA
#define ENC_B_Pin LL_GPIO_PIN_1
#define ENC_B_GPIO_Port GPIOA
#define USART_TX_Pin LL_GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin LL_GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define ENC_CS_Pin LL_GPIO_PIN_4
#define ENC_CS_GPIO_Port GPIOA
#define ENC_SCK_Pin LL_GPIO_PIN_5
#define ENC_SCK_GPIO_Port GPIOA
#define ENC_MISO_Pin LL_GPIO_PIN_6
#define ENC_MISO_GPIO_Port GPIOA
#define ENC_MOSI_Pin LL_GPIO_PIN_7
#define ENC_MOSI_GPIO_Port GPIOA
#define I_A_Pin LL_GPIO_PIN_4
#define I_A_GPIO_Port GPIOC
#define I_B_Pin LL_GPIO_PIN_0
#define I_B_GPIO_Port GPIOB
#define I_C_Pin LL_GPIO_PIN_1
#define I_C_GPIO_Port GPIOB
#define LED_STATUS_Pin LL_GPIO_PIN_10
#define LED_STATUS_GPIO_Port GPIOB
#define DRV_CS_Pin LL_GPIO_PIN_12
#define DRV_CS_GPIO_Port GPIOB
#define DRV_SCK_Pin LL_GPIO_PIN_13
#define DRV_SCK_GPIO_Port GPIOB
#define FET_TEMP_Pin LL_GPIO_PIN_14
#define FET_TEMP_GPIO_Port GPIOB
#define DRV_MOSI_Pin LL_GPIO_PIN_15
#define DRV_MOSI_GPIO_Port GPIOB
#define USER_GPIO_Pin LL_GPIO_PIN_6
#define USER_GPIO_GPIO_Port GPIOC
#define EXT_TEMP_Pin LL_GPIO_PIN_8
#define EXT_TEMP_GPIO_Port GPIOA
#define BATT_IN_Pin LL_GPIO_PIN_9
#define BATT_IN_GPIO_Port GPIOA
#define DRV_MISO_Pin LL_GPIO_PIN_10
#define DRV_MISO_GPIO_Port GPIOA
#define TMS_Pin LL_GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin LL_GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define DRV_nFAULT_Pin LL_GPIO_PIN_15
#define DRV_nFAULT_GPIO_Port GPIOA
#define CAN_RX_Pin LL_GPIO_PIN_3
#define CAN_RX_GPIO_Port GPIOB
#define CAN_TX_Pin LL_GPIO_PIN_4
#define CAN_TX_GPIO_Port GPIOB
#define DRV_ENABLE_Pin LL_GPIO_PIN_5
#define DRV_ENABLE_GPIO_Port GPIOB
#define PWM_A_Pin LL_GPIO_PIN_6
#define PWM_A_GPIO_Port GPIOB
#define PWM_B_Pin LL_GPIO_PIN_8
#define PWM_B_GPIO_Port GPIOB
#define PWM_C_Pin LL_GPIO_PIN_9
#define PWM_C_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
