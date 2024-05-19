/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define M_PADI_CH1_Pin GPIO_PIN_5
#define M_PADI_CH1_GPIO_Port GPIOE
#define M_PADI_CH2_Pin GPIO_PIN_6
#define M_PADI_CH2_GPIO_Port GPIOE
#define ENC4_A_Pin GPIO_PIN_1
#define ENC4_A_GPIO_Port GPIOF
#define ENC4_A_EXTI_IRQn EXTI1_IRQn
#define ENC4_B_Pin GPIO_PIN_2
#define ENC4_B_GPIO_Port GPIOF
#define ENC4_B_EXTI_IRQn EXTI2_IRQn
#define ENC3_B_Pin GPIO_PIN_3
#define ENC3_B_GPIO_Port GPIOF
#define ENC3_B_EXTI_IRQn EXTI3_IRQn
#define SPI_CS_Pin GPIO_PIN_6
#define SPI_CS_GPIO_Port GPIOF
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define PISTON_PELONTAR_Pin GPIO_PIN_0
#define PISTON_PELONTAR_GPIO_Port GPIOC
#define PISTON_A_Pin GPIO_PIN_1
#define PISTON_A_GPIO_Port GPIOC
#define PISTON_B_Pin GPIO_PIN_2
#define PISTON_B_GPIO_Port GPIOC
#define PISTON_BOLA_Pin GPIO_PIN_3
#define PISTON_BOLA_GPIO_Port GPIOC
#define M_EXDL_CH1_Pin GPIO_PIN_0
#define M_EXDL_CH1_GPIO_Port GPIOA
#define M_EXDL_CH2_Pin GPIO_PIN_1
#define M_EXDL_CH2_GPIO_Port GPIOA
#define RF_CH1_Pin GPIO_PIN_6
#define RF_CH1_GPIO_Port GPIOA
#define RF_CH2_Pin GPIO_PIN_7
#define RF_CH2_GPIO_Port GPIOA
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define RB_CH2_Pin GPIO_PIN_1
#define RB_CH2_GPIO_Port GPIOB
#define ENC1_A_Pin GPIO_PIN_11
#define ENC1_A_GPIO_Port GPIOF
#define ENC1_A_EXTI_IRQn EXTI15_10_IRQn
#define ENC2_A_Pin GPIO_PIN_12
#define ENC2_A_GPIO_Port GPIOF
#define ENC2_A_EXTI_IRQn EXTI15_10_IRQn
#define ENC3_A_Pin GPIO_PIN_13
#define ENC3_A_GPIO_Port GPIOF
#define ENC3_A_EXTI_IRQn EXTI15_10_IRQn
#define ENC2_B_Pin GPIO_PIN_15
#define ENC2_B_GPIO_Port GPIOF
#define ENC2_B_EXTI_IRQn EXTI15_10_IRQn
#define LS_2_Pin GPIO_PIN_0
#define LS_2_GPIO_Port GPIOG
#define LS_2_EXTI_IRQn EXTI0_IRQn
#define EN_LF_Pin GPIO_PIN_1
#define EN_LF_GPIO_Port GPIOG
#define LF_CH1_Pin GPIO_PIN_9
#define LF_CH1_GPIO_Port GPIOE
#define LF_CH2_Pin GPIO_PIN_11
#define LF_CH2_GPIO_Port GPIOE
#define LB_CH1_Pin GPIO_PIN_13
#define LB_CH1_GPIO_Port GPIOE
#define LB_CH2_Pin GPIO_PIN_14
#define LB_CH2_GPIO_Port GPIOE
#define ESC1_Pin GPIO_PIN_14
#define ESC1_GPIO_Port GPIOB
#define ESC2_Pin GPIO_PIN_15
#define ESC2_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define LS_4_Pin GPIO_PIN_10
#define LS_4_GPIO_Port GPIOD
#define LS_4_EXTI_IRQn EXTI15_10_IRQn
#define M_PELONTAR_CH1_Pin GPIO_PIN_12
#define M_PELONTAR_CH1_GPIO_Port GPIOD
#define M_PELONTAR_CH2_Pin GPIO_PIN_13
#define M_PELONTAR_CH2_GPIO_Port GPIOD
#define M_BOLA_CH1_Pin GPIO_PIN_14
#define M_BOLA_CH1_GPIO_Port GPIOD
#define M_BOLA_CH2_Pin GPIO_PIN_15
#define M_BOLA_CH2_GPIO_Port GPIOD
#define LS_3_Pin GPIO_PIN_5
#define LS_3_GPIO_Port GPIOG
#define LS_3_EXTI_IRQn EXTI9_5_IRQn
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define LS_7_Pin GPIO_PIN_8
#define LS_7_GPIO_Port GPIOG
#define M_EXDR_CH1_Pin GPIO_PIN_6
#define M_EXDR_CH1_GPIO_Port GPIOC
#define M_EXDR_CH2_Pin GPIO_PIN_7
#define M_EXDR_CH2_GPIO_Port GPIOC
#define RB_CH1_Pin GPIO_PIN_8
#define RB_CH1_GPIO_Port GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define PISTON_PADI_Pin GPIO_PIN_11
#define PISTON_PADI_GPIO_Port GPIOC
#define PISTON_D_Pin GPIO_PIN_12
#define PISTON_D_GPIO_Port GPIOC
#define EN_LB_Pin GPIO_PIN_0
#define EN_LB_GPIO_Port GPIOD
#define PISTON_C_Pin GPIO_PIN_2
#define PISTON_C_GPIO_Port GPIOD
#define LS_5_Pin GPIO_PIN_4
#define LS_5_GPIO_Port GPIOD
#define LS_5_EXTI_IRQn EXTI4_IRQn
#define LS_6_Pin GPIO_PIN_7
#define LS_6_GPIO_Port GPIOD
#define LS_1_Pin GPIO_PIN_9
#define LS_1_GPIO_Port GPIOG
#define LS_1_EXTI_IRQn EXTI9_5_IRQn
#define EN_RF_Pin GPIO_PIN_10
#define EN_RF_GPIO_Port GPIOG
#define EN_EXDL_Pin GPIO_PIN_11
#define EN_EXDL_GPIO_Port GPIOG
#define EN_RB_Pin GPIO_PIN_12
#define EN_RB_GPIO_Port GPIOG
#define EN_PADI_Pin GPIO_PIN_13
#define EN_PADI_GPIO_Port GPIOG
#define ENC1_B_Pin GPIO_PIN_14
#define ENC1_B_GPIO_Port GPIOG
#define ENC1_B_EXTI_IRQn EXTI15_10_IRQn
#define EN_PELONTAR_Pin GPIO_PIN_15
#define EN_PELONTAR_GPIO_Port GPIOG
#define EN_EXDR_Pin GPIO_PIN_6
#define EN_EXDR_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB
#define EN_BOLA_Pin GPIO_PIN_1
#define EN_BOLA_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
