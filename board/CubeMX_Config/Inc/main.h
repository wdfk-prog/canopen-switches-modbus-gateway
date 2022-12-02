/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
#include <rtthread.h>
#include <rtdevice.h>
#include <drv_common.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define VERSION "0.0.3"

#define YEAR ((((__DATE__ [7] - '0') * 10 + (__DATE__ [8] - '0')) * 10 \
    + (__DATE__ [9] - '0')) * 10 + (__DATE__ [10] - '0'))
#define MONTH (__DATE__ [2] == 'n' ? (__DATE__ [1] == 'a' ? 0 : 5)  \
    : __DATE__ [2] == 'b' ? 1 \
    : __DATE__ [2] == 'r' ? (__DATE__ [0] == 'M' ? 2 : 3) \
    : __DATE__ [2] == 'y' ? 4 \
    : __DATE__ [2] == 'l' ? 6 \
    : __DATE__ [2] == 'g' ? 7 \
    : __DATE__ [2] == 'p' ? 8 \
    : __DATE__ [2] == 't' ? 9 \
    : __DATE__ [2] == 'v' ? 10 : 11)
#define DAY ((__DATE__ [4] == ' ' ? 0 : __DATE__ [4] - '0') * 10 \
    + (__DATE__ [5] - '0'))
#define DATE_AS_INT (((YEAR - 2000) * 12 + MONTH) * 31 + DAY)
#define HOUR   ((__TIME__ [0] - '0') * 10 + (__TIME__ [1] - '0'))
#define MINUTE ((__TIME__ [3] - '0') * 10 + (__TIME__ [4] - '0'))
#define SEC    ((__TIME__ [6] - '0') * 10 + (__TIME__ [7] - '0'))
#define BURN_TIME 30
#ifdef CUBE_ERROR
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
#endif
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define SPI5_NSS_Pin GPIO_PIN_6
#define SPI5_NSS_GPIO_Port GPIOF
#define Before_Radar_2_Pin GPIO_PIN_1
#define Before_Radar_2_GPIO_Port GPIOC
#define WK_UP_Pin GPIO_PIN_0
#define WK_UP_GPIO_Port GPIOA
#define WK_UP_EXTI_IRQn EXTI0_IRQn
#define VBATT_Pin GPIO_PIN_1
#define VBATT_GPIO_Port GPIOA
#define Lift_Reflection_Pin GPIO_PIN_4
#define Lift_Reflection_GPIO_Port GPIOA
#define Lift_Reflection_EXTI_IRQn EXTI4_IRQn
#define PROPORTIONAL_VALVE_Pin GPIO_PIN_7
#define PROPORTIONAL_VALVE_GPIO_Port GPIOA
#define Before_Radar_3_Pin GPIO_PIN_4
#define Before_Radar_3_GPIO_Port GPIOC
#define Stop_Button_Pin GPIO_PIN_10
#define Stop_Button_GPIO_Port GPIOB
#define Stop_Button_EXTI_IRQn EXTI15_10_IRQn
#define Article_Crash_Pin GPIO_PIN_9
#define Article_Crash_GPIO_Port GPIOH
#define Article_Crash_EXTI_IRQn EXTI9_5_IRQn
#define Turn_ALM_Pin GPIO_PIN_12
#define Turn_ALM_GPIO_Port GPIOH
#define Turn_ALM_EXTI_IRQn EXTI15_10_IRQn
#define SPILL_VALVE_Pin GPIO_PIN_11
#define SPILL_VALVE_GPIO_Port GPIOD
#define LED1_Pin GPIO_PIN_12
#define LED1_GPIO_Port GPIOD
#define Turn_Zero_Pin GPIO_PIN_2
#define Turn_Zero_GPIO_Port GPIOG
#define Turn_Zero_EXTI_IRQn EXTI2_IRQn
#define Lift_SQP_Pin GPIO_PIN_14
#define Lift_SQP_GPIO_Port GPIOH
#define Lift_SQP_EXTI_IRQn EXTI15_10_IRQn
#define Before_Radar_1_Pin GPIO_PIN_1
#define Before_Radar_1_GPIO_Port GPIOI
#define Before_Radar_1_EXTI_IRQn EXTI1_IRQn
#define PUMP_Pin GPIO_PIN_3
#define PUMP_GPIO_Port GPIOI
#define DBG_Pin GPIO_PIN_12
#define DBG_GPIO_Port GPIOC
#define Turn_Limit_Pin GPIO_PIN_3
#define Turn_Limit_GPIO_Port GPIOD
#define Turn_Limit_EXTI_IRQn EXTI3_IRQn
#define Lower_Limit_Pin GPIO_PIN_13
#define Lower_Limit_GPIO_Port GPIOG
#define Lower_Limit_EXTI_IRQn EXTI15_10_IRQn
#define Start_Button_Pin GPIO_PIN_14
#define Start_Button_GPIO_Port GPIOG
#define Walk_ALM_Pin GPIO_PIN_6
#define Walk_ALM_GPIO_Port GPIOI
#define Walk_ALM_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
