/**
  ******************************************************************************
  * @file    motor.h
  * @brief   电机驱动
  ******************************************************************************
  * @attention  底层驱动尽量不动 计数定时器初始化要再pwm输出初始化前面
  * @author HLY
    优化Walk_Motor电机初始化函数 2022.04.18
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USER_PWM_H
#define __USER_PWM_H
/* Includes ------------------------------------------------------------------*/
#include "sys.h"
#include "stm32f4xx_it.h"
/* Exported constants --------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/** 
  * @brief  MOTOR Status structures definition  
  */  
typedef enum 
{
  PWM_INIT     = 0X01U,//初始化
  PWM_ERROR    = 0X02U,//错误
  PWM_START    = 0X03U,//开始输出
  PWM_STOP     = 0X04U,//停止输出
  PWM_UPADTE   = 0x05U,//频率更新
  PWM_NO_UPADTE= 0x06U,//频率无更新
  PWM_BUSY     = 0x07U,//pwm正在输出
} PWM_StatusTypeDef;

/* Exported macro ------------------------------------------------------------*/

/* Exported variables ---------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
/************************************步进电机**********************************/
extern void              Motor_PWM_init(TIM_HandleTypeDef* TIMx);
extern PWM_StatusTypeDef Motor_PWM_Stop(TIM_HandleTypeDef* TIMx,u32 Channel);
extern PWM_StatusTypeDef Motor_PWM_Start(TIM_HandleTypeDef* TIMx,u32 Channel);
/************************************参数修改**********************************/
extern PWM_StatusTypeDef Modify_TIM_Freq(TIM_HandleTypeDef* TIMx,u32 Channel, uint32_t _ulFreq);
extern PWM_StatusTypeDef Modify_TIM_Duty(TIM_HandleTypeDef* TIMx,u32 Channel,u16 Duty);
#endif /* __USER_PWM_H */
