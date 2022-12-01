/**
 * @file lifter_motor.h
 * @brief 
 * @author HLY (1425075683@qq.com)
 * @version 1.0
 * @date 2022-11-28
 * @copyright Copyright (c) 2022
 * @attention 
 * @par 修改日志:
 * Date       Version Author  Description
 * 2022-11-28 1.0     HLY     first version
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LIFTER_MOTOR_H
#define __LIFTER_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <rtthread.h>
#include <rtdevice.h>
#include "stm32f4xx_it.h"
#include "user_math.h"
/* Exported types ------------------------------------------------------------*/
/** 
  * @brief  叉臂升降状态 structures definition  
  */  
typedef enum
{
  LIFT_STOP,
  LIFT_JACK,
  LIFT_FALL,
}Lifting_State;
/** 
  * @brief  继电器 参数 structures definition  
  */  
typedef struct
{
  PinTypeDef   IO; //继电器IO参数
  position_PID PID;//PID参数
}Relay_TypeDef;
/** 
  * @brief  Lifter_Motor 参数 structures definition  
  */  
typedef struct
{
  int16_t       *target;      //目标定位长度 单位mm
  int16_t       *feedback;    //反馈长度 单位mm
  int16_t       min_output;   //最小输出范围
  uint16_t      period;       //PID控制周期
  uint16_t*     stop_state;   //急停标志
  Relay_TypeDef pump;         //油泵
  Relay_TypeDef spill_valve;  //泄油阀
  Lifting_State state;        //升降状态
}lifter_motor_typedef;
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported variables ---------------------------------------------------------*/
extern lifter_motor_typedef lifter_motor;
/* Exported functions prototypes ---------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __LIFTER_MOTOR_H */