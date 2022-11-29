/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : 
  * @brief          : 
  * @date           :
  ******************************************************************************
  * @attention
  * @author
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LIFT_H
#define __LIFT_H
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "user_math.h"
#include "Smove.h"
/* Exported types ------------------------------------------------------------*/
/** 
  * @brief  继电器 参数 structures definition  
  */  
typedef struct
{
  FunctionalState     flag;              //完成标志
  uint16_t            RTFC;              //所需导通时间
  uint16_t            count;             //当前导通时间
  
  uint16_t            min_RTFC;          //最小导通时间
//  float               min_Length;        //最小运行长度
  
  PinTypeDef          IO;                //继电器IO参数
  position_PID        PID;               //PID参数
}Relay_TypeDef;
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
  * @brief  Lifter_Motor 参数 structures definition  
  */  
typedef struct
{
  float               incoming;          //传入目标定位长度
  int16_t             target;            //目标定位脉冲数
  int16_t             now;               //当前定位值
  int16_t             error;             //定位差值
  uint16_t            period;            //PID控制周期
  int16_t             max_lenth;         //最大定位长度
  int16_t             min_lenth;         //最小定位长度
  
  int16_t             max_position;         //最大定位位置
  int16_t             min_position;         //最小定位位置

  SENSORTypeDef       Lower_Limit;       //下限位传感器
  SENSORTypeDef       SQP;               //接近开关
  SENSORTypeDef       Reflection;        //漫反射传感器
  
  ZERO_state          encoder_flag;      //编码器是否静止状态
  ZERO_state          do_reset_flag;     //回零是否完成状态
  
  Relay_TypeDef       Jack;              //顶升继电器
  Relay_TypeDef       Fall;              //下降继电器
  Lifting_State       state;             //升降状态
  Lifting_State       last_state;        //上一刻升降状态
  
  uint8_t             limit_mode;//限位模式 mode:1：判断限位传感器。2：归零传感器。3：一起判断.4，智能限位
}Lifter_Motor_TypeDef;
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported variables ---------------------------------------------------------*/
extern Lifter_Motor_TypeDef  lift;
/* Exported functions prototypes ---------------------------------------------*/
extern void Lifter_Motor_Stop(Lifter_Motor_TypeDef *p);
extern void Lifter_Motor_Jack(Lifter_Motor_TypeDef *p);
extern void Lifter_Motor_Fall(Lifter_Motor_TypeDef *p);
extern void Lifter_Motor_Set_Target(Lifter_Motor_TypeDef *p,float tar);
#endif /* __LIFT_H */
