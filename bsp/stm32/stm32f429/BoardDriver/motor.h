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
#ifndef __MOTOR_H
#define __MOTOR_H
/* Includes ------------------------------------------------------------------*/
#include "sys.h"
#include "stm32f4xx_it.h"
/* Exported constants --------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/** 
  * @brief  停止代码
  */  
typedef enum
{
  NO_STOP = 0x00U,//无停止            0x00
  HARD_STOP,      //硬件急停          0x02
  SOFT_STOP     , //软急停            0x04
  Detection_STOP, //监控函数急停      0x08
  ENABLE_STOP   , //电机关闭使能急停  0x10
  ALM_STOP      , //报警急停          0X20
  CRASH_STOP    , //防撞条急停        0X40
  VBATT_STOP    , //电压过低急停      0X80
  BEAT_STOP    ,  //电压过低急停      0X100
}Stop_Code;
#define stop_type uint16_t
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
/** 
  * @brief  MOTOR 方向 structures definition  
  */  
typedef enum 
{
  CW  = 0X00U,//顺时针
  CCW = 0XFFU,//逆时针
} Directionstate;
/** 
  * @brief  MOTOR Status structures definition  
  */  
typedef enum 
{
  Motor_STOP     = 0x02U,
  Motor_BUSY     = 0x03U,
  Motor_BREAK    = 0x04U,//刹车
  Motor_RELEASE  = 0x05U,//松开刹车
} Motor_state;
/** 
  * @brief  运动状态structures definition  
  */  
typedef enum 
{
  MOVE_DONE     = 0x01U,//运动完成
  MOVE_UNDONE   = 0x02U,//运动未完成
  MOVE_STOP     = 0X03U,//运动停止 [可能为急停]
  MOVE_UNCHANGE = 0X04U,//传入参数不变
} MOVE_state;
/** 
  * @brief  MOTOR 引脚参数 structures definition  
  */  
typedef struct
{
  PinTypeDef    Brake;             //刹车参数
  PinTypeDef    EN;                //使能参数 
  PinTypeDef    Dir;               //方向参数
}Motor_IOTypeDef;
/** 
  * @brief  MOTOR 状态参数 structures definition  
  */  
typedef struct
{
  PWM_StatusTypeDef      PWM;    //PWM定时器状态
  Motor_state           MOTOR;  //电机状态
}Motor_StatusTypeDef;
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
#endif /* __MOTOR_H */
