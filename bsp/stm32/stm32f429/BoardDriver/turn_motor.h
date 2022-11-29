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
#ifndef __TURN_MOTOR_H
#define __TURN_MOTOR_H
/* Includes ------------------------------------------------------------------*/
#include "Smove.h"
#include "tim.h"
/* Exported types ------------------------------------------------------------*/
/** 
  * @brief  MOTOR参数 structures definition  
  */  
typedef struct
{
  u32                   set_freq;                      //给入的定时器频率
  int32_t               get_freq;                      //计算的频率
  float                 set_radian;                    //设置弧度
  float                 get_radian;                    //反馈弧度
  uint8_t               limit_mode;                    //限位模式 mode:1：判断限位传感器。2：归零传感器。3：一起判断.4，智能限位
  stop_type             Stop_state;                    //软急停标志位
  u8                    jog_flag;                      //点动标志
  
  Motor_IOTypeDef       PIN;                           //引脚参数
  SENSORTypeDef         ALM;                           //报警引脚
  SMove_TypeDef         motor;
  Motor_StatusTypeDef   state;
}TURN_MOTOR_TypeDef;
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported variables ---------------------------------------------------------*/
extern TURN_MOTOR_TypeDef turn;
/* Exported functions prototypes ---------------------------------------------*/
extern void Turn_Motor_Init(void);
extern void Turn_Motor_Stop(void);
extern void Turn_SMove_DoReset(void *p);
extern void Turn_Motor_Detection(void);
#endif /* __TURN_MOTOR_H */
