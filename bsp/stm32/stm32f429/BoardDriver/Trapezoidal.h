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
#ifndef __TRAPEZOIDAL_H
#define __TRAPEZOIDAL_H
/* Includes ------------------------------------------------------------------*/
#include "sys.h"
#include "motor.h"
/* Exported types ------------------------------------------------------------*/
/** 
  * @brief  Motor 加减速参数 structures definition  
  */  
typedef struct
{
  uint16_t step;       //加减速步长
  uint16_t time;       //步长周期
  
  int32_t target;      //目标值
  int32_t last_target; //上一次目标值
  int32_t set_value;   //设定值
}Motor_ANDTypeDef;
/** 
  * @brief  Motor 参数 structures definition  
  */  
typedef struct
{
  /*需要初始化的参数*/
  u32                 cycle_pulse_num;             //编码器周期脉冲数
  u16                 reduction_ratio;             //电机减速比
  int16_t             max_speed;                   //最大限速
  int16_t             min_speed;                   //最小限速
  int16_t             set_speed;                   //给入速度[有正负][RPM]
  float               get_speed;                   //计算的速度
  u32                 set_freq;                    //给入频率
  int32_t             get_freq;                    //给入频率
  TIM_HandleTypeDef*  TIMx;                        //定时器
  u32                 Channel;                     //定时器通道
  GPIO_PinState       clockwise;                   //顺时针方向默认电平
  Directionstate dir;                             //方向
  Motor_ANDTypeDef    AND;                         //加减速参数
  
  void (*Motor_Enable)(void);       //电机使能函数
  void (*Motor_Disenable)(void);    //电机禁用函数
  void (*Motor_Start)(void);        //电机启动函数
  void (*Motor_Stop) (void);        //电机停止函数
  void (*Motor_Direction)(Directionstate dir);//电机换向函数
  void (*Motor_init) (TIM_HandleTypeDef* TIMx);//电机初始化函数
  /*返回true,停止电机并退出控制函数
    返回FALSE,继续运行控制函数
    保证状态异常时，无法继续启用电机
  */
  uint8_t (*Motor_Stop_Priority)(void);//电机停止优先级函数
  PWM_StatusTypeDef (*Modify_Freq)(TIM_HandleTypeDef* TIMx,u32 Channel, uint32_t _ulFreq);//修改频率函数
}Trapezoidal_TypeDef;
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported variables ---------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
extern void Trapezoidal_Init(Trapezoidal_TypeDef *p,u16 reduction_ratio,
int16_t max_speed,int16_t min_speed,TIM_HandleTypeDef* TIMx, u32 Channel,
uint16_t time,uint16_t step,GPIO_PinState clockwise,u32 cycle_pulse_num);   //梯形加减速初始化
extern MOVE_state Trapezoidal_Out_Speed(Trapezoidal_TypeDef* p,int16_t v); //设置速度值
extern MOVE_state Trapezoidal_AND(Trapezoidal_TypeDef* p);                 //加减速公共函数 ，放入定时器运行
#endif /* __TRAPEZOIDAL_H */