/**
  ******************************************************************************
  * @file    Smove.c
  * @brief   电机驱动V3.0
  * @date    2022.08.12
  ******************************************************************************
  * @attention  重构电机驱动，减少耦合
  7s加减速参数尽量不修改
  查看C文件中所述脉冲计数方式进行选择。
  
  使用步骤：1.定义SMove_TypeDef变量;
            2.实现  SMove_Stop
                    Modify_Freq 
                    SMove_init 
                    SMove_Stop_Priority【优先级函数判断为1立刻停止，无法再次启动】
                    SMove_Direction
              函数并挂钩。
            3.初始化SMove_7s_TypeDef参数。可用SMove_7S_Default_Config();初始化默认参数
            4.初始化SMove_Initial函数
   若打开角度定位使能 #define SMove_Angle_Control 1
            除上述步骤外，还需通过SMove_SetAngle_Range函数设置角度范围。
            不设置角度范围默认不判断角度范围
            可通过SMove_GetAngle_Over函数获取输入角度是否超出范围。
            
            通过SMove_SetAngle_Absolute实现绝对角度加减速。
            完成绝对角度加减速后，尽量使用SMove_SetAngle_Cache清除角度缓存。
            避免使用速度函数后再使用角度函数时，两次缓存一致导致无法触发绝对定位。
            
            
   若打开速度使能 #define SMove_Speed_Control 1
            除上述步骤外，还需通过SMove_SetAngle_Range函数设置速度范围。
            不设置角度范围默认不判断速度范围
            可通过SMove_GetAngle_Over函数获取输入速度是否超出范围。
            通过SMove_Get_Speed获取当前速度
            
            通过SMove_SetSpeed实现速度加减速。
            
  * @author HLY
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SMOVE_H
#define __SMOVE_H
/* Includes ------------------------------------------------------------------*/
#include "stdbool.h"
#include "sys.h"
#include "motor.h"

#define SMove_Angle_Control 1 //使能角度控制
#define SMove_Speed_Control 1 //使能速度控制
/* Exported types ------------------------------------------------------------*/
/** 
  * @brief  MOTOR 加减速状态 structures definition  
  */  
typedef enum 
{
  ACCEL,//加速
  CONST,//匀速
  DECEL,//减速
} Motor_PLSRstate;
/** 
  * @brief  MOTOR 定位状态 structures definition  
  */  
typedef enum 
{
  ZERO_INIT      = 0X00U,//初始化
  QUICK_TOUCH    = 0X01U,//高速接近
  LOW_LEAVE      = 0x02U,//低速离开
  ZERO_OK        = 0X03U,//完成
  ZERO_ABNORMAL  = 0X04U,//异常
} ZERO_state;
/** 
  * @brief  MOTOR 限位状态 structures definition  
  */  
typedef enum 
{
  MONITOR_INIT      = 0X00U,//初始化
  MONITOR_ZERO      = 0X01U,//零位异常触碰
  MONITOR_LIMIT     = 0x02U,//限位异常触碰
  MONITOR_NORMAL    = 0X03U,//正常
  MONITOR_ABNORMAL  = 0X04U,//异常
  MONITOR_ALL       = 0X05U,//都触碰
} LIMIT_MONITOR_STATE;
/** 
  * @brief  MOTOR 控制参数 structures definition  
  */  
typedef struct 
{
  //写入信息
  u16 reduction_ratio;              //电机减速比
  GPIO_PinState clockwise;          //顺时针方向默认电平
  TIM_HandleTypeDef* Master_TIMx;   //主定时器
  u32 Channel;                      //定时器通道
  u32 cycle_pulse_num;              //编码器周期脉冲数
  TIM_HandleTypeDef* Slave_TIMx;    //从定时器
  //查询信息
  u32     set_freq;                 //给入的定时器频率
  FunctionalState en;	              //使能
  FunctionalState speedenbale;		  //是否使能速度控制
  FunctionalState running;		  	  //转动完成标志 
  Directionstate dir;        //方向
  //控制信息
  u32 pulsecount;                   //以该频率脉冲输出的脉冲个数
  u32 CurrentIndex;    	            //当前表的位置
  u32 TargetIndex;    	            //目标速度在表中位置
  Motor_PLSRstate  state;          //加减速状态
  u32 RevetDot;			  	 	          //电机运动的减速点
  
  u32 StartTableLength;             //启动数据表
  u32 StopTableLength;              //停止数据表
  
  u32 StartSteps;					          //电机启动步数
  u32 StopSteps;					          //电机停止步数
  
  u32 PulsesGiven;			            //电机运动的总步数
  u32 PulsesHaven;				          //电机已经运行的步数
//  int32_t PulsesErr;                //电机剩余运动步长
  
  u32 CurrentPosition;		          //当前位置
  u32 MaxPosition;				          //最大位置，超过该位置置0
  
  u32 CurrentPosition_Pulse;        //当前位置脉冲个数
  u32 MaxPosition_Pulse;		        //最大位置脉冲个数
  
  u32 Slave_Pulse_IT;               //从定时器溢出中断计数
  u32 Slave_Pulse_Now;		          //从定时器当前记录的脉冲个数
  u32 Slave_Pulse_Last;		          //从定时器上一次记录的脉冲个数
  
  u32 *Counter_Table;  		          //指向启动时，时间基数计数表
  u32 *Step_Table;  			          //指向启动时，每个频率脉冲个数表
  
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
} SMove_Control_TypeDef;
#if (SMove_Angle_Control == 1)
/** 
  * @brief  MOTOR 角度参数 structures definition  
  */  
typedef struct
{
  int16_t max_angle;                    //最大角度
  int16_t min_angle;                    //最小角度
  
  float   set_angle;                    //给入角度
  float   get_angle;                    //反馈角度
  
  u32     dest_position;                //目标位置
  
  FunctionalState over_range;           //是否超出范围标志    ENABLE:超出。DISABLE：无超出
  
  float last;                           //上一次角度
  float err;                            //是否有更新     当角度差不为0时，证明有角度变化，重新计算加减速列表。
}SMove_AngleTypeDef;
#endif
#if (SMove_Speed_Control == 1)
/** 
  * @brief  MOTOR 速度参数 structures definition  
  */  
typedef struct
{
  u16     max_speed;                    //最大限速
  u16     min_speed;                    //最小限速
  int16_t set_speed;                    //给入速度[有正负]
  int32_t get_speed;                    //计算的速度[有正负]
}SMove_SpeedTypeDef;
#endif
/** 
  * @brief  MOTOR 7s加减速参数 structures definition  
  */  
typedef struct
{
  uint8_t  STEP_PARA;				              //任意时刻转动步数修正因子【影响时间与细分度，越小时间越长，越细分】
  uint16_t STEP_AA;						            //加加速阶段，离散化点数【越大越细分，不影响时间】
  uint16_t STEP_UA;				                //匀加速阶段，离散化点数
  uint16_t STEP_RA;			                  //减加速阶段，离散化点数

  uint16_t fstart;                        //电机的启动频率
  uint32_t faa;                           //电机频率的加加速度【决定最大速度上限】
  float taa;                              //电机频率的加加速时间
  float tua;                              //电机频率的匀加速时间
  float tra;                              //电机频率的减加速时间  
  
  uint16_t STEP_LENGTH;  //(STEP_AA + STEP_UA + STEP_RA)//总步长
  
  u32 *MotorTimeTable;  		              //指向启动时，时间基数计数表
  u32 *MotorStepTable;  			            //指向启动时，每个频率脉冲个数表
}SMove_7s_TypeDef;
/** 
  * @brief  MOTOR 回原参数 structures definition  
  */  
typedef struct
{
  int32_t speed_high;     //回原高速
  int32_t speed_low;      //回原低速
  ZERO_state   flag;     //函数状态
  SENSORTypeDef zero;     //零位传感器
  SENSORTypeDef limit;    //限位传感器
  float   init_freq;      //复位角度
  uint8_t init_state;     //0XFF为锁定状态;
}SMove_DoSetTypeDef;
/** 
  * @brief  MOTOR参数 structures definition  
  */  
typedef struct
{
  SMove_Control_TypeDef Ctrl;              //控制参数
  SMove_7s_TypeDef      CFG;               //加减速参数
#if (SMove_Angle_Control == 1)
  SMove_AngleTypeDef    Rel;               //相对角度参数
  SMove_AngleTypeDef    Abs;               //绝对角度参数
#endif
#if (SMove_Speed_Control == 1)
  SMove_SpeedTypeDef    V;                 //速度参数
  SMove_DoSetTypeDef    Do;                //回原参数
#endif
}SMove_TypeDef;
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported variables ---------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
extern void SMove_Stop(SMove_Control_TypeDef* p);                                 //电机停止函数
extern void SMove_Handler(SMove_Control_TypeDef* p);                        //放入us定时器中
extern void SMove_Reinitial(SMove_Control_TypeDef *pmotor);                        //重新初始化控制参数
extern void SMove_Initial(SMove_Control_TypeDef *pmotor,SMove_7s_TypeDef *p,
uint32_t Cycle_pulse_num,uint16_t Reduction_ratio,TIM_HandleTypeDef* Master_timx,
uint32_t HChannel,GPIO_PinState Clockwise,uint32_t *SMove_TimeTable,uint32_t *SMove_StepTable,
TIM_HandleTypeDef* Slave_timx);                                                     //初始化加减速电机
#if (SMove_Speed_Control == 1)  //控制电机运行指定速度函数
extern MOVE_state SMove_SetRPM(SMove_TypeDef * p,int16_t rpm,FunctionalState Stop); //以RPM方式控制速度
extern MOVE_state SMove_SetSpeed(SMove_TypeDef *p,int8_t SpeedIndex,FunctionalState Stop);//控制速度
extern ZERO_state SMove_DoReset(SMove_TypeDef *p,int32_t zero_max,int32_t zero_min,SENSOR_state zero_state, SENSOR_state limit_state);//回原函数
extern LIMIT_MONITOR_STATE SMove_Limit_Detection(SMove_TypeDef* p,uint8_t mode,SENSOR_state zero_state, SENSOR_state limit_state);    //电机监控函数
extern void SMove_SetSpeed_Range(SMove_TypeDef *p,uint32_t max,uint32_t min);        //设置速度范围
extern int32_t SMove_Get_Speed(SMove_Control_TypeDef* p);                            //获取速度
#endif
#if (SMove_Angle_Control == 1)  //控制电机运行指定角度函数
extern FunctionalState SMove_SetAngle_Relative(SMove_TypeDef *p,float relative_angle);         //控制相对角度
extern FunctionalState SMove_SetAngle_Absolute(SMove_TypeDef *p,volatile float absolute_angle);//控制绝对角度
extern void SMove_SetAngle_Range(SMove_AngleTypeDef *p,float max,float min);        //设置角度范围
extern void SMove_SetAngle_Cache(SMove_AngleTypeDef *p);                            //清除角度缓存
extern void SMove_SetAxis_Reset(SMove_Control_TypeDef *p);                          //清除电机机械位置
extern FunctionalState SMove_GetAngle_Over(SMove_AngleTypeDef *p);                  //查询角度是否超出范围
extern float SMove_Get_Angle(SMove_Control_TypeDef* p);                             //获取当前角度 有换算
#endif
#endif /* __SMOVE_H */