/**
  ******************************************************************************
  * @file    Trapezoidal.c
  * @brief   梯形加减速驱动
  * @date    2022.09.04
  ******************************************************************************
  * @attention  重构电机驱动，减少耦合

  速度控制方式.PWM发送脉冲或者发送占空比
  如果也采用查表法7S加减速方式。做到速度变化精度大，需要占较大内存。需要一个数组查表
  采用梯形加减速方式，不需要较大内存,运算简单，不需要进入中断记录脉冲个数控制。
  除非是需要控制位置，才会给7s加减速的速度控制。否则采用梯形加减速效果更好。
  * @author HLY
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "Trapezoidal.h"
/* Private includes ----------------------------------------------------------*/
#include "stdlib.h"
#include "user_math.h"
/* Private define ------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  Trapezoidal始化
  * @param  None.
  * @retval None.
  * @note   None.
*/
void Trapezoidal_Init(Trapezoidal_TypeDef *p,u16 reduction_ratio,
int16_t max_speed,int16_t min_speed,TIM_HandleTypeDef* TIMx, u32 Channel,
uint16_t time,uint16_t step,GPIO_PinState clockwise,u32 cycle_pulse_num)
{
  p->reduction_ratio  = reduction_ratio;
  p->max_speed        = max_speed;
  p->min_speed        = min_speed;
  p->TIMx             = TIMx;
  p->Channel          = Channel;
  p->AND.time         = time;
  p->AND.step         = step;
  p->clockwise        = clockwise;
  
  p->Motor_init(p->TIMx);
  p->Motor_Enable();
}
/**
  * @brief  梯形加减速速度输出函数
  * @param  p 电机各项参数
  * @param  speed 电机转速 转速单位[RPM：转/分钟]
  * @retval None
  * @note   
*/
MOVE_state Trapezoidal_Out_Speed(Trapezoidal_TypeDef* p,int16_t v)
{
  if(p->Motor_Stop_Priority())
    return MOVE_STOP;

  /*范围判断*/
  if(v == 0)
    v = 0;
  else if(abs(v) > p->max_speed * (float)p->reduction_ratio)
    v = p->max_speed * ((v > 0) ? 1 : -1);
  else if(abs(v) < p->min_speed * (float)p->reduction_ratio)
    v = p->min_speed * ((v < 0) ? 1 : -1);

  //打开输出
  p->AND.target = v * (int16_t)p->cycle_pulse_num / 60;//RPM
  return MOVE_UNDONE;
}
/**
  * @brief  加减速函数.
  * @param  None.
  * @retval None.
  * @note   None.
*/
MOVE_state Trapezoidal_AND(Trapezoidal_TypeDef* p)
{
  int32_t error_target;
  int8_t set_value_sign,target_sign;
  int8_t D_flag; //是否反向   1反向 0 正向
  uint8_t and_flag;//计算方式选择
  
  if(p->Motor_Stop_Priority())
    return MOVE_STOP;
  
  if(p->AND.set_value == p->AND.target)
  {
    goto end;
  }

  error_target = p->AND.target - p->AND.last_target;
  if(abs(error_target) >= p->AND.step)//目标值有变化，需加减速处理
  {
     p->AND.last_target = p->AND.target;
  }
  else if(error_target != 0)//目标值变化小或者无变化
  {
    p->AND.set_value = p->AND.target;
    goto end;
  }

  set_value_sign = Compute_Sign_INT(p->AND.set_value,32);
  target_sign    = Compute_Sign_INT(p->AND.target,32);
  //有值为0时，也是不需要反向
  D_flag = set_value_sign * target_sign;
  
  if(D_flag == -1)//需要反向
  {
    if(set_value_sign == 1)//由负到正
    {
      and_flag = 0;
    }
    else
      and_flag = 1;
  }
  else
  {
    if(abs(p->AND.set_value) < abs(p->AND.target))//加速
    {
      if(target_sign != -1)//都是非负数
      {
        and_flag = 1;
      }
      else
      {
        and_flag = 0;
      }
    }
    else//减速
    {
      if(target_sign != -1)//都是非负数
      {
        if(set_value_sign == 1)//正数到0
          and_flag = 0;
        else    //负数到0
          and_flag = 1;
      }
      else
      {
        and_flag = 1;
      }
    }
  }
  
  if(and_flag == 1)//加法
  {
    p->AND.set_value += p->AND.step;
    if(p->AND.set_value > p->AND.target)
      p->AND.set_value = p->AND.target;
  }
  else//减法
  {
    p->AND.set_value -= p->AND.step;
    if(p->AND.set_value < p->AND.target)
      p->AND.set_value = p->AND.target;
  }

  end:
  if(p->AND.set_value > 0)
  {
    p->dir = CW;                                       //顺时针
  }
  else if(p->AND.set_value < 0)
  {
    p->dir    = CCW;                                      //逆时针
  }
  p->Motor_Direction(p->dir);

  p->set_freq = (abs(p->AND.set_value) * (float)p->reduction_ratio);
  if(p->set_freq == 0)
  {
     p->Motor_Stop();
     return MOVE_DONE;
  }
  else
  {
    p->Modify_Freq(p->TIMx,p->Channel,p->set_freq);
    p->Motor_Start();
    return MOVE_UNDONE;
  }
}