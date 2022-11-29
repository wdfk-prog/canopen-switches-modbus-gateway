/**
  ******************************************************************************
  * @file    pwm.c
  * @brief   PWM驱动V1.0
  * @date    2021.01.06
  ******************************************************************************
  * @attention  计数定时器初始化要再pwm输出初始化前面
  * @author HLY
  优化Walk_Motor电机初始化函数             --2022.04.18
  修复Modify_TIM_Freq()中给入频率过高导致占空比与频率误差过大，采用四舍五入宏定义  --2022.07.20
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "motor.h"
/* Private includes ----------------------------------------------------------*/
#include "tim.h"
#include "user_math.h"
/* Private define ------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  电机PWM初始化
  * @param  None
  * @retval None
  * @note   None。
*/
void Motor_PWM_init(TIM_HandleTypeDef* TIMx)
{
  __HAL_TIM_CLEAR_FLAG(TIMx,TIM_FLAG_UPDATE);
//  __HAL_TIM_ENABLE_IT(TIMx,TIM_IT_UPDATE);
  __HAL_TIM_DISABLE(TIMx);
}
/**
  * @brief  PWM停止函数
  * @param  TIMx   : TIM1 - TIM17
  * @param  Channel：使用的定时器通道，范围1 - 4
  * @retval PWM_StatusTypeDef：pwm停止
  * @note   None
*/
PWM_StatusTypeDef Motor_PWM_Stop(TIM_HandleTypeDef* TIMx,u32 Channel)
{
  HAL_TIM_PWM_Stop(TIMx,Channel);
  return PWM_STOP;
}
/**
  * @brief  PWM开始函数
  * @param  TIMx   : TIM1 - TIM17
  * @param  Channel：使用的定时器通道，范围1 - 4
  * @retval PWM_StatusTypeDef：pwm开始
  * @note   当前通道状态不为准备时，返回错误。
*/
PWM_StatusTypeDef Motor_PWM_Start(TIM_HandleTypeDef* TIMx,u32 Channel)
{
  if(TIM_CHANNEL_STATE_GET(TIMx, Channel) != HAL_TIM_CHANNEL_STATE_READY)
  {
      //HAL_TIM_PWM_Stop(TIMx,Channel);
      TIM_CHANNEL_STATE_SET(TIMx,Channel,HAL_TIM_CHANNEL_STATE_READY);
      if(HAL_TIM_PWM_Start(TIMx,Channel) != HAL_OK)
      {
        return PWM_ERROR;
      }
      else
        return PWM_BUSY;
  }
  else if(HAL_TIM_PWM_Start(TIMx,Channel) != HAL_OK)
  {
      return PWM_ERROR;
  }
  return PWM_START;
}
/************************************参数修改**********************************/
/**
  * @brief  修改定时器频率
  * @param  TIMx   : TIM1 - TIM17
  * @param  Channel：使用的定时器通道，范围1 - 4
  * @param  _ulFreq: PWM信号频率，单位Hz (实际测试，可以输出100MHz），0 表示禁止输出
  * @retval PWM_StatusTypeDef：频率更新或者不更新
  * @note   https://blog.csdn.net/qq_35021496/article/details/106120181
            PWM_StatusTypeDef:定时器状态
            当频率无变化时，不计算pwm各项参数。变化时再开始计算
            不对占空比做修改

修复给入频率过高导致占空比与频率误差过大，采用四舍五入宏定义  --2022.07.20
*/
PWM_StatusTypeDef Modify_TIM_Freq(TIM_HandleTypeDef* TIMx,u32 Channel, uint32_t _ulFreq)
{
  uint16_t usPeriod;
	uint16_t usPrescaler;
	uint32_t pulse;
	uint32_t uiTIMxCLK;
  uint32_t error = 0;
  static uint32_t last_freq = 0;
  
  if(_ulFreq == 0)
  {
    return Motor_PWM_Stop(TIMx,Channel);
  }
  
  error = _ulFreq - last_freq;
  last_freq = _ulFreq;

  if(error == 0)
  {
    return PWM_NO_UPADTE;    //频率一致，不更新。
  }
  else
  {
    /* APB1 定时器 = 90M */
    uiTIMxCLK = SystemCoreClock / 2;
    //(uiTIMxCLK / (TIMx->Init.Prescaler + 1 / _ulFreq 四舍五入
    usPeriod =  DIV_ROUND_CLOSEST((uiTIMxCLK / (TIMx->Init.Prescaler + 1)),_ulFreq)  - 1;		/* 自动重装的值 */
    
    __HAL_TIM_SET_AUTORELOAD(TIMx,usPeriod);
    
    TIM_CHANNEL_STATE_SET(TIMx,Channel,HAL_TIM_CHANNEL_STATE_READY);
                                                   //(usPeriod+1)/2 四舍五入
    __HAL_TIM_SET_COMPARE(TIMx,Channel,(uint16_t)(DIV_ROUND_CLOSEST((usPeriod+1),2)));
    return PWM_UPADTE;
  }
}
/**
  * @brief  修改定时器占空比
  * @param  TIMx   : TIM1 - TIM17
  * @param  Channel：使用的定时器通道，范围1 - 4
  * @param  Duty: 占空比 0至100.单位百分比。50代表百分之50
  * @retval PWM_StatusTypeDef：频率更新或者不更新
  * @note   PWM_StatusTypeDef:定时器状态
            当频率无变化时，不计算pwm各项参数。变化时再开始计算
            不对频率做修改
*/
PWM_StatusTypeDef Modify_TIM_Duty(TIM_HandleTypeDef* TIMx,u32 Channel,u16 Duty)
{
  u16 error = 0;
  static u16 last_Duty = 0;
  
  if(Duty > 10000)
    Duty = 10000;
  else if(Duty < 0)
    Duty = 0;
  
  error = Duty - last_Duty;
  last_Duty = Duty;

  if(error == 0)
  {
    return PWM_NO_UPADTE;    //占空比一致，不更新。
  }
  else
  {
    __HAL_TIM_SetCompare(TIMx,Channel,((__HAL_TIM_GET_AUTORELOAD(TIMx) + 1) * Duty) /10000);//修改比较值，修改占空比
    return PWM_UPADTE;
  }
}