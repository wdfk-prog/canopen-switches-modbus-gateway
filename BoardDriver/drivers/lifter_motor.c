/**
 * @file lifter_motor.c
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
/* Includes ------------------------------------------------------------------*/
#include "lifter_motor.h"
/* Private includes ----------------------------------------------------------*/
#include "motor.h"
#include "user_pwm.h"
#include "tim.h"
/*ulog include*/
#define LOG_TAG              "lifter_motor"
#define LOG_LVL              DBG_INFO
#include <ulog.h>
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define PWM_DEV_NAME        "pwm14"         /* PWM设备名称 */
#define PWM_DEV_CHANNEL     1               /* PWM通道 */
#define FREQ                1000            /* Hz*/
#define PERIOD              (1000*1000*1000 / FREQ)/* 周期为1ms，单位为纳秒ns */

#define ENABLE_LVL          GPIO_PIN_RESET  //启动电平
#define DISABLE_LVL         GPIO_PIN_SET    //停止电平

#define MIN_OUTPUT          500             //最小输出范围 单位mm
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
struct rt_device_pwm *pwm_dev;      /* PWM设备句柄 */
lifter_motor_typedef lifter_motor;
/* Private function prototypes -----------------------------------------------*/
static void timer_callback(void *parameter);
/**
 * @brief  比例阀 PWM 初始化
 * @retval int 
 */
static int proportional_valve_pwm_init(void)
{
    /* 查找设备 */
    pwm_dev = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME);
    if (pwm_dev == RT_NULL)
    {
        LOG_W("pwm sample run failed! can't find %s device!", PWM_DEV_NAME);
        return RT_ERROR;
    }
    /* 设置PWM周期和脉冲宽度默认值 */
    rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, PERIOD, 0);
    /* 使能设备 */
    rt_pwm_enable(pwm_dev, PWM_DEV_CHANNEL);

    return RT_EOK;
}
/***********************************初始化函数*********************************/
/**
  * @brief  升降电机停止函数
  * @param  None.
  * @retval None.
  * @note   所有控制引脚置零
*/
void lifter_motor_stop(lifter_motor_typedef *p)
{
  p->pump.IO.level         = DISABLE_LVL;
  p->spill_valve.IO.level  = DISABLE_LVL;

  p->state              = LIFT_STOP;

  rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL,PERIOD, 0);
  HAL_GPIO_WritePin(p->pump.IO.GPIOx,p->pump.IO.GPIO_Pin,p->pump.IO.level);
  HAL_GPIO_WritePin(p->spill_valve.IO.GPIOx,p->spill_valve.IO.GPIO_Pin,p->spill_valve.IO.level);
}
/**
  * @brief  升降电机急停优先级.
  * @param  None.
  * @retval None.
  * @note   None.
*/
uint8_t lifter_motor_stop_priority(lifter_motor_typedef* p)
{
    if(*p->stop_state != NO_STOP)
    {
        lifter_motor_stop(p);
        return 1;
    }
    else
      return 0;
}
/**
  * @brief  升降电机顶升初始化
  * @param  None.
  * @retval None.
  * @note   None.
*/
uint8_t lifter_motor_jack_init(lifter_motor_typedef *p)
{
  static uint8_t cnt = 0;
  //防止电机启动瞬间电流过大
  p->pump.IO.level        = DISABLE_LVL;
  p->spill_valve.IO.level = ENABLE_LVL;
  HAL_GPIO_WritePin(p->pump.IO.GPIOx,p->pump.IO.GPIO_Pin,p->pump.IO.level);
  HAL_GPIO_WritePin(p->spill_valve.IO.GPIOx,p->spill_valve.IO.GPIO_Pin,p->spill_valve.IO.level);
  //延时0.1后关闭
  if(cnt++ >= 100 / p->period)
  {
    p->pump.IO.level        = ENABLE_LVL;
    p->spill_valve.IO.level = DISABLE_LVL;
    HAL_GPIO_WritePin(p->pump.IO.GPIOx,p->pump.IO.GPIO_Pin,p->pump.IO.level);
    HAL_GPIO_WritePin(p->spill_valve.IO.GPIOx,p->spill_valve.IO.GPIO_Pin,p->spill_valve.IO.level);
    cnt = 0;
    return 1;
  }
  return 0;
}
/**
  * @brief  升降电机下降初始化
  * @param  None.
  * @retval None.
  * @note   None.
*/
void lifter_motor_fall_init(lifter_motor_typedef *p)
{
  p->pump.IO.level         = DISABLE_LVL;
  p->spill_valve.IO.level  = ENABLE_LVL;
  HAL_GPIO_WritePin(p->pump.IO.GPIOx,p->pump.IO.GPIO_Pin,p->pump.IO.level);
  HAL_GPIO_WritePin(p->spill_valve.IO.GPIOx,p->spill_valve.IO.GPIO_Pin,p->spill_valve.IO.level);
}
/**
  * @brief  升降电机初始化
  * @param  None.
  * @retval None.
  * @note   None.
*/
void lifter_motor_init(void)
{
  lifter_motor.period = 10;//ms

  lifter_motor.pump.IO.GPIOx            = PUMP_GPIO_Port;
  lifter_motor.pump.IO.GPIO_Pin         = PUMP_Pin;

  lifter_motor.spill_valve.IO.GPIOx     = SPILL_VALVE_GPIO_Port;
  lifter_motor.spill_valve.IO.GPIO_Pin  = SPILL_VALVE_Pin;

  lifter_motor.min_output = MIN_OUTPUT;//限制输出范围

  lifter_motor.pump.PID.P                   = 2;
  lifter_motor.pump.PID.D                   = 0.1;
  lifter_motor.pump.PID.OutputMax           = 10000;
  
  lifter_motor.spill_valve.PID.P            = 2;
  lifter_motor.spill_valve.PID.D            = 0.1;
  lifter_motor.spill_valve.PID.OutputMax    = 10000;

  proportional_valve_pwm_init();

//  lifter_motor_stop(&lifter_motor);

  rt_err_t ret = RT_EOK;
  /* 定时器的控制块 */
  static rt_timer_t timer;
  /* 创建定时器 1  周期定时器 */
  timer = rt_timer_create("lift", timer_callback,
                           RT_NULL, rt_tick_from_millisecond(1),
                           RT_TIMER_FLAG_PERIODIC);

  /* 启动定时器 1 */
  if (timer != RT_NULL) rt_timer_start(timer);
}
/**********************************************************************************/
/**
 * @brief  升降电机PID
 * @param  target 目标值         
 * @param  fb     反馈值
 * @param  p                
 * @retval 0X00,成功;0x01,触发急停;0X02,误差过小停止;0XFF,失败;0XFE,输出异常;
 */
uint8_t lifter_motor_pid(int target,int fb,lifter_motor_typedef *p)
{
  uint16_t duty = 0;//占空比
  uint32_t output = 0;

  if(lifter_motor_stop_priority(p))
  { 
      return 0x01;
  }
  int error = target - fb;//误差

  //误差过小，停止输出
  if(-p->min_output <= error && error <= p->min_output)
  {
      lifter_motor_stop(p);
      return 0X02;
  }
  else if(error > 0)//需要顶升
  {
      if(p->state == LIFT_STOP || p->state == LIFT_FALL)
      {
          if(!lifter_motor_jack_init(p))
          {
              duty = 0;
              goto run;
          }
          else
          {
            p->state = LIFT_JACK;
          }
      }
      if(p->state == LIFT_JACK)
      {
          duty = PID_Cal(&p->pump.PID,fb,target);
      }
  }
  else if(error < 0)//需要下降
  {
      if(p->state == LIFT_STOP || p->state == LIFT_JACK)
      {
           lifter_motor_fall_init(p);
           p->state = LIFT_FALL;
           duty = 0;
      }

      if(p->state == LIFT_FALL)
      {
          duty = -1*PID_Cal(&p->spill_valve.PID,fb,target);
      }
  }

run:
  if(duty < 0 || duty > 10000)
  {
      return 0XFE;
  }
  output = PERIOD / 10000 * duty;
  return rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, PERIOD, output);
}
/**
  * @brief  定时器回调函数.
  * @param  None.
  * @retval None.
  * @note   None.
*/
static void timer_callback(void *parameter)
{
    static uint16_t cnt2 = 0;

    ++cnt2;
    if(!(cnt2 % lifter_motor.period))
    {
        lifter_motor_pid(*lifter_motor.target,*lifter_motor.feedback,&lifter_motor);
        cnt2 = 0;
    }
}