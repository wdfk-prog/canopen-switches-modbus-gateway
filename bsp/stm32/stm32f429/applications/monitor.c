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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "monitor.h"
/* Private includes ----------------------------------------------------------*/
#include <rtthread.h>

#include "led.h"
#include "turn_motor.h"
#include "walk_motor.h"
#include "mb_handler.h"
#include "adc_dma.h"

/*ulog include*/
#define LOG_TAG              "Monitor"
#define LOG_LVL              DBG_INFO
#include <ulog.h>
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/* 监控线程配置 */
#define THREAD_PRIORITY      7  //线程优先级
#define THREAD_TIMESLICE     10 //线程时间片
#define THREAD_STACK_SIZE    1024//栈大小
/* Private macro -------------------------------------------------------------*/
#define WEINVIEW_MB_ADDR     688
#define IPC_MB_ADDR          199
/* Private variables ---------------------------------------------------------*/
u8 RestFlag = 0; //复位方式 0:硬件复位，1看门狗，2软件，3上电
Baet_TypeDef Weinview_Beat = {true};
Baet_TypeDef IPC_Beat = {true};
static rt_timer_t mv_log_timer = RT_NULL;
static rt_bool_t  mv_log_timer_flag = RT_FALSE;
/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  电机运动日志监控
  * @param  None.
  * @retval None.
  * @note   100ms记录一次各项运动参数
*/
static void mv_log_dump(void * p)
{
  LOG_MV("%f %f %d %f %f", 
          turn.motor.Abs.set_angle,
          turn.motor.Abs.get_angle,
          walk.motor.set_speed,
          walk.motor.get_speed, 
          voltage.AD.vaule*ADC_RATIO
  );
}
int mv_log_timer_init(void)
{
    if (mv_log_timer == RT_NULL)
    {
        mv_log_timer = rt_timer_create("mv_log", mv_log_dump, RT_NULL,
            rt_tick_from_millisecond(1000),
            RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
        mv_log_timer_flag = RT_FALSE;
    }

    return RT_EOK;
}
int mv_log_timer_start(void)
{
    if (mv_log_timer_flag == RT_FALSE)
    {
        if (mv_log_timer != RT_NULL)
        {
            rt_timer_start(mv_log_timer);
            mv_log_timer_flag = RT_TRUE;
        }
    }

    return RT_EOK;
}

int mv_log_timer_stop(void)
{
    if (mv_log_timer_flag == RT_TRUE)
    {
        if (mv_log_timer != RT_NULL)
        {
            rt_timer_stop(mv_log_timer);
            mv_log_timer_flag = RT_FALSE;
        }
    }

    return RT_EOK;
}
MSH_CMD_EXPORT(mv_log_timer_start, mv log timer start);
MSH_CMD_EXPORT(mv_log_timer_stop, mv log timer stop);
/**
  * @brief  工控机心跳检测.
  * @param  None.
  * @retval None.
  * @note   01 06 01 F3 12 34 75 72
*/
static void IPC_Beat_Monitor(void)
{
  if(IPC_Beat.EN == ENABLE)
  {
    if(IPC_Beat.Value == 0X1234)
    {
      IPC_BEAT_BIT_SET;
      if(IPC_Beat.button == 0)
      {
        IPC_Beat.button = 1;
      }
      IPC_Beat.flag = true;
    }
    else
    {
      IPC_BEAT_BIT_RESET;
      IPC_Beat.flag = false;
      IPC_Beat.button = 0;
    }
    IPC_Beat.Value = usRegHoldingBuf[IPC_MB_ADDR] = 0;
  }
  else
  {
    IPC_Beat.flag = true;
    IPC_BEAT_BIT_SET;
    if(IPC_Beat.button != 2)
    {
      IPC_Beat.button  = 2;
    }
  }
}
/**
  * @brief  工控机检测心跳周期.
  * @param  None.
  * @retval None.
  * @note   检测心跳周期时无法启用心跳开关
*/
static void IPC_Beat_Cycle_Detection(void)
{
  static int32_t Beat_now = 0,Beat_last = 0;
  if(IPC_Beat.EN == DISABLE)
  {
    if(IPC_Beat.Value == 0X1234)
    {
      Beat_last = Beat_now;
      Beat_now = rt_tick_get();
      IPC_Beat.Value = usRegHoldingBuf[IPC_MB_ADDR] = 0;
    }
    IPC_Beat.Error = Beat_now - Beat_last;
  }
}
/**
  * @brief  威纶通心跳检测.
  * @param  None.
  * @retval None.
  * @note   01 06 01 F3 12 34 75 72
*/
static void Weinview_Beat_Monitor(void)
{
  if(Weinview_Beat.EN == ENABLE)
  {
    if(Weinview_Beat.Value == 1234)
    {
      WEINVIEW_BEAT_BIT_SET;
      if(Weinview_Beat.button == 0)
      {
        Weinview_Beat.button = 1;
      }
      Weinview_Beat.flag = true;
    }
    else
    {
      WEINVIEW_BEAT_BIT_RESET;
      Weinview_Beat.flag = false;
      Weinview_Beat.button = 0;
    }
    Weinview_Beat.Value = usRegHoldingBuf[WEINVIEW_MB_ADDR] = 0;
  }
  else
  {
    Weinview_Beat.flag = true;
    WEINVIEW_BEAT_BIT_SET;
    if(Weinview_Beat.button != 2)
    {
      Weinview_Beat.button  = 2;
    }
  }
}
/**
  * @brief  心跳检测.
  * @param  None.
  * @retval None.
  * @note   01 06 01 F3 12 34 75 72
*/
static void Beat_Monitor_1200ms(void *p)
{
  Weinview_Beat_Monitor();
  IPC_Beat_Monitor();
}
/**
  * @brief  威纶通检测心跳周期.
  * @param  None.
  * @retval None.
  * @note   检测心跳周期时无法启用心跳开关
*/
static void Weinview_Beat_Cycle_Detection(void)
{
  static int32_t Beat_now = 0,Beat_last = 0;
  if(Weinview_Beat.EN == DISABLE)
  {
    if(Weinview_Beat.Value == 1234)
    {
      Beat_last = Beat_now;
      Beat_now =  rt_tick_get();
      Weinview_Beat.Value = usRegHoldingBuf[WEINVIEW_MB_ADDR] = 0;
    }
    Weinview_Beat.Error = Beat_now - Beat_last;
  }
}
/**
  * @brief  监控线程
  * @param  None
  * @retval None
  * @note   监控电机异常
            输出LED异常代码
*/
static void Monitor(void* p)
{
  while(1)
  {
    rt_thread_mdelay(1);
    Weinview_Beat_Cycle_Detection();
    IPC_Beat_Cycle_Detection();
    if((IPC_Beat.flag == false  && IPC_Beat.EN == 1) 
   || (Weinview_Beat.flag == false  && Weinview_Beat.EN == 1))
    {
      USER_SET_BIT(turn.Stop_state,BEAT_STOP);
      USER_SET_BIT(walk.Stop_state,BEAT_STOP);
    }
    else
    {
      USER_CLEAR_BIT(turn.Stop_state,BEAT_STOP);
      USER_CLEAR_BIT(walk.Stop_state,BEAT_STOP);
    }
    Turn_Motor_Detection();
    Walk_Motor_Detection();
    LED_Abnormal_Alarm();
  }
}
/**
  * @brief  停止代码查询
  * @param  None
  * @retval None
  * @note   None
*/

int Motor_StopCode_Get(void)
{
  rt_kprintf(   "Turn motor stop code is 0X%04X"  ,turn.Stop_state);
  rt_kprintf("\nWalk motor stop code is 0X%04X\n",walk.Stop_state);
  return RT_EOK;
}
MSH_CMD_EXPORT(Motor_StopCode_Get,Motor Stop Code Get);
/**
  * @brief  监控函数初始化
  * @param  None
  * @retval None
  * @note   None
*/
static int Monitor_Init(void)
{
  rt_err_t ret = RT_EOK;
  /* 定时器的控制块 */
  static rt_timer_t timer;
  /* 创建定时器 1  周期定时器 */
  timer = rt_timer_create("Beat1.2s", Beat_Monitor_1200ms,
                             RT_NULL, rt_tick_from_millisecond(1200),
                             RT_TIMER_FLAG_PERIODIC);
  /* 启动定时器*/
  if (timer != RT_NULL) rt_timer_start(timer);
  
  /* 创建线程 放在最后 避免定时器还为初始化*/
  rt_thread_t thread = rt_thread_create( "Monitor",           /* 线程名字 */
                                         Monitor,  /* 线程入口函数 */
                                         RT_NULL,               /* 线程入口函数参数 */
                                         THREAD_STACK_SIZE,     /* 线程栈大小 */
                                         THREAD_PRIORITY,       /* 线程的优先级 */
                                         THREAD_TIMESLICE);     /* 线程时间片 */
  /* 创建成功则启动线程 */
  if (thread != RT_NULL)
  {
      rt_thread_startup(thread);
  }
  else
  {
      LOG_E("Monitor thread created failed.");
      ret =  RT_ERROR;
  }
  return ret;
}
INIT_APP_EXPORT(Monitor_Init);
/**
  * @brief  Reset_ModeDetection:复位方式检测函数
  * @param  None.
  * @retval None.
  * @note   https://www.amobbs.com/thread-5470700-1-1.html
*/
static int Reset_ModeDetection(void)
{
  //看门狗复位标志
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET)
  {
    RestFlag = 1;
    WDT_ALARM_SET;
  }
  //软件复位标志
  else if(__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) != RESET)
  {
    RestFlag =  2;        
  }
  //上电复位标志
  else if(__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) != RESET)
  {
    RestFlag = 3;        
  }
  __HAL_RCC_CLEAR_RESET_FLAGS();
  
  return RT_EOK;
}
INIT_BOARD_EXPORT(Reset_ModeDetection);