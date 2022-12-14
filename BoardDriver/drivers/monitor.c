/**
 * @file monitor.c
 * @brief 
 * @author HLY (1425075683@qq.com)
 * @version 1.0
 * @date 2022-11-23
 * @copyright Copyright (c) 2022
 * @attention 
 * @par 修改日志:
 * Date       Version Author  Description
 * 2022-11-23 1.0     HLY     first version
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "monitor.h"
/* Private includes ----------------------------------------------------------*/
#include <rtthread.h>
#include "user_math.h"
#include "motor.h"
#include <string.h>
/*ulog include*/
#define LOG_TAG              "Monitor"
#define LOG_LVL              DBG_INFO
#include <ulog.h>
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define BEAT_PERIOD_TIME 1000 //心跳检测周期 单位ms
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
Beat_TypeDef debug_beat;
bool beat_enable = true;  //心跳监控开关
/* Private function prototypes -----------------------------------------------*/
/**
 * @brief   心跳回调函数
 * @param  value:心跳异常标志
 * @note   flag: 输出日志标志 true,输出一次心跳恢复;FALSE,输出一次心跳异常;
 */
static void debug_beat_callback(uint8_t value)
{
  static bool flag = false;
  if(value == true)
  {
    USER_CLEAR_BIT(*turn_motor[0].stop_state,BEAT_STOP);
    USER_CLEAR_BIT(*walk_motor[0].stop_state,BEAT_STOP);

    if(flag == true)
    {
      ulog_i("debug","Heartbeat communication recovery");
      flag = false;
    }
  }
  else
  {
    USER_SET_BIT(*turn_motor[0].stop_state,BEAT_STOP); 
    USER_SET_BIT(*walk_motor[0].stop_state,BEAT_STOP); 

    if(flag == false)
    {
      flag = true;
      ulog_w("debug","Abnormal heartbeat");
    }
  }
}
/**
  * @brief  调试串口心跳检测.
  * @param  None.
  * @retval None.
  * @note   flag: 输出日志标志 true,输出一次心跳保护开启;FALSE,输出一次心跳保护关闭;
*/
static void debug_beat_monitor(void)
{
  uint16_t beat = *debug_beat.value;
  static uint8_t flag = 0;
  if(USER_GET_BIT(beat,7) == true)
  {
    if(flag == 0 || flag == 2)
    {
      ulog_i("debug","The heartbeat protection is enabled");
      flag = 1;
    }

    beat = beat & 0X7F;//清除最高位
    if(beat != 0)
    {
      if(debug_beat.button == 0)
      {
        debug_beat.button = 1;
      }
      *debug_beat.flag = true;
    }
    else
    {
      *debug_beat.flag = false;
      debug_beat.button = 0;
    }
    *debug_beat.value = 0x80;
  }
  else
  {
    if(flag == 0 || flag == 1)
    {
      ulog_i("debug","The heartbeat protection is disabled");
      flag = 2;
    }

    *debug_beat.flag = true;
    if(debug_beat.button != 2)
    {
      debug_beat.button  = 2;
    }
  }
  debug_beat_callback(*debug_beat.flag);
}
/**
  * @brief  心跳检测.
  * @param  None.
  * @retval None.
  * @note   1s检查一次
*/
static void beat_monitor(void *p)
{
  if(beat_enable)
  {
    debug_beat_monitor();
  }
}
/**
  * @brief  监控函数初始化
  * @param  None
  * @retval None
  * @note   None
*/
int monitor_init(void)
{
  static rt_timer_t timer;
  /* 创建定时器 1  周期定时器 */
  timer = rt_timer_create("beat", 
                          beat_monitor,
                          RT_NULL, rt_tick_from_millisecond(BEAT_PERIOD_TIME),
                          RT_TIMER_FLAG_PERIODIC);
  /* 启动定时器 1 */
  if (timer != RT_NULL)
  {
     rt_timer_start(timer);
  }
  else
  {
      LOG_E("beat timer init falsed");
  }
  return RT_EOK;
}
#ifdef RT_USING_MSH
/**
  * @brief  心跳控制命令
  * @param  None
  * @retval None
  * @note   None
*/
int beat_cmd(uint8_t argc, char **argv)
{
#define BEAT_CMD_ON                    0
#define BEAT_CMD_OFF                   1

  size_t i = 0;

  const char* help_info[] =
  {
    [BEAT_CMD_ON]             = "beat_cmd on  --beat on",
    [BEAT_CMD_OFF]            = "beat_cmd off --beat off",
  };

  if (argc < 2)
  {
      rt_kprintf("Usage:\n");
      for (i = 0; i < sizeof(help_info) / sizeof(char*); i++)
      {
          rt_kprintf("%s\n", help_info[i]);
      }
      rt_kprintf("\n");
  }
  else
  {
      const char *operator = argv[1];

      if (!strcmp(operator, "on"))
      {
        beat_enable = true;
      }
      else if (!strcmp(operator, "off"))
      {
        beat_enable = false;
      }
      else
      {
          rt_kprintf("Usage:\n");
          for (i = 0; i < sizeof(help_info) / sizeof(char*); i++)
          {
              rt_kprintf("%s\n", help_info[i]);
          }
          rt_kprintf("\n");
      }
  }
  return RT_EOK;
}
MSH_CMD_EXPORT(beat_cmd,beat_cmd);
#endif /*RT_USING_MSH*/