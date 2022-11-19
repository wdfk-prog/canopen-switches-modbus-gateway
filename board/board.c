/**
 * @file board.c
 * @brief 
 * @author HLY (1425075683@qq.com)
 * @version 1.0
 * @date 2022-11-18
 * @copyright Copyright (c) 2022
 * @attention 
 * @par 修改日志:
 * Date       Version Author  Description
 * 2022-11-18 1.0     HLY     first version
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "board.h"
/* Private includes ----------------------------------------------------------*/
#include <time.h>
#include "main.h"
#include "modbus_slave_common.h"
#include "filesystem.h"
#ifdef RT_USING_SERIAL
#ifdef RT_USING_SERIAL_V2
#include "drv_usart_v2.h"
#else
#include "drv_usart.h"
#endif /* RT_USING_SERIAL */
#endif /* RT_USING_SERIAL_V2 */
/*ulog include*/
#define LOG_TAG              "board" 
#define LOG_LVL              DBG_INFO
#include <ulog.h>
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define RTC_NAME       "rtc"
/*串口中断优先级设置*/
#define FINSH_IRQ_PRIORITY 3
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
#ifdef FINSH_IRQ_PRIORITY
/**
  * @brief  设置FINSH串口中断优先级
  * @param  None
  * @retval None
  */
static int Set_FINSH_IRQ(void)
{
  rt_err_t ret = RT_EOK;
  /* 串口设备句柄 */
  rt_device_t serial;
  /* 查找串口设备 */
  serial = rt_device_find(RT_CONSOLE_DEVICE_NAME);
  if (!serial)
  {
      LOG_E("find %s failed!", RT_CONSOLE_DEVICE_NAME);
      ret = -RT_ERROR;
  }
  struct stm32_uart *uart;

  uart = rt_container_of(serial, struct stm32_uart, serial);
  /* parameter check */
  RT_ASSERT(uart != RT_NULL);
  HAL_NVIC_SetPriority(uart->config->irq_type,FINSH_IRQ_PRIORITY, 0);
  return ret;
}
INIT_COMPONENT_EXPORT(Set_FINSH_IRQ);
#endif
/**
 * @brief  获取编译时间戳
 * @param  None.              
 * @retval time_t 返回时间戳
 * @note   None.
 */
static time_t rtc_get_build(void)
{
  struct tm tm_new = {0};
  /* update date. */
  tm_new.tm_year  = YEAR - 1900;// years since 1900
  tm_new.tm_mon   = MONTH;//tm_mon: 0~11
  tm_new.tm_mday  = DAY;
  /* update time. */
  tm_new.tm_hour = HOUR;
  tm_new.tm_min  = MINUTE;
  tm_new.tm_sec  = SEC;

  return mktime(&tm_new);   
}
/**
 * @brief  更新rtc时钟
 * @param  t                
 * @retval time_t 返回时间戳
 * @note   若未获取到时间，返回 0;
 */
static time_t rtc_update(void)
{
  struct tm tm_new = {0};
  /* update date. */
  tm_new.tm_year  = modbus_get_register(0,31) - 1900;// years since 1900
  tm_new.tm_mon   = modbus_get_register(0,32) - 1;//tm_mon: 0~11
  tm_new.tm_mday  = modbus_get_register(0,33);
  /* update time. */
  tm_new.tm_hour = modbus_get_register(0,34);
  tm_new.tm_min  = modbus_get_register(0,35);
  tm_new.tm_sec  = modbus_get_register(0,36);
  /* converts the local time into the calendar time. */
  if(tm_new.tm_mon < 0 || !(tm_new.tm_year + tm_new.tm_mday + tm_new.tm_hour + tm_new.tm_min + tm_new.tm_sec))
    return 0;
  else
  {
    return mktime(&tm_new);
  }     
}
/**
  * @brief  rtc_update线程.
  * @param  None.
  * @retval None.
  * @note   1秒获取一次同步时间。
*/
static void rtc_update_thread_entry(void* parameter)
{
  time_t old_time = 0,new_time = 0,err_time = 0;

  while(1)
  {
    rt_thread_mdelay(1000);
    new_time = rtc_update();

    err_time = new_time - old_time;
    if(err_time == 0)//时间值没有更新
    {
      ulog_d("RTC","Out of time synchronization");
    }
    else
    {
      set_timestamp(new_time);//更新时间，进行同步
      
    }
    rtc_time_write();
    old_time = new_time;
  }
}
/**
 * @brief   rtc_update_init
 * @retval  int 
 * @note    初始化更新rtc时间
 */
static int rtc_update_init(void)
{
    rt_thread_t tid;
    rt_err_t ret = RT_EOK;
    time_t now,build;


    rt_device_t device = RT_NULL;
    /*寻找设备*/
    device = rt_device_find(RTC_NAME);
    if (!device)
    {
        LOG_E("find %s failed!", RTC_NAME);
        return RT_ERROR;
    }
    /*初始化RTC设备*/
    if(rt_device_open(device, 0) != RT_EOK)
    {
        LOG_E("open %s failed!", RTC_NAME);
        return RT_ERROR;
    }
    
    build = rtc_get_build();//读取编译时间
    now = rtc_time_read();  //读取flash掉电时间

    if(build < now)
    {
      set_timestamp(now);
      LOG_I("Set the timestamp to the time in the flash");
    }
    else
    {
      set_timestamp(build);
      LOG_I("Set the timestamp to compile time");
    }

    tid = rt_thread_create("rtc", rtc_update_thread_entry, RT_NULL,
                          4096, 0, 0);
    if(tid == RT_NULL)
    {
      LOG_E("rtc thread start failed!");
    }
    else
    {
      rt_thread_startup(tid);
    }
    return RT_EOK;
}
INIT_APP_EXPORT(rtc_update_init);
/*********************************掉电检测******************************************/
#if (PVD_ENABLE == 1)
/* 完成量控制块 */
static struct rt_completion pvd_completion;
/**
  * @brief  None.
  * @param  None.
  * @retval None.
  * @note   None.
*/
static void pvd_thread_entry(void* parameter)
{
  while(1)
  {
      rt_completion_wait(&pvd_completion, RT_WAITING_FOREVER);
      /* 掉电前的紧急处理 */
      ulog_flush();
      rt_kprintf("Flush ULOG buffer complete\n");
  }
}
/**
 * @brief pvd初始化
 * @retval int 
 */
static int pvd_init(void)
{
    /*##-1- Enable Power Clock #################################################*/
    __HAL_RCC_PWR_CLK_ENABLE();           /* 使能PVD */
 
    /*##-2- Configure the NVIC for PVD #########################################*/
    HAL_NVIC_SetPriority(PVD_IRQn, 0, 0); /* 配置PVD中断优先级 */
    HAL_NVIC_EnableIRQ(PVD_IRQn);         /* 使能PVD中断 */
 
    /* Configure the PVD Level to 3 and generate an interrupt on rising and falling
       edges(PVD detection level set to 2.5V, refer to the electrical characteristics
       of you device datasheet for more details) */
    PWR_PVDTypeDef sConfigPVD;
    sConfigPVD.PVDLevel = PWR_PVDLEVEL_6;     /* PVD阈值3.1V */
    sConfigPVD.Mode = PWR_PVD_MODE_IT_RISING; /* 检测掉电 */
    HAL_PWR_ConfigPVD(&sConfigPVD);
 
    /* Enable the PVD Output */
    HAL_PWR_EnablePVD();

    /* 初始化完成量对象 */
    rt_completion_init(&pvd_completion);
    rt_thread_t tid;
    tid = rt_thread_create("PVD", pvd_thread_entry, RT_NULL,
                          512, 0, 20);
    if(tid == RT_NULL)
    {
      LOG_E("PVD thread start failed!");
    }
    else
    {
      rt_thread_startup(tid);
    }
    return RT_EOK;
}
INIT_APP_EXPORT(pvd_init);
/**
  * @brief  PWR PVD interrupt callback
  * @retval None
  */
void HAL_PWR_PVDCallback(void)
{
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_PWR_PVDCallback could be implemented in the user file
   */
   if(__HAL_PWR_GET_FLAG( PWR_FLAG_PVDO ))    /* 1为VDD小于PVD阈值,掉电情况 */
  {
      rt_completion_done(&pvd_completion);
      ulog_i("PVD","Voltage below 3.1V was detected");
  }
}
#endif