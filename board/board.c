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
#include "board.h"
/* Private includes ----------------------------------------------------------*/
#include "main.h"

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
//#define FINSH_IRQ_PRIORITY  
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
  * @brief  设置RTC时间
  * @param  None
  * @retval ulog时间戳准确
  */
static int Set_RTC_Time(void)
{
#include <time.h>
  rt_err_t ret = RT_EOK;
  time_t now;

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

  /* 设置日期 */
  ret = set_date(YEAR, MONTH + 1, DAY);
  if (ret != RT_EOK)
  {
    LOG_E("set RTC date failed");
  }
  /* 设置时间 */
  ret = set_time(HOUR, MINUTE, SEC + BURN_TIME);
  if (ret != RT_EOK)
  {
    LOG_E("set RTC time failed");
  }
  return ret;
}
INIT_COMPONENT_EXPORT(Set_RTC_Time);