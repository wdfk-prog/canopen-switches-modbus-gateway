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
#include <rtthread.h>
#include <rtdevice.h>
/* Private includes ----------------------------------------------------------*/
#include "sys.h"
#include "Smove.h"
#include "turn_motor.h"
/*ulog include*/
#define LOG_TAG              "timer14" 
#define LOG_LVL              DBG_INFO
#include <ulog.h>
/* Private typedef -----------------------------------------------------------*/
struct stm32_hwtimer
{
    rt_hwtimer_t time_device;
    TIM_HandleTypeDef    tim_handle;
    IRQn_Type tim_irqn;
    char *name;
};
/* Private define ------------------------------------------------------------*/
#define HWTIMER_DEV_NAME   "timer14"     /* 定时器名称 */
/* Private macro -------------------------------------------------------------*/
#define IRQ_PRIORITY  2
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  定时器超时回调函数
  * @param  None
  * @retval None
  * @note   None
*/
static rt_err_t timeout_cb(rt_device_t dev, rt_size_t size)
{
    SMove_Handler(&turn.motor.Ctrl);
    return 0;
}
/**
  * @brief  定时器超时回调函数
  * @param  None
  * @retval None
  * @note   None
*/
static int hwtimer14_init(void)
{
    rt_err_t ret = RT_EOK;
    rt_hwtimerval_t timeout_s;      /* 定时器超时值 */
    rt_device_t hw_dev = RT_NULL;   /* 定时器设备句柄 */
    rt_hwtimer_mode_t mode;         /* 定时器模式 */
    rt_uint32_t freq = 1000000;     /* 计数频率 */

    /* 查找定时器设备 */
    hw_dev = rt_device_find(HWTIMER_DEV_NAME);
    if (hw_dev == RT_NULL)
    {
        LOG_E("hwtimer sample run failed! can't find %s device!", HWTIMER_DEV_NAME);
        return RT_ERROR;
    }

    /* 以读写方式打开设备 */
    ret = rt_device_open(hw_dev, RT_DEVICE_OFLAG_RDWR);
    if (ret != RT_EOK)
    {
        LOG_E("open %s device failedn", HWTIMER_DEV_NAME);
        return ret;
    }

    /* 设置超时回调函数 */
    rt_device_set_rx_indicate(hw_dev, timeout_cb);

    /* 设置计数频率(若未设置该项，默认为1Mhz 或 支持的最小计数频率) */
    rt_device_control(hw_dev, HWTIMER_CTRL_FREQ_SET, &freq);
    /* 设置模式为周期性定时器（若未设置，默认是HWTIMER_MODE_ONESHOT）*/
    mode = HWTIMER_MODE_PERIOD;
    ret = rt_device_control(hw_dev, HWTIMER_CTRL_MODE_SET, &mode);
    if (ret != RT_EOK)
    {
        LOG_E("set mode failed! ret is :%d", ret);
        return ret;
    }

    /* 设置定时器超时值为5s并启动定时器 */
    timeout_s.sec  = 0;      /* 秒 */
    timeout_s.usec = 5;      /* 微秒 */
    if (rt_device_write(hw_dev, 0, &timeout_s, sizeof(timeout_s)) != sizeof(timeout_s))
    {
        LOG_E("set timeout value failed");
        return RT_ERROR;
    }
#ifdef IRQ_PRIORITY
    struct stm32_hwtimer *tim_device = RT_NULL;
    tim_device = rt_container_of(hw_dev, struct stm32_hwtimer, time_device);
    
    HAL_NVIC_SetPriority(tim_device->tim_irqn,IRQ_PRIORITY, 0);
#endif
    return ret;
}
/* 导出到 msh 命令列表中 */
INIT_DEVICE_EXPORT(hwtimer14_init);