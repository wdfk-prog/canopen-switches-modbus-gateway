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
#include "led.h"
/* Private includes ----------------------------------------------------------*/
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "signal_led.h"
#include "user_math.h"
#include "main.h"
#include "mb_handler.h"
#include "monitor.h"
/* Private typedef -----------------------------------------------------------*/
/** 
  * @brief LED模式定义
  */  
typedef enum
{
  NORMAL_MODE = 0X00,//正常模式
  IPC_BEAT_MODE,     //工控机心跳异常
  MOTOR_ALM_MODE,    //电机报警异常
  VOLTAGE_MODE,      //电压错误
}LED_MODE;
/* Private define ------------------------------------------------------------*/
#define LED_PORT      LED_GPIO_Port
#define LED_PIN       LED_Pin
#define BRIGHT_STATE  GPIO_PIN_RESET

#define BEEP_PORT     Beep_GPIO_Port
#define BEEP_PIN      Beep_Pin
#define BEEP_STATE    GPIO_PIN_SET
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
//定义内存操作函数接口
led_mem_opreation_t led_mem_opreation;

LED_MODE led_alm_state = NORMAL_MODE;
/* Private function prototypes -----------------------------------------------*/
#if (BEEP_ENABLE == 1)
/*************************蜂鸣器**************************************************/
//定义信号灯对象句柄
led_t *beep =  NULL;
/*  设置信号灯一个周期内的闪烁模式
 *  格式为 “亮、灭、亮、灭、亮、灭 …………” 长度不限
 *  注意：  该配置单位为毫秒，且必须大于 “LED_TICK_TIME” 宏，且为整数倍（不为整数倍则向下取整处理）
 *          必须以英文逗号为间隔，且以英文逗号结尾，字符串内只允许有数字及逗号，不得有其他字符出现
 */
char *beep_blink_mode_0 = "500,500,";                        //1Hz闪烁
char *beep_blink_mode_1 = "200,200,100,500";                 //两次闪烁后长灭
char *beep_blink_mode_2 = "200,200,200,200,200,500";         //三次闪烁后长灭
char *beep_blink_mode_3 = "200,200,200,200,200,200,200,500"; //四次闪烁后长灭
char *beep_blink_mode_off = "0,100,";   //常灭
char *beep_blink_mode_on = "100,0,";    //常亮
/**
  * @brief  用户设置模式
  * @param  None.
  * @retval None.
  * @note   -1，没有次数限制
  运行时间 =  次数 *模式时间；ms
*/
static void User_BEEP_Setmode(LED_MODE mode)
{
  switch(mode)
  {
    case NORMAL_MODE://正常
      led_set_mode(beep, LOOP_PERMANENT, beep_blink_mode_off);
      break;
    case IPC_BEAT_MODE: //心跳异常
       led_set_mode(beep, LOOP_PERMANENT, beep_blink_mode_1);
      break;
    case MOTOR_ALM_MODE://电机报警异常
       led_set_mode(beep, LOOP_PERMANENT, beep_blink_mode_2);
      break;
    case VOLTAGE_MODE://电压异常【温度异常】
       led_set_mode(beep, LOOP_PERMANENT, beep_blink_mode_3);
      break;
    default:
      break;
  }
  led_start(beep);
}
/**
  * @brief  定义开灯函数.
  * @param  None.
  * @retval None.
  * @note   None.
*/
static void beep_switch_on(void* p)
{
  HAL_GPIO_WritePin(BEEP_PORT,BEEP_PIN, BEEP_STATE);
}
/**
  * @brief  定义关灯函数.
  * @param  None.
  * @retval None.
  * @note   None.
*/
static void beep_switch_off(void* p)
{
  HAL_GPIO_WritePin(BEEP_PORT,BEEP_PIN,GPIO_TURN(BEEP_STATE));
}
/**
  * @brief  回调函数.
  * @param  None.
  * @retval None.
  * @note   None.
*/
static void beep_over_callback(led_t *led_handler)
{
  led_set_mode(led_handler, LOOP_PERMANENT, beep_blink_mode_off);
  led_start(led_handler);
}
#endif
/*************************LED**************************************************/
//定义信号灯对象句柄
led_t *led =  NULL;
/*  设置信号灯一个周期内的闪烁模式
 *  格式为 “亮、灭、亮、灭、亮、灭 …………” 长度不限
 *  注意：  该配置单位为毫秒，且必须大于 “LED_TICK_TIME” 宏，且为整数倍（不为整数倍则向下取整处理）
 *          必须以英文逗号为间隔，且以英文逗号结尾，字符串内只允许有数字及逗号，不得有其他字符出现
 */
char *led_blink_mode_0 = "500,500,";                        //1Hz闪烁
char *led_blink_mode_1 = "200,200,100,500";                 //两次闪烁后长灭
char *led_blink_mode_2 = "200,200,200,200,200,500";         //三次闪烁后长灭
char *led_blink_mode_3 = "200,200,200,200,200,200,200,500"; //四次闪烁后长灭
char *led_blink_mode_off = "0,100,";   //常灭
char *led_blink_mode_on = "100,0,";    //常亮
/**
  * @brief  用户设置模式
  * @param  None.
  * @retval None.
  * @note   -1，没有次数限制
  运行时间 =  次数 *模式时间；ms
*/
static void User_Led_Setmode(LED_MODE mode)
{
  switch(mode)
  {
    case NORMAL_MODE://正常
      led_set_mode(led, LOOP_PERMANENT, led_blink_mode_0);
      break;
    case IPC_BEAT_MODE:   //心跳异常
       led_set_mode(led, LOOP_PERMANENT, led_blink_mode_1);
      break;
    case MOTOR_ALM_MODE://电机报警异常
       led_set_mode(led, LOOP_PERMANENT, led_blink_mode_2);
      break;
    case VOLTAGE_MODE://电压异常【温度异常】
       led_set_mode(led, LOOP_PERMANENT, led_blink_mode_3);
      break;
    default:
      break;
  }
  led_start(led);
}
/**
  * @brief  定义开灯函数.
  * @param  None.
  * @retval None.
  * @note   None.
*/
static void led_switch_on(void* p)
{
  HAL_GPIO_WritePin(LED_PORT,LED_PIN, BRIGHT_STATE);
}
/**
  * @brief  定义关灯函数.
  * @param  None.
  * @retval None.
  * @note   None.
*/
static void led_switch_off(void* p)
{
  HAL_GPIO_WritePin(LED_PORT,LED_PIN,GPIO_TURN(BRIGHT_STATE));
}
/**
  * @brief  回调函数.
  * @param  None.
  * @retval None.
  * @note   None.
*/
static void blink_over_callback(led_t *led_handler)
{
    led_set_mode(led_handler, LOOP_PERMANENT, led_blink_mode_0);
    led_start(led_handler);
}
/**
  * @brief  LED运行线程
  * @param  None
  * @retval None
  * @note   
*/
static void led_run(void *parameter)
{
    while(1)
    {
      led_ticks();
      rt_thread_mdelay(LED_TICK_TIME);
    }
}
/**
  * @brief  LED初始化
  * @param  None
  * @retval None
  * @note   
*/
static int rt_led_timer_init(void)
{
/*自定义内存操作接口
 *注意：若要进行自定义内存操作，必须要在调用任何软件包内接口之前作设置，
 *      否则会出现不可意料的错误！！！
 */
    led_mem_opreation.malloc_fn = (void* (*)(size_t))rt_malloc;
    led_mem_opreation.free_fn = rt_free;
    led_set_mem_operation(&led_mem_opreation);
    
    //初始化信号灯对象
    led = led_create(led_switch_on, led_switch_off, NULL);
    User_Led_Setmode(NORMAL_MODE);
//    //设置信号灯闪烁结束回调函数
//    led_set_blink_over_callback(led,blink_over_callback);
    //开启信号灯
    led_start(led);
#if (BEEP_ENABLE == 1)
    //初始化蜂鸣器对象
    beep = led_create(beep_switch_on, beep_switch_off, NULL);
    led_set_mode(beep, 1, beep_blink_mode_0);
    //设置信号灯闪烁结束回调函数
    led_set_blink_over_callback(beep,beep_over_callback);
    //开启信号灯
    led_start(beep);
#endif
    rt_thread_t tid = RT_NULL;
    tid = rt_thread_create("signal_led",
                            led_run, 
                            RT_NULL,
                            512,
                            RT_THREAD_PRIORITY_MAX/2,
                            100);
    if (tid != RT_NULL)
        rt_thread_startup(tid);
    return RT_EOK;
}
INIT_APP_EXPORT(rt_led_timer_init);
/**
  * @brief  异常检测
  * @param  None.
  * @retval None.
  * @note   --led闪烁+蜂鸣器报警.
*/
void LED_Abnormal_Alarm(void)
{
  static LED_MODE last = NORMAL_MODE;
  if(TURN_ALARM_GET == 1 || Walk_ALARM_GET == 1 || CRASH_ALARM_GET == 1)//电机报警 三次闪烁后长灭
  {
    led_alm_state = MOTOR_ALM_MODE;
  }
  else if((IPC_Beat.flag == false  && IPC_Beat.EN == 1) 
  || (Weinview_Beat.flag == false  && Weinview_Beat.EN == 1))//工控机心跳异常 两次闪烁后长灭
  {
    led_alm_state = IPC_BEAT_MODE;
  }
  else if(ADC_ALARM_GET == 1)//电压错误 四次闪烁后长灭
  {
    led_alm_state = VOLTAGE_MODE;
  }
  else
  {
    led_alm_state = NORMAL_MODE;
  }
  if(last != led_alm_state)
  {
    User_Led_Setmode(led_alm_state);
#if (BEEP_ENABLE == 1)
    User_BEEP_Setmode(led_alm_state);
#endif
    last = led_alm_state;
  }
}