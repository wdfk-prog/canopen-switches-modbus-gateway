/**
 * @file bsp_key.c
 * @brief 
 * @author HLY (1425075683@qq.com)
 * @version 1.0
 * @date 2022-12-02
 * @copyright Copyright (c) 2022
 * @attention 
 * @par 修改日志:
 * Date       Version Author  Description
 * 2022-12-02 1.0     HLY     first version
 */
/* Includes ------------------------------------------------------------------*/
#include "bsp_key.h"
/* Private includes ----------------------------------------------------------*/
#include <ipc/ringbuffer.h>
#include <rtthread.h>
#include <board.h>
#include "mfbd.h"
#include "main.h"
/*ulog include*/
#define LOG_TAG              "bsp key" 
#define LOG_LVL              DBG_INFO
#include <ulog.h>
/* Private typedef -----------------------------------------------------------*/
/* 依次定义GPIO */
typedef struct
{
	GPIO_TypeDef* gpio;
	uint16_t pin;
	uint8_t ActiveLevel;	/* 激活电平 */
}X_GPIO_T;
/* 按键ID*/
typedef enum
{
  KID_Stop_Button = 0X00,
  KID_WK_UP,
  KID_Before_Radar_1,
  KID_Before_Radar_2,
  KID_Before_Radar_3,
  KID_Start_Button,
  KID_Article_Crash,
  KID_Turn_ALM,
  KID_Walk_ALM,
  KID_Lift_SQP,
  KID_Lift_Reflection,
  KID_Lower_Limit,
  HARD_KEY_NUM,// 实体按键个数
}KEY_ID;
/*
	定义键值代码, 必须按如下次序定时每个键的按下、弹起和长按事件

	推荐使用enum, 不用#define，原因：
	(1) 便于新增键值,方便调整顺序，使代码看起来舒服点
	(2) 编译器可帮我们避免键值重复。
*/
typedef enum
{
	KEY_NONE = 0X00,			      /* 0 表示按键事件 */
  
  Stop_Button_DOWN_CODE,				   
  Stop_Button_UP_CODE,				    
  Stop_Button_LONG_CODE,				   
  
  WK_UP_DOWN_CODE,				         
  WK_UP_UP_CODE,				           
  WK_UP_LONG_CODE,				         
  
  Before_Radar_1_DOWN_CODE,				
  Before_Radar_1_UP_CODE,				  
  Before_Radar_1_LONG_CODE,				

  Before_Radar_2_DOWN_CODE,				
  Before_Radar_2_UP_CODE,				 
  Before_Radar_2_LONG_CODE,				
  
  Before_Radar_3_DOWN_CODE,			
  Before_Radar_3_UP_CODE,				
  Before_Radar_3_LONG_CODE,			
  
  Start_Button_DOWN_CODE,				 
  Start_Button_UP_CODE,				   
  Start_Button_LONG_CODE,				 
  
  Article_Crash_DOWN_CODE,				
  Article_Crash_UP_CODE,				 
  Article_Crash_LONG_CODE,				

  Turn_ALM_DOWN_CODE,				      
  Turn_ALM_UP_CODE,				        
  Turn_ALM_LONG_CODE,				       
  
  Walk_ALM_DOWN_CODE,				        
  Walk_ALM_UP_CODE,				          
  Walk_ALM_LONG_CODE,				        
  
  Lift_SQP_DOWN_CODE,				    
  Lift_SQP_UP_CODE,				      
  Lift_SQP_LONG_CODE,				    

  Lift_Reflection_DOWN_CODE,				      
  Lift_Reflection_UP_CODE,				       
  Lift_Reflection_LONG_CODE,				      
  
  Lower_Limit_DOWN_CODE,				       
  Lower_Limit_UP_CODE,				         
  Lower_Limit_LONG_CODE,				       
}KEY_CODE;
/* GPIO和PIN定义 */
static const X_GPIO_T s_gpio_list[HARD_KEY_NUM] = 
{
	{Stop_Button_GPIO_Port,           Stop_Button_Pin,          GPIO_PIN_SET},	  /* 急停按钮*/
  {WK_UP_GPIO_Port,                 WK_UP_Pin,                GPIO_PIN_SET},    /* 唤醒按钮*/
  {Before_Radar_1_GPIO_Port,        Before_Radar_1_Pin,       GPIO_PIN_SET},    /* 前雷达1 */
  {Before_Radar_2_GPIO_Port,        Before_Radar_2_Pin,       GPIO_PIN_SET},    /* 前雷达2 */
  {Before_Radar_3_GPIO_Port,        Before_Radar_3_Pin,       GPIO_PIN_SET},    /* 前雷达3 */
  {Start_Button_GPIO_Port,          Start_Button_Pin,         GPIO_PIN_RESET},  /*启动按钮 */
  {Article_Crash_GPIO_Port,         Article_Crash_Pin,        GPIO_PIN_RESET},  /*防撞条   */
  {Turn_ALM_GPIO_Port     ,         Turn_ALM_Pin,             GPIO_PIN_SET},    /*转向报警 */
  {Walk_ALM_GPIO_Port,              Walk_ALM_Pin,             GPIO_PIN_RESET},  /*行走报警 */
  {Lift_SQP_GPIO_Port,              Lift_SQP_Pin,             GPIO_PIN_RESET},  /*叉臂接近开关      添加按键需修改*/
  {Lift_Reflection_GPIO_Port ,      Lift_Reflection_Pin,      GPIO_PIN_RESET},  /*叉臂漫反射传感器  添加按键需修改*/
  {Lower_Limit_GPIO_Port,           Lower_Limit_Pin,          GPIO_PIN_RESET},  /*叉臂下限位        添加按键需修改*/
};	
/* Private define ------------------------------------------------------------*/
/* 扫描线程配置 */
#define SCAN_THREAD_PRIORITY      3//线程优先级
#define SCAN_THREAD_TIMESLICE     10 //线程时间片
#define SCAN_THREAD_STACK_SIZE    512//栈大小
/* 处理线程配置 */
#define HANDLER_THREAD_PRIORITY   4//线程优先级
#define HANDLER_THREAD_TIMESLICE  10 //线程时间片
#define HANDLER_THREAD_STACK_SIZE 1024//栈大小

#define RINGBUFFER_LEN       10//ringbuffer缓冲区大小
#define SCAN_PERIOD          10//扫描周期 单位ms
#define HANDLER_PERIOD       50//处理周期 单位ms
/* Private macro -------------------------------------------------------------*/
/*支持Button单击、长按的操作，不支持多次连击操作*/
#if MFBD_USE_NORMAL_BUTTON//重复时间与长按时间请大于滤波时间
/*                        名称,             按键索引,          滤波时间,        重复时间，     长按时间*/
MFBD_NBTN_DEFAULT_DEFINE(Stop_Button,     KID_Stop_Button,    50/SCAN_PERIOD,100/SCAN_PERIOD, 100/SCAN_PERIOD);//持续读取
MFBD_NBTN_DEFAULT_DEFINE(WK_UP,           KID_WK_UP,          50/SCAN_PERIOD,100/SCAN_PERIOD, 1000/SCAN_PERIOD);//持续读取
MFBD_NBTN_DEFAULT_DEFINE(Before_Radar_1,  KID_Before_Radar_1, 50/SCAN_PERIOD,0/SCAN_PERIOD,   0/SCAN_PERIOD); //触发一次
MFBD_NBTN_DEFAULT_DEFINE(Before_Radar_2,  KID_Before_Radar_2, 50/SCAN_PERIOD,0/SCAN_PERIOD,   0/SCAN_PERIOD); //触发一次
MFBD_NBTN_DEFAULT_DEFINE(Before_Radar_3,  KID_Before_Radar_3, 50/SCAN_PERIOD,0/SCAN_PERIOD,   0/SCAN_PERIOD); //触发一次
MFBD_NBTN_DEFAULT_DEFINE(Start_Button,    KID_Start_Button  , 50/SCAN_PERIOD,0/SCAN_PERIOD,   1000/SCAN_PERIOD);//一次长按
MFBD_NBTN_DEFAULT_DEFINE(Article_Crash,   KID_Article_Crash,  50/SCAN_PERIOD,0/SCAN_PERIOD,   100/SCAN_PERIOD);//持续读取
MFBD_NBTN_DEFAULT_DEFINE(Lift_SQP,        KID_Lift_SQP,       50/SCAN_PERIOD,100/SCAN_PERIOD, 100/SCAN_PERIOD);//持续读取
MFBD_NBTN_DEFAULT_DEFINE(Lift_Reflection, KID_Lift_Reflection,50/SCAN_PERIOD,100/SCAN_PERIOD, 100/SCAN_PERIOD);//持续读取
MFBD_NBTN_DEFAULT_DEFINE(Lower_Limit,     KID_Lower_Limit,    50/SCAN_PERIOD,0/SCAN_PERIOD, 0/SCAN_PERIOD);//触发一次
#endif /* MFBD_USE_NORMAL_BUTTON */
#if MFBD_USE_NORMAL_BUTTON
MFBD_NBTN_ARRAYLIST(nbtn_list, 
                    &Stop_Button, 
                    &WK_UP,
                    &Before_Radar_1,
                    &Before_Radar_2,
                    &Before_Radar_3,
                    &Start_Button,
                    &Article_Crash,
                    &Lift_SQP,
                    &Lift_Reflection,
                    &Lower_Limit
                    );
#endif /* MFBD_USE_NORMAL_BUTTON */
/* Private variables ---------------------------------------------------------*/
static void bsp_btn_value_report(mfbd_btn_code_t btn_value);
static unsigned char bsp_btn_check(mfbd_btn_index_t btn_index);
//环形缓冲区指针
struct rt_ringbuffer * rb;
const mfbd_group_t btn_group =
{
    bsp_btn_check,
    bsp_btn_value_report,

#if MFBD_USE_TINY_BUTTON
    test_tbtn_list,
#endif /* MFBD_USE_TINY_BUTTON */

#if MFBD_USE_NORMAL_BUTTON
    nbtn_list,
#endif /* MFBD_USE_NORMAL_BUTTON */

#if MFBD_USE_MULTIFUCNTION_BUTTON
    test_mbtn_list,
#endif /* MFBD_USE_MULTIFUCNTION_BUTTON */

#if MFBD_PARAMS_SAME_IN_GROUP

#if MFBD_USE_TINY_BUTTON || MFBD_USE_NORMAL_BUTTON || MFBD_USE_MULTIFUCNTION_BUTTON
    3,
#endif /*  MFBD_USE_TINY_BUTTON || MFBD_USE_NORMAL_BUTTON || MFBD_USE_MULTIFUCNTION_BUTTON */

#if MFBD_USE_NORMAL_BUTTON || MFBD_USE_MULTIFUCNTION_BUTTON
    30,
    150,
#endif /* MFBD_USE_NORMAL_BUTTON || MFBD_USE_MULTIFUCNTION_BUTTON */

#if MFBD_USE_MULTIFUCNTION_BUTTON
    75,
#endif /* MFBD_USE_MULTIFUCNTION_BUTTON */

#endif /*MFBD_PARAMS_SAME_IN_GROUP*/

};
/* Private function prototypes -----------------------------------------------*/
/**
 * @brief  按键处理函数
 * @param  ucKeyCode        
 */
void key_handle(mfbd_btn_code_t ucKeyCode)
{
  if (ucKeyCode != KEY_NONE)
  {
    switch (ucKeyCode)
    {
      case Stop_Button_UP_CODE:
        LOG_I("stop button UP");
        break;
      case Stop_Button_DOWN_CODE: 
        break;
      case Stop_Button_LONG_CODE:
        LOG_W("stop button trigger");
        break;
      case WK_UP_UP_CODE: 
        LOG_I("WK_UP UP");
        break;
      case WK_UP_DOWN_CODE:
        LOG_I("WK_UP DOWN");
        break;
      case WK_UP_LONG_CODE:
        LOG_I("WK_UP LONG");
        break;
      case Before_Radar_1_DOWN_CODE:
        break;
      case Before_Radar_1_UP_CODE:
        break;
      case Before_Radar_2_DOWN_CODE:
        break;
      case Before_Radar_2_UP_CODE:
        break;
      case Before_Radar_3_DOWN_CODE:
        break;
      case Before_Radar_3_UP_CODE:
        break;
      case Start_Button_DOWN_CODE:
        LOG_I("Press the start button");
        break;
      case Start_Button_UP_CODE:

        break;
      case Start_Button_LONG_CODE:
        rt_kprintf("\033[31;22m[I/KEY] Long pressthe start button,MCU restart");
        LOG_W("Long pressthe start button,MCU restart");
        break;
        break;
      case Article_Crash_DOWN_CODE://软件清除报警状态
        break;
      case Article_Crash_UP_CODE:
        break;
      case Article_Crash_LONG_CODE:
        LOG_E("The article crash snesor touch");
        break;
      case Lift_SQP_UP_CODE:
        break;
      case Lift_SQP_LONG_CODE:
        LOG_I("lift SQP snesor touch");
        break;
      case Lift_Reflection_DOWN_CODE:

        break;       
      case Lift_Reflection_UP_CODE:		

        break; 
      case Lift_Reflection_LONG_CODE:
        LOG_W("lift Reflection snesor touch");
        break;
      case Lower_Limit_DOWN_CODE:	

        break;
      case Lower_Limit_UP_CODE:
        break;
      default:
        /* 其它的键值不处理 */
        break;
    }
  }
}
/******************************按键驱动********************************************/
/**
  * @brief  KeyPinActive.
  * @param  None.
  * @retval 返回值1 表示按下(导通），0表示未按下（释放）.
  * @note   判断按键是否按下.
*/
static uint8_t key_pin_active(mfbd_btn_index_t _id)
{
	uint8_t level;
	
	if ((s_gpio_list[_id].gpio->IDR & s_gpio_list[_id].pin) == 0)
	{
		level = 0;
	}
	else
	{
		level = 1;
	}

	if (level == s_gpio_list[_id].ActiveLevel)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
/**
  * @brief  按键检测.
  * @param  None.
  * @retval None.
  * @note   STM32 HAL库专用.
*/
static unsigned char bsp_btn_check(mfbd_btn_index_t btn_index)
{
    return key_pin_active(btn_index);
}
/**
  * @brief  键值反馈.
  * @param  None.
  * @retval None.
  * @note   None
*/
static void bsp_btn_value_report(mfbd_btn_code_t btn_value)
{
  LOG_D("%d", btn_value);
  //该接口会继续写入剩余的数据，即用新的数据覆盖旧的数据。
  rt_ringbuffer_put_force(rb,(rt_uint8_t *)&btn_value,sizeof(mfbd_btn_code_t));
}
/**
  * @brief  按键扫描线程.
  * @param  None.
  * @retval None.
  * @note   None
*/
static void mfbd_scan(void *arg)
{
  while (1)
  {
    mfbd_group_scan(&btn_group); /* scan button group */
    rt_thread_mdelay(SCAN_PERIOD); /* scan period: 10ms */
  }
}
/**
  * @brief  按键处理线程.
  * @param  None.
  * @retval None.
  * @note   None
*/
static void mfbd_handler(void *p)
{
  while(1)
  {
    mfbd_btn_code_t recv_data;
    rt_size_t recv = rt_ringbuffer_get(rb, (rt_uint8_t *)&recv_data, sizeof(recv_data));
    if(recv == sizeof(recv_data))
      key_handle(recv_data);    
    rt_thread_mdelay(50); /* handler period: 50ms */
  }
}
/**
  * @brief  按键初始化
  * @param  None.
  * @retval None.
  * @note   None
*/
static void user_button_init(void)
{
  //MX_GPIO_Init();//已在borad中初始化
}
/**
  * @brief  按键线程初始化
  * @param  None.
  * @retval None.
  * @note   None
*/
int mfbd_init(void)
{
  rt_err_t ret = RT_EOK;
  user_button_init();
                                                  //length	缓冲区字节大小
  rb = rt_ringbuffer_create(sizeof(mfbd_btn_code_t) * RINGBUFFER_LEN);
  RT_ASSERT(rb != RT_NULL);

  /* 创建线程*/
  rt_thread_t thread = rt_thread_create( "mfbdscan",    /* 线程名字 */
                                         mfbd_scan,/* 线程入口函数 */
                                         RT_NULL,       /* 线程入口函数参数 */
                                         SCAN_THREAD_STACK_SIZE, /* 线程栈大小 */
                                         SCAN_THREAD_PRIORITY,   /* 线程的优先级 */
                                         SCAN_THREAD_TIMESLICE); /* 线程时间片 */
  /* 创建成功则启动线程 */
  if (thread != RT_NULL)
  {
      rt_thread_startup(thread);
  }
  else
  {
      ret = RT_ERROR;
      LOG_E("mf scan created failed.");
  }
  /* 创建线程*/
  thread = rt_thread_create( "mfdbhandler",    /* 线程名字 */
                            mfbd_handler,       /* 线程入口函数 */
                            RT_NULL,            /* 线程入口函数参数 */
                    HANDLER_THREAD_STACK_SIZE,  /* 线程栈大小 */
                    HANDLER_THREAD_PRIORITY,    /* 线程的优先级 */
                    HANDLER_THREAD_TIMESLICE);  /* 线程时间片 */
  /* 创建成功则启动线程 */
  if (thread != RT_NULL)
  {
      rt_thread_startup(thread);
  }
  else
  {
      ret = RT_ERROR;
      LOG_E("mf hand created failed.");
  }
  return ret;
}