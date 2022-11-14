/**
  ******************************************************************************
  * @file    mb_key.c
  * @brief   mb按键驱动v1.0
  * @date    2021.01.10
  ******************************************************************************
  * @attention  把mb线圈抽象为按键
  mbkey处理方式：1.单独创建一个线程，查询方式循环运行。 【简单，易编写】
                 2.为每个按键创建一个线程，mbkey发送信号量，线程在恢复处理
                 【维护困难，创建无用线程过多】
  * @author hly
  ******************************************************************************
  */
/* includes ------------------------------------------------------------------*/

/* private includes ----------------------------------------------------------*/
#include "modbus_slave_common.h"
#include "motor_control.h"
/*ulog include*/
#define log_tag              "mb key"
#define log_lvl              dbg_info
#include <ulog.h>
/* private typedef -----------------------------------------------------------*/
/** 
  * @brief  按键注册表
  */
typedef enum
{
	KEY_MOTOR_ENABLE,                //电机使能
  MBKEY_NUM,// 必须要有的记录按钮数量，必须在最后
}mbkey_list;
/** 
  * @brief  按键状态机的五种状态
  */ 
typedef enum
{
  MBKEY_ENABLE,   // 使能，赋值为1
	MBKEY_DISABLE,  // 失能，赋值为0
	MBKEY_PRESS,    // 按键按下
	MBKEY_RAISE,    // 按键抬起
}mbkey_status;
/** 
  * @brief  线圈寄存器状态
  */  
typedef enum 
{
  LOW_LEVEL = 0u, 
  HIGH_LEVEL = !LOW_LEVEL
} bits_status;
/** 
  * @brief  按键屏蔽标志
  */  
typedef enum 
{
  DISABLE = 0u, 
  ENABLE = !DISABLE
} mbkey_enable_status;
/** @defgroup gpio_pull_define gpio pull define
 * @brief gpio pull-up or pull-down activation
 * @{
 */  
typedef enum
{
  PULLUP       = 0x00000001u,   //置一进入事件
  PULLDOWN     = 0x00000002u,   //置零进入事件
}bits_activation;
/** 
  * @brief  状态机初始化按键
  */  
typedef struct
{
	uint32_t gpio_pull;		//激活电平
	uint16_t index;	      //索引
	uint16_t sub_index;	  //子索引
	uint8_t  key_nox;
}config;
/** 
  * @brief  状态机类
  */
typedef struct
{
    mbkey_enable_status 	shield; 		//按键屏蔽，disable(0):屏蔽，enable(1):不屏蔽
    uint8_t             	timecount;  //按键长按计数
    bits_status 	        flag;       //标志按键按下标志
    bits_status 	        down_level; //按下时，按键io实际的电平
    mbkey_status          key_status; //按键状态
    mbkey_status          key_event;  //按键事件
    bits_status           (*read_pin)(config bits);//读io电平函数
}components;
/** 
  * @brief  按键类
  */
typedef struct
{
	config 				      board; // 继承初始化父类
	components 	        status; // 继承状态机父类
}mbkey;
/* private define ------------------------------------------------------------*/
/* 线程配置 */
#define THREAD_PRIORITY      4//线程优先级
#define THREAD_TIMESLICE     10//线程时间片
#define THREAD_STACK_SIZE    1024//栈大小
/* private macro -------------------------------------------------------------*/
/*
 * 拼接16位为32位
 * H:16位高位
 * L:16位低位
*/
#define MAKEINT_32(H,L) (uint32_t)((uint16_t)(H) << 16 | (uint16_t)(L))
/*
 * 非零赋值
 * x:判断是否为零数，并是写入数
 * y:被写入数
*/
#define NON_ZERO_W(X,Y)   \
  if(!(Y)){               \
  (X) = (Y);}
/* private variables ---------------------------------------------------------*/
static mbkey mbkey_buf[MBKEY_NUM];	// 创建按键数组
static uint8_t nodeID;//节点ID
static MODE_OPERATION motor_mode;//电机模式
/* private function prototypes -----------------------------------------------*/
static void key_motor_enable(mbkey_status *event)
{
  switch(*event)
  {
    case MBKEY_ENABLE:  //按下处理事件
    {
      switch(motor_mode)
      {
        case PROFILE_POSITION_MODE://位置规划模式
        {
          int32_t position  = MAKEINT_32(modbus_register_get(0,17),
                                        modbus_register_get(0,18));
          int16_t speed     = 60;
          
          NON_ZERO_W(speed,modbus_register_get(0,19));
          bool abs_rel      = modbus_register_get(0,20);
          bool immediately  = modbus_register_get(0,21);

          motor_profile_position(position,speed,abs_rel,immediately,nodeID);
        }
        break;
        case PROFILE_VELOCITY_MODE://速度规划模式
        {
          int16_t speed = modbus_register_get(0,17);
          motor_profile_velocity(speed,nodeID);
        }
        break;
        case PROFILE_TORQUE_MODE://扭矩规划模式
        break;
        case HOMING_MODE://原点复归模式
        {
          bool zero_flag = modbus_register_get(0,17);
          motor_homing_mode(zero_flag,nodeID);
        }
        break;
        case INTERPOLATED_POSITION_MODE://插补位置模式
//        motor_interpolation_position();
        break;
      }
    }
    break;
    case MBKEY_DISABLE: //松开处理事件
    break;
    case MBKEY_PRESS:   //松开到按下事件
    {
      switch(motor_mode)
      {
        case PROFILE_POSITION_MODE://位置规划模式
          motor_on_profile_position(nodeID);
        break;
        case PROFILE_VELOCITY_MODE://速度规划模式
          motor_on_profile_velocity(nodeID);
        break;
        case PROFILE_TORQUE_MODE://扭矩规划模式
        break;
        case HOMING_MODE://原点复归模式
        {
          int32_t offset      = MAKEINT_32(modbus_register_get(0,12),
                                           modbus_register_get(0,13));
          uint8_t method      = 34;
          float switch_speed  = 100;
          float zero_speed    = 20;

          NON_ZERO_W(method,modbus_register_get(0,14));
          NON_ZERO_W(method,modbus_register_get(0,modbus_register_get(0,15) / 10.0f));
          NON_ZERO_W(method,modbus_register_get(0,modbus_register_get(0,16) / 10.0f));

          motor_on_homing_mode(offset,method,switch_speed,zero_speed,nodeID);
        }
        break;
        case INTERPOLATED_POSITION_MODE://插补位置模式
//        motor_interpolation_position();
        break;
      }
    }
    break;
    case MBKEY_RAISE:   //按下到松开事件
      motor_off(nodeID);
      modbus_register_reset(0,11,20);//清除数据
    break;    
  }
}
/**************************状态机**********************************************/
/** 
  * @brief  获取io电平的函数
  按键读取函数
  */  
static bits_status readpin(config bits)
{
  return (bits_status)modbus_bits_get(bits.index,bits.sub_index);
}
/**
  * @brief  读取按键值
  * @param  none.
  * @retval none.
  * @note   根据实际按下按钮的电平去把它换算成虚拟的结果
*/
static void get_level(void)
{
    for(uint8_t i = 0;i < MBKEY_NUM;i++)
    {
        if(mbkey_buf[i].status.shield == DISABLE)	//如果挂起则不进行按键扫描
            continue;
        if(mbkey_buf[i].status.read_pin(mbkey_buf[i].board) == mbkey_buf[i].status.down_level)
            mbkey_buf[i].status.flag = LOW_LEVEL;
        else
            mbkey_buf[i].status.flag = HIGH_LEVEL;
    }
}
/**
  * @brief  创建按键对象
  * @param  mbkey_init
  * @retval none.
  * @note   创建按键对象
*/
static void creat_key(config* init)
{
  for(uint8_t i = 0;i < MBKEY_NUM;i++)
	{
		mbkey_buf[i].board = init[i]; // mbkey_buf按钮对象的初始化属性赋值

		mbkey_buf[i].board.key_nox = i;
		// 初始化按钮对象的状态机属性
		mbkey_buf[i].status.shield = ENABLE;
		mbkey_buf[i].status.timecount = 0;	
		mbkey_buf[i].status.flag = LOW_LEVEL;
    
		if(mbkey_buf[i].board.gpio_pull == PULLUP) // 根据模式进行赋值
			mbkey_buf[i].status.down_level = LOW_LEVEL;
		else
			mbkey_buf[i].status.down_level = HIGH_LEVEL;
    
		mbkey_buf[i].status.key_status = 	MBKEY_DISABLE;
		mbkey_buf[i].status.key_event	= 	MBKEY_DISABLE;
		mbkey_buf[i].status.read_pin 	= 	readpin;	//赋值按键读取函数
	}
}
/**
  * @brief  读取函数
  * @param  none.
  * @retval none.
  * @note   状态机的状态转换
*/
static void read_status(void)
{
  get_level();
  for(uint8_t i = 0;i < MBKEY_NUM;i++)
  {
    switch(mbkey_buf[i].status.key_status)
    {
      //状态0：按键送开
      case MBKEY_DISABLE:
        if(mbkey_buf[i].status.flag == LOW_LEVEL)
        {
            mbkey_buf[i].status.key_status = MBKEY_DISABLE;  //转入状态3
            mbkey_buf[i].status.key_event  = MBKEY_DISABLE;  //空事件
        }
        else
        {
          mbkey_buf[i].status.key_status = MBKEY_PRESS;        //转入状态1
          mbkey_buf[i].status.key_event 	= MBKEY_PRESS;        //过渡事件
        }
        break;
			//状态1：按键按下
      case MBKEY_ENABLE:
        if(mbkey_buf[i].status.flag == HIGH_LEVEL)
        {
            mbkey_buf[i].status.key_status = MBKEY_ENABLE;     //转入状态3
            mbkey_buf[i].status.key_event  = MBKEY_ENABLE;     //空事件
        }
        else
        {
          mbkey_buf[i].status.key_status = MBKEY_RAISE;        //转入状态0
          mbkey_buf[i].status.key_event  = MBKEY_RAISE;        //过渡事件
        }
        break;
      //状态2：按键过渡[送开到按下]
      case MBKEY_PRESS:
        if(mbkey_buf[i].status.flag == HIGH_LEVEL)
        {
            mbkey_buf[i].status.key_status = MBKEY_ENABLE;     //转入状态3
            mbkey_buf[i].status.key_event  = MBKEY_ENABLE;     //空事件
        }
        else
        {
            mbkey_buf[i].status.key_status = MBKEY_DISABLE;  //转入状态3
            mbkey_buf[i].status.key_event  = MBKEY_DISABLE;  //空事件
        }
        break;
			//状态1：按键过渡[按下到送开]
			case MBKEY_RAISE:
				if(mbkey_buf[i].status.flag == LOW_LEVEL)          //按键释放，端口高电平
        {
            mbkey_buf[i].status.key_status = MBKEY_DISABLE;  //转入状态3
            mbkey_buf[i].status.key_event  = MBKEY_DISABLE;  //空事件
        }
				else
        {
            mbkey_buf[i].status.key_status = MBKEY_ENABLE;     //转入状态3
            mbkey_buf[i].status.key_event = MBKEY_ENABLE;      //空事件
        }
        break;
    }
  }
}
/**
  * @brief  函数指针数组.
  * @param  none.
  * @retval none.
  * @note   
注意函数的顺序和数组越界问题
https://blog.csdn.net/feimeng116/article/details/107515317
*/
static void (*operation[MBKEY_NUM])(mbkey_status *event) = 
{
  key_motor_enable,
};
/**
  * @brief  处理函数
  * @param  none.
  * @retval none.
  * @note   放在定时器1ms一次
            当初始化值进入mb模式才运行
            扫描按键后，读取按键状态与事件进行判读处理
*/
void mbkey_handler(void *p)
{
	uint8_t i;
  while(1)
  {
    rt_thread_mdelay(1);
    read_status();
    nodeID = modbus_register_get(0,1);
    motor_mode = modbus_register_get(0,11);
    for(i = 0;i < MBKEY_NUM;i++)
    {
        operation[i](&mbkey_buf[i].status.key_event);
    }
  }
}
/**
  * @brief  mbkey按键屏蔽操作
  * @param  num:mbkey_list注册表中选项
  * @param  option: enable  ：启用。
                    disable ：禁用。
  * @retval none.
  * @note   禁用或者启用
*/
void mbkey_shield_operate(uint8_t num,mbkey_enable_status option)
{
  mbkey_buf[num].status.shield       = option;
  mbkey_buf[num].status.key_event    = MBKEY_DISABLE;//退出刹车事件
}
/**
  * @brief  io初始化初始化
  * @param  none.
  * @retval none.
  * @note   状态机初始化
gpio_pullup：初始给高，端口，位口
*/
static int mbkey_init(void)
{ 
  rt_err_t ret = RT_EOK; 
  config init[MBKEY_NUM]=
  { 
    //激活电平 索引 子索引
    {PULLUP,    0,    1}, //电机使能
  };
  creat_key(init);// 调用按键初始化函数
  /* 创建 MODBUS线程*/
  rt_thread_t thread = rt_thread_create( "mb_key",    /* 线程名字 */
                                         mbkey_handler,/* 线程入口函数 */
                                         RT_NULL,       /* 线程入口函数参数 */
                                         THREAD_STACK_SIZE, /* 线程栈大小 */
                                         THREAD_PRIORITY,   /* 线程的优先级 */
                                         THREAD_TIMESLICE); /* 线程时间片 */
  /* 创建成功则启动线程 */
  if (thread != RT_NULL)
  {
      rt_thread_startup(thread);
  }
  else
  {
      ret = RT_ERROR;
      LOG_E("modbus slave created failed.");
  }
  return ret;
}
INIT_APP_EXPORT(mbkey_init);