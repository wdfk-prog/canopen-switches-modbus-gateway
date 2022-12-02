/**
 * @file modbus_slave_key.c
 * @brief 
 * @author HLY (1425075683@qq.com)
 * @version 1.0
 * @date 2022-11-17
 * @attention 
 * @copyright Copyright (c) 2022
 * @par 修改日志:
 * Date       Version Author  Description
 * 2022-11-17 1.0     HLY     mb线圈抽象为按键
 * 2022-11-23 1.1     HLY     加互斥量保护共享资源，通讯效率没有明显变化
 */
/* includes ------------------------------------------------------------------*/

/* private includes ----------------------------------------------------------*/
#include "modbus_slave_common.h"
#include "master402_canopen.h"
#include "motor_control.h"
#include "motor.h"
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
  KEY_MOTOR_DISABLE,               //电机使能
  KEY_TURN_CONTROL,                //转向电机控制
  KEY_WALK_CONTROL,                //行走电机控制
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
  MB_DISABLE = 0u, 
  MB_ENABLE = !MB_DISABLE
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
#define THREAD_PRIORITY      6//线程优先级
#define THREAD_TIMESLICE     10//线程时间片
#define THREAD_STACK_SIZE    1024//栈大小
/* private macro -------------------------------------------------------------*/
/**
  * @brief  保持寄存器写入数值
  * @param  index:索引
  * @param  sub_index:子索引
  * @param  value:值
  * @param  factor:放大系数
  * @retval none.
  * @note   当前保持寄存器为0，写入value。否则将保持寄存器值写入value中
*/
#define REG_WRITE_VALUE(index,sub_index,value,factor)                 \
if(modbus_get_register((index),(sub_index)) == 0){                    \
modbus_set_register((index),(sub_index),(value) * (factor));}         \
else{(value) = modbus_get_register((index),(sub_index)) / (factor);}  \
/* private variables ---------------------------------------------------------*/
static mbkey mbkey_buf[MBKEY_NUM];	// 创建按键数组
/* private function prototypes -----------------------------------------------*/
/** 
  * @brief  电机使能控制
  * @param  event      
  * @note   写入True使能电机控制,优先级小于禁用电机。
  * @attention 
注意置一后,对电机模式设置前，请注意电机运动参数是否修改为合适值。
电机运动参数被多个控制模式所共用，并且有进行数据拼接操作。
*/ 
static void key_motor_enable(mbkey_status *event)
{
  if(*mb_can.nodeID == MASTER_NODEID || *mb_can.nodeID > MAX_NODE_COUNT || *mb_can.nodeID == 0)
  {
    MB_DEBUG_MOTOR_ENABLE_RESET;
    return;
  }
  switch(*event)
  {
  case MBKEY_ENABLE:  //按下处理事件
  {
    switch(*mb_can.motor_mode)
    {
    case PROFILE_POSITION_MODE://位置规划模式
    {
      int16_t speed     = 60;
      int32_t position  = MAKEINT_32(modbus_get_register(0,18),
                                      modbus_get_register(0,17));
      
      REG_WRITE_VALUE(0,19,speed,1);
      bool abs_rel      = modbus_get_register(0,20);
      bool immediately  = modbus_get_register(0,21);

      if(motor_profile_position(position,speed,abs_rel,immediately,*mb_can.nodeID) == 0XFF)
      {
        motor_on_profile_position(*mb_can.nodeID);
      }
      else
      {
        MB_DEBUG_MOTOR_ENABLE_RESET;
      }
    }
    break;
    case PROFILE_VELOCITY_MODE://速度规划模式
    {
      int16_t speed = modbus_get_register(0,17);
      
      if(motor_profile_velocity(speed,*mb_can.nodeID) == 0xFF)
      {
        motor_on_profile_velocity(*mb_can.nodeID);
      }
    }
    break;
    case PROFILE_TORQUE_MODE://扭矩规划模式
    break;
    case HOMING_MODE://原点复归模式 
    {
      int16_t speed = 60;
      bool zero_flag = modbus_get_register(0,17);
      REG_WRITE_VALUE(0,18,speed,1);

      if(motor_homing_mode(zero_flag,speed,*mb_can.nodeID) >= 0XFD)//第一次配置或者需要回零未设置偏移值重新配置
      {
        int32_t offset = MAKEINT_32(*mb_can.offset_h,*mb_can.offset_l);
        motor_on_homing_mode(offset,*mb_can.method,*mb_can.switch_speed,*mb_can.zero_speed,*mb_can.nodeID);
      }
      else
      {
        MB_DEBUG_MOTOR_ENABLE_RESET;
      }
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
  break;
  case MBKEY_RAISE:   //按下到松开事件
  {
    if(*mb_can.motor_mode == PROFILE_VELOCITY_MODE)
    {
      motor_profile_velocity(0,*mb_can.nodeID);
    }
  }
  break;    
  }
}
/** 
  * @brief  电机禁用控制
  * @param  event 
  * @note   写入True禁用电机控制,优先级高于使用电机。禁用电机将关闭电机使能状态。
  */ 
static void key_motor_disable(mbkey_status *event)
{
  if(*mb_can.nodeID == MASTER_NODEID || *mb_can.nodeID > MAX_NODE_COUNT || *mb_can.nodeID == 0)
  {
    MB_DEBUG_MOTOR_DISABLE_SET;
    return;
  }
  switch(*event)
  {
  case MBKEY_ENABLE:  //按下处理事件
    MB_DEBUG_MOTOR_ENABLE_RESET;//强制退出电机使能控制
  break;
  case MBKEY_DISABLE: //松开处理事件
  break;
  case MBKEY_PRESS:   //松开到按下事件
    motor_off(*mb_can.nodeID);
    modbus_reset_register(0,17,0,30);//清除数据
  break;
  case MBKEY_RAISE:   //按下到松开事件
  break;
  }
}
/** 
  * @brief  转向电机[0]控制
  * @param  event 
  * @note   写入1使能电机后，可进行电机控制。
  *         写入0禁用电机，禁用电机将关闭电机使能状态。
  */ 
static void key_turn0_control(mbkey_status *event)
{
  turn_motor_typeDef *p = &turn_motor[0];

  if(p->nodeID  == MASTER_NODEID || p->nodeID  > MAX_NODE_COUNT || p->nodeID  == 0)
  {
    return;
  }
  if(getNodeState(OD_Data,p->nodeID) != Operational)
  {
    return;
  }

  switch(*event)
  {
  case MBKEY_ENABLE:  //按下处理事件
  {
    int32_t angle = MAKEINT_32(*p->mb.angle_h,*p->mb.angle_l) / 1000.0f;//单位:0.001°
    float   speed = *p->mb.speed / 10;//单位:0.1RPM
    turn_motor_angle_control(angle,speed,p);
  }
    break;
  case MBKEY_DISABLE: //松开处理事件
    turn_motor_stop(p);
    break;
  case MBKEY_PRESS:   //松开到按下事件
    turn_motor_enable(p);
    //更新缓存,保证初始与进入时角度准确
    turn_motor_reentrant(p);
    break;
  case MBKEY_RAISE:   //按下到松开事件
    turn_motor_disable(p);
    break;    
  }
}
/** 
  * @brief  行走电机[0]控制
  * @param  event 
  * @note   写入1使能电机后，可进行电机控制。
  *         写入0禁用电机，禁用电机将关闭电机使能状态。
  */ 
static void key_walk0_control(mbkey_status *event)
{
  walk_motor_typeDef *p = &walk_motor[0];

  if(p->nodeID  == MASTER_NODEID || p->nodeID  > MAX_NODE_COUNT || p->nodeID  == 0)
  {
    return;
  }
  if(getNodeState(OD_Data,p->nodeID) != Operational)
  {
    return;
  }

  switch(*event)
  {
  case MBKEY_ENABLE:  //按下处理事件
  { 
    float   speed = *p->mb.speed / 10;//单位:0.1RPM
    walk_motor_speed_control(speed,p);
  }
    break;
  case MBKEY_DISABLE: //松开处理事件
    break;
  case MBKEY_PRESS:   //松开到按下事件
    walk_motor_enable(p);
    walk_motor_reentrant(p);
    break;
  case MBKEY_RAISE:   //按下到松开事件
    walk_motor_disable(p);
    break;    
  }
}
/**************************状态机**********************************************/
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
  key_motor_disable,
  key_turn0_control,
  key_walk0_control,
};
/** 
  * @brief  获取io电平的函数
  按键读取函数
  */  
static bits_status readpin(config bits)
{
  return (bits_status)modbus_get_bits(bits.index,bits.sub_index);
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
		mbkey_buf[i].status.shield = MB_ENABLE;
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
  * @brief  处理函数
  * @param  p.
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
    rt_thread_mdelay(10);
    modbus_mutex_lock();
    read_status();
    for(i = 0;i < MBKEY_NUM;i++)
    {
        if(operation[i] != RT_NULL)
          operation[i](&mbkey_buf[i].status.key_event);
    }
    modbus_mutex_unlock();
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
  mbkey_buf[num].status.key_event    = MBKEY_DISABLE;//退出事件
}
/**
  * @brief  io初始化初始化
  * @param  none.
  * @retval int.
  * @note   状态机初始化
gpio_pullup：初始给高，端口，位口
*/
int mbkey_init(void)
{ 
  rt_err_t ret = RT_EOK; 
  config init[MBKEY_NUM]=
  { 
    //激活电平 索引 子索引
    {PULLUP,    0,    1},  //电机使能
    {PULLUP,    0,    2},  //电机禁用
    {PULLUP,    0,    11}, //转向电机[1]控制
    {PULLUP,    0,    15}, //行走电机[1]控制
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
