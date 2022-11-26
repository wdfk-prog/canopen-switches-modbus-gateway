/**
 * @file motor.c
 * @brief 
 * @author HLY (1425075683@qq.com)
 * @version 1.0
 * @date 2022-11-21
 * @copyright Copyright (c) 2022
 * @attention 
 * @par 修改日志:
 * Date       Version Author  Description
 * 2022-11-21 1.0     HLY     first version
 */
/* Includes ------------------------------------------------------------------*/
#include "motor.h"
/* Private includes ----------------------------------------------------------*/
#include "motor_control.h"
#include "master402_canopen.h"
#include "modbus_slave_common.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
turn_motor_typeDef turn_motor[TURN_MOTOR_NUM];
walk_motor_typeDef walk_motor[TURN_MOTOR_NUM];
/******************************行走电机函数**************************************/
/**
  * @brief  重入角度定位角度设置
  * @param  None.
  * @retval None.
  * @note   保证再次进入角度定位不会立刻进行定位。
*/
void turn_motor_reentrant(turn_motor_typeDef* p)
{
    //更新缓存,保证初始与进入时角度准确
    p->last = turn_motor_get_angle(p);
    *p->mb.angle_h = (int32_t)(p->last * 1000) >> 16;
    *p->mb.angle_l = (int32_t)(p->last * 1000);
}
/**
  * @brief  转向电机急停优先级.
  * @param  p
  * @retval None.
  * @note   
*/
static uint8_t turn_motor_stop_priority(turn_motor_typeDef* p)
{
  if(*p->stop_state != NO_STOP)
  {
    turn_motor_stop(p);
    turn_motor_reentrant(p);
    ulog_w("turn","turn motor stop,code is 0X%4.4x",*p->stop_state);
    return 1;
  }
  else
    return 0;
}
/**
 * @brief 转向电机使能
 * @param  p
 * @retval 成功返回0X00,模式错误返回0XFF.超时返回0XFE.
           触发急停返回0X01
 * @note  初始化为位置规划模式
 */
uint8_t turn_motor_enable(turn_motor_typeDef* p)
{
  return motor_on_profile_position(p->nodeID);
}
/**
 * @brief 转向电机禁用
 * @param  p
 * @retval 成功返回0X00,模式错误返回0XFF.超时返回0XFE.
           触发急停返回0X01
 * @note  关闭电机使能
 */
uint8_t turn_motor_disable(turn_motor_typeDef* p)
{
  return motor_off(p->nodeID);
}
/**
 * @brief  转向电机停止运动控制
 * @param  p   
 * @retval 成功返回0X00,模式错误返回0XFF.超时返回0XFE.
           触发急停返回0X01     
 * @note   none
 */
uint8_t turn_motor_stop(turn_motor_typeDef* p)
{
  return 0;
}
/**
 * @brief  转向电机开始运动控制
 * @param  position         
 * @param  speed 单位：RPM
 * @param  p
 * @retval 成功返回0X00,模式错误返回0XFF.超时返回0XFE.
           触发急停返回0X01
 * @note   以绝对位置模式，立刻触发指令运动      
 */
static uint8_t turn_motor_start(int32_t position,float speed,turn_motor_typeDef* p)
{
  if (turn_motor_stop_priority(p))
    return 0X01;
  
  return motor_profile_position(position,speed,0,1,p->nodeID);
}
/**
 * @brief   角度换算
 * @param  angle            
 * @retval float 
 * @note   角度换算 范围 -180~180
 */
static float angle_conversion(float angle)
{
  angle = fmod(angle,360);
  if(angle == 180)
    angle = -180;
  else if(angle > 180)
    angle =  angle- 360;
  else if(angle < -180)
    angle = angle + 360;
  return angle;
}
/**
  * @brief  判断电机角度范围
  * @param  SMove_AngleTypeDef 电机角度结构体
  * @param  input ：需要判读的角度
  * @retval None
  * @note   判断是否超出，超出设置为临界值
            若没有设置角度范围，则不判断范围限制，返回输入值
*/
static float angle_range_judgment(turn_motor_typeDef *p,float angle)
{
  p->max_angle = *p->mb.max_angle;//最大角度
  p->min_angle = *p->mb.min_angle;//最小角度

  if(p->min_angle == 0 && p->max_angle == 0)
    return angle;
  if(p->min_angle <= angle && angle <= p->max_angle)
  {
    *p->over_range = false;//输入没有超出角度
  }
  else if(angle < p->min_angle)
  {
    angle = p->min_angle;
    *p->over_range = true;//输入超出角度
    ulog_w("turn","Input beyond minimum Angle");
  }
  else if(angle > p->max_angle)
  {
    angle = p->max_angle;
    *p->over_range = true;//输入超出角度
    ulog_w("turn","Input beyond maximum Angle");
  }
  return angle;
}
/**
 * @brief  转向电机角度控制
 * @param  angle       
 * @param  speed  单位 RPM
 * @param  p                
 * @note   成功返回0X00,模式错误返回0XFF.超时返回0XFE
           触发急停返回0X01,角度无变化返回0X02
 * @note   已绝对角度输出
 */
uint8_t turn_motor_angle_control(float angle,float speed,turn_motor_typeDef* p)
{
  int32_t dest_position = 0;//目标位置
  //角度换算
  angle = angle_conversion(angle);

  /*角度更新判断*/
  p->err  = angle - p->last;
  p->last = angle;
  
  if(p->err != 0)
  {
    //角度限幅
    angle = angle_range_judgment(p,angle);
    //角度换算为位置
    dest_position = angle / 360 * p->cfg.numerator;
    //开始运动
    return turn_motor_start(dest_position,speed,p);
  }
  return 0X02;
}
/**
  * @brief  返回电机角度
  * @param  p
  * @retval 角度
  * @note   
*/
float turn_motor_get_angle(turn_motor_typeDef* p)
{
  long position = 0;
  float degree = 0;
  //获取反馈位置
  motor_get_position(&position,p->nodeID);
  //换算为角度
  degree = (float)position / p->cfg.numerator * 360;
  //角度换算
  degree = angle_conversion(degree);
  return degree;
}
/******************************行走电机函数**************************************/
/**
  * @brief  
  * @param  None.
  * @retval None.
  * @note   
*/
void walk_motor_reentrant(walk_motor_typeDef* p)
{

}
/**
  * @brief  行走电机急停优先级.
  * @param  p
  * @retval None.
  * @note   
*/
static uint8_t walk_motor_stop_priority(walk_motor_typeDef* p)
{
  if(*p->stop_state != NO_STOP)
  {
    walk_motor_stop(p);
    walk_motor_reentrant(p);
    ulog_w("walk","walk motor stop,code is 0X%4.4x",*p->stop_state);
    return 1;
  }
  else
    return 0;
}
/**
 * @brief 行走电机使能
 * @param  p
 * @retval 成功返回0X00,模式错误返回0XFF.超时返回0XFE.
           触发急停返回0X01
 * @note  初始化为速度规划模式
 */
uint8_t walk_motor_enable(walk_motor_typeDef* p)
{
  return motor_on_profile_velocity(p->nodeID);
}
/**
 * @brief 行走电机禁用
 * @param  p
 * @retval 成功返回0X00,模式错误返回0XFF.超时返回0XFE.
           触发急停返回0X01
 * @note  关闭电机使能
 */
uint8_t walk_motor_disable(walk_motor_typeDef* p)
{
  return motor_off(p->nodeID);
}
/**
 * @brief  行走电机停止运动控制
 * @param  p   
 * @retval 成功返回0X00,模式错误返回0XFF.超时返回0XFE.
           触发急停返回0X01     
 * @note   none
 */
uint8_t walk_motor_stop(walk_motor_typeDef* p)
{
  return 0;
}
/**
 * @brief  行走电机开始运动控制
    
 * @param  speed 单位：RPM
 * @param  p
 * @retval 成功返回0X00,模式错误返回0XFF.超时返回0XFE.
           触发急停返回0X01
 * @note   速度规划模式    
 */
static uint8_t walk_motor_start(int16_t speed,walk_motor_typeDef* p)
{
  if (walk_motor_stop_priority(p))
    return 0X01;
  
  return motor_profile_velocity(speed,p->nodeID);
}
/**
  * @brief  判断电机速度范围
  * @param  SMove_AngleTypeDef 电机速度结构体
  * @param  input ：需要判读的速度
  * @retval None
  * @note   判断是否超出，超出设置为临界值
            若没有设置速度范围，则不判断范围限制，返回输入值
*/
static float speed_range_judgment(walk_motor_typeDef *p,float input)
{
  float speed = fabs(input);

  p->max_speed = *p->mb.max_speed;//最大数度
  p->min_speed = 0;

  if(p->min_speed == 0 && p->max_speed == 0)
    return speed;

  if(p->min_speed <= speed && speed <= p->max_speed)
  {
    *p->over_range = false;//输入没有超出
  }
  else if(speed < p->min_speed)
  {
    speed = p->min_speed;
    *p->over_range = true;//输入超出
    ulog_w("walk","Input beyond minimum speed");
  }
  else if(speed > p->max_speed)
  {
    speed = p->max_speed;
    *p->over_range = true;//输入超出
    ulog_w("walk","Input beyond maximum speed");
  }
  return speed;
}
/**
 * @brief  行走电机速度控制  
 * @param  speed  单位 RPM
 * @param  p                
 * @note   成功返回0X00,模式错误返回0XFF.超时返回0XFE
           触发急停返回0X01,速度无变化返回0X02
 * @note   
 */
uint8_t walk_motor_speed_control(float speed,walk_motor_typeDef* p)
{
  /*角度更新判断*/
  p->err  = speed - p->last;
  p->last = speed;
  
  if(p->err != 0)
  {
    //速度限幅
    speed = speed_range_judgment(p,speed);
    //开始运动
    return walk_motor_start(speed,p);
  }
  return 0X02;
}
/**
  * @brief  返回电机速度
  * @param  p
  * @retval 速度 RPM
  * @note   
*/
float walk_motor_get_speed(walk_motor_typeDef* p)
{
  long speed;
  //获取速度
  motor_get_velocity(&speed,p->nodeID);
  return (float)speed / 10;//注意单位为0.1rpm
}
/******************************公共函数******************************************/
/**
 * @brief  电机初始化线程
 * @param  p       
 * @note   1ms一次查询电机节点是否进行操作状态，进入再进行对应节点初始化
 */
static void motor_init_thread(void * p)
{
  turn_motor[0].nodeID = SERVO_NODEID_1;
  nodeID_get_config(&turn_motor[0].cfg,turn_motor[0].nodeID);
  walk_motor[0].nodeID = SERVO_NODEID_2;
  nodeID_get_config(&walk_motor[0].cfg,walk_motor[0].nodeID);

  uint16_t event = 0;
  USER_SET_BIT(event,1); 
  USER_SET_BIT(event,2);
  while(1)
  {
    if(getNodeState(OD_Data,turn_motor[0].nodeID) == Operational)
    {
      MB_TURN1_SET;
      USER_CLEAR_BIT(event,1); 
    }
    if(getNodeState(OD_Data,walk_motor[0].nodeID) == Operational)
    {
      MB_WALK1_SET;
      USER_CLEAR_BIT(event,2); 
    }
    if(event == 0)
    {
      rt_kprintf("All the motors have been powered on and enabled\n");
      return;
    }
    rt_thread_mdelay(1);
  }
}
/**
 * @brief 电机初始化
 * @note  在canopen进去操作模式后进行
 */
void motor_init(void)
{
  rt_err_t ret = RT_EOK;
  /* 创建 MODBUS从机线程*/
  rt_thread_t thread = rt_thread_create( "motor_init",      /* 线程名字 */
                                         motor_init_thread, /* 线程入口函数 */
                                         RT_NULL,           /* 线程入口函数参数 */
                                         1024,              /* 线程栈大小 */
                                         5,                 /* 线程的优先级 */
                                         20);               /* 线程时间片 */
  /* 创建成功则启动线程 */
  if (thread != RT_NULL)
  {
      rt_thread_startup(thread);
  }
  else
  {
      ret = RT_ERROR;
      LOG_E("motor_init created failed.");
  }
}