/**
 * @file registers.c
 * @brief 
 * @author HLY (1425075683@qq.com)
 * @version 1.0
 * @date 2022-11-17
 * @attention 
 * @copyright Copyright (c) 2022
 * @par 修改日志:
 * Date       Version Author  Description
 * 2022-11-17 1.0     HLY     first version
 * 2022-11-23 1.1     HLY     指针挂载寄存器地址，减少赋值时间
 */
/* Includes ------------------------------------------------------------------*/
#include "modbus_slave_common.h"
/* Private includes ----------------------------------------------------------*/
#include "motor.h"
#include "lifter_motor.h"
#include "monitor.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static uint16_t _tab_registers[MODBUS_REG_MAX_NUM];
/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  写入输入寄存器默认值
  * @param  None
  * @retval int
  * @note   None
*/
void modbus_slave_register_default(void)
{
  //01D~10D 节点参数区域
  _tab_registers[1]   = 1;                            //节点ID
  //02D~10D CAN保留区域
  //11D~30D 电机参数区域
  _tab_registers[11]  = 0;                            //电机模式
  _tab_registers[12]  = 0;                            
  _tab_registers[13]  = 0;                            //原点偏移值 单位PUU
  _tab_registers[14]  = 34;                           //回原方式
  _tab_registers[15]  = 100;                          //寻找原点开关速度 单位0.1rpm
  _tab_registers[16]  = 20;                           //寻找 Z脉冲速度   单位0.1rpm
  //17D~21D 电机参数区域
  //31D~40D心跳时间参数区域
  //41D~60D 转向电机区域
  _tab_registers[49] = TURN_MOTOR0_SPEED_DEFAULT;    //转向电机[0]速度输入 单位0.1RPM
  _tab_registers[53] = TURN_MOTOR0_MAX_ANGLE_DEFAULT;//转向电机[0]最大角度
  _tab_registers[54] = TURN_MOTOR0_MIN_ANGLE_DEFAULT;//转向电机[0]最小角度
  //31D~40D 心跳时间参数区域
  //41D~60D 转向电机区域
  //61D~70D 行走电机区域
  _tab_registers[65] = WALK_MOTOR0_MAX_SPEED_DEFAULT;//行走电机[0]最大速度 单位RPM
  //71D~80D 升降电机区域
  
}
/**
  * @brief  保持寄存器初始化
  * @param  None
  * @retval int
  * @note   None
*/
void modbus_slave_register_init(void)
{
  //01D~10D 节点参数区域
  mb_can.nodeID       = &_tab_registers[1];   //节点ID
  //02D~10D CAN保留区域
  //11D~30D 电机参数区域
  mb_can.motor_mode   = &_tab_registers[11];  //电机模式
  mb_can.offset_l     = &_tab_registers[12];  //原点偏移值 单位PUU
  mb_can.offset_h     = &_tab_registers[12];  //原点偏移值 单位PUU
  mb_can.method       = &_tab_registers[14];  //回原方式
  mb_can.switch_speed = &_tab_registers[15];  //寻找原点开关速度 单位rpm
  mb_can.zero_speed   = &_tab_registers[16];  //寻找 Z脉冲速度   单位rpm
  //17D~21D 电机参数区域
  //31D~40D 心跳时间参数区域
  /* update date. */
  mb_tm.year        = &_tab_registers[31];
  mb_tm.mon         = &_tab_registers[32];
  mb_tm.mday        = &_tab_registers[33];     
  /* update time. */
  mb_tm.hour        = &_tab_registers[34];
  mb_tm.min         = &_tab_registers[35];
  mb_tm.sec         = &_tab_registers[36];

  debug_beat.value  = &_tab_registers[38];  //调试串口心跳
  //41D~60D 转向电机区域
  turn_motor[0].mb.angle_l   = &_tab_registers[41];
  turn_motor[0].mb.angle_h   = &_tab_registers[42];//转向电机[0]角度输入 单位:0.001°
  turn_motor[0].mb.speed     = &_tab_registers[49];//转向电机[0]速度输入 单位:0.1RPM
  turn_motor[0].mb.max_angle = &_tab_registers[53];//转向电机[0]最大角度
  turn_motor[0].mb.min_angle = &_tab_registers[54];//转向电机[0]最小角度
  //61D~70D 行走电机区域
  walk_motor[0].mb.speed     = (int16_t*)&_tab_registers[61];//行走电机[0]速度输入 单位:0.1RPM
  walk_motor[0].mb.max_speed = &_tab_registers[65];//行走电机[0]最大速度 单位:RPM
  //71D~80D 升降电机区域
  lifter_motor.target        = (int16_t*)&_tab_registers[71];//升降电机定位高度 单位:mm

  lifter_motor.feedback      = (int16_t*)&_tab_registers[80];//升降电机定位高度 单位:mm 
}
/**
  * @brief  读取保持寄存器至本机数据中
  * @param  None
  * @retval None
  * @note   None
*/
void modbus_slave_register_read(void)
{
  //01D~10D 节点参数区域
  //11D~30D 电机参数区域
  //31D~40D 心跳时间参数区域
  //41D~60D 转向电机区域
  //61D~70D 行走电机区域
}
/**********************************************************************************/
/**
  * @brief  获取MODBUS保持寄存器数据
  * @param  index:数组索引
  * @param  index:数组子索引
  * @retval uint16_t
  * @note   None
*/
uint16_t modbus_get_register(uint16_t index,uint16_t sub_index)
{
  return _tab_registers[sub_index];
}
/**
  * @brief  设置MODBUS保持寄存器数据
  * @param  index:数组索引
  * @param  index:数组子索引
  * @param  data:赋值数据
  * @retval None
  * @note   None
*/
void modbus_set_register(uint16_t index,uint16_t sub_index,uint16_t data)
{
  _tab_registers[sub_index] = data;
}
/**
  * @brief  设置MODBUS保持寄存器数据清零
  * @param  start_index:开始数组索引
  * @param  start_sub_index:开始数组子索引
  * @param  end_index:结束数组索引
  * @param  end_sub_index:结束数组子索引
  * @retval 成功返回0，失败返回0XFF
  * @note   None
*/
uint8_t modbus_reset_register(uint16_t start_index,uint16_t start_sub_index,uint16_t end_index,uint16_t end_sub_index)
{
  uint16_t *ptr = (uint16_t *)_tab_registers + start_sub_index;
  int16_t len = end_sub_index - start_sub_index;
  if(len <= 0)
    return 0XFF;
  else
  {
    rt_memset(ptr,0,len);
    return 0x00;
  }
}
/**
 * @brief Get the map buf object
 * @param  buf              
 * @param  bufsz            
 * @retval int 
 */
static int get_map_buf(void *buf, int bufsz)
{
    uint16_t *ptr = (uint16_t *)buf;

    modbus_mutex_lock();
    rt_memcpy(ptr,_tab_registers + MODBUS_START_ADDR,sizeof(_tab_registers));
    modbus_mutex_unlock();
    return 0;
}
/**
 * @brief Set the map buf object
 * @param  index            
 * @param  len              
 * @param  buf              
 * @param  bufsz            
 * @retval int 
 */
static int set_map_buf(int index, int len, void *buf, int bufsz)
{
    uint16_t *ptr = (uint16_t *)buf;

    modbus_mutex_lock();
    rt_memcpy(_tab_registers + MODBUS_START_ADDR + index,ptr + index,len * sizeof(uint16_t));
    modbus_mutex_unlock();
    return 0;
}
/**
 * @brief 保持寄存器
 */
const agile_modbus_slave_util_map_t register_maps[REGISTER_MAPS_NUM] = 
{
   //起始地址                     结束地址                          获取接口   设置接口 
   {0,        sizeof(_tab_registers) / sizeof(_tab_registers[0]),    get_map_buf, set_map_buf},
};