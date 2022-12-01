/**
 * @file input_registers.c
 * @brief 
 * @author HLY (1425075683@qq.com)
 * @version 1.0
 * @date 2022-11-17
 * @attention 
 * @copyright Copyright (c) 2022  
 * @par 修改日志:
 * Date       Version Author  Description
 * 2022-11-17 1.0     HLY     first version
 */
/* Includes ------------------------------------------------------------------*/
#include "modbus_slave_common.h"
/* Private includes ----------------------------------------------------------*/
#include "main.h"
#include "master402_canopen.h"
#include "filesystem.h"
#include "motor_control.h"
#include "lifter_motor.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static uint16_t _tab_input_registers[MODBUS_REG_MAX_NUM];
/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  写入输入寄存器默认值
  * @param  None
  * @retval int
  * @note   None
*/
void modbus_slave_input_register_default(void)
{
  //01D~10D节点参数区域
  _tab_input_registers[1]  = MAX_NODE_COUNT - 1;    //节点数量
  _tab_input_registers[2]  = 0X0F;                  //节点NMT状态
  nodeID_get_name((char *)&_tab_input_registers[3], //03D~06D 节点名称
                   modbus_get_register(0,1));       //节点名称
  _tab_input_registers[7]  = 0X00;                  //节点错误代码
  _tab_input_registers[8]  = 0X00;                  //节点具体错误
  _tab_input_registers[9]  = 0X00;                  //节点具体错误
  _tab_input_registers[10] = 0X00;                  //节点具体错误
  //11D~20D电机参数区域
  _tab_input_registers[11] = 0X00;  //控制指令
  _tab_input_registers[12] = 0X00;  //状态位
  _tab_input_registers[13] = 0X00;  //当前位置低16位
  _tab_input_registers[14] = 0X00;  //当前位置高16位
  _tab_input_registers[15] = 0X00;  //当前速度低16位
  _tab_input_registers[16] = 0X00;  //当前速度高16位
  //21D~40D芯片参数区域
  rt_memcpy((char *)(_tab_input_registers+21),VERSION,sizeof(VERSION));              //打印版本信息 21D~27D编译日期
  _tab_input_registers[24] = ((uint32_t)(YEAR*10000+(MONTH + 1)*100+DAY) & 0xffff);  //日期低16bti
	_tab_input_registers[25] = ((uint32_t)(YEAR*10000+(MONTH + 1)*100+DAY)>>16);		   //日期高16bti
	_tab_input_registers[26] = ((uint32_t)(HOUR*10000+MINUTE*100+SEC)&0xffff);         //时间低16bti
	_tab_input_registers[27] = ((uint32_t)(HOUR*10000+MINUTE*100+SEC)>>16);		         //时间高16bti
  _tab_input_registers[28] =  HAL_GetUIDw0();
  _tab_input_registers[29] =  HAL_GetUIDw0() >> 16;
  _tab_input_registers[30] =  HAL_GetUIDw1();
  _tab_input_registers[31] =  HAL_GetUIDw1() >> 16;
  _tab_input_registers[32] =  HAL_GetUIDw2();
  _tab_input_registers[33] =  HAL_GetUIDw2() >> 16;
  _tab_input_registers[34] =  HAL_GetHalVersion();
  _tab_input_registers[35] =  HAL_GetHalVersion() >> 16;
  _tab_input_registers[38] =  boot_count_read();    //复位次数
  //41D~60D转向电机区域
}
/**
  * @brief  输入寄存器初始化
  * @param  None
  * @retval int
  * @note   None
*/
void modbus_slave_input_register_init(void)
{
  //01D~10D节点参数区域
  //11D~20D电机参数区域
  //21D~40D芯片参数区域
  //41D~60D 转向电机区域
  turn_motor[0].stop_state = &_tab_input_registers[53]; //转向电机[0]停止代码
  //61D~70D 行走电机区域
  walk_motor[0].stop_state = &_tab_input_registers[65]; //行走电机[0]停止代码
  //71D~80D 升降电机区域
  lifter_motor.stop_state  = &_tab_input_registers[71]; //升降电机停止代码
}
/**
  * @brief  写入本机数据至输入寄存器中
  * @param  None
  * @retval None
  * @note   None
*/
void modbus_slave_input_register_write(void)
{
  static int now_time = 0,last_time = 0;

  last_time = now_time;
  now_time = rt_tick_get_millisecond();

  uint8_t nodeID = modbus_get_register(0,1);
  //01D~10D节点参数区域
  _tab_input_registers[2] = nodeID_get_nmt(nodeID);           //节点NMT状态
  nodeID_get_name((char *)&_tab_input_registers[3],nodeID);   //节点名称 03D~06D
  _tab_input_registers[7] = nodeID_get_errcode(nodeID);       //节点错误代码
  nodeID_get_errSpec((char *)&_tab_input_registers[8],nodeID);//节点具体错误 08D~10D
  //11D~20D电机参数区域
  _tab_input_registers[11] = motor_get_controlword(nodeID);         //控制指令
  _tab_input_registers[12] = motor_get_statusword(nodeID);          //状态位
  motor_get_position((INTEGER32 *)&_tab_input_registers[13],nodeID);//当前位置
  motor_get_velocity((INTEGER32 *)&_tab_input_registers[15],nodeID);//当前速度 单位 0.1RPM
  //21D~40D芯片参数区域
  _tab_input_registers[36] =  now_time;
  _tab_input_registers[37] =  now_time >> 16;                   //芯片运行时间
  _tab_input_registers[39] =  (uint16_t)(now_time - last_time); //modbus通信周期 单位ms
  //41D~60D 转向电机区域
  _tab_input_registers[41] =  turn_motor_get_angle(&turn_motor[0]) * 1000;        //转向电机[0]角度反馈
  _tab_input_registers[42] =  (int32_t)(turn_motor_get_angle(&turn_motor[0]) * 1000) >> 16;
  motor_get_velocity((INTEGER32 *)&_tab_input_registers[49],turn_motor[0].nodeID);//转向电机[0]当前速度 单位 0.1RPM
  //61D~70D 行走电机区域
  _tab_input_registers[61] =  walk_motor_get_speed(&walk_motor[0]) * 10;        //行走电机[0] 单位:0.1RPM
}
/**
  * @brief  获取MODBUS输入寄存器数据
  * @param  index:数组索引
  * @param  index:数组子索引
  * @retval uint16_t
  * @note   None
*/
uint16_t modbus_get_input_register(uint16_t index,uint16_t sub_index)
{
  return _tab_input_registers[sub_index];
}
/**
 * @brief Get the map buf object
 * @param  buf              地址
 * @param  bufsz            长度
 * @retval int 
 */
static int get_map_buf(void *buf, int bufsz)
{
  uint16_t *ptr = (uint16_t *)buf;

  modbus_mutex_lock();
  //使用memcpy比数组赋值快15us左右
  rt_memcpy(ptr,_tab_input_registers + MODBUS_START_ADDR,sizeof(_tab_input_registers));
  modbus_mutex_unlock();
  return 0;
}
/**
 * @brief 输入寄存器
 */
const agile_modbus_slave_util_map_t input_register_maps[INPUT_REGISTER_MAPS_NUM] = 
{
  //起始地址                          结束地址                                  获取接口      设置接口 
  {0,         sizeof(_tab_input_registers) / sizeof(_tab_input_registers[0]),    get_map_buf,    NULL},
};