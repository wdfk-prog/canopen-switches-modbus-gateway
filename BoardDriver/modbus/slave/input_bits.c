/**
 * @file input_bits.c
 * @brief 
 * @author HLY (1425075683@qq.com)
 * @version 1.0
 * @date 2022-11-17
 * 
 * @copyright Copyright (c) 2022  
 * 
 * @par 修改日志:
 * Date       Version Author  Description
 * 2022-11-17 1.0     HLY     first version
 */
/* Includes ------------------------------------------------------------------*/
#include "modbus_slave_common.h"
/* Private includes ----------------------------------------------------------*/
#include "motor.h"
#include "monitor.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static uint8_t _tab_input_bits[MODBUS_BIT_MAX_NUM];
/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  写入离散输入寄存器默认值
  * @param  None
  * @retval int
  * @note   None
*/
void modbus_slave_input_bits_default(void)
{
  //01D~04D心跳报警区域
  //06D~10D转向电机区域
}
/**
  * @brief  离散输入寄存器初始化
  * @param  None
  * @retval int
  * @note   None
*/
void modbus_slave_input_bits_init(void)
{
  //01D~04D心跳报警区域
  debug_beat.flag = &_tab_input_bits[1];
  //06D~10D转向电机区域
  turn_motor[0].over_range = &_tab_input_bits[6]; //转向电机[0]角度超出范围标志
  //11D~15D行走电机区域
  walk_motor[0].over_range = &_tab_input_bits[11]; //转向电机[0]角度超出范围标志
}
/**
  * @brief  写入本机数据至离散输入寄存器中
  * @param  None
  * @retval None
  * @note   None
*/
void modbus_slave_input_bits_write(void)
{

}
/**
 * @brief Get the map buf object
 * @param  buf   目标地址
 * @param  bufsz 获取长度
 * @retval int 
 * @note   None
 */
static int get_map_buf(void *buf, int bufsz)
{
    uint8_t *ptr = (uint8_t *)buf;

    modbus_mutex_lock();
    rt_memcpy(ptr,_tab_input_bits + MODBUS_START_ADDR,sizeof(_tab_input_bits));
    modbus_mutex_unlock();
    return 0;
}
/**
  * @brief  获取MODBUS离散输入线圈寄存器数据
  * @param  index:数组索引
  * @param  index:数组子索引
  * @retval 离散输入线圈寄存器数据
  * @note   None
*/
uint8_t modbus_get_input_bits(uint16_t index,uint16_t sub_index)
{
  return _tab_input_bits[sub_index];
}
/**
  * @brief  离散输入线圈寄存器数据
  * @note   None
*/
const agile_modbus_slave_util_map_t input_bit_maps[INPUT_BIT_MAPS_NUM] = 
{
   //起始地址           结束地址                                   获取接口      设置接口 
    {0,       sizeof(_tab_input_bits) / sizeof(_tab_input_bits[0]), get_map_buf,    NULL}
};