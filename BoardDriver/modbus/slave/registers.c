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
 */
/* Includes ------------------------------------------------------------------*/
#include "modbus_slave_common.h"
/* Private includes ----------------------------------------------------------*/
#include "master402_canopen.h"
#include "stm32f4xx_hal.h"
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
int modbus_slave_register_default(void)
{
  //节点参数区域
  _tab_registers[1] = 1;  //设置需操作的节点ID
  //02D~10D CAN保留区域
  //11D~30D 电机参数区域
  //31D~36D 时间同步区域

  return RT_EOK;
}
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