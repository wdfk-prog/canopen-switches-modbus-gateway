/**
 * @file bits.c
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

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static uint8_t _tab_bits[MODBUS_BIT_MAX_NUM];
/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  写入线圈寄存器默认值
  * @param  None
  * @retval None
  * @note   None
*/
void modbus_slave_bits_default(void)
{
  _tab_bits[1] = 0;  //电机控制使能
  _tab_bits[2] = 1;  //电机控制禁用
  //调试保留区域
}
/**
  * @brief  线圈寄存器初始化
  * @param  None
  * @retval None
  * @note   None
*/
void modbus_slave_bits_init(void)
{

}
/**
  * @brief  读取线圈寄存器至本机数据中
  * @param  None
  * @retval None
  * @note   None
*/
void modbus_slave_bits_read(void)
{

}
/**
 * @brief Get the map buf object
 * @param  buf   接口地址      
 * @param  bufsz 长度
 * @retval int 
 * @note   None
 */
static int get_map_buf(void *buf, int bufsz)
{
    uint8_t *ptr = (uint8_t *)buf;

    modbus_mutex_lock();
    rt_memcpy(ptr,_tab_bits + MODBUS_START_ADDR,sizeof(_tab_bits));
    modbus_mutex_unlock();
    return 0;
}
/**
 * @brief Set the map buf object
 * @param  index            索引
 * @param  len              长度
 * @param  buf              目标地址
 * @param  bufsz            设置长度
 * @retval int 
 */
static int set_map_buf(int index, int len, void *buf, int bufsz)
{
    uint8_t *ptr = (uint8_t *)buf;

    modbus_mutex_lock();
    rt_memcpy(_tab_bits + MODBUS_START_ADDR + index,ptr + index,len * sizeof(uint8_t));
    modbus_mutex_unlock();
    return 0;
}
/**
  * @brief  获取MODBUS线圈寄存器数据
  * @param  index:数组索引
  * @param  index:数组子索引
  * @retval MODBUS线圈寄存器数据
  * @note   None
*/
uint8_t modbus_get_bits(uint16_t index,uint16_t sub_index)
{
  return _tab_bits[sub_index];
}
/**
  * @brief  设置MODBUS线圈寄存器数据
  * @param  index:数组索引
  * @param  index:数组子索引
  * @param  data:赋值数据
  * @retval None
  * @note   None
*/
void modbus_set_bits(uint16_t index,uint16_t sub_index,uint16_t data)
{
  _tab_bits[sub_index] = data;
}
/**
  * @brief  线圈寄存器数组
  * @note   数组范围：BIT_MAPS_NUM
*/
const agile_modbus_slave_util_map_t bit_maps[BIT_MAPS_NUM] = 
{
   //起始地址               结束地址                      获取接口   设置接口 
   {0,        sizeof(_tab_bits) / sizeof(_tab_bits[0]), get_map_buf, set_map_buf}
};