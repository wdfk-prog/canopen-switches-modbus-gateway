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
#include "modbus_slave_common.h"
/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define BITS_START 0x00
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static uint8_t _tab_bits[MODBUS_BIT_MAX_NUM];
/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  
  * @param  None
  * @retval None
  * @note   None
*/
static int get_map_buf(void *buf, int bufsz)
{
    uint8_t *ptr = (uint8_t *)buf;

    rt_memcpy(ptr,_tab_bits + MODBUS_START_ADDR,sizeof(_tab_bits));
    return 0;
}
/**
  * @brief  
  * @param  None
  * @retval None
  * @note   None
*/
static int set_map_buf(int index, int len, void *buf, int bufsz)
{
    uint8_t *ptr = (uint8_t *)buf;

//    for (int i = 0; i < len; i++) 
//    {
//        _tab_bits[MODBUS_START_ADDR + index + i] = ptr[index + i];
//    }
    rt_memcpy(_tab_bits + MODBUS_START_ADDR + index,ptr + index,len);
    return 0;
}
/**
  * @brief  获取MODBUS线圈寄存器数据
  * @param  index:数组索引
  * @param  index:数组子索引
  * @retval None
  * @note   None
*/
uint8_t modbus_bits_get(uint16_t index,uint16_t sub_index)
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
void modbus_bits_set(uint16_t index,uint16_t sub_index,uint16_t data)
{
  _tab_bits[sub_index] = data;
}
/**
  * @brief  
  * @param  None
  * @retval None
  * @note   None
*/
const agile_modbus_slave_util_map_t bit_maps[1] = 
{
   //起始地址     结束地址                                                获取接口   设置接口 
    {BITS_START, BITS_START +  sizeof(_tab_bits) / sizeof(_tab_bits[0]), get_map_buf, set_map_buf}
};