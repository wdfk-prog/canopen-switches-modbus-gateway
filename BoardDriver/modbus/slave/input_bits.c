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
#define INPUT_BITS_START 0x00
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static uint8_t _tab_input_bits[MODBUS_BIT_MAX_NUM];
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

    rt_memcpy(ptr,_tab_input_bits + MODBUS_START_ADDR,sizeof(_tab_input_bits));
    return 0;
}
/**
  * @brief  获取MODBUS离散输入线圈寄存器数据
  * @param  index:数组索引
  * @param  index:数组子索引
  * @retval None
  * @note   None
*/
uint8_t modbus_input_bits_get(uint16_t index,uint16_t sub_index)
{
  return _tab_input_bits[sub_index];
}
/**
  * @brief  
  * @param  None
  * @retval None
  * @note   None
*/
const agile_modbus_slave_util_map_t input_bit_maps[1] = 
{
   //起始地址           结束地址                                                                获取接口      设置接口 
    {INPUT_BITS_START, INPUT_BITS_START + sizeof(_tab_input_bits) / sizeof(_tab_input_bits[0]), get_map_buf,    NULL}
};