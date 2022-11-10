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
static uint8_t _tab_bits[10] = {0, 1, 0, 1, 0, 1, 0, 1, 0, 1};
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

    for (int i = 0; i < sizeof(_tab_bits); i++) 
    {
        ptr[i] = _tab_bits[i];
    }

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

    for (int i = 0; i < len; i++) 
    {
        _tab_bits[index + i] = ptr[index + i];
    }

    return 0;
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