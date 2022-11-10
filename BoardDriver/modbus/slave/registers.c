#include <stdint.h>
#include <string.h>
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
#define REG_START 0x00
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static uint16_t _tab_registers[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  
  * @param  None
  * @retval None
  * @note   None
*/
static int get_map_buf(void *buf, int bufsz)
{
    uint16_t *ptr = (uint16_t *)buf;

    for (int i = 0; i < sizeof(_tab_registers) / sizeof(_tab_registers[0]); i++) {
        ptr[i] = _tab_registers[i];
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
    uint16_t *ptr = (uint16_t *)buf;

    for (int i = 0; i < len; i++) {
        _tab_registers[index + i] = ptr[index + i];
    }

    return 0;
}
/**
  * @brief  
  * @param  None
  * @retval None
  * @note   None
*/
const agile_modbus_slave_util_map_t register_maps[1] = 
{
   //起始地址   结束地址                                                        获取接口   设置接口 
    {REG_START, REG_START + sizeof(_tab_registers) / sizeof(_tab_registers[0]), get_map_buf, set_map_buf}
};
