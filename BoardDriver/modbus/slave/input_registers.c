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
#define INPUT_REG_START 0x00
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static uint16_t _tab_input_registers[10] = {0, 1, 2, 3, 4, 9, 8, 7, 6, 5};

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

    for (int i = 0; i < sizeof(_tab_input_registers) / sizeof(_tab_input_registers[0]); i++) {
        ptr[i] = _tab_input_registers[i];
    }

    return 0;
}
/**
  * @brief  
  * @param  None
  * @retval None
  * @note   None
*/
const agile_modbus_slave_util_map_t input_register_maps[1] = 
{
   //起始地址           结束地址                                                                        获取接口      设置接口 
    {INPUT_REG_START, INPUT_REG_START + sizeof(_tab_input_registers) / sizeof(_tab_input_registers[0]), get_map_buf,    NULL}
};