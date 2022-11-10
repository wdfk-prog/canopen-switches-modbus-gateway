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
static uint8_t _tab_input_bits[10] = {0, 1, 1, 0, 0, 1, 1, 0, 0, 1};
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

    for (int i = 0; i < sizeof(_tab_input_bits); i++) {
        ptr[i] = _tab_input_bits[i];
    }

    return 0;
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