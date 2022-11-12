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
#include "master402_canopen.h"
/* Private typedef -----------------------------------------------------------*/
enum input_registers_name
{
  node_id = 0X01,

};
/* Private define ------------------------------------------------------------*/
#define REG_START 0x00

#define CAN_START 1
#define CAN_END   10
#define MOTOR_START 11
#define MOTOR_END   20
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static uint16_t _tab_registers[MODBUS_REG_MAX_NUM];
/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  写入输入寄存器默认值
  * @param  None
  * @retval None
  * @note   None
*/
int modbus_slave_register_default(void)
{
  _tab_registers[node_id] = 1;

  return RT_EOK;
}
INIT_DEVICE_EXPORT(modbus_slave_register_default);
/**
  * @brief  写入本机数据至保持寄存器中
  * @param  None
  * @retval None
  * @note   None
*/
void modbus_slave_register_write(void)
{

}
/**
  * @brief  获取MODBUS保持寄存器数据
  * @param  index:数组索引
  * @param  index:数组子索引
  * @retval None
  * @note   None
*/
uint16_t modbus_register_get(uint16_t index,uint16_t sub_index)
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
void modbus_register_set(uint16_t index,uint16_t sub_index,uint16_t data)
{
  _tab_registers[sub_index] = data;
}
/**
  * @brief  
  * @param  None
  * @retval None
  * @note   None
*/
static int get_map_buf(void *buf, int bufsz)
{
    uint16_t *ptr = (uint16_t *)buf;

    rt_memcpy(ptr,_tab_registers + MODBUS_START_ADDR,sizeof(_tab_registers));
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

//    for (int i = 0; i < len; i++) 
//    {
//        _tab_registers[MODBUS_START_ADDR + index + i] = ptr[index + i];
//    }
    rt_memcpy(_tab_registers + MODBUS_START_ADDR + index,ptr + index,len);
    return 0;
}
/**
  * @brief  
  * @param  None
  * @retval None
  * @note   None
*/
const agile_modbus_slave_util_map_t register_maps[REGISTER_MAPS_NUM] = 
{
   //起始地址   结束地址 获取接口   设置接口 
    {CAN_START, CAN_END, get_map_buf, set_map_buf},
};