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

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static uint16_t _tab_input_registers[MODBUS_REG_MAX_NUM];
/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  写入输入寄存器默认值
  * @param  None
  * @retval None
  * @note   None
*/
int modbus_slave_input_register_default(void)
{
  _tab_input_registers[1]  = MAX_NODE_COUNT - 1;  //节点数量
  _tab_input_registers[2]  = 0X0F;                 //节点NMT状态
  //从03D~06D
  nodeID_get_name((char *)&_tab_input_registers[3],
                   modbus_register_get(0,1));     //节点名称
  _tab_input_registers[7]  = 0X00;                 //节点错误代码
  //08D~10D节点具体错误
  _tab_input_registers[8]  = 0X00;                 //节点具体错误
  _tab_input_registers[9]  = 0X00;                 //节点具体错误
  _tab_input_registers[10] = 0X00;                 //节点具体错误
  return RT_EOK;
}

/**
  * @brief  写入本机数据至输入寄存器中
  * @param  None
  * @retval None
  * @note   None
*/
void modbus_slave_input_register_write(void)
{
  uint8_t nodeID = modbus_register_get(0,1);

  _tab_input_registers[2] = nodeID_get_nmt(nodeID);         //节点NMT状态
   //从03D~06D
  nodeID_get_name((char *)&_tab_input_registers[3],nodeID); //节点名称
  _tab_input_registers[7] = nodeID_get_errcode(nodeID);     //节点错误代码
  //08D~10D节点具体错误
  nodeID_get_errSpec((char *)&_tab_input_registers[8],nodeID);
}
/**
  * @brief  获取MODBUS输入寄存器数据
  * @param  index:数组索引
  * @param  index:数组子索引
  * @retval None
  * @note   None
*/
uint16_t modbus_input_register_get(uint16_t index,uint16_t sub_index)
{
  return _tab_input_registers[sub_index];
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

  modbus_mutex_lock();
  //使用memcpy比数组赋值快15us左右
  rt_memcpy(ptr,_tab_input_registers + MODBUS_START_ADDR,sizeof(_tab_input_registers));
  modbus_mutex_unlock();
  return 0;
}
/**
  * @brief  
  * @param  None
  * @retval None
  * @note   None
*/
const agile_modbus_slave_util_map_t input_register_maps[INPUT_REGISTER_MAPS_NUM] = 
{
  //起始地址                          结束地址                                  获取接口      设置接口 
  {0,         sizeof(_tab_input_registers) / sizeof(_tab_input_registers[0]),    get_map_buf,    NULL},
};