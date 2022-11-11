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

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
extern void modbus_slave_input_register_write(void);
extern void modbus_slave_register_write(void);
/**
  * @brief  对共享地址写入本机数据
  * @param  None
  * @retval None
  * @note   None
*/
void modbus_slave_write(void)
{
    modbus_slave_input_register_write();
    modbus_slave_register_write();
}