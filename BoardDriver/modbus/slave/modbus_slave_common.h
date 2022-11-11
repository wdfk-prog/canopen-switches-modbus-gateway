/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : 
  * @brief          : 
  * @date           :
  ******************************************************************************
  * @attention
  起始地址和结束地址决定的寄存器个数有限制。更改函数内部 map_buf 数组大小可使其变大。
  bit 寄存器 < 250
  register 寄存器 < 125

  接口函数为 NULL，寄存器对应的功能码能响应且为成功。

  get 接口
  将地址域内的数据全部拷贝到 buf 中。

  set 接口
  index: 地址域内的偏移
  len: 长度
  根据 index 和 len 修改数据。
  * @author
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MODBUS_SLAVE_COMMON_H
#define __MODBUS_SLAVE_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <agile_modbus.h>
#include "agile_modbus_slave_util.h"
#include <stdint.h>
#include <string.h>
#include "rtthread.h"
/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
#define MODBUS_START_ADDR 1
/* Exported macro ------------------------------------------------------------*/

/* Exported variables ---------------------------------------------------------*/
extern const agile_modbus_slave_util_map_t bit_maps[1];
extern const agile_modbus_slave_util_map_t input_bit_maps[1];
extern const agile_modbus_slave_util_map_t register_maps[1];
extern const agile_modbus_slave_util_map_t input_register_maps[1];
/* Exported functions prototypes ---------------------------------------------*/
static int addr_check(agile_modbus_t *ctx, struct agile_modbus_slave_info *slave_info)
{
    int slave = slave_info->sft->slave;
    if ((slave != ctx->slave) && (slave != AGILE_MODBUS_BROADCAST_ADDRESS) && (slave != 0xFF))
        return -AGILE_MODBUS_EXCEPTION_UNKNOW;

    return 0;
}

extern void modbus_slave_write(void);
extern uint16_t modbus_register_get(uint16_t index,uint16_t sub_index);
extern void modbus_register_set(uint16_t index,uint16_t sub_index,uint16_t data);

#ifdef __cplusplus
}
#endif

#endif /* __MODBUS_SLAVE_COMMON_H */