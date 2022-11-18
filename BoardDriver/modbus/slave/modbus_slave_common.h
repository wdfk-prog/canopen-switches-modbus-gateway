/**
 * @file modbus_slave_common.h
 * @brief 
 * @author HLY (1425075683@qq.com)
 * @version 1.0
 * @date 2022-11-17
 * @copyright Copyright (c) 2022
 * @attention   起始地址和结束地址决定的寄存器个数有限制。更改函数内部 map_buf 数组大小可使其变大。
                bit 寄存器 < 250  register 寄存器 < 125
                接口函数为 NULL，寄存器对应的功能码能响应且为成功。
 * @par 修改日志:
 * Date       Version Author  Description
 * 2022-11-17 1.0     HLY     first version
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
#include <stdbool.h>
#include "rtthread.h"
/* Exported types ------------------------------------------------------------*/
/**
 * @brief 串口调试结构体
 */
typedef struct
{
  uint8_t   read_buf[AGILE_MODBUS_MAX_ADU_LENGTH];
  rt_size_t size;
  bool      flag; //置一开启拷贝
}uart_debug;
/* Exported constants --------------------------------------------------------*/
#define MODBUS_START_ADDR       1     
#define MODBUS_REG_MAX_NUM      125
#define MODBUS_BIT_MAX_NUM      250

#define BIT_MAPS_NUM            1
#define INPUT_BIT_MAPS_NUM      1
#define REGISTER_MAPS_NUM       2
#define INPUT_REGISTER_MAPS_NUM 2
/* Exported macro ------------------------------------------------------------*/
#define UART_DEBUG 0//输出至调试串口
/* Exported variables ---------------------------------------------------------*/
extern const agile_modbus_slave_util_map_t bit_maps[BIT_MAPS_NUM];
extern const agile_modbus_slave_util_map_t input_bit_maps[INPUT_BIT_MAPS_NUM];
extern const agile_modbus_slave_util_map_t register_maps[REGISTER_MAPS_NUM];
extern const agile_modbus_slave_util_map_t input_register_maps[INPUT_REGISTER_MAPS_NUM];
/* Exported functions prototypes ---------------------------------------------*/
extern int addr_check(agile_modbus_t *ctx, struct agile_modbus_slave_info *slave_info);
//保护资源
extern void modbus_mutex_lock(void);
extern void modbus_mutex_unlock(void);
//写入资源
extern void modbus_slave_write(void);
//保持寄存器
extern uint16_t modbus_register_get(uint16_t index,uint16_t sub_index);
extern void modbus_register_set(uint16_t index,uint16_t sub_index,uint16_t data);
extern uint8_t modbus_register_reset(uint16_t start_index,uint16_t start_sub_index,uint16_t end_index,uint16_t end_sub_index);
//输入寄存器
extern uint16_t modbus_input_register_get(uint16_t index,uint16_t sub_index);
//线圈寄存器
extern uint8_t modbus_bits_get(uint16_t index,uint16_t sub_index);
extern void modbus_bits_set(uint16_t index,uint16_t sub_index,uint16_t data);
//离散输入线圈寄存器
extern uint8_t modbus_input_bits_get(uint16_t index,uint16_t sub_index);

#ifdef __cplusplus
}
#endif

#endif /* __MODBUS_SLAVE_COMMON_H */