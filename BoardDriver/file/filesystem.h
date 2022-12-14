/**
 * @file filesystem.h
 * @brief 
 * @author HLY (1425075683@qq.com)
 * @version 1.0
 * @date 2022-11-18
 * @copyright Copyright (c) 2022
 * @attention 
 * @par 修改日志:
 * Date       Version Author  Description
 * 2022-11-18 1.0     HLY     first version
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FILESYSTEM_H
#define __FILESYSTEM_H

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <flashdb.h>
/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
#define FLASHDB_FILE_ENABLE 1
/* Exported variables ---------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
extern void rtc_time_write(void);
extern time_t rtc_time_read(void);
extern uint16_t boot_count_read(void);

#ifdef __cplusplus
}
#endif

#endif /* __FILESYSTEM_H */