/**
 * @file monitor.h
 * @brief 
 * @author HLY (1425075683@qq.com)
 * @version 1.0
 * @date 2022-11-23
 * @copyright Copyright (c) 2022
 * @attention 
 * @par –ﬁ∏ƒ»’÷æ:
 * Date       Version Author  Description
 * 2022-11-23 1.0     HLY     first version
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MONITOR_H
#define __MONITOR_H
/* Includes ------------------------------------------------------------------*/
#include "sys.h"
#include <stdbool.h>
/* Private includes ----------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef struct  
{
  uint8_t*    flag;
  uint8_t     button;
  uint16_t*   value;
}Beat_TypeDef;
/* Exported macro ------------------------------------------------------------*/

/* Exported variables ---------------------------------------------------------*/
extern Beat_TypeDef debug_beat;
/* Exported functions prototypes ---------------------------------------------*/

#endif /* __MONITOR_H */
