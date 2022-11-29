/**
  ******************************************************************************
  * @file    
  * @brief   
  ******************************************************************************
  * @attention  
  * @author 
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MONITOR_H
#define __MONITOR_H
/* Includes ------------------------------------------------------------------*/
#include "sys.h"
/* Private includes ----------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef struct  
{
  volatile uint8_t     flag;
  volatile uint8_t     button;
  volatile uint16_t    Value;
  FunctionalState      EN;
  int32_t              Error;//心跳间隔时间
}Baet_TypeDef;
/* Exported macro ------------------------------------------------------------*/

/* Exported variables ---------------------------------------------------------*/
extern u8 RestFlag;
extern Baet_TypeDef Weinview_Beat;
extern Baet_TypeDef IPC_Beat;
/* Exported functions prototypes ---------------------------------------------*/
extern int mv_log_timer_init(void);
extern int mv_log_timer_start(void);
#endif /* __MONITOR_H */
