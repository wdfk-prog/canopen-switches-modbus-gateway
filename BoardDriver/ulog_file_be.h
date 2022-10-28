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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ULOG_FILE_BE_H
#define __ULOG_FILE_BE_H
/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
#define MOTION_TAG  "MOVE"
/* Exported macro ------------------------------------------------------------*/
#define OUT_FILE_ENABLE 0 //使能文件后端输出

#define LOG_MV(...)  ulog_i(MOTION_TAG, __VA_ARGS__)
/* Exported variables ---------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
extern void sys_log_file_backend_init(void);
extern void motion_log_file_backend_init(void);
#endif /* __ULOG_FILE_BE_H */
