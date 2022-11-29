/**
  ******************************************************************************
  * @file    mb_key.h
  * @brief   mb按键驱动V1.0
  ******************************************************************************
  * @attention  None.
  * @author HLY
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MB_KEY_H
#define __MB_KEY_H
/* Includes ------------------------------------------------------------------*/
#include "sys.h"
/* Exported types ------------------------------------------------------------*/
/** 
  * @brief  按键注册表
  */
typedef enum
{
	KEY_SOFT_STOP,                //软急停
  KEY_TURN_ZERO,                //转向电机复位
  KEY_TURN_LOCATE,              //转向电机定位
  KEY_TURN_UP,                  //转向电机轴点动+  
  KEY_TURN_DOWN,                //转向电机轴点动-  
  KEY_TURN_AXIS,                //转向电机轴清零  
  KEY_TURN_ENABLE,              //转向电机使能
  KEY_WALK_BREAK,               //行走电机刹车
  KEY_WALK_ENABLE,              //行走电机使能
  KEY_LIFT_UP,                  //叉臂顶升
  KEY_LIFT_DOWN,                //叉臂下降
  KEY_LIFT_LOCATE,              //叉臂定位
  KEY_LIFT_ZERO,                //叉臂回零
  KEY_LIFT_SQP_SHIELD,          //叉臂接近开关屏蔽
  KEY_LIFT_4MA_SET,             //拉绳编码器4ma设置
  KEY_LIFT_20MA_SET,            //拉绳编码器20ma设置
  KEY_FACTORY,                  //恢复出厂设置
  KEY_READ,                     //读取EEPROM数据
  KEY_SAVE,                     //保存EEPROM数据
  KEY_TSET_WDT,                 //看门狗测试
  MBKEY_NUM,// 必须要有的记录按钮数量，必须在最后
}MBKEY_LIST;
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported variables ---------------------------------------------------------*/
extern uint16_t WDT_key;
/* Exported functions prototypes ---------------------------------------------*/
extern void MBKEY_Init(void);//IO初始化
extern void MBKey_Handler(void *p);//按键处理函数
extern void MBKey_Shield_Operate(uint8_t num,FunctionalState option);
#endif /* __MB_KEY_H */
