/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : adc_dma.h
  * @brief          : adc_dma program body
  ******************************************************************************
  * @attention
  * F7 开启L1-cache会导致DMA数据不更新。例如串口，adc
  * https://bbs.21ic.com/icview-2380632-1-1.html
  * 如果程序默认内存地址设置为0x20020000开始，即运行于SRAM1，才会出现这种现象。
  * 如果默认内存地址为0x20000000开始，即运行于DTCM，结果是正常的，开启D-cache正常。
  * DMA如果使用了SRAM1，要么就不使用D-cache，要使用D-cache就要手动clean，或者设置为write-through
  * 另一种解决办法如本代码所示，
  * https://blog.csdn.net/weifengdq/article/details/121802176
  实现LED报警闪烁，可以实现正常后恢复原来闪烁状态                     2022.04.18
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_DMA_H
#define __ADC_DMA_H
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "led.h"
#include "user_math.h"
#include "motor.h"
#include "stm32f4xx_it.h"
/* Exported types ------------------------------------------------------------*/
/** 
  * @brief ADC计算参数定义
  */  
typedef struct
{
  float Compensation;//补偿值
  float vaule;       //adc采样值
  FOL_TypeDef FOL;
}ADC_Cal_TypeDef;
/** 
  * @brief ADC监视定义
  */  
typedef struct
{
  ADC_Cal_TypeDef AD;
  volatile  float       VPT;        //报警阈值
}ADC_Monitor_TypeDef;
/** 
  * @brief 模拟采集编码器定义
  */  
typedef struct
{
  ADC_Cal_TypeDef AD; //电压结构体
  float value;        //电流值(mA)
}ADC_Encoder_TypeDef;
/** 
  * @brief 拉绳编码器定义
  */  
typedef struct
{
  ADC_Encoder_TypeDef Current;
  uint16_t position;
  SENSOR_state state; //SENSOR_TOUCH为在线状态
  uint16_t height_4ma; //4ma位置高度, 单位mm [100mm]
  volatile uint16_t height_20ma;//20ma位置高度,单位mm [2900mm]
}Rope_Encoder_TypeDef;
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
//理论计算分压系数为7.0606 实际测试需要为7.2准确
#define ADC_RATIO 11.0f //((10+1)/ 1))
/* Exported variables ---------------------------------------------------------*/
extern ADC_Monitor_TypeDef voltage;
extern volatile uint8_t voltage_count;
extern uint16_t Vrefint_vaule;
extern Rope_Encoder_TypeDef lifter_encoder; //4-20ma拉绳编码器
/* Exported functions prototypes ---------------------------------------------*/
extern void Rope_Encoder_Get_Postion(void);
extern void Rope_Encoder_Set_4mA(void);
extern void Rope_Encoder_Set_20mA(void);
#endif /* __ADC_DMA_H */
