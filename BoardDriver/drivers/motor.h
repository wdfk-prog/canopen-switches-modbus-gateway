/**
 * @file motor.h
 * @brief 
 * @author HLY (1425075683@qq.com)
 * @version 1.0
 * @date 2022-11-21
 * @copyright Copyright (c) 2022
 * @attention 
 * @par 修改日志:
 * Date       Version Author  Description
 * 2022-11-21 1.0     HLY     first version
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "user_math.h"

/* Exported types ------------------------------------------------------------*/
/** 
  * @brief  停止代码
  */  
typedef enum
{
  NO_STOP = 0x00U,//无停止            0x00
  HARD_STOP,      //硬件急停          0x02
  SOFT_STOP     , //软急停            0x04
  Detection_STOP, //心跳异常          0x08
  ENABLE_STOP   , //电机关闭使能      0x10
  ALM_STOP      , //报警报警          0X20
  CRASH_STOP    , //防撞条报警        0X40
  VBATT_STOP    , //电压过低报警      0X80
  BEAT_STOP    ,  //心跳异常          0X100
}Stop_Code;

#define stop_type uint16_t

/**
 * @brief 电机配置结构体
 * @note  numerator:  运动一圈所需脉冲数
 *        denominator:电机编码器分辨率
 */
typedef struct
{
  uint32_t numerator;   //电子齿轮比分子 
  uint32_t denominator; //电子齿轮比分母
}motor_config;
/**
 * @brief 转向电机结构体
 * @note  over_range:true 超出范围
 */
typedef struct
{
  uint8_t       nodeID;         //电机节点ID
  uint8_t*      over_range;     //角度超出范围标志
  stop_type     stop_state;     //急停标志
  int16_t       max_angle;      //最大角度
  int16_t       min_angle;      //最小角度
  float         last;           //上一次角度
  float         err;            //角度更新误差
  motor_config  cfg;            //电机配置
}turn_motor_typeDef;
/* Exported constants --------------------------------------------------------*/
#define TURN_MOTOR_NUM 1//转向电机电机数量
/* Exported macro ------------------------------------------------------------*/
#define TURN_MOTOR_SPEED_DEFAULT 60*10    //0.1RPM
#define TURN_MOTOR_MAX_ANGLE_DEFAULT 90 
#define TURN_MOTOR_MIN_ANGLE_DEFAULT -90
/* Exported variables ---------------------------------------------------------*/
extern turn_motor_typeDef turn_motor[TURN_MOTOR_NUM];
/* Exported functions prototypes ---------------------------------------------*/
extern uint8_t turn_motor_enable(turn_motor_typeDef* p);
extern uint8_t turn_motor_disable(turn_motor_typeDef* p);
extern uint8_t turn_motor_stop(turn_motor_typeDef* p);
extern uint8_t turn_motor_angle_control(float angle,float speed,turn_motor_typeDef* p);
extern float turn_motor_get_angle(turn_motor_typeDef* p);
extern bool turn_motor_get_over_range(turn_motor_typeDef* p);
extern void turn_motor_set_angle_range(int16_t max,int16_t min,turn_motor_typeDef* p);
#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */