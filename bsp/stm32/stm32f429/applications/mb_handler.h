/**
  ******************************************************************************
  * @file    demo.h
  * @brief   FREEMODBUS自定义
  ******************************************************************************
  * @attention  
  * @author HLY
  步进电机参数更改需要更改 周期脉冲个数
  
  优化转向电机顺时针电平设置选择                         2022.07.20
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MB_HANDLER_H
#define __MB_HANDLER_H
/* Includes ------------------------------------------------------------------*/
#include "sys.h"
#include "user_math.h"
/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
//输入寄存器起始地址
#define REG_INPUT_START 0
//输入寄存器数量
#define REG_INPUT_NREGS 1000
//保持寄存器起始地址
#define REG_HOLDING_START 0
//保持寄存器数量
#define REG_HOLDING_NREGS 1000
//线圈起始地址
#define REG_COILS_START 0
//线圈数量
#define REG_COILS_SIZE 160 //8的倍数
//开关寄存器起始地址
#define REG_DISCRETE_START 0
//开关寄存器数量
#define REG_DISCRETE_SIZE 160//8的倍数

#define Turn_Cycle_Pulse_Num     20000         //一周期需要脉冲数
#define Turn_REDUCTION_RATIO     1             //减速比
#define Turn_CLOCKWISE           GPIO_PIN_RESET//转向电机方向电平值
#define Turn_ANGLE_MAX           180           //最大角度 单位角度
#define Turn_ANGLE_MIN           -180          //最小角度 单位角度
#define Turn_SPEED_MAX           1000          //最大速度 单位RPM
#define Turn_SPEED_MIN           0             //最小速度 单位RPM
#define Turn_SPEED_JOG           5             //点动速度 单位RPM
#define Turn_SANGLE_INIT         1169//-66.98  //复位角度 单位弧度
#define Turn_BACK_HIGH_SPEED     -16           //回原高速 单位RPM
#define Turn_BACK_LOW_SPEED      1             //回原低速 单位RPM

#define Walk_RESOLUTION          1024
#define Walk_Cycle_Pulse_Num     Walk_RESOLUTION * 4//一周期需要脉冲数 = 分辨率编码器 * 4倍频
#define Walk_REDUCTION_RATIO     1             //减速比
#define Walk_CLOCKWISE           GPIO_PIN_RESET//转向电机方向电平值
#define Walk_SPEED_MAX           1000          //最大速度 单位RPM
#define Walk_SPEED_MIN           0             //最小速度 单位RPM
#define Walk_STEP                100           //步长     单位RPM
#define Walk_TIME                1             //周期     单位MS

#define ROPE_4MA_HEIGHT   0   
#define ROPE_20MA_HEIGHT  2130 
/* Exported macro ------------------------------------------------------------*/
#define ushort uint16_t //强制转换无符号整型
#define _short int16_t  //强制转换有符号整型
#define _float(x) ((float)((int16_t)(x)))
#define MB_SET_BIT(x)    USER_SET_BIT  (ucRegCoilsBuf[(uint8_t)((x) / 8)],(x - (uint8_t)((x) / 8) * 8))
#define MB_RESET_BIT(x)  USER_CLEAR_BIT(ucRegCoilsBuf[(uint8_t)((x) / 8)],(x - (uint8_t)((x) / 8) * 8))
#define MB_GET_BIT(x)    USER_GET_BIT  (ucRegCoilsBuf[(uint8_t)((x) / 8)],(x - (uint8_t)((x) / 8) * 8))
/*******************控制器操作********************************/
/*******************0X01~0X23*********************************/
#define SOFT_STOP_SET             MB_SET_BIT  (1)  //软急停按钮按下
#define SOFT_STOP_RESET           MB_RESET_BIT(1)  //软急停按钮松开

#define TURN_ZERO_SET             MB_SET_BIT  (2)  //转向电机复位启用
#define TURN_ZERO_RESET           MB_RESET_BIT(2)  //转向电机复位禁用

#define TURN_LOCATE_SET           MB_SET_BIT  (3)  //转向电机定位启用
#define TURN_LOCATE_RESET         MB_RESET_BIT(3)  //转向电机定位禁用

#define TURN_UP_SET               MB_SET_BIT  (4)  //转向电机轴点动+启用
#define TURN_UP_RESET             MB_RESET_BIT(4)  //转向电机轴点动+禁用

#define TURN_DOWN_SET             MB_SET_BIT  (5)  //转向电机轴点动-启用
#define TURN_DOWN_RESET           MB_RESET_BIT(5)  //转向电机轴点动-禁用

#define TURN_AXIS_RESET           MB_RESET_BIT(6)  //转向电机轴清零

#define TURN_ENABLE_SET           MB_SET_BIT  (7)  //转向电机使能
#define TURN_ENABLE_RESET         MB_RESET_BIT(7)  //转向电机禁用

#define WALK_BREAK_SET            MB_SET_BIT  (8)  //行走电机刹车
#define WALK_BREAK_RESET          MB_RESET_BIT(8)  //行走电机松开刹车

#define WALK_ENABLE_SET           MB_SET_BIT  (9)  //行走电机使能
#define WALK_ENABLE_RESET         MB_RESET_BIT(9)  //行走电机禁用

#define LIFT_UP_SET               MB_SET_BIT  (10) //叉臂手动顶升
#define LIFT_UP_RESET             MB_RESET_BIT(10) //叉臂取消手动顶升
#define LIFT_UP_GET               MB_GET_BIT  (10) //叉臂手动顶升查询

#define LIFT_DOWN_SET             MB_SET_BIT  (11) //叉臂手动下降
#define LIFT_DOWN_RESET           MB_RESET_BIT(11) //叉臂取消手动下降
#define LIFT_DOWN_GET             MB_GET_BIT  (11) //叉臂手动下降查询

#define LIFT_LOCATE_SET           MB_SET_BIT  (12) //叉臂定位使能
#define LIFT_LOCATE_RESET         MB_RESET_BIT(12) //叉臂定位禁用
#define LIFT_LOCATE_GET           MB_GET_BIT  (12) //叉臂定位查询

#define LIFT_ZERO_SET             MB_SET_BIT  (13) //叉臂回零使能
#define LIFT_ZERO_RESET           MB_RESET_BIT(13) //叉臂回零禁用

#define LIFT_BREAK_SET            MB_SET_BIT  (14) //叉臂刹车
#define LIFT_BREAK_RESET          MB_RESET_BIT(14) //
#define LIFT_BREAK_GET            MB_GET_BIT  (14) //叉臂刹车查询

#define LIFT_SQP_SHIELD_SET       MB_SET_BIT  (15) //叉臂接近开关屏蔽
#define LIFT_SQP_SHIELD_RESET     MB_RESET_BIT(15) //
#define LIFT_SQP_SHIELD_GET       MB_GET_BIT  (15) //

#define LIFT_ENCODER_4MA_SET      MB_SET_BIT  (16) //拉绳编码器4ma设置
#define LIFT_ENCODER_4MA_RESET    MB_RESET_BIT(16) //

#define LIFT_ENCODER_20MA_SET     MB_SET_BIT  (17) //拉绳编码器20ma设置
#define LIFT_ENCODER_20MA_RESET   MB_RESET_BIT(17) //

#define REMOVE_ALARM_SET          MB_SET_BIT  (18) //清除报警信息
#define REMOVE_ALARM_RESET        MB_RESET_BIT(18) //清除报警信息
/*******************系统操作区域********************************/
/*******************0X24~0X47***********************************/
#define DEFAULT_DATA_SET          MB_SET_BIT  (24) //恢复出厂设置
#define DEFAULT_DATA_RESET        MB_RESET_BIT(24) //恢复出厂设置

#define READ_SET                  MB_SET_BIT  (25) //读取EEPROM数据
#define READ_RESET                MB_RESET_BIT(25) //读取EEPROM数据

#define SAVE_SET                  MB_SET_BIT  (26) //保存数据
#define SAVE_RESET                MB_RESET_BIT(26) //保存数据

#define WDT_TEST_SET              MB_SET_BIT  (27) //看门狗测试
#define WDT_TEST_RERSET           MB_RESET_BIT(27) //看门狗测试

#define ROPE_RESET_SET            MB_SET_BIT  (28) //拉绳参数清零
#define ROPE_RESET_RESET          MB_RESET_BIT(28) //拉绳参数清零
#define ROPE_RESET_GET            MB_GET_BIT  (28) //拉绳参数清零
/*******************IO监看区域**********************************/
/*******************0X80~0X103**********************************/
#define SENSOR_LIMIT_SET          MB_SET_BIT  (80) //限点限位接触
#define SENSOR_LIMIT_RESET        MB_RESET_BIT(80) //限点限位离开

#define SENSOR_ZERO_SET           MB_SET_BIT  (81) //零点限位接触
#define SENSOR_ZERO_RESET         MB_RESET_BIT(81) //零点限位离开

#define ReadEMG_SET               MB_SET_BIT  (82) //急停按下
#define ReadEMG_RESET             MB_RESET_BIT(82) //急停松开

#define BEFORE_RADAR_1_SET        MB_SET_BIT  (83) //前雷达1
#define BEFORE_RADAR_1_RESET      MB_RESET_BIT(83) //
#define BEFORE_RADAR_2_SET        MB_SET_BIT  (84) //前雷达2
#define BEFORE_RADAR_2_RESET      MB_RESET_BIT(84) //
#define BEFORE_RADAR_3_SET        MB_SET_BIT  (85) //前雷达3
#define BEFORE_RADAR_3_RESET      MB_RESET_BIT(85) //
#define BEFORE_RADAR_3_GET        MB_GET_BIT  (85) //

#define AFTER_RADAR_1_SET         MB_SET_BIT  (86) //后雷达1
#define AFTER_RADAR_1_RESET       MB_RESET_BIT(86) //
#define AFTER_RADAR_2_SET         MB_SET_BIT  (87) //后雷达2
#define AFTER_RADAR_2_RESET       MB_RESET_BIT(87) //
#define AFTER_RADAR_3_SET         MB_SET_BIT  (88) //后雷达3
#define AFTER_RADAR_3_RESET       MB_RESET_BIT(88) //
#define AFTER_RADAR_3_GET         MB_GET_BIT  (88) //

#define CRASH_SET                 MB_SET_BIT  (89) //防撞条触发
#define CRASH_RESET               MB_RESET_BIT(89) //

#define START_SET                 MB_SET_BIT  (90) //启动按钮按下
#define START_RESET               MB_RESET_BIT(90) //启动按钮松开

#define LIFT_SQP_SET              MB_SET_BIT  (91) //叉臂接近开关
#define LIFT_SQP_RESET            MB_RESET_BIT(91) 
#define LIFT_SQP_GET              MB_GET_BIT(91) 

#define LIFT_REFLECTION_SET       MB_SET_BIT  (92) //叉臂漫反射传感器
#define LIFT_REFLECTION_RESET     MB_RESET_BIT(92) 
#define LIFT_REFLECTION_GET       MB_GET_BIT  (92)

#define LIFT_LOWER_SET            MB_SET_BIT  (93) //叉臂下限位
#define LIFT_LOWER_RESET          MB_RESET_BIT(93) //

#define LIFT_ENCODER_SET          MB_SET_BIT  (94) //叉臂绝对式拉绳编码器在线
#define LIFT_ENCODER_RESET        MB_RESET_BIT(94) //叉臂绝对式拉绳编码器掉线
/*******************报警信息区域********************************/
/*******************0X104~0X127*********************************/
#define ADC_ALARM_SET             MB_SET_BIT  (104)//ADC报警位
#define ADC_ALARM_RESET           MB_RESET_BIT(104)//ADC报警位
#define ADC_ALARM_GET             MB_GET_BIT  (104)//ADC报警位

#define ADC_SHUTDOWN_SET          MB_SET_BIT  (105)//ADC停止运动位
#define ADC_SHUTDOWN_RESET        MB_RESET_BIT(105)//ADC停止运动位
#define ADC_SHUTDOWN_GET          MB_GET_BIT  (105)

#define TURN_ALARM_SET            MB_SET_BIT  (106) //转向电机报警
#define TURN_ALARM_RESET          MB_RESET_BIT(106) //转向电机不报警
#define TURN_ALARM_GET            MB_GET_BIT  (106)

#define Walk_ALARM_SET            MB_SET_BIT  (107)//行走电报警位
#define Walk_ALARM_RESET          MB_RESET_BIT(107)
#define Walk_ALARM_GET            MB_GET_BIT  (107)

#define WDT_ALARM_SET             MB_SET_BIT  (108)//看门狗溢出报警
#define WDT_ALARM_RESET           MB_RESET_BIT(108)//看门狗溢出报警无异常

#define IPC_BEAT_BIT_SET          MB_SET_BIT  (109)//工控机心跳包异常
#define IPC_BEAT_BIT_RESET        MB_RESET_BIT(109)//工控机心跳包无异常

#define SERIAL1_SET               MB_SET_BIT  (110)//串口1异常
#define SERIAL1_RESET             MB_RESET_BIT(110)//串口1正常

#define SERIAL2_SET               MB_SET_BIT  (111)//串口2异常
#define SERIAL2_RESET             MB_RESET_BIT(111)//串口2正常

#define WEINVIEW_BEAT_BIT_SET     MB_SET_BIT  (112)//威纶通心跳包异常
#define WEINVIEW_BEAT_BIT_RESET   MB_RESET_BIT(112)//威纶通心跳包无异常

#define TURN_LIMIT_ALARM_SET      MB_SET_BIT  (113)//转向电机正极限报警
#define TURN_LIMIT_ALARM_RESET    MB_RESET_BIT(113)//

#define TURN_ZERO_ALARM_SET       MB_SET_BIT  (114)//转向电机负极限报警
#define TURN_ZERO_ALARM_RESET     MB_RESET_BIT(114)//

#define CRASH_ALARM_SET           MB_SET_BIT  (115)//防撞条报警
#define CRASH_ALARM_RESET         MB_RESET_BIT(115)
#define CRASH_ALARM_GET           MB_GET_BIT  (115)

#define LIFT_ENCODER_ALARM_SET    MB_SET_BIT  (116) //叉臂绝对式拉绳编码器掉线报警
#define LIFT_ENCODER_ALARM_RESET  MB_RESET_BIT(116) //
#define LIFT_ENCODER_ALARM_GET    MB_GET_BIT  (116)
/* Exported variables ---------------------------------------------------------*/
extern uint16_t usRegHoldingBuf[REG_HOLDING_NREGS];
extern uint8_t  ucRegCoilsBuf  [REG_COILS_SIZE/ 8];
/* Exported functions prototypes ---------------------------------------------*/
extern void Modbus_Handler(void);
extern void Modbus_Data_Init(void);
extern void VarData_To_Save(void);
extern void VarData_To_Read(void);
/* Exported functions prototypes ---------------------------------------------*/
#endif /* __MB_HANDLER_H */
