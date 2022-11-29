/*
 * FreeModbus Libary: BARE Demo Application
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id$
 */
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : mb_handler.c
  * @brief          : MODBUS处理
  * @date           :2022.08.26
  ******************************************************************************
  * @attention https://www.amobbs.com/thread-5491615-1-1.html
  
  从机地址	功能码	起始地址高位	起始地址低位	寄存器数量高位	寄存器数量低位	CRC高位	CRC低位
  01        03          00          01            00              01              D5      CA
  
  MB_SLAVE1 <-读写-> |           |  使用互斥量避免破坏
 使用互斥量避免破坏  | MB_Buffer |  <-读写-> MB_Handler
  MB_SLAVE2 <-读写-> |           |

  * @author hly
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "mb_handler.h"
#include "mb.h"
#include "mbutils.h"
#include <string.h>
#include <stdlib.h>
/* Private includes ----------------------------------------------------------*/
#include "main.h"
#include "mb_key.h"
#include "adc_dma.h"
#include "monitor.h"
#include "turn_motor.h"
#include "walk_motor.h"
/*ulog include*/
#define LOG_TAG         "mb_handl"
#define LOG_LVL         DBG_INFO
#include <ulog.h>
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define MB_OFFSET 0//屏幕偏移地址。开始地址从1开始
/* Private macro -------------------------------------------------------------*/
/* 线程配置 */
#define THREAD_PRIORITY      11//线程优先级
#define THREAD_TIMESLICE     10//线程时间片
#define THREAD_STACK_SIZE    4096//栈大小

/* Variables' number */
#define NB_OF_VAR             ((uint8_t)20)
/* Private variables ---------------------------------------------------------*/
//输入寄存器内容
uint16_t usRegInputBuf[REG_INPUT_NREGS];
//输入寄存器起始地址
int16_t usRegInputStart = REG_INPUT_START + MB_OFFSET;
//保持寄存器内容
uint16_t usRegHoldingBuf[REG_HOLDING_NREGS];
//保持寄存器起始地址
int16_t usRegHoldingStart = REG_HOLDING_START + MB_OFFSET;
//线圈状态
uint8_t  ucRegCoilsBuf[REG_COILS_SIZE / 8];
//线圈起始地址
int16_t ucRegCoilsStart = REG_COILS_START;
//开关输入状态
uint8_t  ucRegDiscreteBuf[REG_DISCRETE_SIZE / 8];
//开关起始地址
int16_t ucRegDiscreteStart = REG_DISCRETE_START;
/* Private function prototypes -----------------------------------------------*/
extern int Flash_KVDB_Init(void);
/**
  * @brief  MODBUS--数据初始化
  * @param  None
  * @retval None
  * @note   完成数据传入传出
*/
void Modbus_Data_Init(void)
{
  /*线圈初始化*/
  TURN_ENABLE_SET;
  WALK_ENABLE_SET;
  /*保持寄存器初始化*/
  /*转向电机设置区域*/
  usRegHoldingBuf[501] = (ushort)Turn_Cycle_Pulse_Num;                                          //设置转向电机周期脉冲数
  usRegHoldingBuf[502] = (ushort)Turn_REDUCTION_RATIO;                                          //设置转向电机减速比
  usRegHoldingBuf[503] = (ushort)Turn_CLOCKWISE;                                                //顺时针方向对应的值
  usRegHoldingBuf[504] = (ushort)Turn_ANGLE_MAX;                                                //设置最大角度
  usRegHoldingBuf[505] = (ushort)Turn_ANGLE_MIN;                                                //设置最小角度
  usRegHoldingBuf[506] = (ushort)Turn_SPEED_MAX;                                                //设置最大速度
  usRegHoldingBuf[507] = (ushort)Turn_SPEED_MIN;                                                //设置最小速度
  usRegHoldingBuf[508] = (ushort)Turn_SPEED_JOG;                                                //设置转向电机复位角度
  usRegHoldingBuf[509] = (ushort)Turn_SANGLE_INIT;                                              //设置转向电机复位角度
  usRegHoldingBuf[510] = (ushort)Turn_BACK_HIGH_SPEED;                                          //回原高速
  usRegHoldingBuf[511] = (ushort)Turn_BACK_LOW_SPEED;                                           //回原低速
  /*行走电机设置区域*/
  usRegHoldingBuf[521] = (ushort)Walk_Cycle_Pulse_Num;                                          //设置电机周期脉冲数
  usRegHoldingBuf[522] = (ushort)Walk_REDUCTION_RATIO;                                          //设置电机减速比
  usRegHoldingBuf[523] = (ushort)Walk_CLOCKWISE;                                                //顺时针方向对应的值
  usRegHoldingBuf[524] = (ushort)Walk_SPEED_MAX;                                                //最大限速
  usRegHoldingBuf[525] = (ushort)Walk_SPEED_MIN;                                                //最小限速
  usRegHoldingBuf[526] = (ushort)Walk_STEP;                                                     //加减速步长
  usRegHoldingBuf[527] = (ushort)Walk_TIME;                                                     //加减速周期
  
  usRegHoldingBuf[699] = (ushort)0;                                                             //ADC采样补偿
  usRegHoldingBuf[700] = (ushort)2200;                                                          //adc阈值
  /*编译信息*/
  strcpy((char *)(usRegHoldingBuf+900),VERSION);                                        //打印版本信息
  usRegHoldingBuf[988] = (ushort)((uint32_t)(YEAR*10000+(MONTH + 1)*100+DAY) & 0xffff); //日期低16bti
	usRegHoldingBuf[989] = (ushort)((uint32_t)(YEAR*10000+(MONTH + 1)*100+DAY)>>16);		  //日期高16bti
	usRegHoldingBuf[990] = (ushort)((uint32_t)(HOUR*10000+MINUTE*100+SEC)&0xffff);        //时间低16bti
	usRegHoldingBuf[991] = (ushort)((uint32_t)(HOUR*10000+MINUTE*100+SEC)>>16);		        //时间高16bti
  /*ID参数区域[随机数]*/
  usRegHoldingBuf[992] =  HAL_GetUIDw0();
  usRegHoldingBuf[993] =  HAL_GetUIDw0() >> 16;
  usRegHoldingBuf[994] =  HAL_GetUIDw1();
  usRegHoldingBuf[995] =  HAL_GetUIDw1() >> 16;
  usRegHoldingBuf[996] =  HAL_GetUIDw2();
  usRegHoldingBuf[997] =  HAL_GetUIDw2() >> 16;
  usRegHoldingBuf[998] =  HAL_GetHalVersion();
  usRegHoldingBuf[999] =  HAL_GetHalVersion() >> 16;
}
/**
  * @brief  MODBUS--数据处理函数初始化
  * @param  None
  * @retval None
  * @note   完成数据传入传出
*/
void Modbus_Handler_Init(void)
{
  Modbus_Data_Init();
  /*MB按键初始化*/
  MBKEY_Init();
  /*数据库初始化*/
  Flash_KVDB_Init();
}
/**
  * @brief  MODBUS--数据处理函数
  * @param  None
  * @retval None
  * @note   完成数据传入传出
*/
void Modbus_Handler(void)
{
/**********************工控机处理地址[0-500]**************************************/
/*******传入位********************************************************************/ 

/*******传出位********************************************************************/

/*******传入寄存器[1-100]****************************************************************/
  /*转向电机设置区域*/
  turn.set_radian                          = _float(usRegHoldingBuf[1])/1000.0f;                 //设置转向电机【1】角度【单位：弧度】【放大1000倍，精确小数点后3位】
  //  turn_motor[1].set_radian                 = (_short)usRegHoldingBuf[2]/1000.0f;              //设置转向电机【2】角度【单位：弧度】【放大1000倍，精确小数点后3位】
  //  turn_motor[2].set_radian                 = (_short)usRegHoldingBuf[3]/1000.0f;              //设置转向电机【3】角度【单位：弧度】【放大1000倍，精确小数点后3位】
  //  turn_motor[3].set_radian                 = (_short)usRegHoldingBuf[4]/1000.0f;              //设置转向电机【4】角度【单位：弧度】【放大1000倍，精确小数点后3位】
  /*行走电机设置区域*/
  walk.motor.set_speed                     = (_short)usRegHoldingBuf[5];                         //设置行走电机【1】转速【单位：RPM】【需提前设置最大RPM限速】
//  walk_motor[1].set_speed                  = (_short)usRegHoldingBuf[6];                        //设置行走电机【2】转速【单位：RPM】
//  walk_motor[2].set_speed                  = (_short)usRegHoldingBuf[7];                        //设置行走电机【3】转速【单位：RPM】
//  walk_motor[3].set_speed                  = (_short)usRegHoldingBuf[8];                        //设置行走电机【4】转速【单位：RPM】
  /*其他处理区域*/
  IPC_Beat.Value                           = (ushort)usRegHoldingBuf[199];                       //工控机心跳包。1s写入一次。1.2s清除一次
  IPC_Beat.EN                              = (ushort)usRegHoldingBuf[200];                       //工控机心跳检测开关                  
/*******传出寄存器[201-500]****************************************************************/
  /*转向电机查询区域*/
  usRegHoldingBuf[201]                     = (_short)(turn.get_radian*1000);                     //查询转向电机【1】角度【单位：弧度】【放大1000倍，精确小数点后3位】
  //  usRegHoldingBuf[202]                  = (_short)(turn_motor[1].get_radian*1000);             //查询转向电机【2】角度【单位：弧度】【放大1000倍，精确小数点后3位】
  //  usRegHoldingBuf[203]                  = (_short)(turn_motor[2].get_radian*1000);             //查询转向电机【3】角度【单位：弧度】【放大1000倍，精确小数点后3位】
  //  usRegHoldingBuf[204]                  = (_short)(turn_motor[3].get_radian*1000);             //查询转向电机【4】角度【单位：弧度】【放大1000倍，精确小数点后3位】
  /*行走查询速度区域*/
  usRegHoldingBuf[205]                     = (_short)walk.motor.get_speed;                       //查询行走电机【1】转速【单位：RPM 】
//  usRegHoldingBuf[206]                     = (_short)walk_motor[1].get_speed;                    //查询行走电机【2】转速【单位：RPM 】
//  usRegHoldingBuf[207]                     = (_short)walk_motor[2].get_speed;                    //查询行走电机【3】转速【单位：RPM 】
//  usRegHoldingBuf[208]                     = (_short)walk_motor[3].get_speed;                    //查询行走电机【4】转速【单位：RPM 】
  usRegHoldingBuf[500]                     = (ushort)(voltage.AD.vaule*ADC_RATIO*100);           //电池电压
/**********************调试处理地址[501-999]**************************************/
/*******传入位********************************************************************/ 

/*******传出位********************************************************************/

/*******传入寄存器[501-700]****************************************************************/
  /*转向电机设置区域*/
  turn.motor.Ctrl.cycle_pulse_num       = (ushort)usRegHoldingBuf[501];                          //设置电机周期脉冲数
  turn.motor.Ctrl.reduction_ratio       = (ushort)usRegHoldingBuf[502];                          //设置电机减速比
  turn.motor.Ctrl.clockwise             = (ushort)usRegHoldingBuf[503];                          //顺时针方向对应的值
  turn.motor.Abs.max_angle              = (ushort)usRegHoldingBuf[504];                          //设置最大角度
  turn.motor.Abs.min_angle              = (ushort)usRegHoldingBuf[505];                          //设置最小角度
  turn.motor.V.max_speed                = (ushort)usRegHoldingBuf[506],                          //设置最大速度
  turn.motor.V.min_speed                = (ushort)usRegHoldingBuf[507];                          //设置最小速度
  turn.motor.V.set_speed                = (ushort)usRegHoldingBuf[508];                          //设置点动速度
  turn.motor.Do.init_freq               =  _float(usRegHoldingBuf[509])/1000.0f;                 //设置转向电机复位角度
  turn.motor.Do.speed_high              = (_short)usRegHoldingBuf[510];                          //回原高速
  turn.motor.Do.speed_low               = (_short)usRegHoldingBuf[511];                          //回原低速
  /*行走电机设置区域*/
  walk.motor.cycle_pulse_num            = (ushort)usRegHoldingBuf[521];                          //设置电机周期脉冲数
  walk.motor.reduction_ratio            = (ushort)usRegHoldingBuf[522];                          //设置电机减速比
  walk.motor.clockwise                  = (ushort)usRegHoldingBuf[523];                          //顺时针方向对应的值
  walk.motor.max_speed                  = (ushort)usRegHoldingBuf[524];                          //最大限速
  walk.motor.min_speed                  = (ushort)usRegHoldingBuf[525];                          //最小限速
  walk.motor.AND.step                   = (ushort)usRegHoldingBuf[526];                          //加减速步长
  walk.motor.AND.time                   = (ushort)usRegHoldingBuf[527];                          //加减速周期
  
  WDT_key                               = (ushort)usRegHoldingBuf[687];                          //看门狗密码输入
  Weinview_Beat.Value                   = (ushort)usRegHoldingBuf[688];                          //威纶通心跳包。1s写入一次。1.2s清除一次
  Weinview_Beat.EN                      = (ushort)usRegHoldingBuf[689];                          //威纶通心跳检测开关                  
  voltage.AD.Compensation               = _float(usRegHoldingBuf[699])/9.0f/100;                 //ADC采样补偿
  voltage.VPT                           = _float(usRegHoldingBuf[700])/100.0f;                   //adc阈值
/*******传出寄存器[701-999]****************************************************************/
  /*转向电机查询参数区域*/
  usRegHoldingBuf[701]                  = (ushort)turn.state.MOTOR;                              //查询电机状态
  usRegHoldingBuf[702]                  = (ushort)(turn.set_freq / 10);                          //查询给入的频率
  usRegHoldingBuf[703]                  = (_short)(turn.get_freq / 10);                          //查询计算的频率
  usRegHoldingBuf[704]                  = (ushort)turn.PIN.Dir.level;                            //方向标志
  usRegHoldingBuf[705]                  = (ushort)turn.PIN.Brake.level;                          //刹车标志
  usRegHoldingBuf[706]                  = (ushort)turn.PIN.EN.level;                             //使能标志
  usRegHoldingBuf[707]                  = (_short)turn.motor.Ctrl.CurrentPosition_Pulse;         //机械位置
  usRegHoldingBuf[708]                  = (ushort)turn.motor.Rel.dest_position;                  //所需脉冲
  usRegHoldingBuf[709]                  = (ushort)turn.motor.Abs.over_range;                     //输入是否超出角度
  usRegHoldingBuf[710]                  = (_short)turn.motor.V.get_speed;                        //计算的速度
  usRegHoldingBuf[711]                  = (ushort)turn.motor.Do.flag;                            //归零函数状态
  usRegHoldingBuf[712]                  = (ushort)turn.motor.Do.init_state;                      //电机初始化状态
  usRegHoldingBuf[713]                  = (ushort)turn.limit_mode;                               //电机限位模式
  usRegHoldingBuf[714]                  = (ushort)turn.Stop_state;                              //电机停止代码
  /*行走电机查询参数区域*/
  usRegHoldingBuf[721]                  = (ushort)walk.state.MOTOR;                              //查询电机状态
  usRegHoldingBuf[722]                  = (ushort)(walk.motor.set_freq / 10);                    //查询给入的频率
  usRegHoldingBuf[723]                  = (ushort)walk.PIN.Brake.level;                          //刹车标志
  usRegHoldingBuf[724]                  = (ushort)walk.PIN.Dir.level;                            //方向标志
  usRegHoldingBuf[725]                  = (ushort)walk.PIN.EN.level;                             //使能标志
  usRegHoldingBuf[726]                  = (ushort)walk.limit_mode;                               //电机限位模式
  usRegHoldingBuf[727]                  = (ushort)walk.Stop_state;                               //电机停止代码
  /*其他查询参数区域*/
  
  usRegHoldingBuf[802]                  = (ushort)RestFlag;                                      //复位标志
  usRegHoldingBuf[803]                  = (_short)Weinview_Beat.Error;                           //查询威纶通心跳间隔时间
  usRegHoldingBuf[804]                  = (_short)IPC_Beat.Error;                                //查询工控机心跳间隔时间
}
/**
  * @brief  modbus初始化
  * @param  None.
  * @retval None.
  * @note   开启MB处理线程，并初始化信号量再MB串口判断为MODBUS处理函数时进行数据读写
*/
static int Modbus_Init(void)
{
  rt_err_t ret = RT_EOK;    
  Modbus_Handler_Init();
  Turn_Motor_Init();
  Walk_Motor_Init();
  /* 创建 MODBUS线程*/
  rt_thread_t thread = rt_thread_create( "mb key",    /* 线程名字 */
                                         MBKey_Handler,/* 线程入口函数 */
                                         RT_NULL,       /* 线程入口函数参数 */
                                         THREAD_STACK_SIZE, /* 线程栈大小 */
                                         THREAD_PRIORITY,   /* 线程的优先级 */
                                         THREAD_TIMESLICE); /* 线程时间片 */
  /* 创建成功则启动线程 */
  if (thread != RT_NULL)
  {
      rt_thread_startup(thread);
  }
  else
  {
      ret = RT_ERROR;
      LOG_E("modbus slave created failed.");
  }
  return ret;
}
/* 导出到 msh 命令列表中 */
INIT_ENV_EXPORT(Modbus_Init);
/**
  * @brief  打印MODBUS列表
  * @param  第一个参数，int型的argc，为整型，用来统计程序运行时发送给main函数的命令行参数的个数
  * @retval 第二个参数，char*型的argv[]，为字符串数组，用来存放指向的字符串参数的指针数组，每一个元素指向一个参数。
  * @note   None.
*/
static void Modbus_list(int argc, char**argv)
{
  int16_t len,start,cnt = 0,flag = 0;
  if (argc < 3)
  {
      rt_kprintf("Please input'Modbus_list <type|start_addr|lenth>'\n");
      rt_kprintf("       e.g : Modbus_list  hold    100       10\n");
      rt_kprintf("      type : hold | coil\r\n");
      rt_kprintf("      EN   : EN is Only look at not zero data\r\n");
      return;
  }
  if(strspn(argv[2], "0123456789") == strlen(argv[2]))//判断传入字符串是否全数字
  {
    start = atoi(argv[2]);
  }
  else
  {
    rt_kprintf("Please enter only digits\n");
    return;
  }
  
  if(strspn(argv[3], "0123456789") == strlen(argv[3]))//判断传入字符串是否全数字
  {
    len = atoi(argv[3]);
  }
  else
  {
    rt_kprintf("Please enter only digits\n");
    return;
  }
  if(argv[4])
  {
    if(!rt_strcmp(argv[4],"EN"))
    {
      flag = 1;
    }
  }
  if(!rt_strcmp(argv[1], "hold"))
  {
    if(start+len > REG_INPUT_NREGS)
    {
      rt_kprintf("Input out of index range\r\n");
      return;
    }
    for(uint16_t i = start;i < start+len;i++)
    {
      if(flag == 1)
      {
        if(usRegHoldingBuf[i] != 0)
        {
          if(cnt % 5 == 0)
          {
            rt_kprintf("\r\n");
            cnt = 0;
          }
          cnt++;
          rt_kprintf("num:%3d|hold:%5d#",i,usRegHoldingBuf[i]);
        }
      }
      else
      {
        if(cnt % 5 == 0)
        {
          rt_kprintf("\r\n");
          cnt = 0;
        }
        cnt++;
        rt_kprintf("num:%3d|hold:%5d#",i,usRegHoldingBuf[i]);
      }
    }
    rt_kprintf("\r\n");
  }
  else if (!rt_strcmp(argv[1], "coil"))
  {
    if(start+len > REG_COILS_SIZE)
    {
      rt_kprintf("Input out of index range\r\n");
      return;
    }
    for(uint16_t i = start;i < start+len;i++)
    {
      if(flag == 1)
      {
        if(MB_GET_BIT(i)!=0)
        {
          if(cnt % 7 == 0)
          {
            rt_kprintf("\r\n");
            cnt = 0;
          }
          cnt++;
          rt_kprintf("num:%3d|coil:1#",i);
        }
      }
      else
      {
        if(cnt % 7 == 0)
        {
          rt_kprintf("\r\n");
          cnt = 0;
        }
        cnt++;
        if(MB_GET_BIT(i))
        {
          rt_kprintf("num:%3d|coil:1#",i);
        }
        else
        {
          rt_kprintf("num:%3d|coil:0#",i);
        }
      }
    }
    rt_kprintf("\r\n");
  }
  else
  {
      rt_kprintf("Please input'Modbus_list <type|start_addr|lenth>'\n");
      rt_kprintf("       e.g : Modbus_list  hold    100       10\n");
      rt_kprintf("      type : hold | coil\r\n");
      rt_kprintf("      EN   : EN is Only look at not zero data\r\n");
      return;
  }
}
MSH_CMD_EXPORT_ALIAS(Modbus_list,Modbus_list,Modbus_list <type|start_addr|lenth>);
/**
  * @brief  设置MODBUS数据
  * @param  第一个参数，int型的argc，为整型，用来统计程序运行时发送给main函数的命令行参数的个数
  * @retval 第二个参数，char*型的argv[]，为字符串数组，用来存放指向的字符串参数的指针数组，每一个元素指向一个参数。
  * @note   None.
*/
static void Modbus_Set(int argc, char**argv)
{
  int16_t num,addr;
  if (argc < 3)
  {
      rt_kprintf("Please input'Modbus_Set <type|addr|num>'\n");
      rt_kprintf("       e.g : Modbus_Set  hold 100  10\n");
      rt_kprintf("       type: hold | coil\r\n");
      return;
  }
  if(rt_strcmp(argv[1], "hold") && rt_strcmp(argv[1], "coil"))
  {
    rt_kprintf("Please enter the:hold | coil, before entering the value\n");
    return;
  }
  if(strspn(argv[2], "0123456789") == strlen(argv[2]))//判断传入字符串是否全数字
  {
    addr = atoi(argv[2]);
  }
  else
  {
    rt_kprintf("Please enter only digits\n");
    return;
  }
  
  if(strspn(argv[3], "0123456789") == strlen(argv[3]))//判断传入字符串是否全数字
  {
    num = atoi(argv[3]);
  }
  else
  {
    rt_kprintf("Please enter only digits\n");
    return;
  }
  if(!rt_strcmp(argv[1], "hold"))
  {
    if(addr > REG_INPUT_NREGS || addr < 0)
    {
      rt_kprintf("Input out of index range\r\n");
      return;
    }
    usRegHoldingBuf[addr] = num;
    rt_kprintf("usRegHoldingBuf[%d] set value:%d\r\n",addr,num);
  }
  else if (!rt_strcmp(argv[1], "coil"))
  {
    if(addr > REG_COILS_SIZE || addr < 0 || (num != 0 && num != 1))
    {
      rt_kprintf("Input out of index range\r\n");
      return;
    }
    if(num)
      MB_SET_BIT(addr);
    else
      MB_RESET_BIT(addr);
    rt_kprintf("ucRegCoilsBuf[%d] set value:%d\r\n",addr,num);
  }
  else
  {
      rt_kprintf("Please input'Modbus_Set <type|start_addr|lenth|EN>'\n");
      rt_kprintf("       e.g : Modbus_Set  hold    100       10\n");
      rt_kprintf("       type: hold | coil\r\n");
      return;
  }
}
MSH_CMD_EXPORT_ALIAS(Modbus_Set,Modbus_Set,Modbus_Set <type|addr|num>);
/****************************************************************************
* 名	  称:eMBRegInputCB 
* 功    能:读取输入寄存器,对应功能码是 04 eMBFuncReadInputRegister
* 入口参数:pucRegBuffer: 数据缓存区,用于响应主机   
*						usAddress: 寄存器地址
*						usNRegs: 要读取的寄存器个数
* 出口参数:
* 注	  意:上位机发来的 帧格式是: SlaveAddr(1 Byte)+FuncCode(1 Byte)
*								+StartAddrHiByte(1 Byte)+StartAddrLoByte(1 Byte)
*								+LenAddrHiByte(1 Byte)+LenAddrLoByte(1 Byte)+
*								+CRCAddrHiByte(1 Byte)+CRCAddrLoByte(1 Byte)
*							3 区
****************************************************************************/
eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;
    //查询是否在寄存器范围内
    //为了避免警告，修改为有符号整数
    if( ( usAddress >= REG_INPUT_START )
        && ( usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS ) )
    {   //获得操作偏移量，本次操作起始地址-输入寄存器的初始地址
        iRegIndex = ( int )( usAddress - usRegInputStart );
        while( usNRegs > 0 )//逐个赋值
        {
            *pucRegBuffer++ = ( UCHAR )( usRegInputBuf[iRegIndex] >> 8 );//赋值高字节
            *pucRegBuffer++ = ( UCHAR )( usRegInputBuf[iRegIndex] & 0xFF );//赋值低字节
            iRegIndex++;//偏移量增加
            usNRegs--;//被操作寄存器数量递减
        }
    }
    else
    {
        eStatus = MB_ENOREG;//返回错误状态，无寄存器
    }

    return eStatus;
}

/****************************************************************************
* 名	  称:eMBRegHoldingCB 
* 功    能:对应功能码有:06 写保持寄存器 eMBFuncWriteHoldingRegister 
*													16 写多个保持寄存器 eMBFuncWriteMultipleHoldingRegister
*													03 读保持寄存器 eMBFuncReadHoldingRegister
*													23 读写多个保持寄存器 eMBFuncReadWriteMultipleHoldingRegister
* 入口参数:pucRegBuffer: 数据缓存区,用于响应主机   
*						usAddress: 寄存器地址
*						usNRegs: 要读写的寄存器个数
*						eMode: 功能码
* 出口参数:
* 注	  意:4 区
****************************************************************************/
eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode )
{
	eMBErrorCode    eStatus = MB_ENOERR;//错误状态
	int             iRegIndex;          //偏移量

  //判断寄存器是不是在范围内
	if((usAddress >= REG_HOLDING_START)&&\
		((usAddress+usNRegs) <= (REG_HOLDING_START + REG_HOLDING_NREGS)))
	{
		iRegIndex = (int)(usAddress - usRegHoldingStart);//计算偏移量
		switch(eMode)
		{                                       
			case MB_REG_READ://读 MB_REG_READ = 0//读处理函数
        while(usNRegs > 0)
				{
					*pucRegBuffer++ = (uint8_t)(usRegHoldingBuf[iRegIndex] >> 8);            
					*pucRegBuffer++ = (uint8_t)(usRegHoldingBuf[iRegIndex] & 0xFF); 
          iRegIndex++;
          usNRegs--;					
				}                            
        break;
			case MB_REG_WRITE://写 MB_REG_WRITE = 0//写处理函数
				while(usNRegs > 0)
				{         
					usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
          usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
          iRegIndex++;
          usNRegs--;
        }		
        break;
			}
	}
	else//错误
	{
		eStatus = MB_ENOREG;//返回错误状态
	}	
	
	return eStatus;
}

/****************************************************************************
* 名	  称:eMBRegCoilsCB 
* 功    能:对应功能码有:01 读线圈 eMBFuncReadCoils
*													05 写线圈 eMBFuncWriteCoil
*													15 写多个线圈 eMBFuncWriteMultipleCoils
* 入口参数:pucRegBuffer: 数据缓存区,用于响应主机   
*						usAddress: 线圈地址
*						usNCoils: 要读写的线圈个数
*						eMode: 功能码
* 出口参数:
* 注	  意:如继电器 
           FF00H请求线圈处于ON状态，0000H请求线圈处于OFF状态。
*						0 区
****************************************************************************/
eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode )
{
  //错误状态
  eMBErrorCode eStatus = MB_ENOERR;
  //寄存器个数
  int16_t iNCoils = ( int16_t )usNCoils;
  //寄存器偏移量
  int16_t usBitOffset;

  //检查寄存器是否在指定范围内
  if( ( (int16_t)usAddress >= REG_COILS_START ) && ( usAddress + usNCoils <= REG_COILS_START + REG_COILS_SIZE ) )
  {
    //计算寄存器偏移量
    usBitOffset = ( int16_t )( usAddress - ucRegCoilsStart );
    switch ( eMode )
    {
      //读操作
      case MB_REG_READ:
      while( iNCoils > 0 )
      {
        *pucRegBuffer++ = xMBUtilGetBits( ucRegCoilsBuf, usBitOffset,
        ( uint8_t )( iNCoils > 8 ? 8 : iNCoils ) );
        iNCoils -= 8;
        usBitOffset += 8;
      }
      break;

      //写操作
      case MB_REG_WRITE:
      while( iNCoils > 0 )
      {
        xMBUtilSetBits( ucRegCoilsBuf, usBitOffset,
        ( uint8_t )( iNCoils > 8 ? 8 : iNCoils ),
        *pucRegBuffer++ );
        iNCoils -= 8;
      }
      break;
    }
  }
  else
  {
    eStatus = MB_ENOREG;
  }
  return eStatus;
}
/****************************************************************************
* 名	  称:eMBRegDiscreteCB 
* 功    能:读取离散寄存器,对应功能码有:02 读离散寄存器 eMBFuncReadDiscreteInputs
* 入口参数:pucRegBuffer: 数据缓存区,用于响应主机   
*						usAddress: 寄存器地址
*						usNDiscrete: 要读取的寄存器个数
* 出口参数:
* 注	  意:1 区
****************************************************************************/
eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
  //错误状态
  eMBErrorCode eStatus = MB_ENOERR;
  //操作寄存器个数
  int16_t iNDiscrete = ( int16_t )usNDiscrete;
  //偏移量
  uint16_t usBitOffset;

  //判断寄存器时候再制定范围内
  if( ( (int16_t)usAddress >= REG_DISCRETE_START ) &&
  ( usAddress + usNDiscrete <= REG_DISCRETE_START + REG_DISCRETE_SIZE ) )
  {
    //获得偏移量
    usBitOffset = ( uint16_t )( usAddress - ucRegDiscreteStart );

    while( iNDiscrete > 0 )
    {
      *pucRegBuffer++ = xMBUtilGetBits( ucRegDiscreteBuf, usBitOffset,
      ( uint8_t)( iNDiscrete > 8 ? 8 : iNDiscrete ) );
      iNDiscrete -= 8;
      usBitOffset += 8;
    }

  }
  else
  {
    eStatus = MB_ENOREG;
  }
  return eStatus;
}
