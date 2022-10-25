/*
This file is part of CanFestival, a library implementing CanOpen Stack. 

Copyright (C): Edouard TISSERANT and Francis DUPIN

See COPYING file for copyrights details.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : 
  * @brief          : 
  * @date           :
  ******************************************************************************
  * @attention
  * 异常处理
  * MCU 初始化时:CAN总线断开[已处理] 节点掉线[已处理]
  * MCU 预操作时:CAN总线断开[已处理] 节点掉线[已处理]
  * MCU 操作态时:CAN总线断开[已处理] 节点掉线[已处理]
  * @author
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "canopen_callback.h"
/* Private includes ----------------------------------------------------------*/
#include <rtthread.h>
#include <stdlib.h>
#include "canfestival.h"

#include "master402_od.h"
#include "master402_canopen.h"
/* Private typedef -----------------------------------------------------------*/
/*修复程序结构体
*/
typedef struct
{
  UNS8 lock;    //锁定线程，确保只有一个线程创建
  UNS8 try_cnt; //修复尝试次数
}Fix_Typedef;
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static Fix_Typedef node;
static Fix_Typedef cfg;
/* Private function prototypes -----------------------------------------------*/
static void master402_fix_node_Disconnected(void* parameter);
/**
  * @brief  主站节点心跳异常回调
  * @param  None.
  * @retval None.
  * @note   产生原因：1.从机掉线。从机上线后可通过emcy回调解决
  * 				 2.主机从机间can线异常。查询从机节点状态从Disconnected切换后，证明从机重新上线。
*/
void master402_heartbeatError(CO_Data* d, UNS8 heartbeatID)
{
  if(++node.try_cnt <= 5)
  {
    LOG_E("heartbeatError!heartbeatID:0x%x,try cnt = %d", heartbeatID,node.try_cnt);
    if(node.lock == 0)
    {
      rt_thread_t tid;
      tid = rt_thread_create("fix_nodeID", master402_fix_node_Disconnected,
                              (void *)(int)heartbeatID,//强制转换为16位数据与void*指针字节一致，以消除强制转换大小不匹配警告
                              2048, 12, 2);

      if(tid == RT_NULL)
      {
        LOG_E("canfestival get nodeid state thread start failed!");
      }
      else
      {
        rt_thread_startup(tid);
      }
    }
    else
    {
      LOG_W("Unable to create a new thread because an existing thread is running");
    }
  }
  else if(node.try_cnt == 4)
  {
     LOG_E("The number of repairs is too many. It is confirmed that it is not an occasional anomaly. It will not be repaired any more.");
  }
}
/**
  * @brief  主站节点初始化回调.
  * @param  None.
  * @retval None.
  * @note   None.
*/
void master402_initialisation(CO_Data* d)
{
	LOG_I("canfestival enter initialisation state");
}
/**
  * @brief  主站节点预操作回调.
  * @param  None.
  * @retval None.
  * @note   开启从站参数配置操作
*/
void master402_preOperational(CO_Data* d)
{
	rt_thread_t tid;
	LOG_I("canfestival enter preOperational state");
	tid = rt_thread_create("co_cfg", canopen_start_thread_entry, RT_NULL, 2048, 12, 2);
	if(tid == RT_NULL)
	{
		LOG_E("canfestival config thread start failed!");
	}
	else
	{
		rt_thread_startup(tid);
	}
}
/**
  * @brief  主站节点进入操作模式回调.
  * @param  None.
  * @retval None.
  * @note   None.
*/
void master402_operational(CO_Data* d)
{
  extern void canSend_Clear_errcnt(void);
  canSend_Clear_errcnt();
	LOG_I("canfestival enter operational state");
}
/**
  * @brief  主站节点进入停止回调.
  * @param  None.
  * @retval None.
  * @note   None.
*/
void master402_stopped(CO_Data* d)
{
	LOG_E("canfestival enter stop state");
}
/**
  * @brief  主站节点发送sync回调.
  * @param  None.
  * @retval None.
  * @note   None.
*/
void master402_post_sync(CO_Data* d)
{
//	rt_kprintf(" SYNC received. Proceed.\n");
}
/**
  * @brief  主站节点初发送TPDO回调.
  * @param  None.
  * @retval None.
  * @note   None.
*/
void master402_post_TPDO(CO_Data* d)
{
//    rt_kprintf("Send TPDO \n");
}
/**
  * @brief  主站节点储存回调.
  * @param  None.
  * @retval None.
  * @note   None.
*/
void master402_storeODSubIndex(CO_Data* d, UNS16 wIndex, UNS8 bSubindex)
{
	/*TODO : 
	 * - call getODEntry for index and subindex, 
	 * - save content to file, database, flash, nvram, ...
	 * 
	 * To ease flash organisation, index of variable to store
	 * can be established by scanning d->objdict[d->ObjdictSize]
	 * for variables to store.
	 * 
	 * */
	LOG_I("storeODSubIndex : %4.4x %2.2x", wIndex,  bSubindex);
}
/**
  * @brief  主站节点emcy紧急事件回调.
  * @param  None.
  * @retval None.
  * @note   None.
errReg:如果某一特定的错误发生相应位置 1b。通用错误位是强制设备必须支持的，其它可选。任何错误产生都将置位通用错误。
位 M/O     意义
0   M     通用错误
1   O     电流
2   O     电压
3   O     温度
4   O     通信错误(溢出、错误状态)
5   O     设备协议指定
6   O     保留(始终为 0b)
7   O     制造商指定

ErrorCode: 6200  用户软件 -常规错误[]
ErrorCode: 3130  主回路电源异常[驱动器掉电]
ErrorCode: 8200  协议错误–常规 AL121 当上位机发送 PDO 给驱动器时，PDO 所指定的对象字典 Index 号码不正确，导致驱动器无法辨识。
                              AL124 PDO所欲存取的对象字典范围错误 讯息中的数据超出指定对象字典的范围[例如写入速度4000超出0~3000范围]
ErrorCode: 8110  CAN overflow (object loss) AL113 TxPDO 传送失败
*/
void master402_post_emcy(CO_Data* d, UNS8 nodeID, UNS16 errCode, UNS8 errReg, const UNS8 errSpec[5])
{
  if(errCode != 0)//应急错误代码
  {
      LOG_E("received EMCY message. Node: %2.2x  ErrorCode: %4.4x  ErrorRegister: %2.2x", nodeID, errCode, errReg);
      if(errSpec[0]+errSpec[1]+errSpec[2]+errSpec[3]+errSpec[4])
      {
        LOG_E("Manufacturer Specific: 0X%X %2.2X %2.2X %2.2X %2.2X",errSpec[0],errSpec[1],errSpec[2],errSpec[3],errSpec[4]);
      }
      if(errCode == 0x8130)//节点保护错误或者心跳错误
      {
        /*产生此错误代码原因：从机掉线或者can信号线异常、掉线
          能够接收到此错误代码，证明通信恢复正常。此错误可以通过重置NMT清除。
          从机进入pre状态，不可进行操作
          切换从机进入start状态*/
        LOG_W("ErrorCode: %4.4x :Node protection error or heartbeat error",errCode);
        masterSendNMTstateChange(d,nodeID,NMT_Start_Node);
        LOG_W("Determine the line recovery and switch the slave machine into operation mode");
      }
  }
}
/*******************************ERROR FIX CODE*****************************************************************/
/**
  * @brief  修复节点NMT异常
  * @param  None.
  * @retval None.
  * @note   查询从机节点状态变为pre后，证明从机重新上线。
            可以进行NMT命令重新恢复通信
*/
static void master402_fix_node_Disconnected(void* parameter)
{
	static e_nodeState last;
	e_nodeState now,master;

	int heartbeatID = (int)parameter;//强制转换为16位数据与void*指针字节一致，以消除强制转换大小不匹配警告
	LOG_E("heartbeatID abnormal:0x%x  ", heartbeatID);
  node.lock = 0xff;
	while (1)
	{
		last = now;
		now = getNodeState(OD_Data,heartbeatID);
		if(now != last)//节点状态发生变化，证明从机上线
		{
			if(now == Operational)//由0x8130错误码处理程序处理
			{
        LOG_I("Handled by the 0x8130 error code handler,def ThreadFinished");
        node.lock = 0;
				return;//删除线程
			}
			else if(now == Pre_operational)
			{
        master = getState(OD_Data);//获取主站节点状态
        if(master == Stopped)//can通信异常，发送失败多次，进入停止状态
        {
          //Stop状态时会删除生产者心跳定时器
          if (*OD_Data->ProducerHeartBeatTime)//恢复到OP状态时检查是否有生产者心跳时间
          {
            TIMEVAL time = *OD_Data->ProducerHeartBeatTime;
            extern void ProducerHeartbeatAlarm(CO_Data* d, UNS32 id);
            //设置生产者时间定时器，并设置定时回调
            LOG_W("Restart the producer heartbeat");
            OD_Data->ProducerHeartBeatTimer = SetAlarm(OD_Data, 0, &ProducerHeartbeatAlarm, MS_TO_TIMEVAL(time), MS_TO_TIMEVAL(time));
          }
          LOG_W("The master station enters the operation state from the stop state");
          setState(OD_Data, Operational);//转入Operational状态
        }
				masterSendNMTstateChange(OD_Data,heartbeatID,NMT_Start_Node);
				LOG_W("Determine the line recovery and switch the slave machine into operation mode,def ThreadFinished");
        node.lock = 0;
				return;//退出线程
			}
      else if(now == Initialisation)
      {
        setState(OD_Data, Initialisation);//Initialisation
        LOG_I("After the heartbeat of the node is abnormal, the node is shut down and powered on");
        node.lock = 0;
				return;//退出线程
      }
		}

		rt_thread_mdelay(CONSUMER_HEARTBEAT_TIME);//按照获取心跳间隔时间进行判断
	}
}
/**
  * @brief  修复配置错误线程
  * @param  None.
  * @retval None.
  * @note   None.
*/
static void master402_fix_config_err_thread_entry(void* parameter)
{
	static e_nodeState last;
	e_nodeState now;
  int nodeId = (int)parameter;//强制转换为16位数据与void*指针字节一致，以消除强制转换大小不匹配警告
	LOG_E("nodeId abnormal:0x%x  ", nodeId);
  while (1)
  {
    now = getNodeState(OD_Data,nodeId);
    if(now == Unknown_state)//没有进行通信
    {
      masterRequestNodeState(OD_Data,nodeId);//发送Node Guarding Request命令查询节点状态
    }
    else if(now == Disconnected)//断开连接
    {
      masterRequestNodeState(OD_Data,nodeId);//发送Node Guarding Request命令查询节点状态
    }
    else if(now == Pre_operational)//通信恢复
    {
      setState(OD_Data, Initialisation);//Initialisation ->自动转入pre状态 ->转入op
      LOG_I("The line communication of the node is restored");
      return; //删除线程
    }
    else if(now == Initialisation)//节点断电后上电
    {
      setState(OD_Data, Initialisation);//Initialisation
      LOG_I("Restart the node after the node is shut down");
      return; //删除线程
    }
    else
    {
      masterRequestNodeState(OD_Data,nodeId);//发送Node Guarding Request命令查询节点状态
    }
      
    rt_thread_mdelay(CONSUMER_HEARTBEAT_TIME);//按照获取心跳间隔时间进行判断
  }  
}
/**
  * @brief  修复配置错误
  * @param  None.
  * @retval None.
  * @note   None.
*/
void master402_fix_config_err(UNS8 nodeId)
{
  if(++cfg.try_cnt <= 3)
  {
    setState(OD_Data, Stopped);
    LOG_I("Enabling the repair thread,Repair times = %d",cfg.try_cnt);
    rt_thread_t tid = rt_thread_create("fix_config_err", master402_fix_config_err_thread_entry,
                      (void *)(int)nodeId,//强制转换为16位数据与void*指针字节一致，以消除强制转换大小不匹配警告
                      2048, 12, 2);

    if(tid == RT_NULL)
    {
      LOG_E("canfestival fix_config_err thread start failed!");
    }
    else
    {
      rt_thread_startup(tid);
    }
  }
  else if(cfg.try_cnt == 4)
  {
     LOG_E("The number of repairs is too many. It is confirmed that it is not an occasional anomaly. It will not be repaired any more.");
  }
}