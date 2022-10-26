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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "master402_canopen.h"
/* Private includes ----------------------------------------------------------*/
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

#include <rtthread.h>
#include <finsh.h>

#include "canfestival.h"
#include "canopen_callback.h"
#include "timers_driver.h"
/* Private typedef -----------------------------------------------------------*/
typedef struct 
{
  uint8_t nodeID;
	uint8_t state;
	uint8_t try_cnt;
  uint8_t err_code;//配置参数错误代码 0xff,配置未发送,本地字典出错。 0x03,配置回复未响应，节点字典出错
	struct rt_semaphore finish_sem;
}servo_config_state;
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/*DS301心跳协议写入节点心跳格式
  NodeID  : 节点
  Interval: 心跳间隔*/
#define HEARTBEAT_FORMAT(NodeID,Interval) (((NodeID) << 16) | (Interval))
/*DS301协议 31位置1关闭PDO（RPDO/TPDO）传输
  NodeID  : 节点
  Default :  默认值*/
#define PDO_DISANBLE(NodeID,Default) ((1<<31) | (Default) | (NodeID))
/*DS301协议 31位置1使能PDO（RPDO/TPDO）传输
  NodeID  : 节点
  Default :  默认值*/
#define PDO_ENANBLE(Default,NodeID) ((Default) | (NodeID))
/*DS301协议 31位置1使能SYNC同步消息
  NodeID  : 节点*/
#define SYNC_ENANBLE(NodeID) ((1 << 30) | (NodeID))
/* Private variables ---------------------------------------------------------*/
CO_Data *OD_Data = &master402_Data;
s_BOARD agv_board  = {CANFESTIVAL_CAN_DEVICE_NAME,"1M"};//没用,兼容CANFESTIVAL
static servo_config_state servo_conf[MAX_NODE_COUNT - 2] = 
{
  {SERVO_NODEID_1,},
//  {SERVO_NODEID_2,},
};//配置状态
/* Private function prototypes -----------------------------------------------*/
static void config_servo_param(uint8_t nodeId, servo_config_state *conf);
/***********************初始化操作状态函数**************************************************/
/**
  * @brief  初始节点
  * @param  None
  * @retval None
  * @note   
*/
static void InitNodes(CO_Data* d, UNS32 id)
{
	setNodeId(OD_Data,CONTROLLER_NODEID);
	setState(OD_Data, Initialisation);
}
/**
  * @brief  初始化canopen
  * @param  None
  * @retval None
  * @note   
*/
static int canopen_init(void)
{
	OD_Data->heartbeatError = master402_heartbeatError;
	OD_Data->initialisation = master402_initialisation;
	OD_Data->preOperational = master402_preOperational;
	OD_Data->operational = master402_operational;
	OD_Data->stopped   = master402_stopped;
	OD_Data->post_sync = master402_post_sync;
	OD_Data->post_TPDO = master402_post_TPDO;
	OD_Data->storeODSubIndex = (storeODSubIndex_t)master402_storeODSubIndex;
	OD_Data->post_emcy = (post_emcy_t)master402_post_emcy;

	canOpen(&agv_board, OD_Data);
	initTimer();

	// Start timer thread
	StartTimerLoop(&InitNodes);
	
	return 0;
}
INIT_APP_EXPORT(canopen_init);
/**
  * @brief  None
  * @param  None
  * @retval None
  * @note   
*/
static void Exit(CO_Data* d, UNS32 id)
{

}
#ifdef RT_USING_MSH
/**
  * @brief  打印节点状态
  * @param  state：节点状态
  * @retval None.
  * @note   None.
*/
void printf_state(e_nodeState state)
{
  switch(state)
  {
    case Initialisation:
      rt_kprintf("nodeID state is Initialisation\n");
      break;
    case Stopped:
      rt_kprintf("nodeID state is Stopped\n");
      break;
    case Operational:
      rt_kprintf("nodeID state is Operational\n");
      break;
    case Pre_operational:
      rt_kprintf("nodeID state is Pre_operational\n");
      break;
    case Disconnected:
      rt_kprintf("nodeID state is Disconnected\n");
      break;   
    case Unknown_state:
      rt_kprintf("nodeID state is Unknown_state\n");
      break;
    default:
      rt_kprintf("nodeID state is %d\n",state);
      break;
  }
}
/**
  * @brief  MSH控制canopen主站nmt状态
  * @param  None
  * @retval None
  * @note   None
*/
static void cmd_canopen_nmt(uint8_t argc, char **argv) 
{
#define NMT_CMD_GET                     0
#define NMT_CMD_SLAVE_SET               1
#define NMT_CMD_PRE                     3
  size_t i = 0;

  const char* help_info[] =
    {
        [NMT_CMD_GET]            = "canopen_nmt get [nodeID]                - Get nodeID state.",
        [NMT_CMD_SLAVE_SET]      = "canopen_nmt s   [operational] <nodeID>  - Slvae  NMT set start/stop/pre/rn/rc.",
        [NMT_CMD_SLAVE_SET+1]    = "                                        - rn:Reset_Node rc:Reset_Comunication.",
        [NMT_CMD_PRE]            = "canopen_nmt m   [operational] <nodeID>  - Master NMT set start/stop/pre.",
    };

    if (argc < 2)
    {
        rt_kprintf("Usage:\n");
        for (i = 0; i < sizeof(help_info) / sizeof(char*); i++)
        {
            rt_kprintf("%s\n", help_info[i]);
        }
        rt_kprintf("\n");
    }
    else
    {
        const char *operator = argv[1];
        uint32_t addr, size;
        if (!strcmp(operator, "get"))
        {
          if(argc <= 2) 
          {
              rt_kprintf("Usage:canoepn get [nodeID]  - Get nodeID state\n");
              return;
          }
          uint8_t nodeid = atoi(argv[2]);
          if(nodeid == CONTROLLER_NODEID)
          {
            rt_kprintf("Master NodeID:%2X  ",CONTROLLER_NODEID);
            printf_state(getState(OD_Data));            
          }
          else
          {
            rt_kprintf("Slave  NodeID:%2X  ",nodeid);
            printf_state(getNodeState(OD_Data,nodeid));
          }
        }
        else if (!strcmp(operator, "s"))
        {
          uint8_t nodeID = 2;
          UNS8    cs = 0X00; 
          if(argc <= 2) 
          {
              rt_kprintf("Usage:canopen_nmt s   [operational] <nodeID>  - Slvae  NMT set start/stop/pre/rn/rc.\n");
              rt_kprintf("                                              - rn:Reset_Node rc:Reset_Comunication.\n");
              return;
          }
          if(argc > 2)
          {
            const char *operator = argv[2];
            if (!strcmp(operator, "start"))
            {
              cs = NMT_Start_Node;
            }
            else if(!strcmp(operator, "stop"))
            {
              cs = NMT_Stop_Node;
            } 
            else if(!strcmp(operator, "pre"))
            {
              cs = NMT_Enter_PreOperational;
            } 
            else if(!strcmp(operator, "rn"))
            {
              cs = NMT_Reset_Node;
            } 
            else if(!strcmp(operator, "rc"))
            {
              cs = NMT_Reset_Comunication;
            }
            else
            {
              rt_kprintf("Usage:canopen_nmt s   [operational] <nodeID>  - Slvae  NMT set start/stop/pre.\n");
              return;
            }
          }
          if(argc > 3)
          {
             nodeID = atoi(argv[4]);
          }
          masterSendNMTstateChange(OD_Data,nodeID,cs);
        }
        else if (!strcmp(operator, "m"))
        {
          const char *operator = argv[2];
          if(argc <= 2) 
          {
              rt_kprintf("Usage:canopen_nmt m   [operational] <nodeID>  - Master NMT set start/stop/pre.\n");
              return;
          }
          if(argc > 2)
          {
            e_nodeState newState;
            if (!strcmp(operator, "start"))
            {
              newState = Operational;
            }
            else if (!strcmp(operator, "stop"))
            {
              newState = Stopped;
            }
            else if (!strcmp(operator, "pre"))
            {
              newState = Pre_operational;
            }
            else
            {
                rt_kprintf("Usage:canopen_nmt m   [operational] <nodeID>  - Master NMT set start/stop/pre/rn/rc.\n");
                rt_kprintf("                                              - rn:Reset_Node rc:Reset_Comunication.\n");
                return;
            }
            setState(OD_Data, newState);
          }
        }
        else
        {
          rt_kprintf("Usage:\n");
          for (i = 0; i < sizeof(help_info) / sizeof(char*); i++)
          {
              rt_kprintf("%s\n", help_info[i]);
          }
          rt_kprintf("\n");
        }
    }
}
MSH_CMD_EXPORT_ALIAS(cmd_canopen_nmt,canopen_nmt,canoepn nmt cmd.);
#endif
/***********************预操作状态函数**************************************************/
/**
  * @brief  配置参数完成回调
  * @param  None
  * @retval None
  * @note   None
*/
static void config_servo_param_cb(CO_Data* d, UNS8 nodeId)
{
	UNS32 abortCode;
	UNS8 res;
	servo_config_state *conf;

	conf = &servo_conf[nodeId - 2];
	res = getWriteResultNetworkDict(OD_Data, nodeId, &abortCode);
	closeSDOtransfer(OD_Data, nodeId, SDO_CLIENT);
	if(res != SDO_FINISHED)
	{
		conf->try_cnt++;
		if(conf->try_cnt < 3)
		{
      LOG_W(" write SDO failed!  nodeId = %d, abortCode = 0x%08X,step = %d", nodeId, abortCode,conf->state - 1);
			config_servo_param(nodeId, conf);
		}
		else
		{
			rt_sem_release(&(conf->finish_sem));
			conf->state = 0;
			conf->try_cnt = 0;
      conf->err_code = 0x03;
			LOG_E("SDO config try count > 3, config failed!,error = %d",conf->err_code);
		}
	}
	else
	{
    conf->err_code = 0;
		conf->state++;
		conf->try_cnt = 0;
		config_servo_param(nodeId, conf);
	}
}
/**
  * @brief  配置伺服参数
  * @param  None
  * @retval None
  * @note   互斥量等待配置完成
*/
static void config_servo(uint8_t nodeId)
{
	servo_conf[nodeId - 2].state = 0;
	servo_conf[nodeId - 2].try_cnt = 0;
	rt_sem_init(&(servo_conf[nodeId - 2].finish_sem), "servocnf", 0, RT_IPC_FLAG_FIFO);

	EnterMutex();
	config_servo_param(nodeId, &servo_conf[nodeId - 2]);
	LeaveMutex();
	rt_sem_take(&(servo_conf[nodeId - 2].finish_sem), RT_WAITING_FOREVER);
	rt_sem_detach(&(servo_conf[nodeId - 2].finish_sem));
}
/**
  * @brief  None
  * @param  None
  * @retval None
  * @note   None
*/
static void config_single_servo(void *parameter)
{
	uint32_t nodeId;
	nodeId = (uint32_t)parameter;
	config_servo(nodeId);
	masterSendNMTstateChange(OD_Data, nodeId, NMT_Start_Node);
}
/**
  * @brief  从机心跳上线处理函数
  * @param  None
  * @retval None
  * @note   创建config_single_servo线程
*/
static void slaveBootupHdl(CO_Data* d, UNS8 nodeId)
{
	rt_thread_t tid;

	tid = rt_thread_create("co_cfg", config_single_servo, (void *)(int)nodeId, 1024, 12 + nodeId, 2);
	if(tid == RT_NULL)
	{
		LOG_E("canopen config thread start failed!");
	}
	else
	{
		rt_thread_startup(tid);
	}
}
/**
  * @brief  写入本地字典参数，并切换nmt启动
  * @param  None
  * @retval None
  * @note   nmt状态切换为operation状态处理线程，运行完删除线程
*/
void canopen_start_thread_entry(void *parameter)
{
  assert(parameter);
	UNS32 sync_id, size;
	UNS8 data_type, sub_cnt;
	UNS32 consumer_heartbeat_time;
  UNS8 nodeId = 0;
  /*写入节点字典*/
  for (UNS8 i = 0; i < MAX_NODE_COUNT - 2; i++)
  {
    nodeId = servo_conf[i].nodeID;
    Write_SLAVE_control_word(nodeId,0x80);//初始化进行错误重置
    
    rt_thread_delay(200);
    config_servo(nodeId);
    if(servo_conf[i].err_code != 0X00)//因配置错误导致的退出
    {
      LOG_E("Failed to configure the dictionary for node %d",nodeId);
      if(servo_conf[i].err_code == 0XFF)
      {
        LOG_E("The configuration was not sent because the local dictionary failed");
      }
      else if(servo_conf[i].err_code == 0X03)
      {
        LOG_E("The configuration reply did not respond, and the node dictionary failed");
      }
      LOG_W("Waiting for the repair to complete, CAN communication is currently unavailable");
      master402_fix_config_err(nodeId);
      return; //退出线程
    }
  }
  LOG_I("Node configuration Complete");
  /*写入本地字典*/
	OD_Data->post_SlaveBootup = slaveBootupHdl;
  /**写入主机消费者/接收端判断心跳超时时间  DS301定义**/
  /**有格式定义，字典工具没有支持，需要自己写入**/
	consumer_heartbeat_time = HEARTBEAT_FORMAT(nodeId,CONSUMER_HEARTBEAT_TIME);//写入节点2的心跳时间
	size = 4;
	writeLocalDict(OD_Data, 0x1016, 1, &consumer_heartbeat_time, &size, 0);
	consumer_heartbeat_time = HEARTBEAT_FORMAT(nodeId + 1,CONSUMER_HEARTBEAT_TIME);//写入节点3的心跳时间,没有节点即跳过
	writeLocalDict(OD_Data, 0x1016, 2, &consumer_heartbeat_time, &size, 0);
	sub_cnt = 2;
	size = 1;
	writeLocalDict(OD_Data, 0x1016, 0, &sub_cnt, &size, 0);
  /**写入主机生成者/发送端心跳发送时间  DS301定义**/
  UNS16 producer_heartbeat_time;
	producer_heartbeat_time = PRODUCER_HEARTBEAT_TIME;
	size = 2;
	writeLocalDict(OD_Data, 0x1017, 0, &producer_heartbeat_time, &size, 0);
  /**有格式定义，字典工具没有支持，需要自己写入**/
	data_type = uint32;
	setState(OD_Data, Operational);
	masterSendNMTstateChange(OD_Data, nodeId, NMT_Start_Node);
	size = 4;
	readLocalDict(OD_Data, 0x1005, 0, &sync_id, &size, &data_type, 0);
	sync_id = SYNC_ENANBLE(sync_id);//DS301 30位置1 ，启用CANopen设备生成同步消息
	writeLocalDict(OD_Data, 0x1005, 0, &sync_id, &size, 0);
}
/******************************写入节点字典操作函数**********************************/
/**
  * @brief  发送完成回调
  * @param  d:字典结构体
  * @param  nodeId：节点ID
  * @retval None
  * @note   None
*/
static void writeNetworkDictSyncCb(CO_Data* d, UNS8 nodeId) 
{
    servo_config_state *conf = &servo_conf[nodeId - 2];
    rt_sem_release(&conf->finish_sem);
}
/**
  * @brief  异步写入节点字典
  * @param  d:字典结构体
  * @param  nodeId：节点ID
  * @retval None
  * @note   None
*/
static bool writeNetworkDictSync (CO_Data* d, UNS8 nodeId, UNS16 index,
        UNS8 subIndex, UNS32 count, UNS8 dataType, void *data, UNS8 useBlockMode) 
{

    if(nodeId < 1 || nodeId > MAX_NODE_COUNT - 1) 
    {
      LOG_W("invalid nodeId:%d, should between 2 and %d",nodeId,MAX_NODE_COUNT-1);
      return false;
    }

    int try_cnt = 3;
    servo_config_state* conf = &servo_conf[nodeId - 2];
    rt_sem_init(&conf->finish_sem, "servocnf", 0, RT_IPC_FLAG_FIFO);
    
    while(try_cnt--) 
    {
        if(writeNetworkDictCallBack(d, nodeId, index, subIndex,
                count, dataType, data, writeNetworkDictSyncCb, useBlockMode) != 0)
        {
            LOG_W("write SDO failed!  nodeId = %d, index: %d, subIndex: %d", nodeId, index, subIndex);
            closeSDOtransfer(OD_Data, nodeId, SDO_CLIENT);
            continue;
        }

        if(rt_sem_take(&conf->finish_sem, SDO_REPLY_TIMEOUT) != RT_EOK) 
        {
            LOG_W("write SDO timeout!  nodeId = %d, index: %d, subIndex: %d", nodeId, index, subIndex);
            closeSDOtransfer(OD_Data, nodeId, SDO_CLIENT);
            continue;
        }

        UNS32 abortCode = 0;
        UNS8 res = getWriteResultNetworkDict(OD_Data, nodeId, &abortCode);
        closeSDOtransfer(OD_Data, nodeId, SDO_CLIENT);

        if(res != SDO_FINISHED)
        {
            LOG_W("get SDO write result failed!  nodeId = %d, index: 0x%x, subIndex: %d, abortCode = 0x%08X", nodeId, index, subIndex, abortCode);
            continue;
        }
        rt_sem_detach(&conf->finish_sem);
        return true;
    }
    rt_sem_detach(&conf->finish_sem);
    return false;
}
/**
  * @brief  读取本地字典参数并写入节点字典
  * @param  d:字典结构体
  * @param  nodeId：节点ID
  * @param  index:索引
  * @param  subIndex:子索引
  * @retval None
  * @note   None
*/
static UNS8 Read_local_Send_Node(CO_Data* d,UNS8 nodeId,UNS16 index,UNS8  subIndex)
{
  UNS32  size = SDO_MAX_LENGTH_TRANSFER;
  UNS32 pdo_map_val = 0,errorCode = 0; 
  UNS8  dataType = 0;

  errorCode = readLocalDict(d,index,subIndex,(void *)&pdo_map_val,&size,&dataType,0);
  if(errorCode != OD_SUCCESSFUL)
  {
    LOG_E("index:0X%X,subIndex:0X%X,read Local Dict false,SDO abort code is 0X%X",index,subIndex,errorCode);
    return 0xFF;
  }
  errorCode = writeNetworkDictSync(d,nodeId,index,subIndex,size,dataType,&pdo_map_val,0);
  if(errorCode != true)
  {
    LOG_E("index:0X%X,subIndex:0X%X,write slave Dict false",index,subIndex);
    return 0xFF;
  }
  return 0x00;
}
/**
  * @brief  读取本地字典值并发送至从机节点
  * @param  None.
  * @retval None.
  * @note   0XFF:错误，0X00:成功.
*/
static UNS8 local_od_send(UNS16 index,UNS8 subIndex,uint8_t nodeId)
{
  UNS32 pdo_map_val;
  UNS32  size = SDO_MAX_LENGTH_TRANSFER;
  UNS8  dataType;
  UNS32 errorCode;
  UNS16  slave_Index;
  
  if(0X1600 <= index && index <= 0X17FF)//从机 Receive PDO Parameters
  {
    //在主站Transmit PDO Parameters寻找对应参数
    slave_Index = index + 0X400;
  }
  else if(0X1A00 <= index && index <= 0X1BFF)//Transmit PDO Parameters
  {
    //在主站Receive PDO Parameters寻找对应参数
    slave_Index = index - 0X400;
  }
  errorCode = readLocalDict(OD_Data,slave_Index,subIndex,(void *)&pdo_map_val,&size,&dataType,0);
  if(errorCode != OD_SUCCESSFUL)
  {
    LOG_E("index:0X%X,subIndex:0X%X,read Local Dict false,SDO abort code is 0X%X",index,subIndex,errorCode);
    return 0XFF;
  }
  else
  {
    return writeNetworkDictCallBack(OD_Data,nodeId,index,subIndex,size,dataType,&pdo_map_val,config_servo_param_cb,0);
  }
}
/******************************SDO设置参数操作**********************************/
/**
  * @brief  写入从机Modes_of_operation设置
  * @param  nodeId:节点ID
  * @param  mode:Modes_of_operation 
  * @retval 成功返回0X00,失败返回0XFF
  * @note   None.
*/
UNS8 Write_SLAVE_Modes_of_operation(UNS8 nodeId,INTEGER8 mode)
{
  UNS8 errcode = 0;
  if(0 <= Modes_of_operation && Modes_of_operation <= 10)
  {
    //参数写入本地字典
    Modes_of_operation = mode;
  }
  else
  {
    LOG_E("The Modes_of_operation is out of range. Select a value from 0 to 10");
    return 0XFF;
  }

  //参数发送节点字典
  if(Read_local_Send_Node(OD_Data,nodeId,0x6060,0x00) == 0XFF)
    return 0XFF;

  return 0X00;
}
/**
  * @brief  写入从机control_word设置
  * @param  nodeId:节点ID
  * @param  value:Controlword 
  * @retval 成功返回0X00,失败返回0XFF
  * @note   None.
*/
UNS8 Write_SLAVE_control_word(UNS8 nodeId,UNS16 value)
{
  UNS8 errcode = 0;
  //参数写入本地字典
  Controlword = value;

  //参数发送节点字典
  if(Read_local_Send_Node(OD_Data,nodeId,0x6040,0x00) == 0XFF)
    return 0XFF;

  return 0X00;
}
/**
  * @brief  写入从机位置规划模式的速度设置
  * @param  nodeId:节点ID
  * @param  speed:运动速度 单位:RPM
  * @retval 成功返回0X00,失败返回0XFF
  * @note   Profile_velocity 单位PUU/s 设定范围 UNSIGNED32 默认值 10000
*/
UNS8 Write_SLAVE_profile_position_speed_set(UNS8 nodeId,UNS32 speed)
{
  UNS8 errcode = 0;
  //参数写入本地字典
  if(0 <= speed && speed <= 3000)
    Profile_velocity = speed * ELECTRONIC_GEAR_RATIO_NUMERATOR / 60;//
  else
  {
    LOG_E("The Profile_velocity is out of range. Select a value from 0 to 3000");
    return 0XFF;
  }

  //参数发送节点字典
  if(Read_local_Send_Node(OD_Data,nodeId,0X6081,0x00) == 0XFF)
    return 0XFF;

  return 0X00;
}
/**
  * @brief  写入从机0X60C2插补周期参数
  * @param  nodeId:节点ID
  * @retval 成功返回0X00,失败返回0XFF
  * @note   设定需与通讯周期 OD 1006h相同
  * 插补周期时间 = 60C2H sub1 × 10^(60C2H sub2)
  * 范例：若需要设定插补周期时间为 10 ms，需将 OD 60C2h sub1 = 1，OD 60C2h sub2 = -4
    插补周期时间 = 1 × 10^-2 = 0.01s = 10ms
*/
UNS8 Write_SLAVE_Interpolation_time_period(UNS8 nodeId)
{
  UNS32  size = SDO_MAX_LENGTH_TRANSFER;
  UNS32 pdo_map_val,errorCode; 
  UNS8  dataType;
  UNS16 index = 0X1006;//通讯周期值
  UNS8 subIndex = 0;//子索引0

  errorCode = readLocalDict(OD_Data,index,subIndex,(void *)&pdo_map_val,&size,&dataType,0);//读取0X1006通讯周期值
  if(errorCode != OD_SUCCESSFUL)
  {
    LOG_E("index:0X%X,subIndex:0X%X,read Local Dict false,SDO abort code is 0X%X",index,subIndex,errorCode);
    return 0xFF;
  }
  extern int Bit_Int_2(long n);
  Interpolation_time_period_Interpolation_time_index = (Bit_Int_2(pdo_map_val) - 1) - 6;//十的次方数 = 10^(n-6)
  Interpolation_time_period_Interpolation_time_units = pdo_map_val / pow(10,(Bit_Int_2(pdo_map_val) - 1));//插补周期时间常数
  
  if(Read_local_Send_Node(OD_Data,nodeId,0X60C2,0x01) == 0XFF)
    return 0XFF;
  if(Read_local_Send_Node(OD_Data,nodeId,0X60C2,0x02) == 0XFF)
    return 0XFF;

  return 0x00;
}
/**
  * @brief  写入从机原点复归模式的相关设置
  * @param  nodeId:节点ID
  * @param  offest:原点偏移值 单位:PUU [注意:只是把原点偏移值点当为0坐标点，并不会运动到0坐标点处]
  * @param  method:回原方式   范围:0 ~ 35
  * @param  switch_speed:寻找原点开关速度 设定范围 0.1 ~ 200 默认值 10  单位rpm 精度:小数点后一位
  * @param  zero_speed:寻找 Z脉冲速度     设定范围 0.1 ~ 50  默认值 2   单位rpm 精度:小数点后一位
  * @retval 成功返回0X00,失败返回0XFF
  * @note   Home Position为执行原点复归时所找到的原点参考点，如原点 Sensor、Z脉冲等。
            当原点参考点找到后，从该点所偏移的位置即为用户定义的原点。

            共 35种原点复归模式供用户设定。第 1 ~ 16种回原点模式会找寻 Z脉冲； 
            而第 17 ~ 34种则是不找寻 Z脉冲；第 35种则是定义当前位置为原点。

            switch_speed和zero_speed写入单位为0.1rpm

            设定原点复归加/减速时间 OD 609Ah 默认值 100 单位 ms 
            此对象所设定的时间斜率为加速 (0 rpm加速到 3000 rpm) 及减速 (3000 rpm减速到0 rpm) 所需要的时间。此对象仅适用于 Homing Mode (原点复归模式)。 
*/
UNS8 Write_SLAVE_Homing_set(UNS8 nodeId,UNS32 offset,UNS8 method,float switch_speed,float zero_speed)
{
  UNS8 errcode = 0;
  //参数写入本地字典
  Home_offset = offset;
  if(0 <= method && method <= 35)
    Homing_method = method;
  else
  {
    LOG_E("The method is out of range. Select a value from 0 to 35");
    return 0XFF;
  }
  if(0.1 <= switch_speed && switch_speed <= 200)
    Homing_speeds_Speed_for_switch_search = (uint32_t)(switch_speed * 10);
  else
  {
    LOG_E("The switch_speed is out of range. Select a value from 0.1 to 200");
    return 0XFF;
  }
  if(0.1 <= zero_speed && zero_speed <= 50)
    Homing_speeds_Speed_for_zero_search = (uint32_t)(zero_speed * 10);
  else
  {
    LOG_E("The zero_speed is out of range. Select a value from 0.1 to 50");
    return 0XFF;
  }

  //参数发送节点字典
  if(Read_local_Send_Node(OD_Data,nodeId,0X607C,0x00) == 0XFF)
    return 0XFF;
  if(Read_local_Send_Node(OD_Data,nodeId,0X6098,0x00) == 0XFF)
    return 0XFF;
  if(Read_local_Send_Node(OD_Data,nodeId,0X6099,0x01) == 0XFF)
    return 0XFF;
  if(Read_local_Send_Node(OD_Data,nodeId,0X6099,0x02) == 0XFF)
    return 0XFF;

  return 0X00;
}
/******************************TPDO1 设置参数操作
                               数据长度被限制为 1~8 字节*********************************/
/**
  * 分析仪显示PDO1发送 , 帧ID0X182 DLC:8
  * 字典工具：0x1400 Receive PDO 1 Parameter:COB ID used by PDO:0x00000182
  * 字典工具：0x1600 Receive PDO 1 Mapping.填入需要发送数据
  * 为从机发送通道TPDO1,即主机接收通道RPDO1
  * PDO DATA:0x50 06 4E C3 00 00 00 00  小端模式.
  * SUBINDEX2:32位,占用4个字节.即00 00 C3 4E = 49998 Position actual value (Ox6064)
  * SUBINDEX2:32位,占用4个字节.即00 00 C3 4E = 49998 Velocity_actual_value (Ox606C)
*/
static UNS8 DIS_SLAVE_TPDO1(uint8_t nodeId)
{
// disable Slave's TPDO，关闭从机TPDO1发送用来写入参数
  UNS32 TPDO_COBId = PDO_DISANBLE(0x00000180,nodeId);//0x80000180 + nodeId;
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1800, 1, 
    4, uint32, &TPDO_COBId, config_servo_param_cb, 0);
}
static UNS8 Write_SLAVE_TPDO1_Type(uint8_t nodeId)
{
  UNS8 trans_type = PDO_TRANSMISSION_TYPE;//写入类型
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1800, 2, 
    1, uint8, &trans_type, config_servo_param_cb, 0);
}
static UNS8 Clear_SLAVE_TPDO1_Cnt(uint8_t nodeId)
{
  UNS8 pdo_map_cnt = 0;//清除从机遗留索引数
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1A00, 0, 
    1, uint8, &pdo_map_cnt, config_servo_param_cb, 0);
}
static UNS8 Write_SLAVE_TPDO1_Sub1(uint8_t nodeId)
{
  return local_od_send(0X1A00,1,nodeId);//写入从机PDO1需要发送映射，即主站需要PDO1接收映射
}
static UNS8 Write_SLAVE_TPDO1_Sub2(uint8_t nodeId)
{
  return local_od_send(0X1A00,2,nodeId);//写入从机PDO1需要发送映射，即主站需要PDO1接收映射
}
static UNS8 Write_SLAVE_TPDO1_Sub0(uint8_t nodeId)
{
  return local_od_send(0X1A00,0,nodeId);//重新写入正确索引数
}
static UNS8 EN_SLAVE_TPDO1(uint8_t nodeId)
{
//enable Slave's TPDO1 使能从机发送PDO1
  UNS32 TPDO_COBId = PDO_ENANBLE(0x00000180,nodeId);
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1800, 1, 
    4, uint32, &TPDO_COBId, config_servo_param_cb, 0);
}
/******************************TPDO2 设置参数操作
                               数据长度被限制为 1~8 字节*********************************/
/**
  * 分析仪显示PDO2发送 , 帧ID0X282 DLC:4
  * 字典工具：0x1401 Receive PDO 2 Parameter:COB ID used by PDO:0x00000282
  * 字典工具：0x1601 Receive PDO 2 Mapping.填入需要发送数据
  * 为从机发送通道TPDO2,即主机接收通道RPDO2
  * PDO DATA:0x37 16   小端模式.
  * SUBINDEX1:16位,占用2个字节.即0X1637 Statusword (Ox6041) 
*/
static UNS8 DIS_SLAVE_TPDO2(uint8_t nodeId)
{
 //disable Slave's TPDO 关闭从机TPDO2发送用来写入参数
  UNS32 TPDO_COBId = PDO_DISANBLE(0x00000280,nodeId);//0x80000280 + nodeId;
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1801, 1, 
    4, uint32, &TPDO_COBId, config_servo_param_cb, 0);
}
static UNS8 Write_SLAVE_TPDO2_Type(uint8_t nodeId)
{
  UNS8 trans_type = PDO_TRANSMISSION_TYPE;//写入类型
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1801, 2, 
    1, uint8, &trans_type, config_servo_param_cb, 0);
}
static UNS8 Clear_SLAVE_TPDO2_Cnt(uint8_t nodeId)
{
  UNS8 pdo_map_cnt = 0;//清除从机遗留索引数
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1A01, 0, 
    1, uint8, &pdo_map_cnt, config_servo_param_cb, 0);
}
static UNS8 Write_SLAVE_TPDO2_Sub1(uint8_t nodeId)
{
  return local_od_send(0X1A01,1,nodeId);//写入从机PDO1需要发送映射，即主站需要PDO1接收映射
}
static UNS8 Write_SLAVE_TPDO2_Sub0(uint8_t nodeId)
{
  return local_od_send(0X1A01,0,nodeId);//重新写入正确索引数
}
static UNS8 EN_SLAVE_TPDO2(uint8_t nodeId)
{
//enable Slave's TPDO1 使能从机发送PDO1
  UNS32 TPDO_COBId = PDO_ENANBLE(0x00000280,nodeId);
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1801, 1, 
    4, uint32, &TPDO_COBId, config_servo_param_cb, 0);
}
/******************************RPDO1 设置参数操作
                               数据长度被限制为 1~8 字节*********************************/
/**
  * 分析仪显示PDO1接收 , 帧ID0X202 DLC:8
  * 字典工具：0x1800 Transmit PDO 1 Parameter:COB ID used by PDO:0x00000202
  * 字典工具：0x1A00 Transmit PDO 1 Mapping.填入需要发送数据
  * 为从机接收通道RPDO1,即主机发送通道TPDO1
  * PDO DATA:0x00 00 01 00 00 00 00   小端模式.
  * SUBINDEX1:32位,占用4个字节.即00 00 C3 50 = 50000  Target position (0x607A)
  * SUBINDEX2:32位,占用4个字节.即00 01 86 A0 = 100000 Target_velocity (0x60FF)
*/
static UNS8 DIS_SLAVE_RPDO1(uint8_t nodeId)
{
// disable Slave's RPDO
  UNS32 RPDO_COBId = PDO_DISANBLE(0x80000200,nodeId);;
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1400, 1, 
    4, uint32, &RPDO_COBId, config_servo_param_cb, 0);
}
static UNS8 Write_SLAVE_RPDO1_Type(uint8_t nodeId)
{
  UNS8 trans_type = PDO_TRANSMISSION_TYPE;
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1400, 2, 
    1, uint8, &trans_type, config_servo_param_cb, 0);
}
static UNS8 Clear_SLAVE_RPDO1_Cnt(uint8_t nodeId)
{
  UNS8 pdo_map_cnt = 0;//清除索引
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1600, 0, 
    1, uint8, &pdo_map_cnt, config_servo_param_cb, 0);
}
static UNS8 Write_SLAVE_RPDO1_Sub1(uint8_t nodeId)
{
  return local_od_send(0X1600,1,nodeId);//写入从机PDO1需要接收映射，即主站需要PDO1发送映射
}
static UNS8 Write_SLAVE_RPDO1_Sub2(uint8_t nodeId)
{
  return local_od_send(0X1600,2,nodeId);//写入从机PDO1需要接收映射，即主站需要PDO1发送映射
}
static UNS8 Write_SLAVE_RPDO1_Sub0(uint8_t nodeId)
{
  return local_od_send(0X1600,0,nodeId);//重新写入正确索引数
}
static UNS8 EN_SLAVE_RPDO1(uint8_t nodeId)
{
// enable Slave's RPDO
  UNS32 RPDO_COBId = PDO_ENANBLE(0x00000200,nodeId);
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1400, 1, 
    4, uint32, &RPDO_COBId, config_servo_param_cb, 0);
}
/******************************RPDO2 设置参数操作
                               数据长度被限制为 1~8 字节*********************************/
/**
  * 分析仪显示PDO2接收 , 帧ID0X302 DLC:4
  * 字典工具：0x1801 Transmit PDO 2 Parameter:COB ID used by PDO:0x00000302
  * 字典工具：0x1A01 Transmit PDO 2 Mapping.填入需要发送数据
  * 为从机接收通道RPDO2,即主机发送通道TPDO2
  * PDO DATA:0x50 C3 00 00  小端模式.
  * SUBINDEX1:32位,占用4个字节.即00 00 C3 50 = 50000  POS_CMD (0x60C1)
*/
static UNS8 DIS_SLAVE_RPDO2(uint8_t nodeId)
{
// disable Slave's RPDO
  UNS32 RPDO_COBId = PDO_DISANBLE(0x80000300,nodeId);;
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1401, 1, 
    4, uint32, &RPDO_COBId, config_servo_param_cb, 0);
}
static UNS8 Write_SLAVE_RPDO2_Type(uint8_t nodeId)
{
  UNS8 trans_type = PDO_TRANSMISSION_TYPE;
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1401, 2, 
    1, uint8, &trans_type, config_servo_param_cb, 0);
}
static UNS8 Clear_SLAVE_RPDO2_Cnt(uint8_t nodeId)
{
  UNS8 pdo_map_cnt = 0;//清除索引
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1601, 0, 
    1, uint8, &pdo_map_cnt, config_servo_param_cb, 0);
}
static UNS8 Write_SLAVE_RPDO2_Sub1(uint8_t nodeId)
{
  return local_od_send(0X1601,1,nodeId);//写入从机PDO1需要接收映射，即主站需要PDO1发送映射
}
static UNS8 Write_SLAVE_RPDO2_Sub0(uint8_t nodeId)
{
  return local_od_send(0X1601,0,nodeId);//重新写入正确索引数
}
static UNS8 EN_SLAVE_RPDO2(uint8_t nodeId)
{
// enable Slave's RPDO
  UNS32 TPDO_COBId = PDO_ENANBLE(0x00000300,nodeId);
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1401, 1, 
    4, uint32, &TPDO_COBId, config_servo_param_cb, 0);
}
/******************************心跳时间 设置参数操作*********************************/
static UNS8 Write_SLAVE_P_heartbeat(uint8_t nodeId)
{
  UNS16 producer_heartbeat_time = PRODUCER_HEARTBEAT_TIME;
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1017, 0, 
    2, uint16, &producer_heartbeat_time, config_servo_param_cb, 0);
}
static UNS8 Write_SLAVE_C_heartbeat(uint8_t nodeId)
{
  //需要本地字典写入生产者心跳时间，不然驱动器报错
  UNS32 consumer_heartbeat_time = HEARTBEAT_FORMAT(CONTROLLER_NODEID,CONSUMER_HEARTBEAT_TIME);//写入节点1的心跳时间
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1016, 1, 
    4, uint16, &consumer_heartbeat_time, config_servo_param_cb, 0);
}
/*****************************其他参数操作*********************************/
/**
  * @brief  开启从机S型加减速功能
  * @param  None
  * @retval None
  * @note   None
*/
static UNS8 Write_SLAVE_S_Move(uint8_t nodeId)
{
  extern subindex master402_Index2124;
  S_move = TRUE;
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x2124, 0, 
    master402_Index2124.size, master402_Index2124.bDataType, &S_move, config_servo_param_cb, 0);
}
/*****************************结束设置参数操作*********************************/
static UNS8 MotorCFG_Done(uint8_t nodeId)
{
	servo_config_state *conf;
	conf = &servo_conf[nodeId - 2];
  rt_sem_release(&(conf->finish_sem));
  return 0;
}
/**
  * @brief  函数指针数组.
  * @param  None.
  * @retval None.
  * @note   
注意函数的顺序和数组越界问题
https://blog.csdn.net/feimeng116/article/details/107515317
*/
static UNS8 (*MotorCFG_Operation[])(uint8_t nodeId) = 
{ 
  //TPDO1通道操作
  DIS_SLAVE_TPDO1,
  Write_SLAVE_TPDO1_Type,
  Clear_SLAVE_TPDO1_Cnt,
  Write_SLAVE_TPDO1_Sub1,
  Write_SLAVE_TPDO1_Sub2,
  Write_SLAVE_TPDO1_Sub0,
  EN_SLAVE_TPDO1,
  //TPDO2通道操作
  DIS_SLAVE_TPDO2,
  Write_SLAVE_TPDO2_Type,
  Clear_SLAVE_TPDO2_Cnt,
  Write_SLAVE_TPDO2_Sub1,
//  Write_SLAVE_TPDO2_Sub2,
  Write_SLAVE_TPDO2_Sub0,
  EN_SLAVE_TPDO2,
  //RPDO1通道操作
  DIS_SLAVE_RPDO1,
  Write_SLAVE_RPDO1_Type,
  Clear_SLAVE_RPDO1_Cnt,
  Write_SLAVE_RPDO1_Sub1,
  Write_SLAVE_RPDO1_Sub2,
  Write_SLAVE_RPDO1_Sub0,
  EN_SLAVE_RPDO1,
  //RPDO2通道操作
  DIS_SLAVE_RPDO2,
  Write_SLAVE_RPDO2_Type,
  Clear_SLAVE_RPDO2_Cnt,
  Write_SLAVE_RPDO2_Sub1,
  Write_SLAVE_RPDO2_Sub0,
  EN_SLAVE_RPDO2,
  //写入心跳
  Write_SLAVE_P_heartbeat,
  Write_SLAVE_C_heartbeat,
  //其他配置
  Write_SLAVE_S_Move,
  //结束配置
  MotorCFG_Done,
};
/**
  * @brief  配置参数
  * @param  None
  * @retval None
  * @note   None
*/
static void config_servo_param(uint8_t nodeId, servo_config_state *conf)
{
  //判断数据越界 指针无法判断指向内容大小
  //https://bbs.csdn.net/topics/80323809
  if(conf->state < sizeof(MotorCFG_Operation) / sizeof(MotorCFG_Operation[0]))
  {
    conf->err_code = MotorCFG_Operation[conf->state](nodeId);
  }
  if(conf->err_code != 0)//配置参数错误
  {
    rt_sem_release(&(conf->finish_sem));
    LOG_W("Step %d configuration parameters are incorrect",conf->state - 1);
    conf->state = 0;
  }
  else
  {
    conf->err_code = 0;
  }
}
/**
  * @brief  配置参数
  * @param  None
  * @retval None
  * @note   None
*/
/*static void config_servo_param(uint8_t nodeId, struct servo_config_state *conf)
{
	switch(conf->state)
	{
	case 0:
		{ // disable Slave's TPDO，关闭从机TPDO1发送用来写入参数
			UNS32 TPDO_COBId = PDO_DISANBLE(0x00000180,nodeId);//0x80000180 + nodeId;
			writeNetworkDictCallBack(OD_Data, nodeId, 0x1800, 1, 
				4, uint32, &TPDO_COBId, config_servo_param_cb, 0);
		}
		break;
	case 1:
		{
			UNS8 trans_type = PDO_TRANSMISSION_TYPE;//写入类型
			writeNetworkDictCallBack(OD_Data, nodeId, 0x1800, 2, 
				1, uint8, &trans_type, config_servo_param_cb, 0);
		}
		break;
	case 2:
		{
			UNS8 pdo_map_cnt = 0;//清除从机遗留索引数
			writeNetworkDictCallBack(OD_Data, nodeId, 0x1A00, 0, 
				1, uint8, &pdo_map_cnt, config_servo_param_cb, 0);
		}
		break;
	case 3:
    local_od_send(0X1A00,1,nodeId);//写入从机PDO1需要发送映射，即主站需要PDO1接收映射
		break;
	case 4:
		local_od_send(0X1A00,2,nodeId);//写入从机PDO1需要发送映射，即主站需要PDO1接收映射
		break;
	case 5:
    local_od_send(0X1A00,0,nodeId);//重新写入正确索引数
		break;
	case 6:
		{ // enable Slave's TPDO1 使能从机发送PDO1
			UNS32 TPDO_COBId = PDO_ENANBLE(0x00000180,nodeId);
			writeNetworkDictCallBack(OD_Data, nodeId, 0x1800, 1, 
				4, uint32, &TPDO_COBId, config_servo_param_cb, 0);
		}
		break;
	case 7:
		{ // disable Slave's TPDO 关闭从机TPDO2发送用来写入参数
			UNS32 TPDO_COBId = 0x80000280 + nodeId;
			writeNetworkDictCallBack(OD_Data, nodeId, 0x1801, 1, 
				4, uint32, &TPDO_COBId, config_servo_param_cb, 0);
		}
		break;
	case 8:
		{
			UNS8 trans_type = PDO_TRANSMISSION_TYPE;
			writeNetworkDictCallBack(OD_Data, nodeId, 0x1801, 2, 
				1, uint8, &trans_type, config_servo_param_cb, 0);
		}
		break;
	case 9:
		{
			UNS8 pdo_map_cnt = 0;//清除索引
			writeNetworkDictCallBack(OD_Data, nodeId, 0x1A01, 0, 
				1, uint8, &pdo_map_cnt, config_servo_param_cb, 0);
		}
		break;
	case 10:
    local_od_send(0X1A01,1,nodeId);//写入从机PDO1需要发送映射，即主站需要PDO1接收映射
		break;
	case 11:
      local_od_send(0X1A01,0,nodeId);//写入从机PDO1需要发送映射，即主站需要PDO1接收映射
		break;
	case 12:
		{ // enable Slave's TPDO
			UNS32 TPDO_COBId =PDO_ENANBLE(0x00000280,nodeId);
			writeNetworkDictCallBack(OD_Data, nodeId, 0x1801, 1, 
				4, uint32, &TPDO_COBId, config_servo_param_cb, 0);
		}
		break;
	case 13:
		{ // disable Slave's RPDO
			UNS32 RPDO_COBId = PDO_DISANBLE(0x80000200,nodeId);;
			writeNetworkDictCallBack(OD_Data, nodeId, 0x1400, 1, 
				4, uint32, &RPDO_COBId, config_servo_param_cb, 0);
		}
		break;
	case 14:
		{
			UNS8 trans_type = PDO_TRANSMISSION_TYPE;
			writeNetworkDictCallBack(OD_Data, nodeId, 0x1400, 2, 
				1, uint8, &trans_type, config_servo_param_cb, 0);
		}
		break;
	case 15:
		{
			UNS8 pdo_map_cnt = 0;//清除索引
			writeNetworkDictCallBack(OD_Data, nodeId, 0x1600, 0, 
				1, uint8, &pdo_map_cnt, config_servo_param_cb, 0);
		}
		break;
	case 16:
      local_od_send(0X1600,1,nodeId);
		break;
	case 17:
      local_od_send(0X1600,2,nodeId);
		break;
	case 18:
      local_od_send(0X1600,0,nodeId);
		break;
	case 19:
		{ // enable Slave's RPDO
			UNS32 RPDO_COBId = PDO_ENANBLE(0x00000200,nodeId);
			writeNetworkDictCallBack(OD_Data, nodeId, 0x1400, 1, 
				4, uint32, &RPDO_COBId, config_servo_param_cb, 0);
		}
		break;
	case 20:
		{ // disable Slave's RPDO
			UNS32 RPDO_COBId =PDO_DISANBLE(0x80000300,nodeId);
			writeNetworkDictCallBack(OD_Data, nodeId, 0x1401, 1, 
				4, uint32, &RPDO_COBId, config_servo_param_cb, 0);
		}
		break;
	case 21:
		{
			UNS8 trans_type = PDO_TRANSMISSION_TYPE;
			writeNetworkDictCallBack(OD_Data, nodeId, 0x1401, 2, 
				1, uint8, &trans_type, config_servo_param_cb, 0);
		}
		break;
	case 22:
		{
			UNS8 pdo_map_cnt = 0;//清除索引
			writeNetworkDictCallBack(OD_Data, nodeId, 0x1601, 0, 
				1, uint8, &pdo_map_cnt, config_servo_param_cb, 0);
		}
		break;
	case 23:
      local_od_send(0X1601,1,nodeId);
		break;
	case 24:
      local_od_send(0X1601,2,nodeId);
		break;
	case 25:
      local_od_send(0X1601,0,nodeId);
		break;	
	case 26:
		{ // enable Slave's RPDO
			UNS32 TPDO_COBId = PDO_ENANBLE(0x00000300,nodeId);
			writeNetworkDictCallBack(OD_Data, nodeId, 0x1401, 1, 
				4, uint32, &TPDO_COBId, config_servo_param_cb, 0);
		}
		break;
	case 27:
		{
			UNS16 producer_heartbeat_time = PRODUCER_HEARTBEAT_TIME;
			writeNetworkDictCallBack(OD_Data, nodeId, 0x1017, 0, 
				2, uint16, &producer_heartbeat_time, config_servo_param_cb, 0);
		}
		break;
	case 28:
		{
      //需要本地字典写入生产者心跳时间，不然驱动器报错
			UNS32 consumer_heartbeat_time = HEARTBEAT_FORMAT(MASTER_NODEID,CONSUMER_HEARTBEAT_TIME);//写入节点1的心跳时间
      writeNetworkDictCallBack(OD_Data, nodeId, 0x1016, 1, 
				4, uint16, &consumer_heartbeat_time, config_servo_param_cb, 0);
		}
		break;
	case 29:
		rt_sem_release(&(conf->finish_sem));
		break;
	default:
		break;
	}
}*/
