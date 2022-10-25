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
struct servo_config_state
{
	uint8_t state;
	uint8_t try_cnt;
  uint8_t err_code;//���ò���������� 0xff,����δ����,�����ֵ������ 0x03,���ûظ�δ��Ӧ���ڵ��ֵ����
	struct rt_semaphore finish_sem;
};
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/*DS301����Э��д��ڵ�������ʽ
  NodeID  : �ڵ�
  Interval: �������*/
#define HEARTBEAT_FORMAT(NodeID,Interval) (((NodeID) << 16) | (Interval))
/*DS301Э�� 31λ��1�ر�PDO��RPDO/TPDO������
  NodeID  : �ڵ�
  Default :  Ĭ��ֵ*/
#define PDO_DISANBLE(NodeID,Default) ((1<<31) | (Default) | (NodeID))
/*DS301Э�� 31λ��1ʹ��PDO��RPDO/TPDO������
  NodeID  : �ڵ�
  Default :  Ĭ��ֵ*/
#define PDO_ENANBLE(Default,NodeID) ((Default) | (NodeID))
/*DS301Э�� 31λ��1ʹ��SYNCͬ����Ϣ
  NodeID  : �ڵ�*/
#define SYNC_ENANBLE(NodeID) ((1 << 30) | (NodeID))
/* Private variables ---------------------------------------------------------*/
CO_Data *OD_Data = &master402_Data;
s_BOARD agv_board  = {"0", "1M"};//û��
static struct servo_config_state servo_conf[MAX_SERVO_COUNT + 1];//����״̬
/* Private function prototypes -----------------------------------------------*/
static void config_servo_param(uint8_t nodeId, struct servo_config_state *conf);
/***********************��ʼ������״̬����**************************************************/
/**
  * @brief  ��ʼ�ڵ�
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
  * @brief  ��ʼ��canopen
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
  * @brief  ��ӡ�ڵ�״̬
  * @param  state���ڵ�״̬
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
  * @brief  MSH����canopen��վnmt״̬
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
/***********************Ԥ����״̬����**************************************************/
/**
  * @brief  ���ò�����ɻص�
  * @param  None
  * @retval None
  * @note   None
*/
static void config_servo_param_cb(CO_Data* d, UNS8 nodeId)
{
	UNS32 abortCode;
	UNS8 res;
	struct servo_config_state *conf;

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
  * @brief  �����ŷ�����
  * @param  None
  * @retval None
  * @note   �������ȴ��������
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
  * @brief  �ӻ��������ߴ�������
  * @param  None
  * @retval None
  * @note   ����config_single_servo�߳�
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
  * @brief  д�뱾���ֵ���������л�nmt����
  * @param  None
  * @retval None
  * @note   nmt״̬�л�Ϊoperation״̬�����̣߳�������ɾ���߳�
*/
void canopen_start_thread_entry(void *parameter)
{
	UNS32 sync_id, size;
	UNS8 data_type, sub_cnt;
	UNS32 consumer_heartbeat_time;
  Write_SLAVE_control_word(SERVO_NODEID,0x80);//��ʼ�����д�������
  
  rt_thread_delay(200);
	config_servo(SERVO_NODEID);
  if(servo_conf[SERVO_NODEID - 2].err_code != 0X00)//�����ô����µ��˳�
  {
    if(servo_conf[SERVO_NODEID - 2].err_code == 0XFF)
    {
      LOG_E("The configuration was not sent because the local dictionary failed");
    }
    else if(servo_conf[SERVO_NODEID - 2].err_code == 0X03)
    {
      LOG_E("The configuration reply did not respond, and the node dictionary failed");
    }
    LOG_W("Waiting for the repair to complete, CAN communication is currently unavailable");
    master402_fix_config_err(SERVO_NODEID);
    return; //�˳��߳�
  }
	OD_Data->post_SlaveBootup = slaveBootupHdl;
  /**д������������/���ն��ж�������ʱʱ��  DS301����**/
  /**�и�ʽ���壬�ֵ乤��û��֧�֣���Ҫ�Լ�д��**/
	consumer_heartbeat_time = HEARTBEAT_FORMAT(SERVO_NODEID,CONSUMER_HEARTBEAT_TIME);//д��ڵ�2������ʱ��
	size = 4;
	writeLocalDict(OD_Data, 0x1016, 1, &consumer_heartbeat_time, &size, 0);
	consumer_heartbeat_time = HEARTBEAT_FORMAT(SERVO_NODEID + 1,CONSUMER_HEARTBEAT_TIME);//д��ڵ�3������ʱ��,û�нڵ㼴����
	writeLocalDict(OD_Data, 0x1016, 2, &consumer_heartbeat_time, &size, 0);
	sub_cnt = 2;
	size = 1;
	writeLocalDict(OD_Data, 0x1016, 0, &sub_cnt, &size, 0);
  /**д������������/���Ͷ���������ʱ��  DS301����**/
  UNS16 producer_heartbeat_time;
	producer_heartbeat_time = PRODUCER_HEARTBEAT_TIME;
	size = 2;
	writeLocalDict(OD_Data, 0x1017, 0, &producer_heartbeat_time, &size, 0);
  /**�и�ʽ���壬�ֵ乤��û��֧�֣���Ҫ�Լ�д��**/
	data_type = uint32;
	setState(OD_Data, Operational);
	masterSendNMTstateChange(OD_Data, SERVO_NODEID, NMT_Start_Node);
	size = 4;
	readLocalDict(OD_Data, 0x1005, 0, &sync_id, &size, &data_type, 0);
	sync_id = SYNC_ENANBLE(sync_id);//DS301 30λ��1 ������CANopen�豸����ͬ����Ϣ
	writeLocalDict(OD_Data, 0x1005, 0, &sync_id, &size, 0);
}
/******************************д��ڵ��ֵ��������**********************************/
/**
  * @brief  ������ɻص�
  * @param  d:�ֵ�ṹ��
  * @param  nodeId���ڵ�ID
  * @retval None
  * @note   None
*/
static void writeNetworkDictSyncCb(CO_Data* d, UNS8 nodeId) 
{
    struct servo_config_state *conf = &servo_conf[nodeId - 2];
    rt_sem_release(&conf->finish_sem);
}
/**
  * @brief  �첽д��ڵ��ֵ�
  * @param  d:�ֵ�ṹ��
  * @param  nodeId���ڵ�ID
  * @retval None
  * @note   None
*/
static bool writeNetworkDictSync (CO_Data* d, UNS8 nodeId, UNS16 index,
        UNS8 subIndex, UNS32 count, UNS8 dataType, void *data, UNS8 useBlockMode) 
{

    if(nodeId < 1 || nodeId > MAX_SERVO_COUNT + 1) 
    {
      LOG_W("invalid nodeId:%d, should between 1 and %d",nodeId,MAX_SERVO_COUNT + 1);
        return false;
    }

    int try_cnt = 3;
    struct servo_config_state* conf = &servo_conf[nodeId - 2];
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
  * @brief  ��ȡ�����ֵ������д��ڵ��ֵ�
  * @param  d:�ֵ�ṹ��
  * @param  nodeId���ڵ�ID
  * @param  index:����
  * @param  subIndex:������
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
  * @brief  ��ȡ�����ֵ�ֵ���������ӻ��ڵ�
  * @param  None.
  * @retval None.
  * @note   0XFF:����0X00:�ɹ�.
*/
static UNS8 local_od_send(UNS16 index,UNS8 subIndex,uint8_t nodeId)
{
  UNS32 pdo_map_val;
  UNS32  size = SDO_MAX_LENGTH_TRANSFER;
  UNS8  dataType;
  UNS32 errorCode;
  UNS16  slave_Index;
  
  if(0X1600 <= index && index <= 0X17FF)//�ӻ� Receive PDO Parameters
  {
    //����վTransmit PDO ParametersѰ�Ҷ�Ӧ����
    slave_Index = index + 0X400;
  }
  else if(0X1A00 <= index && index <= 0X1BFF)//Transmit PDO Parameters
  {
    //����վReceive PDO ParametersѰ�Ҷ�Ӧ����
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
/******************************SDO���ò�������**********************************/
/**
  * @brief  д��ӻ�Modes_of_operation����
  * @param  nodeId:�ڵ�ID
  * @param  mode:Modes_of_operation 
  * @retval �ɹ�����0X00,ʧ�ܷ���0XFF
  * @note   None.
*/
UNS8 Write_SLAVE_Modes_of_operation(UNS8 nodeId,INTEGER8 mode)
{
  UNS8 errcode = 0;
  if(0 <= Modes_of_operation && Modes_of_operation <= 10)
  {
    //����д�뱾���ֵ�
    Modes_of_operation = mode;
  }
  else
  {
    LOG_E("The Modes_of_operation is out of range. Select a value from 0 to 10");
    return 0XFF;
  }

  //�������ͽڵ��ֵ�
  if(Read_local_Send_Node(OD_Data,nodeId,0x6060,0x00) == 0XFF)
    return 0XFF;

  return 0X00;
}
/**
  * @brief  д��ӻ�control_word����
  * @param  nodeId:�ڵ�ID
  * @param  value:Controlword 
  * @retval �ɹ�����0X00,ʧ�ܷ���0XFF
  * @note   None.
*/
UNS8 Write_SLAVE_control_word(UNS8 nodeId,UNS16 value)
{
  UNS8 errcode = 0;
  //����д�뱾���ֵ�
  Controlword = value;

  //�������ͽڵ��ֵ�
  if(Read_local_Send_Node(OD_Data,nodeId,0x6040,0x00) == 0XFF)
    return 0XFF;

  return 0X00;
}
/**
  * @brief  д��ӻ�λ�ù滮ģʽ���ٶ�����
  * @param  nodeId:�ڵ�ID
  * @param  speed:�˶��ٶ� ��λ:RPM
  * @retval �ɹ�����0X00,ʧ�ܷ���0XFF
  * @note   Profile_velocity ��λPUU/s �趨��Χ UNSIGNED32 Ĭ��ֵ 10000
*/
UNS8 Write_SLAVE_profile_position_speed_set(UNS8 nodeId,UNS32 speed)
{
  UNS8 errcode = 0;
  //����д�뱾���ֵ�
  if(0 <= speed && speed <= 3000)
    Profile_velocity = speed * ELECTRONIC_GEAR_RATIO_NUMERATOR / 60;//
  else
  {
    LOG_E("The Profile_velocity is out of range. Select a value from 0 to 3000");
    return 0XFF;
  }

  //�������ͽڵ��ֵ�
  if(Read_local_Send_Node(OD_Data,nodeId,0X6081,0x00) == 0XFF)
    return 0XFF;

  return 0X00;
}
/**
  * @brief  д��ӻ�0X60C2�岹���ڲ���
  * @param  nodeId:�ڵ�ID
  * @retval �ɹ�����0X00,ʧ�ܷ���0XFF
  * @note   �趨����ͨѶ���� OD 1006h��ͬ
  * �岹����ʱ�� = 60C2H sub1 �� 10^(60C2H sub2)
  * ����������Ҫ�趨�岹����ʱ��Ϊ 10 ms���轫 OD 60C2h sub1 = 1��OD 60C2h sub2 = -4
    �岹����ʱ�� = 1 �� 10^-2 = 0.01s = 10ms
*/
UNS8 Write_SLAVE_Interpolation_time_period(UNS8 nodeId)
{
  UNS32  size = SDO_MAX_LENGTH_TRANSFER;
  UNS32 pdo_map_val,errorCode; 
  UNS8  dataType;
  UNS16 index = 0X1006;//ͨѶ����ֵ
  UNS8 subIndex = 0;//������0

  errorCode = readLocalDict(OD_Data,index,subIndex,(void *)&pdo_map_val,&size,&dataType,0);//��ȡ0X1006ͨѶ����ֵ
  if(errorCode != OD_SUCCESSFUL)
  {
    LOG_E("index:0X%X,subIndex:0X%X,read Local Dict false,SDO abort code is 0X%X",index,subIndex,errorCode);
    return 0xFF;
  }
  extern int Bit_Int_2(long n);
  Interpolation_time_period_Interpolation_time_index = (Bit_Int_2(pdo_map_val) - 1) - 6;//ʮ�Ĵη��� = 10^(n-6)
  Interpolation_time_period_Interpolation_time_units = pdo_map_val / pow(10,(Bit_Int_2(pdo_map_val) - 1));//�岹����ʱ�䳣��
  
  if(Read_local_Send_Node(OD_Data,nodeId,0X60C2,0x01) == 0XFF)
    return 0XFF;
  if(Read_local_Send_Node(OD_Data,nodeId,0X60C2,0x02) == 0XFF)
    return 0XFF;

  return 0x00;
}
/**
  * @brief  д��ӻ�ԭ�㸴��ģʽ���������
  * @param  nodeId:�ڵ�ID
  * @param  offest:ԭ��ƫ��ֵ ��λ:PUU [ע��:ֻ�ǰ�ԭ��ƫ��ֵ�㵱Ϊ0����㣬�������˶���0����㴦]
  * @param  method:��ԭ��ʽ   ��Χ:0 ~ 35
  * @param  switch_speed:Ѱ��ԭ�㿪���ٶ� �趨��Χ 0.1 ~ 200 Ĭ��ֵ 10  ��λrpm ����:С�����һλ
  * @param  zero_speed:Ѱ�� Z�����ٶ�     �趨��Χ 0.1 ~ 50  Ĭ��ֵ 2   ��λrpm ����:С�����һλ
  * @retval �ɹ�����0X00,ʧ�ܷ���0XFF
  * @note   Home PositionΪִ��ԭ�㸴��ʱ���ҵ���ԭ��ο��㣬��ԭ�� Sensor��Z����ȡ�
            ��ԭ��ο����ҵ��󣬴Ӹõ���ƫ�Ƶ�λ�ü�Ϊ�û������ԭ�㡣

            �� 35��ԭ�㸴��ģʽ���û��趨���� 1 ~ 16�ֻ�ԭ��ģʽ����Ѱ Z���壻 
            ���� 17 ~ 34�����ǲ���Ѱ Z���壻�� 35�����Ƕ��嵱ǰλ��Ϊԭ�㡣

            switch_speed��zero_speedд�뵥λΪ0.1rpm

            �趨ԭ�㸴���/����ʱ�� OD 609Ah Ĭ��ֵ 100 ��λ ms 
            �˶������趨��ʱ��б��Ϊ���� (0 rpm���ٵ� 3000 rpm) ������ (3000 rpm���ٵ�0 rpm) ����Ҫ��ʱ�䡣�˶���������� Homing Mode (ԭ�㸴��ģʽ)�� 
*/
UNS8 Write_SLAVE_Homing_set(UNS8 nodeId,UNS32 offset,UNS8 method,float switch_speed,float zero_speed)
{
  UNS8 errcode = 0;
  //����д�뱾���ֵ�
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

  //�������ͽڵ��ֵ�
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
/******************************TPDO1 ���ò�������
                               ���ݳ��ȱ�����Ϊ 1~8 �ֽ�*********************************/
/**
  * ��������ʾPDO1���� , ֡ID0X182 DLC:8
  * �ֵ乤�ߣ�0x1400 Receive PDO 1 Parameter:COB ID used by PDO:0x00000182
  * �ֵ乤�ߣ�0x1600 Receive PDO 1 Mapping.������Ҫ��������
  * Ϊ�ӻ�����ͨ��TPDO1,����������ͨ��RPDO1
  * PDO DATA:0x50 06 4E C3 00 00 00 00  С��ģʽ.
  * SUBINDEX2:32λ,ռ��4���ֽ�.��00 00 C3 4E = 49998 Position actual value (Ox6064)
  * SUBINDEX2:32λ,ռ��4���ֽ�.��00 00 C3 4E = 49998 Velocity_actual_value (Ox606C)
*/
static UNS8 DIS_SLAVE_TPDO1(uint8_t nodeId)
{
// disable Slave's TPDO���رմӻ�TPDO1��������д�����
  UNS32 TPDO_COBId = PDO_DISANBLE(0x00000180,nodeId);//0x80000180 + nodeId;
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1800, 1, 
    4, uint32, &TPDO_COBId, config_servo_param_cb, 0);
}
static UNS8 Write_SLAVE_TPDO1_Type(uint8_t nodeId)
{
  UNS8 trans_type = PDO_TRANSMISSION_TYPE;//д������
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1800, 2, 
    1, uint8, &trans_type, config_servo_param_cb, 0);
}
static UNS8 Clear_SLAVE_TPDO1_Cnt(uint8_t nodeId)
{
  UNS8 pdo_map_cnt = 0;//����ӻ�����������
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1A00, 0, 
    1, uint8, &pdo_map_cnt, config_servo_param_cb, 0);
}
static UNS8 Write_SLAVE_TPDO1_Sub1(uint8_t nodeId)
{
  return local_od_send(0X1A00,1,nodeId);//д��ӻ�PDO1��Ҫ����ӳ�䣬����վ��ҪPDO1����ӳ��
}
static UNS8 Write_SLAVE_TPDO1_Sub2(uint8_t nodeId)
{
  return local_od_send(0X1A00,2,nodeId);//д��ӻ�PDO1��Ҫ����ӳ�䣬����վ��ҪPDO1����ӳ��
}
static UNS8 Write_SLAVE_TPDO1_Sub0(uint8_t nodeId)
{
  return local_od_send(0X1A00,0,nodeId);//����д����ȷ������
}
static UNS8 EN_SLAVE_TPDO1(uint8_t nodeId)
{
//enable Slave's TPDO1 ʹ�ܴӻ�����PDO1
  UNS32 TPDO_COBId = PDO_ENANBLE(0x00000180,nodeId);
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1800, 1, 
    4, uint32, &TPDO_COBId, config_servo_param_cb, 0);
}
/******************************TPDO2 ���ò�������
                               ���ݳ��ȱ�����Ϊ 1~8 �ֽ�*********************************/
/**
  * ��������ʾPDO2���� , ֡ID0X282 DLC:4
  * �ֵ乤�ߣ�0x1401 Receive PDO 2 Parameter:COB ID used by PDO:0x00000282
  * �ֵ乤�ߣ�0x1601 Receive PDO 2 Mapping.������Ҫ��������
  * Ϊ�ӻ�����ͨ��TPDO2,����������ͨ��RPDO2
  * PDO DATA:0x37 16   С��ģʽ.
  * SUBINDEX1:16λ,ռ��2���ֽ�.��0X1637 Statusword (Ox6041) 
*/
static UNS8 DIS_SLAVE_TPDO2(uint8_t nodeId)
{
 //disable Slave's TPDO �رմӻ�TPDO2��������д�����
  UNS32 TPDO_COBId = PDO_DISANBLE(0x00000280,nodeId);//0x80000280 + nodeId;
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1801, 1, 
    4, uint32, &TPDO_COBId, config_servo_param_cb, 0);
}
static UNS8 Write_SLAVE_TPDO2_Type(uint8_t nodeId)
{
  UNS8 trans_type = PDO_TRANSMISSION_TYPE;//д������
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1801, 2, 
    1, uint8, &trans_type, config_servo_param_cb, 0);
}
static UNS8 Clear_SLAVE_TPDO2_Cnt(uint8_t nodeId)
{
  UNS8 pdo_map_cnt = 0;//����ӻ�����������
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1A01, 0, 
    1, uint8, &pdo_map_cnt, config_servo_param_cb, 0);
}
static UNS8 Write_SLAVE_TPDO2_Sub1(uint8_t nodeId)
{
  return local_od_send(0X1A01,1,nodeId);//д��ӻ�PDO1��Ҫ����ӳ�䣬����վ��ҪPDO1����ӳ��
}
static UNS8 Write_SLAVE_TPDO2_Sub0(uint8_t nodeId)
{
  return local_od_send(0X1A01,0,nodeId);//����д����ȷ������
}
static UNS8 EN_SLAVE_TPDO2(uint8_t nodeId)
{
//enable Slave's TPDO1 ʹ�ܴӻ�����PDO1
  UNS32 TPDO_COBId = PDO_ENANBLE(0x00000280,nodeId);
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1801, 1, 
    4, uint32, &TPDO_COBId, config_servo_param_cb, 0);
}
/******************************RPDO1 ���ò�������
                               ���ݳ��ȱ�����Ϊ 1~8 �ֽ�*********************************/
/**
  * ��������ʾPDO1���� , ֡ID0X202 DLC:8
  * �ֵ乤�ߣ�0x1800 Transmit PDO 1 Parameter:COB ID used by PDO:0x00000202
  * �ֵ乤�ߣ�0x1A00 Transmit PDO 1 Mapping.������Ҫ��������
  * Ϊ�ӻ�����ͨ��RPDO1,����������ͨ��TPDO1
  * PDO DATA:0x00 00 01 00 00 00 00   С��ģʽ.
  * SUBINDEX1:32λ,ռ��4���ֽ�.��00 00 C3 50 = 50000  Target position (0x607A)
  * SUBINDEX2:32λ,ռ��4���ֽ�.��00 01 86 A0 = 100000 Target_velocity (0x60FF)
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
  UNS8 pdo_map_cnt = 0;//�������
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1600, 0, 
    1, uint8, &pdo_map_cnt, config_servo_param_cb, 0);
}
static UNS8 Write_SLAVE_RPDO1_Sub1(uint8_t nodeId)
{
  return local_od_send(0X1600,1,nodeId);//д��ӻ�PDO1��Ҫ����ӳ�䣬����վ��ҪPDO1����ӳ��
}
static UNS8 Write_SLAVE_RPDO1_Sub2(uint8_t nodeId)
{
  return local_od_send(0X1600,2,nodeId);//д��ӻ�PDO1��Ҫ����ӳ�䣬����վ��ҪPDO1����ӳ��
}
static UNS8 Write_SLAVE_RPDO1_Sub0(uint8_t nodeId)
{
  return local_od_send(0X1600,0,nodeId);//����д����ȷ������
}
static UNS8 EN_SLAVE_RPDO1(uint8_t nodeId)
{
// enable Slave's RPDO
  UNS32 RPDO_COBId = PDO_ENANBLE(0x00000200,nodeId);
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1400, 1, 
    4, uint32, &RPDO_COBId, config_servo_param_cb, 0);
}
/******************************RPDO2 ���ò�������
                               ���ݳ��ȱ�����Ϊ 1~8 �ֽ�*********************************/
/**
  * ��������ʾPDO2���� , ֡ID0X302 DLC:4
  * �ֵ乤�ߣ�0x1801 Transmit PDO 2 Parameter:COB ID used by PDO:0x00000302
  * �ֵ乤�ߣ�0x1A01 Transmit PDO 2 Mapping.������Ҫ��������
  * Ϊ�ӻ�����ͨ��RPDO2,����������ͨ��TPDO2
  * PDO DATA:0x50 C3 00 00  С��ģʽ.
  * SUBINDEX1:32λ,ռ��4���ֽ�.��00 00 C3 50 = 50000  POS_CMD (0x60C1)
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
  UNS8 pdo_map_cnt = 0;//�������
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1601, 0, 
    1, uint8, &pdo_map_cnt, config_servo_param_cb, 0);
}
static UNS8 Write_SLAVE_RPDO2_Sub1(uint8_t nodeId)
{
  return local_od_send(0X1601,1,nodeId);//д��ӻ�PDO1��Ҫ����ӳ�䣬����վ��ҪPDO1����ӳ��
}
static UNS8 Write_SLAVE_RPDO2_Sub0(uint8_t nodeId)
{
  return local_od_send(0X1601,0,nodeId);//����д����ȷ������
}
static UNS8 EN_SLAVE_RPDO2(uint8_t nodeId)
{
// enable Slave's RPDO
  UNS32 TPDO_COBId = PDO_ENANBLE(0x00000300,nodeId);
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1401, 1, 
    4, uint32, &TPDO_COBId, config_servo_param_cb, 0);
}
/******************************����ʱ�� ���ò�������*********************************/
static UNS8 Write_SLAVE_P_heartbeat(uint8_t nodeId)
{
  UNS16 producer_heartbeat_time = PRODUCER_HEARTBEAT_TIME;
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1017, 0, 
    2, uint16, &producer_heartbeat_time, config_servo_param_cb, 0);
}
static UNS8 Write_SLAVE_C_heartbeat(uint8_t nodeId)
{
  //��Ҫ�����ֵ�д������������ʱ�䣬��Ȼ����������
  UNS32 consumer_heartbeat_time = HEARTBEAT_FORMAT(CONTROLLER_NODEID,CONSUMER_HEARTBEAT_TIME);//д��ڵ�1������ʱ��
  return writeNetworkDictCallBack(OD_Data, nodeId, 0x1016, 1, 
    4, uint16, &consumer_heartbeat_time, config_servo_param_cb, 0);
}
/*****************************������������*********************************/
/**
  * @brief  �����ӻ�S�ͼӼ��ٹ���
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
/*****************************�������ò�������*********************************/
static UNS8 MotorCFG_Done(uint8_t nodeId)
{
	struct servo_config_state *conf;
	conf = &servo_conf[nodeId - 2];
  rt_sem_release(&(conf->finish_sem));
  return 0;
}
/**
  * @brief  ����ָ������.
  * @param  None.
  * @retval None.
  * @note   
ע�⺯����˳�������Խ������
https://blog.csdn.net/feimeng116/article/details/107515317
*/
static UNS8 (*MotorCFG_Operation[])(uint8_t nodeId) = 
{ 
  //TPDO1ͨ������
  DIS_SLAVE_TPDO1,
  Write_SLAVE_TPDO1_Type,
  Clear_SLAVE_TPDO1_Cnt,
  Write_SLAVE_TPDO1_Sub1,
  Write_SLAVE_TPDO1_Sub2,
  Write_SLAVE_TPDO1_Sub0,
  EN_SLAVE_TPDO1,
  //TPDO2ͨ������
  DIS_SLAVE_TPDO2,
  Write_SLAVE_TPDO2_Type,
  Clear_SLAVE_TPDO2_Cnt,
  Write_SLAVE_TPDO2_Sub1,
//  Write_SLAVE_TPDO2_Sub2,
  Write_SLAVE_TPDO2_Sub0,
  EN_SLAVE_TPDO2,
  //RPDO1ͨ������
  DIS_SLAVE_RPDO1,
  Write_SLAVE_RPDO1_Type,
  Clear_SLAVE_RPDO1_Cnt,
  Write_SLAVE_RPDO1_Sub1,
  Write_SLAVE_RPDO1_Sub2,
  Write_SLAVE_RPDO1_Sub0,
  EN_SLAVE_RPDO1,
  //RPDO2ͨ������
  DIS_SLAVE_RPDO2,
  Write_SLAVE_RPDO2_Type,
  Clear_SLAVE_RPDO2_Cnt,
  Write_SLAVE_RPDO2_Sub1,
  Write_SLAVE_RPDO2_Sub0,
  EN_SLAVE_RPDO2,
  //д������
  Write_SLAVE_P_heartbeat,
  Write_SLAVE_C_heartbeat,
  //��������
  Write_SLAVE_S_Move,
  //��������
  MotorCFG_Done,
};
/**
  * @brief  ���ò���
  * @param  None
  * @retval None
  * @note   None
*/
static void config_servo_param(uint8_t nodeId, struct servo_config_state *conf)
{
  //�ж�����Խ�� ָ���޷��ж�ָ�����ݴ�С
  //https://bbs.csdn.net/topics/80323809
  if(conf->state < sizeof(MotorCFG_Operation) / sizeof(MotorCFG_Operation[0]))
  {
    conf->err_code = MotorCFG_Operation[conf->state](nodeId);
  }
  if(conf->err_code != 0)//���ò�������
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
  * @brief  ���ò���
  * @param  None
  * @retval None
  * @note   None
*/
/*static void config_servo_param(uint8_t nodeId, struct servo_config_state *conf)
{
	switch(conf->state)
	{
	case 0:
		{ // disable Slave's TPDO���رմӻ�TPDO1��������д�����
			UNS32 TPDO_COBId = PDO_DISANBLE(0x00000180,nodeId);//0x80000180 + nodeId;
			writeNetworkDictCallBack(OD_Data, nodeId, 0x1800, 1, 
				4, uint32, &TPDO_COBId, config_servo_param_cb, 0);
		}
		break;
	case 1:
		{
			UNS8 trans_type = PDO_TRANSMISSION_TYPE;//д������
			writeNetworkDictCallBack(OD_Data, nodeId, 0x1800, 2, 
				1, uint8, &trans_type, config_servo_param_cb, 0);
		}
		break;
	case 2:
		{
			UNS8 pdo_map_cnt = 0;//����ӻ�����������
			writeNetworkDictCallBack(OD_Data, nodeId, 0x1A00, 0, 
				1, uint8, &pdo_map_cnt, config_servo_param_cb, 0);
		}
		break;
	case 3:
    local_od_send(0X1A00,1,nodeId);//д��ӻ�PDO1��Ҫ����ӳ�䣬����վ��ҪPDO1����ӳ��
		break;
	case 4:
		local_od_send(0X1A00,2,nodeId);//д��ӻ�PDO1��Ҫ����ӳ�䣬����վ��ҪPDO1����ӳ��
		break;
	case 5:
    local_od_send(0X1A00,0,nodeId);//����д����ȷ������
		break;
	case 6:
		{ // enable Slave's TPDO1 ʹ�ܴӻ�����PDO1
			UNS32 TPDO_COBId = PDO_ENANBLE(0x00000180,nodeId);
			writeNetworkDictCallBack(OD_Data, nodeId, 0x1800, 1, 
				4, uint32, &TPDO_COBId, config_servo_param_cb, 0);
		}
		break;
	case 7:
		{ // disable Slave's TPDO �رմӻ�TPDO2��������д�����
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
			UNS8 pdo_map_cnt = 0;//�������
			writeNetworkDictCallBack(OD_Data, nodeId, 0x1A01, 0, 
				1, uint8, &pdo_map_cnt, config_servo_param_cb, 0);
		}
		break;
	case 10:
    local_od_send(0X1A01,1,nodeId);//д��ӻ�PDO1��Ҫ����ӳ�䣬����վ��ҪPDO1����ӳ��
		break;
	case 11:
      local_od_send(0X1A01,0,nodeId);//д��ӻ�PDO1��Ҫ����ӳ�䣬����վ��ҪPDO1����ӳ��
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
			UNS8 pdo_map_cnt = 0;//�������
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
			UNS8 pdo_map_cnt = 0;//�������
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
      //��Ҫ�����ֵ�д������������ʱ�䣬��Ȼ����������
			UNS32 consumer_heartbeat_time = HEARTBEAT_FORMAT(MASTER_NODEID,CONSUMER_HEARTBEAT_TIME);//д��ڵ�1������ʱ��
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