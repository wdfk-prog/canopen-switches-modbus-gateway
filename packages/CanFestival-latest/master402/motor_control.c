#include <stdint.h>
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : 
  * @brief          : 
  * @date           :
  ******************************************************************************
  * @attention
1  Profile Position Mode (λ�ù滮ģʽ) 
  1. �趨ģʽ��OD 6060h = 01h��Ϊλ�ÿ���ģʽ�� 
  2. �趨Ŀ��λ�ã�OD 607Ah (��λ��PUU)�� 
  PUU(Pulse of User Unit),�˵�λΪ�������ӳ��ֱ������ŵ���
  3. �趨�ٶ����OD 6081h (��λ��PUU/sec)�� //��Ĭ��ֵ
  4. �趨���ٶ�ʱ��б�ʣ�OD 6083h (��λ��ms)��//��Ĭ��ֵ
  5. �趨���ٶ�ʱ��б�ʣ�OD 6084h (��λ��ms)��//��Ĭ��ֵ
  6. �趨����ָ�OD 6040h�����������²������������ 6.1�� 6.2��Ϊ��ʹ������ 
  ��״̬�� (state machine) ����׼��״̬��״̬��˵��������½� 12.4�� OD 6040h
  ˵���� 
  ����  ˵�� 
  6.1  Shutdown (�ر�) 
  6.2  Switch on (�ŷ� Servo On׼��) 
  6.3  Enable Operation (�ŷ� Servo On) 
  6.4  ����� (��Ե����) 1 1 1 1 1��bit0~bit4��
  bitλ  ����          ֵ   ˵��
  4      ���趨��      0     û�ге�Ŀ��λ��
                       1     ����Ŀ��λ��
  5      ���������趨  0     ���ʵ�ʶ�λ��Ȼ��ʼ��һ����λ
                       1     �ж�ʵ�ʶ�λ����ʼ��һ����λ
  6      abs / rel     0     Ŀ��λ����һ������ֵ
                       1     Ŀ��λ����һ�����ֵ
  7      ֹͣ          0     ִ�ж�λ
                       1     ���������ٵ�ֹͣ��(���û����������֧��)
  7. ����ɵ�һ���˶����������Ҫִ����һ���˶����������趨Ŀ��λ�á��ٶȵ� 
  ������ 
  8. �趨����ָ�OD 6040h���������������Ե��������˱����Ƚ� Bit 4��Ϊ off
  ������ on�� 
  ����  ˵�� 
  8.1   Enable Operation (�ŷ� Servo On) 
  8.2   ����� (��Ե����) 
  ��ȡ��������Ϣ�� 
  1. ��ȡ OD 6064hȡ��Ŀǰ�������λ�á� 
  2. ��ȡ OD 6041hȡ����������״̬������ following error (׷�����)��set-point 
  acknowledge (�յ�����֪ͨ) �� target reached (����Ŀ��֪ͨ)
  * @author
  ̨��A3�������ΪCANOPENģʽ
  1.�ָ���������
  �û��������в������� CANopen��λ���� ASDA-A3�ŷ��������� 
  1. �趨 CANopenģʽ�������� P1.001��Ϊ 0x0C�� 
  2. �趨�ڵ� ID���� P3.000��Χ��Ϊ 01h ~ 7Fh�� 
  3. ������ P3.001��Ϊ 0403h��P3.001.Z�趨���� 1 Mbps (0��125 Kbps�� 
  1��250 Kbps��2��500 Kbps��3��750 Kbps��4��1 Mbps)�� 
  4. ���齫 P3.012.Z�趨Ϊ 1����ʵ�ֽ��±������ϵ籣�ֵĹ��ܡ� 
  ���������������µ���ǽ���ͨѶ���ú��±��� P������ά�ֱ������趨��
  ������� CANopen / DMCNET / EtherCAT��������ֵ��
  5. ���鿪����̬��բ���ܣ�P1.032 = 0x0000��
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include <rtthread.h>
#include <rtdevice.h>
#ifdef RT_USING_FINSH
#include <finsh.h>
#endif

#include "canfestival.h"
#include "timers_driver.h"
#include "master402_od.h"
#include "master402_canopen.h"

/* Private typedef -----------------------------------------------------------*/
/*0x6040����ָ�� Controlword ״̬λ����*/
typedef enum
{
  WRITE_SWITCH_ON             = 1 << 0,//���¿���
  EN_VOLTAGE                  = 1 << 1,//ʹ�ܵ�Դ
  QUICK_STOP                  = 1 << 2,//��ͣ
  EN_OPERATION                = 1 << 3,//����ʹ��
  FAULT_RESET                 = 1 << 7,//��������
  HALT                        = 1 << 8,//��ͣ
}CONTROL_WORD;
/*0x6041״̬λ Statusword����*/
typedef enum
{
  READY_TO_SWITCH_ON          = 0,//׼����������
  READ_SWITCH_ON              = 1,//�ŷ�׼�����
  OPERATION_ENABLED           = 2,//�ŷ�ʹ��
  FAULT                       = 3,//�쳣�ź�
  VOLTAGE_ENABLED             = 4,//�ŷ�������ѹ���
  READ_QUICK_STOP             = 5,//����ֹͣ [0:������ͣ 1:�رռ�ͣ]
  SWITCH_ON_DISABLED          = 6,//�ŷ�׼�����ܹر�
  WARNING                     = 7,//�����ź�
  REMOTE                      = 9,//Զ�̿���
  TARGET_REACHED              = 10,//Ŀ�굽��
  POSITIVE_LIMIT              = 14,//������ת��ֹ����
  NEGATIVE_LIMIT              = 15,//������ת��ֹ����
}STATUS_WORD;
/*0X6060 ģʽ�趨Modes of operation����*/
typedef enum
{
  PROFILE_POSITION_MODE       = 1,//λ�ù滮ģʽ
  PROFILE_VELOCITY_MODE       = 3,//�ٶȹ滮ģʽ
  PROFILE_TORQUE_MODE         = 4,//Ť�ع滮ģʽ
  HOMING_MODE                 = 6,//ԭ�㸴��ģʽ
  INTERPOLATED_POSITION_MODE  = 7,//�岹λ��ģʽ
}MODE_OPERATION;
/*λ�ù滮ģʽ��Controlword����ģʽ�ض�λ*/
typedef enum
{
  NEW_SET_POINT               = 1 << 4,//�����(��Ե����)
  CHANGE_SET_IMMEDIATELY      = 1 << 5,//����������Чָ��  ��Ϊ 0���ر�����������Чָ��
  ABS_REL                     = 1 << 6,//����Ϊ���Զ�λ����Զ�λ  0��Ŀ��λ����һ������ֵ 1��Ŀ��λ����һ�����ֵ
}PROFILE_POSITION_CONTROLWORD;
/*λ�ù滮ģʽ��Statusword����*/
typedef enum
{
  SET_POINT_ACKNOWLEDGE       = 12,//�ŷ��յ������ź�
  FOLLOWING_ERROR             = 13,//׷�����
}PROFILE_POSITION_STATUSWORD;
/*�岹λ��ģʽ��Controlword����ģʽ�ض�λ
  Name          Value   Description
Enable ip mode    0     Interpolated position mode inactive 
                  1     Interpolated position mode active*/
typedef enum
{
   ENABLE_IP_MODE             = 1 << 4,//ʹ��IPģʽ
}Interpolated_Position_CONTROLWORD;
/*�岹λ��ģʽ��Statusword����
    Name        Value   Description
ip mode active    1     Interpolated position mode active
*/
typedef enum
{
  IP_MODE_ACTIVE              = 12,//�岹λ��ģʽ�Ƿ���Ч
}Interpolated_Position_STATUSWORD;
/*ԭ�㸴��ģʽ��Controlword����ģʽ�ض�λ
  Name          Value   Description
Homing          0       Homing mode inactive
operation       0 �� 1  Start homing mode
star            1       Homing mode active
                1 �� 0  Interrupt homing mode*/
typedef enum
{
  HOMING_OPERATION_STAR       = 1 << 4,//ʹ��IPģʽ
}HOMING_CONTROLWORD;
/*ԭ�㸴��ģʽ��Statusword����
    Name          Value   Description
Homing attained    0      Homing mode not yet completed.
                   1      Homing mode carried out successfully

Homing error       0      No homing error
                   1      Homing error occurred;Homing mode carried out not successfully;The error cause is found by reading the error code
*/
typedef enum
{
  HOMING_ATTAINED              = 12,//�ص�ԭ��
  HOMING_ERROR                 = 13,//��ԭ����
}HOMING_STATUSWORD;
/* Private define ------------------------------------------------------------*/
/*ʹ�������ӳ�ȥ���Ĳ��������ܱ�֤�����Ѿ�����
���Ĳ���ģʽ������ʹ��0X6061�鿴ģʽ��ȡ��֤���ĳɹ�
�����̣߳��жϳ�ʱ���ָ�����
*/
#define SYNC_DELAY            rt_thread_mdelay(20)//������ʱ
#define MAX_WAIT_TIME         50                                    //ms

/*0X6040 ����ָ�� Controlword ״̬��*/
//�ŷ� Servo Off
#define CONTROL_WORD_SHUTDOWN         (EN_VOLTAGE | QUICK_STOP) & (~WRITE_SWITCH_ON & ~FAULT_RESET)
//�ŷ�Servo On
#define CONTROL_WORD_SWITCH_ON        (WRITE_SWITCH_ON | EN_VOLTAGE | QUICK_STOP) & (~EN_OPERATION)  
//ִ���˶�ģʽ
#define CONTROL_WORD_ENABLE_OPERATION (WRITE_SWITCH_ON | EN_VOLTAGE | QUICK_STOP | EN_OPERATION & ~FAULT_RESET)
//�رչ���
#define CONTROL_WORD_DISABLE_VOLTAGE  (0 & ~EN_VOLTAGE & ~FAULT_RESET) 
//��ͣ
#define CONTROL_WORD_QUICK_STOP       (EN_VOLTAGE & ~QUICK_STOP & ~FAULT_RESET)
/* Private macro -------------------------------------------------------------*/
#define	CANOPEN_GET_BIT(x, bit)	  ((x &   (1 << bit)) >> bit)	/* ��ȡ��bitλ */
/*�ж�����ֵ��Ϊ0X00�˳���ǰ������*/
#define FAILED_EXIT(CODE){  \
if(CODE != 0X00)            \
   return 0XFF;             \
}
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  �����̲߳�ѯֵ�ض�λ�Ƿ���һ
  * @param  value:��Ҫ��ѯ�ı���
  * @param  expect:Ԥ��ֵ
  * @param  timeout:��ʱ�˳�ʱ�䣬��λms
  * @param  block_time:����ʱ�䣬��λms
  * @retval None
  * @note   ������ѯ
  *         ���ж��ض�λ�Ƿ���һ�������ж�ֵ�Ƿ���ȡ�
  *         ��ֹ��������£�����λ��һ�޷��ж�Ϊ�ı�ɹ�
*/
/*static UNS8 block_query_change(UNS16 *value,UNS16 expect,uint16_t timeout,uint16_t block_time)
{
  uint16_t cnt = 0;

  while((*value & expect) != expect)
  {
    if((cnt * block_time) < timeout)
    {
      cnt++;
      rt_thread_mdelay(block_time);
    }
    else
    {
      return 0xFF;
    }
  }
  return 0x00;
}
*/
/**
  * @brief  �����̲߳�ѯֵ�ض�λ�Ƿ���һ
  * @param  value:��Ҫ��ѯ�ı���
  * @param  bit:��ѯ��λ
  * @param  timeout:��ʱ�˳�ʱ�䣬��λms
  * @param  block_time:����ʱ�䣬��λms
  * @retval None
  * @note   ������ѯ
  *         ���ж��ض�λ�Ƿ���һ�������ж�ֵ�Ƿ���ȡ�
  *         ��ֹ��������£�����λ��һ�޷��ж�Ϊ�ı�ɹ�
*/
static UNS8 block_query_BIT_change(UNS16 *value,UNS8 bit,uint16_t timeout,uint16_t block_time)
{
  uint16_t cnt = 0;

  while(!CANOPEN_GET_BIT(*value,bit))
  {
    if((cnt * block_time) < timeout)
    {
      cnt++;
      rt_thread_mdelay(block_time);
    }
    else
    {
      return 0xFF;
    }
  }
  return 0x00; 
}  
/******************************�˶�ģʽѡ����******************************************************************/
/**
  * @brief  ���Ƶ��ʹ�ܲ�ѡ��λ�ù滮ģʽ
  * @param  nodeId:�ڵ�ID
  * @retval �ɹ�����0X00,ʧ�ܷ���0XFF
  * @note   None
*/
static UNS8 motor_on_profile_position(UNS8 nodeId)
{
  Target_position = 0;
  SYNC_DELAY;
  FAILED_EXIT(Write_SLAVE_Modes_of_operation(nodeId,PROFILE_POSITION_MODE));
  FAILED_EXIT(Write_SLAVE_profile_position_speed_set(nodeId,0));
  FAILED_EXIT(Write_SLAVE_control_word(nodeId,CONTROL_WORD_SHUTDOWN));
  FAILED_EXIT(Write_SLAVE_control_word(nodeId,CONTROL_WORD_SWITCH_ON));
	FAILED_EXIT(Write_SLAVE_control_word(nodeId,CONTROL_WORD_ENABLE_OPERATION));

  return 0x00;
}
/**
  * @brief  ���Ƶ��ʹ�ܲ�ѡ��岹λ��ģʽ
  * @param  nodeId:�ڵ�ID
  * @retval �ɹ�����0X00,ʧ�ܷ���0XFF
  * @note   None
*/
static UNS8 motor_on_interpolated_position(UNS8 nodeId)
{
  FAILED_EXIT(Write_SLAVE_Interpolation_time_period(nodeId));
  FAILED_EXIT(Write_SLAVE_Modes_of_operation(nodeId,INTERPOLATED_POSITION_MODE));
  FAILED_EXIT(Write_SLAVE_control_word(nodeId,CONTROL_WORD_SHUTDOWN));
  FAILED_EXIT(Write_SLAVE_control_word(nodeId,CONTROL_WORD_SWITCH_ON));
  /*State Transition 1: NO IP-MODE SELECTED => IP-MODE INACTIVE
  Event: Enter in the state OPERATION ENABLE with controlword and select ip 
  mode with modes of operation*/
//FAILED_EXIT(Write_SLAVE_control_word(SERVO_NODEID,CONTROL_WORD_ENABLE_OPERATION));
  return 0X00;
}
/**
  * @brief  ���Ƶ��ʹ�ܲ�ѡ��ԭ�㸴λģʽ
  * @param  offest:ԭ��ƫ��ֵ ��λ:PUU [ע��:ֻ�ǰ�ԭ��ƫ��ֵ�㵱Ϊ0����㣬�������˶���0����㴦]
  * @param  method:��ԭ��ʽ   ��Χ:0 ~ 35
  * @param  switch_speed:Ѱ��ԭ�㿪���ٶ� �趨��Χ 0.1 ~ 200 Ĭ��ֵ 10  ��λrpm ����:С�����һλ
  * @param  zero_speed:Ѱ�� Z�����ٶ�     �趨��Χ 0.1 ~ 50  Ĭ��ֵ 2   ��λrpm ����:С�����һλ
  * @param  nodeId:�ڵ�ID
  * @retval �ɹ�����0X00,ʧ�ܷ���0XFF
  * @note   None
*/
static UNS8 motor_on_homing_mode(int32_t offset,uint8_t method,float switch_speed,float zero_speed,UNS8 nodeId)
{
  FAILED_EXIT(Write_SLAVE_Modes_of_operation(nodeId,HOMING_MODE));
  FAILED_EXIT(Write_SLAVE_Homing_set(nodeId,offset,method,switch_speed,zero_speed));
  FAILED_EXIT(Write_SLAVE_control_word(nodeId,CONTROL_WORD_SHUTDOWN));
  FAILED_EXIT(Write_SLAVE_control_word(nodeId,CONTROL_WORD_SWITCH_ON));
	FAILED_EXIT(Write_SLAVE_control_word(nodeId,CONTROL_WORD_ENABLE_OPERATION));

  return 0x00;
}
/**
  * @brief  ���Ƶ��ʹ�ܲ�ѡ���ٶȹ滮ģʽ
  * @param  nodeId:�ڵ�ID
  * @retval �ɹ�����0X00,ʧ�ܷ���0XFF
  * @note   None
            2. �趨����ʱ��б�� OD 6083h�� 
            3. �趨����ʱ��б�� OD 6084h��
            606Fh�����ٶ�׼λ (Velocity threshold) 
            �˶����趨���ٶ��źŵ������Χ�����������ת�ٶ� (����ֵ) ���ڴ��趨ֵʱ��DO: 0x03(ZSPD) ����� 1��
Name          Value                   Description
rfg enable      0     Velocity reference value is controlled in any other (manufacturer specific) way, e.g. by a test function generator or manufacturer specific halt function.
                1     Velocity reference value accords to ramp output value. 
rfg unlock      0     Ramp output value is locked to current output value.
                1     Ramp output value follows ramp input value.
rfg use ref     0     Ramp input value is set to zero.
                1     Ramp input value accords to ramp reference.*/
static UNS8 motor_on_profile_velocity(UNS8 nodeId)
{
  Target_velocity = 0;
  SYNC_DELAY;
  FAILED_EXIT(Write_SLAVE_Modes_of_operation(nodeId,PROFILE_VELOCITY_MODE));
  FAILED_EXIT(Write_SLAVE_profile_position_speed_set(nodeId,0));
  FAILED_EXIT(Write_SLAVE_control_word(nodeId,CONTROL_WORD_SHUTDOWN));
  FAILED_EXIT(Write_SLAVE_control_word(nodeId,CONTROL_WORD_SWITCH_ON));
	FAILED_EXIT(Write_SLAVE_control_word(nodeId,CONTROL_WORD_ENABLE_OPERATION));
  
  return 0x00;
}
/******************************�˶�ģʽ��������******************************************************************/
/**
  * @brief  ���Ƶ����λ�ù滮ģʽ�˶�
  * @param  position:   λ��    ��λPUU �������
  * @param  speed:      �ٶ�    ��λRPM
  * @param  abs_rel:    �˶�ģʽ�� ��Ϊ0�������˶�ģʽ����Ϊ1������˶�ģʽ
  * @param  immediately:����������Чָ� ��Ϊ 0���ر�����������Чָ�� 1��������Ч����δ����Ŀ��λ��Ҳ��ִ���´��˶�
  * @param  nodeId:�ڵ�ID
  * @retval �ɹ�����0X00,ʧ�ܷ���0XFF
  * @note   �����ظ��˺����������Ƶ���˶���ͬλ�á� ��һod 0x2124����S�ͼӼ���
  ����ٶ�����      607Fh Ĭ��ֵ 3000rpm
  ����������      607Dh Ĭ��ֵ 2147483647
  ����������      607Dh Ĭ��ֵ -2147483647
  ���ٶ�ʱ��б��    6083h Ĭ��ֵ 200ms [0    rpm���ٵ� 3000 rpm����Ҫ��ʱ��]
  ���ٶ�ʱ��б��    6084h Ĭ��ֵ 200ms [3000 rpm���ٵ� 0    rpm����Ҫ��ʱ��]
  ��ͣ����ʱ��б��  6085h Ĭ��ֵ 200ms [3000 rpm���ٵ� 0    rpm����Ҫ��ʱ��]
  ��߼��ٶ�        60C5h Ĭ��ֵ 1  ms [0    rpm���ٵ� 3000 rpm����Ҫ��ʱ��]
  ��߼��ٶ�        60C6h Ĭ��ֵ 1  ms [3000 rpm���ٵ� 0    rpm����Ҫ��ʱ��]

  ������OD 607Ah �����λ�� (OD 6064h) ֮������ֵС�ڴ˶���ʱ��
  ��ʱ��ά�ִ��� OD 6068h (λ�õ��ﷶΧʱ��)��״̬λStatusword 6041h�� Bit10Ŀ�굽�Ｔ�����
  λ�õ��ﷶΧ      6067H:Ĭ��ֵ 100PUU
  λ�õ��ﷶΧʱ��  6068h:Ĭ��ֵ 0  ms

  ��λ����� (60F4h) �������趨��Χʱ���ŷ������쾯 AL009λ�������� 
  λ����������  6065h:Ĭ��ֵ50331648PUU //50331648 / 16777216 = 3
  */
static UNS8 motor_profile_position(int32_t position,uint32_t speed,bool abs_rel,bool immediately,UNS8 nodeId)
{
  UNS16 value = 0;
  if(Modes_of_operation != PROFILE_POSITION_MODE)
  {
    LOG_W("Motion mode selection error, the current motion mode is %d",Modes_of_operation);
    return 0XFF;
  }

  Target_position  = position;
  SYNC_DELAY;
  FAILED_EXIT(Write_SLAVE_profile_position_speed_set(SERVO_NODEID,speed));
  //�������������Ե��������˱����Ƚ� Bit 4��Ϊ off
  FAILED_EXIT(Write_SLAVE_control_word(nodeId,CONTROL_WORD_ENABLE_OPERATION));

  if(immediately == false)//����Ϊ�����ģʽ����������Ч
  {
    value = CONTROL_WORD_ENABLE_OPERATION | NEW_SET_POINT & ~CHANGE_SET_IMMEDIATELY;//�������������Ե������Bit 4��Ϊ������ on�� 
  }
  else//����Ϊ�����ģʽ��������Ч
  {
    value = CONTROL_WORD_ENABLE_OPERATION | NEW_SET_POINT |  CHANGE_SET_IMMEDIATELY;//�������������Ե������Bit 4��Ϊ������ on�� 
  }

  if(abs_rel == false)//����Ϊ�����˶�
  {
    value &= ~ABS_REL;
  }
  else//����Ϊ����˶�
  {
    value |=  ABS_REL;
  }

  FAILED_EXIT(Write_SLAVE_control_word(nodeId,value));

  return 0X00;
}
/**
  * @brief  ���Ƶ���Բ岹λ��ģʽ�˶�
  * @param  nodeId:�ڵ�ID
  * @retval �ɹ�����0X00,ʧ�ܷ���0XFF.
  * @note   None
*/
static UNS8 motor_interpolation_position (UNS8 nodeId)
{
  if(Modes_of_operation != INTERPOLATED_POSITION_MODE)
  {
    LOG_W("Motion mode selection error, the current motion mode is %d",Modes_of_operation);
    return 0XFF;
  }
  /* State Transition 3: IP-MODE INACTIVE => IP-MODE ACTIVE
  Event: Set bit enable ip mode (bit4) of the controlword while in ip mode and 
  OPERATION ENABLE*/
  FAILED_EXIT(Write_SLAVE_control_word(nodeId,CONTROL_WORD_ENABLE_OPERATION | ENABLE_IP_MODE));//Bit 4���� on�� 

  for(uint16_t i = 0; i < 1000;i++)
  {
    Interpolation_data_record_Parameter1_of_ip_function += 100;
    rt_thread_mdelay(1);
  }
  return 0X00;
}
/**
  * @brief  ���Ƶ������ԭ�㸴��ģʽ
  * @param  zero_flag��0�����践��0��λ�á� zero_flag��1������0��λ��
  * @param  nodeId:�ڵ�ID
  * @retval �ɹ�����0X00,ʧ�ܷ���0XFF.
  * @note   None
*/
static UNS8 motor_homing_mode (bool zero_flag,UNS8 nodeId)
{
  if(Modes_of_operation != HOMING_MODE)
  {
    LOG_W("Motion mode selection error, the current motion mode is %d",Modes_of_operation);
    return 0xFF;
  }
  //�������������Ե��������˱����Ƚ� Bit 4��Ϊ off
  FAILED_EXIT(Write_SLAVE_control_word(nodeId,CONTROL_WORD_ENABLE_OPERATION));
  FAILED_EXIT(Write_SLAVE_control_word(nodeId,CONTROL_WORD_ENABLE_OPERATION | NEW_SET_POINT));//�������������Ե������Bit 4��Ϊ������ on�� 
  LOG_I("Motor runing homing");
  if(block_query_BIT_change(&Statusword,HOMING_ATTAINED,5000,1) != 0x00)
  {
    LOG_W("Motor runing homing time out");
    return 0XFF;
  }
  else
  {
    LOG_I("Motor return to home is complete");
  }
  //�ص����
  if(zero_flag == true && Home_offset != 0)
  {
    LOG_I("The motor is returning to zero");
    motor_on_profile_position(nodeId);
    FAILED_EXIT(motor_profile_position(0,60,0,0,nodeId));

    if(block_query_BIT_change(&Statusword,TARGET_REACHED,5000,1) != 0x00)
    {
      LOG_W("Motor runing zero time out");
    }
    else
    {
      LOG_I("Motor return to zero is complete");
    }
  }
  
  return 0;
}
/**
  * @brief  ���Ƶ�����ٶȹ滮ģʽ�˶�
  * @param  speed:      �ٶ�    ��λRPM
  * @param  nodeId:�ڵ�ID
  * @retval �ɹ�����0X00,ʧ�ܷ���0XFF
  * @note   �����ظ��˺����������Ƶ���˶���ͬλ�á� ��һod 0x2124����S�ͼӼ���
  */
static UNS8 motor_profile_velocity(uint32_t speed,UNS8 nodeId)
{
  UNS16 value = 0;
  if(Modes_of_operation != PROFILE_VELOCITY_MODE)
  {
    LOG_W("Motion mode selection error, the current motion mode is %d",Modes_of_operation);
    return 0XFF;
  }

  Target_velocity  = speed * 10;//Target_velocity ��λΪ0.1 rpm,��Ҫ*10

  return 0X00;
}
/******************************�˶��رռ���ѯ����******************************************************************/
/**
  * @brief  ���Ƶ���ر�.
  * @param  nodeId:�ڵ�ID
  * @retval �ɹ�����0X00,ʧ�ܷ���0XFF.
  * @note   ����������ͣ
  ��ͣ����ʱ��б��  6085h Ĭ��ֵ 200ms [3000 rpm���ٵ� 0    rpm����Ҫ��ʱ��]
*/
static UNS8 motor_off(UNS8 nodeId)
{
  FAILED_EXIT(Write_SLAVE_control_word(nodeId,CONTROL_WORD_SHUTDOWN));
  FAILED_EXIT(Write_SLAVE_control_word(nodeId,CONTROL_WORD_DISABLE_VOLTAGE));
  
  return 0x00;
}
/**
  * @brief  ��ѯ���״̬.
  * @param  None.
  * @retval None.
  * @note   
  ������
  ״̬��
  �����������
  ��ǰλ��
  ��ǰ�ٶ�

  Alarm_code��AL180 �����쳣
              AL3E3 ͨѶͬ���źų�ʱ[IPģʽδ�յ�����]
              AL022 ����·��Դ�쳣[����������]
              AL009 λ�ÿ��������� [��Ҫ�ҷ�λ������]
*/
static void motor_state(void)
{
  LOG_I("Mode operation:%d",Modes_of_operation);
	LOG_I("ControlWord 0x%0X", Controlword);
  LOG_I("StatusWord 0x%0X", Statusword);
  
  if(CANOPEN_GET_BIT(Statusword , FAULT))
    LOG_E("motor fault!");
  if(CANOPEN_GET_BIT(Statusword , WARNING))
    LOG_W("motor warning!");
  if(CANOPEN_GET_BIT(Statusword , FOLLOWING_ERROR))
  {
    if(Modes_of_operation == PROFILE_POSITION_MODE)
      LOG_E("motor following error!");
  }
  if(CANOPEN_GET_BIT(Statusword , POSITIVE_LIMIT))
    LOG_W("motor touch  positive limit!");
  if(!CANOPEN_GET_BIT(Statusword , TARGET_REACHED))
    LOG_W("The node did not receive the target arrival command!");
  
  if(CANOPEN_GET_BIT(Statusword , 13))
  {
    if(Modes_of_operation == PROFILE_POSITION_MODE)
    {
      LOG_E("motor following error!");
    }
    else if(Modes_of_operation == HOMING_MODE)
    {
      LOG_E("motor Homing error occurred!");
    }
  }

	LOG_I("current position %d PUU", Position_actual_value);  //ע��Ϊ0X6064��0X6063Ϊ����ʽλ��
	LOG_I("current speed %.1f RPM", Velocity_actual_value / 10.0f);//ע�ⵥλΪ0.1rpm
}
#ifdef RT_USING_MSH
/**
  * @brief  MSH���Ƶ���˶�
  * @param  None
  * @retval None
  * @note   None
*/
static void cmd_motor(uint8_t argc, char **argv) 
{
#define MOTOR_CMD_ON                    0
#define MOTOR_CMD_OFF                   1
#define MOTOR_CMD_STATE                 2
#define MOTOR_CMD_PPMOVE                3
#define MOTOR_CMD_IPMOVE                4
#define MOTOR_CMD_HMMOVE                5
#define MOTOR_CMD_PVMOVE                6
  size_t i = 0;

  const char* help_info[] =
    {
            [MOTOR_CMD_ON]             = "motor on pp/ip/hm/pv                                      - Enable motor control",
            [MOTOR_CMD_OFF]            = "motor off                                                 - Disenable motor control",
            [MOTOR_CMD_STATE]          = "motor state                                               - Display motor status.",
            [MOTOR_CMD_PPMOVE]         = "motor pp_move                                             - Setting motor pp motion position",
            [MOTOR_CMD_IPMOVE]         = "motor ip_move                                             - Setting motor ip motion position.",
            [MOTOR_CMD_HMMOVE]         = "motor hm_move                                             - Setting motor into homing mode.",
            [MOTOR_CMD_PVMOVE]         = "motor pv_move                                             - Setting motor pv motion position.",
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

        if (!strcmp(operator, "on"))
        {
          if(argc <= 2) 
          {
              rt_kprintf("Usage: motor [mode] <nodeid> -pp ip hm pv pt\n");
              return;
          }
          if (!strcmp(argv[2], "pp"))
          {
            UNS8 nodeId = SERVO_NODEID;
            if(argc > 3) 
            {
              nodeId = atoi(argv[3]);
            }   
            motor_on_profile_position(nodeId);
          }
          else if (!strcmp(argv[2], "ip"))
          {
            UNS8 nodeId = SERVO_NODEID;
            if(argc > 3) 
            {
              nodeId = atoi(argv[3]);
            }   
            motor_on_interpolated_position(nodeId);
          }
          else if (!strcmp(argv[2], "hm"))
          {
            int32_t offset = 0;
            uint8_t method = 34;//34 ��ʼ������Ѱ��Z���� 33 ��ʼ������Ѱ��Z���� 
            float switch_speed = 100;//��λRPM
            float zero_speed = 20;//��λRPM
            UNS8 nodeId = SERVO_NODEID;

            if(argc <= 3) 
            {
                rt_kprintf("Usage: motor on hm [offset] <switch_speed> <zero_speed> <method> <nodeId>\n");
                return;
            }
            offset = atoi(argv[3]);
            
            if(argc > 4) 
            {
              switch_speed = atoi(argv[4]);
            }
            if(argc > 5) 
            {zero_speed = atoi(argv[5]);
            }
            if(argc > 6) 
            {
              method = atoi(argv[6]);
            }
            
            if(argc > 7) 
            {
              nodeId = atoi(argv[7]);
            }   
            rt_kprintf("homing mode set offset: %d, method: %d\n", offset, method);
            rt_kprintf("switch_speed: %.1f, zero_speed: %.1f\n", switch_speed, zero_speed);
            motor_on_homing_mode(offset,method,switch_speed,zero_speed,nodeId);
          }
          else if (!strcmp(argv[2], "pv"))
          {
            UNS8 nodeId = SERVO_NODEID;
            if(argc > 3) 
            {
              nodeId = atoi(argv[3]);
            }   
            motor_on_profile_velocity(nodeId);
          }
          else
          {
              rt_kprintf("Usage: motor [mode] <nodeId> -pp ip hm pv pt\n");
              return;
          }
        }
        else if (!strcmp(operator, "off"))
        {
            UNS8 nodeId = SERVO_NODEID;
            if(argc > 3) 
            {
              nodeId = atoi(argv[3]);
            }   
            motor_off(nodeId);
        }
        else if (!strcmp(operator, "state"))
        {
            motor_state();
        }
        else if (!strcmp(operator, "pp_move"))
        {
            int32_t position;
            int32_t speed = 60;//RPM:60
            bool immediately = false;
            bool abs_rel     = false;
            UNS8 nodeId = SERVO_NODEID;
            if(argc <= 2) 
            {
                rt_kprintf("Usage: motor pp_move [position] <speed> <0:abs/1:rel> <immediately> <nodeId>\n");
                return;
            }

            position = atoi(argv[2]);
            if(argc > 3) 
            {
                speed = atoi(argv[3]);
            }
            if(argc > 4) 
            {
                abs_rel = atoi(argv[4]);
            }
            if(argc > 5) 
            {
                immediately = atoi(argv[5]);
            }
            if(argc > 6) 
            {
              nodeId = atoi(argv[6]);
            }  
            rt_kprintf("move to position: %d, speed: %d\n", position, speed);
            rt_kprintf("abs_rel: %d, immediately: %d\n", abs_rel, immediately);
            motor_profile_position(position,speed,abs_rel,immediately,nodeId);
        }
        else if (!strcmp(operator, "ip_move"))
        {
          UNS8 nodeId = SERVO_NODEID;
          if(argc > 2) 
          {
            nodeId = atoi(argv[2]);
          }  
          motor_interpolation_position(nodeId);
        }
        else if (!strcmp(operator, "hm_move"))
        {
            bool zero_flag = false;
            UNS16 nodeId = SERVO_NODEID;
            if(argc <= 2) 
            {
                rt_kprintf("Usage: motor hm_move [zero] <nodeId> --homing move zero:1,run zero postion\n");
                return;
            }
            zero_flag = atoi(argv[2]);
            if(argc > 3) 
            {
                nodeId = atoi(argv[3]);
            }
            motor_homing_mode(zero_flag,nodeId);
        }
        else if (!strcmp(operator, "pv_move"))
        {
          int32_t speed = 0;//RPM:0
          UNS8 nodeId = SERVO_NODEID;
          if(argc <= 2) 
          {
            rt_kprintf("Usage: motor pv_move [speed] <nodeId>\n");
            return;
          }
          speed = atoi(argv[2]);
          if(argc > 3) 
          {
              nodeId = atoi(argv[3]);
          }
          motor_profile_velocity(speed,nodeId);
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
MSH_CMD_EXPORT_ALIAS(cmd_motor,motor,motor command.);
static void cmd_CONTROL_WORD(uint8_t argc, char **argv)
{
#define CW_CMD_ON                  0
#define CW_CMD_EN                  1
#define CW_CMD_DIS                 2
#define CW_CMD_RESET               3
    size_t i = 0;

    const char* help_info[] =
    {
            [CW_CMD_ON]             = "canoepn_cw [nodeID] on                       --SWITCH ON",
            [CW_CMD_EN]             = "canoepn_cw [nodeID] en                       --ENABLE OPERATION",
            [CW_CMD_DIS]            = "canoepn_cw [nodeID] dis                      --DISABLE VOLTAGE",
            [CW_CMD_RESET]          = "canoepn_cw [nodeID] reset                    --FAULT_RESET",
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
        UNS8 nodeId = SERVO_NODEID;
        if(argc > 1) 
        {
            nodeId = atoi(argv[1]);
        }
        const char *operator = argv[2];
        if (!strcmp(operator, "shutdown"))
        {
            Write_SLAVE_control_word(nodeId,CONTROL_WORD_SHUTDOWN);
        }
        else if(!strcmp(operator, "on"))
        {
            Write_SLAVE_control_word(nodeId,CONTROL_WORD_SWITCH_ON);
        }
        else if(!strcmp(operator, "en"))
        {
            Write_SLAVE_control_word(nodeId,CONTROL_WORD_ENABLE_OPERATION);
        }
        else if(!strcmp(operator, "dis"))
        {
            Write_SLAVE_control_word(nodeId,CONTROL_WORD_DISABLE_VOLTAGE);
        }
        else if(!strcmp(operator, "reset"))
        {
            Write_SLAVE_control_word(nodeId,FAULT_RESET);
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
MSH_CMD_EXPORT_ALIAS(cmd_CONTROL_WORD,canopen_cw,canopen control word.);
#endif