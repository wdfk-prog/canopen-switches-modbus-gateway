/**
  ******************************************************************************
  * @file    mb_key.c
  * @brief   mb��������v1.0
  * @date    2021.01.10
  ******************************************************************************
  * @attention  ��mb��Ȧ����Ϊ����
  mbkey������ʽ��1.��������һ���̣߳���ѯ��ʽѭ�����С� ���򵥣��ױ�д��
                 2.Ϊÿ����������һ���̣߳�mbkey�����ź������߳��ڻָ�����
                 ��ά�����ѣ����������̹߳��ࡿ
  * @author hly
  ******************************************************************************
  */
/* includes ------------------------------------------------------------------*/

/* private includes ----------------------------------------------------------*/
#include "modbus_slave_common.h"
/*ulog include*/
#define log_tag              "mb key"
#define log_lvl              dbg_info
#include <ulog.h>
/* private typedef -----------------------------------------------------------*/
/** 
  * @brief  ����ע���
  */
typedef enum
{
	KEY_MOTOR_ENABLE,                //���ʹ��
  MBKEY_NUM,// ����Ҫ�еļ�¼��ť���������������
}mbkey_list;
/** 
  * @brief  ����״̬��������״̬
  */ 
typedef enum
{
  MBKEY_ENABLE,   // ʹ�ܣ���ֵΪ1
	MBKEY_DISABLE,  // ʧ�ܣ���ֵΪ0
	MBKEY_PRESS,    // ��������
	MBKEY_RAISE,    // ����̧��
}mbkey_status;
/** 
  * @brief  ��Ȧ�Ĵ���״̬
  */  
typedef enum 
{
  LOW_LEVEL = 0u, 
  HIGH_LEVEL = !LOW_LEVEL
} bits_status;
/** 
  * @brief  �������α�־
  */  
typedef enum 
{
  DISABLE = 0u, 
  ENABLE = !DISABLE
} mbkey_enable_status;
/** @defgroup gpio_pull_define gpio pull define
 * @brief gpio pull-up or pull-down activation
 * @{
 */  
typedef enum
{
  PULLUP       = 0x00000001u,   //��һ�����¼�
  PULLDOWN     = 0x00000002u,   //��������¼�
}bits_activation;
/** 
  * @brief  ״̬����ʼ������
  */  
typedef struct
{
	uint32_t gpio_pull;		//�����ƽ
	uint16_t index;	      //����
	uint16_t sub_index;	  //������
	uint8_t  key_nox;
}config;
/** 
  * @brief  ״̬����
  */
typedef struct
{
    mbkey_enable_status 	shield; 		//�������Σ�disable(0):���Σ�enable(1):������
    uint8_t             	timecount;  //������������
    bits_status 	        flag;       //��־�������±�־
    bits_status 	        down_level; //����ʱ������ioʵ�ʵĵ�ƽ
    mbkey_status          key_status; //����״̬
    mbkey_status          key_event;  //�����¼�
    bits_status           (*read_pin)(config bits);//��io��ƽ����
}components;
/** 
  * @brief  ������
  */
typedef struct
{
	config 				      board; // �̳г�ʼ������
	components 	        status; // �̳�״̬������
}mbkey;
/* private define ------------------------------------------------------------*/
/* �߳����� */
#define THREAD_PRIORITY      4//�߳����ȼ�
#define THREAD_TIMESLICE     10//�߳�ʱ��Ƭ
#define THREAD_STACK_SIZE    1024//ջ��С
/* private macro -------------------------------------------------------------*/

/* private variables ---------------------------------------------------------*/
mbkey mbkey_buf[MBKEY_NUM];	// ������������
/* private function prototypes -----------------------------------------------*/
static void key_motor_enable(mbkey_status *event)
{
  switch(*event)
  {
    case MBKEY_ENABLE:  //���´����¼�
    break;
    case MBKEY_DISABLE: //�ɿ������¼�
    break;
    case MBKEY_PRESS:   //�ɿ��������¼�
    break;
    case MBKEY_RAISE:   //���µ��ɿ��¼�
    break;    
  }
}
/**************************״̬��**********************************************/
/** 
  * @brief  ��ȡio��ƽ�ĺ���
  ������ȡ����
  */  
static bits_status readpin(config bits)
{
  return (bits_status)modbus_bits_get(bits.index,bits.sub_index);
}
/**
  * @brief  ��ȡ����ֵ
  * @param  none.
  * @retval none.
  * @note   ����ʵ�ʰ��°�ť�ĵ�ƽȥ�������������Ľ��
*/
static void get_level(void)
{
    for(uint8_t i = 0;i < MBKEY_NUM;i++)
    {
        if(mbkey_buf[i].status.shield == DISABLE)	//��������򲻽��а���ɨ��
            continue;
        if(mbkey_buf[i].status.read_pin(mbkey_buf[i].board) == mbkey_buf[i].status.down_level)
            mbkey_buf[i].status.flag = LOW_LEVEL;
        else
            mbkey_buf[i].status.flag = HIGH_LEVEL;
    }
}
/**
  * @brief  ������������
  * @param  mbkey_init
  * @retval none.
  * @note   ������������
*/
static void creat_key(config* init)
{
  for(uint8_t i = 0;i < MBKEY_NUM;i++)
	{
		mbkey_buf[i].board = init[i]; // mbkey_buf��ť����ĳ�ʼ�����Ը�ֵ

		mbkey_buf[i].board.key_nox = i;
		// ��ʼ����ť�����״̬������
		mbkey_buf[i].status.shield = ENABLE;
		mbkey_buf[i].status.timecount = 0;	
		mbkey_buf[i].status.flag = LOW_LEVEL;
    
		if(mbkey_buf[i].board.gpio_pull == PULLUP) // ����ģʽ���и�ֵ
			mbkey_buf[i].status.down_level = LOW_LEVEL;
		else
			mbkey_buf[i].status.down_level = HIGH_LEVEL;
    
		mbkey_buf[i].status.key_status = 	MBKEY_DISABLE;
		mbkey_buf[i].status.key_event	= 	MBKEY_DISABLE;
		mbkey_buf[i].status.read_pin 	= 	readpin;	//��ֵ������ȡ����
	}
}
/**
  * @brief  ��ȡ����
  * @param  none.
  * @retval none.
  * @note   ״̬����״̬ת��
*/
static void read_status(void)
{
  get_level();
  for(uint8_t i = 0;i < MBKEY_NUM;i++)
  {
    switch(mbkey_buf[i].status.key_status)
    {
      //״̬0�������Ϳ�
      case MBKEY_DISABLE:
        if(mbkey_buf[i].status.flag == LOW_LEVEL)
        {
            mbkey_buf[i].status.key_status = MBKEY_DISABLE;  //ת��״̬3
            mbkey_buf[i].status.key_event  = MBKEY_DISABLE;  //���¼�
        }
        else
        {
          mbkey_buf[i].status.key_status = MBKEY_PRESS;        //ת��״̬1
          mbkey_buf[i].status.key_event 	= MBKEY_PRESS;        //�����¼�
        }
        break;
			//״̬1����������
      case MBKEY_ENABLE:
        if(mbkey_buf[i].status.flag == HIGH_LEVEL)
        {
            mbkey_buf[i].status.key_status = MBKEY_ENABLE;     //ת��״̬3
            mbkey_buf[i].status.key_event  = MBKEY_ENABLE;     //���¼�
        }
        else
        {
          mbkey_buf[i].status.key_status = MBKEY_RAISE;        //ת��״̬0
          mbkey_buf[i].status.key_event  = MBKEY_RAISE;        //�����¼�
        }
        break;
      //״̬2����������[�Ϳ�������]
      case MBKEY_PRESS:
        if(mbkey_buf[i].status.flag == HIGH_LEVEL)
        {
            mbkey_buf[i].status.key_status = MBKEY_ENABLE;     //ת��״̬3
            mbkey_buf[i].status.key_event  = MBKEY_ENABLE;     //���¼�
        }
        else
        {
            mbkey_buf[i].status.key_status = MBKEY_DISABLE;  //ת��״̬3
            mbkey_buf[i].status.key_event  = MBKEY_DISABLE;  //���¼�
        }
        break;
			//״̬1����������[���µ��Ϳ�]
			case MBKEY_RAISE:
				if(mbkey_buf[i].status.flag == LOW_LEVEL)          //�����ͷţ��˿ڸߵ�ƽ
        {
            mbkey_buf[i].status.key_status = MBKEY_DISABLE;  //ת��״̬3
            mbkey_buf[i].status.key_event  = MBKEY_DISABLE;  //���¼�
        }
				else
        {
            mbkey_buf[i].status.key_status = MBKEY_ENABLE;     //ת��״̬3
            mbkey_buf[i].status.key_event = MBKEY_ENABLE;      //���¼�
        }
        break;
    }
  }
}
/**
  * @brief  ����ָ������.
  * @param  none.
  * @retval none.
  * @note   
ע�⺯����˳�������Խ������
https://blog.csdn.net/feimeng116/article/details/107515317
*/
static void (*operation[MBKEY_NUM])(mbkey_status *event) = 
{
  key_motor_enable,
};
/**
  * @brief  ��������
  * @param  none.
  * @retval none.
  * @note   ���ڶ�ʱ��1msһ��
            ����ʼ��ֵ����mbģʽ������
            ɨ�谴���󣬶�ȡ����״̬���¼������ж�����
*/
void mbkey_handler(void *p)
{
	uint8_t i;
  while(1)
  {
    rt_thread_mdelay(1);
    read_status();
    for(i = 0;i < MBKEY_NUM;i++)
    {
        operation[i](&mbkey_buf[i].status.key_event);
    }
  }
}
/**
  * @brief  mbkey�������β���
  * @param  num:mbkey_listע�����ѡ��
  * @param  option: enable  �����á�
                    disable �����á�
  * @retval none.
  * @note   ���û�������
*/
void mbkey_shield_operate(uint8_t num,mbkey_enable_status option)
{
  mbkey_buf[num].status.shield       = option;
  mbkey_buf[num].status.key_event    = MBKEY_DISABLE;//�˳�ɲ���¼�
}
/**
  * @brief  io��ʼ����ʼ��
  * @param  none.
  * @retval none.
  * @note   ״̬����ʼ��
gpio_pullup����ʼ���ߣ��˿ڣ�λ��
*/
static int mbkey_init(void)
{ 
  rt_err_t ret = RT_EOK; 
  config init[MBKEY_NUM]=
  { 
    //�����ƽ ���� ������
    {PULLUP,    0,    1}, //���ʹ��
  };
  creat_key(init);// ���ð�����ʼ������
  /* ���� MODBUS�߳�*/
  rt_thread_t thread = rt_thread_create( "mb key",    /* �߳����� */
                                         mbkey_handler,/* �߳���ں��� */
                                         RT_NULL,       /* �߳���ں������� */
                                         THREAD_STACK_SIZE, /* �߳�ջ��С */
                                         THREAD_PRIORITY,   /* �̵߳����ȼ� */
                                         THREAD_TIMESLICE); /* �߳�ʱ��Ƭ */
  /* �����ɹ��������߳� */
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
INIT_APP_EXPORT(mbkey_init);