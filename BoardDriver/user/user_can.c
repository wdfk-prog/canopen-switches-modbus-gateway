/**
 * @file user_can.c
 * @brief CAN调试使用
 * @author HLY (1425075683@qq.com)
 * @version 1.0
 * @date 2022-11-17
 * @copyright Copyright (c) 2022
 * @attention 
 * @par 修改日志:
 * Date       Version Author  Description
 * 2022-11-17 1.0     HLY     first version
 */
/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
#include <rtthread.h>
#include "rtdevice.h"

#define CAN_DEV_NAME        "can1"      /* CAN 设备名称 */
/*ulog include*/
#define LOG_TAG             CAN_DEV_NAME
#define LOG_LVL              DBG_INFO
#include <ulog.h>
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* 监控线程配置 */
#define THREAD_PRIORITY      12  //线程优先级
#define THREAD_TIMESLICE     10  //线程时间片
#define THREAD_STACK_SIZE    1024//栈大小
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static struct rt_semaphore rx_sem;     /* 用于接收消息的信号量 */
static rt_device_t can_dev;            /* CAN 设备句柄 */
/**
  * @brief  接收数据回调函数
  * @param  None
  * @retval None
  * @note   
*/
static rt_err_t can_rx_call(rt_device_t dev, rt_size_t size)
{
    /* CAN 接收到数据后产生中断，调用此回调函数，然后发送接收信号量 */
    rt_sem_release(&rx_sem);

    return RT_EOK;
}
/**
  * @brief  can接收线程
  * @param  None
  * @retval None
  * @note   
*/
static void can_rx_thread(void *parameter)
{
    int i;
    rt_err_t res;
    struct rt_can_msg rxmsg = {0};

    /* 设置接收回调函数 */
    rt_device_set_rx_indicate(can_dev, can_rx_call);

#ifdef RT_CAN_USING_HDR
    struct rt_can_filter_item items[5] =
    {
        RT_CAN_FILTER_ITEM_INIT(0x100, 0, 0, 0, 0x700, RT_NULL, RT_NULL), /* std,match ID:0x100~0x1ff，hdr 为 - 1，设置默认过滤表 */
        RT_CAN_FILTER_ITEM_INIT(0x300, 0, 0, 0, 0x700, RT_NULL, RT_NULL), /* std,match ID:0x300~0x3ff，hdr 为 - 1 */
        RT_CAN_FILTER_ITEM_INIT(0x211, 0, 0, 0, 0x7ff, RT_NULL, RT_NULL), /* std,match ID:0x211，hdr 为 - 1 */
        RT_CAN_FILTER_STD_INIT(0x486, RT_NULL, RT_NULL),                  /* std,match ID:0x486，hdr 为 - 1 */
        {0x555, 0, 0, 0, 0x7ff, 7,}                                       /* std,match ID:0x555，hdr 为 7，指定设置 7 号过滤表 */
    };
    struct rt_can_filter_config cfg = {5, 1, items}; /* 一共有 5 个过滤表 */
    /* 设置硬件过滤表 */
    res = rt_device_control(can_dev, RT_CAN_CMD_SET_FILTER, &cfg);
    RT_ASSERT(res == RT_EOK);
#endif /*RT_CAN_USING_HDR*/

    while (1)
    {
        /* hdr 值为 - 1，表示直接从 uselist 链表读取数据 */
        rxmsg.hdr_index = -1;
        /* 阻塞等待接收信号量 */
        rt_sem_take(&rx_sem, RT_WAITING_FOREVER);
        /* 从 CAN 读取一帧数据 */
        rt_device_read(can_dev, 0, &rxmsg, sizeof(rxmsg));
        /* 打印数据 ID 及内容 */
        rt_kprintf("\n");
        rt_kprintf("ID:%x:", rxmsg.id);
        for (i = 0; i < 8; i++)
        {
            rt_kprintf("%2x", rxmsg.data[i]);
        }

        rt_kprintf("\n");
    }
}
/**
  * @brief  CAN初始化函数
  * @param  None
  * @retval None
  * @note   
*/
int can1_init(void)
{
    struct rt_can_msg msg = {0};
    rt_err_t res;
    rt_size_t  size;
    /* 查找 CAN 设备 */
    can_dev = rt_device_find(CAN_DEV_NAME);
    if (!can_dev)
    {
        LOG_E("find %s failed", CAN_DEV_NAME);
        return RT_ERROR;
    }

    /* 初始化 CAN 接收信号量 */
    rt_sem_init(&rx_sem, "rx_sem", 0, RT_IPC_FLAG_FIFO);

    /* 以中断接收及发送方式打开 CAN 设备 */
    res = rt_device_open(can_dev, RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX);
    RT_ASSERT(res == RT_EOK);
    /* 创建线程 放在最后 避免定时器还为初始化*/
    rt_thread_t thread = rt_thread_create( "can_rx",           /* 线程名字 */
                                           can_rx_thread,  /* 线程入口函数 */
                                           RT_NULL,               /* 线程入口函数参数 */
                                           THREAD_STACK_SIZE,     /* 线程栈大小 */
                                           THREAD_PRIORITY,       /* 线程的优先级 */
                                           THREAD_TIMESLICE);     /* 线程时间片 */
    /* 创建成功则启动线程 */
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
    else
    {
        LOG_E("Monitor thread created failed.");
        res =  RT_ERROR;
    }

    msg.id = 0x78;              /* ID 为 0x78 */
    msg.ide = RT_CAN_STDID;     /* 标准格式 */
    msg.rtr = RT_CAN_DTR;       /* 数据帧 */
    msg.len = 8;                /* 数据长度为 8 */
    /* 待发送的 8 字节数据 */
    msg.data[0] = 0x00;
    msg.data[1] = 0x11;
    msg.data[2] = 0x22;
    msg.data[3] = 0x33;
    msg.data[4] = 0x44;
    msg.data[5] = 0x55;
    msg.data[6] = 0x66;
    msg.data[7] = 0x77;
//    /* 发送一帧 CAN 数据 */
//    size = rt_device_write(can_dev, 0, &msg, sizeof(msg));
//    if (size == 0)
//    {
//        LOG_E("can dev write data failed!");
//    }

    return res;
}
//INIT_APP_EXPORT(can1_init);
/**
  * @brief  CAN发送测试
  * @param  None
  * @retval None
  * @note   
*/
void can_send_test(void)
{
    struct rt_can_msg msg = {0};
    rt_size_t  size;
    static rt_uint8_t num = 0;
    /* 查找 CAN 设备 */
    can_dev = rt_device_find(CAN_DEV_NAME);
    if (!can_dev)
    {
        LOG_E("find %s failed", CAN_DEV_NAME);
    }
    msg.id = 0x78;              /* ID 为 0x78 */
    msg.ide = RT_CAN_STDID;     /* 标准格式 */
    msg.rtr = RT_CAN_DTR;       /* 数据帧 */
    msg.len = 8;                /* 数据长度为 8 */

    /* 待发送的 8 字节数据 */
    msg.data[0] = 0x00;
    msg.data[1] = num++;
    msg.data[2] = 0x22;
    msg.data[3] = 0x33;
    msg.data[4] = num++;
    msg.data[5] = 0x55;
    msg.data[6] = 0x66;
    msg.data[7] = 0x77;
    /* 发送一帧 CAN 数据 */
    size = rt_device_write(can_dev, 0, &msg, sizeof(msg));
    if (size == 0)
    {
        rt_kprintf("can dev write data failed!\n");
    }
}
/* 导出到 msh 命令列表中 */
MSH_CMD_EXPORT(can_send_test, can send test);