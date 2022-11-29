/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : modbus_savle1.c
  * @brief          : 
  * @date           : 2022.08.10
  ******************************************************************************
  * @attention  
  MB_SLAVE1 <-读写-> |           |  使用互斥量避免破坏
 使用互斥量避免破坏  | MB_Buffer |  <-读写-> MB_Handler
  MB_SLAVE2 <-读写-> |           |
  * @author HLY
  ******************************************************************************
  */
/* USER CODE END Header */
#include "mb.h"
#include "mbrtu.h"
#include "mbevent.h"
/* Private includes ----------------------------------------------------------*/
#include "sys.h"
#include <rtthread.h>
#include <drv_usart_v2.h>
#include "mb_handler.h"
#include "main.h"

#define UART_NAME       "uart1"      /* 串口设备名称 */
/*ulog include*/
#define LOG_TAG         UART_NAME
#define LOG_LVL         DBG_INFO
#include <ulog.h>
/* Private typedef -----------------------------------------------------------*/
/* 串口接收消息结构 */
struct rx_msg
{
    rt_device_t dev;
    rt_size_t size;
};
/* Private define ------------------------------------------------------------*/
/*端口定义*/
#define ucRTU_DCB	     (mbrtu1)
#define ucMB_RTU_PORT	 (mbrtuport1)
/*波特率设置*/
#define BAUD_RATE BAUD_RATE_115200
/*从机地址设置*/
#define SLAVE_ADDRESS 0X01
/*串口中断优先级设置*/
#define IRQ_PRIORITY  3
/* Private macro -------------------------------------------------------------*/
/* 线程配置 */
#define THREAD_PRIORITY      9//线程优先级
#define THREAD_TIMESLICE     10//线程时间片
#define THREAD_STACK_SIZE    1024//栈大小
/* Private variables ---------------------------------------------------------*/
static int serial_receive(uint8_t *buf, int bufsz);
/* 串口设备句柄 */
static rt_device_t serial = RT_NULL;
/* 消息队列控制块 */
static rt_mq_t rx_mq      = RT_NULL;
/* 指向互斥量的指针 */
rt_mutex_t mb_slave_mutex = RT_NULL;
/* MODBUS函数 */
static void MBPortSerialInit(ULONG ulBaudRate, eMBParity eParity);
static void MBPortTimersInit( USHORT usTim1Timerout50us );
static void MBPortSerialGetByte(UCHAR *pucByte);
static void MBPortSerialPutByte(UCHAR ucByte);
static void MBPortTimersEnable(USHORT Timerout50us);
static void MBPortTimersDisable(void);
static void MBPortSerialEnable(BOOL xRxEnable,BOOL xTxEnable);
static void EnterCriticalSection(void);
static void ExitCriticalSection(void);
/* MODBUS结构体 */
static _CONST MB_RTU_PORT mbrtuport1 = 
{
	MBPortSerialInit,
	MBPortTimersInit,
	MBPortSerialGetByte,
	MBPortSerialPutByte,
	MBPortTimersEnable,
	MBPortTimersDisable,
	MBPortSerialEnable,
	EnterCriticalSection,
	ExitCriticalSection,
};
/* MODBUS结构体指针 */
static MB_RTU_DCB mbrtu1;
/* Private function prototypes -----------------------------------------------*/
/**
 * This function will print a formatted string on system console.
 *
 * @param fmt is the format parameters.
 *
 * @return The number of characters actually written to buffer.
 */
static int kprintf(const char *fmt, ...)
{
    va_list args;
    rt_size_t length;
    static char rt_log_buf[RT_CONSOLEBUF_SIZE];

    va_start(args, fmt);
    /* the return value of vsnprintf is the number of bytes that would be
     * written to buffer had if the size of the buffer been sufficiently
     * large excluding the terminating null byte. If the output string
     * would be larger than the rt_log_buf, we have to adjust the output
     * length. */
    length = rt_vsnprintf(rt_log_buf, sizeof(rt_log_buf) - 1, fmt, args);
    if (length > RT_CONSOLEBUF_SIZE - 1)
        length = RT_CONSOLEBUF_SIZE - 1;

    rt_device_write(serial, 0, rt_log_buf, length);
    va_end(args);
    return length;
}
/**
  * @brief  AT指令函数
  * @param  None
  * @retval None
  * @note   发送AT，返回OK
            发送后会清空数组。没有发现影响
*/
static void AT(void)
{
  if(rt_strstr((char *)ucRTU_DCB.ucBuffer,"AT\r\n") != NULL)
  {
    rt_memset((void *)ucRTU_DCB.ucBuffer, 0, sizeof ucRTU_DCB.ucBuffer);
    kprintf("\n \\ | /\n");
    kprintf("- HLY -    Version F4_Laser_forklif V%s \n",VERSION);
    kprintf(" / | \\     build %s %s\n",__DATE__,__TIME__);
    kprintf("device is %s",UART_NAME);
  }
}
/**
  * @brief  MODBUS线程
  * @param  p:无用，不定义创建线程有警告
  * @retval None.
  * @note   None.
*/
static void modbus_thread(void* p)
{
  //初始化modbusRTU，从站地址为0x01,波特率为115200，无校验。
  eMBRTUInit(&ucRTU_DCB,&ucMB_RTU_PORT,SLAVE_ADDRESS,BAUD_RATE,MB_PAR_NONE); 
  eMBRTUStart(&ucRTU_DCB);
  //F4使能空闲中断不会立马进入中断函数进行状态转换，手动切换
  ucRTU_DCB.eRcvState = STATE_RX_IDLE;
  while(1)
  {
    struct rx_msg msg;
    rt_memset(&msg, 0, sizeof(msg));
    /* 从消息队列中读取消息 */
    rt_err_t result = rt_mq_recv(rx_mq, &msg, sizeof(msg), RT_WAITING_FOREVER);
    if (result == RT_EOK)
    {
        /* 从串口读取数据 */
        rt_uint32_t rx_length = rt_device_read(msg.dev, 0,(CHAR *)ucRTU_DCB.ucBuffer, msg.size);
        if(rx_length != msg.size)
        {
          LOG_W("Incorrect receive length");
          continue;
        }
        else
        {
          ucRTU_DCB.ucBufferCount = rx_length;              //写入读取大小
          xMBPortEventPost(&ucRTU_DCB,EV_FRAME_RECEIVED);   //改变事件状态为接收事件
          rt_mutex_take(mb_slave_mutex, RT_WAITING_FOREVER);//保存内存
          AT();
          Modbus_Handler();//读取
          eMBRTUPoll(&ucRTU_DCB);                           //读写内存
          Modbus_Handler();//写入
          HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
          rt_mutex_release(mb_slave_mutex);                 //允许操作内存
        }
    }
  }
}
/*******************************************串口收发处理函数*********************************************************************/
/**
 * @brief This function will set the reception indication callback function. This callback function
 *        is invoked when this device receives data.
 *
 * @param dev is the pointer of device driver structure.
 *
 * @param rx_ind is the indication callback function.
 *
 * @return RT_EOK
 */
static rt_err_t uart_input(rt_device_t dev, rt_size_t size)
{
    struct rx_msg msg;
    rt_err_t result;
    msg.dev = dev;
    msg.size = size;

    result = rt_mq_send(rx_mq, &msg, sizeof(msg));
    if (result == -RT_EFULL)
    {
        /* 消息队列满 */
        LOG_E("message queue full!");
    }
    return 1;
}
/**
  * @brief  串口接收函数.
  * @param  buf:接收缓存
  * @param  bufsz:接收缓存大小
  * @retval 接收长度。-1表示接受失败
  * @note   消息队列阻塞接收，一直等待消息到来
*/
static int serial_receive(uint8_t *buf, int bufsz)
{
    struct rx_msg msg;
    rt_err_t result;
    rt_uint32_t rx_length;

    while (1)
    {
        rt_memset(&msg, 0, sizeof(msg));
        /* 从消息队列中读取消息 */
        result = rt_mq_recv(rx_mq, &msg, sizeof(msg), RT_WAITING_FOREVER);
        if (result == RT_EOK)
        {
            /* 从串口读取数据 */
            rx_length = rt_device_read(msg.dev, 0, buf, msg.size);
            /* 打印数据 */
//            LOG_HEX(UART_NAME,8,buf,rx_length);
            return rx_length;
        }
    }
}
/**
  * @brief  串口发送函数..
  * @param  buf:发送缓存
  * @param  bufsz:发送缓存大小
  * @retval 成功：1，失败：0
  * @note   None.
*/
static int serial_send(uint8_t *buf, int len)
{
  rt_size_t send_len = 0; 
  /* 通过串口设备 serial 输出读取到的消息 */
  send_len = rt_device_write(serial, 0, buf, len);
  if(send_len == len)
  {
    return 1;
  }
  else
  {
    LOG_E("send failed");
    return 0;
  }
}
/**
  * @brief  modbus 从机1 初始化
  * @param  None.
  * @retval None.
  * @note   None.
*/
static int Modbus_Slave1_Init(void)
{
    rt_err_t ret = RT_EOK;
    struct rx_msg msg;
    /* 初始化消息队列 */
    rx_mq = rt_mq_create("mbs1ave1",
                         sizeof(struct rx_msg),    /* 一条消息的最大长度 */
                         BSP_UART1_RX_BUFSIZE,     /* 存放消息的缓冲区大小 */
                         RT_IPC_FLAG_FIFO); 

    /* 查找串口设备 */
    serial = rt_device_find(UART_NAME);
    if (!serial)
    {
        rt_mq_detach(rx_mq);
        LOG_E("find %s failed!", UART_NAME);
        return RT_ERROR;
    }
               
    /* 创建一个动态互斥量 */
    mb_slave_mutex = rt_mutex_create("mb slave", RT_IPC_FLAG_PRIO);
    if (mb_slave_mutex == RT_NULL)
    {
        LOG_E("create mb_slave_mutex failed.");
        return -1;
    }
    
    /* 创建 MODBUS从机线程*/
    rt_thread_t thread = rt_thread_create( UART_NAME,         /* 线程名字 */
                                           modbus_thread,     /* 线程入口函数 */
                                           RT_NULL,           /* 线程入口函数参数 */
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
INIT_COMPONENT_EXPORT(Modbus_Slave1_Init);
/*******************************************FREEMODBUS接口函数*************************/
/**
  * @brief  MB串口初始化
  * @param  波特率、奇偶校验
  * @retval None
  * @note   
  */
static void MBPortSerialInit(ULONG ulBaudRate, eMBParity eParity)
{
    /* 查找串口设备 */
    serial = rt_device_find(UART_NAME);
    if (!serial)
    {
        rt_mq_detach(rx_mq);
        LOG_E("find %s failed!", UART_NAME);
    }
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;  /* 初始化配置参数 */
    /* step2：修改串口配置参数 */
    config.baud_rate = ulBaudRate;            // 修改波特率
    config.data_bits = DATA_BITS_8;           // 数据位 8
    config.stop_bits = STOP_BITS_1;           // 停止位 1
    config.rx_bufsz  = BSP_UART1_RX_BUFSIZE;  // 修改缓冲区 rx buff size 为 128
    config.tx_bufsz  = BSP_UART1_TX_BUFSIZE;  // 修改缓冲区 rx buff size 为 128
    switch(eParity)
    {
      case MB_PAR_ODD:// 奇校验
          config.parity    = PARITY_ODD;
          config.data_bits = DATA_BITS_9;//带奇校验数据位为9bits
          break;
      case MB_PAR_EVEN://偶校验
          config.parity    = PARITY_EVEN;
          config.data_bits = DATA_BITS_9;//带偶校验数据位为9bits
          break;
      default://无校验
          config.parity    = PARITY_NONE;
          config.data_bits = DATA_BITS_8;//无奇偶校验数据位为8bits
          break;
    }
    /* step3：控制串口设备。通过控制接口传入命令控制字，与控制参数 */
    rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config);
    /* 以 DMA 接收及轮询发送方式打开串口设备 */
    rt_device_open(serial, RT_DEVICE_FLAG_RX_NON_BLOCKING | RT_DEVICE_FLAG_TX_NON_BLOCKING);
    /* 设置接收回调函数 */
    rt_device_set_rx_indicate(serial, uart_input);
#ifdef IRQ_PRIORITY
    struct stm32_uart *uart;
    uart = rt_container_of(serial, struct stm32_uart, serial);
    /* parameter check */
    RT_ASSERT(uart != RT_NULL);
    HAL_NVIC_SetPriority(uart->config->irq_type,IRQ_PRIORITY, 0);
#endif
}
/**
  * @brief  MB定时器初始化
  * @param  定时时间
  * @retval None
  * @note   分开使用不同定时器，容易调节波特率
            固定使用波特率，可用一个定时器。采用比较时间模式
  */
static void MBPortTimersInit( USHORT usTim1Timerout50us )
{

}
/**
  * @brief  MB串口读取一个比特
  * @param  获取地址
  * @retval None
  * @note   采用FIFO+DMA。
            实际从FIFO缓冲中读取
  */
static void MBPortSerialGetByte(UCHAR *pucByte)
{

}
/**
  * @brief  MB写入串口一个比特
  * @param  发送值
  * @retval None
  * @note   采用FIFO+DMA。
            实际发送至FIFO缓冲中
  */
static void MBPortSerialPutByte(UCHAR ucByte)
{
   rt_device_write(serial, 0,(uint8_t *)ucRTU_DCB.pucBuffer,ucRTU_DCB.ucBufferCount);
}
/**
  * @brief  MB定时器使能
  * @param  定时时间
  * @retval None
  * @note   大于19200，固定3.5US*50=1750US
            否则进行计算
  */
static void MBPortTimersEnable(USHORT Timerout50us)
{
}
/**
  * @brief  MB定时器禁用
  * @param  None
  * @retval None
  * @note   None
  */
static void MBPortTimersDisable(void)
{

}
/**
  * @brief  MB串口发送接收使能、禁用
  * @param  None
  * @retval None
  * @note   None
  */
static void MBPortSerialEnable(BOOL xRxEnable,BOOL xTxEnable)
{
  
}
/**
  * @brief  进入中断
  * @param  None
  * @retval None
  * @note   None
  */
static void EnterCriticalSection(void)
{

}
/**
  * @brief  退出中断
  * @param  None
  * @retval None
  * @note   None
  */
static void ExitCriticalSection(void)
{

}