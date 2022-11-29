# STM32F429开发板 BSP 说明

## 简介

本文档为 STM32F429 开发板提供的 BSP (板级支持包) 说明。

主要内容如下：

- 开发板资源介绍
- BSP 快速上手
- 进阶使用方法

通过阅读快速上手章节开发者可以快速地上手该 BSP，将 RT-Thread 运行在开发板上。在进阶使用指南章节，将会介绍更多高级功能，帮助开发者利用 RT-Thread 驱动更多板载资源。

## 开发板介绍

挑战者 STM32F429 是一款基于 ARM Cortex-M4 内核的开发板，最高主频为 180Mhz，该开发板具有丰富的板载资源，可以充分发挥 STM32F429 的芯片性能。

开发板外观如下图所示：

![board](figures/board.jpg)

该开发板常用 **板载资源** 如下：

- MCU：STM32F429IGT6，主频 180MHz，1024KB FLASH ，256KB RAM
- 外部 RAM：IS42S16400J（SDRAM，8MB）
- 外部 FLASH：W25Q128（SPI，16MB）
- 常用外设
  - LED：PD14
  - 按键：2个，K1（兼具唤醒功能，PA0）
- 常用接口：USB 转串口、LCD 接口
- 调试接口，标准 JTAG/SWD

开发板更多详细信息请参考野火 [STM32 挑战者开发板介绍](https://fire-stm32.taobao.com/index.htm)。

## 外设支持

本 BSP 目前对外设的支持情况如下：

| **板载外设**      | **支持情况** | **备注**                              |
| :----------------- | :----------: | :------------------------------------- |
| USB 转串口        |     支持     |                                       |
| RS232         |     支持     |  |
| SPI Flash         |     支持     |                                       |
| 电位器             |     支持     |     使用 ADC1                          |
| 以太网            |     支持     |                                       |
| MPU6050六轴传感器 |     支持     |                   |
| SDRAM             |     支持     |                                       |
| LCD               |     支持     | 支持 RGB 屏                            |
| SD卡              |   即将支持   |                                       |
| CAN               |   即将支持   |                                       |
| EMW1062 | 暂不支持 | |
| **片上外设**      | **支持情况** | **备注**                              |
| GPIO              |     支持     | PA0, PA1... PK15 ---> PIN: 0, 1...176 |
| UART              |     支持     | UART1/2                             |
| SPI               |     支持     | SPI1/2/5                              |
| I2C               |     支持     | 软件 I2C                              |
| ADC               |     支持     |                                     |
| RTC               |     支持     | 支持外部晶振和内部低速时钟 |
| WDT               |     支持     |                                       |
| FLASH | 支持 | 已适配 [FAL](https://github.com/RT-Thread-packages/fal) |
| SDIO              |   暂不支持   | 即将支持                              |
| PWM               |   暂不支持   | 即将支持                              |
| USB Device        |   暂不支持   | 即将支持                              |
| USB Host          |   暂不支持   | 即将支持                              |
| **扩展模块**      | **支持情况** | **备注**                              |
| 暂无         |   暂不支持   | 暂不支持                              |

## 使用说明

使用说明分为如下两个章节：

- 快速上手

    本章节是为刚接触 RT-Thread 的新手准备的使用说明，遵循简单的步骤即可将 RT-Thread 操作系统运行在该开发板上，看到实验效果 。

- 进阶使用

    本章节是为需要在 RT-Thread 操作系统上使用更多开发板资源的开发者准备的。通过使用 ENV 工具对 BSP 进行配置，可以开启更多板载资源，实现更多高级功能。


### 快速上手

本 BSP 为开发者提供 MDK4、MDK5 和 IAR 工程，并且支持 GCC 开发环境。下面以 MDK5 开发环境为例，介绍如何将系统运行起来。

#### 硬件连接

使用数据线连接开发板到 PC，打开电源开关。

#### 编译下载

双击 project.uvprojx 文件，打开 MDK5 工程，编译并下载程序到开发板。

> 工程默认配置使用 JLink 下载程序，在通过 JLink 连接开发板的基础上，点击下载按钮即可下载程序到开发板

#### 运行结果

下载程序成功之后，系统会自动运行，观察开发板上 LED 的运行效果，红色 LED 常亮、绿色 LED 会周期性闪烁。

连接开发板对应串口到 PC , 在终端工具里打开相应的串口（115200-8-1-N），复位设备后，可以看到 RT-Thread 的输出信息:

```bash
 \ | /
- RT -     Thread Operating System
 / | \     3.1.1 build Nov 19 2018
 2006 - 2018 Copyright by rt-thread team
msh >
```
### 进阶使用

此 BSP 默认只开启了 GPIO 和 串口1 的功能，如果需使用 SD 卡、Flash 等更多高级功能，需要利用 ENV 工具对BSP 进行配置，步骤如下：

1. 在 bsp 下打开 env 工具。

2. 输入`menuconfig`命令配置工程，配置好之后保存退出。

3. 输入`pkgs --update`命令更新软件包。

4. 输入`scons --target=mdk4/mdk5/iar` 命令重新生成工程。

本章节更多详细的介绍请参考 [STM32 系列 BSP 外设驱动使用教程](../docs/STM32系列BSP外设驱动使用教程.md)。

## 注意事项

暂无

## 联系人信息

维护人:

更新信息:
V0.0.1 
    创建RTT工程
    加入CmBacktrace组件，并更新最新版本，修复AC6报错问题
Total RW  Size (RW Data + ZI Data)              4752 (   4.64kB)
Total ROM Size (Code + RO Data + RW Data)      40128 (  39.19kB)
2022.08.03
V0.0.1
    加入ULOG组件，启用时间戳功能
    启用硬件RTC时钟，ULOG时间错正常
    CmBacktrace在ULOG模式下输出异常，已改为用rt_printf
    加载
Total RW  Size (RW Data + ZI Data)              5456 (   5.33kB)
Total ROM Size (Code + RO Data + RW Data)      54752 (  53.47kB)
2022.08.04
V0.0.1 
    加入agile MODBSU组件，修改util文件。适配MODBUS
    使用RTT SERIALV2文件并更新至0806最新版本使用。
    串口1使用DMA RX 阻塞+ TX 阻塞。
    串口3为调试串口
    编译器优化选择改为 -0，方便调试
    SYSWTACH组件堆栈改为1024，避免无优化下爆栈
    Total RW  Size (RW Data + ZI Data)              9416 (   9.20kB)
    Total ROM Size (Code + RO Data + RW Data)     123092 ( 120.21kB)
2022.08.06
V0.0.1 
    删除agile MODBSU组件，接收处理函数响应不够。增加特殊功能码不易实现
    增加FREEMODBUSV1.5支持多从机修改版。并改造为RTT系统接收阻塞模式。
    增加支持串口模式修改波特率与校验位。
    Total RW  Size (RW Data + ZI Data)             11536 (  11.27kB)
    Total ROM Size (Code + RO Data + RW Data)     125992 ( 123.04kB)
2022.08.11
V0.0.1 
    搭建电机驱动框架，初步完成速度模式S型加减速过程
    Total RW  Size (RW Data + ZI Data)             13192 (  12.88kB)
    Total ROM Size (Code + RO Data + RW Data)     142888 ( 139.54kB)
2022.08.24
V0.0.1 
    测试加减速波形成功，完成角度与速度加减速控制
    启用5us定时器固定时间判断是否加减速
    通过RTT定时器驱动启用PWM中断
    Total RW  Size (RW Data + ZI Data)             13992 (  13.66kB)
    Total ROM Size (Code + RO Data + RW Data)     145176 ( 141.77kB)
2022.08.25
V0.0.1 
    新增MB处理线程，读写数据并处理任务。使用信号量恢复线程，互斥量保护MB缓冲区
    MB从机线程增加互斥量保护MB缓冲区，使用信号量挂起MB处理线程
    MB处理线程中运行MB_KEY函数。完成转向电机角度、手动、使能、软急停
    Total RW  Size (RW Data + ZI Data)             13680 (  13.36kB)
    Total ROM Size (Code + RO Data + RW Data)     152336 ( 148.77kB)
2022.08.26
V0.0.1 
    修复转向电机点动速度不变bug
    Total RW  Size (RW Data + ZI Data)             13680 (  13.36kB)
    Total ROM Size (Code + RO Data + RW Data)     152336 ( 148.77kB)
2022.08.29
V0.0.1 
    MB按键判断为回原按键，创建回原线程，回原执行完成退出函数，删除线程
    Total RW  Size (RW Data + ZI Data)             13680 (  13.36kB)
    Total ROM Size (Code + RO Data + RW Data)     152336 ( 148.77kB)
2022.08.29
V0.0.1 
    优化7S速度模式。减速停车模式需要查询判断是否完成。
    在按键松开时创建减速停车线程，并运行减速函数，判断完成减速停车后删除线程。
    Total RW  Size (RW Data + ZI Data)             11960 (  11.68kB)
    Total ROM Size (Code + RO Data + RW Data)     166796 ( 162.89kB)
2022.08.31
V0.0.1 
    7S速度模式使用二分法查表，可传入RPM速度。
    发送脉冲频率大于100K时，中断优先级高将打断串口通信，导致串口线程假死。
    解决办法1：设置为低速频率，2串口优先级抬高（可能会丢步）
    假死问题解决：关闭PWM中断，改为定时器主从模式采集脉冲个数
    从定时器开启溢出中断，确保脉冲计数精确
    Total RW  Size (RW Data + ZI Data)             11872 (  11.59kB)
    Total ROM Size (Code + RO Data + RW Data)     169980 ( 166.00kB)
2022.09.01
V0.0.1 
    MBHANDELER和MBKEY   模式从阻塞线程改为循环查询线程
    优化转向逻辑
    优化MBKEY逻辑，增加屏蔽函数操作
    Total RW  Size (RW Data + ZI Data)             11984 (  11.70kB)
    Total ROM Size (Code + RO Data + RW Data)     169744 ( 165.77kB)
2022.09.04
V0.0.1 
    添加转向电机报警函数
    优化版本控制与RTC设置
    增加行走电机框架与驱动。
    添加 编码器反馈
    Total RW  Size (RW Data + ZI Data)             12192 (  11.91kB)
    Total ROM Size (Code + RO Data + RW Data)     166112 ( 162.22kB)
2022.09.05
V0.0.1 
    调试串口命令行增加输出MODBUS数据与设置MODBUS数据命令
    增加MFBD按键驱动，添加RINGBUFFER，生成按键驱动框架
    Total RW  Size (RW Data + ZI Data)             12616 (  12.32kB)
    Total ROM Size (Code + RO Data + RW Data)     176208 ( 172.08kB)
2022.09.06
V0.0.1 
    修复转向电机报警传感器异常时回原线程不删除问题。
    添加转向电机停止代码监控函数，并打印停止代码
    调整打印版本函数在组件初始化中运行。避免干扰其他线程打印
    修复转向电机点动时急停，恢复时会继续运行bug
    增加AT指令于MODBUS串口中
    增加叉臂传感器并添加行走监控线程
    增加LED闪烁函数与LED闪烁报警函数
    增加ADC驱动
    Total RW  Size (RW Data + ZI Data)             13088 (  12.78kB)
    Total ROM Size (Code + RO Data + RW Data)     182588 ( 178.31kB)
2022.09.08
V0.0.1 
    增加心跳检测与报警
    电压检测与报警
    优化电机报警线程统一为一个监控线程中处理
    添加叉臂控制【未验证】
    添加fal框架与sfud spiflash
    Total RW  Size (RW Data + ZI Data)             14344 (  14.01kB)
    Total ROM Size (Code + RO Data + RW Data)     228068 ( 222.72kB)
2022.09.09
V0.0.1 
    添加虚拟文件系统DFS，挂载littlefs文件系统
    添加romfs文件系统，挂载/flash目录
    输出日志信息至文件系统保存为日志文件。存放在片外flash中
    编写初始化时，对于日志后端初始化的判断
    添加Ymodem传输协议，测试发送/读取正常。
    对于msh 读取命令进行编写，已适配保存至片外flash中
    Total RW  Size (RW Data + ZI Data)             15472 (  15.11kB)
    Total ROM Size (Code + RO Data + RW Data)     293580 ( 286.70kB)
2022.09.13
V0.0.1 
    littlefs文件系统写入速度过慢，且结构导致无法在片内flash中使用。
    转而搭载elm-fatfs文件系统。写入速度改善。【参数太多照样卡顿】
    flashdb文件模式保存读取参数测试成功
    使用ulog文件后端改造，完成系统日志与运动日志保存在不同区域
    使用ulog日志对于不同后端输出不同内容
    Total RW  Size (RW Data + ZI Data)             17256 (  16.85kB)
    Total ROM Size (Code + RO Data + RW Data)     179348 ( 175.14kB)
2022.09.15
