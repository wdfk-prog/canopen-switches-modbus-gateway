/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : 
  * @brief          : 
  * @date           :
  ******************************************************************************
  * @attention
  设响应时间为1s转动180度，则RPM = 120，为2转/S
  S型加减速完成加减速所需步长为159363
  一圈需要脉冲数=电子齿轮数20*10000细分，则需要发送速度频率为200k*2=400k
  
  加减速运行效果好，会减少响应时间。提升响应时间，加减速效果差。
  ****************************电机控制*********************************************
  减速线程   <-读写-> |         |
  MB处理任务 <-只读-> |控制参数 | <-读写-> us定时器中断  
  回原线程   <-读写-> |         | <-只读-> 10ms软件定时器 
                      
                      |         |
  MB处理任务 <-只读-> |显示参数 | <-只写-> 10ms软件定时器
                      |         |
  ****************************电机监控线程*****************************************
            创建监控线程             优先级高于控制线程
  电机初始化------------>监控线程监控------------------>停止电机
                                      判断到异常
  ****************************回原处理*********************************************
             回原按键          执行完成              
  MB处理线程--------------->回原线程--------->挂起回原线程
             恢复回原线程
  * @author
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "turn_motor.h"
/* Private includes ----------------------------------------------------------*/
#include "mb_key.h"
#include "mb_handler.h"
#include "user_math.h"
/*ulog include*/
#define LOG_TAG              "turn"
#define LOG_LVL              DBG_INFO
#include <ulog.h>
/* Private typedef -----------------------------------------------------------*/
struct stm32_hwtimer
{
    rt_hwtimer_t time_device;
    TIM_HandleTypeDef    tim_handle;
    IRQn_Type tim_irqn;
    char *name;
};
/* Private define ------------------------------------------------------------*/
/*端口定义*/
#define TURN      	       (turn)
 /* 定时器名称 */
#define HWTIMER_DEV_NAME   "timer9"    
#define IRQ_PRIORITY  15

#define MASTER_TIM          htim2
#define CHANNELX            TIM_CHANNEL_2   
#define SLAVE_TIM           htim9
#define Cycle_Pulse_Num     Turn_Cycle_Pulse_Num//一周期需要脉冲数
#define REDUCTION_RATIO     Turn_REDUCTION_RATIO//减速比
#define CLOCKWISE           Turn_CLOCKWISE      //转向电机方向电平值

#define ANGLE_MAX           Turn_ANGLE_MAX
#define ANGLE_MIN           Turn_ANGLE_MIN

#define SPEED_MAX           Turn_SPEED_MAX
#define SPEED_MIN           Turn_SPEED_MIN

#define BACK_HIGH_SPEED     Turn_BACK_HIGH_SPEED
#define BACK_LOW_SPEED      Turn_BACK_LOW_SPEED
#define SANGLE_INIT         Turn_SANGLE_INIT

#define BREAK_ENABLE        0//有刹车引脚

#define SET_ANGLE_ADDR      1
#define GET_ANGLE_ADDR      201
#define SET_ANGLE_MAX_ADDR  504
#define SET_ANGLE_MIN_ADDR  505
/* Private macro -------------------------------------------------------------*/
/*MOTOR 7S加减速参数 structures definition*/
#define TURN_STEP_PARA		  50      			 //任意时刻转动步数修正因子【影响时间与细分度，越小时间越长，越细分】
#define TURN_STEP_AA			  31       		   //加加速阶段，离散化点数【越大越细分，不影响时间】
#define TURN_STEP_UA			  31			  	   //匀加速阶段，离散化点数
#define TURN_STEP_RA			  31					   //减加速阶段，离散化点数
#define TURN_STEP_LENGTH (TURN_STEP_AA + TURN_STEP_UA + TURN_STEP_RA)//总步长
  
#define M_FRE_START	        1600;           //电机的启动频率【启动频率低，该频率步长为0】
#define M_FRE_AA					  10000;          //电机频率的加加速度【决定最大速度上限】[影响总步长]
#define M_T_AA					    1;              //电机频率的加加速时间
#define M_T_UA					    3;              //电机频率的匀加速时间
#define M_T_RA					    1;              //电机频率的减加速时间
/* Private variables ---------------------------------------------------------*/
TURN_MOTOR_TypeDef turn;
/*S型曲线参数生成的表格 公共表格*/
static uint32_t SMove_TimeTable [2 * TURN_STEP_LENGTH + 1];
static uint32_t SMove_StepTable [2 * TURN_STEP_LENGTH + 1];
/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  重入角度定位角度设置
  * @param  None.
  * @retval None.
  * @note   保证再次进入角度定位不会立刻进行定位。
*/
static void Turn_Motor_Reentrant_SetAngle(void)
{
  TURN.motor.Abs.get_angle = SMove_Get_Angle(&TURN.motor.Ctrl);
  TURN.get_radian          = TURN.motor.Abs.get_angle * PI / 180;
  usRegHoldingBuf[GET_ANGLE_ADDR]     = (_short)(TURN.get_radian*1000);
  usRegHoldingBuf[SET_ANGLE_ADDR]       = usRegHoldingBuf[GET_ANGLE_ADDR];
  TURN.set_radian          = TURN.get_radian ;
  TURN.motor.Abs.last      = TURN.motor.Abs.get_angle;
}
/************************线程及中断函数******************************************/
/**
  * @brief  转向电机监控函数
  * @param  监控电机状态
  * @retval None.
  * @note   
*/
void Turn_Motor_Detection(void)
{
  static LIMIT_MONITOR_STATE last = 0XFF;
  static stop_type last_code = NO_STOP;
  
  LIMIT_MONITOR_STATE ret = SMove_Limit_Detection(&TURN.motor,TURN.limit_mode,TURN.motor.Do.zero.state,TURN.motor.Do.limit.state);
  if(ret != last)
  {
    switch(ret)
    {
      case MONITOR_ABNORMAL:
        USER_SET_BIT(TURN.Stop_state,Detection_STOP);
        LOG_W("Abnormal motor monitoring.");
        break;
      case MONITOR_ALL:
        TURN_LIMIT_ALARM_SET;
        TURN_ZERO_ALARM_SET;
        USER_SET_BIT(TURN.Stop_state,Detection_STOP);
        LOG_W("Abnormal motor sensor,All touch.");
        break;
      case MONITOR_INIT:
        break;
      case MONITOR_LIMIT:
        TURN_LIMIT_ALARM_SET;
        USER_SET_BIT(TURN.Stop_state,Detection_STOP);
        LOG_W("The limit sensor is abnormally touched.");
        break;
      case MONITOR_NORMAL:
        USER_CLEAR_BIT(TURN.Stop_state,Detection_STOP);
        break;
      case MONITOR_ZERO:
        TURN_ZERO_ALARM_SET;
        USER_SET_BIT(TURN.Stop_state,Detection_STOP);
        LOG_W("The zero sensor is abnormally touched.");
        break;
    }
    last = ret;
  }
  if(TURN.Stop_state != last_code)
  {
    if(TURN.Stop_state != NO_STOP)
     LOG_W("Motor stop, stop code is 0X%04X",TURN.Stop_state);
    else
     LOG_I("Motor return to normal,code is 0X%04X",TURN.Stop_state);
    last_code = TURN.Stop_state;
  }
}
/**
  * @brief  转向电机回原函数
  * @param  主要完成转向电机回正过程
  * @retval None.
  * @note   完成后退出循环，即删除线程
*/
void Turn_SMove_DoReset(void * p)
{
  while(1)
  {
    rt_thread_mdelay(1);
    switch(TURN.motor.Do.init_state)
    {
      case 1:
        //屏蔽行走按键
        WALK_BREAK_SET;
        //屏蔽转向按键
        TURN_LOCATE_RESET;
        TURN_UP_RESET;
        TURN_DOWN_RESET;
        TURN_ZERO_SET;
        MBKey_Shield_Operate(KEY_TURN_UP,DISABLE);    //不允许扫描按键
        MBKey_Shield_Operate(KEY_TURN_DOWN,DISABLE);  //不允许扫描按键
        MBKey_Shield_Operate(KEY_TURN_LOCATE,DISABLE);//不允许扫描按键
        //限点限位
        TURN.limit_mode = 1;
        TURN.jog_flag   = 4;
        //回原函数
        ZERO_state ret = SMove_DoReset(&TURN.motor,TURN.motor.Do.speed_high,TURN.motor.Do.speed_low,TURN.motor.Do.zero.state,TURN.motor.Do.limit.state);
        if(ret == ZERO_OK)
        {
          TURN.motor.Ctrl.CurrentPosition_Pulse = TURN.motor.Ctrl.MaxPosition_Pulse * (1 - TURN.motor.Do.init_freq * 180 / PI / 360);//定位当前为需要复位角度;
          SMove_SetAngle_Cache(&TURN.motor.Abs);
          TURN.motor.Do.init_state = 2;//进入第二步初始化
          LOG_I("Coming back to the positive");
        }
        else if(ret == ZERO_ABNORMAL)
        {
          //恢复行走按键
          WALK_BREAK_RESET;
          //恢复转向按键
          TURN_ZERO_RESET;
          MBKey_Shield_Operate(KEY_TURN_UP,ENABLE);    //允许扫描按键
          MBKey_Shield_Operate(KEY_TURN_DOWN,ENABLE);  //允许扫描按键
          MBKey_Shield_Operate(KEY_TURN_LOCATE,ENABLE);//允许扫描按键
          MBKey_Shield_Operate(KEY_TURN_ZERO,ENABLE);  //允许扫描按键
          TURN.motor.Do.init_state = 0XFF;
          LOG_I("The motor is abnormal, and the thread is quit");
          return;
        }
        break;
      case 2:
        //智能限位
        TURN.limit_mode = 4;
        TURN.jog_flag   = 4;
        //回到零角度位置
        TURN.motor.Abs.set_angle = 0;
        if(SMove_SetAngle_Absolute(&TURN.motor,TURN.motor.Abs.set_angle) == DISABLE)
        {
          usRegHoldingBuf[SET_ANGLE_MAX_ADDR] = (uint16_t)ANGLE_MAX;
          usRegHoldingBuf[SET_ANGLE_MIN_ADDR] = (uint16_t)ANGLE_MIN;
          SMove_SetAngle_Cache(&TURN.motor.Abs);
          //恢复行走按键
          WALK_BREAK_RESET;
          //恢复转向按键
          TURN_ZERO_RESET;
          TURN_LOCATE_SET;
          MBKey_Shield_Operate(KEY_TURN_UP,ENABLE);    //允许扫描按键
          MBKey_Shield_Operate(KEY_TURN_DOWN,ENABLE);  //允许扫描按键
          MBKey_Shield_Operate(KEY_TURN_LOCATE,ENABLE);//允许扫描按键
          MBKey_Shield_Operate(KEY_TURN_ZERO,ENABLE);  //允许扫描按键
          TURN.motor.Do.init_state = 3;//软件初始化结束
          LOG_I("Back to the original finish");
        }
        break;
      case 3: //回原初始化结束
        LOG_I("Return to the origin and the initialization is complete");
        return;
        break;
      default:
        break;
    }
  }
}
/**
  * @brief  转向电机定时查询函数
  * @param  None.
  * @retval None.
  * @note   None.
            放入定时器10ms中断
*/
static void Turn_Motor_Get_Value_10MS(void *parameter)
{
  if(TURN.motor.Ctrl.running == 1)
    TURN.set_freq  = TURN.motor.Ctrl.Counter_Table[TURN.motor.Ctrl.CurrentIndex];
  else
    TURN.set_freq = 0;
  
  TURN.get_freq  = SMove_Get_Speed(&TURN.motor.Ctrl)  * 100;
  TURN.motor.V.get_speed = 60 * TURN.get_freq / (int32_t)(TURN.motor.Ctrl.cycle_pulse_num * TURN.motor.Ctrl.reduction_ratio);
  TURN.motor.Abs.get_angle = SMove_Get_Angle(&TURN.motor.Ctrl);
  TURN.get_radian    = TURN.motor.Abs.get_angle * PI / 180;
}
/**
  * @brief  定时器超时回调函数
  * @param  None
  * @retval None
  * @note   None
*/
static rt_err_t timeout_cb(rt_device_t dev, rt_size_t size)
{
    TURN.motor.Ctrl.Slave_Pulse_IT++;
    return 0;
}
/**
  * @brief  打印S型加减速列表
  * @param  None.
  * @retval None.
  * @note   None.
*/
static void SMove_list(void)
{
  for(uint8_t i = 0;i < TURN_STEP_LENGTH+1;i++)
  {
    if(i % 3 == 0)
    {
      rt_kprintf("\n");
    }
    rt_kprintf("num%3d|Hz:%7d|Step:%4d#",i,SMove_TimeTable[i],SMove_StepTable[i]);
  }
  rt_kprintf("\r\n");
  rt_kprintf("The total step:%d\r\n",TURN.motor.Ctrl.StartSteps+TURN.motor.Ctrl.StopSteps);
}
//MSH_CMD_EXPORT_ALIAS(SMove_list,turn_SMove_list,TURN motor hz / step list);
/************************用户挂钩编写函数******************************************/
/**
  * @brief  电机急停优先级.
  * @param  None.
  * @retval None.
  * @note   
软急停按下或者电机未使能
*/
static uint8_t Motor_Stop_Priority(void)
{
  if(TURN.Stop_state != NO_STOP)
  {
    Turn_Motor_Stop();
    Turn_Motor_Reentrant_SetAngle();
    TURN.motor.Do.init_state = 0xFF;//进入锁定状态，防止急停结束后进入初始化状态。
    return 1;
  }
  else
    return 0;
}
/**
  * @brief  使能电机
  * @param  None.
  * @retval None.
  * @note   低电平使能
*/
static void Motor_Enable(void)
{
  USER_CLEAR_BIT(TURN.Stop_state,ENABLE_STOP);
  TURN.PIN.EN.level = GPIO_PIN_RESET;
  HAL_GPIO_WritePin(TURN.PIN.EN.GPIOx,TURN.PIN.EN.GPIO_Pin,TURN.PIN.EN.level);    //使能电机
}
/**
  * @brief  禁用电机
  * @param  None.
  * @retval None.
  * @note   None.
*/
static void Motor_Disenable(void)
{
  USER_SET_BIT(TURN.Stop_state,ENABLE_STOP);
  TURN.PIN.EN.level = GPIO_PIN_SET;
  HAL_GPIO_WritePin(TURN.PIN.EN.GPIOx,TURN.PIN.EN.GPIO_Pin,TURN.PIN.EN.level);    //禁用电机
}
/**
  * @brief  电机开始函数
  * @param  None.
  * @retval None.
  * @note   None.
*/
static void Motor_Start(void)
{
  TURN.state.PWM   = Motor_PWM_Start(TURN.motor.Ctrl.Master_TIMx,TURN.motor.Ctrl.Channel);//开启主定时器
  TURN.state.MOTOR = Motor_BUSY;                                     //电机正在输出
#if(BREAK_ENABLE == 1)
  TURN.PIN.Brake.level = GPIO_PIN_RESET;
  HAL_GPIO_WritePin(TURN.PIN.Brake.GPIOx,TURN.PIN.Brake.GPIO_Pin,TURN.PIN.Brake.level);    //送开刹车
#endif
}
/**
  * @brief  电机停止函数
  * @param  None.
  * @retval None.
  * @note   None.
*/
void Turn_Motor_Stop(void)
{ 
  TURN.state.PWM   = Motor_PWM_Stop(TURN.motor.Ctrl.Master_TIMx,TURN.motor.Ctrl.Channel);//关闭主定时器
  TURN.state.MOTOR = Motor_STOP;                                                //电机停止输出
  Turn_Motor_Reentrant_SetAngle();
#if(BREAK_ENABLE == 1)
  TURN.PIN.Brake.level = GPIO_PIN_SET;
  HAL_GPIO_WritePin(TURN.PIN.Brake.GPIOx,TURN.PIN.Brake.GPIO_Pin,TURN.PIN.Brake.level);    //刹车
#endif
}
/**
  * @brief  设置电机运行速度
  * @param  dir:传入方向
  * @retval None.
  * @note   None.
*/
static void Motor_Direction(Directionstate dir)
{
  assert_param(TURN.PIN.Dir.GPIOx);
  assert_param(TURN.PIN.Dir.GPIO_Pin);
  if(dir == CW)
  {
    TURN.PIN.Dir.level = TURN.motor.Ctrl.clockwise;
  }
  else if(dir == CCW)
  {
    TURN.PIN.Dir.level = GPIO_TURN(TURN.motor.Ctrl.clockwise);
  }
  HAL_GPIO_WritePin(TURN.PIN.Dir.GPIOx,TURN.PIN.Dir.GPIO_Pin,TURN.PIN.Dir.level);
}
/************************用户初始化函数******************************************/
/**
  * @brief  PWM初始化函数.
  * @param  None.
  * @retval None.
  * @note   None
*/
static int PWM_Init(void)
{
    rt_err_t ret = RT_EOK;
    rt_hwtimerval_t timeout_s;      /* 定时器超时值 */
    rt_device_t hw_dev = RT_NULL;   /* 定时器设备句柄 */
    rt_hwtimer_mode_t mode;         /* 定时器模式 */

    /* 查找定时器设备 */
    hw_dev = rt_device_find(HWTIMER_DEV_NAME);
    if (hw_dev == RT_NULL)
    {
        LOG_E("hwtimer sample run failed! can't find %s device!", HWTIMER_DEV_NAME);
        return RT_ERROR;
    }

    /* 以读写方式打开设备 */
    ret = rt_device_open(hw_dev, RT_DEVICE_OFLAG_RDWR);
    if (ret != RT_EOK)
    {
        LOG_E("open %s device failedn", HWTIMER_DEV_NAME);
        return ret;
    }

    /* 设置超时回调函数 */
    rt_device_set_rx_indicate(hw_dev, timeout_cb);

    /* 设置模式为周期性定时器（若未设置，默认是HWTIMER_MODE_ONESHOT）*/
    mode = HWTIMER_MODE_PERIOD;
    ret = rt_device_control(hw_dev, HWTIMER_CTRL_MODE_SET, &mode);
    if (ret != RT_EOK)
    {
        LOG_E("set mode failed! ret is :%d", ret);
        return ret;
    }
    
    MX_TIM2_Init();//先初始化主定时器
    MX_TIM9_Init();//再初始化从定时器 不可打乱顺序
    HAL_TIM_Base_Start_IT(&htim9);  /* 启动TIM9*/
    
#ifdef IRQ_PRIORITY
    struct stm32_hwtimer *tim_device = RT_NULL;
    tim_device = rt_container_of(hw_dev, struct stm32_hwtimer, time_device);
    
    HAL_NVIC_SetPriority(tim_device->tim_irqn,IRQ_PRIORITY, 0);
#endif
    return ret;
}
/**
  * @brief  RTT软件定时器初始化
  * @param  None
  * @retval None
  * @note   None
*/
static void RTT_Timer_Init(void)
{
  rt_err_t ret = RT_EOK;
  /* 定时器的控制块 */
  static rt_timer_t timer;
  /* 创建定时器 1  周期定时器 */
  timer = rt_timer_create("Turn10ms", Turn_Motor_Get_Value_10MS,
                             RT_NULL, rt_tick_from_millisecond(10),
                             RT_TIMER_FLAG_PERIODIC);

  /* 启动定时器 1 */
  if (timer != RT_NULL) rt_timer_start(timer);
}
/**
  * @brief  转向电机初始化
  * @param  None
  * @retval None
  * @note   None
*/
void Turn_Motor_Init(void)
{ 
  PWM_Init();
  RTT_Timer_Init();
  
  TURN.PIN.EN.GPIOx             = Turn_ENA_GPIO_Port;
  TURN.PIN.EN.GPIO_Pin          = Turn_ENA_Pin;
  
  TURN.PIN.Dir.GPIOx            = Turn_DIR_GPIO_Port;
  TURN.PIN.Dir.GPIO_Pin         = Turn_DIR_Pin;
#if(BREAK_ENABLE == 1)
  TURN.motor.PIN.Brake.GPIOx       = Turn_Brake_GPIO_Port;
  TURN.motor.PIN.Brake.GPIO_Pin    = Turn_Brake_Pin;
#endif
  TURN.ALM.IO.GPIOx             = Turn_ALM_GPIO_Port; 
  TURN.ALM.IO.GPIO_Pin          = Turn_ALM_Pin;
  
  HAL_GPIO_EXTI_Callback(TURN.ALM.IO.GPIO_Pin);//初始化读取一次电平
  
  TURN.motor.Ctrl.Motor_Enable        = Motor_Enable;
  TURN.motor.Ctrl.Motor_Disenable     = Motor_Disenable;
  TURN.motor.Ctrl.Motor_Start         = Motor_Start;
  TURN.motor.Ctrl.Motor_Stop          = Turn_Motor_Stop;
  TURN.motor.Ctrl.Motor_Direction     = Motor_Direction;
  TURN.motor.Ctrl.Modify_Freq         = Modify_TIM_Freq;
  TURN.motor.Ctrl.Motor_init          = Motor_PWM_init;
  TURN.motor.Ctrl.Motor_Stop_Priority = Motor_Stop_Priority;
  
  TURN.motor.CFG.fstart               = M_FRE_START;
  TURN.motor.CFG.faa                  = M_FRE_AA;
  TURN.motor.CFG.taa                  = M_T_AA;
  TURN.motor.CFG.tua                  = M_T_UA;
  TURN.motor.CFG.tra                  = M_T_RA;
  
  TURN.motor.CFG.STEP_PARA					  = TURN_STEP_PARA;
  TURN.motor.CFG.STEP_AA					    = TURN_STEP_AA;
  TURN.motor.CFG.STEP_UA					    = TURN_STEP_UA;
  TURN.motor.CFG.STEP_RA					    = TURN_STEP_RA;

  TURN.motor.CFG.MotorTimeTable				= SMove_TimeTable;
  TURN.motor.CFG.MotorStepTable				= SMove_StepTable;
  
  TURN.motor.Do.zero.IO.GPIOx         = Turn_Zero_GPIO_Port;
  TURN.motor.Do.zero.IO.GPIO_Pin      = Turn_Zero_Pin;
  TURN.motor.Do.limit.IO.GPIOx        = Turn_Limit_GPIO_Port;
  TURN.motor.Do.limit.IO.GPIO_Pin     = Turn_Limit_Pin;
  
  TURN.motor.Do.speed_high            = BACK_HIGH_SPEED;
  TURN.motor.Do.speed_low             = BACK_LOW_SPEED;
  TURN.motor.Do.init_freq             = SANGLE_INIT;
  TURN.motor.Do.init_state            = 0xFF;
  
  TURN.limit_mode = 4;
  
  HAL_GPIO_EXTI_Callback(TURN.motor.Do.zero.IO.GPIO_Pin);//初始化读取一次电平
  HAL_GPIO_EXTI_Callback(TURN.motor.Do.limit.IO.GPIO_Pin);//初始化读取一次电平
  
  SMove_Initial(&TURN.motor.Ctrl,             //电机控制结构体
                &TURN.motor.CFG,              //电机加减速参数结构体
                Cycle_Pulse_Num,              //一周期需要脉冲数
                REDUCTION_RATIO,              //减速比
                &MASTER_TIM,                  //使用定时器
                CHANNELX,                     //使用通道
                CLOCKWISE,                    //顺时针电平
                TURN.motor.CFG.MotorTimeTable,//加减速频率列表
                TURN.motor.CFG.MotorStepTable,//加减速步长列表
                &SLAVE_TIM
                );
                
  SMove_SetSpeed_Range(&TURN.motor,SPEED_MAX,SPEED_MIN);    //设置角度范围
  SMove_SetAngle_Range(&TURN.motor.Abs,ANGLE_MAX,ANGLE_MIN);//设置角度范围
}