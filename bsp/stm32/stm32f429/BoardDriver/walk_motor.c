/**
  ******************************************************************************
  * @file    walk_motor.c
  * @brief   行走电机驱动
  * @date    2022.09.05
  ******************************************************************************
  * @attention  
            方向高向前，低向后
            使能低电平
            刹车高电平
  * @author HLY
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "walk_motor.h"
/* Private includes ----------------------------------------------------------*/
#include "tim.h"
#include "motor.h"
#include "user_math.h"
#include "mb_handler.h"

/*ulog include*/
#define LOG_TAG              "walk"
#define LOG_LVL              DBG_INFO
#include <ulog.h>
/* Private typedef -----------------------------------------------------------*/
struct stm32_pulse_encoder_device
{
    struct rt_pulse_encoder_device pulse_encoder;
    TIM_HandleTypeDef tim_handler;
    IRQn_Type encoder_irqn;
    rt_int32_t over_under_flowcount;
    char *name;
};
/* Private define ------------------------------------------------------------*/
#define RESOLUTION    Walk_RESOLUTION     //光电编码器线数
#define RATIO         Walk_REDUCTION_RATIO
#define Cycle_Pulse   Walk_Cycle_Pulse_Num//一周期需要脉冲数 一圈1024分辨率编码器 * 4倍频
#define CLOCKWISE     Walk_CLOCKWISE//电机顺时针方向电平值
#define MAX_SPEED     Walk_SPEED_MAX//RPM 转/分钟
#define MIN_SPEED     Walk_SPEED_MIN   //RPM 转/分钟
#define STEP          Walk_STEP  //10RPM
#define TIME          Walk_TIME  //ms
#define TIMX          htim3
#define CHANNELX      TIM_CHANNEL_2
/* Private macro -------------------------------------------------------------*/
/*端口定义*/
#define WALK      	  (walk)
#define SET_V_ADDR    5
/* 脉冲编码器名称 */
#define PULSE_ENCODER_DEV_NAME    "pulse4"    
#define IRQ_PRIORITY  15
/* Private variables ---------------------------------------------------------*/
WALK_MOTOR_TypeDef walk;
/* 脉冲编码器设备句柄 */
static rt_device_t pulse_encoder_dev = RT_NULL; 
/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  重入函数
  * @param  None.
  * @retval None.
  * @note   保证再次进入不会立刻进行。
*/
static void Walk_Motor_Reentrant(void)
{
  WALK.motor.set_speed = 0;
  usRegHoldingBuf[SET_V_ADDR] = 0;     //杜绝刹车时给入速度，松开刹车马上运行
    
  WALK.motor.AND.last_target = 0;
  WALK.motor.AND.target = 0;
  WALK.motor.AND.set_value = 0;
  WALK.motor.set_freq = 0;
}
/**
  * @brief  编码器计数，速度计算
  * @param  None
  * @retval None
  * @note   https://blog.csdn.net/csol1607408930/article/details/112793292
            M法
*/
static void Motor_Encoder_Calculation(void)
{
  rt_int32_t count;
  static rt_int32_t  last_count;
  /* 读取脉冲编码器计数值 */
  rt_device_read(pulse_encoder_dev, 0, &count, 1);
  WALK.motor.get_freq  = ( count -  last_count) * 10;
  WALK.motor.get_speed = 60 * (count - last_count) * 10 / RESOLUTION;
  last_count =  count;
}
/************************线程及中断函数********************************************/
/**
  * @brief  行走电机监控函数.
  * @param  雷达感应的方向：1为前雷达。2为后雷达
  * @retval None.
  * @note   None.
*/
void Walk_Motor_Detection(void)
{
  static stop_type last_code = NO_STOP;
  if(BEFORE_RADAR_3_GET == 1)//前雷达触碰
  {
    if(WALK.motor.set_speed > 0)//前进
    {
      WALK.limit_mode = 1;
      USER_SET_BIT(WALK.Stop_state,Detection_STOP);
    }
    else//触碰时后退，可以执行
    {
      WALK.limit_mode = 3;
      USER_CLEAR_BIT(WALK.Stop_state,Detection_STOP);
    }
  }

  //接近开关触碰或者漫反射触碰
  if(LIFT_SQP_GET == 1 || LIFT_REFLECTION_GET == 1)
  {
    if(WALK.motor.set_speed < 0)//后退
    {
      WALK.limit_mode = 2;
      USER_SET_BIT(WALK.Stop_state,Detection_STOP);
    }
    else
    {
      WALK.limit_mode = 3;
      USER_CLEAR_BIT(WALK.Stop_state,Detection_STOP);
    }
  }

  if(WALK.Stop_state != last_code)
  {
    if(WALK.Stop_state != NO_STOP)
     LOG_W("Motor stop, stop code is 0X%04X",WALK.Stop_state);
    else
     LOG_I("Motor return to normal,code is 0X%04X",WALK.Stop_state);
    last_code = WALK.Stop_state;
  }
}
/**
  * @brief  电机加减速运行
  * @param  None.
  * @retval None.
  * @note   None.
*/
static void Walk_Motor_1MS(void *parameter)
{
  static uint16_t cnt = 0,cnt1 = 0;
  ++cnt;
  ++cnt1;
  if(!(cnt % WALK.motor.AND.time))
  {
    Trapezoidal_AND(&WALK.motor);
    cnt = 0;
  }
  if(!(cnt1 % 100))
  {
    HAL_GPIO_TogglePin(DEBUG_GPIO_Port,DEBUG_Pin);
    Motor_Encoder_Calculation();
  }
}
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
  if(WALK.Stop_state != NO_STOP)
  {
    Walk_Motor_Stop();
    Walk_Motor_Reentrant();
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
  USER_CLEAR_BIT(WALK.Stop_state,ENABLE_STOP);
  WALK.PIN.EN.level = GPIO_PIN_RESET;
  HAL_GPIO_WritePin(WALK.PIN.EN.GPIOx,WALK.PIN.EN.GPIO_Pin,WALK.PIN.EN.level);    //使能电机
}
/**
  * @brief  禁用电机
  * @param  None.
  * @retval None.
  * @note   None.
*/
static void Motor_Disenable(void)
{
  USER_SET_BIT(WALK.Stop_state,ENABLE_STOP);
  WALK.PIN.EN.level = GPIO_PIN_SET;
  HAL_GPIO_WritePin(WALK.PIN.EN.GPIOx,WALK.PIN.EN.GPIO_Pin,WALK.PIN.EN.level);    //禁用电机
}
/**
  * @brief  电机开始函数
  * @param  None.
  * @retval None.
  * @note   None.
*/
static void Motor_Start(void)
{
  WALK.state.PWM   = Motor_PWM_Start(WALK.motor.TIMx,WALK.motor.Channel);//开启定时器
  WALK.state.MOTOR = Motor_RELEASE;                                      //电机松开刹车
  WALK.PIN.Brake.level = GPIO_PIN_RESET;
  HAL_GPIO_WritePin(WALK.PIN.Brake.GPIOx,WALK.PIN.Brake.GPIO_Pin,WALK.PIN.Brake.level);    //送开刹车
}
/**
  * @brief  转向电机停止函数
  * @param  None.
  * @retval None.
  * @note   None.
*/
void Walk_Motor_Stop(void)
{ 
  WALK.state.PWM   = Motor_PWM_Stop(WALK.motor.TIMx,WALK.motor.Channel);//关闭定时器
  WALK.state.MOTOR = Motor_STOP;                                        //电机停止输出
  WALK.PIN.Brake.level = GPIO_PIN_SET;
  HAL_GPIO_WritePin(WALK.PIN.Brake.GPIOx,WALK.PIN.Brake.GPIO_Pin,WALK.PIN.Brake.level);    //刹车
}
/**
  * @brief  设置电机运行速度
  * @param  dir:传入方向
  * @retval None.
  * @note   None.
*/
static void Motor_Direction(Directionstate dir)
{
  assert_param(WALK.PIN.Dir.GPIOx);
  assert_param(WALK.PIN.Dir.GPIO_Pin);
  if(dir == CW)
  {
    WALK.PIN.Dir.level = WALK.motor.clockwise;
  }
  else if(dir == CCW)
  {
    WALK.PIN.Dir.level = GPIO_TURN(WALK.motor.clockwise);
  }
  HAL_GPIO_WritePin(WALK.PIN.Dir.GPIOx,WALK.PIN.Dir.GPIO_Pin,WALK.PIN.Dir.level);
}

/************************用户初始化函数******************************************/
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
  timer = rt_timer_create("Walk1ms", Walk_Motor_1MS,
                             RT_NULL, rt_tick_from_millisecond(1),
                             RT_TIMER_FLAG_PERIODIC);

  /* 启动定时器*/
  if (timer != RT_NULL) rt_timer_start(timer);
}
/**
  * @brief  RTT编码器初始化
  * @param  None
  * @retval None
  * @note   None
*/
static void RTT_Encoder_Init(void)
{
  rt_err_t ret = RT_EOK;
 /* 查找脉冲编码器设备 */
  pulse_encoder_dev = rt_device_find(PULSE_ENCODER_DEV_NAME);
  if (pulse_encoder_dev == RT_NULL)
  {
      LOG_E("pulse encoder run failed! can't find %s device!", PULSE_ENCODER_DEV_NAME);
  }
  /* 以只读方式打开设备 */
  ret = rt_device_open(pulse_encoder_dev, RT_DEVICE_OFLAG_RDONLY);
  if (ret != RT_EOK)
  {
      LOG_E("open %s device failed!", PULSE_ENCODER_DEV_NAME);
  }
#ifdef IRQ_PRIORITY
  struct stm32_pulse_encoder_device *tim_device = RT_NULL;
  tim_device = rt_container_of(pulse_encoder_dev, struct stm32_pulse_encoder_device, pulse_encoder);
  
  HAL_NVIC_SetPriority(tim_device->encoder_irqn,IRQ_PRIORITY, 0);
#endif
}
/**
  * @brief  行走电机初始化
  * @param  None
  * @retval None
  * @note   None
*/
void Walk_Motor_Init(void)
{
  MX_TIM3_Init();
  RTT_Timer_Init();
  RTT_Encoder_Init();
  
  WALK.PIN.EN.GPIOx             = Walk_EN_GPIO_Port;
  WALK.PIN.EN.GPIO_Pin          = Walk_EN_Pin;
  
  WALK.PIN.Dir.GPIOx            = Walk_FR_GPIO_Port;
  WALK.PIN.Dir.GPIO_Pin         = Walk_FR_Pin;
  
  WALK.PIN.Brake.GPIOx          = Walk_BK_GPIO_Port;
  WALK.PIN.Brake.GPIO_Pin       = Walk_BK_Pin;
  
  WALK.ALM.IO.GPIOx             = Walk_ALM_GPIO_Port; 
  WALK.ALM.IO.GPIO_Pin          = Walk_ALM_Pin;
  
  WALK.motor.Motor_Enable        = Motor_Enable;
  WALK.motor.Motor_Disenable     = Motor_Disenable;
  WALK.motor.Motor_Start         = Motor_Start;
  WALK.motor.Motor_Stop          = Walk_Motor_Stop;
  WALK.motor.Motor_Direction     = Motor_Direction;
  WALK.motor.Modify_Freq         = Modify_TIM_Freq;
  WALK.motor.Motor_init          = Motor_PWM_init;
  WALK.motor.Motor_Stop_Priority = Motor_Stop_Priority;
  
  WALK.limit_mode = 3;
    
  Trapezoidal_Init(&WALK.motor,   //梯形加减速指针
                    RATIO,        //减速比
                    MAX_SPEED,    //最大限速
                    MIN_SPEED,    //最小限速
                   &TIMX,         //使用定时器
                    TIM_CHANNEL_2,//使用通道值
                    TIME,         //定时周期
                    STEP,         //定时步长
                    CLOCKWISE,    //顺时针方向电平值
                    Cycle_Pulse   //周期脉冲数
                  );
}