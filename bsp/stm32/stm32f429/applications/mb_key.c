/**
  ******************************************************************************
  * @file    mb_key.c
  * @brief   mb按键驱动V1.0
  * @date    2021.01.10
  ******************************************************************************
  * @attention  把MB线圈抽象为按键
  MBKEY处理方式：1.单独创建一个线程，查询方式循环运行。 【简单，易编写】
                 2.为每个按键创建一个线程，MBKEY发送信号量，线程在恢复处理
                 【维护困难，创建无用线程过多】
  * @author HLY
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "mb_key.h"
/* Private includes ----------------------------------------------------------*/
#include "user_math.h"
#include "mb_handler.h"
#include "turn_motor.h"
#include "walk_motor.h"
#include "lifter_motor.h"
#include "adc_dma.h"
/*ulog include*/
#define LOG_TAG              "mb key"
#define LOG_LVL              DBG_INFO
#include <ulog.h>
/* Private typedef -----------------------------------------------------------*/
/** 
  * @brief  按键状态机的五种状态
  */ 
typedef enum _MBKEY_STATUS_LIST
{
  MBKEY_ENABLE,   // 使能，赋值为1
	MBKEY_DISENABLE,// 失能，赋值为0
	MBKEY_PRESS,    // 按键按下
	MBKEY_RAISE,    // 按键抬起
}MBKEY_STATUS_LIST;
/** 
  * @brief  按键屏蔽标志
  */  
typedef FunctionalState MBKEY_ENABLE_STATUS;
/** 
  * @brief  按键电平状态
  */  
typedef  GPIO_PinState IO_STATUS_LIST;
/** 
  * @brief  状态机初始化按键
  */  
typedef struct
{
	uint32_t GPIO_Pull;		//按键的上下拉模式
	uint16_t GPIOx;	      //按键对应的端口
	uint16_t GPIO_Pin_x;	//按键的引脚
	uint8_t  key_nox;
}MBKey_Init;
/** 
  * @brief  状态机类
  */
typedef struct _MBKEY_COMPONENTS
{
    MBKEY_ENABLE_STATUS 	MBKEY_SHIELD; 		//按键屏蔽，DISABLE(0):屏蔽，ENABLE(1):不屏蔽
    uint8_t             	MBKEY_TIMECOUNT;  //按键长按计数
    IO_STATUS_LIST 	    	MBKEY_FLAG;       //标志按键按下标志
    IO_STATUS_LIST 	    	MBKEY_DOWN_LEVEL; //按下时，按键IO实际的电平
    MBKEY_STATUS_LIST     MBKEY_STATUS; 		//按键状态
    MBKEY_STATUS_LIST     MBKEY_EVENT;  		//按键事件
    IO_STATUS_LIST (*MBREAD_PIN)(MBKey_Init Key);//读IO电平函数
}MBKEY_COMPONENTS;
/** 
  * @brief  按键类
  */
typedef struct
{
	MBKey_Init 				MBKey_Board; // 继承初始化父类
	MBKEY_COMPONENTS 	MBKeyStatus; // 继承状态机父类
}MBKey_Config;
/* Private define ------------------------------------------------------------*/
#define MKKEY_HANDLE_NUM 4
/*按键IO读取标志*/
#define  LOW_LEVEL 	   GPIO_PIN_RESET
#define  HIGH_LEVER    GPIO_PIN_SET
/* Private macro -------------------------------------------------------------*/
/* 线程配置 */
#define THREAD_PRIORITY      12//线程优先级
#define THREAD_TIMESLICE     10//线程时间片
#define THREAD_STACK_SIZE    1024//栈大小
/* Private variables ---------------------------------------------------------*/
MBKey_Config MBKey_Buf[MBKEY_NUM];	// 创建按键数组
SENSOR_state Stop_Button_state   = SENSOR_INIT;//急停按钮状态
uint16_t WDT_key = 0;//看门狗开关密码
/* Private function prototypes -----------------------------------------------*/
extern void MB_Param_Read(void);
extern void MB_Param_Save(void);
/**
  * @brief  按下处理事件
  * @param  None.
  * @retval None.
  * @note   None.
*/
static void MBKEY_ENABLE_Handle(uint8_t i)
{
  switch(i)
  {
    case KEY_SOFT_STOP:         //软急停
      TURN_ZERO_RESET;
      TURN_LOCATE_RESET;
      TURN_UP_RESET;
      TURN_DOWN_RESET;
      SMove_Stop(&turn.motor.Ctrl);
      USER_SET_BIT(turn.Stop_state,SOFT_STOP);
    
      WALK_BREAK_SET;
      USER_SET_BIT(walk.Stop_state,SOFT_STOP);
      break;
    case KEY_TURN_ZERO:         //转向电机复位
      break;
    case KEY_TURN_LOCATE:       //转向电机定位
      TURN_ZERO_RESET;
      TURN_UP_RESET;
      TURN_DOWN_RESET;
      turn.motor.Abs.set_angle = turn.set_radian * 180 / PI;
      SMove_SetAngle_Absolute(&turn.motor,turn.motor.Abs.set_angle);
      break;
    case KEY_TURN_UP:           //转向电机轴点动+
      TURN_ZERO_RESET;
      TURN_LOCATE_RESET;
      TURN_DOWN_RESET;
      SMove_SetRPM(&turn.motor,turn.motor.V.set_speed,DISABLE);
      break;
    case KEY_TURN_DOWN:         //转向电机轴点动-
      TURN_ZERO_RESET;
      TURN_LOCATE_RESET;
      TURN_UP_RESET;
      SMove_SetRPM(&turn.motor,-turn.motor.V.set_speed,DISABLE);
      break;
    case KEY_TURN_AXIS:         //转向电机轴清零
      break;
    case KEY_TURN_ENABLE:       //转向电机使能
      turn.motor.Ctrl.Motor_Enable();
      break;
    case KEY_WALK_BREAK:        //行走电机刹车
      Walk_Motor_Stop();
      break;
    case KEY_WALK_ENABLE:       //行走电机使能
      walk.motor.Motor_Enable();
      break;
    case KEY_LIFT_UP:           //叉臂顶升
      lift.limit_mode = 4;
      LIFT_DOWN_RESET;
      LIFT_LOCATE_RESET;
      LIFT_ZERO_RESET;
      Lifter_Motor_Jack(&lift);
      break;
    case KEY_LIFT_DOWN:         //叉臂下降
      LIFT_UP_RESET;
      LIFT_LOCATE_RESET;
      LIFT_ZERO_RESET;
      Lifter_Motor_Fall(&lift);
      break;
    case KEY_LIFT_LOCATE:       //叉臂定位
      LIFT_UP_RESET;
      LIFT_DOWN_RESET;
      Lifter_Motor_Set_Target(&lift,lift.incoming);
      break;
    case KEY_LIFT_ZERO:         //叉臂回零
      break;
    case KEY_LIFT_SQP_SHIELD:   //叉臂接近开关屏蔽
      lift.SQP.state = SENSOR_LEAVE;   //设置标志
      LIFT_SQP_RESET;
      break;
    case KEY_LIFT_4MA_SET://拉绳编码器4ma设置
      break;
    case KEY_LIFT_20MA_SET://拉绳编码器20ma设置
      break;
    case KEY_FACTORY:           //恢复出厂设置
      break;
    case KEY_READ:              //读取EEPROM数据
      break;
    case KEY_SAVE:              //保存EEPROM数据
      break;
    case KEY_TSET_WDT:          //看门狗测试
      break;
  }
}
/**
  * @brief  松开处理事件
  * @param  None.
  * @retval None.
  * @note   None.
*/
static void MBKEY_DISENABLE_Handle(u8 i)
{
  switch(i)
  {
    case KEY_SOFT_STOP:         //软急停
      break;
    case KEY_TURN_ZERO:         //转向电机复位
      break;
    case KEY_TURN_LOCATE:       //转向电机定位
      break;
    case KEY_TURN_UP:           //转向电机轴点动+
      if(turn.jog_flag == 1)
        SMove_SetRPM(&turn.motor,0,ENABLE);
      break;
    case KEY_TURN_DOWN:         //转向电机轴点动-
      if(turn.jog_flag == 2)
        SMove_SetRPM(&turn.motor,0,ENABLE);
      break;
    case KEY_TURN_AXIS:         //转向电机轴清零
      break;
    case KEY_TURN_ENABLE:       //转向电机不使能
      turn.motor.Ctrl.Motor_Disenable();
      TURN_ZERO_RESET;
      TURN_LOCATE_RESET;
      TURN_UP_RESET;
      TURN_DOWN_RESET;
      SMove_Stop(&turn.motor.Ctrl);
      break;
    case KEY_WALK_BREAK:        //行走电机刹车
      Trapezoidal_Out_Speed(&walk.motor,walk.motor.set_speed);
      break;
    case KEY_WALK_ENABLE:       //行走电机不使能
      walk.motor.Motor_Disenable();
      WALK_BREAK_SET;
      break;
    case KEY_LIFT_UP:           //叉臂顶升
      break;
    case KEY_LIFT_DOWN:         //叉臂下降
      break;
    case KEY_LIFT_LOCATE:       //叉臂定位
      break;
    case KEY_LIFT_ZERO:         //叉臂回零
      break;
    case KEY_LIFT_SQP_SHIELD:   //叉臂接近开关屏蔽
      break;
    case KEY_LIFT_4MA_SET://拉绳编码器4ma设置
      break;
    case KEY_LIFT_20MA_SET://拉绳编码器20ma设置
      break;
    case KEY_FACTORY:           //恢复出厂设置
      break;
    case KEY_READ:              //读取EEPROM数据
      break;
    case KEY_SAVE:              //保存EEPROM数据
      break;
    case KEY_TSET_WDT:          //看门狗测试
      break;
  }
}
/**
  * @brief  松开到按下事件
  * @param  None.
  * @retval None.
  * @note   None.
*/
static void MBKEY_PRESS_Handle(u8 i)
{
  switch(i)
  {
    case KEY_SOFT_STOP:         //软急停
      turn.limit_mode = 4;
      turn.jog_flag   = 4;
      MBKey_Shield_Operate(KEY_TURN_ZERO,DISABLE);   //不允许扫描按键
      MBKey_Shield_Operate(KEY_TURN_DOWN,DISABLE);   //不允许扫描按键
      MBKey_Shield_Operate(KEY_TURN_UP,DISABLE);     //不允许扫描按键
      MBKey_Shield_Operate(KEY_TURN_LOCATE,DISABLE); //不允许扫描按键
    
      WALK_BREAK_RESET; //松开刹车
      MBKey_Shield_Operate(KEY_WALK_BREAK,DISABLE); //不允许扫描按键
      break;
    case KEY_TURN_ZERO:         //转向电机复位
      if(Stop_Button_state != SENSOR_TOUCH)
      {
        //如果初始化状态在完成或者初始化进入回原流程
        if(turn.motor.Do.init_state == 3 ||  turn.motor.Do.init_state == 0XFF)
        {
          turn.motor.Do.init_state = 1;
          /* 创建线程*/
          rt_thread_t thread = rt_thread_create( "turn  do",            /* 线程名字 */
                                                 Turn_SMove_DoReset,    /* 线程入口函数 */
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
              LOG_E("turn  do thread created failed.");
          }
        }
      }
      break;
    case KEY_TURN_LOCATE:       //转向电机定位
      turn.limit_mode = 4;
      turn.jog_flag   = 4;
      //禁用按键
      MBKey_Shield_Operate(KEY_TURN_ZERO,DISABLE);   //不允许扫描按键
      MBKey_Shield_Operate(KEY_TURN_DOWN,DISABLE);   //不允许扫描按键
      MBKey_Shield_Operate(KEY_TURN_UP,DISABLE);     //不允许扫描按键
      break;
    case KEY_TURN_UP:           //转向电机轴点动+
      turn.limit_mode = 1;
      turn.jog_flag   = 1;
      //禁用按键
      MBKey_Shield_Operate(KEY_TURN_ZERO,DISABLE);   //不允许扫描按键
      MBKey_Shield_Operate(KEY_TURN_DOWN,DISABLE);   //不允许扫描按键
      MBKey_Shield_Operate(KEY_TURN_LOCATE,DISABLE); //不允许扫描按键
      break;
    case KEY_TURN_DOWN:         //转向电机轴点动-
      turn.limit_mode = 2;
      turn.jog_flag   = 2;
      //禁用按键
      MBKey_Shield_Operate(KEY_TURN_ZERO,DISABLE);   //不允许扫描按键
      MBKey_Shield_Operate(KEY_TURN_UP,DISABLE);     //不允许扫描按键
      MBKey_Shield_Operate(KEY_TURN_LOCATE,DISABLE); //不允许扫描按键
      break;
    case KEY_TURN_AXIS:        //转向电机轴清零
      SMove_SetAxis_Reset(&turn.motor.Ctrl);
      TURN_AXIS_RESET;
      break;
    case KEY_TURN_ENABLE:      //转向电机使能
      break;
    case KEY_WALK_BREAK:       //行走电机刹车
      break;
    case KEY_WALK_ENABLE:      //行走电机使能
      break;
    case KEY_LIFT_UP:           //叉臂顶升
      lift.limit_mode = 1;
      //禁用按键
      MBKey_Shield_Operate(KEY_LIFT_DOWN,DISABLE); //不允许扫描按键
      MBKey_Shield_Operate(KEY_LIFT_LOCATE,DISABLE); //不允许扫描按键
      MBKey_Shield_Operate(KEY_LIFT_ZERO,DISABLE); //不允许扫描按键
      break;
    case KEY_LIFT_DOWN:         //叉臂下降
      lift.limit_mode = 2;
      //禁用按键
      MBKey_Shield_Operate(KEY_LIFT_UP,DISABLE); //不允许扫描按键
      MBKey_Shield_Operate(KEY_LIFT_LOCATE,DISABLE); //不允许扫描按键
      MBKey_Shield_Operate(KEY_LIFT_ZERO,DISABLE); //不允许扫描按键
      break;
    case KEY_LIFT_LOCATE:       //叉臂定位
      lift.limit_mode = 3;
      //禁用按键
      MBKey_Shield_Operate(KEY_LIFT_UP,DISABLE); //不允许扫描按键
      MBKey_Shield_Operate(KEY_LIFT_DOWN,DISABLE); //不允许扫描按键
      MBKey_Shield_Operate(KEY_LIFT_ZERO,DISABLE); //不允许扫描按键
      break;
    case KEY_LIFT_ZERO:         //叉臂回零
      break;
    case KEY_LIFT_SQP_SHIELD:   //叉臂接近开关屏蔽
      break;
    case KEY_LIFT_4MA_SET://拉绳编码器4ma设置
      Rope_Encoder_Set_4mA();
      break;
    case KEY_LIFT_20MA_SET://拉绳编码器20ma设置
      Rope_Encoder_Set_20mA();
      break;
    case KEY_FACTORY:           //恢复出厂设置
      Modbus_Data_Init();
      DEFAULT_DATA_RESET;
      break;
    case KEY_READ:              //读取EEPROM数据
      MB_Param_Read();
      READ_RESET;
      break;
    case KEY_SAVE:              //保存EEPROM数据
      MB_Param_Save();
      SAVE_RESET;
      break;
    case KEY_TSET_WDT:          //看门狗测试
      if(WDT_key == 2234)
      {
        rt_hw_us_delay(5000*1000);//5S延时，触发看门狗
      }
      break;
  }
}
/**
  * @brief  按下到松开事件
  * @param  None.
  * @retval None.
  * @note   None.
*/
static void MBKEY_RAISE_Handle(u8 i)
{
  switch(i)
  {
    case KEY_SOFT_STOP:         //软急停
      if(turn.motor.Do.init_state == 3)//修复启动按钮按下复位至第二部时按下急停，导致再次按下启动按钮复位后卡死至第二步程序。 2022.07.25 
        TURN_LOCATE_SET;//启用转向电机定位
      USER_CLEAR_BIT(turn.Stop_state,SOFT_STOP);
      MBKey_Shield_Operate(KEY_TURN_ZERO,ENABLE);   //允许扫描按键
      MBKey_Shield_Operate(KEY_TURN_UP,ENABLE);     //允许扫描按键
      MBKey_Shield_Operate(KEY_TURN_DOWN,ENABLE);   //允许扫描按键
      MBKey_Shield_Operate(KEY_TURN_LOCATE,ENABLE); //允许扫描按键
      
      WALK_BREAK_RESET;      //松开刹车
      USER_CLEAR_BIT(walk.Stop_state,SOFT_STOP);
      MBKey_Shield_Operate(KEY_WALK_BREAK,ENABLE);  //允许扫描按键
      break;
    case KEY_TURN_ZERO:         //转向电机复位
      break;
    case KEY_TURN_LOCATE:       //转向电机定位
      //启用按键
      MBKey_Shield_Operate(KEY_TURN_ZERO,ENABLE);   //允许扫描按键
      MBKey_Shield_Operate(KEY_TURN_UP,ENABLE);     //允许扫描按键
      MBKey_Shield_Operate(KEY_TURN_DOWN,ENABLE);   //允许扫描按键
      break;
    case KEY_TURN_UP:           //转向电机轴点动+
    case KEY_TURN_DOWN:         //转向电机轴点动-
      //启用按键
      MBKey_Shield_Operate(KEY_TURN_ZERO,ENABLE);   //允许扫描按键
      MBKey_Shield_Operate(KEY_TURN_UP,ENABLE);     //允许扫描按键
      MBKey_Shield_Operate(KEY_TURN_DOWN,ENABLE);   //允许扫描按键
      MBKey_Shield_Operate(KEY_TURN_LOCATE,ENABLE); //允许扫描按键
      break;
    case KEY_TURN_AXIS:         //转向电机轴清零
      break;
    case KEY_TURN_ENABLE:       //转向电机不使能
      break;
    case KEY_WALK_BREAK:        //行走电机刹车
      break;
    case KEY_WALK_ENABLE:       //行走电机使能
      break;
    case KEY_LIFT_UP:           //叉臂顶升
      MBKey_Shield_Operate(KEY_LIFT_DOWN,ENABLE);   //允许扫描按键
      MBKey_Shield_Operate(KEY_LIFT_LOCATE,ENABLE);     //允许扫描按键
      MBKey_Shield_Operate(KEY_LIFT_ZERO,ENABLE);   //允许扫描按键
      break;
    case KEY_LIFT_DOWN:         //叉臂下降
      MBKey_Shield_Operate(KEY_LIFT_UP,ENABLE);   //允许扫描按键
      MBKey_Shield_Operate(KEY_LIFT_LOCATE,ENABLE);     //允许扫描按键
      MBKey_Shield_Operate(KEY_LIFT_ZERO,ENABLE);   //允许扫描按键
      break;
    case KEY_LIFT_LOCATE:       //叉臂定位
      MBKey_Shield_Operate(KEY_LIFT_UP,ENABLE);   //允许扫描按键
      MBKey_Shield_Operate(KEY_LIFT_DOWN,ENABLE); //允许扫描按键
      MBKey_Shield_Operate(KEY_LIFT_ZERO,ENABLE);   //允许扫描按键
      break;
    case KEY_LIFT_ZERO:         //叉臂回零
      break;
    case KEY_LIFT_SQP_SHIELD:   //叉臂接近开关屏蔽
      walk.limit_mode = 3;//恢复可以正常行走状态
      break;
    case KEY_LIFT_4MA_SET://拉绳编码器4ma设置
      break;
    case KEY_LIFT_20MA_SET://拉绳编码器20ma设置
      break;
    case KEY_FACTORY:           //恢复出厂设置
      break;
    case KEY_READ:              //读取EEPROM数据
      break;
    case KEY_SAVE:              //保存EEPROM数据
      break;
    case KEY_TSET_WDT:          //看门狗测试
      break;
  }
}
/** 
  * @brief  获取IO电平的函数
  按键读取函数
  */  
static IO_STATUS_LIST MBKEY_ReadPin(MBKey_Init Key)
{
  return (IO_STATUS_LIST)MB_GET_BIT(Key.GPIO_Pin_x);
}
/**
  * @brief  读取按键值
  * @param  None.
  * @retval None.
  * @note   根据实际按下按钮的电平去把它换算成虚拟的结果
*/
static void Get_MBKEY_Level(void)
{
    uint8_t i;
    for(i = 0;i < MBKEY_NUM;i++)
    {
        if(MBKey_Buf[i].MBKeyStatus.MBKEY_SHIELD == DISABLE)	//如果挂起则不进行按键扫描
            continue;
        if(MBKey_Buf[i].MBKeyStatus.MBREAD_PIN(MBKey_Buf[i].MBKey_Board) == MBKey_Buf[i].MBKeyStatus.MBKEY_DOWN_LEVEL)
            MBKey_Buf[i].MBKeyStatus.MBKEY_FLAG = LOW_LEVEL;
        else
            MBKey_Buf[i].MBKeyStatus.MBKEY_FLAG = HIGH_LEVER;
    }
}
/**
  * @brief  创建按键对象
  * @param  MBKey_Init
  * @retval None.
  * @note   创建按键对象
*/
static void MBCreat_Key(MBKey_Init* Init)
{
	uint8_t i; 
  for(i = 0;i < MBKEY_NUM;i++)
	{
		MBKey_Buf[i].MBKey_Board = Init[i]; // MBKey_Buf按钮对象的初始化属性赋值

		MBKey_Buf[i].MBKey_Board.key_nox = i;
		// 初始化按钮对象的状态机属性
		MBKey_Buf[i].MBKeyStatus.MBKEY_SHIELD = ENABLE;
		MBKey_Buf[i].MBKeyStatus.MBKEY_TIMECOUNT = 0;	
		MBKey_Buf[i].MBKeyStatus.MBKEY_FLAG = LOW_LEVEL;
    
		if(MBKey_Buf[i].MBKey_Board.GPIO_Pull == GPIO_PULLUP) // 根据模式进行赋值
			MBKey_Buf[i].MBKeyStatus.MBKEY_DOWN_LEVEL = LOW_LEVEL;
		else
			MBKey_Buf[i].MBKeyStatus.MBKEY_DOWN_LEVEL = HIGH_LEVER;
    
		MBKey_Buf[i].MBKeyStatus.MBKEY_STATUS = 	MBKEY_DISENABLE;
		MBKey_Buf[i].MBKeyStatus.MBKEY_EVENT	= 	MBKEY_DISENABLE;
		MBKey_Buf[i].MBKeyStatus.MBREAD_PIN 	= 	MBKEY_ReadPin;	//赋值按键读取函数
	}
}
/**
  * @brief  读取函数
  * @param  None.
  * @retval None.
  * @note   状态机的状态转换
*/
static void ReadMBKeyStatus(void)
{
  uint8_t i;
  Get_MBKEY_Level();
  for(i = 0;i < MBKEY_NUM;i++)
  {
    switch(MBKey_Buf[i].MBKeyStatus.MBKEY_STATUS)
    {
      //状态0：按键送开
      case MBKEY_DISENABLE:
        if(MBKey_Buf[i].MBKeyStatus.MBKEY_FLAG == LOW_LEVEL)
        {
            MBKey_Buf[i].MBKeyStatus.MBKEY_STATUS = MBKEY_DISENABLE;  //转入状态3
            MBKey_Buf[i].MBKeyStatus.MBKEY_EVENT  = MBKEY_DISENABLE;  //空事件
        }
        else
        {
          MBKey_Buf[i].MBKeyStatus.MBKEY_STATUS = MBKEY_PRESS;        //转入状态1
          MBKey_Buf[i].MBKeyStatus.MBKEY_EVENT 	= MBKEY_PRESS;        //过渡事件
        }
        break;
			//状态1：按键按下
      case MBKEY_ENABLE:
        if(MBKey_Buf[i].MBKeyStatus.MBKEY_FLAG == HIGH_LEVER)
        {
            MBKey_Buf[i].MBKeyStatus.MBKEY_STATUS = MBKEY_ENABLE;     //转入状态3
            MBKey_Buf[i].MBKeyStatus.MBKEY_EVENT  = MBKEY_ENABLE;     //空事件
        }
        else
        {
          MBKey_Buf[i].MBKeyStatus.MBKEY_STATUS = MBKEY_RAISE;        //转入状态0
          MBKey_Buf[i].MBKeyStatus.MBKEY_EVENT  = MBKEY_RAISE;        //过渡事件
        }
        break;
      //状态2：按键过渡[送开到按下]
      case MBKEY_PRESS:
        if(MBKey_Buf[i].MBKeyStatus.MBKEY_FLAG == HIGH_LEVER)
        {
            MBKey_Buf[i].MBKeyStatus.MBKEY_STATUS = MBKEY_ENABLE;     //转入状态3
            MBKey_Buf[i].MBKeyStatus.MBKEY_EVENT  = MBKEY_ENABLE;     //空事件
        }
        else
        {
            MBKey_Buf[i].MBKeyStatus.MBKEY_STATUS = MBKEY_DISENABLE;  //转入状态3
            MBKey_Buf[i].MBKeyStatus.MBKEY_EVENT  = MBKEY_DISENABLE;  //空事件
        }
        break;
			//状态1：按键过渡[按下到送开]
			case MBKEY_RAISE:
				if(MBKey_Buf[i].MBKeyStatus.MBKEY_FLAG == LOW_LEVEL)          //按键释放，端口高电平
        {
            MBKey_Buf[i].MBKeyStatus.MBKEY_STATUS = MBKEY_DISENABLE;  //转入状态3
            MBKey_Buf[i].MBKeyStatus.MBKEY_EVENT  = MBKEY_DISENABLE;  //空事件
        }
				else
        {
            MBKey_Buf[i].MBKeyStatus.MBKEY_STATUS = MBKEY_ENABLE;     //转入状态3
            MBKey_Buf[i].MBKeyStatus.MBKEY_EVENT = MBKEY_ENABLE;      //空事件
        }
        break;
    }
  }
}
/**
  * @brief  函数指针数组.
  * @param  None.
  * @retval None.
  * @note   
注意函数的顺序和数组越界问题
https://blog.csdn.net/feimeng116/article/details/107515317
*/
static void (*MBKEY_Operation[MKKEY_HANDLE_NUM])(u8 i) = 
{ 
  MBKEY_ENABLE_Handle,
  MBKEY_DISENABLE_Handle,
  MBKEY_PRESS_Handle,
  MBKEY_RAISE_Handle,
};
/**
  * @brief  处理函数
  * @param  None.
  * @retval None.
  * @note   放在定时器1ms一次
            当初始化值进入MB模式才运行
            扫描按键后，读取按键状态与事件进行判读处理
*/
void MBKey_Handler(void *p)
{
	uint8_t i;
  while(1)
  {
    rt_thread_mdelay(1);
    ReadMBKeyStatus();
    for(i = 0;i < MBKEY_NUM;i++)
    {
      //判断数据越界 指针无法判断指向内容大小
      //https://bbs.csdn.net/topics/80323809
      if(MBKey_Buf[i].MBKeyStatus.MBKEY_EVENT <= MKKEY_HANDLE_NUM)
        MBKEY_Operation[MBKey_Buf[i].MBKeyStatus.MBKEY_EVENT](i);
    }
  }
}
/**
  * @brief  MBKEY按键屏蔽操作
  * @param  num:MBKEY_LIST注册表中选项
  * @param  option: ENABLE  ：启用。
                    DISABLE ：禁用。
  * @retval None.
  * @note   禁用或者启用
*/
void MBKey_Shield_Operate(uint8_t num,FunctionalState option)
{
  MBKey_Buf[num].MBKeyStatus.MBKEY_SHIELD   = option;
  MBKey_Buf[num].MBKeyStatus.MBKEY_EVENT    = MBKEY_DISENABLE;//退出刹车事件
}
/**
  * @brief  IO初始化初始化
  * @param  None.
  * @retval None.
  * @note   状态机初始化
GPIO_PULLUP：初始给高，端口，位口
*/
void MBKEY_Init(void)
{ 
	MBKey_Init MBKeyInit[MBKEY_NUM]=
	{ 
		{GPIO_PULLUP, 0, 1}, //软件急停
		{GPIO_PULLUP, 0, 2}, //转向电机复位
		{GPIO_PULLUP, 0, 3}, //转向电机定位
		{GPIO_PULLUP, 0, 4}, //转向电机轴点动+
		{GPIO_PULLUP, 0, 5}, //转向电机轴点动-
    {GPIO_PULLUP, 0, 6}, //转向电机轴清零
    {GPIO_PULLUP, 0, 7}, //转向电机使能
    {GPIO_PULLUP, 0, 8}, //行走电机刹车
    {GPIO_PULLUP, 0, 9}, //行走电机使能
    {GPIO_PULLUP, 0, 10},//叉臂顶升
    {GPIO_PULLUP, 0, 11},//叉臂下降
    {GPIO_PULLUP, 0, 12},//叉臂定位使能
    {GPIO_PULLUP, 0, 13},//叉臂回零
    {GPIO_PULLUP, 0, 14},//拉绳编码器4ma设置
    {GPIO_PULLUP, 0, 15},//拉绳编码器20ma设置
    {GPIO_PULLUP, 0, 16},//叉臂接近开关屏蔽
    {GPIO_PULLUP, 0, 24},//恢复出厂设置
    {GPIO_PULLUP, 0, 25},//读取EEPROM数据
    {GPIO_PULLUP, 0, 26},//保存EEPROM数据
    {GPIO_PULLUP, 0, 27},//看门狗测试
	};
	MBCreat_Key(MBKeyInit);// 调用按键初始化函数
}