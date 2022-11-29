/**
  ******************************************************************************
  * @file    Smove.c
  * @brief   电机驱动V3.0
  * @date    2022.08.12
  ******************************************************************************
  * @attention  重构电机驱动，减少耦合
                编写7S加减速
                完成角度定位
                完成速度控制
                完成回原函数
                完成限位监控
                
  发脉冲两种目的
  1）速度控制
  2）位置控制

  速度控制方式.PWM发送脉冲或者发送占空比
  如果也采用查表法7S加减速方式。做到速度变化精度大，需要占较大内存。需要一个数组查表
  采用梯形加减速方式，不需要较大内存,运算简单，不需要进入中断记录脉冲个数控制。
  除非是需要控制位置，才会给7s加减速的速度控制。否则采用梯形加减速效果更好。
  
  发送脉冲方式为PWM，速率稳定而且资源占用少
  stm32位置控制需要获得发送的脉冲数，有下面4种手段
  1）每发送一个脉冲，做一次中断计数
  2）根据发送的频率×发送的时间，获得脉冲数量，对于变速的脉冲，可以累计积分的方法来获得总脉冲
  3）一个定时器作为主发送脉冲，另外一个定时器作为从，对发送的脉冲计数
  4）使用DMA方式，例如共发送1000个脉冲，那么定义u16 per[1001],每发送一个脉冲，dma会从数组中更新下一个占空比字，数组最后一个字为0，表示停发脉冲

  上面4种方法的用途和特点
  1）对于低速率脉冲比较好，可以说低速发脉冲的首选，例如10Khz以下的，否则中断占用太多的cpu，这种方法要注意将中断优先级提高，否则会丢计数，
  2）用作定时的计时精确高，可以允许有脉冲计数丢失的情况
  3）主从方式，需额外的定时器来计数，例如tim1发脉冲 tim2计数，最方便的方式，无论高速低速即可，同时占用cpu最低，只是要占用多一个定时器
  4）DMA方式也算是一个很确定的方式，不会丢失脉冲，但是高速的时候，会较多的占用内部总线同时会使用一个多余的DMA控制器，而且有个缺点，就是使用起来比较复杂，没有达到KISS原则

  * @author HLY
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "Smove.h"
/* Private includes ----------------------------------------------------------*/
#include <stdlib.h>
#include "user_math.h"

/*ulog include*/
#define LOG_TAG              "Smove"
#define LOG_LVL              DBG_INFO
#include <ulog.h>
/* Private define ------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
#define GPIO_TURN(x) (((x) == (GPIO_PIN_RESET))?(GPIO_PIN_SET):(GPIO_PIN_RESET))//电平翻转
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  S型加减速电机停止.
  * @param  None.
  * @retval None.
  * @note   需要停止请使用这个函数
*/
void SMove_Stop(SMove_Control_TypeDef* p)
{
  p->Motor_Stop();
  p->running         = DISABLE;
  p->en              = DISABLE;
}
/**
  * @brief  电机S型曲线算法公共处理函数.
  * @param  None.
  * @retval None.
  * @note   放入us级定时器中运行.
*/
void SMove_Handler(SMove_Control_TypeDef* p)
{ 
  assert_param(p);
	if(p->en == ENABLE)
	{ 	
    p->Slave_Pulse_Now = __HAL_TIM_GET_COUNTER(p->Slave_TIMx) + 65535 * p->Slave_Pulse_IT;
    int err = p->Slave_Pulse_Now - p->Slave_Pulse_Last;
    if(err != 0)
    {    
      //位置计算
      if(p->dir == CW)
      {
        p->CurrentPosition_Pulse+=err;
        if(p->CurrentPosition_Pulse >= p->MaxPosition_Pulse)
        {
          p->CurrentPosition_Pulse=0;
        }
      }
      else //逆时针为正角度值
      {
        p->CurrentPosition_Pulse-=err;
        if(p->CurrentPosition_Pulse == 0xffffffff)
        {
          p->CurrentPosition_Pulse = p->MaxPosition_Pulse - 1;
        }
      }
      p->Slave_Pulse_Last = p->Slave_Pulse_Now;
      
      p->CurrentPosition = p->CurrentPosition_Pulse / p->reduction_ratio;
      
      //速度控制使能 且 （当前位置等于目标位置 或者 目标 + 当前 等于 总步长）
      if(p->speedenbale && (p->CurrentIndex == p->TargetIndex || p->TargetIndex + p->CurrentIndex == p->StartTableLength + p->StopTableLength - 1))
      {
        return;
      }
      p->PulsesHaven++; //已运行步数
      p->pulsecount++;  //以该频率脉冲输出的脉冲个数
      //旋转预定脉冲数，停止，running=0，可以进行下一次旋转
      if(p->PulsesHaven >= p->PulsesGiven)
      {
        SMove_Stop(p);
        p->en = DISABLE;
        p->running = DISABLE;
        p->CurrentIndex = 0;
      }
      uint32_t stepstop = 0;
      //对称反转
      //		if(p->RevetDot==p->PulsesHaven)
      //		{
      //			p->pulsecount=p->Step_Table[p->CurrentIndex];
      //		}
      if(p->pulsecount >= p->Step_Table[p->CurrentIndex])//需要进入下一频率
      { 
        //对于速度控制，此处不能判断p->Ctrl.PulsesHaven>=(p->Ctrl.PulsesGiven>>1)
        //if(p->Ctrl.PulsesGiven-p->Ctrl.PulsesHaven<=p->Ctrl.StopSteps&&p->Ctrl.PulsesHaven>=(p->Ctrl.PulsesGiven>>1))
        if((p->PulsesGiven - p->PulsesHaven <= p->StopSteps && p->speedenbale == ENABLE)||//速度模式 判断 剩余步数，进入减速阶段
           (p->PulsesGiven - p->PulsesHaven <= p->StopSteps && p->PulsesHaven >= (p->PulsesGiven >> 1) && p->speedenbale == DISABLE)) //当前步长超过总步长一半
        {
          //减速阶段
          if(p->CurrentIndex < p->StartTableLength)
          {
            p->CurrentIndex = p->StartTableLength + p->StopTableLength - p->CurrentIndex;
          }
          p->CurrentIndex++;
          p->pulsecount = 0;
          p->state = DECEL;
          if(p->CurrentIndex >= p->StartTableLength + p->StopTableLength)//数组越界,完成不了加减速过程。重新计算需要减速
          {
//            p->CurrentIndex = p->StartTableLength + p->StopTableLength - 1;
            uint8_t i = 0;
            uint32_t stepstop = 0;
            while(stepstop < p->PulsesGiven - p->PulsesHaven)//剩余步长
            {
              i++;
              stepstop += p->Step_Table[i];
            }
            p->CurrentIndex = i;
          }
        }
        else if(p->PulsesHaven <= p->StartSteps)//加速阶段
        {
          //起步阶段
          if(p->CurrentIndex < p->StartTableLength)
          {
            p->CurrentIndex++;
            p->pulsecount = 0;
            if(p->CurrentIndex >= p->StartTableLength)//如果速度模式下，设置的当前位置大于开始步长，等于当前步长
              p->CurrentIndex = p->StartTableLength;
          }
          p->state = ACCEL;
        }
        else
        {
          p->state = CONST;
        }
        p->set_freq = p->Counter_Table[p->CurrentIndex];
        
        if(p->Step_Table[p->CurrentIndex] != 0)
          p->Modify_Freq(p->Master_TIMx,p->Channel,p->set_freq - 1);
      }
    }
	}
}
/**
  * @brief  GetFreAtTime:根据S型曲线参数获取某个时刻的频率
  * @param  fstart:开始频率
  * @param  faa   :二次加速度jerk
  * @param  taa   :加加速度时间
  * @param  tua   :匀加速度时间
  * @param  tra   :减加速度时间
  * @param  t     :当前时间
  * @retval 所需频率
  * @note   https://blog.csdn.net/xueluowutong/article/details/89069363
            https://www.eet-china.com/mp/a45617.html
  tk(k = 0,1,...7) :表示各个阶段的过渡点时刻
  τk(k = 0,1,...7):局部时间坐标，表示以各个阶段的起始点作为时间零点的时间表示
  τk = t - t(k-1);(k = 1,...7)
  Tk(k = 1,...7):各个阶段的持续运行时间
  T1 = taa,T2 = tua,T3 = tra
*/
static float GetFreAtTime(float fstart,float faa,float taa,float tua,float tra,float t)
{
    float V1,V2,V3,A_MAX;
    //a_max = J * T1;T1 = t1 - 0;当t = t1 时
      A_MAX = faa * taa;
    //V1 =      Vs + 1/2  ( J  * T1  * T1 )，当t = t1时
      V1 = (fstart + 0.5 * faa * taa * taa);
    //V2 = V1 + (A_MAX * T2),当t = t2时
      V2 = V1 + A_MAX * tua;

		//根据公式计算从开始到最高速过冲中，t时刻的转动频率
	  if(t >= 0 && t <= taa)
    {
      /*加加速阶段 
             Vs:起始频率，   J:最大加加速度
        V  = Vs     + 1/2 * (J * (τ1)^2  ),0 ≤ t ≤ t1
           = Vs     + 1/2 * (J * (t - 0)^2),0 ≤ t ≤ t1*/
			return fstart + 0.5 * faa * t * t;
		}
    else if(taa < t && t <= (taa + tua))
    {
      /*匀加速阶段 
      V = V1 + a_max * τ2,taa ≤ t ≤ tua
        =    V1 + a_max * (t - taa),taa ≤ t ≤ tua*/
			return V1 + A_MAX * (t - taa);
		}
    else if((taa+tua) < t && t <= (taa+tua+tra))
    {
      /*减加速阶段
      V = V2 + a_max * τ3 - 1/2(J*(τ3)^2),tua ≤ t ≤ tra
        = V2 + faa * taa * (t - (taa+tua)) - 1/2*(faa*((t - (taa+tua))^2))*/
			//源代码 算法
      //return V2 + 0.5*faa*taa*tra - 0.5*faa*taa*(taa+tua+tra-t)*(taa+tua+tra-t)/(tra);
      return V2 + A_MAX * (t - (taa+tua)) - 0.5 * faa * (t - (taa+tua)) * (t - (taa+tua));
    }		
		return 0;
}
/**
  * @brief  计算S型曲线算法的每一步定时器周期及转向数
  * @param  None.
  * @retval None.
  * @note   None.
*/
static void CalcMotorPeriStep_CPF(SMove_7s_TypeDef *p)
{
  int  i;
	float fi;
  
  p->STEP_LENGTH = p->STEP_AA + p->STEP_UA + p->STEP_RA;
  
	for(i = 0; i < p->STEP_AA; i++)
	{
		fi=GetFreAtTime(p->fstart,p->faa,p->taa,p->tua,p->tra,p->taa/p->STEP_AA*i);
		p->MotorTimeTable[i]=fi;
		p->MotorStepTable[i]=fi*(p->taa/p->STEP_AA)/p->STEP_PARA;
	}
	for(i=p->STEP_AA;i<p->STEP_AA+p->STEP_UA;i++)
	{
		fi=GetFreAtTime(p->fstart,p->faa,p->taa,p->tua,p->tra,p->taa+(p->tua/p->STEP_UA)*(i-p->STEP_AA));
		p->MotorTimeTable[i]=fi;
		p->MotorStepTable[i]=fi*(p->tua/p->STEP_UA)/p->STEP_PARA;
	}
	for(i=p->STEP_AA+p->STEP_UA;i<p->STEP_AA+p->STEP_UA+p->STEP_RA;i++)
	{
		fi=GetFreAtTime(p->fstart,p->faa,p->taa,p->tua,p->tra,p->taa+p->tua+p->tra/p->STEP_RA*(i-p->STEP_AA-p->STEP_UA));
		p->MotorTimeTable[i]=fi;
		p->MotorStepTable[i]=fi*(p->tra/p->STEP_RA)/p->STEP_PARA;
	}
	fi=GetFreAtTime(p->fstart,p->faa,p->taa,p->tua,p->tra,p->taa+p->tua+p->tra);//可达到最大速度

	p->MotorTimeTable[p->STEP_AA+p->STEP_UA+p->STEP_RA]=fi;
	p->MotorStepTable[p->STEP_AA+p->STEP_UA+p->STEP_RA]=fi*(p->tra/p->STEP_RA)/p->STEP_PARA;
	
	
	for(i=p->STEP_AA+p->STEP_UA+p->STEP_RA+1;i<2*p->STEP_LENGTH+1;i++)
	{ 
		p->MotorTimeTable[i]=p->MotorTimeTable[2*p->STEP_LENGTH-i];
		p->MotorStepTable[i]=p->MotorStepTable[2*p->STEP_LENGTH-i];
	}
}
/**
  * @brief  MotorRunParaInitial:电机运行参数初始化
  * @param  p:7S加减速结构体
  * @retval None.
  * @note   None.
*/
static void SMove_RunParaInitial(SMove_7s_TypeDef* p)
{ 
  CalcMotorPeriStep_CPF(p); 	
}
/**
  * @brief  重新初始化电机运行时相关参数
  * @param  None.
  * @retval None.
  * @note   None.
*/
void SMove_Reinitial(SMove_Control_TypeDef *pmotor)
{
  assert_param(pmotor);
	pmotor->pulsecount    = 0;
	pmotor->CurrentIndex  = 0;
  pmotor->StartSteps    = 0;                              //必须清零，后面是累加，否则会把前一次的加上
  pmotor->StopSteps     = 0;                              //同上
	pmotor->speedenbale = DISABLE;

	for(int i = 0; i < pmotor->StartTableLength; i++)
	 pmotor->StartSteps += pmotor->Step_Table[i];
	for(int i = 0;i < pmotor->StopTableLength; i++)
	 pmotor->StopSteps += pmotor->Step_Table[i + pmotor->StartTableLength];
}
/**
  * @brief  Initial_Motor  :初始化电机
  * @param  pmotor         :电机控制结构体
  * @param  HChannel       :使用通道
  * @param  Cycle_pulse_num:周期脉冲数
  * @param  Reduction_ratio:减速比
  * @param  Master_timx    :使用主定时器
  * @param  Slave_timx     :使用从定时器
  * @param  HChannel       :使用通道
  * @param  HChannel       :顺时针电平
  * @retval None.
  * @note   None.
*/
void SMove_Initial(SMove_Control_TypeDef *pmotor,SMove_7s_TypeDef *p,uint32_t Cycle_pulse_num,uint16_t Reduction_ratio,TIM_HandleTypeDef* Master_timx,uint32_t HChannel,GPIO_PinState Clockwise
  ,uint32_t *SMove_TimeTable,uint32_t *SMove_StepTable,TIM_HandleTypeDef* Slave_timx)
{
  /* Check the TIM handle allocation */
  assert_param(Master_timx);
  assert_param(Slave_timx);
  assert_param(pmotor);
  assert_param(p);
  
  SMove_RunParaInitial(p);  //初始化加减速表格
  /*设置参数*/
  pmotor->Master_TIMx           = Master_timx;
  pmotor->Channel               = HChannel;
  pmotor->Slave_TIMx            = Slave_timx;
  pmotor->clockwise             = Clockwise;//设置顺时针对应值
  pmotor->reduction_ratio       = Reduction_ratio;
  pmotor->cycle_pulse_num       = Cycle_pulse_num;
  pmotor->MaxPosition           = Cycle_pulse_num;
  pmotor->MaxPosition_Pulse     = pmotor->MaxPosition * pmotor->reduction_ratio;
  /*赋值*/
  p->STEP_LENGTH                = p->STEP_AA + p->STEP_UA + p->STEP_RA;
  pmotor->StartTableLength      = p->STEP_LENGTH;
  pmotor->StopTableLength       = p->STEP_LENGTH; 
  pmotor->Counter_Table         = SMove_TimeTable;//指向启动时，时间基数计数表
  pmotor->Step_Table            = SMove_StepTable;//指向启动时，每个频率脉冲个数表
  pmotor->speedenbale           = DISABLE;
  /*清零*/
  pmotor->CurrentPosition       = 0;
  pmotor->CurrentPosition_Pulse = 0;
  pmotor->CurrentIndex          = 0;
  pmotor->StartSteps            = 0;//必须清零，后面是累加，否则会把前一次的加上
  pmotor->StopSteps             = 0;//同上
  
  pmotor->Modify_Freq(pmotor->Master_TIMx,pmotor->Channel,pmotor->Counter_Table[0] - 1);
  
  pmotor->PulsesGiven = -1;
  SMove_Reinitial(pmotor);
  pmotor->Motor_Enable();
  pmotor->Motor_init(pmotor->Master_TIMx); //初始化电机
}
/*****************************控制电机运行指定角度函数****************************************************/
/**
  * @brief  Start_SMove_S:启动电机按照S型曲线参数运行
  * @param  p:控制参数
  * @param  Dir:旋转方向
  * @param  DestPosition:目标位置
  * @retval None.
  * @note   不能放在while中
            不需要判断是否换向
*/
static void SMove_Start(SMove_Control_TypeDef *p,Directionstate Dir,uint32_t DestPosition)
{
  assert_param(p);
  uint32_t conte_temp  = 0,stop_temp = 0,start_step = 0;
  GPIO_PinState level  = GPIO_PIN_RESET;
  p->MaxPosition  = p->cycle_pulse_num;                      //更新脉冲周期
  p->MaxPosition_Pulse = p->MaxPosition * p->reduction_ratio;//更新脉冲周期
  if(DestPosition==0)return;  
  
  /*返回true,停止电机并退出控制函数
  返回FALSE,继续运行控制函数
  保证状态异常时，无法继续启用电机
  */
  if(p->Motor_Stop_Priority())
  {
    SMove_Stop(p);
    p->running         = DISABLE;
    p->en              = DISABLE;
    return;
  }
 
  p->Motor_Direction(Dir);
  
  if(p->running == DISABLE)//角度更新完成
  {
    p->PulsesHaven = 0;
    p->PulsesGiven = DestPosition;
    p->CurrentIndex = 0;
    
    p->en           = ENABLE;
    p->speedenbale  = DISABLE;
    
    p->set_freq = p->Counter_Table[0];
    p->Modify_Freq(p->Master_TIMx,p->Channel,p->set_freq - 1);
    p->Motor_Start();
    p->running = ENABLE;
  }
  else//角度还在更新 
  {   
    p->speedenbale  = DISABLE;
    switch(p->state)
    {
      case ACCEL:
        if(DestPosition < p->StopSteps)//重设值跑不完减速【会出现不连续 -》》 开始减速值比现在值高。开始减速值比现在值低】
        {
          p->PulsesHaven = p->StartSteps;//当前步长在减速阶段开始
          p->PulsesGiven = DestPosition + p->PulsesHaven;//目标步长个数
          for(int i = 0; i < p->StopTableLength; i ++)
          {
            stop_temp   += p->Step_Table[p->StartTableLength + p->StopTableLength - i - 1];//可以结束步长
            if(stop_temp > DestPosition)
            {
              conte_temp = i;
              p->CurrentIndex= p->StartTableLength + p->StopTableLength - conte_temp;//直接可以到0的减速索引
              p->pulsecount=p->Step_Table[p->CurrentIndex];
              break;
            }
          }
          /*直接跑减速*/
        }
        else//重设值跑得完减速【会出现不连续 -》》 开始减速值比现在值高】
        {
          p->PulsesHaven = p->StartSteps;//当前步长在减速阶段开始
          p->PulsesGiven = DestPosition + p->PulsesHaven;//目标步长个数
          p->CurrentIndex = p->StartTableLength + 1;//匀速不需要改变当前速度
          p->pulsecount=p->Step_Table[p->CurrentIndex];
        }
        break;
      case CONST:
        if(DestPosition < p->StopSteps)//重设值跑不完减速
        {
          p->PulsesHaven = p->PulsesHaven;//当前步长在匀速阶段 -》 减速阶段
          p->PulsesGiven = DestPosition + p->PulsesHaven;//目标步长个数
          for(int i = 0; i < p->StopTableLength; i ++)
          {
            stop_temp   += p->Step_Table[p->StartTableLength + p->StopTableLength - i - 1];//可以结束步长
            if(stop_temp > DestPosition)
            {
              conte_temp = i;
              p->CurrentIndex= p->StartTableLength + p->StopTableLength - conte_temp;//直接可以到0的减速索引
              p->pulsecount=p->Step_Table[p->CurrentIndex];
              break;
            }
          }
          /*直接跑减速*/
        }
        else//重设值跑得完减速
        {
          p->PulsesHaven = p->StartSteps;//当前步长在加速阶段 -》 匀速阶段
          p->PulsesGiven = DestPosition + p->PulsesHaven;//目标步长个数
          p->CurrentIndex = p->CurrentIndex;//匀速不需要改变当前速度
          p->pulsecount=p->Step_Table[p->CurrentIndex];
        }
        break;
      case DECEL:
       if(DestPosition < p->StopSteps)//重设值跑不完减速【会出现不连续 -》》 开始减速值比现在值高。开始减速值比现在值低】
        {
          p->PulsesHaven = p->StartSteps;//当前步长在减速阶段开始
          p->PulsesGiven = DestPosition + p->PulsesHaven;//目标步长个数
          for(int i = 0; i < p->StopTableLength; i ++)
          {
            stop_temp   += p->Step_Table[p->StartTableLength + p->StopTableLength - i - 1];//可以结束步长
            if(stop_temp > DestPosition)
            {
              conte_temp = i;
              p->CurrentIndex= p->StartTableLength + p->StopTableLength - conte_temp;//直接可以到0的减速索引
              p->pulsecount=p->Step_Table[p->CurrentIndex];
              break;
            }
          }
          /*直接跑减速*/
        }
        else//重设值跑得完减速【会出现不连续 -》》 开始减速值比现在值高】
        {
          p->PulsesHaven = p->StartSteps;//当前步长在加速阶段 -》 匀速阶段
          p->PulsesGiven = DestPosition + p->PulsesHaven;//目标步长个数
          p->CurrentIndex = p->StartTableLength + 1;//匀速不需要改变当前速度
          p->pulsecount=p->Step_Table[p->CurrentIndex];
        }
        break;
    }
  }
}
/**
  * @brief  判断电机角度范围
  * @param  SMove_AngleTypeDef 电机角度结构体
  * @param  input ：需要判读的角度
  * @retval None
  * @note   判断是否超出，超出设置为临界值
            若没有设置角度范围，则不判断范围限制，返回输入值
*/
static float Angle_Range_Judgment(SMove_AngleTypeDef *p,float angle)
{
  assert_param(p);
  float input = angle;
  if(p->min_angle == 0 && p->max_angle == 0)
    return input;
  if(p->min_angle <= input && input <= p->max_angle)
  {
    p->over_range = DISABLE;//输入没有超出角度
  }
  else if(input < p->min_angle)
  {
    input = p->min_angle;
    p->over_range = ENABLE;//输入超出角度
    LOG_W("Input beyond minimum Angle");
  }
  else if(input > p->max_angle)
  {
    input = p->max_angle;
    p->over_range = ENABLE;//输入超出角度
    LOG_W("Input beyond maximum Angle");
  }
  return input;
}
/**
  * @brief  返回电机角度（预估值）[进行换算]
  * @param  SMove_Control_TypeDef 电机各项参数
  * @retval 角度
  * @note   根据脉冲个数预估当前电机角度。
*/
float SMove_Get_Angle(SMove_Control_TypeDef* p)
{
  assert_param(p);
  return Angle_Conversion(((float)p->CurrentPosition_Pulse / p->cycle_pulse_num / p->reduction_ratio) * 360);
}
/**
  * @brief  电机绝对角度输出函数
  * @param  SMove_TypeDef 电机各项参数
  * @retval None
  * @note   角度无换算
*/
FunctionalState SMove_SetAngle_Relative(SMove_TypeDef *p,float relative_angle)
{
  assert_param(p);
  
  p->Rel.set_angle = Angle_Conversion(relative_angle);
  
  if(p->Rel.set_angle == 0)
    return p->Ctrl.running;
  else if(p->Rel.set_angle > 0)
  {
    p->Ctrl.dir = CW;
  }
  else  if(p->Rel.set_angle < 0)
  {
    p->Ctrl.dir = CCW;
  }
  
  p->Rel.set_angle = Angle_Range_Judgment(&p->Rel,p->Rel.set_angle);

  p->Rel.dest_position = (fabs(p->Rel.set_angle) / 360) * p->Ctrl.MaxPosition_Pulse;

  SMove_Start(&p->Ctrl,p->Ctrl.dir,p->Rel.dest_position);
  
  return p->Ctrl.running;
}
/**
  * @brief  电机相对角度输出函数
  * @param  SMove_TypeDef 电机各项参数
  * @retval None
  * @note   角度换算 范围【-180~180】
*/
FunctionalState SMove_SetAngle_Absolute(SMove_TypeDef *p,float absolute_angle)
{
  assert_param(p);
  absolute_angle = Angle_Conversion(absolute_angle);
  p->Abs.set_angle = absolute_angle;
  
  /*角度更新判断*/
  p->Abs.err  = (float)(absolute_angle - p->Abs.last);//判断角度有无变化【相对小角度】
  p->Abs.last = absolute_angle;                       //保存上一次角度与当前角度做差
  
  
  /*仅当角度发生变化才进行输出*/
  if(fabs(p->Abs.err) != 0)
  {
    absolute_angle = Angle_Range_Judgment(&p->Abs,absolute_angle);
    p->Abs.get_angle = SMove_Get_Angle(&p->Ctrl);
    return SMove_SetAngle_Relative(p,absolute_angle - p->Abs.get_angle);
  }

  return p->Ctrl.running;
}
/**
  * @brief  设置电机角度范围
  * @param  SMove_AngleTypeDef 电机角度结构体
  * @retval None
  * @note   设置角度范围
*/
void SMove_SetAngle_Range(SMove_AngleTypeDef *p,float max,float min)
{
  assert_param(p);
  p->max_angle = max;
  p->min_angle = min;
}
/**
  * @brief  清除角度缓存
  * @param  SMove_AngleTypeDef 电机角度结构体
  * @retval None
  * @note   绝对角度定位需要记录上一次角度，减少运算量，并避免多次运动
*/
void SMove_SetAngle_Cache(SMove_AngleTypeDef *p)
{
  assert_param(p);
  p->last = 720;//角度范围超过180则会换算，不会设置到720度。
}
/**
  * @brief  电机清除机械位置信息
  * @param  SMove_Control_TypeDef 控制结构体.
  * @retval None.
  * @note   None.
*/
void SMove_SetAxis_Reset(SMove_Control_TypeDef *p)
{
  assert_param(p);
  p->CurrentPosition_Pulse = 0;
}
/**
  * @brief  查询角度是否超出量程
  * @param  SMove_AngleTypeDef 电机角度结构体
  * @retval None
  * @note   None
*/
FunctionalState SMove_GetAngle_Over(SMove_AngleTypeDef *p)
{
  assert_param(p);
  return p->over_range;
}
/*****************************控制电机运行指定速度函数****************************************************/
/**
  * @brief  设置电机运行速度，以RPM单位设置
  * @param  p:控制参数:
  * @param  rpm:转速/每分钟 
  * @param  Stop:停止电机标志位
  * @retval 停止是否完成 0：未完成 1：完成 -1：判断到急停停止 -2:传入参数不变
  * @note   不考虑换向
            带加减速
            加速时只需发送一次。
            减速时需要循环查询判断。
*/
MOVE_state SMove_SetRPM(SMove_TypeDef * p,int16_t rpm,FunctionalState Stop)
{
  if(rpm == 0 && Stop ==DISABLE)//在不是减速模式时，传入速度为0停止
  {
    SMove_Stop(&p->Ctrl);
    return MOVE_STOP;
  }
  
  uint32_t freq = abs(rpm) * p->Ctrl.cycle_pulse_num * p->Ctrl.reduction_ratio / 60;//转/S * 一转所需脉冲数
  int16_t num = Binary_Search(freq,p->Ctrl.Counter_Table,p->Ctrl.StartTableLength-1);
  
  if(num == 0)//为0传入函数无法判断方向
    num = 1;
  
  if(rpm < 0)
    num = -num;

  return SMove_SetSpeed(p,num,Stop);
}
/**
  * @brief  设置电机运行速度
  * @param  p:控制参数:
  * @param  speedindex:速度等级：范围从0~STEP_LENGTH。
  * @param  Stop:停止电机标志位
  * @retval 停止是否完成 0：未完成 1：完成 -1：判断到急停停止 -2:传入参数不变
  * @note   不考虑换向
            带加减速
            加速时只需发送一次。
            减速时需要循环查询判断。
*/
MOVE_state SMove_SetSpeed(SMove_TypeDef *p,int8_t SpeedIndex,FunctionalState Stop)
{
	int currentindex = 0,i = 0;
	unsigned int stepstostop  = 0;
  uint8_t speedindex;
  
  assert_param(p);
  p->Ctrl.MaxPosition = p->Ctrl.cycle_pulse_num;                            //更新脉冲周期
  p->Ctrl.MaxPosition_Pulse = p->Ctrl.MaxPosition * p->Ctrl.reduction_ratio;//更新脉冲周期
  if(p->Ctrl.Motor_Stop_Priority())
    return MOVE_STOP;
  
  if(Stop == 1)//减速并停止电机
  {
    //停止电机,p->Ctrl.CurrentIndex=currentindex-1;直接向下一减速
    p->Ctrl.speedenbale = DISABLE;
    currentindex = p->Ctrl.CurrentIndex; 
    
    if(p->Ctrl.CurrentIndex >= p->Ctrl.StartTableLength )
    {
        currentindex = p->Ctrl.StartTableLength + p->Ctrl.StopTableLength - p->Ctrl.CurrentIndex - 1;
    }
    
    for(i=0;i<currentindex;i++)
    {
       stepstostop += p->Ctrl.Step_Table[i];
    }
    //进入减速index
    if(p->Ctrl.CurrentIndex < p->Ctrl.StartTableLength )
    {
        currentindex = p->Ctrl.StartTableLength + p->Ctrl.StopTableLength - p->Ctrl.CurrentIndex - 1;  
    }
    p->Ctrl.CurrentIndex = currentindex;
    p->Ctrl.pulsecount   = p->Ctrl.Step_Table[p->Ctrl.CurrentIndex];
    p->Ctrl.PulsesGiven  = p->Ctrl.PulsesHaven + stepstostop;
    p->Ctrl.PulsesHaven  = p->Ctrl.PulsesGiven >> 1;
    if(stepstostop == 0)
    {
      SMove_Stop(&p->Ctrl);
      p->Ctrl.running         = DISABLE;
      p->Ctrl.en              = DISABLE;
      return MOVE_DONE;//停止
    }
    else
      return MOVE_UNDONE;
  }
  else if(Stop == 0)
  {
    if(SpeedIndex > 0)
    {
      p->Ctrl.dir = CW;
    }
    else if(SpeedIndex < 0)
    {
      p->Ctrl.dir = CCW;
    }
    
    p->Ctrl.Motor_Direction(p->Ctrl.dir);
    
    speedindex = abs(SpeedIndex);
    
    if(speedindex > p->V.max_speed)
      speedindex = p->V.max_speed;
    else if(speedindex < p->V.min_speed)
      speedindex = p->V.min_speed;
     
    //直接向下一速度
    currentindex = p->Ctrl.CurrentIndex;//当前正在加速阶段，当前位置不做变化

    p->Ctrl.PulsesHaven = 0;//清除运行步数
    if(p->Ctrl.CurrentIndex >= p->Ctrl.StartTableLength)//当前速度在不在加速状态
    {
        currentindex = p->Ctrl.StartTableLength + p->Ctrl.StopTableLength - p->Ctrl.CurrentIndex - 1;
    }
    
    if(currentindex >= speedindex)//当前速度大于等于目标速度
    {
        //需要减速
        p->Ctrl.PulsesGiven = p->Ctrl.PulsesHaven + p->Ctrl.StopSteps - 2;//总步长等于停止步长
    }
    else//当前速度小于目标速度
    {
      //需要加速
      p->Ctrl.PulsesGiven = 0xffffffff;
    }
    p->Ctrl.CurrentIndex = currentindex;
    p->Ctrl.pulsecount   = p->Ctrl.Step_Table[p->Ctrl.CurrentIndex];
    p->Ctrl.TargetIndex  = speedindex;
    p->Ctrl.speedenbale  = ENABLE;
    p->Ctrl.running      = ENABLE;
    p->Ctrl.en           = ENABLE;
    p->Ctrl.Motor_Start();
    return MOVE_UNDONE;
  }
   return -1;
}
/**
  * @brief  设置电机速度范围
  * @param  SMove_SpeedTypeDef 电机速度结构体
  * @retval None
  * @note   设置速度范围
*/
void SMove_SetSpeed_Range(SMove_TypeDef *p,uint32_t max,uint32_t min)
{
  p->V.max_speed = Binary_Search(max * p->Ctrl.cycle_pulse_num * p->Ctrl.reduction_ratio / 60,p->Ctrl.Counter_Table,p->Ctrl.StartTableLength-1);
  p->V.min_speed = Binary_Search(min * p->Ctrl.cycle_pulse_num * p->Ctrl.reduction_ratio / 60,p->Ctrl.Counter_Table,p->Ctrl.StartTableLength-1);
}
/**
  * @brief  返回转向电机速度频率
  * @param  p:控制参数
  * @retval 频率值
  * @note   根据设置频率估计，与实际速率无关。
            放在定时器里 10ms一次。
*/
int32_t SMove_Get_Speed(SMove_Control_TypeDef* p)
{
  assert_param(p);
  static uint32_t num = 0,last_num = 0;
  last_num = num;
  num = p->CurrentPosition_Pulse;
  return num - last_num;
}
/*****************************电机带传感器动作函数****************************************************/
/**
  * @brief  电机复位
  * @param  zero_max:回原高速速度等级【带S型加减速】
  * @param  zero_min:回原低速
  * @param  zero_state:零点状态
  * @param  limit_state:限位状态
  * @retval 回原运行状态
  * @note   None.
*/
ZERO_state SMove_DoReset(SMove_TypeDef *p,int32_t zero_max,int32_t zero_min,SENSOR_state zero_state, SENSOR_state limit_state)
{
  assert_param(p);
  uint8_t speed_stop = 0;//速度模式停止电机标志 0:不减速，1：开始减速，2：减速完成

  p->Do.speed_high   = zero_max;
  p->Do.speed_low    = zero_min;
  
  if(limit_state == SENSOR_TOUCH)
  {
    SMove_Stop(&p->Ctrl);
    p->Do.flag = ZERO_ABNORMAL;
    return p->Do.flag;
  }
  
  if(zero_state == SENSOR_LEAVE)//没碰到
  {
    if(p->Do.flag != LOW_LEAVE)//没碰到高速
    {
      p->V.set_speed = p->Do.speed_high;
      speed_stop = 0;
      p->Do.flag = QUICK_TOUCH;
    }
    if(p->Do.flag == LOW_LEAVE)//结束
    {
      p->Ctrl.running = DISABLE;
      speed_stop =1;
      p->V.set_speed = 0;
    }
  }
  else if(zero_state == SENSOR_TOUCH)//碰到
  {  
    p->V.set_speed = p->Do.speed_low;
    speed_stop = 0;
    p->Do.flag = LOW_LEAVE;//碰到低速
  }
  else
  {
    SMove_Stop(&p->Ctrl);
    p->Do.flag = ZERO_ABNORMAL;
    return p->Do.flag;
  }
  MOVE_state ret = SMove_SetRPM(p,p->V.set_speed,speed_stop);
  if(ret == MOVE_DONE)//减速完成
  {
    p->Do.flag = ZERO_OK;//复位完成
  }
  else if(ret == MOVE_STOP)
  {
    p->Do.flag = ZERO_ABNORMAL;
  }
  
  return p->Do.flag;
}
/**
  * @brief  电机限制监控函数
  * @param  mode:1：判断限位传感器。2：归零传感器。3：一起判断.4，智能限位。0不判断
  * @param  SMove_TypeDef* p 电机各项参数
  * @param  zero_state：限位传感器状态
  * @param  limit_state 零位传感器状态
  * @retval SMove_MONITOR_state.
  * @note   None.
*/
LIMIT_MONITOR_STATE SMove_Limit_Detection(SMove_TypeDef* p,uint8_t mode,SENSOR_state zero_state, SENSOR_state limit_state)
{
  if(limit_state == SENSOR_TOUCH && zero_state == SENSOR_TOUCH)
  {
      SMove_Stop(&p->Ctrl);
      return MONITOR_ALL;//异常状态，不可能同时触碰两个传感器
  }
  
  switch(mode)
  {
    case 0://不做判断
      break;
    case 1:
      if(limit_state == SENSOR_TOUCH || limit_state == SENSOR_TIMEOUT)
      {
         SMove_Stop(&p->Ctrl);
         return MONITOR_LIMIT;
      }
      else if(limit_state == SENSOR_LEAVE || limit_state == SENSOR_INIT)
      {
         return MONITOR_NORMAL;
      }
      break;
    case 2:
      if(zero_state == SENSOR_TOUCH || zero_state == SENSOR_TIMEOUT)
      {
         SMove_Stop(&p->Ctrl);
         return MONITOR_ZERO;
       }
      else if(zero_state == SENSOR_LEAVE || zero_state == SENSOR_INIT)
      {
         return MONITOR_NORMAL;
      }
      break;
    case 3:
      if(zero_state == SENSOR_TOUCH   ||  zero_state == SENSOR_TIMEOUT)
      {
         SMove_Stop(&p->Ctrl);
         return MONITOR_ZERO;
      }
      else if(limit_state == SENSOR_TIMEOUT || limit_state == SENSOR_TOUCH)//碰到
      {
         SMove_Stop(&p->Ctrl);
         return MONITOR_LIMIT;
      }
      else if(zero_state == SENSOR_LEAVE && limit_state == SENSOR_LEAVE)
      {
         return MONITOR_NORMAL;
      }
      else
      {
          SMove_Stop(&p->Ctrl);
          return MONITOR_ABNORMAL;//异常状态
      }
      break;
    case 4:
      if(p->Ctrl.dir == CCW)
      {
        if(zero_state == SENSOR_TOUCH || zero_state == SENSOR_TIMEOUT)
        {
           SMove_Stop(&p->Ctrl);
           return MONITOR_ZERO;
        }
        else if(zero_state == SENSOR_LEAVE || zero_state == SENSOR_INIT)
        {
           return MONITOR_NORMAL;
        }
      }
      else if(p->Ctrl.dir == CW)
      {
        if(limit_state == SENSOR_TOUCH || limit_state == SENSOR_TIMEOUT)
        {
           SMove_Stop(&p->Ctrl);
           return MONITOR_LIMIT;
        }
        else if(limit_state == SENSOR_LEAVE || limit_state == SENSOR_INIT)
        {
           return MONITOR_NORMAL;
        }
      }
      else
      {
        if(zero_state == SENSOR_TOUCH   ||  zero_state == SENSOR_TIMEOUT)
        {
           SMove_Stop(&p->Ctrl);
           return MONITOR_ZERO;
        }
        else if(limit_state == SENSOR_TIMEOUT || limit_state == SENSOR_TOUCH)//碰到
        {
           SMove_Stop(&p->Ctrl);
           return MONITOR_LIMIT;
        }
        else if(zero_state == SENSOR_LEAVE && limit_state == SENSOR_LEAVE)
        {
           return MONITOR_NORMAL;
        }
        else
        {
            SMove_Stop(&p->Ctrl);
            return MONITOR_ABNORMAL;//异常状态
        }
      }
      break;
    default://传入错误模式
      SMove_Stop(&p->Ctrl);
      return MONITOR_ABNORMAL;//异常状态
      break;
  }
  return MONITOR_INIT;
}