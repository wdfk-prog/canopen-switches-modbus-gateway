/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : adc_dma.c
  * @brief          : adc_dma program body
  ******************************************************************************
  * @attention
  * F7 开启L1-cache会导致DMA数据不更新。例如串口，adc
  * https://bbs.21ic.com/icview-2380632-1-1.html
  * 如果程序默认内存地址设置为0x20020000开始，即运行于SRAM1，才会出现这种现象。
  * 如果默认内存地址为0x20000000开始，即运行于DTCM，结果是正常的，开启D-cache正常。
  * DMA如果使用了SRAM1，要么就不使用D-cache，要使用D-cache就要手动clean，或者设置为write-through
  * 另一种解决办法如本代码所示，
  * https://blog.csdn.net/weifengdq/article/details/121802176
    开启CACHE加速 需要设置IRAM1从0x20000000开始
                           使用从0x20200000 会导致DMA错误   2022.03.21
  * ADC转换速率越高 ，波动越大，容易丢失精度
  * 非必要，尽量使用最大周期采样
  * F7 必需启用 EOC flag at the end of all conversions，否则测量不准确2022.04.16
    实现LED报警闪烁，可以实现正常后恢复原来闪烁状态                     2022.04.18
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "adc_dma.h"
/* Private includes ----------------------------------------------------------*/
#include "adc.h"
#include "mb_handler.h"
#include "turn_motor.h"
#include "walk_motor.h"
/* Private typedef -----------------------------------------------------------*/
/** 
  * @brief ADC1通道值定义
  */  
typedef enum
{
  ADC_Channel = 0x00,
  Vrefint_Channel,
  Encoder_Channel,
}ADC1_CHANNEL;
/* Private define ------------------------------------------------------------*/
#define ADC1_CHANNEL_NUM 3  //ADC1用了2通道
#define ADC1_BUFFER_SIZE 8 //每个通道存32组, 方便做平均
#define ADC1_BUFFER_LEN  ADC1_BUFFER_SIZE*ADC1_CHANNEL_NUM
#define VABBT_LONG_TIME  30//低电平持续时间判断
/* Private macro -------------------------------------------------------------*/
/*
@Note If the  application is using the DTCM/ITCM memories (@0x20000000/ 0x0000000: not cacheable and only accessible
      by the Cortex M7 and the  MDMA), no need for cache maintenance when the Cortex M7 and the MDMA access these RAMs.
      If the application needs to use DMA(or other masters) based access or requires more RAM, then  the user has to:
              - Use a non TCM SRAM. (example : D1 AXI-SRAM @ 0x24000000)
              - Add a cache maintenance mechanism to ensure the cache coherence between CPU and other masters(DMAs,DMA2D,LTDC,MDMA).
              - The addresses and the size of cacheable buffers (shared between CPU and other masters)
                must be	properly defined to be aligned to L1-CACHE line size (32 bytes). 
*/
//32字节对齐(地址+大小)
//adc1_data指定到 AXI SRAM 的0x20020000 //设置不正确会进不了debug
//adc3_data指定到 SRAM4 的0x38000000    //
//ALIGN_32BYTES (__IO uint16_t adc1_data[ADC1_BUFFER_SIZE]) __attribute__((section(".ARM.__at_0x20020000")));
//ALIGN_32BYTES (uint16_t adc3_data[ADC3_BUFFER_SIZE]) __attribute__((section(".ARM.__at_0x38000000")));
/* Private variables ---------------------------------------------------------*/
uint16_t adc1_data[ADC1_BUFFER_LEN];
uint16_t Vrefint_vaule = 0;   //内部参考电压值
ADC_Monitor_TypeDef   voltage;//电池电压
Rope_Encoder_TypeDef  lifter_encoder; //4-20ma拉绳编码器
/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  获取芯片内部参考电压ADC采样值
  * @param  None
  * @retval None
  * @note   内部电压较稳定，做平均值既可，采用滤波增加延时
*/

static float Get_Vrefint_Value(void)
{
  float    Vrefint_K     = 0;//转换系数 约等于 3.3 / 4095
  uint32_t sum           = 0;
  for(int i = Vrefint_Channel; i < ADC1_BUFFER_LEN; i += ADC1_CHANNEL_NUM)
  {
    sum += adc1_data[i];
  }
  Vrefint_vaule = sum / ADC1_BUFFER_SIZE;
  
  Vrefint_K = 1.2 / Vrefint_vaule;//内部电压参考值为1.2V
  
  return Vrefint_K;
}
/**
  * @brief  返回adc值
  * @param  None
  * @retval None
  * @note   None
*/

static float Adc_Get_Value(void)
{
  float adc_value = 0;
  for(int i = ADC_Channel; i < ADC1_BUFFER_LEN; i += ADC1_CHANNEL_NUM)
  {
    adc_value = First_Order_Lag(&voltage.AD.FOL,adc1_data[i] * Get_Vrefint_Value());
  }
  return adc_value;
}
/**
  * @brief  电池报警检测
  * @param  None
  * @retval None
  * @note   放入定时中断1000MS一次
*/
static void Battery_Alarm_Detection(void)
{
  static uint16_t fifter_count = 0;
  voltage.AD.vaule = Adc_Get_Value() + voltage.AD.Compensation;
  if(voltage.AD.vaule <= (voltage.VPT) / ADC_RATIO)
  {
    if(++fifter_count > VABBT_LONG_TIME)//低电量持续30秒报警
    {
      //报警
      ADC_ALARM_SET;//ADC报警位
      fifter_count = 0;
      USER_SET_BIT(turn.Stop_state,VBATT_STOP);
      USER_SET_BIT(walk.Stop_state,VBATT_STOP);
    }
  }
  else if(voltage.AD.vaule >= (voltage.VPT + 0.5f) / ADC_RATIO)
  {
    fifter_count = 0;
    ADC_ALARM_RESET;//ADC报警位
    USER_CLEAR_BIT(turn.Stop_state,VBATT_STOP);
    USER_CLEAR_BIT(walk.Stop_state,VBATT_STOP);
  }
}
/***********************************4-20ma拉绳编码器******************************************/
/**
  * @brief  初始化拉绳编码器
  * @param  None
  * @retval None
  * @note   None
*/
static void Rope_Encoder_Init(void)
{
  lifter_encoder.Current.AD.FOL.K = 0.9;
  lifter_encoder.state = SENSOR_INIT;
  lifter_encoder.height_4ma  = ROPE_4MA_HEIGHT; //单位mm
  lifter_encoder.height_20ma = ROPE_20MA_HEIGHT;//单位mm
  lifter_encoder.Current.AD.Compensation = 0.01;
}
/**
  * @brief  设置拉绳编码器4ma位置.
  * @param  None.
  * @retval None.
  * @note   None.
*/
void Rope_Encoder_Set_4mA(void)
{
  HAL_GPIO_WritePin(SET4MA_GPIO_Port,SET4MA_Pin,GPIO_PIN_SET);
  lifter_encoder.state = SENSOR_INIT;
}
/**
  * @brief  设置拉绳编码器4ma位置.
  * @param  None.
  * @retval None.
  * @note   None.
*/
void Rope_Encoder_Set_20mA(void)
{
  HAL_GPIO_WritePin(SET20MA_GPIO_Port,SET20MA_Pin,GPIO_PIN_SET);
  lifter_encoder.state = SENSOR_INIT;
}
/**
  * @brief  设置拉绳编码器位置完成
  * @param  None.
  * @retval None.
  * @note   放入1m一次循环.
*/
static void Rope_Encoder_Set_Low(void)
{
  HAL_GPIO_WritePin(SET4MA_GPIO_Port,SET4MA_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SET20MA_GPIO_Port,SET20MA_Pin,GPIO_PIN_RESET);
  LIFT_ENCODER_4MA_RESET;
  LIFT_ENCODER_20MA_RESET;
}
/**
  * @brief  返回拉绳编码器adc值
  * @param  None
  * @retval None
  * @note   None
*/
static float Rope_Encoder_ADC_Get_Value(void)
{
  float adc_value = 0;
  for(int i = Encoder_Channel; i < ADC1_BUFFER_LEN; i += ADC1_CHANNEL_NUM)
  {
    adc_value = First_Order_Lag(&lifter_encoder.Current.AD.FOL,adc1_data[i] * Get_Vrefint_Value());
  }
  return adc_value + lifter_encoder.Current.AD.Compensation;
}
/**
  * @brief  返回拉绳编码器电流值
  * @param  None
  * @retval None
  * @note   4~20ma转换为电压640mV~3200mV
VOUT / ILOOP = K3 * (R5 * R3) / R1 + R3
             = （200k * 30）/ 1.54k + 30
             ≈ 160    参数不可修改，调整硬件可调电阻
K3 = K2 / K1 = Constant = 1
电压范围在0.64~3.26V
*/
static float Rope_Encoder_Get_Current(void)
{
  lifter_encoder.Current.AD.vaule = (Rope_Encoder_ADC_Get_Value());//电压
  return lifter_encoder.Current.AD.vaule * 1000000 / 160;//电流(uA)
}
/**
  * @brief  返回拉绳编码器位置
  * @param  None
  * @retval None
  * @note   4~20ma电流环 16ma量程
*/
void Rope_Encoder_Get_Postion(void)
{
  uint16_t range = lifter_encoder.height_20ma - lifter_encoder.height_4ma;
  
  if(range <= 0)
  {
    //报警 传感器定义错误
    lifter_encoder.state = SENSOR_LEAVE;
    return;
  }
  
  float dpi = range / 16000.0f;//分辨率 位置范围 / 16000
  
  lifter_encoder.Current.value = Rope_Encoder_Get_Current();
 
  if(0 <= lifter_encoder.Current.value  && lifter_encoder.Current.value < 2000)//0~4ma之间认为掉线 。 1ma误差范围
  {
    //报警 传感器掉线
    lifter_encoder.state = SENSOR_TIMEOUT;
    LIFT_ENCODER_RESET;
    LIFT_ENCODER_ALARM_SET;
  }
  else if(2000 <= lifter_encoder.Current.value  && lifter_encoder.Current.value <= 22000)//一般认为2ma~22ma之间为正常
  {
    //传感器在线
    lifter_encoder.state = SENSOR_TOUCH;
    LIFT_ENCODER_SET;
    if(lifter_encoder.Current.value < 4000)//高度最低为0，无负数，小于4ma即为0高度
      lifter_encoder.Current.value = 4000;
    lifter_encoder.position = (dpi * (lifter_encoder.Current.value - 4000) + lifter_encoder.height_4ma);
  }
  else
  {
    //报警 传感器电路故障
    lifter_encoder.state = SENSOR_LEAVE;
    LIFT_ENCODER_RESET;
    LIFT_ENCODER_ALARM_SET;
  }
}
/**
  * @brief  ADC 1S.
  * @param  None.
  * @retval None.
  * @note   None.
*/
static void adc_1s(void* p)
{
  Rope_Encoder_Set_Low();
  Battery_Alarm_Detection();
}

/**
  * @brief  adc_dma初始化
  * @param  None
  * @retval None
  * @note   函数放在初始化阶段
*/
int ADC_DMA_Init(void)
{
  MX_ADC1_Init();
  if(HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_data, ADC1_BUFFER_LEN) != HAL_OK)
  {
    Error_Handler();
  }
  voltage.AD.FOL.K = 0.5;
  Rope_Encoder_Init();
  rt_err_t ret = RT_EOK;
  /* 定时器的控制块 */
  static rt_timer_t timer;
  /* 创建定时器 1  周期定时器 */
  timer = rt_timer_create("ADC 1s", adc_1s,
                           RT_NULL, rt_tick_from_millisecond(1000),
                           RT_TIMER_FLAG_PERIODIC);

  /* 启动定时器 */
  if (timer != RT_NULL) 
  {
    rt_timer_start(timer);
    ret = RT_EOK;
  }
  else
  {
    ret = RT_ERROR;
  }
  return ret;
}
INIT_APP_EXPORT(ADC_DMA_Init);
/**
  * @brief  ADC_DMA 半满回调函数
  * @param  None
  * @retval None
  * @note   ADC转换半满中断中把数据存到数组的前半部分
            ping-pong存储
            FIFO
*/
//void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
//{
//  /* Invalidate Data Cache to get the updated content of the SRAM on the first half of the ADC converted data buffer */
//  if(hadc->Instance == ADC1) {
//      SCB_InvalidateDCache_by_Addr((uint32_t *) &adc1_data[0], ADC1_BUFFER_LEN);
//  } 
//}
/**
  * @brief  ADC_DMA 转换完成回调函数
  * @param  None
  * @retval None
  * @note   ADC转换完成中断中把数据存到数组的后半部分
*/
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
//{
//   /* Invalidate Data Cache to get the updated content of the SRAM on the second half of the ADC converted data buffer */
//   if(hadc->Instance == ADC1) 
//  {
//       SCB_InvalidateDCache_by_Addr((uint32_t *) &adc1_data[ADC1_BUFFER_LEN/2], ADC1_BUFFER_LEN);
//   } 
//}