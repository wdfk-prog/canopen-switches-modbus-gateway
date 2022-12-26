#include <rtthread.h>
#include <rtdevice.h>
#include <main.h>

#define ADC_DEV_NAME        "adc1"      /* ADC 设备名称 */
#define ADC_VREF_CHANNEL     ADC_CHANNEL_VREFINT - ADC_CHANNEL_0    /* ADC Vref 通道 */
#define ADC_TEMP_CHANNEL     ADC_CHANNEL_TEMPSENSOR - ADC_CHANNEL_0 /* ADC Temp 通道 */

static int adc_vol_sample(void)
{
    rt_uint32_t vref_value,temp_value;
    rt_uint16_t vref_mv,temp_mv; 

    rt_adc_device_t adc_dev;
    rt_err_t ret = RT_EOK;
    
    /* 查找设备 */
    adc_dev = (rt_adc_device_t)rt_device_find(ADC_DEV_NAME);
    if (adc_dev == RT_NULL)
    {
        rt_kprintf("adc sample run failed! can't find %s device!\n", ADC_DEV_NAME);
        return RT_ERROR;
    }

    /* 使能设备 */
    ret = rt_adc_enable(adc_dev, ADC_VREF_CHANNEL);
    /* 读取采样值 */
    vref_value = rt_adc_read(adc_dev, ADC_VREF_CHANNEL);
    /* 关闭通道 */
    ret = rt_adc_disable(adc_dev, ADC_VREF_CHANNEL);

    /* 使能设备 */
    ret = rt_adc_enable(adc_dev, ADC_TEMP_CHANNEL);
    /* 读取采样值 */
    temp_value = rt_adc_read(adc_dev, ADC_TEMP_CHANNEL);
    /* 关闭通道 */
    ret = rt_adc_disable(adc_dev, ADC_TEMP_CHANNEL);
  
    rt_kprintf("Vref  is %u.\n", vref_value);
    rt_kprintf("Temp is %u.\n" , temp_value);
    
    // Calculating Vref voltage
    vref_mv = __LL_ADC_CALC_VREFANALOG_VOLTAGE(vref_value, ADC_RESOLUTION_12B);
    rt_kprintf("Vref voltage is %u mV.\n", vref_mv);
    // Calculate Temperature
    rt_kprintf("%d are Temperature in Degree C.\n",
    __LL_ADC_CALC_TEMPERATURE_TYP_PARAMS(2500,760,10,vref_mv, temp_value, ADC_RESOLUTION_12B));
    return RT_EOK;
}
/* 导出到 msh 命令列表中 */
MSH_CMD_EXPORT(adc_vol_sample, adc voltage convert sample);