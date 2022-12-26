/*
 * 程序清单： ADC 设备使用例程
 * 例程导出了 adc_sample 命令到控制终端
 * 命令调用格式：adc_sample
 * 程序功能：通过 ADC 设备采样电压值并转换为数值。
 *           示例代码参考电压为3.3V,转换位数为12位。
*/

#include <rtthread.h>
#include <rtdevice.h>

#define ADC_DEV_NAME        "adc1"      /* ADC 设备名称 */
#define ADC_DEV_CHANNEL     1           /* ADC 通道 */
#define REFER_VOLTAGE       330         /* 参考电压 3.3V,数据精度乘以100保留2位小数*/
#define CONVERT_BITS        (1 << 12)   /* 转换位数为12位 */

/**
 * @brief 新版例程使用
 * @param  argc             
 * @param  argv             
 * @retval int 
 */
static int adc_vol_sample(int argc, char *argv[])
{
    rt_device_t adc_dev;
    rt_uint32_t value, vol;
    rt_err_t ret = RT_EOK;

    /* 查找设备 */
    adc_dev = rt_device_find(ADC_DEV_NAME);
    if (adc_dev == RT_NULL)
    {
        rt_kprintf("adc sample run failed! can't find %s device!\n", ADC_DEV_NAME);
        return RT_ERROR;
    }

    /* 以DMA rx方式打开 adc 设备 */
    ret = rt_device_open(adc_dev,RT_DEVICE_FLAG_DMA_RX);
		/* 设置 adc */
		ret = rt_device_control(adc_dev, RT_DEVICE_CTRL_CONFIG, (void *)CAN500kBaud);
    /* 读取采样值 */
    rt_device_read(adc_dev,ADC_DEV_CHANNEL,&value,1);
    rt_kprintf("the value is :%d \n", value);

    /* 转换为对应电压值 */
    vol = value * REFER_VOLTAGE / CONVERT_BITS;
    rt_kprintf("the voltage is :%d.%02d \n", vol / 100, vol % 100);

    /* 关闭通道 */
    ret = rt_adc_disable((rt_adc_device_t)adc_dev, ADC_DEV_CHANNEL);
		/* 关闭设备 */
		ret = rt_device_close(adc_dev);
    return ret;
}
/* 导出到 msh 命令列表中 */
MSH_CMD_EXPORT(adc_vol_sample, adc voltage convert sample);