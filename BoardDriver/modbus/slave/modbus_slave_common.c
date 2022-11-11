/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : 
  * @brief          : 
  * @date           :
  ******************************************************************************
  * @attention
  * @author
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "modbus_slave_common.h"
/* Private includes ----------------------------------------------------------*/
#include "gpio.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
extern void modbus_slave_input_register_write(void);
extern void modbus_slave_register_write(void);
/**
  * @brief  对共享地址写入本机数据
  * @param  None
  * @retval None
  * @note   None
*/
void modbus_slave_write(void)
{
    modbus_slave_input_register_write();
    modbus_slave_register_write();
    HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
}
/**
  * @brief  打印MODBUS列表
  * @param  第一个参数，int型的argc，为整型，用来统计程序运行时发送给main函数的命令行参数的个数
  * @retval 第二个参数，char*型的argv[]，为字符串数组，用来存放指向的字符串参数的指针数组，每一个元素指向一个参数。
  * @note   None.
*/
static void modbus_get(int argc, char**argv)
{
#define MODBUS_LIST                   0
#define MODBUS_EG                     1
#define MODBUS_TYPE                   2
#define MODBUS_EN                     3
  int16_t len,start,cnt = 0,flag = 0;
  const char* help_info[] =
    {
        [MODBUS_LIST]           = "Please input'modbus_get <type|start_addr|lenth>'",
        [MODBUS_EG]             = "       e.g : modbus_get  hold    100       10.",
        [MODBUS_TYPE]           = "      type : hold | coil | input | in_bit.",
        [MODBUS_EN]             = "      EN   : EN is Only look at not zero data.",
    };

  if (argc < 3)
  {
      rt_kprintf("Usage:\n");
      for (size_t i = 0; i < sizeof(help_info) / sizeof(char*); i++)
      {
          rt_kprintf("%s\n", help_info[i]);
      }
      rt_kprintf("\n");
      return;
  }

  if(strspn(argv[2], "0123456789") == strlen(argv[2]))//判断传入字符串是否全数字
  {
    start = atoi(argv[2]);
  }
  else
  {
    rt_kprintf("Please enter only digits\n");
    return;
  }
  
  if(strspn(argv[3], "0123456789") == strlen(argv[3]))//判断传入字符串是否全数字
  {
    len = atoi(argv[3]);
  }
  else
  {
    rt_kprintf("Please enter only digits\n");
    return;
  }
  if(argv[4])
  {
    if(!rt_strcmp(argv[4],"en"))
    {
      flag = 1;
    }
    else
    {
      rt_kprintf("Please enter en\n");
    }
  }
  if(!rt_strcmp(argv[1], "hold"))
  {
    if(start+len > MODBUS_REG_MAX_NUM)
    {
      rt_kprintf("Input out of index range,The value ranges from 1 to %d\n",MODBUS_REG_MAX_NUM);
      return;
    }
    for(uint16_t i = start;i < start+len;i++)
    {
      if(flag == 1)
      {
        if(modbus_register_get(0,i) != 0)
        {
          if(cnt % 5 == 0)
          {
            rt_kprintf("\r\n");
            cnt = 0;
          }
          cnt++;
          rt_kprintf("num:%3d|hold:%5d#",i,modbus_register_get(0,i));
        }
      }
      else
      {
        if(cnt % 5 == 0)
        {
          rt_kprintf("\r\n");
          cnt = 0;
        }
        cnt++;
        rt_kprintf("num:%3d|hold:%5d#",i,modbus_register_get(0,i));
      }
    }
    rt_kprintf("\r\n");
  }
  else if(!rt_strcmp(argv[1], "input"))
  {
    if(start+len > MODBUS_REG_MAX_NUM)
    {
      rt_kprintf("Input out of index range,The value ranges from 1 to %d\n",MODBUS_REG_MAX_NUM);
      return;
    }
    for(uint16_t i = start;i < start+len;i++)
    {
      if(flag == 1)
      {
        if(modbus_input_register_get(0,i) != 0)
        {
          if(cnt % 5 == 0)
          {
            rt_kprintf("\r\n");
            cnt = 0;
          }
          cnt++;
          rt_kprintf("num:%3d|input:%5d#",i,modbus_input_register_get(0,i));
        }
      }
      else
      {
        if(cnt % 5 == 0)
        {
          rt_kprintf("\r\n");
          cnt = 0;
        }
        cnt++;
        rt_kprintf("num:%3d|input:%5d#",i,modbus_input_register_get(0,i));
      }
    }
    rt_kprintf("\r\n");
  }
  else if (!rt_strcmp(argv[1], "coil"))
  {
    if(start+len > MODBUS_BIT_MAX_NUM)
    {
      rt_kprintf("Input out of index range,The value ranges from 1 to %d\n",MODBUS_BIT_MAX_NUM);
      return;
    }
    for(uint16_t i = start;i < start+len;i++)
    {
      if(flag == 1)
      {
        if( modbus_bits_get(0,i)!=0)
        {
          if(cnt % 7 == 0)
          {
            rt_kprintf("\r\n");
            cnt = 0;
          }
          cnt++;
          rt_kprintf("num:%3d|coil:1#",i);
        }
      }
      else
      {
        if(cnt % 7 == 0)
        {
          rt_kprintf("\r\n");
          cnt = 0;
        }
        cnt++;
        if(modbus_bits_get(0,i))
        {
          rt_kprintf("num:%3d|coil:1#",i);
        }
        else
        {
          rt_kprintf("num:%3d|coil:0#",i);
        }
      }
    }
    rt_kprintf("\r\n");
  }
  else if (!rt_strcmp(argv[1], "in_bit"))
  {
    if(start+len > MODBUS_BIT_MAX_NUM)
    {
      rt_kprintf("Input out of index range,The value ranges from 1 to %d\n",MODBUS_BIT_MAX_NUM);
      return;
    }
    for(uint16_t i = start;i < start+len;i++)
    {
      if(flag == 1)
      {
        if( modbus_input_bits_get(0,i)!=0)
        {
          if(cnt % 7 == 0)
          {
            rt_kprintf("\r\n");
            cnt = 0;
          }
          cnt++;
          rt_kprintf("num:%3d|bits:1#",i);
        }
      }
      else
      {
        if(cnt % 7 == 0)
        {
          rt_kprintf("\r\n");
          cnt = 0;
        }
        cnt++;
        if(modbus_input_bits_get(0,i))
        {
          rt_kprintf("num:%3d|bits:1#",i);
        }
        else
        {
          rt_kprintf("num:%3d|bits:0#",i);
        }
      }
    }
    rt_kprintf("\r\n");
  }
  else
  {
      rt_kprintf("Usage:\n");
      for (size_t i = 0; i < sizeof(help_info) / sizeof(char*); i++)
      {
          rt_kprintf("%s\n", help_info[i]);
      }
      rt_kprintf("\n");
      return;
  }
}
MSH_CMD_EXPORT_ALIAS(modbus_get,modbus_get,modbus_get <type|start_addr|lenth>);
/**
  * @brief  设置MODBUS数据
  * @param  第一个参数，int型的argc，为整型，用来统计程序运行时发送给main函数的命令行参数的个数
  * @retval 第二个参数，char*型的argv[]，为字符串数组，用来存放指向的字符串参数的指针数组，每一个元素指向一个参数。
  * @note   None.
*/
static void modbus_set(int argc, char**argv)
{
  int16_t num,addr;
  if (argc < 3)
  {
      rt_kprintf("Please input'modbus_set <type|addr|num>'\n");
      rt_kprintf("       e.g : modbus_set  hold 100  10\n");
      rt_kprintf("       type: hold | coil\r\n");
      return;
  }
  if(rt_strcmp(argv[1], "hold") && rt_strcmp(argv[1], "coil"))
  {
     rt_kprintf("Input out of index range,The value ranges from 1 to %d\n",MODBUS_REG_MAX_NUM);
    return;
  }
  if(strspn(argv[2], "0123456789") == strlen(argv[2]))//判断传入字符串是否全数字
  {
    addr = atoi(argv[2]);
  }
  else
  {
    rt_kprintf("Please enter only digits\n");
    return;
  }
  
  if(strspn(argv[3], "0123456789") == strlen(argv[3]))//判断传入字符串是否全数字
  {
    num = atoi(argv[3]);
  }
  else
  {
    rt_kprintf("Please enter only digits\n");
    return;
  }
  if(!rt_strcmp(argv[1], "hold"))
  {
    if(addr > MODBUS_REG_MAX_NUM || addr < 0)
    {
      rt_kprintf("Input out of index range\r\n");
      return;
    }
    modbus_register_set(0,addr,num);
    rt_kprintf("usRegHoldingBuf[%d] set value:%d\r\n",addr,num);
  }
  else if (!rt_strcmp(argv[1], "coil"))
  {
    if(addr > MODBUS_BIT_MAX_NUM || addr < 0)
    {
      rt_kprintf("Input out of index range,The value ranges from 1 to %d\n",MODBUS_BIT_MAX_NUM);
      return;
    }
    if(num != 0 && num != 1)
    {
      rt_kprintf("Please enter 0 or 1\n");
    }
    modbus_bits_set(0,addr,num);
    rt_kprintf("ucRegCoilsBuf[%d] set value:%d\r\n",addr,num);
  }
  else
  {
      rt_kprintf("Please input'modbus_set <type|addr|num>'\n");
      rt_kprintf("       e.g : modbus_set  hold 100  10\n");
      rt_kprintf("       type: hold | coil\r\n");
      return;
  }
}
MSH_CMD_EXPORT_ALIAS(modbus_set,modbus_set,modbus_set <type|addr|num>);