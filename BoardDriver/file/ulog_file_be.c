/**
 * @file ulog_file_be.c
 * @brief 
 * @author HLY (1425075683@qq.com)
 * @version 1.0
 * @date 2022-11-17
 * @copyright Copyright (c) 2022
 * @attention 
 * @par 修改日志:
 * Date       Version Author  Description
 * 2022-11-17 1.0     HLY     first version
 */
/* Includes ------------------------------------------------------------------*/
#include "ulog_file_be.h"
#include <ulog_be.h>
/* Private includes ----------------------------------------------------------*/
#include "main.h"
/* Private typedef -----------------------------------------------------------*/
/*
* 后端注册表
*/
struct _log_file
{
    const char *name; 
    ulog_backend_t backend;
    struct ulog_file_be *file_be;
    const char *dir_path; 
    rt_size_t max_num;
    rt_size_t max_size;
    rt_size_t buf_size;
};
/*
* 文件后端标识
*/
typedef enum 
{
    console_id,
    sys_id,
    motion_id,
}ulog_file_be_name;
/* Private define ------------------------------------------------------------*/
#define ROOT_PATH "/flash/log"
#define FILE_SIZE 512 * 1024
#define BUFF_SIZE 512
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static struct ulog_backend sys_log_backend;
static struct ulog_file_be sys_log_file;
static struct ulog_backend motion_log_backend;
static struct ulog_file_be motion_log_file;
static struct ulog_file_be console_log_file;

static struct _log_file table[] =
{
    {"console"  ,RT_NULL,&console_log_file,                                    },
    {"sys"      ,&sys_log_backend,&sys_log_file,ROOT_PATH,10,FILE_SIZE,BUFF_SIZE},
    {"motion"   ,&motion_log_backend,&motion_log_file,ROOT_PATH,5,FILE_SIZE,BUFF_SIZE,},
};
/* Private function prototypes -----------------------------------------------*/
/************************系统日志文件后端操作函数*****************************************/
/**
  * @brief  系统日志文件后端滤波器设置.
  * @param  None.
  * @retval The filter will be call before output. It will return TRUE when the filter condition is math.
  * @note   None.
*/
static rt_bool_t sys_log_file_backend_filter(struct ulog_backend *backend, rt_uint32_t level, const char *tag, rt_bool_t is_raw, 
                                      const char *log, rt_size_t len)
{
    if (rt_strncmp(tag,MOTION_TAG, sizeof(MOTION_TAG)) == 0)//排除带有"MOVE"标签输出
      return RT_FALSE;
    else
      return RT_TRUE;
}
/**
  * @brief  系统日志文件后端初始化.
  * @param  None.
  * @retval None.
  * @note   None.
*/
void sys_log_file_backend_init(void)
{
    struct ulog_file_be *file_be = &sys_log_file;
    uint8_t id = sys_id;
    file_be->parent = sys_log_backend;
    ulog_backend_filter_t filter = sys_log_file_backend_filter;

    ulog_file_backend_init( file_be, 
                            table[id].name,
                            table[id].dir_path,
                            table[id].max_num,
                            table[id].max_size,
                            table[id].buf_size);
    
    ulog_file_backend_enable(file_be);

    ulog_backend_set_filter(&file_be->parent,filter);
}
/************************运动日志文件后端操作函数*****************************************/
/**
  * @brief  运动日志文件后端滤波器设置.
  * @param  None.
  * @retval The filter will be call before output. It will return TRUE when the filter condition is math.
  * @note   None.
*/
static rt_bool_t motion_log_file_backend_filter(struct ulog_backend *backend, rt_uint32_t level, const char *tag, rt_bool_t is_raw, 
                                      const char *log, rt_size_t len)
{
    if (rt_strncmp(tag,MOTION_TAG, sizeof(MOTION_TAG)) == 0)//带有"MOVE"标签输出
      return RT_TRUE;
    else
      return RT_FALSE;
}

/**
  * @brief  运动日志文件后端初始化.
  * @param  None.
  * @retval None.
  * @note   None.
*/
void motion_log_file_backend_init(void)
{
    struct ulog_file_be *file_be = &motion_log_file;
    uint8_t id = motion_id;
    file_be->parent = motion_log_backend;
    ulog_backend_filter_t filter = motion_log_file_backend_filter;

    ulog_file_backend_init( file_be, 
                            table[id].name,
                            table[id].dir_path,
                            table[id].max_num,
                            table[id].max_size,
                            table[id].buf_size);
    
    ulog_file_backend_enable(file_be);

    ulog_backend_set_filter(&file_be->parent,filter);
}
/************************MSH命令*****************************************/
#ifdef RT_USING_MSH
/**
  * @brief  控制台后端滤波器设置.
  * @param  None.
  * @retval The filter will be call before output. It will return TRUE when the filter condition is math.
  * @note   None.
*/
rt_bool_t ulog_console_backend_filter(struct ulog_backend *backend, rt_uint32_t level, const char *tag, rt_bool_t is_raw, 
                                      const char *log, rt_size_t len)
{
    if(console_log_file.enable == RT_TRUE)
    {
      if (rt_strncmp(tag,MOTION_TAG, sizeof(MOTION_TAG)) == 0)//排除带有"MOVE"标签输出
        return RT_FALSE;
      else
        return RT_TRUE;
    }
    else
    {
      return RT_TRUE;
    }
}
/**
  * @brief  控制台后端滤波器设置.
  * @param  None.
  * @retval The filter will be call before output. It will return TRUE when the filter condition is math.
  * @note   None.
*/
int ulog_console_backend_filter_init(void)
{
    ulog_backend_t console = ulog_backend_find("console");
    console_log_file.enable = RT_TRUE;
    ulog_backend_set_filter(console,ulog_console_backend_filter);
    return 0;
}
INIT_DEVICE_EXPORT(ulog_console_backend_filter_init);
/**
  * @brief  日志文件后端卸载
  * @param  None.
  * @retval None.
  * @note   None.
*/
static void cmd_log_file_backend(uint8_t argc, char **argv)
{
#define FILE_CMD_LIST                  0
#define FILE_CMD_DEINIT                1
#define FILE_CMD_CONTROL               2

    size_t i = 0;

    const char* help_info[] =
    {
        [FILE_CMD_LIST]           = "ulog_be_cmd list             - Prints the back-end status of all files.",
        [FILE_CMD_DEINIT]         = "ulog_be_cmd deinit   [name]  - Deinit ulog file backend [name].",
        [FILE_CMD_CONTROL]        = "ulog_be_cmd control  [name]  - Control ulog file backend [name] [enable/disable].",
    };

    const char *operator = argv[1];
    if (argc < 2)
    {
        rt_kprintf("Usage:\n");
        for (i = 0; i < sizeof(help_info) / sizeof(char*); i++)
        {
            rt_kprintf("%s\n", help_info[i]);
        }
        rt_kprintf("\n");
        return;
    }
    else
    {
        if(!rt_strcmp(operator,"list")) 
        {
            const char *item_title = "file_be";
            int maxlen = RT_NAME_MAX;    

            rt_kprintf("%-*.*s init_state\n",maxlen,maxlen,item_title);
            rt_kprintf("-------- ----------\n");
            for(uint8_t i = 0; i < sizeof(table) / sizeof(table[0]); i++)
            {
                rt_kprintf("%-*.*s",maxlen,maxlen,table[i].name);
                ulog_backend_t file_be = ulog_backend_find(table[i].name);
                if(file_be != RT_NULL)
                { 
                    rt_kprintf("  init");
                }
                else
                { 
                    rt_kprintf("  deinit ");
                }
                rt_kprintf("\n");
            }
        }
        else if(!rt_strcmp(operator,"deinit")) 
        {
          if (argc < 3)
          {
              rt_kprintf("Usage:\n");
              rt_kprintf("Deinit ulog file backend [name]\n");
              return;
          }
          const char *operator = argv[2];
          const char *name = RT_NULL;
          uint8_t i;
          for(i = 0; i < sizeof(table) / sizeof(table[0]); i++)
          {
              if(!rt_strcmp(operator,table[i].name))
              {
                  name = table[i].name;
                  break;
              }
              else
              {
                  continue;
              }
          }
          if(name != RT_NULL)
          {
            ulog_file_backend_deinit(table[i].file_be);
            ulog_file_backend_disable(table[i].file_be);
            rt_kprintf("The file backend %s is deinit\n",operator);
          }
          else
          {
            rt_kprintf("File backend %s not found\n",operator);
            return;
          }
        }
        else if(!rt_strcmp(operator,"control"))
        {
            const char *operator = argv[2];
            const char *flag = argv[3];
            const char *name = RT_NULL;
            uint8_t i;
            if (argc < 4)
            {
                rt_kprintf("Usage:\n");
                rt_kprintf("control ulog file backend [name] [enable/disable]\n");
                return;
            }

            for(i = 0; i < sizeof(table) / sizeof(table[0]); i++)
            {
                if(!rt_strcmp(operator,table[i].name))
                {
                    name = table[i].name;
                    break;
                }
                else
                {
                    continue;
                }
            }
            if(name != RT_NULL)
            {
              if(!rt_strcmp(flag,"disable"))
              {
                ulog_file_backend_disable(table[i].file_be);
                rt_kprintf("The file backend %s is disabled\n",operator);
              }
              else if(!rt_strcmp(flag,"enable"))
              {
                ulog_file_backend_enable(table[i].file_be);
                rt_kprintf("The file backend %s is enable\n",operator);
              }
              else
              {
                rt_kprintf("Usage:\n");
                rt_kprintf("control ulog file backend [name] [enable/disable]\n");
                return;
              }
            }
            else
            {
                rt_kprintf("File backend %s not found\n",operator);
                return;
            }
        }
        else
        {
            rt_kprintf("Usage:\n");
            for (i = 0; i < sizeof(help_info) / sizeof(char*); i++)
            {
                rt_kprintf("%s\n", help_info[i]);
            }
            rt_kprintf("\n");
            return;
        }
    }
}
MSH_CMD_EXPORT_ALIAS(cmd_log_file_backend,ulog_be_cmd,ulog file cmd);
#endif /*RT_USING_MSH*/