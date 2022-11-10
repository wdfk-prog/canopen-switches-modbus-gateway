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
#include "ulog_file_be.h"
#include <ulog_be.h>
/* Private includes ----------------------------------------------------------*/
#include "main.h"
/* Private typedef -----------------------------------------------------------*/
/*
* ���ע���
*/
struct _log_file
{
    const char *name; 
    const char *dir_path; 
    rt_size_t max_num;
    rt_size_t max_size;
    rt_size_t buf_size;
};
/*
* �ļ���˱�ʶ
*/
typedef enum 
{
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

static struct _log_file table[] =
{
    {"sys"      ,ROOT_PATH,10,FILE_SIZE,BUFF_SIZE},
    {"motion"   ,ROOT_PATH,5,FILE_SIZE,BUFF_SIZE},
};
/* Private function prototypes -----------------------------------------------*/
/************************ϵͳ��־�ļ���˲�������*****************************************/
/**
  * @brief  ϵͳ��־�ļ�����˲�������.
  * @param  None.
  * @retval The filter will be call before output. It will return TRUE when the filter condition is math.
  * @note   None.
*/
static rt_bool_t sys_log_file_backend_filter(struct ulog_backend *backend, rt_uint32_t level, const char *tag, rt_bool_t is_raw, 
                                      const char *log, rt_size_t len)
{
    if (rt_strncmp(tag,MOTION_TAG, sizeof(MOTION_TAG)) == 0)//�ų�����"MOVE"��ǩ���
      return RT_FALSE;
    else
      return RT_TRUE;
}
/**
  * @brief  ϵͳ��־�ļ���˳�ʼ��.
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
/************************�˶���־�ļ���˲�������*****************************************/
/**
  * @brief  �˶���־�ļ�����˲�������.
  * @param  None.
  * @retval The filter will be call before output. It will return TRUE when the filter condition is math.
  * @note   None.
*/
static rt_bool_t motion_log_file_backend_filter(struct ulog_backend *backend, rt_uint32_t level, const char *tag, rt_bool_t is_raw, 
                                      const char *log, rt_size_t len)
{
    if (rt_strncmp(tag,MOTION_TAG, sizeof(MOTION_TAG)) == 0)//����"MOVE"��ǩ���
      return RT_TRUE;
    else
      return RT_FALSE;
}

/**
  * @brief  �˶���־�ļ���˳�ʼ��.
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
/************************MSH����*****************************************/
#ifdef RT_USING_MSH
/**
  * @brief  ����̨����˲�������.
  * @param  None.
  * @retval The filter will be call before output. It will return TRUE when the filter condition is math.
  * @note   None.
*/
rt_bool_t ulog_console_backend_filter(struct ulog_backend *backend, rt_uint32_t level, const char *tag, rt_bool_t is_raw, 
                                      const char *log, rt_size_t len)
{
    if (rt_strncmp(tag,MOTION_TAG, sizeof(MOTION_TAG)) == 0)//�ų�����"MOVE"��ǩ���
      return RT_FALSE;
    else
      return RT_TRUE;
}
/**
  * @brief  ��־�ļ����ж��
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
          const char *operator = argv[2];
          if(!rt_strcmp(operator,"motion"))
          {
            ulog_file_backend_deinit(&motion_log_file);
            ulog_file_backend_disable(&motion_log_file);
            rt_kprintf("The file backend %s is deinit\n",operator);
          }
          else if(!rt_strcmp(operator,"sys"))
          {
            ulog_file_backend_deinit(&sys_log_file);
            ulog_file_backend_disable(&sys_log_file);
            rt_kprintf("The file backend %s is deinit\n",operator);
          }
          else
          {
            rt_kprintf("Usage:\n");
            rt_kprintf("Deinit ulog file backend [name]\n");
            return;
          }
        }
        else if(!rt_strcmp(operator,"control"))
        {
            const char *operator = argv[2];
            const char *flag = argv[3];
            if (argc < 4)
            {
                rt_kprintf("Usage:\n");
                rt_kprintf("control ulog file backend [name] [enable/disable]\n");
                return;
            }
            else if(!rt_strcmp(operator,table[sys_id].name))
            {
              if(!rt_strcmp(flag,"disable"))
              {
                ulog_file_backend_disable(&sys_log_file);
                rt_kprintf("The file backend %s is disabled\n",operator);
              }
              else if(!rt_strcmp(flag,"enable"))
              {
                ulog_file_backend_enable(&sys_log_file);
                rt_kprintf("The file backend %s is enable\n",operator);
              }
              else
              {
                rt_kprintf("Usage:\n");
                rt_kprintf("control ulog file backend [name] [enable/disable]\n");
                return;
              }
            }
            else if(!rt_strcmp(operator,table[motion_id].name))
            {
              if(!rt_strcmp(flag,"disable"))
              {
                ulog_file_backend_disable(&motion_log_file);
                rt_kprintf("The file backend %s is disabled\n",operator);
              }
              else if(!rt_strcmp(flag,"enable"))
              {
                ulog_file_backend_enable(&motion_log_file);
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
              rt_kprintf("Failed to find the file backend:%s\n",operator);
            }
        }
    }
}
MSH_CMD_EXPORT_ALIAS(cmd_log_file_backend,ulog_be_cmd,ulog file cmd);
#endif