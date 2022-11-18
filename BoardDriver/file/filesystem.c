/**
 * @file filesystem.c
 * @brief 
 * @author HLY (1425075683@qq.com)
 * @version 1.0
 * @date 2022-11-17
 * @copyright Copyright (c) 2022
 * @attention 
 * @par 修改日志:
 * Date       Version Author  Description
 * 2022-11-17 1.0     HLY     first version
 * 2022-11-18 1.1     HLY     修改flashdb至文件系统
 */
/**
  操作内容
  ls	显示文件和目录的信息
  cd	进入指定目录
  cp	复制文件
  rm	删除文件或目录 //文件夹内有内容删除不掉，得使用 rm-rf
  mv	将文件移动位置或改名
  echo	将指定内容写入指定文件，当文件存在时，就写入该文件，当文件不存在时就新创建一个文件并写入
  cat	展示文件的内容
  pwd	打印出当前目录地址
  mkdir	创建文件夹
  mkfs	格式化文件系统
*/
/* Includes ------------------------------------------------------------------*/
#include "filesystem.h"
/* Private includes ----------------------------------------------------------*/
#include <fal.h>
#include <dfs_fs.h>
#include <dfs_romfs.h>
#include <fal_cfg.h>
#include "ulog_file_be.h"
#include "main.h"
//#include "mb_handler.h"
/* 添加 DEBUG 头文件 */
#define DBG_SECTION_NAME               "file"
#define DBG_LEVEL                      DBG_INFO
#include <rtdbg.h>
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define FS_PARTITION_NAME "filesystem"//"W25Q128" //在fal_cfg.h中FAL_PART_TABLE定义

#define ENV_VERSION       002          //默认参数版本
#define SEC_SIZE          4096         // 设置扇区大小
#define DB_SIZE           SEC_SIZE * 2 //设置最大数据容量
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/* KVDB object */
static struct fdb_kvdb mb_param = { 0 };
/*运行次数*/
static uint16_t boot_count = 0;
static time_t  rtc_time = 0;
/* default KV nodes 第一次初始化时保存信息*/
static struct fdb_default_kv_node default_kv_table[] = 
{
  {"boot_count", &boot_count,           sizeof(boot_count)},
  {"rtc_time",   &rtc_time,             sizeof(boot_count)}, 
};
/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  MB_Param_Save
  * @param  None.
  * @retval None.
  * @note   读取数据遇到问题，请先尝试重新保存数据在读出尝试
*/
void MB_Param_Save(void)
{
  struct fdb_blob blob;
  const uint16_t *temp;
  for(uint8_t i = 0; i < sizeof(default_kv_table) / sizeof(struct fdb_default_kv_node);i++)
  {
    temp = (const uint16_t *)default_kv_table[i].value;
    //构造 blob 对象
    fdb_kv_set_blob(&mb_param,default_kv_table[i].key, fdb_blob_make(&blob,temp,default_kv_table[i].value_len));
  }
}
/**
  * @brief  MB_Param_Read
  * @param  None.
  * @retval None.
  * @note   读取数据遇到问题，请先尝试重新保存数据在读出尝试
*/
void MB_Param_Read(void)
{
  struct fdb_blob blob;
  const uint16_t *temp;
  for(uint8_t i = 0; i < sizeof(default_kv_table) / sizeof(struct fdb_default_kv_node);i++)
  {
    temp = (const uint16_t *)default_kv_table[i].value;
    //构造 blob 对象
    fdb_kv_get_blob(&mb_param,default_kv_table[i].key, fdb_blob_make(&blob,temp,default_kv_table[i].value_len));
  }
}
/**
 * @brief  rtc_time_write_file
 * @param  none.   
 * @note   写入时间戳
 */
void rtc_time_write(void)
{
    struct fdb_blob blob;

  /* get the "boot_count" KV value */
  fdb_kv_get_blob(&mb_param, "rtc_time", fdb_blob_make(&blob, &rtc_time, sizeof(rtc_time)));
  /* the blob.saved.len is more than 0 when get the value successful */
  if (blob.saved.len > 0) 
  {
      LOG_D("get the 'rtc_time' value is %d", rtc_time);
  } 
  else 
  {
      LOG_W("get the 'rtc_time' failed\n");
  }
 
  get_timestamp(&rtc_time);
  fdb_kv_set_blob(&mb_param, "rtc_time", fdb_blob_make(&blob, &rtc_time, sizeof(rtc_time)));
}
/**************************本地操作***************************************************/
/**
 * @brief  boot_count_wr
 * @param  kvdb   
 * @note   读取并写入芯片复位/上电次数
 */
static void boot_count_wr(fdb_kvdb_t kvdb)
{
    struct fdb_blob blob;

    { /* GET the KV value */
        /* get the "boot_count" KV value */
        fdb_kv_get_blob(kvdb, "boot_count", fdb_blob_make(&blob, &boot_count, sizeof(boot_count)));
        /* the blob.saved.len is more than 0 when get the value successful */
        if (blob.saved.len > 0) 
        {
            LOG_D("get the 'boot_count' value is %d", boot_count);
        } else 
        {
            LOG_W("get the 'boot_count' failed");
        }
    }

    { /* CHANGE the KV value */
        /* increase the boot count */
        boot_count ++;
        /* change the "boot_count" KV's value */
        fdb_kv_set_blob(kvdb, "boot_count", fdb_blob_make(&blob, &boot_count, sizeof(boot_count)));
        LOG_I("Burn the number :%d", boot_count);
    }
}
/**
 * @brief Flash_KVDB_Init 
 * @retval int 
 */
int Flash_KVDB_Init(void)
{
  fdb_err_t result;
  struct fdb_default_kv default_kv;
  bool file_mode = RT_TRUE;
  uint32_t sec_size = SEC_SIZE;
  uint32_t db_size  = DB_SIZE;
  default_kv.kvs = default_kv_table;
  default_kv.num = sizeof(default_kv_table) / sizeof(default_kv_table[0]);
  mb_param.ver_num = ENV_VERSION;//每次初始化检测版本号，自动更新版本
  
  fdb_kvdb_control(&mb_param, FDB_KVDB_CTRL_SET_FILE_MODE, &file_mode);
  fdb_kvdb_control(&mb_param, FDB_KVDB_CTRL_SET_SEC_SIZE, &sec_size);
  fdb_kvdb_control(&mb_param, FDB_KVDB_CTRL_SET_MAX_SIZE, &db_size);
  
  /*初始化 KVDB
    参数	      描述
    db	        数据库对象
    name	      数据库名称
    path	      FAL 模式：分区表中的分区名，文件模式：数据库保存的路径
    default_kv	默认 KV 集合，第一次初始化时，将会把默认 KV 写入数据库中
    user_data	  用户自定义数据，没有时传入 NULL
    返回	      错误码
  */
  result = fdb_kvdb_init(&mb_param, "env", "/flash", &default_kv, NULL);
  if (result != FDB_NO_ERR)   
  {
   return RT_ERROR;
  }
  boot_count_wr(&mb_param);
  return RT_EOK;
}
/**
  * @brief  文件系统初始化.
  * @param  None.
  * @retval None.
  * @note   None.
*/
static int FileSystem_Init(void)
{
  ulog_e("","\n--------------------------------FILESYSTEM----------------------------------");
  /* 初始化 fal */
  fal_init();
  //必须先创建rom文件系统，以提供后面的挂载目录
  if(dfs_mount(RT_NULL,"/","rom",0,&(romfs_root)) == 0)
  {
      LOG_I("ROM file system initializated!");
      /* 生成 mtd 设备 */
      struct rt_device *mtd_dev = fal_mtd_nor_device_create(FS_PARTITION_NAME);
      if (!mtd_dev)
      {
          LOG_E("Can't create a mtd device on '%s' partition.", FS_PARTITION_NAME);
      }
      else
      {
        /* 挂载 littlefs  */
        if (dfs_mount(FS_PARTITION_NAME, "/flash", "lfs", 0, 0) == 0)
        {
            LOG_I("Filesystem /flash initialized!");
        }
        else
        {
            /* 格式化文件系统 */
            dfs_mkfs("lfs", FS_PARTITION_NAME);
            /* 挂载 lfs */
            if (dfs_mount(FS_PARTITION_NAME, "/flash", "lfs", 0, 0) == 0)
            {
                LOG_I("Filesystem /flash initialized!");
            }
            else
            {
                LOG_E("Failed to initialize filesystem /flash!");
            }
        }
      }
  }
  else
  {
      LOG_E("ROM file system initializate failed!");
  }
  #if(OUT_FILE_ENABLE == 1)
  extern void sys_log_file_backend_init(void);
  sys_log_file_backend_init();
  extern void motion_log_file_backend_init(void);
  motion_log_file_backend_init();
  #endif
  /*数据库初始化*/
  Flash_KVDB_Init();
  ulog_e("","\n--------------------------------FILESYSTEM----------------------------------");
  return RT_EOK;
}
INIT_ENV_EXPORT(FileSystem_Init);
/**
  * @attention  
  1.开启onchip-flash 开启SPI-FLSAH  
  2.开启SFUD 开启FAL框架
  3.配置 <fal_cfg.h> 分配表
  4.添加虚拟文件系统DFS组件。打开MFD
  5.添加elm fatfs文件系统。
  6.romfs.c在tool目录下新建romfs文件夹并自定义下属一级目录与尼尔使用mkromfs.py生成
  python mkromfs.py romfs romfs.c

  7.生成LOGS目录
  
  传输ULOG日志至上位机中：
      1.cd至ulog文件所在目录 cd/flash/logs
      2.sy ulog.log
      提示：No permission to write on the specified folder.因为有"/"符号无法保存
  从上位机中下载数据至mcu：
      1.xshell发送ry既可。默认接收至/flash目录下
*/
/**  
 * @attention  
  片上onchip使用littlefs系统，创建两个目录后及报错空间不足。
  由文件系统结构决定，扇区不足。虽然大，但扇区不够。
  littlefs写入速度相比于fatfs慢。保存参数过慢，改用fatfs.fatfs没有掉电保存与读写平衡。
  容易造成BAD file sys 文件。改回littlefs
  
  fatfs文件系统需注意，意外掉电可能造成文件损坏，需要重新格式化 命令 mkfs -t elm filesystem
  
  关于CMB的日志文件保存。第一：只能对分区进行操作。没有办法保存到其他目录下。
  第二，使用rom构建根目录后，创建一个分区无法使用littlefs文件系统格式化。只能是rom系统，只读属性，不可写入。
*/
/*
msh /flash/logs>fal bench 4096
DANGER: It will erase full chip or partition! Please run 'fal bench 4096 yes'.
msh /flash/logs>fal bench 4096 yes
Erasing 1048576 bytes data, waiting...
Erase benchmark success, total time: 3.548S.
Writing 1048576 bytes data, waiting...
Write benchmark success, total time: 4.097S.
Reading 1048576 bytes data, waiting...
Read benchmark success, total time: 1.546S.

Erase benchmark success, total time: 33.946S.
Writing the sf_cmd 16777216 bytes data, waiting...
Write benchmark success, total time: 65.537S.
Reading the sf_cmd 16777216 bytes data, waiting...
Read benchmark success, total time: 26.302S.
*/