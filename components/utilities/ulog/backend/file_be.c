/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-01-07     ChenYong     first version
 * 2021-12-20     armink       add multi-instance version
 */

#include <rtthread.h>
#include <dfs_file.h>
#include <unistd.h>

#include <ulog.h>
#include <ulog_be.h>

#ifdef ULOG_BACKEND_USING_FILE

#if defined(ULOG_ASYNC_OUTPUT_THREAD_STACK) && (ULOG_ASYNC_OUTPUT_THREAD_STACK < 2048)
#error "The value of ULOG_ASYNC_OUTPUT_THREAD_STACK must be greater than 2048."
#endif

#define LOG_FILE_NAME_MAX              50

struct _log_file
{
    const rt_uint8_t id;
    rt_int32_t file_fd;
    const char *root_path;
    const char *log_path;
    const char *log_name;
    const char *file_type;
    const rt_uint32_t file_size;
    const rt_uint32_t file_num;
};

static struct _log_file _log_file_tbl[] =
{
    { LOG_SYS_ID, -1, "/", "/log/", "sys", ".log", 512 * 1024, 10 },
    { LOG_PM_ID,  -1, "/", "/log/", "pm",  ".log", 512 * 1024, 5 },
};

static struct ulog_backend ulog_file;

static struct rt_mutex log_file_lock;

static rt_bool_t check_file_path(const char *root_path, const char *file_path)
{
    if (root_path == NULL || file_path == NULL)
    {
        rt_kprintf("check_file_path: error! the root path is %s, file path is %s\n", root_path, file_path);
        return RT_FALSE;
    }

    if (access(root_path, 0) != 0)
    {
        rt_kprintf("check_file_path: error! the root path %s does not exist\n", root_path);
        return RT_FALSE;
    }

    if (access(file_path, 0) != 0)
    {
        if (mkdir(file_path, 0777) == -1)
        {
            rt_kprintf("check_file_path: error! the file_path path %s does not exist\n", file_path);
            return RT_FALSE;
        }
    }
    return RT_TRUE;
}

static rt_bool_t check_file_handle(int fd)
{
    if (fd >= 0)
        return RT_TRUE;
    return RT_FALSE;
}

/* get log file fd */
int get_file_handle(rt_uint8_t log_id)
{
    char file_name[LOG_FILE_NAME_MAX] = {0};
    rt_bool_t result = RT_FALSE;
    static rt_bool_t check_dir = RT_FALSE;

    if (!check_dir)
    {
        result = check_file_path(_log_file_tbl[log_id].root_path,
            _log_file_tbl[log_id].log_path);
        if (result == RT_FALSE)
        {
            rt_kprintf ("log path is not exsit\n");
            return -1;
        }
        check_dir = RT_TRUE;
    }

    rt_mutex_take(&log_file_lock, RT_WAITING_FOREVER);

    if (!check_file_handle(_log_file_tbl[log_id].file_fd))
    {
        rt_snprintf(file_name, LOG_FILE_NAME_MAX - 1, "%s%s%s",
            _log_file_tbl[log_id].log_path,
            _log_file_tbl[log_id].log_name,
            _log_file_tbl[log_id].file_type);

        _log_file_tbl[log_id].file_fd = open(file_name, O_CREAT | O_RDWR);

        if(_log_file_tbl[log_id].file_fd < 0)
        {
            rt_kprintf ("log file open error,fd=%d!!\n", _log_file_tbl[log_id].file_fd);
        }
    }

    rt_mutex_release(&log_file_lock);

    return _log_file_tbl[log_id].file_fd;
}

int get_log_file_number(rt_uint8_t log_id, const int number, const char *path)
{
    char file_name[LOG_FILE_NAME_MAX] = {0};
    int status = -1;
    int file_num = 0;

    for (int32_t i = number; i > 0; i--)
    {
        rt_memset(file_name, 0, LOG_FILE_NAME_MAX);
        rt_snprintf(file_name, LOG_FILE_NAME_MAX - 1, "%s%s_%d%s",
            path, _log_file_tbl[log_id].log_name, i,
            _log_file_tbl[log_id].file_type);

        status = access(file_name, 0);
        /* get file max log number */
        if (status == 0)
        {
            file_num = i;
            break;
        }
    }
    return file_num;
}

int log_file_rename(rt_uint8_t log_id, int max_index, const char *path, const int max_number, const char *file_name)
{
    int32_t i = 0;
    int ret = 0;
    char old_file_name[LOG_FILE_NAME_MAX] = {0};
    char new_file_name[LOG_FILE_NAME_MAX] = {0};

    if (max_index >= max_number)
    {
        char max_index_file[LOG_FILE_NAME_MAX] = {0};
        rt_snprintf(max_index_file, LOG_FILE_NAME_MAX - 1, "%s%s_%d%s",
            path, _log_file_tbl[log_id].log_name, max_number,
            _log_file_tbl[log_id].file_type);
        unlink(max_index_file);
    }
    else
    {
        max_index++;
    }

    for (i = max_index; i > 1; i--)
    {
        rt_memset(old_file_name, 0, LOG_FILE_NAME_MAX);
        rt_memset(new_file_name, 0, LOG_FILE_NAME_MAX);
        rt_snprintf(new_file_name, LOG_FILE_NAME_MAX - 1, "%s%s_%d%s",
            path, _log_file_tbl[log_id].log_name, i, _log_file_tbl[log_id].file_type);

        rt_snprintf(old_file_name, LOG_FILE_NAME_MAX - 1, "%s%s_%d%s",
            path, _log_file_tbl[log_id].log_name, i - 1, _log_file_tbl[log_id].file_type);

        ret = rename(old_file_name, new_file_name);

        if(ret != 0)
        {
            rt_kprintf ("file rename error:%s\n",old_file_name);
        }
    }

    rt_memset(old_file_name, 0, LOG_FILE_NAME_MAX);
    rt_memset(new_file_name, 0, LOG_FILE_NAME_MAX);
    rt_snprintf(old_file_name, LOG_FILE_NAME_MAX - 1, "%s", file_name);
    rt_snprintf(new_file_name, LOG_FILE_NAME_MAX - 1, "%s%s_%d%s",
        path, _log_file_tbl[log_id].log_name, i,
        _log_file_tbl[log_id].file_type);

    ret = rename(old_file_name, new_file_name);
    if(ret != 0)
    {
      rt_kprintf ("file rename error:%s\n",old_file_name);
    }
    return ret;
}

void log_file_rename_all(rt_uint8_t log_id, const char *path, const int32_t max_number)
{
    char file_name[LOG_FILE_NAME_MAX] = {0};
    int32_t max_index = 0;

    max_index = get_log_file_number(log_id, max_number, path);
    rt_snprintf(file_name, LOG_FILE_NAME_MAX - 1, "%s%s%s",
        path, _log_file_tbl[log_id].log_name,
        _log_file_tbl[LOG_SYS_ID].file_type);

    log_file_rename(log_id, max_index, path, max_number, file_name);
}

void close_log_file_handle(rt_uint8_t log_id)
{
    if (check_file_handle(_log_file_tbl[log_id].file_fd))
    {
        close(_log_file_tbl[log_id].file_fd);
        _log_file_tbl[log_id].file_fd = -1;
    }
}

void log_file_handle_update(rt_uint8_t log_id, int fd)
{
    _log_file_tbl[log_id].file_fd = fd;
}

void ulog_file_backend_output(struct ulog_backend *backend, rt_uint8_t log_id, rt_uint32_t level,
    const char *tag, rt_bool_t is_raw, const char *log, size_t len)
{
    off_t file_size = 0;
    int fd = -1;
    int32_t buf_len = 0;
    int32_t offset = 0;

    fd = get_file_handle(log_id);

    if (fd < 0)
    {
        rt_kprintf ("get_file_handle fail: %d.\n", fd);
        return;
    }
    for (buf_len = len; buf_len > 0; buf_len -= ULOG_LINE_BUF_SIZE)
    {
        file_size = lseek(fd, 0, SEEK_END);
        if (file_size > _log_file_tbl[log_id].file_size) /* log file max size check*/
        {
            close(fd);
            fd = -1;
            log_file_handle_update(log_id, fd);
            log_file_rename_all(log_id, _log_file_tbl[log_id].log_path,
                _log_file_tbl[log_id].file_num);
        }

        if (fd < 0)
        {
            fd = get_file_handle(log_id);
            if (fd < 0)
            {
                rt_kprintf("new file get_file_fd fail: %d.\n", fd);
                return;
            }
        }

        if (buf_len < ULOG_LINE_BUF_SIZE)
        {
            write(fd, log + offset, buf_len);
            offset += buf_len;
        }
        else
        {
            write(fd, log + offset, ULOG_LINE_BUF_SIZE);
            offset += ULOG_LINE_BUF_SIZE;
        }
        fsync(fd);
    }
}

int ulog_file_backend_init(void)
{
    /* create device filesystem lock */
    rt_mutex_init(&log_file_lock, "logfile", RT_IPC_FLAG_FIFO);

    ulog_init();
    rt_thread_mdelay(1000);
    ulog_file.output = ulog_file_backend_output;
    ulog_backend_register(&ulog_file, "file", RT_FALSE);

    return 0;
}
#endif /* ULOG_BACKEND_USING_FILE */
