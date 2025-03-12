#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <rthw.h>
#include <rtthread.h>

#include <dfs.h>
#include <dfs_fs.h>
#include <dfs_posix.h>

#include "./config.h"

struct config_handler {
  uint32_t command; // const char *command;
  void (*handler)(char *value);
};

/* 0: success, -1: failed */
/* if successed, user should close(fd), and rt_free(buffer) to release resources
 */
static int read_config_file(int *fd, char **buffer, size_t *buffer_size) {
  int _fd = 0;
  size_t _buffer_size = 0;
  char *_buffer = NULL;

  _fd = open(RT_SMART_CONFIG_FILE_PATH, O_RDWR);
  if (0 > _fd) {
    goto _failed;
  }

  _buffer_size = lseek(_fd, 0, SEEK_END);
  lseek(_fd, 0, SEEK_SET);

  if (0x00 == _buffer_size) {
    close(_fd);
    goto _failed;
  }

  if (NULL == (_buffer = rt_malloc(_buffer_size))) {
    close(_fd);
    goto _failed;
  }

  if (_buffer_size != read(_fd, _buffer, _buffer_size)) {
    rt_kprintf("read config.txt failed\n");
    close(_fd);
    rt_free(buffer);
    goto _failed;
  }

  *fd = _fd;
  *buffer = _buffer;
  *buffer_size = _buffer_size;

  return 0;

_failed:
  *fd = -1;
  *buffer = NULL;
  *buffer_size = 0;
  return -1;
}

#if defined(CONFIG_RT_AUTO_RESIZE_PARTITION) && defined(RT_USING_SDIO)
#define DPT_ADDRESS 0x1be /* device partition offset in Boot Sector */
#define DPT_ITEM_SIZE 16  /* partition item size */

struct partition_entry {
  u_int8_t active;
  u_int8_t start_head;
  u_int16_t start_cylsec;
  u_int8_t type;
  u_int8_t end_head;
  u_int16_t end_cylsec;
  u_int32_t lba_sector;
  u_int32_t lba_len;
};

_Static_assert(sizeof(struct partition_entry) == DPT_ITEM_SIZE,
               "Error partition entry size error");

_Static_assert(4 > CONFIG_RT_AUTO_RESIZE_PARTITION_NR,
               "MBR max partition is 4");

static inline __attribute__((always_inline)) void disable_auto_resize(void) {
  int fd = -1;
  char *file_buffer = NULL;
  size_t file_buffer_size = 0;

  if ((0x00 == read_config_file(&fd, &file_buffer, &file_buffer_size)) &&
      (0 <= fd) && (NULL != file_buffer)) {
    /* change auto_resize to n */
    int offset_auto_resize = 0;
    char *p_auto_resize = rt_strstr(file_buffer, "auto_resize=");
    if (p_auto_resize) {
      lseek(fd, p_auto_resize - file_buffer + sizeof("auto_resize=") - 1,
            SEEK_SET);
      write(fd, "n", 1);
      close(fd);

    } else {
      close(fd);
    }

    if (file_buffer) {
      rt_free(file_buffer);
    }
  } else {
    rt_kprintf("%s, read config file failed\n", __func__);
  }
}

static void execute_auto_resize(char *value) {
  int ret = 0;
  int fd = -1;
  char enable = 0;

  char dev_name[16];
  uint8_t *buffer = NULL;
  struct dfs_partition part, last_part;

  rt_device_t dev_sd = NULL;
  struct rt_device_blk_geometry dev_sd_geometry;
  struct partition_entry part_entry;

  if (0x01 != sscanf(value, "%c", &enable)) {
    rt_kprintf("%s parse value failed.\n", __func__);
    return;
  }

  if ('y' != enable) {
    return;
  }

  disable_auto_resize();

  rt_snprintf(dev_name, sizeof(dev_name), "sd%d", SDCARD_ON_SDIO_DEV);
  if (NULL == (dev_sd = rt_device_find(dev_name))) {
    rt_kprintf("%s find /dev/sd failed.\n", __func__);
    goto _exit;
  }

  if (RT_EOK != (ret = rt_device_open(dev_sd, RT_DEVICE_OFLAG_RDWR))) {
    rt_kprintf("%s open /dev/sd failed.\n", __func__);
    goto _exit;
  }

  if (NULL == (buffer = rt_malloc_align(SECTOR_SIZE, RT_CPU_CACHE_LINE_SZ))) {
    rt_kprintf("%s malloc failed.\n", __func__);
    goto _exit;
  }

  if (0x01 != rt_device_read(dev_sd, 0, buffer, 1)) {
    rt_kprintf("%s read sector failed.\n", __func__);
    goto _exit;
  }

  if (RT_EOK != dfs_filesystem_get_partition(
                    &part, buffer, CONFIG_RT_AUTO_RESIZE_PARTITION_NR)) {

    if (RT_EOK !=
        dfs_filesystem_get_partition(&last_part, buffer,
                                     CONFIG_RT_AUTO_RESIZE_PARTITION_NR - 1)) {
      rt_kprintf("%s get last partition info failed.\n", __func__);
      goto _exit;
    }

    rt_device_control(dev_sd, RT_DEVICE_CTRL_BLK_GETGEOME, &dev_sd_geometry);

    rt_memset(&part_entry, 0, sizeof(part_entry));

    part_entry.active = 0x00; // not bootable
    part_entry.start_head = 0xFE;
    part_entry.start_cylsec = 0xFFFF;
    part_entry.end_head = 0xFE;
    part_entry.end_cylsec = 0xFFFF;
    part_entry.type = 0x0C; // FAT32 (LBA)
    part_entry.lba_sector = ((last_part.offset + last_part.size) + 0x200000) &
                            ~(0x200000 - 1); // align to 1GiB

    /* we suppose user not use sdcard bigger than 128GiB, so no need to limit
     * the lba_len  */
    part_entry.lba_len = dev_sd_geometry.sector_count - part_entry.lba_sector;

    rt_memcpy(buffer + DPT_ADDRESS +
                  CONFIG_RT_AUTO_RESIZE_PARTITION_NR * DPT_ITEM_SIZE,
              &part_entry, sizeof(part_entry));

    if (0x01 == rt_device_write(dev_sd, 0, buffer, 1)) {
      rt_kprintf("Create new partiton 0x%x - 0x%x\n",
                 part_entry.lba_sector * SECTOR_SIZE,
                 (part_entry.lba_sector + part_entry.lba_len) * SECTOR_SIZE);

      rt_device_close(dev_sd);
      rt_free_align(buffer);

      /* create /bin/auto_mkfs */
      if (0 > (fd = open("/bin/auto_mkfs_data", O_CREAT | O_WRONLY))) {
        rt_kprintf("%s, create /bin/auto_mkfs_data failed\n", __func__);
      } else {
        close(fd);
      }
    } else {
      rt_kprintf("!!!FATAL ERROR, Update MBR FAILED.\n");
    }

    /* reboot */
    rt_kprintf("reboot...\n");
    rt_hw_cpu_reset();
  }

_exit:
  if (dev_sd) {
    rt_device_close(dev_sd);
  }

  if (buffer) {
    rt_free_align(buffer);
  }
}
#else
static void execute_auto_resize(char *value) {
  rt_kprintf("not support reisze partition.\n");
}
#endif

static const struct config_handler hanlders[] = {
    {
        0x277A782F /* auto_resize */,
        execute_auto_resize,
    },
};

static inline __attribute__((always_inline)) void excete_line(char *line) {
  uint32_t hash_command;

  char *p_command = line;
  char *p_value = NULL;

  if (NULL == (p_value = rt_strstr(p_command, "="))) {
    rt_kprintf("Invaild line '%s'\n", line);
    return;
  }
  p_value[0] = '\0';
  p_value++;

  hash_command = shash(p_command);

  for (size_t i = 0; i < sizeof(hanlders) / sizeof(hanlders[0]); i++) {
    if (hash_command == hanlders[i].command) {
      hanlders[i].handler(p_value);
      break;
    }
  }
}

void excute_sdcard_config(void) {
  int fd = -1;
  size_t length = 0;
  int line_length = 0;

  char *buffer = NULL;
  char *buffer_end = NULL;
  char *p_line_start = NULL;
  char *p_line_end = NULL;

  char line[LINE_MAX_SIZE];

  if (0x00 == read_config_file(&fd, &buffer, &length)) {
    close(fd);

    buffer_end = buffer + length;
    p_line_start = buffer;

    while (p_line_start < buffer_end) {
      p_line_end = rt_strstr(p_line_start, "\n");
      if (NULL == p_line_end) {
        break;
      }
      line_length = p_line_end - p_line_start;
      if (0x00 == line_length) {
        break;
      }
      if (LINE_MAX_SIZE <= line_length) {
        rt_kprintf("parse config, line data too long\n");
        p_line_start = p_line_end;
        continue;
      }
      memcpy(line, p_line_start, line_length);

      excete_line(line);

      p_line_start = p_line_end;
    }
  } else {
    rt_kprintf("open %s failed\n", RT_SMART_CONFIG_FILE_PATH);
  }

  if (buffer) {
    rt_free(buffer);
  }
}
