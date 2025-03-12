#include "lwp_user_mm.h"
#include <dfs_file.h>
#include <rtthread.h>
#include <string.h>
#include <time.h>

#ifdef RT_USING_LWP
#include <lwp.h>
#include <mmu.h>
#include <page.h>
#endif
#ifndef ARCH_PAGE_SIZE
#define ARCH_PAGE_SIZE 0
#endif

#ifdef RT_USING_USAGE
#include "usage.h"
#endif

#ifdef PKG_NETUTILS_NTP
  #include "ntp.h"
#endif

struct misc_dev_handle {
  int cmd;
  int (*func)(void *args);
};

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
typedef struct {
  rt_list_t *list;
  rt_list_t **array;
  rt_uint8_t type;
  int nr;     /* input: max nr, can't be 0 */
  int nr_out; /* out: got nr */
} list_get_next_t;

static void list_find_init(list_get_next_t *p, rt_uint8_t type,
                           rt_list_t **array, int nr) {
  struct rt_object_information *info;
  rt_list_t *list;

  info = rt_object_get_information((enum rt_object_class_type)type);
  list = &info->object_list;

  p->list = list;
  p->type = type;
  p->array = array;
  p->nr = nr;
  p->nr_out = 0;
}

static rt_list_t *list_get_next(rt_list_t *current, list_get_next_t *arg) {
  int first_flag = 0;
  rt_ubase_t level;
  rt_list_t *node, *list;
  rt_list_t **array;
  int nr;

  arg->nr_out = 0;

  if (!arg->nr || !arg->type) {
    return (rt_list_t *)RT_NULL;
  }

  list = arg->list;

  if (!current) /* find first */
  {
    node = list;
    first_flag = 1;
  } else {
    node = current;
  }

  level = rt_hw_interrupt_disable();

  if (!first_flag) {
    struct rt_object *obj;
    /* The node in the list? */
    obj = rt_list_entry(node, struct rt_object, list);
    if ((obj->type & ~RT_Object_Class_Static) != arg->type) {
      rt_hw_interrupt_enable(level);
      return (rt_list_t *)RT_NULL;
    }
  }

  nr = 0;
  array = arg->array;
  while (1) {
    node = node->next;

    if (node == list) {
      node = (rt_list_t *)RT_NULL;
      break;
    }
    nr++;
    *array++ = node;
    if (nr == arg->nr) {
      break;
    }
  }

  rt_hw_interrupt_enable(level);
  arg->nr_out = nr;
  return node;
}
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

static struct rt_device misc_dev;

static int misc_open(struct dfs_fd *file) { return 0; }

static int misc_close(struct dfs_fd *file) { return 0; }

#define MISC_DEV_CMD_READ_HEAP          (0x1024 + 0)
#define MISC_DEV_CMD_READ_PAGE          (0x1024 + 1)
#define MISC_DEV_CMD_CPU_USAGE          (0x1024 + 2)
#define MISC_DEV_CMD_NTP_SYNC           (0x1024 + 3)
#define MISC_DEV_CMD_GET_TIME_T         (0x1024 + 4)
#define MISC_DEV_CMD_GET_CPU_TICK       (0x1024 + 5)

struct meminfo_t {
  size_t total_size;
  size_t free_size;
  size_t used_size;
};

static int misc_get_heap_info(void *args) {
  struct meminfo_t meminfo;
  size_t _total = 0, _free = 0, _used = 0;

#ifdef RT_USING_MEMHEAP_AS_HEAP
#define LIST_FIND_OBJ_NR 8

  rt_ubase_t level;
  list_get_next_t find_arg;
  rt_list_t *obj_list[LIST_FIND_OBJ_NR];
  rt_list_t *next = (rt_list_t *)RT_NULL;

  const char *item_title = "memheap";

  list_find_init(&find_arg, RT_Object_Class_MemHeap, obj_list,
                 sizeof(obj_list) / sizeof(obj_list[0]));

  do {
    next = list_get_next(next, &find_arg);
    {
      int i;
      for (i = 0; i < find_arg.nr_out; i++) {
        struct rt_object *obj;
        struct rt_memheap *mh;

        obj = rt_list_entry(obj_list[i], struct rt_object, list);
        level = rt_hw_interrupt_disable();
        if ((obj->type & ~RT_Object_Class_Static) != find_arg.type) {
          rt_hw_interrupt_enable(level);
          continue;
        }

        rt_hw_interrupt_enable(level);

        mh = (struct rt_memheap *)obj;

        _total += mh->pool_size;
        _free += mh->available_size;
        _used += mh->max_used_size;
      }
    }
  } while (next != (rt_list_t *)RT_NULL);
#else // RT_USING_MEMHEAP_AS_HEAP
  rt_uint32_t t = 0, u = 0;
#ifdef RT_MEM_STATS
  rt_memory_info(&t, &u, RT_NULL);
#endif // RT_MEM_STATS

  size_t total_pages = 0, free_pages = 0;
#ifdef RT_USING_USERSPACE
  rt_page_get_info(&total_pages, &free_pages);
#endif // RT_USING_USERSPACE

  _total = t + total_pages * ARCH_PAGE_SIZE;
  _used = u + (total_pages - free_pages) * ARCH_PAGE_SIZE;
  _free = _total - _used;
#endif // RT_USING_MEMHEAP_AS_HEAP

  meminfo.total_size = _total;
  meminfo.free_size = _free;
  meminfo.used_size = _used;

  if (sizeof(struct meminfo_t) !=
      lwp_put_to_user(args, &meminfo, sizeof(struct meminfo_t))) {
    rt_kprintf("%s put_to_user failed\n", __func__);
    return -1;
  }

  return 0;
}

static int misc_get_page_info(void *args) {
  struct meminfo_t meminfo;
  size_t total, free;
  rt_page_get_info(&total, &free);

  meminfo.total_size = total * PAGE_SIZE;
  meminfo.free_size = free * PAGE_SIZE;
  meminfo.used_size = meminfo.total_size - meminfo.free_size;

  if (sizeof(struct meminfo_t) !=
      lwp_put_to_user(args, &meminfo, sizeof(struct meminfo_t))) {
    rt_kprintf("%s put_to_user failed\n", __func__);
    return -1;
  }
  return 0;
}

static int misc_get_cpu_usage(void *args) {
  int usage = -1;

#ifdef RT_USING_USAGE
  usage = (int)sys_cpu_usage(0);
#endif

  if (sizeof(int) != lwp_put_to_user(args, &usage, sizeof(int))) {
    rt_kprintf("%s put_to_user failed\n", __func__);
    return -1;
  }

  return 0;
}

static int misc_ntp_sync(void *args) {
  int result = 0;

#ifdef PKG_NETUTILS_NTP
  if(0x00 < ntp_sync_to_rtc(RT_NULL)) {
    result = 1;
  }
#endif

  if (sizeof(int) != lwp_put_to_user(args, &result, sizeof(int))) {
    rt_kprintf("%s put_to_user failed\n", __func__);
    return -1;
  }

  return 0;
}

static int misc_get_time_t(void *args) {
  time_t t;

  time(&t);

  if (sizeof(time_t) != lwp_put_to_user(args, &t, sizeof(time_t))) {
    rt_kprintf("%s put_to_user failed\n", __func__);
    return -1;
  }

  return 0;
}

static volatile uint64_t time_elapsed = 0;

static inline __attribute__((always_inline)) uint64_t get_ticks()
{
    __asm__ __volatile__(
        "rdtime %0"
        : "=r"(time_elapsed));
    return time_elapsed;
}

static int misc_get_cpu_tick(void *args) {
  uint64_t tick = get_ticks();

  if (sizeof(uint64_t) != lwp_put_to_user(args, &tick, sizeof(uint64_t))) {
    rt_kprintf("%s put_to_user failed\n", __func__);
    return -1;
  }

  return 0;
}

static const struct misc_dev_handle misc_handles[] = {
  {
    .cmd = MISC_DEV_CMD_READ_HEAP,
    .func = misc_get_heap_info,
  },
  {
    .cmd = MISC_DEV_CMD_READ_PAGE,
    .func = misc_get_page_info,
  },
  {
    .cmd = MISC_DEV_CMD_CPU_USAGE,
    .func = misc_get_cpu_usage,
  },
  {
    .cmd = MISC_DEV_CMD_NTP_SYNC,
    .func = misc_ntp_sync,
  },
  {
    .cmd = MISC_DEV_CMD_GET_TIME_T,
    .func = misc_get_time_t,
  },
  {
    .cmd = MISC_DEV_CMD_GET_CPU_TICK,
    .func = misc_get_cpu_tick,
  },
};

static int misc_ioctl(struct dfs_fd *file, int cmd, void *args) {
  int result = 0;

  for(size_t i = 0; i < sizeof(misc_handles) / sizeof(misc_handles[0]); i++) {
    if((cmd == misc_handles[i].cmd) && (misc_handles[i].func)) {
      return misc_handles[i].func(args);
    }
  }

  rt_kprintf("%s unknown cmd 0x%x\n", __func__, cmd);

  return -1;
}

static const struct dfs_file_ops meminfo_fops = {
    .open = misc_open,   // int (*open)     (struct dfs_fd *fd);
    .close = misc_open,  // int (*close)    (struct dfs_fd *fd);
    .ioctl = misc_ioctl, // int (*ioctl)    (struct dfs_fd *fd, int cmd, void
                         // *args);
    // int (*read)     (struct dfs_fd *fd, void *buf, size_t count);
    // int (*write)    (struct dfs_fd *fd, const void *buf, size_t count);
    // int (*flush)    (struct dfs_fd *fd);
    // int (*lseek)    (struct dfs_fd *fd, off_t offset);
    // int (*getdents) (struct dfs_fd *fd, struct dirent *dirp, uint32_t count);
    // int (*poll)     (struct dfs_fd *fd, struct rt_pollreq *req);
};

int misc_device_init(void) {
  RT_ASSERT(!rt_device_find("canmv_misc"));
  misc_dev.type = RT_Device_Class_Miscellaneous;

  /* no private */
  misc_dev.user_data = RT_NULL;

  rt_device_register(&misc_dev, "canmv_misc", RT_DEVICE_FLAG_RDONLY);

  misc_dev.fops = &meminfo_fops;

  return 0;
}
INIT_DEVICE_EXPORT(misc_device_init);
