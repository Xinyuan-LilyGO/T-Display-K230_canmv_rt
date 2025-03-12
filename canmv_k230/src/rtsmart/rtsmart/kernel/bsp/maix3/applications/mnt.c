#include <rtthread.h>

#ifdef RT_USING_DFS
#include <dfs_fs.h>
#include <dfs_romfs.h>

const struct romfs_dirent _root_dirent[] = {
    {ROMFS_DIRENT_DIR, "bin",     RT_NULL, 0},
    {ROMFS_DIRENT_DIR, "sdcard",  RT_NULL, 0},
    {ROMFS_DIRENT_DIR, "data",     RT_NULL, 0},
#ifdef RT_USING_DFS_DEVFS
    {ROMFS_DIRENT_DIR, "dev",     RT_NULL, 0},
#endif
#ifdef RT_USING_DFS_NFS
    {ROMFS_DIRENT_DIR, "nfs",     RT_NULL, 0},
#endif
#ifdef RT_USING_DFS_PROCFS
    {ROMFS_DIRENT_DIR, "proc",    RT_NULL, 0},
#endif
#ifdef RT_USING_DFS_TMPFS
    {ROMFS_DIRENT_DIR, "tmp",     RT_NULL, 0},
#endif
};

const struct romfs_dirent romfs_root =
{
  ROMFS_DIRENT_DIR, "/", (rt_uint8_t *)_root_dirent, sizeof(_root_dirent) / sizeof(_root_dirent[0])
};

int mnt_init(void) {
  rt_err_t ret;

  if (dfs_mount(RT_NULL, "/", "rom", 0, &romfs_root) != 0) {
    rt_kprintf("Dir / mount failed!\n");
    return -1;
  }

  mkdir("/dev/shm", 0x777);

  if (dfs_mount(RT_NULL, "/dev/shm", "tmp", 0, 0) != 0) {
    rt_kprintf("Dir /dev/shm mount failed!\n");
  }

#ifdef RT_USING_DFS_TMPFS
  if (dfs_mount(RT_NULL, "/tmp", "tmp", 0, 0) != 0) {
    rt_kprintf("Dir /tmp mount failed!\n");
  }
#endif

#ifndef RT_FASTBOOT
  rt_kprintf("/dev/shm file system initialization done!\n");
#endif

  return 0;
}
INIT_ENV_EXPORT(mnt_init);

#endif
