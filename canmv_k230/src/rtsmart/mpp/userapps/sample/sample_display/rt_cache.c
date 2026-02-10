#include "rt_cache.h"
void cache_init(pkg_cache_t *cache) {
    rt_memset(cache->buffer, 0, sizeof(cache->buffer));
    cache->wr_idx = cache->rd_idx = 0;
    cache->sem = rt_sem_create("cache_sem", 0, RT_IPC_FLAG_FIFO);
    cache->lock = rt_mutex_create("cache_lock", RT_IPC_FLAG_FIFO);
}

rt_err_t cache_write(pkg_cache_t *cache, const void *data) {
    rt_mutex_take(cache->lock, RT_WAITING_FOREVER);
    if (((cache->wr_idx + 1) % PKG_NUM) == cache->rd_idx) {
        //rt_sem_release(cache->sem);
        rt_mutex_release(cache->lock);
        //rt_kprintf("cache_write err\n");
        return -RT_EFULL;
    }
    rt_memcpy(cache->buffer[cache->wr_idx], data, PKG_SIZE);
    cache->wr_idx = (cache->wr_idx + 1) % PKG_NUM;    
     rt_sem_release(cache->sem);  // 先发信号量
    rt_mutex_release(cache->lock); // 后放锁
    return RT_EOK;
}

rt_err_t cache_read(pkg_cache_t *cache, void *output) {
    rt_sem_take(cache->sem, RT_WAITING_FOREVER);
    rt_mutex_take(cache->lock, RT_WAITING_FOREVER);
    rt_memcpy(output, cache->buffer[cache->rd_idx], PKG_SIZE);
    cache->rd_idx = (cache->rd_idx + 1) % PKG_NUM;
    rt_mutex_release(cache->lock);
    return RT_EOK;
}

void cache_deinit(pkg_cache_t *cache) {
    rt_mutex_take(cache->lock, RT_WAITING_FOREVER);
    rt_sem_delete(cache->sem);
    rt_mutex_release(cache->lock);
    rt_mutex_delete(cache->lock);
}


// 添加强制恢复函数
void cache_force_recover(pkg_cache_t *cache) {
    rt_mutex_take(cache->lock, RT_WAITING_FOREVER);
    // 重置缓冲区状态
    cache->wr_idx = 0;
    cache->rd_idx = 0;
    rt_memset(cache->buffer, 0, sizeof(cache->buffer));
    
    // 释放可能阻塞的读取线程
    //while (rt_sem_take(cache->sem,10) == RT_EOK) {
        // 清空所有信号量
         rt_sem_release(cache->sem);  // 先发信号量
    //}
    rt_mutex_release(cache->lock);
}

