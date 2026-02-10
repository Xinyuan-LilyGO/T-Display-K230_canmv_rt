#ifndef _RT_CACHE_H
#define _RT_CACHE_H
#include <rtthread.h>
#define PKG_SIZE 225
#define PKG_NUM 20

typedef struct {
    rt_uint8_t buffer[PKG_NUM][PKG_SIZE];
    volatile rt_uint32_t wr_idx;
    volatile rt_uint32_t rd_idx;
    rt_sem_t sem;
    rt_mutex_t lock;
} pkg_cache_t;
#ifdef __cplusplus
extern "C" {
#endif
void cache_init(pkg_cache_t *cache);
rt_err_t cache_write(pkg_cache_t *cache, const void *data);
rt_err_t cache_read(pkg_cache_t *cache, void *output);
void cache_deinit(pkg_cache_t *cache);
void cache_force_recover(pkg_cache_t *cache);
#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
