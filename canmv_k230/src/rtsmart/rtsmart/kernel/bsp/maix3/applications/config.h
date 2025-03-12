#pragma once

#include <stdint.h>

#define LINE_MAX_SIZE       (64)

#define RT_SMART_CONFIG_FILE_PATH  "/bin/config.txt"

static inline __attribute__((always_inline)) uint32_t shash(const char *s) {
  uint32_t v = 5381;
  if (s) {
    while (*s)
      v = (v << 5) + v + (*s++);
  }
  return v;
}

void excute_sdcard_config(void);
