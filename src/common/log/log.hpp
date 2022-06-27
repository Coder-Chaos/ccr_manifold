#pragma once
#include "log.h"

#ifdef __cplusplus
extern "C" {
#endif

// condition debug log
#define dlog_if(condition, ...)                                                \
  do {                                                                         \
    if (condition)                                                             \
      log_log(LOG_DEBUG, __FILE__, __LINE__, __VA_ARGS__);                     \
  } while (0)

#ifdef __cplusplus
}
#endif
