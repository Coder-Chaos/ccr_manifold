#pragma once

#include "common.hpp"
#include "log.h"
// linux timer
#include <errno.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

namespace utility {
#define PAUSE()                                                                \
  do {                                                                         \
    printf("---------------press Enter key to exit!---------------\n");        \
    getchar();                                                                 \
  } while (0)

/* linux system */
#ifdef __linux__

// delay us
inline void delay_us(uint32_t us) { usleep(us); }

// delay ms
inline void delay_ms(uint32_t ms) { usleep(ms * 1000); }

// get process time, ms
class GetTime {
private:
  struct timeval tv1;
  struct timeval tv2;
  long t1, t2, time;

public:
  // get start time
  inline void Start() { gettimeofday(&tv1, NULL); }
  // get stop time
  inline void Stop() { gettimeofday(&tv2, NULL); }

  // get process time
  inline long Process() {
    // gettimeofday(&tv2, NULL);
    t1 = tv2.tv_sec - tv1.tv_sec;
    t2 = tv2.tv_usec - tv1.tv_usec;
    time = (long)(t1 * 1000 + t2 / 1000);
    return time;
  }

  GetTime() {}
  ~GetTime() {}
};

/* file system */
int CreateDirectory(const char *dir_name);

inline std::string GetSystemTime() {
  std::time_t time;
  time = std::time(nullptr);
  std::tm *tm;
  tm = std::localtime(&time);
  return std::to_string(tm->tm_year + 1900) + "_" +
         std::to_string(tm->tm_mon + 1) + "_" + std::to_string(tm->tm_mday) +
         "_" + std::to_string(tm->tm_hour) + "_" + std::to_string(tm->tm_min);
}

/* endianness */
// inline void ChangeEndian(unsigned short& in){
//   in = (unsigned short)(in << 8 | in >> 8);
// }

inline unsigned short ChangeEndian(unsigned short const &in)
{
  return (unsigned short)(in << 8 | in >> 8);
}
#endif

}; // namespace utility