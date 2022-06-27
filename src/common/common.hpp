#pragma once

// create folder
#include <sys/types.h>
#include <sys/stat.h>


#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <thread>

// #include "log.h"

namespace common {
// A global initialization function that you should call in your main function.
// Currently it initializes google flags and google logging.
void GlobalInit(int *pargc, char ***pargv);
void GlobalInit(void);

// global close
void GlobalClose(void);

}  // namespace common