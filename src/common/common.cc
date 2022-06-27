#include "common.hpp"
#include "log.h"

namespace common{
static char **cmd_argv = NULL;

// signal handler
static void SignalHandler(int sig) {
  if (SIGINT == sig || SIGTERM == sig || SIGSEGV == sig || SIGPIPE == sig)
  {
    switch (sig) {
    case SIGINT:
    {
    }
    break;

    case SIGTERM:
    {
    }
    break;

    case SIGPIPE:
    {
      log_error("pipe broken.");
    }
    break;

    default: { } break; }
    
  }
  exit(-1);
}

void GlobalInit(int *pargc, char ***pargv) {
  cmd_argv = *pargv;

  // register signal handler
  signal(SIGINT, SignalHandler);
  signal(SIGTERM, SignalHandler);
  signal(SIGSEGV, SignalHandler);
}

void GlobalInit(void)
{
  // cmd_argv = *pargv;

  // register signal handler
  signal(SIGINT, SignalHandler);
  signal(SIGTERM, SignalHandler);
  signal(SIGSEGV, SignalHandler);
  signal(SIGPIPE, SignalHandler);
}
}