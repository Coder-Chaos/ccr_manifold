#include "utility.hpp"

namespace utility
{

  int CreateDirectory(const char *dir_name)
  {
    struct stat st = {0};
    int ret;
    // check if folder already exists
    if (stat(dir_name, &st) == -1)
    {
      log_info("create folder:%s", dir_name);
      ret = mkdir(dir_name, 0755);
    }
    else
    {
      log_info("save uvc pic folder:%s exists!", dir_name);
    }

    return ret;
  }
}