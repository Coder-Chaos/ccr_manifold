#pragma once

#include "common.hpp"

namespace common {
// convert I420 to NV12
int I4202NV12(char *yuv_i420, int width, int height);
}  // namespace common