#include "image_proc.hpp"
namespace common {

int I4202NV12(char *yuv_i420, int width, int height) {
  char *tmp = yuv_i420; // if you get YUVphead only
  int i;

  char aa[width * height / 2]; // save UV
  char *x = tmp;
  char *y;
  char *p;
  char *sp;
  int temp;
  temp = width * height;
  while (temp--) {
    y++;
    x++; // ydata no change
  }

  temp = width * height / 4;
  p = x; // uhead
  sp = x;
  while (temp--)
    x++; // vhead

  // NV21:VUVU
  // for (i = 0; i < width * height / 2; i++) {
  //   if (i % 2 == 0)  // v
  //     aa[i] = *x++;
  //   if (i % 2 == 1)  // u
  //     aa[i] = *p++;
  // }
  // NV12:UVUV change VU to UV
  for (i = 0; i < width * height / 2; i++) {
    if (i % 2 == 0) // u
      aa[i] = *p++;
    if (i % 2 == 1) // v
      aa[i] = *x++;
  }

  for (i = 0; i < width * height / 2; i++) // back to tmp
  {
    *sp++ = aa[i];
  }
  return 0;
};
} // namespace common