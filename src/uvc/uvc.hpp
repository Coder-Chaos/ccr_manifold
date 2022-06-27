#pragma once
#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/poll.h>
#include <sys/prctl.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <thread>

#define VIDEO_NAME "/dev/video0"
#define TEST_STREAM_SAVE_PATH "./cap_image"
#define BUFFER_NUM (4)
#define V4L2_BUF_TYPE (V4L2_BUF_TYPE_VIDEO_CAPTURE)

typedef struct
{
  void *start;
  int length;
} BUFTYPE;

class uvc
{
private:
  // device
  const char *uvc_device_;

  struct v4l2_buffer frame_buf;

  unsigned int n_buffer = 0;

  // file handler
  int g_video_fd;

  // connect state
  bool uvc_camera_connected = false;

public:
  // image address
  BUFTYPE *usr_buf;

  struct v4l2_buffer q_buf_;

  // frame counter
  long long frame_cnt_ = 0;

  // camera input size
  int img_width_;
  int img_height_;

public:
  uvc(/* args */);
  uvc(int img_width, int img_height, const char *uvc_device);
  ~uvc();

  // init uvc
  int Init();

  // deinit uvc
  int DeInit();

  // open camera
  int Open();

  // init camera
  int InitCamera();

  // close camera
  int CloseCamera();

  // setup format
  int SetFormat();

  // init
  int InitMmp();

  // Start Capture
  int StartCapture();

  // int Stop Capture
  int StopCapture();

  // read frame
  int ReadFrame();

  // streaming video
  int Stream();

  // Exchange a buffer with the driver
  int DQBuf()
  {
    memset(&q_buf_, 0, sizeof(q_buf_));
    q_buf_.type = V4L2_BUF_TYPE;
    q_buf_.memory = V4L2_MEMORY_MMAP;

    // put cache from queue
    if (-1 == ioctl(g_video_fd, VIDIOC_DQBUF, &q_buf_))
    {
      perror("Fail to ioctl 'VIDIOC_DQBUF'");
      return -1;
    }

    if (q_buf_.index >= n_buffer)
      return -1;
    return 0;
  }

  int QBuf()
  {
    if (-1 == ioctl(g_video_fd, VIDIOC_QBUF, &q_buf_))
    {
      perror("Fail to ioctl 'VIDIOC_QBUF'");
      return -1;
    }

    return 0;
  }
};
