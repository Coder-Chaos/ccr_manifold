#include "uvc.hpp"

#include <stdio.h>

#include "common.hpp"

#if 0
// init udp client
common::Net udp_client{common::protocal_type::UDP, common::type::CLIENT, "10.60.2.97", 7000};

// init image processor
quick_inspection::image_process image_processor{std::move(udp_client)};

quick_inspection::visual_buffer_info_t visual_buf;
#endif

uvc::uvc(/* args */) {}

uvc::uvc(int img_width, int img_height, const char *uvc_device)
    : img_width_(img_width), img_height_(img_height), uvc_device_(uvc_device) {}

uvc::~uvc() {}

int uvc::Init()
{
  int s32Ret = 0;

  // 1 open device
  s32Ret = Open();
  if (s32Ret < 0)
  {
    printf("open camera failed ! \n");
    return -1;
  }

  // Check and set device properties  set frame
  s32Ret = InitCamera();
  if (s32Ret < 0)
  {
    printf("init camera failed ! \n");
    CloseCamera();
    return -1;
  }

  uvc_camera_connected = true;

  SetFormat();

  // Apply for a video buffer
  InitMmp();

  // start capture
  StartCapture();

  return 0;
}

int uvc::DeInit()
{
  // CloseCamera();

  StopCapture();

  CloseCamera();

  return 0;
}

int uvc::Open()
{
  struct v4l2_input inp;
  int i = 0;
  int ret = -1;
  g_video_fd = open(uvc_device_, O_RDWR | O_NONBLOCK, 0);
  if (g_video_fd < 0)
  {
    printf("%s open failed ! \n", uvc_device_);
    return ret;
  };

  for (i = 0; i < 16; i++)
  {
    inp.index = i;
    if (-1 == ioctl(g_video_fd, VIDIOC_S_INPUT, &inp))
    {
      printf("VIDIOC_S_INPUT  failed %d !\n", i);
    }
    else
    {
      printf("VIDIOC_S_INPUT  success %d !\n", i);
      ret = 0;
      break;
    }
  }

  return ret;
}

int uvc::InitCamera()
{
  struct v4l2_capability cap;  /* decive fuction, such as video input */
  struct v4l2_fmtdesc fmtdesc; /* detail control value */

  int ret = 0;
  if (g_video_fd <= 0)
    return -1;

  /*show all the support format*/
  memset(&fmtdesc, 0, sizeof(fmtdesc));
  fmtdesc.index = 0; /* the number to check */
  fmtdesc.type = V4L2_BUF_TYPE;

  /* check video decive driver capability */
  if (ret = ioctl(g_video_fd, VIDIOC_QUERYCAP, &cap) < 0)
  {
    fprintf(stderr, "fail to ioctl VIDEO_QUERYCAP \n");
    return -1;
  }

  /*judge wherher or not to be a video-get device*/
  if (!(cap.capabilities & V4L2_BUF_TYPE))
  {
    fprintf(stderr, "The Current device is not a video capture device \n");
    return -1;
  }

  /*judge whether or not to supply the form of video stream*/
  if (!(cap.capabilities & V4L2_CAP_STREAMING))
  {
    printf("The Current device does not support streaming i/o\n");
    return -1;
  }

  printf("\ncamera driver name is : %s\n", cap.driver);
  printf("camera device name is : %s\n", cap.card);
  printf("camera bus information: %s\n", cap.bus_info);

  /*display the format device support*/
  printf("\n");
  while (ioctl(g_video_fd, VIDIOC_ENUM_FMT, &fmtdesc) != -1)
  {
    printf("support device %d.%s\n", fmtdesc.index + 1, fmtdesc.description);
    fmtdesc.index++;
  }
  printf("\n");

  return 0;
}

int uvc::SetFormat()
{
  struct v4l2_format tv_fmt; /* frame format */

  /*set the form of camera capture data*/
  tv_fmt.type = V4L2_BUF_TYPE; /*v4l2_buf_typea,camera must use
                                  V4L2_BUF_TYPE_VIDEO_CAPTURE*/
  tv_fmt.fmt.pix.width = img_width_;
  tv_fmt.fmt.pix.height = img_height_;
  tv_fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
  tv_fmt.fmt.pix.field = V4L2_FIELD_NONE; /*V4L2_FIELD_NONE*/
  if (ioctl(g_video_fd, VIDIOC_S_FMT, &tv_fmt) < 0)
  {
    fprintf(stderr, "VIDIOC_S_FMT set err\n");
    return -1;
  }
  return 0;
}

int uvc::InitMmp()
{
  /*to request frame cache, contain requested counts*/
  struct v4l2_requestbuffers reqbufs;

  memset(&reqbufs, 0, sizeof(reqbufs));
  reqbufs.count = BUFFER_NUM; /*the number of buffer*/
  reqbufs.type = V4L2_BUF_TYPE;
  reqbufs.memory = V4L2_MEMORY_MMAP;

  if (-1 == ioctl(g_video_fd, VIDIOC_REQBUFS, &reqbufs))
  {
    perror("Fail to ioctl 'VIDIOC_REQBUFS'");
    return -1;
  }

  n_buffer = reqbufs.count;
  printf("n_buffer = %d\n", n_buffer);
  usr_buf = (BUFTYPE *)calloc(reqbufs.count, sizeof(BUFTYPE));
  if (usr_buf == NULL)
  {
    printf("Out of memory\n");
    return -1;
  }

  /*map kernel cache to user process*/
  for (n_buffer = 0; n_buffer < reqbufs.count; n_buffer++)
  {
    // stand for a frame
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = n_buffer;

    /*check the information of the kernel cache requested*/
    if (-1 == ioctl(g_video_fd, VIDIOC_QUERYBUF, &buf))
    {
      perror("Fail to ioctl : VIDIOC_QUERYBUF");
      return -1;
    }

    usr_buf[n_buffer].length = buf.length;
    usr_buf[n_buffer].start =
        (char *)mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED,
                     g_video_fd, buf.m.offset);

    if (MAP_FAILED == usr_buf[n_buffer].start)
    {
      perror("Fail to mmap");
      return -1;
    }
  }

  return 0;
}

int uvc::StartCapture()
{
  unsigned int i;
  enum v4l2_buf_type type;

  /*place the kernel cache to a queue*/
  for (i = 0; i < n_buffer; i++)
  {
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;

    if (-1 == ioctl(g_video_fd, VIDIOC_QBUF, &buf))
    {
      perror("Fail to ioctl 'VIDIOC_QBUF'");
      exit(EXIT_FAILURE);
    }
  }

  type = V4L2_BUF_TYPE;
  if (-1 == ioctl(g_video_fd, VIDIOC_STREAMON, &type))
  {
    printf("i=%d.\n", i);
    perror("VIDIOC_STREAMON");
    close(g_video_fd);
    exit(EXIT_FAILURE);
  }

  return 0;
}

int uvc::CloseCamera()
{
  unsigned int i;

  for (i = 0; i < n_buffer; i++)
  {
    if (-1 == munmap(usr_buf[i].start, usr_buf[i].length))
    {
      exit(-1);
    }
  }

  if (NULL != usr_buf)
    free(usr_buf);

  if (g_video_fd > 0)
    close(g_video_fd);

  return 0;
}

int uvc::StopCapture()
{
  enum v4l2_buf_type type;
  type = V4L2_BUF_TYPE;
  if (-1 == ioctl(g_video_fd, VIDIOC_STREAMOFF, &type))
  {
    perror("Fail to ioctl 'VIDIOC_STREAMOFF' \n");
    return -1;
  }
  return 0;
}

int uvc::ReadFrame()
{
  // struct v4l2_buffer buf;
  // VDEC_STREAM_S stStream;

  int s32Ret;

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

#if 0
    // send one frame
    stStream.u64PTS = 0;
    // address
    stStream.pu8Addr = (HI_U8 *)(usr_buf[buf.index].start);
    // length
    stStream.u32Len = usr_buf[buf.index].length;
    stStream.bEndOfFrame = true;
    stStream.bEndOfStream = true;
    stStream.bDisplay = true;

    // printf("send a frame to vdec channel 0.\n");
    // send frame to vdec
    s32Ret = HI_MPI_VDEC_SendStream(0, &stStream, 1000);

#endif

// process frame
#if 0
    // save a file
    FILE *image = fopen("uvc.jpg", "wb");

    fwrite((char *)(usr_buf[buf.index].start), usr_buf[buf.index].length, 1, image);

    fclose(image);
#endif

#if 0
    quick_inspection::photo_info_t *photo_info = (quick_inspection::photo_info_t *)visual_buf.data;

    photo_info->position = 4;
    photo_info->group = 0;
    photo_info->sequence = 0;
    photo_info->time_stamp = frame_cnt_;
    photo_info->width = img_width_;
    photo_info->height = img_height_;
    photo_info->bytes_per_pixel = 3;
    photo_info->reserved = 0;

    memcpy((char *)visual_buf.data + sizeof(quick_inspection::photo_info_t), (char *)usr_buf[buf.index].start, usr_buf[buf.index].length);

    visual_buf.data_length = usr_buf[buf.index].length + sizeof(quick_inspection::photo_info_t);
    visual_buf.position = 4;
    visual_buf.time_stamp = frame_cnt_;
    // udp send
    image_processor.send_visual(visual_buf);
#endif

  if (-1 == ioctl(g_video_fd, VIDIOC_QBUF, &q_buf_))
  {
    perror("Fail to ioctl 'VIDIOC_QBUF'");
    return -1;
  }

  return 0;
}

// uvc streaming video
int uvc::Stream()
{
  // loop while uvc camera connected
  while (uvc_camera_connected == true)
  {
    fd_set fds;
    struct timeval tv;
    int r;

    FD_ZERO(&fds);
    FD_SET(g_video_fd, &fds);

    /*Timeout*/
    tv.tv_sec = 2;
    tv.tv_usec = 0;
    r = select(g_video_fd + 1, &fds, NULL, NULL, &tv);

    if (-1 == r)
    {
      if (EINTR == errno)
        continue;
      perror("Fail to select \n");
      return -1;
    }
    if (0 == r)
    {
      fprintf(stderr, "select Timeout \n");
      return -1;
    }

    // dequeue a filled (capturing) or displayed (output) buffer from the
    // driver’s outgoing queue
    if (DQBuf() == 0)
    {
      // increase frame count
      frame_cnt_++;
      break;
    }
    else
    {
      // fail to open device
      uvc_camera_connected = false;
      // close camera
      DeInit();
    }
  }

  // try to reconncet uvc camera
  while (uvc_camera_connected != true)
  {
    Init();
    // retry after 100ms
    usleep(100000);
  }

  return 0;
}