#pragma once
#include <gst/gst.h>

#include "camera.hpp"
#include "ccr_common.hpp"
#include "common.hpp"
#include "data_structure.hpp"
#include "do3think.hpp"
#include "image_proc.hpp"
#include "log.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgcodecs/imgcodecs.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "shared_mem.h"

#define DEBUG_ON 1

// resize
// 目前分辨率360x240，帧率15,h264,vlc缓存1s，效果相对稳定，猜测编码器对图片的长宽和帧率有限制。
#define RESIZE_WIDTH 360
#define RESIZE_HEIGHT 240
#define SAMPLE_RATE 5

#define MAX_CAM_NUM 4
#define VIDEO_WIDTH 1280
#define VIDEO_HEIGHT 1024

#define IN_MMP_SIZE VIDEO_WIDTH *VIDEO_HEIGHT * 3
#define OUT_MMP_SIZE RESIZE_WIDTH *RESIZE_HEIGHT * 3

#define STREAM_WIDTH RESIZE_WIDTH * 2
#define STREAM_HEIGHT RESIZE_HEIGHT * 2
#define CONCAT_IMG_SIZE STREAM_WIDTH *STREAM_HEIGHT * 3

static constexpr int YUV_SIZE = STREAM_WIDTH * STREAM_HEIGHT * 3 / 2;

struct App {
  // camera cnt
  uint8_t cam_cnt;
  uint8_t id;
  GstElement *pipeline;
  GstElement *appsrc;
  gchar *datasrc_pipeline_str;
  GMainLoop *loop;
  GstBuffer *buffer;

  guint64 num_samples; /* Number of samples generated so far (for timestamp
                          generation) */

  guint sourceid; /* To control the GSource */

  GstClockTime timestamp;

  //   source name
  std::string source_name;

  //   host ip
  std::string host_ip;

  //   host port
  int port;

  // end of stream
  gboolean is_eos;

  // shared memory
  ccr_quick_detection::VisionMessage *vision_msg;
};

void GstAppPipInit(App &app, void (*call_back)(GstElement *));
void GstAppPipInit(App &app);

void GstAppSrcPlay(App &app);
void GstAppPipStop(App &app);

// save picture
void SavePics(App &app);

// do3think takepicture to save
void TakePic(App &app);

// video stream
void VideoStream(App &app);
