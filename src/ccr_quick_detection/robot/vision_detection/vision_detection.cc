#include "ccr_common.hpp"
#include "gst_appsrc.h"
#include "utility.hpp"
App app_0;

int main(int argc, char **argv)
{
// init logger
#if DEBUG_ON
  // set log level
  log_set_level(1);
#else
  log_set_level(2);
#endif
  int cam_cnt = 4;
  // init do3think camera
  int ret = Camera::init(cam_cnt, S_RAW8);
  if (ret != 0)
  {
    log_error("do3think cam init failed!");
    exit(-1);
  }

  // not enough parameter
  if (argc < 3)
  {
    log_info("not enough parameter!");
    log_info("example: <exe> host_ip host_port");
    exit(-1);
  }

  //   init parameter
  app_0.id = 0;
  app_0.source_name = "do3think_source";
  app_0.host_ip = argv[1];
  app_0.port = atoi(argv[2]);
  app_0.cam_cnt = cam_cnt;
  // get shared mem
  app_0.vision_msg = (ccr_quick_detection::VisionMessage *)get_shared_memory(
      ccr_quick_detection::kShareMemName.c_str(),
      ccr_quick_detection::kSemaphoreName.c_str(),
      ccr_quick_detection::kSharedMemLength);

  // wait vision system on
  while (app_0.vision_msg->vision_system_control != 1)
  {
    sleep(1);
    log_info("wait vision system on command.");
  }

  /* create video stream thread */
  std::thread video_stream_thread(VideoStream, std::ref(app_0));

  /* create save pics thread, create first */
  std::thread save_cam_pics_thread(SavePics, std::ref(app_0));

  /* create take pics thread */
  std::thread take_cam_pics_thread(TakePic, std::ref(app_0));

  video_stream_thread.join();
  take_cam_pics_thread.join();
  save_cam_pics_thread.join();

  Camera::fini();
  log_info("shutdown vision detection system.");

  return 0;
}
