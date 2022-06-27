#include "gst_appsrc.h"
#include "log.hpp"
#include "motion.hpp"
#include "udmabuf.hpp"
#include "utility.hpp"
//#include "xstream_preprocess.h"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>
static std::atomic<bool> take_img_flag;
static std::condition_variable queue_cond;
static std::mutex queue_mu;
// #define SAVE_PIC
#define USE_OPENMP 1

#define QUEUE_DEBUG 1

/* usrspace dma buffer */
static common::udmabuf udmabuf_in[MAX_CAM_NUM] = {{"udmabuf0", IN_MMP_SIZE},
                                                  {"udmabuf1", IN_MMP_SIZE},
                                                  {"udmabuf2", IN_MMP_SIZE},
                                                  {"udmabuf3", IN_MMP_SIZE}};

static common::udmabuf udmabuf_concat_img{"udmabuf4", CONCAT_IMG_SIZE};
static common::udmabuf udmabuf_y{"udmabuf5", STREAM_WIDTH *STREAM_HEIGHT};
static common::udmabuf udmabuf_uv{"udmabuf6", STREAM_WIDTH *STREAM_HEIGHT / 2};

/* hardware accelerator */
//static XStream_preprocess accel;

// frame count
static long long cnt = 0;

// do3think image
static dvpFrame frame_info_buf_gst[MAX_CAM_NUM];
uint8_t *pBuffer_gst[MAX_CAM_NUM];

static gboolean datasrc_message(GstBus *bus, GstMessage *message, App *app)
{
  switch (GST_MESSAGE_TYPE(message))
  {
  case GST_MESSAGE_ERROR:
    g_error("\nReceived error from datasrc_pipeline...\n");
    if (g_main_loop_is_running(app->loop))
      g_main_loop_quit(app->loop);
    break;
  case GST_MESSAGE_EOS:
    g_print("\nReceived EOS from datasrc_pipeline...\n");
    app->is_eos = TRUE;
    if (g_main_loop_is_running(app->loop))
      g_main_loop_quit(app->loop);
    break;
  default:
    break;
  }
  return TRUE;
}

static int AppSrcGetFrame(App *app)
{
  // concat vector
  int ret[MAX_CAM_NUM];

  if (take_img_flag == false)
  {
#if USE_OPENMP
#pragma omp parallel for
#endif
    for (int i = 0; i < app->cam_cnt; i++)
    {

      // check result
      do
      {
        // retry
        ret[i] = Camera::TakePicture(i, &frame_info_buf_gst[i],
                                     (void **)&pBuffer_gst[i], 100);
      } while (ret[i] != DVP_STATUS_OK);

      // copy data to hw accelerator
      memcpy(udmabuf_in[i].GetVirtualAddr(), (char *)pBuffer_gst[i],
             IN_MMP_SIZE);
    }
  }

  // set odometer number
  /*XStream_preprocess_Set_text_num(&accel, (int)(app->vision_msg->odometer));

#if 0
  // cv::cvtColor(stream_frame, stream_yuv, CV_RGB2YUV_I420);
  common::I4202NV12((char*)stream_yuv.data, STREAM_WIDTH, STREAM_HEIGHT);

  // use hardware
#else
  // start
  XStream_preprocess_Start(&accel);

  // wait done
  while (!XStream_preprocess_IsDone(&accel))
  {
    usleep(200);
  }

#endif
*/
  cnt++;
  return 0;
}

/* push data into appsrc */
static gboolean push_data(App *app)
{
  GstBuffer *buffer;
  GstFlowReturn ret;
  GstMapInfo gst_map;

  // once a frame
  gint num_samples = 1;

  buffer = gst_buffer_new_and_alloc(YUV_SIZE);

  AppSrcGetFrame(app);

  gst_buffer_map(buffer, &gst_map, GST_MAP_WRITE);
  // memcpy((guchar *)gst_map.data, (char *)yuv, YUV_SIZE);
  memcpy((guchar *)gst_map.data, (unsigned char *)udmabuf_y.GetVirtualAddr(),
         STREAM_WIDTH * STREAM_HEIGHT);
  memcpy((guchar *)(gst_map.data + (STREAM_WIDTH * STREAM_HEIGHT)),
         (unsigned char *)udmabuf_uv.GetVirtualAddr(),
         STREAM_WIDTH * STREAM_HEIGHT / 2);

  /* Set its timestamp and duration */
  GST_BUFFER_TIMESTAMP(buffer) =
      gst_util_uint64_scale(app->num_samples, GST_SECOND, SAMPLE_RATE);
  GST_BUFFER_DURATION(buffer) =
      gst_util_uint64_scale(num_samples, GST_SECOND, SAMPLE_RATE);

  /* Push the buffer into the appsrc */
  g_signal_emit_by_name(app->appsrc, "push-buffer", buffer, &ret);
  gst_buffer_unmap(buffer, &gst_map);
  app->num_samples += num_samples;
  /* Free the buffer now that we are done with it */
  gst_buffer_unref(buffer);

  /* stop stream */
  if (app->vision_msg->host_motion_cmd.video_stream_control == 0)
  {
    /* no more buffers left in queue and datasrc_pipeline is EOS */
    g_signal_emit_by_name(app->appsrc, "end-of-stream", &ret);
    return TRUE;
  }

  if (ret != GST_FLOW_OK)
  {
    /* We got some error, stop sending data */
    log_error("push data error.");
    // /* no more buffers left in queue and datasrc_pipeline is EOS */
    // g_signal_emit_by_name(app->appsrc, "end-of-stream", &ret);
    return FALSE;
  }

  return TRUE;
}

/* This signal callback triggers when appsrc needs data. Here, we add an idle
 * handler to the mainloop to start pushing data into the appsrc */
static void start_feed(GstElement *source, guint size, App *app)
{
  if (app->sourceid == 0)
  {
    dlog_if(DEBUG_ON, "Start feeding");
    app->sourceid = g_idle_add((GSourceFunc)push_data, app);
  }
}

/* This callback triggers when appsrc has enough data and we can stop sending.
 * We remove the idle handler from the mainloop */
static void stop_feed(GstElement *source, App *app)
{
  if (app->sourceid != 0 ||
      (app->vision_msg->host_motion_cmd.video_stream_control) == 0)
  {
    dlog_if(DEBUG_ON, "Stop feeding");
    g_source_remove(app->sourceid);
    app->sourceid = 0;
  }
}

// init appsrc
void GstAppPipInit(App &app)
{
  app.datasrc_pipeline_str = g_strdup_printf(
      "appsrc name=%s "
      "caps=video/x-raw,format=%s,width=%d,height=%d,framerate=%d/1 ! "
      "omxh265enc qp-mode=auto gop-mode=basic gop-length=60 "
      "target-bitrate=2000 control-rate=low-latency ! video/x-h265 "
      "!queue ! mpegtsmux alignment=7 name=mux ! "
      "rtpmp2tpay !"
      "udpsink host=%s port=%d max-lateness=-1 qos-dscp=60 async=false "
      "max-bitrate=60000000 -v",
      app.source_name.c_str(), "NV12", STREAM_WIDTH, STREAM_HEIGHT, SAMPLE_RATE,
      app.host_ip.c_str(), app.port);

  app.pipeline = gst_parse_launch(app.datasrc_pipeline_str, NULL);

  /* add watch for messages */
  GstBus *datasrc_bus = gst_element_get_bus(app.pipeline);
  gst_bus_add_watch(datasrc_bus, (GstBusFunc)datasrc_message, &app);

  /* setup appsrc */
  app.appsrc =
      gst_bin_get_by_name(GST_BIN(app.pipeline), app.source_name.c_str());
  g_object_set(G_OBJECT(app.appsrc), "stream-type", 0, "format",
               GST_FORMAT_TIME, NULL);
  // g_signal_connect(app.appsrc, "need-data", G_CALLBACK(call_back), NULL);
  g_signal_connect(app.appsrc, "need-data", G_CALLBACK(start_feed), &app);
  g_signal_connect(app.appsrc, "enough-data", G_CALLBACK(stop_feed), &app);

  log_info("Changing state of pipeline %s to PLAYING...",
           app.source_name.c_str());
  gst_element_set_state(app.pipeline, GST_STATE_PLAYING);
}

void GstAppSrcPlay(App &app)
{
  log_info("Changing state of pipeline to PLAYING...");
  gst_element_set_state(app.pipeline, GST_STATE_PLAYING);
  g_main_loop_run(app.loop);
}

// stop loop
void GstAppPipStop(App &app)
{
  log_info("Changing state of pipeline %s to NULL...\n",
           app.source_name.c_str());
  /* clean up */
  gst_element_set_state(app.pipeline, GST_STATE_NULL);
  gst_object_unref(GST_OBJECT(app.pipeline));
}

/* queue */
#define USE_QUEUE 1

struct RawImg
{
  int odom;
  uint8_t data[VIDEO_WIDTH * VIDEO_HEIGHT * 3];
};

struct RawMat
{
  int odom;
  cv::Mat raw_mat{VIDEO_HEIGHT, VIDEO_WIDTH, CV_8UC3, cv::Scalar::all(0)};
};

// queue to store raw image
static const int kQueueLen = 100;
static common::Queue<RawMat> raw_mat_queue[MAX_CAM_NUM];

/*
push raw image to queue
 */
static void PushRaw(const App &app, const RawMat *tmp_raw_mat)
{
  dlog_if(QUEUE_DEBUG, "take pic thread push to queue.");

  std::unique_lock<std::mutex> lock(queue_mu);
  // std::lock_guard<std::mutex> lock(queue_mu);
  dlog_if(QUEUE_DEBUG, "take pic thread get lock.");
#pragma omp parallel for
  for (int i = 0; i < app.cam_cnt; i++)
  {
    // push to queue
    if (raw_mat_queue[i].QueueISFull())
    {
      log_info("raw_mat_queue[i] full!", i);
    }
    else
    {
      raw_mat_queue[i].EnQueue(std::move(tmp_raw_mat[i]));
    }
  }
  lock.unlock();
  dlog_if(QUEUE_DEBUG, "take pic thread free lock.");
  queue_cond.notify_one();
}

/* take picture */
void TakePic(App &app)
{
  int ret[MAX_CAM_NUM];
  dvpFrame frame_info_buf[MAX_CAM_NUM];
  static uint8_t pBuffer[MAX_CAM_NUM][16 * 1024 * 1024];
  RawMat tmp_raw_mat[MAX_CAM_NUM];

  // odometer counter
  double odm_last_time = 0;
  double odm_this_time = 0;
  double odm_delta;

  // take picture when robot does not fallback
  while (app.vision_msg->host_motion_cmd.motion_mode !=
             ccr_quick_detection::kReturnMode &&
         app.vision_msg->vision_system_control == 1)
  {

    if (app.vision_msg->host_motion_cmd.take_image_control == 1)
    {
      // check odometer
      odm_this_time = app.vision_msg->odometer;
      // log_debug("odm this time:%f", odm_this_time);
      odm_delta = abs(odm_this_time - odm_last_time);

      /* save raw image */
      if (odm_delta > ccr_quick_detection::kDefaultSampleInterval)
      {
        // set take picture flag
        take_img_flag = true;
        // get odometer
        for (int i = 0; i < MAX_CAM_NUM; i++)
        {
          tmp_raw_mat[i].odom = (int)(app.vision_msg->odometer);
        }
        dlog_if(QUEUE_DEBUG, "start take picture.");
        /* take picture */
#pragma omp parallel for
        //for (int i = 0; i < app.cam_cnt; i++)
        for (int i = 0; i < 4; i++)
        {
          do
          {
            // retry
            ret[i] = Camera::TakePicture(i, frame_info_buf + i, pBuffer[i], 100);
          } while (ret[i] != DVP_STATUS_OK);

          memcpy((char *)tmp_raw_mat[i].raw_mat.data, pBuffer[i],
                 IN_MMP_SIZE);
          // tmp_raw_mat[i].odom = cam_odm[i];
        }
        // push to queue
        PushRaw(app, tmp_raw_mat);
        take_img_flag = false;
        odm_last_time = odm_this_time;
      }
    }
    else
    {
      // queue_cond.notify_one();
    }
    // refresh every 50ms
    usleep(50000);
  }
  log_info("exit take pic thread.");
}

#define IMAGE_COMPRESS 0

/*
compress and save image
*/
static void CompressImg(const App &app, const std::string *save_folder_name)
{
  utility::GetTime compress_jpg_time, save_raw_time;
  // temp raw mat
  RawMat *tmp_mat[MAX_CAM_NUM];

  // wait until queue is not empty
  if (raw_mat_queue[0].Empty() || raw_mat_queue[1].Empty() ||
      raw_mat_queue[2].Empty() || raw_mat_queue[3].Empty() ||
      app.vision_msg->host_motion_cmd.take_image_control != 1)
  {
  }
  else
  {
    dlog_if(QUEUE_DEBUG, "save pic thread pop from queue.");
    std::unique_lock<std::mutex> lock(queue_mu);
    // std::lock_guard<std::mutex> lock(queue_mu);
    dlog_if(QUEUE_DEBUG, "save pic thread get lock.");
#if 0

    dlog_if(QUEUE_DEBUG, "save pic thread wait condition.");
    queue_cond.wait(lock, []() {
      // queue size must >=2! don't know why yet...
      return ((raw_mat_queue[0].Size() > 1 && raw_mat_queue[1].Size() > 1 &&
        raw_mat_queue[2].Size() > 1 && raw_mat_queue[3].Size() > 1));
      });
    dlog_if(QUEUE_DEBUG, "save pic thread condition reached.");
    // print debug info
    dlog_if(QUEUE_DEBUG, "raw_mat_queue[0] size:%d", raw_mat_queue[0].Size());
    dlog_if(QUEUE_DEBUG, "raw_mat_queue[1] size:%d", raw_mat_queue[1].Size());
    dlog_if(QUEUE_DEBUG, "raw_mat_queue[2] size:%d", raw_mat_queue[2].Size());
    dlog_if(QUEUE_DEBUG, "raw_mat_queue[3] size:%d", raw_mat_queue[3].Size());
#else
    queue_cond.wait(lock);
#endif

    for (int i = 0; i < MAX_CAM_NUM; i++)
    {
      // get data pointer
      tmp_mat[i] = raw_mat_queue[i].DeQueue();
      // compress by opencv
#if IMAGE_COMPRESS
      compress_jpg_time.Start();
      cv::imwrite((save_folder_name[i] + "/" + "cam" + std::to_string(i) + "_" +
                   std::to_string(tmp_mat[i]->odom) + ".png")
                      .c_str(),
                  tmp_mat[i]->raw_mat);

      compress_jpg_time.Stop();
      log_info("compress cam%d img time:%dms", i, compress_jpg_time.Process());
#else
      // save raw data
      save_raw_time.Start();
      FILE *fp = fopen((save_folder_name[i] + "/" + "cam" + std::to_string(i) + "_" +
                        std::to_string(tmp_mat[i]->odom) + ".raw")
                           .c_str(),
                       "wb");
      fwrite(tmp_mat[i], VIDEO_WIDTH * VIDEO_HEIGHT * 3, 1, fp);
      fclose(fp);
      save_raw_time.Stop();
      log_info("cam%d save raw img time:%dms", i, save_raw_time.Process());
#endif
    }
    dlog_if(QUEUE_DEBUG, "save pic thread free lock.");
    // lock.unlock();
  }
}

/* save image */
void SavePics(App &app)
{

  // cam folder name
  std::string save_folder_name[MAX_CAM_NUM];

  /* create folders */
  // wait bridge_info_set signal
  while (app.vision_msg->host_motion_cmd.bridge_info_set != 1)
  {
    sleep(1);
    log_info("please set bridge information");
  }

  // bridge name
  std::string bridge_name =
      std::to_string(app.vision_msg->host_motion_cmd.bridge_info.name) + "_" +
      std::to_string(app.vision_msg->host_motion_cmd.bridge_info.cable_no) +
      "_" + std::to_string(app.vision_msg->host_motion_cmd.bridge_info.cable_diameter) +
      "_" +
      std::to_string(app.vision_msg->host_motion_cmd.bridge_info.cable_len);

  // create bridge folder
  utility::CreateDirectory(bridge_name.c_str());

  // save images
  while (app.vision_msg->vision_system_control == 1)
  {

    if (app.vision_msg->host_motion_cmd.take_image_control == 1)
    {
      // get system time
      // std::string sys_time = utility::GetSystemTime();
      std::string sys_time;
      sys_time = std::to_string(utility::ChangeEndian(app.vision_msg->host_motion_cmd.time_stamp.year)) + "_" +
                 std::to_string(utility::ChangeEndian(app.vision_msg->host_motion_cmd.time_stamp.month)) + "_" +
                 std::to_string(utility::ChangeEndian(app.vision_msg->host_motion_cmd.time_stamp.day)) + "_" +
                 std::to_string(utility::ChangeEndian(app.vision_msg->host_motion_cmd.time_stamp.hour)) + "_" +
                 std::to_string(utility::ChangeEndian(app.vision_msg->host_motion_cmd.time_stamp.minute));
      for (int i = 0; i < MAX_CAM_NUM; i++)
      {
        // get folder names
        save_folder_name[i] =
            bridge_name + "/" + "cam" + std::to_string(i) + "_" + sys_time;
        utility::CreateDirectory(save_folder_name[i].c_str());
      }

      while (app.vision_msg->host_motion_cmd.motion_mode !=
                 ccr_quick_detection::kReturnMode &&
             app.vision_msg->host_motion_cmd.take_image_control == 1)
      {
        CompressImg(app, save_folder_name);
        // save every 200ms
        std::this_thread::sleep_for(std::chrono::microseconds(200));
      }
      //  finish save pics
      log_info("all image saved.");
    }
    // save every 200ms
    std::this_thread::sleep_for(std::chrono::microseconds(200));
  }
  log_info("exit save pic thread.");
}

void VideoStream(App &app)
{
  int ret;
  /* init queue */
  for (int i = 0; i < MAX_CAM_NUM; i++)
  {
    raw_mat_queue[i].Create(kQueueLen);
  }

  /* init GStreamer */
  gst_init(NULL, NULL);
  app.loop = g_main_loop_new(NULL, FALSE);

  /* init hardware accelerator */
  /*ret = XStream_preprocess_Initialize(&accel, "stream_preprocess");

  if (ret == 0)
  {
    log_info("hw accelerator init success.");
  }
  else
  {
    log_error("hw accelerator init failed!");
    exit(-1);
  }

  // set hw accel parameter
  XStream_preprocess_Set_img_inp0(&accel, udmabuf_in[0].GetPhyAddr());
  XStream_preprocess_Set_img_inp1(&accel, udmabuf_in[1].GetPhyAddr());
  XStream_preprocess_Set_img_inp2(&accel, udmabuf_in[2].GetPhyAddr());
  XStream_preprocess_Set_img_inp3(&accel, udmabuf_in[3].GetPhyAddr());
  XStream_preprocess_Set_imgOutput0(&accel, udmabuf_y.GetPhyAddr());
  XStream_preprocess_Set_imgOutput1(&accel, udmabuf_uv.GetPhyAddr());
  */
  for (;;)
  {
    if (app.vision_msg->vision_system_control == 1)
    {
      // check stream control
      if (app.vision_msg->host_motion_cmd.video_stream_control == 1)
      {

        // init pipline
        log_info("init pipeline.");
        GstAppPipInit(app);

        // start main loop
        log_info("start main loop.");
        g_main_loop_run(app.loop);

        // stop main loop
        GstAppPipStop(app);
      }
    }
    else
    {
      // vision system off
      break;
    }

    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
  log_info("exit video stream thread.");

  // release hardware accelerator
  //XStream_preprocess_Release(&accel);
}