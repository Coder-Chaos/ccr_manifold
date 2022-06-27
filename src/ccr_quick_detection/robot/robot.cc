#include "robot.hpp"
#include "log.hpp"
#include <chrono>
#define SAVE_UVC_PIC 0

namespace ccr_quick_detection
{
  // folder name
  static std::string uvc_folder_name;

  void Robot::Guard()
  {
    uint16_t heart_beat_last, heart_beat_this;
    log_info("start listen host.");
    // heart_beat_flag_ = true;
    heart_beat_last = utility::ChangeEndian(motion_.motion_cmd_->time_stamp.second);
    
    guard_thread_ = std::thread([&]()
                                {
                                  for (;;)
                                  {
                                    // listen every 30s
                                    std::this_thread::sleep_for(std::chrono::seconds(30));
                                    heart_beat_this = utility::ChangeEndian(motion_.motion_cmd_->time_stamp.second);
                                    // check heart beat
                                    if (heart_beat_last == heart_beat_this)
                                    {
                                      log_error("I'm lost!!!!!");
                                      // set lost flag
                                      heart_beat_flag_ = false;
                                    }
                                    heart_beat_last = heart_beat_this;
                                  }
                                });
    guard_thread_.detach();
    
  }

  static void Piphandler(int sig)
  {
    if (SIGPIPE == sig)
    {
      log_error("pipe broken.");
    }
  }

  void
  Robot::ConnectHost()
  {
    log_info("try to connect host.");
    signal(SIGPIPE, Piphandler);
    // init modbus tcp
    if (eMBTCPInit(MODBUS_TCP_PORT) != MB_ENOERR)
    {
      log_error("Can't Initialize Modbus Stack!");
      log_error("Exit program!");
      // disable modbus stack
      (void)eMBDisable();

      // close modbus connection
      (void)eMBClose();
      exit(-1);
    }
    // enable protocal
    if (eMBEnable() != MB_ENOERR)
    {
      log_error("Can't enable modbus protocal!");
      exit(-1);
    }

    /* modbus poll thread */
    modbus_thread_ = std::thread([&]()
                                 {
                                   log_info("modbus start polling.");
                                   heart_beat_flag_ = true;
                                   while (false == disconnect_host_signal_)
                                   {
                                     if (eMBPoll() == MB_ENOERR)
                                     {
                                       // refresh rate 50ms
                                       usleep(50000);
                                     }
                                     else
                                     {
                                       log_error("Modbus Poll Error!");
                                       break;
                                     }
                                   }
                                   log_debug("exit modbus polling.");

#if 1
                                   // disable modbus stack
                                   (void)eMBDisable();

                                   // close modbus connection
                                   (void)eMBClose();
#endif
                                 });
  }

  void Robot::DisConnectHost()
  {
    log_info("disconnect host.");
    disconnect_host_signal_ = true;
    log_debug("wait modbus thread finish.");
    modbus_thread_.detach();

    log_info("host disconnected.");
  }

  void Robot::TFminiRecv()
  {
    //
    float dist = 0;
    std::string id = "TFmini_Plus";
    std::string portName = "/dev/tfminip";
    int baud_rate = 115200;
    tfmini_obj = new benewake::TFmini(portName, baud_rate);     
    Range_->radiation_type = INFRARED;
    Range_->field_of_view = 0.04f;
    Range_->min_range = 0.3f;
    Range_->max_range = 12.0f;   
    Range_->frame_id = id;
    
    /* tfmini recv thread */
    tfmini_thread_ = std::thread([&]()
                                 {
                                   log_info("tfmini start receiving.");
                                   while (stop_tfmini_signal_ == false)
                                   {
                                     dist = tfmini_obj->getDist();
                                     //printf("\rdist=%4.2f; max=%4.2f.\n", dist, Range_->max_range);
                                     if(dist > 0 && dist < Range_->max_range)
                                     {
                                       Range_->range = dist;
                                       //Range_->stamp = ros::Time::now();                         
                                       if(dist < 0.5f){
                                         //printf("\range %f.\n", Range_->range);
                                         //motion_.motion_cmd_->quick_stop = kQuickStopOn;
                                         motion_.motion_cmd_->command = kStopCmd;
                                         }
                                     }
                                     else if(dist == -1.0)
                                     {
                                       log_error("Failed to read data!");
                                       break;
                                     }
                                     else if(dist == 0.0)
                                     {
                                       log_error("Data 0.0 error!");
                                     }
                                     usleep(5000);
                                   }
                                   log_debug("exit tfmini receiving.");
#if 1
                                   tfmini_obj->closePort();
#endif
                                  });
  }

  void Robot::TFminiStop()
  {
    stop_tfmini_signal_ = true;
    tfmini_thread_.join();
    tfmini_obj->closePort();
    log_info("TFminiStop.");
  }

  void Robot::Run()
  {
    /*  */

    /* create motion thread */
    motion_thread_ = std::thread([&]()
                                 {
                                   //   motion thread
                                   while (false == motion_stop_signal_)
                                   {

                                     // motion fsm
                                     motion_.SystemMotionFSM(heart_beat_flag_);

                                     GetParam();

                                     // refresh rate ms
                                     usleep(1000 * kControlPeriod);
                                   }
                                 });
  }

#define TIME_STAMP_DEBUG 0

  void Robot::GetParam()
  {

    // get odometer
    unsigned int tmp_odm = (unsigned int)GetOdometer();

    param_->odometer = (int)((tmp_odm << 16) | (tmp_odm >> 16));

    // time stamp
    dlog_if(TIME_STAMP_DEBUG, "-----");
    // utility::ChangeEndian(motion_.motion_cmd_->time_stamp.year);
    // uint32_t time_stamp = (uint32_t)((((uint64_t)r15 << 48) + ((uint64_t)r14 << 32) + ((uint64_t)r13 << 16) + (uint64_t)r12) / (1000 * 10000));
    // struct tm *time = localtime(&time_stamp);

    // uint32_t time_stamp = utility::ChangeEndian((uint16_t)((motion_.motion_cmd_->time_stamp) << 48));

    dlog_if(TIME_STAMP_DEBUG, "year:%d", utility::ChangeEndian(motion_.motion_cmd_->time_stamp.year));
    dlog_if(TIME_STAMP_DEBUG, "month:%d", utility::ChangeEndian(motion_.motion_cmd_->time_stamp.month));
    dlog_if(TIME_STAMP_DEBUG, "day:%d", utility::ChangeEndian(motion_.motion_cmd_->time_stamp.day));
    dlog_if(TIME_STAMP_DEBUG, "hour:%d", utility::ChangeEndian(motion_.motion_cmd_->time_stamp.hour));
    dlog_if(TIME_STAMP_DEBUG, "minute:%d", utility::ChangeEndian(motion_.motion_cmd_->time_stamp.minute));
    dlog_if(TIME_STAMP_DEBUG, "second:%d", utility::ChangeEndian(motion_.motion_cmd_->time_stamp.second));

    // copy to shared memory
    // synchronize with semaphore
    // sem_wait(*command_semaphore);
    vision_msg_->odometer = (int)tmp_odm;
    memcpy((MotionCmd *)&(vision_msg_->host_motion_cmd),
           (MotionCmd *)motion_.motion_cmd_, sizeof(MotionCmd));
    // sem_post(*command_semaphore);

    // get speed, mm/s
    odom_this_time_ = GetOdometer();
    param_->speed =
        (short)((odom_this_time_ - odom_last_time_) * 1000 / kControlPeriod);
    // log_debug("delta odom:%f", odom_this_time_ - odom_last_time_);
    // log_debug("int size:%d", sizeof(int));
    odom_last_time_ = odom_this_time_;
    //if(odom_this_time_ > 1000.f &&(short)odom_this_time_%1000 < 50){printf("\r odom_this_time_-%f mm;speed-%d mm/s.\n", odom_this_time_, param_->speed);}
    // log_debug("speed %d mm/s", param_->speed);
  }

  void Robot::Stop()
  {
    motion_stop_signal_ = true;
    motion_thread_.join();
    // get_motor_param_thread_.join();
    // force stop
    motion_.Stop();
    log_info("stop robot.");
  }

  void Robot::StartUVC()
  {
    visual_buffer_info_t visual_buf;
    // create uvc thread, use lambda
    printf("\rstart uvc camera.\n");

#if SAVE_UVC_PIC
    /* create uvc pic folder */
    // create folder
    uvc_folder_name = "uvc_" + utility::GetSystemTime();

    utility::CreateDirectory(uvc_folder_name.c_str());
#endif
    uvc_vision_thread_ = std::thread([&]()
                                     {
                                       FILE *uvc_file;
                                       std::string uvc_file_name;
                                       double odm_last_time = 0;
                                       double odm_this_time = 0;
                                       double odm_delta;
                                       // double odm_total = 0;
                                       while (stop_uvc_signal_ == false)
                                       {
                                         uvc_cam_.Stream();
                                         

                                         /* udp send */
                                         photo_info_t *photo_info = (photo_info_t *)visual_buf.data;

                                         photo_info->position = 4;
                                         photo_info->group = 0;
                                         photo_info->sequence = 0;
                                         photo_info->time_stamp = uvc_cam_.frame_cnt_;
                                         photo_info->width = uvc_cam_.img_width_;
                                         photo_info->height = uvc_cam_.img_height_;
                                         photo_info->bytes_per_pixel = 3;
                                         photo_info->reserved = 0;

                                         memcpy((char *)visual_buf.data + sizeof(photo_info_t),
                                                (char *)uvc_cam_.usr_buf[uvc_cam_.q_buf_.index].start,
                                                uvc_cam_.usr_buf[uvc_cam_.q_buf_.index].length);

#if SAVE_UVC_PIC
                                         //  save uvc jpg
                                         odm_this_time = param_->odometer;

                                         odm_delta = abs(odm_this_time - odm_last_time);

                                         // only save uvc pic every 500mm and move up direction.
                                         if (odm_delta > 500 && motion_.motion_cmd_->command == kMoveUpCmd)
                                         {
                                           log_debug("odm delta:%f", odm_delta);
                                           uvc_file_name = uvc_folder_name + "/uvc_odm_" +
                                                           std::to_string(static_cast<int>(odm_this_time)) + "mm" +
                                                           ".jpg";
                                           uvc_file = fopen(uvc_file_name.c_str(), "wb");
                                           fwrite((char *)uvc_cam_.usr_buf[uvc_cam_.q_buf_.index].start,
                                                  uvc_cam_.usr_buf[uvc_cam_.q_buf_.index].length, 1, uvc_file);
                                           fclose(uvc_file);
                                           odm_last_time = odm_this_time;
                                         }
#endif
                                         // send uvc frame to host
                                         visual_buf.data_length =
                                             uvc_cam_.usr_buf[uvc_cam_.q_buf_.index].length + sizeof(photo_info_t);
                                         visual_buf.position = 4;
                                         visual_buf.time_stamp = uvc_cam_.frame_cnt_;

                                         // udp send
                                         send_visual(visual_buf, udp_client_);
                                         uvc_cam_.QBuf();

                                         // referesh time, 100ms
                                         usleep(100000);
                                       }
                                     });
  }

  void Robot::StopUVC()
  {
    stop_uvc_signal_ = true;
    uvc_vision_thread_.join();

    log_info("\rstop uvc camera.");
    // stop uvc
    uvc_cam_.DeInit();
  }

  // shut down robot
  void Robot::ShutDown()
  {
    // shutdown system
    log_info("shutdown system");

    // modbus_thread_.join();
  }

  double Robot::GetOdometer() const { return motion_.odometer_; }

} // namespace ccr_quick_detection

