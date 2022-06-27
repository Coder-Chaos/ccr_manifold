#pragma once
#include <ctime>
#include <string>
#include "ccr_common.hpp"
#include "freemodbus_tcp.h"
#include "motion.hpp"
#include "net.hpp"
#include "shared_mem.h"
#include "udp_send.hpp"
#include "uvc.hpp"
#include "tfminip.hpp"
#define DEBUG_ON 0
// modbus holding registor
extern USHORT usRegHoldingBuf[REG_HOLDING_NREGS];
namespace ccr_quick_detection
{

  struct RobotParam
  {
    // odometer
    int odometer;

    // ps core and pl core temperature
    unsigned short ps_temp;
    unsigned short pl_temp;

    // speed, mm/s
    short speed;

    // image saving state
    unsigned short img_saving_state;

    // heart beat
    // std::time_t sys_time_;
    // std::tm *tm_ = nullptr;

    // // uvc pic folder name
    // std::string uvc_folder_name_;
  };

  //Range
  struct Range
  {
    //id,type...
    unsigned short radiation_type;
    float field_of_view;
    float min_range;
    float max_range;
    float range;
    std::string frame_id;
    //std::time_t stamp;
    // std::tm *tm_ = nullptr;

    // // uvc pic folder name
    // std::string uvc_folder_name_;
  };

  class Robot
  {
  private:
    /* data */

    double odom_last_time_ = 0;
    double odom_this_time_;
    // control period, ms
    static const int kControlPeriod = 100;

    benewake::TFmini *tfmini_obj;
    Range *Range_ = (Range *)malloc(sizeof(Range));
    static const USHORT ULTRASOUND = 0;
    static const USHORT INFRARED = 1;

    // can device
    can can0{(char *)"can0"}, can1{(char *)"can1"};

    /* vision */
    // uvc camera
    uvc uvc_cam_{640, 480, "/dev/video0"};

    // udp client
    //common::Net udp_client_{common::protocal_type::UDP, common::type::CLIENT,
    //                        "192.168.1.111", 7000};
    common::Net udp_client_{common::protocal_type::UDP, common::type::CLIENT,
                            "10.60.2.101", 7000};                        

    /* motion */
    //   tmotor pointer
    std::vector<TMotor> tmotors = {
        {&can0, 1, (TMotorParam *)&usRegHoldingBuf[150]},
        {&can0, 2, (TMotorParam *)&usRegHoldingBuf[200]},
        {&can0, 3, (TMotorParam *)&usRegHoldingBuf[250]},
        {&can0, 4, (TMotorParam *)&usRegHoldingBuf[300]},
        {&can0, 5, (TMotorParam *)&usRegHoldingBuf[350]},
        {&can0, 6, (TMotorParam *)&usRegHoldingBuf[400]},
        {&can0, 7, (TMotorParam *)&usRegHoldingBuf[450]},
        {&can0, 8, (TMotorParam *)&usRegHoldingBuf[500]}
        };

    // motion
    Motion motion_ = {(MotionCmd *)&usRegHoldingBuf[0],
                      (MotionState *)&usRegHoldingBuf[100], &tmotors};

    // shared memory for inter-process comunication
    char *shared_mem_ = nullptr;

    // modbus thread
    std::thread modbus_thread_;

    // tfmini thread
    std::thread tfmini_thread_;

    // motion thread
    std::thread motion_thread_;

    // uvc vision thread
    std::thread uvc_vision_thread_;

    //  guard thread
    std::thread guard_thread_;

    // save uvc thread
    std::thread save_uvc_pic_thread_;

    // robot parameter
    RobotParam *param_ = nullptr;

    // message to vision process
    VisionMessage *vision_msg_ = nullptr;

    // heartbeat flag
    std::atomic_bool heart_beat_flag_;

    // control signal
    bool stop_uvc_signal_ = false;

    bool disconnect_host_signal_ = false;

    bool motion_stop_signal_ = false;

    bool stop_tfmini_signal_ =false;

  public:
    Robot()
    {
      // get shared memory
      shared_mem_ = (char *)get_shared_memory(
          kShareMemName.c_str(), kSemaphoreName.c_str(), kSharedMemLength);

      /* perform some init operation */
      param_ = (RobotParam *)&usRegHoldingBuf[50];

      int ret = 0;
      // init uvc camera
      ret = uvc_cam_.Init();
      if (!ret)
      {
        log_info("uvc camera init success.");
      }

      // point to shared memory
      vision_msg_ = (VisionMessage *)shared_mem_;
    }
    ~Robot() {}

    // robot safety guard
    void Guard();

    // connect host
    void ConnectHost();

    // disconnect host
    void DisConnectHost();

    //tfmini range
    void TFminiRecv();

    void TFminiStop();

    void StartVisionDetection() { vision_msg_->vision_system_control = 1; }
    void StopVisionDetection() { vision_msg_->vision_system_control = 0; }

    // start uvc
    void StartUVC();

    void StopUVC();

    // robot run
    void Run();

    // robot stop
    void Stop();

    // shut down robot
    void ShutDown();

  private:
    // get robot parameter
    void GetParam();

    // get odometer
    double GetOdometer() const;

    // get robot speed
    unsigned int GetSpeed();
  };
} // namespace ccr_quick_detection
