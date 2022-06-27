#pragma once
#include "ccr_common.hpp"
#include "common.hpp"
#include "freemodbus_tcp.h"
#include "tmotor.hpp"
#include <vector>
#include <atomic>

#define LEAK_ROB 1

namespace ccr_quick_detection {
  // total motor number
  static const int kTMotorNum = 4;

  // wheel perimeter 370mm
  static const USHORT kWheelPerimeter = 370;

  /* robot state */
  struct MotionState {
    //   motion mode
    MotionModeState motion_mode_state;

    // //   motion
    USHORT current_cmd;

    // //   FSM state
    USHORT current_state;
  };

  class Motion {
    friend class Robot;

  private:
    int test_dir_ = 0;
 
    // host command
    MotionCmd* motion_cmd_ = nullptr;

    //   motion state
    MotionState* motion_state_ = nullptr;

    //  tmotors
    std::vector<TMotor>* tmotors_ = nullptr;

    // moving odometer
    double odometer_;
    double odometer_0;
    /* motor parameters */
    // kd, 0-5
    static constexpr float Kd = 5;

    // kp, 0-500
    static constexpr float Kp = 5;

    // read motor parameters thread
    std::thread read_motors_param_thread_;

  public:
    Motion(MotionCmd* motion_cmd, MotionState* motion_state,
      std::vector<TMotor>* tmotors)
      : motion_cmd_(motion_cmd), motion_state_(motion_state),
      tmotors_(tmotors) {
      // init motor
      for (auto tmotor : *tmotors_) {
        tmotor.Init();
      }

      // read motor parameter thread
      read_motors_param_thread_ = std::thread([&]() { GetTMotorsParam(); });
      read_motors_param_thread_.detach();

      // set default param
      motion_cmd_->sample_interval = kDefaultSampleInterval;
      motion_cmd_->speed_gear = kLow;
    }

    Motion(/* args */) {}
    ~Motion() {
      // exit motor mode
      for (auto tmotor : *tmotors_) {
        tmotor.ExitMotorMode();
      }
    }

    // System motion FSM
    void SystemMotionFSM();
    void SystemMotionFSM(const std::atomic_bool &heartbeat_flag);

    // Manual motion FSM
    void ManualMotonFSM();

    // return motion
    int Return();

    // move up function
    void MoveUp();
    void MoveUpLeak();

    // move down function
    void MoveDown();
    void MoveDownLeak();

    // Stop function
    void Stop();
    void Stop1();

    // get motor parameter
    void GetTMotorsParam();

    //  speed gear
    float GearBox();

    // check heartbeat
    bool IsHostAlive();

  };

} // namespace ccr_quick_detection
