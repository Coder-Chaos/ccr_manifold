#include "motion.hpp"
#include "freemodbus_tcp.h"
#include "log.h"
#include "tmotor.hpp"
#include <cmath>
#include <thread>
#include <chrono>
#include <iostream>

using namespace std;
#define MIMIC_MOTION 0


namespace ccr_quick_detection
{

  void Motion::ManualMotonFSM()
  {
    //if(motion_cmd_->command == 0){motion_cmd_->command = 1;}
    // select mannual motion command
    switch (motion_cmd_->command)
    {
    case kMoveUpCmd:
#if MIMIC_MOTION
      log_debug("move up.");
      log_debug("gearbox:%f", GearBox());
#else

#if LEAK_ROB
      MoveUpLeak();
#else
      MoveUp();

#endif
#endif
      motion_state_->current_cmd = motion_cmd_->command;
      motion_state_->current_state = kMoveUpState;
      //motion_cmd_->command = 2;
      break;

    case kMoveDownCmd:
#if MIMIC_MOTION
      log_debug("move down.");
      log_debug("gearbox:%f", GearBox());
#else
#if LEAK_ROB
      MoveDownLeak();
#else
      MoveDown();
#endif
#endif
      motion_state_->current_cmd = motion_cmd_->command;
      motion_state_->current_state = kMoveDownState;
      //motion_cmd_->command = 1;
      break;

    case kStopCmd:
#if MIMIC_MOTION
      log_debug("stop!");
      log_debug("gearbox:%f", GearBox());
#else
      Stop();
#endif
      motion_state_->current_cmd = motion_cmd_->command;
      motion_state_->current_state = kStopState;
      break;

      // defualt stop
    default:
#if MIMIC_MOTION
      log_debug("stop state.");
      log_debug("gearbox:%f", GearBox());
#else
      Stop();
#endif
      motion_state_->current_cmd = motion_cmd_->command;
      motion_state_->current_state = kStopState;
      break;
    }
  }

  int Motion::Return()
  {
    int ret;

    // odometer > 100mm, move back
    if (odometer_ > 100)
    {
      log_debug("odom %f mm", odometer_);
#if LEAK_ROB
      MoveDownLeak();
#else
      MoveDown();
#endif
      ret = -1;
    }
    else
    {
      log_debug("stop!");
      Stop();
      ret = 0;
    }
    return ret;
  }
  static int stop_cnt = 0;
  /* motion FSM */
  void Motion::SystemMotionFSM(const std::atomic_bool &heartbeat_flag)
  {
    // check heartbeat
    if (heartbeat_flag == false)
    {
      if (stop_cnt == 0)
      {
        stop_cnt++;
        Stop();
        sleep(1);
      }
      motion_cmd_->motion_mode = kReturnMode;
    }

    // check quick stop command
    if (motion_cmd_->quick_stop != kQuickStopOn)
    {
      //motion_cmd_->motion_mode = 4;
      //   select motion mode
      switch (motion_cmd_->motion_mode)
      {
        //     mannual mode
      case kManualMode:
        // update state
        //log_info("ManualMode：odometer_ = %f mm", odometer_);
        motion_state_->motion_mode_state = MotionModeState::ManualMode;
        ManualMotonFSM();
        break;

        // auto mode
      case kAutoMode:
        motion_state_->motion_mode_state = MotionModeState::AutoMode;
        log_debug("auto mode.");
        break;

        // return mode
      case kReturnMode:
        int ret;
        log_debug("return mode.");
        if (motion_state_->motion_mode_state != MotionModeState::ReturnMode)
        {
          do
          {
            // check if quick stop
            if (motion_cmd_->quick_stop == kQuickStopOn)
            {
              //   quick stop
              Stop();
              break;
            }
            ret = Return();
            // usleep(200000);
            std::this_thread::sleep_for(std::chrono::microseconds(200));
          } while (ret != 0);
          motion_state_->motion_mode_state = MotionModeState::ReturnMode;
        }
        // change to mannual mode
        motion_cmd_->motion_mode = kManualMode;
        break;

        // set zero
      case kSetZero:
        if (motion_state_->motion_mode_state != MotionModeState::SetZero)
        {
          // clear motor parameters
          for (auto tmotor : *tmotors_)
          {
            odometer_ = 0;
            tmotor.ClearParam();
          }
          motion_state_->motion_mode_state = MotionModeState::SetZero;
        }
        // change to mannual mode
        log_debug("finish set zero.");
        motion_cmd_->motion_mode = kManualMode;
        break;

        // test
      case kTestMode:
        motion_state_->motion_mode_state = MotionModeState::TestMode;
        log_info("odometer_ = %f mm.", odometer_);
        if (test_dir_ == 0)
        {
          // move up 1000mm
          if (odometer_ < 2000)
          {
#if LEAK_ROB
            MoveUpLeak();
#else
            MoveUp();

#endif
          }
          else
          {
            // change to move down
            test_dir_ = -1;
            // stop motor
            Stop();
            sleep(1);
          }
        }
        else if (test_dir_ == -1)
        {
          // move up 1000mm
          if (odometer_ > 600)
          {
#if LEAK_ROB
            MoveDownLeak();
#else
            MoveDown();
#endif
          }
          else
          {
            // change to move up
            test_dir_ = 0;
            // stop motor
            Stop();
            sleep(1);
          }
        }
        break;

        // default stop
      default:
        Stop();
        break;
      }
    }
    else
    {
      //   quick stop
      Stop();
      motion_cmd_->motion_mode = kManualMode;
      motion_cmd_->quick_stop = kQuickStopOff;
    }
  }

  // robot move up
  void Motion::MoveUp()
  {
    // motors move up
    for (auto tmotor : *tmotors_)
    {
      tmotor.param_->dir = 1;
      tmotor.param_->tmtSetVel = GearBox();
      tmotor.param_->tmtSetKD = Kd;
      tmotor.SetVel(tmotor.param_->dir, tmotor.param_->tmtSetVel,
                    tmotor.param_->tmtSetKD);
    }
  }

  void Motion::MoveUpLeak()
  {
    // motor 1、2
    for (int i = 0; i < 2; i++)
    {
      tmotors_->at(i).param_->dir = -1;
      tmotors_->at(i).param_->tmtSetVel = GearBox();
      tmotors_->at(i).param_->tmtSetKD = Kd;
      tmotors_->at(i).SetVel(tmotors_->at(i).param_->dir, tmotors_->at(i).param_->tmtSetVel,
                                 tmotors_->at(i).param_->tmtSetKD);
    }

    // motor 3 4
    for (int i = 2; i < 4; i++)
    {
      tmotors_->at(i).param_->dir = 1;
      tmotors_->at(i).param_->tmtSetVel = GearBox();
      tmotors_->at(i).param_->tmtSetKD = Kd;
      tmotors_->at(i).SetVel(tmotors_->at(i).param_->dir, tmotors_->at(i).param_->tmtSetVel,
                                     tmotors_->at(i).param_->tmtSetKD);
    }
    // motor 5 6
    for (int i = 4; i < 6; i++)
    {
      tmotors_->at(i).param_->dir = -1;
      tmotors_->at(i).param_->tmtSetVel = GearBox();
      tmotors_->at(i).param_->tmtSetKD = Kd;
      tmotors_->at(i).SetVel(tmotors_->at(i).param_->dir, tmotors_->at(i).param_->tmtSetVel,
                                 tmotors_->at(i).param_->tmtSetKD);
    }
    // motor 7 8
    for (int i = 6; i < 8; i++)
    {
      tmotors_->at(i).param_->dir = 1;
      tmotors_->at(i).param_->tmtSetVel = GearBox();
      tmotors_->at(i).param_->tmtSetKD = Kd;
      tmotors_->at(i).SetVel(tmotors_->at(i).param_->dir, tmotors_->at(i).param_->tmtSetVel,
                                 tmotors_->at(i).param_->tmtSetKD);
    }
  }

  // robot move down
  void Motion::MoveDown()
  {
    // motors move down
    for (auto tmotor : *tmotors_)
    {
      // set direction
      tmotor.param_->dir = -1;
      tmotor.param_->tmtSetVel = GearBox();
      tmotor.param_->tmtSetKD = Kd;
      tmotor.SetVel(tmotor.param_->dir, tmotor.param_->tmtSetVel,
                    tmotor.param_->tmtSetKD);
    }
  }

  void Motion::MoveDownLeak()
  {
    // motor 1、2
    for (int i = 0; i < 2; i++)
    {
      tmotors_->at(i).param_->dir = 1;
      tmotors_->at(i).param_->tmtSetVel = GearBox();
      tmotors_->at(i).param_->tmtSetKD = Kd;
      tmotors_->at(i).SetVel(tmotors_->at(i).param_->dir, tmotors_->at(i).param_->tmtSetVel,
                                 tmotors_->at(i).param_->tmtSetKD);
    }

    // motor 3 4
    for (int i = 2; i < 4; i++)
    {
      tmotors_->at(i).param_->dir = -1;
      tmotors_->at(i).param_->tmtSetVel = GearBox();
      tmotors_->at(i).param_->tmtSetKD = Kd;
      tmotors_->at(i).SetVel(tmotors_->at(i).param_->dir, tmotors_->at(i).param_->tmtSetVel,
                                     tmotors_->at(i).param_->tmtSetKD);
    }

    for (int i = 4; i < 6; i++)
    {
      tmotors_->at(i).param_->dir = 1;
      tmotors_->at(i).param_->tmtSetVel = GearBox();
      tmotors_->at(i).param_->tmtSetKD = Kd;
      tmotors_->at(i).SetVel(tmotors_->at(i).param_->dir, tmotors_->at(i).param_->tmtSetVel,
                                 tmotors_->at(i).param_->tmtSetKD);
    }
    for (int i = 6; i < 8; i++)
    {
      tmotors_->at(i).param_->dir = -1;
      tmotors_->at(i).param_->tmtSetVel = GearBox();
      tmotors_->at(i).param_->tmtSetKD = Kd;
      tmotors_->at(i).SetVel(tmotors_->at(i).param_->dir, tmotors_->at(i).param_->tmtSetVel,
                                 tmotors_->at(i).param_->tmtSetKD);
    }
  }

  // robot move down
  void Motion::Stop1()
  {
    bool stop_flag = false;
    // check Vel_min=0.0125
    for (auto tmotor : *tmotors_)
    {
      if (fabs(tmotor.param_->tmtGetVel) < 0.05f)
      {
        stop_flag = true;
      }
    }

    if (stop_flag == false)
    {
      // motors stop
      for (auto tmotor : *tmotors_)
      {
        tmotor.param_->dir = 0;
        tmotor.param_->tmtSetVel = 0;
        tmotor.param_->tmtSetKD = Kd;
        tmotor.SetVel(tmotor.param_->dir, tmotor.param_->tmtSetVel,
                      tmotor.param_->tmtSetKD);
        
      }
    }
  }

  // robot move down
  void Motion::Stop()
  {
    bool stop_flag = false;
    // check Vel_min=0.0125
    
    for (auto tmotor : *tmotors_)
    {
      if (fabs(tmotor.param_->tmtGetVel) < 0.05f)
      {
        //tmotor.SetVel(0, tmotor.param_->tmtSetKD);
        //usleep(3 * 1000);
        tmotor.ExitMotorMode();
        usleep(3 * 1000);       
        tmotor.EnterMotorMode();
        stop_flag = true;
      }
    }
    if (stop_flag == false)
    {
      
      for (auto tmotor : *tmotors_)
      {
        //tmotor.param_->tmtSetVel = -0.1f * (odom_this_time_ - odom_last_time_);
        //tmotor.param_->tmtSetVel = -0.1f * param_->speed * 0.1f / 59;
        tmotor.param_->tmtSetVel = -3.0f * tmotor.param_->tmtGetVel * 0.1f;
        tmotor.param_->tmtSetKD = Kd;
        tmotor.SetVel(tmotor.param_->tmtSetVel, tmotor.param_->tmtSetKD);
      }
      
    }
    
  }
#define USE_VECTOR 0

#if USE_VECTOR
  static std::vector<double> odom_turns;
  static std::vector<double>::iterator max_turn;
#endif

  void Motion::GetTMotorsParam()
  {
    // get received can frame
    can_frame recv_frame;
    double odm_sum = 0;
    double max_turn = 0;
    //   poll to get frame, change to epoll?
    for (;;)
    {
      // all motors share one CAN bus
      tmotors_->begin()->can_dev_->receive(&recv_frame);

      //   assign parameters
      for (auto tmotor : *tmotors_)
      {
        tmotor.GetParam(recv_frame);
      }

#if USE_VECTOR
      // calc odometer
      for (auto tmotor : *tmotors_)
      {
        // odm_sum += tmotor.param_->turns;
        odom_turns.emplace_back(std::move(tmotor.param_->turns));
      }

#if 1
      // sort odom vector
      std::sort(odom_turns.begin(), odom_turns.end());

      // clear the max element
      odom_turns.pop_back();

      odometer_ =
          (double)(std::accumulate(odom_turns.begin(), odom_turns.end(), 0) /
                   odom_turns.size() * 370.52);
#else

      max_turn = std::max_element(odom_turns.begin(), odom_turns.end());

      odometer_ =
          (double)((std::accumulate(odom_turns.begin(), odom_turns.end(), 0) -
                    *max_turn) /
                   (odom_turns.size() - 1) * 370.52);
#endif
      // clear vector
      odom_turns.clear();

#else

#if LEAK_ROB
      for (int i = 0; i < 2; i++)
      {
        odm_sum -= tmotors_->at(i).param_->turns;
        odm_sum -= tmotors_->at(i+4).param_->turns;
      }

      for (int i = 2; i < 4; i++)
      {
        odm_sum += tmotors_->at(i).param_->turns;
        odm_sum += tmotors_->at(i+4).param_->turns;
      }

#else
      // calc odometer
      for (auto tmotor : *tmotors_)
      {
        odm_sum += tmotor.param_->turns;
      }

#endif
      log_debug("turn:%f", (double)(odm_sum / tmotors_->size()));

      odometer_ = (double)(odm_sum / (tmotors_->size()) * 370.52); // mm

      odm_sum = 0; // clear
      max_turn = 0;
#endif
      log_debug("odometer:%f mm", odometer_);
      // refresh 10ms
      usleep(10000);
    }
  }

  // select speed
  float Motion::GearBox()
  {
    switch (motion_cmd_->speed_gear)
    {
    case kNormal:
#if MIMIC_MOTION
      log_info("normal speed");
#else
      return kNormalSpeed;
#endif
      break;

    case kHigh:
#if MIMIC_MOTION
      log_info("high speed");
#else
      return 2 * kNormalSpeed;
#endif
      break;

    case kLow:
#if MIMIC_MOTION
      log_info("low speed");
#else
      return 0.3 * kNormalSpeed;
#endif
      break;

      // defualt normal speed
    default:
#if MIMIC_MOTION
      log_info("normal speed");
#else
      return kNormalSpeed;
#endif
      break;
    }

#if MIMIC_MOTION
    return 0;
#endif
  }

  bool IsHostAlive()
  {
    //check every 30s
  }
} // namespace ccr_quick_detection
