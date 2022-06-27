#pragma once
#include "can.hpp"
#include "common.hpp"
#include "log.h"
#include "utility.hpp"

namespace ccr_quick_detection {

#define AK80_9

// #ifdef AK80_6
// #undefine AK80_9
// #endif

// #ifdef AK80_9
// #undefine AK80_6
// #endif

#define P_MAX 12.5f
#define P_MIN -12.5f
#ifdef AK80_6
#define V_MAX 38.2f // Velocity: AK80-6 ±38.2f; AK80-9 ±25.64f
#define V_MIN -38.2f
#else
#ifdef AK80_9
#define V_MAX 25.64f // Velocity: AK80-6 ±38.2f; AK80-9 ±25.64f
#define V_MIN -25.64f
#endif
#endif
#define KP_MAX 500.0f
#define KP_MIN 0.0f
#define KD_MAX 5.0f
#define KD_MIN 0.0f

#ifdef AK80_6
#define T_MAX 12.0f // Torque: AK80-6 ±12.0f; AK80-9 ±18.0f
#define T_MIN -12.0f
#else
#ifdef AK80_9
#define T_MAX 18.0f // Torque: AK80-6 ±12.0f; AK80-9 ±18.0f
#define T_MIN -18.0f
#endif
#endif

#define PI 3.14159265359f
#define SQRT3 1.73205080757f

#define TMOTOR_DELAY 1000

/* utility functions */
inline float fmaxf(float x, float y) {
  /// Returns maximum of x, y ///
  return (((x) > (y)) ? (x) : (y));
}

inline float fminf(float x, float y) {
  /// Returns minimum of x, y ///
  return (((x) < (y)) ? (x) : (y));
}

inline float fmaxf3(float x, float y, float z) {
  /// Returns maximum of x, y, z ///
  return (x > y ? (x > z ? x : z) : (y > z ? y : z));
}

inline float fminf3(float x, float y, float z) {
  /// Returns minimum of x, y, z ///
  return (x < y ? (x < z ? x : z) : (y < z ? y : z));
}

inline void limit(float *x, float min, float max) {
  *x = fmaxf(fminf(*x, max), min);
}

inline int float_to_uint(float x, float x_min, float x_max, int bits) {
  /// Converts a float to an unsigned int, given range and number of bits ///
  float span = x_max - x_min;
  float offset = x_min;
  return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

inline float uint_to_float(int x_int, float x_min, float x_max, int bits) {
  /// converts unsigned int to float, given range and number of bits ///
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/* tmotor parameter */
struct TMotorParam {
  uint16_t iMotorID;

  /* set vaule */
  float tmtSetPos;
  float tmtSetVel;
  float tmtSetTor;
  float tmtSetKP;
  float tmtSetKD;

  /* actual value */
  float tmtGetPos;
  float tmtGetVel;
  float tmtGetTor;

  short dir;
  // number of turns, 6.25 rad/turn
  double turns;
};

class TMotor {
  friend class Motion;
  friend class Robot;

private:
  // can device pointer
  can *can_dev_ = nullptr;

  // set motor id
  uint16_t id_;

  // tmotor parametert
  TMotorParam *param_ = nullptr;

  // tmotor delay time ,nms
  static const int kDelayOneMs = 1;

public:
  TMotor(/* args */) {}
  // constructor with motor id
  TMotor(can *can_dev, uint16_t id, TMotorParam *motor_param)
      : can_dev_(can_dev), id_(id), param_(motor_param) {
    // param_->iMotorID = id_;
  }
  ~TMotor() {}

  // TMotor Function
  ssize_t CANSend(float p_des, float v_des, float kp, float kd, float t_ff);

  bool Init();
  void GetParam(const can_frame &recv_frame);
  ssize_t EnterMotorMode();
  ssize_t ExitMotorMode();
  ssize_t SetZeroPostion();
  ssize_t StopMotor();

  // clear tmotor parameter
  void ClearParam();
  // speed mode
  void SetVel(float vel, float kd);
  void SetVel(int16_t dir, float vel, float kd);
  void SetVelToq(float vel, float kd, float toq);
  void SetPos(float pos, float kp, float kd);
  void TmotorDelay(int n_ms) { utility::delay_ms(n_ms); }
};
} // namespace ccr_quick_detection
