#include "tmotor.hpp"
#include <cmath>
namespace ccr_quick_detection {
/* tmotor */
ssize_t TMotor::CANSend(float p_des, float v_des, float kp, float kd,
                        float t_ff) {
  can_frame canFrameTMotor;

  // limit data to be within bounds
  p_des = fminf(fmaxf(P_MIN, p_des), P_MAX);
  v_des = fminf(fmaxf(V_MIN, v_des), V_MAX);
  kp = fminf(fmaxf(KP_MIN, kp), KP_MAX);
  kd = fminf(fmaxf(KD_MIN, kd), KD_MAX);
  t_ff = fminf(fmaxf(T_MIN, t_ff), T_MAX);
  p_des = fminf(fmaxf(P_MIN, p_des), P_MAX);

  // convert floats to unsigned ints
  int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
  int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
  int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
  int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
  int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);

  // canFrameTMotor.can_id = slave_id;
  canFrameTMotor.can_id = id_;
  canFrameTMotor.can_dlc = 8;

  canFrameTMotor.data[0] = p_int >> 8;
  canFrameTMotor.data[1] = p_int & 0xFF;
  canFrameTMotor.data[2] = v_int >> 4;
  canFrameTMotor.data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
  canFrameTMotor.data[4] = kp_int & 0xFF;
  canFrameTMotor.data[5] = kd_int >> 4;
  canFrameTMotor.data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
  canFrameTMotor.data[7] = t_int & 0xFF;

  return can_dev_->send((can_frame *)&canFrameTMotor);
}

ssize_t TMotor::EnterMotorMode() {
  can_frame canFrameTMotor;

  canFrameTMotor.can_id = id_;
  canFrameTMotor.can_dlc = 8;

  canFrameTMotor.data[0] = 0xFF;
  canFrameTMotor.data[1] = 0xFF;
  canFrameTMotor.data[2] = 0xFF;
  canFrameTMotor.data[3] = 0xFF;
  canFrameTMotor.data[4] = 0xFF;
  canFrameTMotor.data[5] = 0xFF;
  canFrameTMotor.data[6] = 0xFF;
  canFrameTMotor.data[7] = 0xFC;

  return can_dev_->send((can_frame *)&canFrameTMotor);
}

ssize_t TMotor::ExitMotorMode() {
  can_frame canFrameTMotor;

  canFrameTMotor.can_id = id_;
  canFrameTMotor.can_dlc = 8;

  canFrameTMotor.data[0] = 0xFF;
  canFrameTMotor.data[1] = 0xFF;
  canFrameTMotor.data[2] = 0xFF;
  canFrameTMotor.data[3] = 0xFF;
  canFrameTMotor.data[4] = 0xFF;
  canFrameTMotor.data[5] = 0xFF;
  canFrameTMotor.data[6] = 0xFF;
  canFrameTMotor.data[7] = 0xFD;

  return can_dev_->send((can_frame *)&canFrameTMotor);
}

ssize_t TMotor::SetZeroPostion() {
  can_frame canFrameTMotor;

  canFrameTMotor.can_id = id_;
  canFrameTMotor.can_dlc = 8;

  canFrameTMotor.data[0] = 0xFF;
  canFrameTMotor.data[1] = 0xFF;
  canFrameTMotor.data[2] = 0xFF;
  canFrameTMotor.data[3] = 0xFF;
  canFrameTMotor.data[4] = 0xFF;
  canFrameTMotor.data[5] = 0xFF;
  canFrameTMotor.data[6] = 0xFF;
  canFrameTMotor.data[7] = 0xFE;

  return can_dev_->send((can_frame *)&canFrameTMotor);
}

ssize_t TMotor::StopMotor() {
  can_frame canFrameTMotor;

  canFrameTMotor.can_id = id_;
  canFrameTMotor.can_dlc = 8;

  canFrameTMotor.data[0] = 0x7F;
  canFrameTMotor.data[1] = 0xFF;
  canFrameTMotor.data[2] = 0x7F;
  canFrameTMotor.data[3] = 0xF0;
  canFrameTMotor.data[4] = 0x51;
  canFrameTMotor.data[5] = 0x00;
  canFrameTMotor.data[6] = 0x07;
  canFrameTMotor.data[7] = 0xFF;

  return can_dev_->send((can_frame *)&canFrameTMotor);
}

bool TMotor::Init() {
  log_info("TMotor[%d] Init::START....\n", id_);
  // Motor Inited
  // if (*fInit == true) return true;

  // Enter CAN Motor Mode
  EnterMotorMode();
  utility::delay_us(TMOTOR_DELAY * 3);
  log_info("TMotor[%d] EnterMotorMode: Pos[%lf]-Vel[%lf]-Tor[%lf].\n", id_,
           param_->tmtGetPos, param_->tmtGetVel, param_->tmtGetTor);

  // Stop Motor
  StopMotor();
  utility::delay_us(TMOTOR_DELAY * 3);
  log_info("TMotor[%d] StopMotor: Pos[%lf]-Vel[%lf]-Tor[%lf].\n", id_,
           param_->tmtGetPos, param_->tmtGetVel, param_->tmtGetTor);

  // Set Motor Zero Position
  SetZeroPostion();
  utility::delay_us(TMOTOR_DELAY * 3);
  log_info("TMotor[%d] SetZeroPostion: Pos[%lf]-Vel[%lf]-Tor[%lf].\n", id_,
           param_->tmtGetPos, param_->tmtGetVel, param_->tmtGetTor);

  // clear parameter
  param_->turns = 0;

  log_info("TMotor[%d] Init::OVER....\n", id_);

  return true;
}

void TMotor::GetParam(const can_frame &recv_frame) {
  // check motor id
  if (recv_frame.data[0] == id_) {
    // log_debug("motor id:%d", recv_frame.data[0]);

    /// unpack ints from can buffer ///
    param_->iMotorID = recv_frame.data[0];
    int p_int = (recv_frame.data[1] << 8) | recv_frame.data[2]; //电机位置数据
    int v_int =
        (recv_frame.data[3] << 4) | (recv_frame.data[4] >> 4); //电机速度数据
    int t_int =
        ((recv_frame.data[4] & 0xF) << 8) | recv_frame.data[5]; //电机扭矩数据
    /// convert ints to floats ///
    float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
    float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
    float t = uint_to_float(t_int, T_MIN, T_MAX, 12);

    param_->tmtGetVel = v;
    param_->tmtGetTor = t;
#if 1
    // calc turns
    param_->turns +=
        fabs((fabs(param_->tmtGetPos) - fabs(p)) / 6.25) * param_->dir;

#else
    // overflow
    if (param_->tmtGetPos * p < 0) {
      param_->turns +=
          (fabs(fabs(param_->tmtGetPos) - 12.5) + fabs(fabs(p) - 12.5)) / 6.25 *
          param_->dir;
    } else {
      // calc turns
      param_->turns +=
          fabs((fabs(param_->tmtGetPos) - fabs(p)) / 6.25) * param_->dir;
    }
#endif
    // param_->turns += fabs((fabs(param_->tmtGetPos) - fabs(p)) ) *
    // param_->dir;
    // log_info("motor[%d] tmtGetPos:%f",id_,fabs(param_->tmtGetPos));
    // log_info("motor[%d] p:%f",id_,fabs(p));
    // log_info("motor[%d] turns:%f", id_, param_->turns);

    // update position
    param_->tmtGetPos = p;
  }
}

void TMotor::ClearParam() {
  memset((TMotorParam *)param_, 0, sizeof(TMotorParam));
}

void TMotor::SetVel(float vel, float kd) {
  CANSend(0, vel, 0, kd, 0);
  utility::delay_us(TMOTOR_DELAY * 3);
}

void TMotor::SetVel(int16_t dir, float vel, float kd) {
  CANSend(0, dir * vel, 0, kd, 0);
  utility::delay_us(TMOTOR_DELAY * 3);
}

void TMotor::SetVelToq(float vel, float kd, float toq) {
  CANSend(0, vel, 0, kd, toq);
  utility::delay_us(TMOTOR_DELAY * 3);
}

void TMotor::SetPos(float pos, float kp, float kd) {
  CANSend(pos, 0, kp, kd, 0);
  utility::delay_us(TMOTOR_DELAY * 3);
}

} // namespace ccr_quick_detection
