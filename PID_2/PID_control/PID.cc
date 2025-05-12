#include "PID.h"

PID::PID(float kp, float ki, float kd, float target, unsigned long time) : kp_(kp) , ki_(ki), kd_(kp), setpoint_(target), time_step_(time){}

float PID::compute(float pos, unsigned long time){
  time_step_ = time - time_step_;
  error_ = setpoint_ - pos;
  integral_error_ += error_ + time_step_;
  derivative_error_ = (error_ - error_last_) / time_step_;
  error_last_ = error_;
  output_ = kp_*error_ + ki_ * integral_error_ + kp_ * derivative_error_;
  return output_;

}

