#include <math.h>

#ifndef PID_h
#define PID_h

// I saw some dude on yt who made a python class on it with a turtle-impromptu physics "engine". it was cute and cool.
// https://www.youtube.com/watch?v=ZMI_kpNUgJM
//btw, I compared to yours (obviously) and noticed: we don't have the same coding style ( internal_variable_, initialiser list)

class PID{

  private:
  float kp_ = 0.0f;
  float ki_ = 0.0f;
  float kd_ = 0.0f;
  float setpoint_ = 0.0f;
  float error_ = 0.0f;
  float integral_error_ = 0.0f;
  float error_last_ = 0.0f;
  float derivative_error_ = 0.0f;
  float output_ = 0.0f;
  float time_step_ = 0.0f;

  public:
    PID(float kp, float ki, float kd, float target, unsigned long time);
    float compute(float pos, unsigned long time);
};

#endif PID_h

