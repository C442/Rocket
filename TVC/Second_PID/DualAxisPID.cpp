#include "DualAxisPID.h"

DualAxisPID::DualAxisPID(float outerKp, float outerKi, float outerKd,
                         float innerKp, float innerKi, float innerKd, float N)
    : _outerKp(outerKp), _outerKi(outerKi), _outerKd(outerKd),
      _innerKp(innerKp), _innerKi(innerKi), _innerKd(innerKd), _N(N) {}

float DualAxisPID::computeOuterLoop(float targetAngle, float currentAngle, PIDState& state, float dt) {
    float error = targetAngle - currentAngle;
    state.integral += error * dt;
    float derivative = (error - state.prevError) / dt;
    state.prevError = error;
    return _outerKp * error + _outerKi * state.integral + _outerKd * derivative;
}

float DualAxisPID::computeInnerLoop(float targetRate, float measuredRate, PIDState& state, float dt) {
    float error = targetRate - measuredRate;
    state.integral += error * dt;
    if (state.integral > 50) state.integral = 50;
    else if (state.integral < -50) state.integral = -50;

    float rawD = (error - state.prevError) / dt;
    float alpha = _N * dt / (1 + _N * dt);
    state.filteredDerivative = alpha * rawD + (1 - alpha) * state.filteredDerivative;
    state.prevError = error;

    return _innerKp * error + _innerKi * state.integral + _innerKd * state.filteredDerivative;
}

void DualAxisPID::setOuterGains(float kp, float ki, float kd) {
  _outerKp = kp;
  _outerKi = ki;
  _outerKd = kd;
}
void DualAxisPID::setInnerGains(float kp, float ki, float kd) {
  _innerKp = kp;
  _innerKi = ki;
  _innerKd = kd;
}
