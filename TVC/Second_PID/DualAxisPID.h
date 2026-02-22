#ifndef DUAL_AXIS_PID_H
#define DUAL_AXIS_PID_H

struct PIDState {
    float integral = 0.0;
    float prevError = 0.0;
    float filteredDerivative = 0.0;
};

class DualAxisPID {
public:
    DualAxisPID(float outerKp, float outerKi, float outerKd,
                float innerKp, float innerKi, float innerKd, float N);

    float computeOuterLoop(float targetAngle, float currentAngle, PIDState& state, float dt);
    float computeInnerLoop(float targetRate, float measuredRate, PIDState& state, float dt);
    void DualAxisPID::setOuterGains(float kp, float ki, float kd);
    void DualAxisPID::setInnerGains(float kp, float ki, float kd);

private:
    float _outerKp, _outerKi, _outerKd;
    float _innerKp, _innerKi, _innerKd;
    float _N;
};

#endif // DUAL_AXIS_PID_H
