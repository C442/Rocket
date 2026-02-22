
#ifndef QUATERNION_KALMAN_H
#define QUATERNION_KALMAN_H

struct Quaternion {
    float w, x, y, z;
};

struct EulerAngles {
    float roll, pitch, yaw;
};

class QuaternionKalman {
public:
    QuaternionKalman();
    void update(float gx, float gy, float gz, float ax, float ay, float az, float dt);
    Quaternion getQuaternion() const;
    EulerAngles getEulerAngles() const;

private:
    Quaternion q;
    float P[4][4];
    float Q = 0.001f;
    float R = 0.9f;

    void normalizeQuaternion();
    void accelToQuaternion(float ax, float ay, float az, Quaternion& q_acc);
    Quaternion slerp(const Quaternion& q1, const Quaternion& q2, float t);
};

#endif
