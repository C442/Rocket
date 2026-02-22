
#include "QuaternionKalman.h"
#include <math.h>

QuaternionKalman::QuaternionKalman() {
    q = {1.0f, 0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            P[i][j] = (i == j) ? 1.0f : 0.0f;
}

void QuaternionKalman::update(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    float half_dt = 0.5f * dt;
    Quaternion dq = {
        1.0f,
        gx * half_dt,
        gy * half_dt,
        gz * half_dt
    };
    Quaternion q_new = {
        q.w - q.x*dq.x - q.y*dq.y - q.z*dq.z,
        q.w*dq.x + q.x*dq.w + q.y*dq.z - q.z*dq.y,
        q.w*dq.y - q.x*dq.z + q.y*dq.w + q.z*dq.x,
        q.w*dq.z + q.x*dq.y - q.y*dq.x + q.z*dq.w
    };
    q = q_new;
    normalizeQuaternion();

    Quaternion q_acc;
    accelToQuaternion(ax, ay, az, q_acc);
    q = slerp(q, q_acc, 1.0f - R);
    normalizeQuaternion();
}

void QuaternionKalman::normalizeQuaternion() {
    float norm = sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    q.w /= norm;
    q.x /= norm;
    q.y /= norm;
    q.z /= norm;
}

void QuaternionKalman::accelToQuaternion(float ax, float ay, float az, Quaternion& q_acc) {
    float norm = sqrt(ax*ax + ay*ay + az*az);
    ax /= norm; ay /= norm; az /= norm;
    float pitch = atan2(-ax, sqrt(ay*ay + az*az));
    float roll = atan2(ay, az);

    float cy = cos(0);
    float sy = sin(0);
    float cp = cos(pitch/2);
    float sp = sin(pitch/2);
    float cr = cos(roll/2);
    float sr = sin(roll/2);

    q_acc.w = cr * cp * cy + sr * sp * sy;
    q_acc.x = sr * cp * cy - cr * sp * sy;
    q_acc.y = cr * sp * cy + sr * cp * sy;
    q_acc.z = cr * cp * sy - sr * sp * cy;
}

Quaternion QuaternionKalman::slerp(const Quaternion& q1, const Quaternion& q2, float t) {
    float dot = q1.w*q2.w + q1.x*q2.x + q1.y*q2.y + q1.z*q2.z;
    Quaternion q2c = q2;
    if (dot < 0.0f) {
        dot = -dot;
        q2c.w = -q2c.w; q2c.x = -q2c.x; q2c.y = -q2c.y; q2c.z = -q2c.z;
    }

    if (dot > 0.995f) {
        return {
            q1.w + t*(q2c.w - q1.w),
            q1.x + t*(q2c.x - q1.x),
            q1.y + t*(q2c.y - q1.y),
            q1.z + t*(q2c.z - q1.z)
        };
    }

    float theta_0 = acos(dot);
    float theta = theta_0 * t;
    float sin_theta = sin(theta);
    float sin_theta_0 = sin(theta_0);

    float s0 = cos(theta) - dot * sin_theta / sin_theta_0;
    float s1 = sin_theta / sin_theta_0;

    return {
        s0*q1.w + s1*q2c.w,
        s0*q1.x + s1*q2c.x,
        s0*q1.y + s1*q2c.y,
        s0*q1.z + s1*q2c.z
    };
}

Quaternion QuaternionKalman::getQuaternion() const {
    return q;
}

EulerAngles QuaternionKalman::getEulerAngles() const {
    EulerAngles eul;
    float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    eul.roll = atan2(sinr_cosp, cosr_cosp);

    float sinp = 2.0f * (q.w * q.y - q.z * q.x);
    eul.pitch = fabs(sinp) >= 1 ? copysign(M_PI / 2, sinp) : asin(sinp);

    float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    eul.yaw = atan2(siny_cosp, cosy_cosp);

    return eul;
}
