#include "HealthMonitor.h"
#include <Arduino.h> 

static inline bool validFloat(float x) {
    return !isnan(x) && isfinite(x);
}

void HealthMonitor::markOK(SensorHealth& h) {
    h.lastOk_us = micros();
    h.badStreak = 0;
}

void HealthMonitor::markBAD(SensorHealth& h, uint16_t limit, const char* who, const char* detail, float flightTime) {
    h.badStreak++;
    if (!h.failed && h.badStreak >= limit) {
        h.failed = true;
        String msg = String(who) + " FAIL: " + detail;
        sdCard.logEvent(msg.c_str(), flightTime);
    }
}

void HealthMonitor::tick(float flightTime) {
    uint32_t now = micros();

    // IMU timeout → inhibit TVC
    if (!imu.failed && (now - imu.lastOk_us) > 150000) {
        markBAD(imu, 0, "IMU", "timeout", flightTime);
        tvcInhibited = true;
        sdCard.logEvent("TVC INHIBITED (IMU fail)", flightTime);
    }

    // Baro timeout
    if (!baro.failed && (now - baro.lastOk_us) > 500000) {
        markBAD(baro, 0, "BARO", "timeout", flightTime);
    }

    // SD timeout
    if (!sd.failed && (now - sd.lastOk_us) > 500000) {
        markBAD(sd, 0, "SD", "no writes", flightTime);
    }

    // Summary every 2 s
    if (now - lastReport_us > 2'000'000) {
        lastReport_us = now;
        String s = "HEALTH IMU:";
        s += imu.failed ? "F" : "OK";
        s += " BARO:"; s += baro.failed ? "F" : "OK";
        s += " SD:";   s += sd.failed ? "F" : "OK";
        s += " TVC:";  s += tvcInhibited ? "OFF" : "ON";
        sdCard.logEvent(s.c_str(), flightTime);
    }
}
