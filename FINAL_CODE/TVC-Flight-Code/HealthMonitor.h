#pragma once
#include <Arduino.h>
#include "SDCard.h"

// Simple container for one sensor's health state
struct SensorHealth {
    uint32_t lastOk_us = 0;
    uint16_t badStreak = 0;
    bool     failed    = false;
};

class HealthMonitor {
public:
    HealthMonitor(SDCard& sd) : sdCard(sd) {}

    // called when you get a good value
    void markOK(SensorHealth& h);

    // called when you detect a bad value
    void markBAD(SensorHealth& h, uint16_t limit, const char* who, const char* detail, float flightTime);

    // called periodically (~2 Hz) to check timeouts + log summary
    void tick(float flightTime);

    // Public health structs for Rocket to update
    SensorHealth imu;
    SensorHealth baro;
    SensorHealth sd;

    bool tvcInhibited = false;

private:
    SDCard& sdCard;
    uint32_t lastReport_us = 0;
};
