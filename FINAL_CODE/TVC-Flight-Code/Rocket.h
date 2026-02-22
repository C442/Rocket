/*
 * Rocket code file
 * 
 * Contains code that will be run in a specific state
 * Functions are called in StateMachine.cpp
 * Header file contains all constants that are specific to the model rocket
 */

#include <Wire.h>
#include <Arduino.h>
#include <Servo.h>
#include "SDCard.h"
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP3XX.h>
#include <math.h>
#include <MadgwickAHRS.h>
#include "LED.h"
#include "Buzzer.h"
#include "HealthMonitor.h"

#define Y_SERVO_PIN 7               // Y Servo (y axis labeled on IMU) = Y Servo (labeled on pcb)
#define X_SERVO_PIN 6               // X Servo (z axis labeled on IMU) = X Servo (labeled on pcb)

#define Y_SERVO_MIDDLE 94           // Y Servo horn vertical position (motor mount is vertical)
#define X_SERVO_MIDDLE 103           // X Servo horn vertical position (motor mount is vertical)

#define PARACHUTE_SERVO_PIN 14       // Parachute Servo = Servo 3 (labeled on pcb)
#define PARACHUTE_CLOSED 90          // Position of the parachute servo before ejection (in deg)
#define PARACHUTE_EJECT -90         // Position of the parachute servo after ejection (in deg)

#define DEG2RAD 0.01745329251f      // Convert degrees to radians by multiplying with this number
#define RAD2DEG 57.2957795131f      // Convert radians to degrees by multiplying with this number

#define SEA_LEVEL_PRESSURE 1013.25  // Standard atmospheric pressure at sea level (in hPa)

// ===== State-Controller (LQR-style) config (from FINAL_TVC_CODE) =====

// Thrust curve (s, N) for interpolation
static const int TVC_THRUST_POINTS = 12;
extern const float TVC_THRUST_T[TVC_THRUST_POINTS]; // declared below in class as static
extern const float TVC_THRUST_F[TVC_THRUST_POINTS];           // declared below in class as static

// Motor mount geometry
static constexpr float TVC_MOMENT_ARM = 0.33f; // meters

// 2x1 gains [angle, rate] for roll and pitch axes
static constexpr float TVC_K_ROLL[2]  = {2.0f, 0.8619f};
static constexpr float TVC_K_PITCH[2] = {2.0f, 0.8619f};

class Rocket {
  public:
    Rocket();

    SDCard sdCard;
    LED led;
    Buzzer buzzer;

    // Angular Velocity Vector
    // Three axis of rotation speed in radians per second (rad/s)
    imu::Vector<3> gyro;

    // Acceleration Vector
    // Three axis of acceleration (gravity + linear motion) in m/s^2
    imu::Vector<3> acc;

    float flightTime = 0.0f; // in s
    float flightStartTime = 0.0f; // in ms
    
    void padIdle();
    void ascent();
    void maxApogee();
    void descent();
    void landed();

  private:
    // ===== TVC state controller members =====
    Adafruit_BNO055 bno = Adafruit_BNO055(55);
    Madgwick filter;
    unsigned long lastMicros = 0;
    bool launched = false; // if you have external launch detection, wire it in

    // Thrust curve tables (definition in .cpp)
    static const float TVC_T_TBL[TVC_THRUST_POINTS];
    static const float TVC_F_TBL[TVC_THRUST_POINTS];

    // Helpers
    float tvcInterpolateThrust(float tSec) const;
    float tvcStateController(const float K[2], float x0, float x1, const float sp0, const float sp1) const;
    float tvcTorqueToAngleRad(float torque, float thrust) const;
    float tvcGimbalToServo(float a, float b, float gimbalAngleDeg) const;

    Adafruit_BMP3XX bmp;
    
    Servo tvcServoY;
    Servo tvcServoX;
    Servo parachuteServo;
    
    unsigned long currentTime = 0.0;
    unsigned long previousTime = 0.0;
    float deltaTime = 0.0f;

    // Cached values for logging (updated in ascent)
    float lastRoll   = 0.0f;
    float lastPitch  = 0.0f;
    int   lastServoY = 0;
    int   lastServoX = 0;
    float lastThrust = 0.0f;

    //Health Monitor
    HealthMonitor health;
};
