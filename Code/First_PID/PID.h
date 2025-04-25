/*
 * PID-Controller
 * 
 * The orientation of the motor mount is calculated with two separate PID-Controllers
 */

#include <Wire.h>
#include <Arduino.h>

class PID {
  public:
    PID(float Kp, float Ki, float Kd, float filter, float setpoint);
    float update(float input);

  private:
    // Proportional gain
    float Kp = 0.0f;
    // Integral gain
    float Ki = 0.0f;
    // Derivative gain
    float Kd = 0.0f;

    // Loop interval time
    float deltaTime = 0.0f;
    float error = 0.0f;
    float setpoint = 0.0f;
    float errorIntegral = 0.0f;
    float rawDerivative = 0.0f;
    float filteredDerivative = 0.0f;
    float errorLastCycle = 0.0f;

    // Some sort of low pass filter on the derivative term
    float filter = 0.0f;
    
    float output = 0.0f;

    unsigned long currentTime = 0.0;
    unsigned long previousTime = 0.0;

    //Anti-windup clamp
    float maxIntegral = 50.0;
};
