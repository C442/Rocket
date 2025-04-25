/*
 * PID Controller, inspired by https://gist.github.com/bradley219/5373998
 */

#include "PID.h"

PID::PID(float Kp, float Ki, float Kd, float filter, float setpoint) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  this->filter = filter;
  this->setpoint = setpoint;
}

float PID::update(float input) {
  // Calculate delata time
  currentTime = micros();
  deltaTime = (currentTime - previousTime) / 1000000.0f;

  // Calculate error
  error = setpoint - input;

  // Integral term (with anti-windup clamp)
  errorIntegral += error * deltaTime;
  errorIntegral = constrain(errorIntegral, -maxIntegral, maxIntegral);

  // Derivative term (filtered)
  rawDerivative = (error - errorLastCycle) / deltaTime;
  float alpha = filter * deltaTime / (1.0f + filter * deltaTime);
  filteredDerivative = alpha * rawDerivative + (1.0f - alpha) * filteredDerivative;

  // Calculate total output
  output = Kp * error + Ki * errorIntegral + Kd * filteredDerivative;

  // Save current error and time for next cycle
  errorLastCycle = error;
  previousTime = currentTime;

  return output;
}
