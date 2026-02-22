/*
 * State Machine
 * 
 * The code is divided into four states:
 * 
 * LAUNCH_PAD_IDLE
 *   Check the acceleration of the rocket on vertical axis
 * ASCENT
 *   Thrust vector control is active
 *   Flight data is logged to SD-Card    ‾\
 * DESCENT                                 | Eject parachute
 *   Flight data is logged to SD-Card   <-/
 * LANDED
 *   No active code (computer waiting to be turned off)
 *   Computer beeps in regular intervals (easier to find rocket)
 */

#include <Arduino.h>
#include <Wire.h>
#include "Rocket.h"

#define LIFT_THRESHOLD -3 //(in m/s^2)

// Calculated with a MATLAB flight simulation
#define TIME_TO_APOGEE 11     // Time from start until rocket reaches max apogee and parachute will be ejected (in s)
#define TIME_TO_LANDING 60   // Time from start until rocket lands (in s)

enum State {
  LAUNCH_PAD_IDLE,
  ASCENT,
  DESCENT,
  LANDED
};

class StateMachine {
  public:
    StateMachine();
    void stateMachineLoop();
    bool liftOffCheck();
    bool maxApogeeCheck();
    bool landedCheck();

  private:
    Rocket rocket;
    State activeState;
};
