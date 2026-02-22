#include "StateMachine.h"

StateMachine::StateMachine() {
  activeState = LAUNCH_PAD_IDLE; // Default state
  rocket.led.blue();
}

void StateMachine::stateMachineLoop() {
  switch (activeState) {
    case LAUNCH_PAD_IDLE:
      rocket.padIdle();
      // Detect lift off -> Ascent flight
      if (liftOffCheck()) {
        // Save the time of lift off
        rocket.flightStartTime = millis();
        rocket.led.green();
        activeState = ASCENT;
        //Open new SD Card Files
        if (!rocket.sdCard.isOpen()) {                // <-- prevents re-opening
          if (rocket.sdCard.startLogs("DATA","EVENT")) {
            rocket.sdCard.logEvent("Liftoff detected", rocket.flightTime);
          }
        }
      }
      break;
    case ASCENT:
      // TVC active
      // Log Data
      rocket.ascent();
      // Detect max apogee -> Descent flight and eject Parachute
      if (maxApogeeCheck()) {
        rocket.maxApogee();
        activeState = DESCENT;
        rocket.sdCard.logEvent("Apogee reached, chute ejected", rocket.flightTime);
        rocket.led.red();
      }
      break;
    case DESCENT:
      rocket.descent();
      // Log Data
      // Detect landing
      if (landedCheck()) {
        rocket.sdCard.logEvent("Landing detected", rocket.flightTime);
        rocket.sdCard.flushNow();
        rocket.sdCard.close();
        activeState = LANDED;
        rocket.led.purple();
      }
      break;
    case LANDED:
      // Do nothing
      rocket.buzzer.beepInterval(500);
      break;
  }
}

// Check if rocket has launched by looking at the acceleration on vertical axis (y-axis)
bool StateMachine::liftOffCheck() {
  if (rocket.acc.y() < LIFT_THRESHOLD) {
    return true;
  }
  return false;
}

// Check if rocket has reached apogee by looking at the time it takes to reach apogee
bool StateMachine::maxApogeeCheck() {
  if (rocket.flightTime > TIME_TO_APOGEE) {
    return true;
  }
  return false;
}

// Check if rocket has landed by looking at the time it takes to land
bool StateMachine::landedCheck() {
  if (rocket.flightTime > TIME_TO_LANDING) {
    return true;
  }
  return false;
}
