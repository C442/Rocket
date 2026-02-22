#include <Servo.h>
#include <Arduino.h> // not strictly needed, but explicit

Servo servox, servoy;

const int X_SERVO_MIDDLE = 103;  // degrees
const int Y_SERVO_MIDDLE = 94;   // degrees
const float RADIUS_DEG    = 30.0; // circle radius in degrees
const uint16_t STEP_DELAY_MS = 10; // ~50 Hz update
const int STEPS = 1000;

void setup() {
  Serial.begin(9600);
  servox.attach(6);
  servoy.attach(7);

  for (int lap = 0; lap < 5; ++lap) {
    for (int i = 0; i <= STEPS; ++i) {
      float t = (float)i / (float)STEPS;     // 0..1
      float theta = t * TWO_PI;              // radians
      int x = (int)lround(X_SERVO_MIDDLE + RADIUS_DEG * cosf(theta));
      int y = (int)lround(Y_SERVO_MIDDLE + RADIUS_DEG * sinf(theta));
      // safety clamp (optional)
      x = constrain(x, 0, 180);
      y = constrain(y, 0, 180);

      servox.write(x);
      servoy.write(y);
      delay(STEP_DELAY_MS);
    }
  }

  // recentre when done
  servox.write(X_SERVO_MIDDLE);
  servoy.write(Y_SERVO_MIDDLE);
}

void loop() {
  // empty: program finishes after 5 circles
}
