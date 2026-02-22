#include "Buzzer.h"

Buzzer::Buzzer(int p) : pin(p) {}

void Buzzer::begin() {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

// Make the buzzer beep in a specific interval
void Buzzer::beepInterval(unsigned long intervalMs) {
  unsigned long now = millis();
  if (now - lastToggle >= intervalMs) {
    state = !state;
    digitalWrite(pin, state ? HIGH : LOW);
    lastToggle = now;
  }
}

// Turn the buzzer off
void Buzzer::off() {
  digitalWrite(pin, LOW);
  state = false;
}
