#pragma once
#include <Arduino.h>

class Buzzer {
  int pin;
  unsigned long lastToggle = 0;
  bool state = false;

public:
  Buzzer(int p);

  void begin();
  void beepInterval(unsigned long intervalMs);
  void off();
};
