#include "StateMachine.h"

StateMachine *stateMachine;

unsigned long lastLoopMicros = 0;
const unsigned long LOOP_INTERVAL_US = 10000; // 10 ms = 100 Hz

void setup() {
    Serial.begin(115200);
    stateMachine = new StateMachine();
}

void loop() {
    unsigned long now = micros();

    // Run exactly every 10 ms
    if (now - lastLoopMicros >= LOOP_INTERVAL_US) {
        lastLoopMicros = now;
        stateMachine->stateMachineLoop();
    }
}
