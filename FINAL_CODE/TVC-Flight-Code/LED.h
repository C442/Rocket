/*
 * Common Cathode LED
 * 
 * Changes color according to the state the Flight Controller is currently in
 * 
 * STATE           | COLOR
 * ----------------|--------
 * LAUNCH_PAD_IDLE | blue
 * ASCENT          | green
 * DESCENT         | red
 * LANDED          | purple
 */

#include <Arduino.h>

#define R_LED 2
#define G_LED 3
#define B_LED 4

class LED {
  public:
    LED();
    void red();
    void green();
    void blue();
    void purple();
    void off();     
};
