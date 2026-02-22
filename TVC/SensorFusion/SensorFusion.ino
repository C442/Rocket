#include "QuaternionKalman.h"
#include <Adafruit_BNO055.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55);
QuaternionKalman qFilter;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  if (!bno.begin()) {
    Serial.println("No BNO055 detected!");
    while (1);
  }
  bno.setMode(OPERATION_MODE_IMUPLUS);
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  static unsigned long lastTime = micros();
  unsigned long now = micros();
  float dt = (now - lastTime) / 1e6;
  lastTime = now;

  sensors_event_t accel_event, gyro_event;
  bno.getEvent(&accel_event, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gyro_event, Adafruit_BNO055::VECTOR_GYROSCOPE);

// Raw inputs in SI units: gyro in rad/s, accel in m/s^2
  qFilter.update(
    gyro_event.gyro.x,
    gyro_event.gyro.y,
    gyro_event.gyro.z,
    accel_event.acceleration.x,
    accel_event.acceleration.y,
    accel_event.acceleration.z,
    dt
  );
  // Get orientation
  EulerAngles eul = qFilter.getEulerAngles();

  // Convert to degrees for debugging
  float roll  = eul.roll  * 180.0 / PI;
  float pitch = eul.pitch * 180.0 / PI;
  float yaw   = eul.yaw   * 180.0 / PI;
  
  Serial.print("Pitch:"); Serial.print(pitch);
  Serial.print(",Roll:"); Serial.print(roll);
  Serial.print(",Yaw:"); Serial.println(yaw);
}
