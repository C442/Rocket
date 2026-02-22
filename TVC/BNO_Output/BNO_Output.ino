#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Servo.h>
#include <math.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);
Servo tvcServoY, tvcServoX;

float RadToDegrees(float rad){
  return rad * 180/M_PI;
}

void quaternionToEuler(float w, float x, float y, float z, float &yaw, float &pitch, float &roll) {
  float sinr_cosp = 2 * (w * x + y * z);
  float cosr_cosp = 1 - 2 * (x * x + y * y);
  roll = atan2(sinr_cosp, cosr_cosp);

  float sinp = 2 * (w * y - z * x);
  pitch = fabs(sinp) >= 1 ? copysign(M_PI / 2, sinp) : asin(sinp);

  float siny_cosp = 2 * (w * z + x * y);
  float cosy_cosp = 1 - 2 * (y * y + z * z);
  yaw = atan2(siny_cosp, cosy_cosp);
}

void setup() {
  // put your setup code here, to run once:
    // put your setup code here, to run once:
  Serial.begin(9600);
  bno.begin();
  delay(1000);
  bno.setExtCrystalUse(true);
  Serial.println("Hell yeah");
}

void loop() {
  // put your main code here, to run repeatedly:
  imu::Quaternion quat = bno.getQuat();
  float roll, pitch, yaw;
  quaternionToEuler(quat.w(), quat.x(), quat.y(), quat.z(), yaw, pitch, roll);
  Serial.print("Roll:"); Serial.print(RadToDegrees(roll));
  Serial.print(",Pitch:"); Serial.print(RadToDegrees(pitch));
  Serial.print(",Yaw:"); Serial.println(RadToDegrees(yaw));
}
