#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Servo.h>
#include <math.h>
#include "PID.h"

// IMU and Servo setup
Adafruit_BNO055 bno = Adafruit_BNO055(55);
Servo tvcServoY;
Servo tvcServoX;

// Target roll/pitch (upright)
float targetRoll = 90.0f;
float targetPitch = 0.0f;

//Setting up the PID Controllers
PID PIDy = PID(0.35f, 0.05f, 0.4f, 50.0f, targetRoll);
PID PIDx = PID(0.35f, 0.05f, 0.4f, 50.0f, targetPitch);

//Middle Position of X and Y Servo
const int X_SERVO_MIDDLE = 102;
const int Y_SERVO_MIDDLE = 88;

// Convert quaternion to Euler angles
void quaternionToEuler(float w, float x, float y, float z, float &yaw, float &pitch, float &roll) {
  float sinr_cosp = 2 * (w * x + y * z);
  float cosr_cosp = 1 - 2 * (x * x + y * y);
  roll = atan2(sinr_cosp, cosr_cosp);

  float sinp = 2 * (w * y - z * x);
  pitch = fabs(sinp) >= 1 ? copysign(M_PI / 2, sinp) : asin(sinp);

  float siny_cosp = 2 * (w * z + x * y);
  float cosy_cosp = 1 - 2 * (y * y + z * z);
  yaw = atan2(siny_cosp, cosy_cosp);

  // Convert to degrees
  pitch *= 180.0 / M_PI;
  yaw *= 180.0 / M_PI;
  roll *= 180.0 / M_PI;
}


double gimbal_to_servo(double a, double b, double c, double gimbal_angle_deg){
  double c_prime = c - abs(gimbal_angle_deg);
  double discriminant = b*b - 4 * a * c_prime;

  if(discriminant < 0){
    Serial.println("No real solution for this gimbal angle.");
    return 0;
  }


  double servo_angle = (-b + sqrt(discriminant)) / (2 * a);
  if(gimbal_angle_deg<0){
    servo_angle *= -1;
  }
  return servo_angle;
}

void setup() {
  Serial.begin(9600);
  bno.begin();
  delay(1000);
  bno.setExtCrystalUse(true);
  tvcServoY.attach(9);
  tvcServoX.attach(10);
  tvcServoX.write(X_SERVO_MIDDLE);
}

void loop() {
  imu::Quaternion quat = bno.getQuat();

  float pitch, roll, yaw;
  quaternionToEuler(quat.w(), quat.x(), quat.y(), quat.z(), yaw, pitch, roll);

  // Compute PID output for pitch/roll
  float pidOutputY = PIDy.update(roll);
  float pidOutputX = PIDx.update(pitch);
  
  //Convert demanded gimbal angle to servo angle (via CurveFitting)
  float servo_angle_y = gimbal_to_servo(-0.00050, 0.22154, 0.04156, pidOutputY);
  float servo_angle_x = gimbal_to_servo(0.00094, 0.1562, -0.0443, pidOutputX);

  // Map output to servo range (30 degrees in each direction)
  int servoAngleY = constrain(Y_SERVO_MIDDLE + servo_angle_y, 58, 118);
  int servoAngleX = constrain(X_SERVO_MIDDLE - servo_angle_x, 72, 132);
  
  //Write angle to servos
  tvcServoY.write(servoAngleY);
  tvcServoX.write(servoAngleX);

  // Debug
  Serial.print("Roll:"); Serial.print(roll);
  Serial.print(",PIDy:"); Serial.print(pidOutputY);
  Serial.print(",ServoY:"); Serial.println(servoAngleY);

  Serial.print("Pitch:"); Serial.print(pitch);
  Serial.print(",PIDx:"); Serial.print(pidOutputX);
  Serial.print(",ServoX:"); Serial.println(servoAngleX);
  delay(50);
}
