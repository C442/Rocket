#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_TiCoServo.h>
#include <math.h>
#include <MadgwickAHRS.h>
Madgwick filter;

Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_TiCoServo tvcServoY, tvcServoX; // create servo object to control a servo;

const int X_SERVO_MIDDLE = 102; //Pitch = 0
const int Y_SERVO_MIDDLE = 88; //Roll = 0

unsigned long lastTime = 0;
unsigned long start = 0;
unsigned long now = 0;
float dt = 0;

double gimbal_to_servo(double a, double b, double gimbal_angle_deg){
  return a * gimbal_angle_deg + b;
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

float RadToDegrees(float rad){
  return rad * 180/M_PI;
}

long readVcc() {
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // wait for Vref to settle
  ADCSRA |= _BV(ADSC); // start conversion
  while (bit_is_set(ADCSRA, ADSC));
  int result = ADC;
  long vcc = 1125300L / result; // 1125300 = 1.1*1023*1000
  return vcc; // in millivolts
}

void sys_id(float target_angle){
  Serial.println("dt, Target_Angle, Response_Angle");
  if((now - start) / 1e6 <= 5.0){
    now = micros();
    dt = (now - lastTime) / 1e6;
    lastTime = now;

    int servoAngleX = constrain(gimbal_to_servo(-5.094, 102, target_angle), 72, 132);
    tvcServoX.write(servoAngleX);

    imu::Quaternion quat = bno.getQuat();
    float roll, pitch, yaw;
    quaternionToEuler(quat.w(), quat.x(), quat.y(), quat.z(), yaw, pitch, roll);


    Serial.print(dt);
    Serial.print(", ");
    Serial.print(target_angle);
    Serial.print(", ");
    Serial.println(RadToDegrees(pitch));
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  bno.begin();
  delay(1000);
  bno.setExtCrystalUse(true);

  filter.begin(100);       // sample frequency

  tvcServoY.attach(9);
  tvcServoY.write(Y_SERVO_MIDDLE);
  tvcServoX.attach(10);
  tvcServoX.write(X_SERVO_MIDDLE);
  lastTime = micros();

  float servo_angle_y = gimbal_to_servo(6.0169, 88, 0);
  float servo_angle_x = gimbal_to_servo(-5.094, 102, 0);

  // Map output to servo range (30 degrees in each direction)
  int servoAngleY = constrain(servo_angle_y, 58, 118);
  int servoAngleX = constrain(servo_angle_x, 72, 132);

  //Write angle to servos
  tvcServoY.write(servoAngleY);
  tvcServoX.write(servoAngleX);

  delay(500);

  unsigned long start = micros();
  unsigned long now = micros();
  
  Serial.println("Starting curve fitting now...");
  delay(100);
  Serial.println("Servo_Angle, Gimbal_Angle");
  for(int i = 72; i <= 132; i++){
    tvcServoX.write(i);
    delay(500);
    imu::Quaternion quat = bno.getQuat();
    float roll, pitch, yaw;
    quaternionToEuler(quat.w(), quat.x(), quat.y(), quat.z(), yaw, pitch, roll);
    Serial.print(i);
    Serial.print(", ");
    Serial.println(RadToDegrees(pitch));
  }
  tvcServoX.write(X_SERVO_MIDDLE);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  
  imu::Quaternion quat = bno.getQuat();
  float roll, pitch, yaw;
  quaternionToEuler(quat.w(), quat.x(), quat.y(), quat.z(), yaw, pitch, roll);
  
  long vcc = readVcc();
  Serial.println(vcc); //mV
  
}


