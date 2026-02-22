#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Servo.h>
#include "DualAxisPID.h"
#include <math.h>

// --- Settings ---
//#define BYPASS_INNER_LOOP  // Comment this out for full mode

// Thrust curve data
const int thrustPoints = 9;
float thrustTimes[thrustPoints] = {0, 0.05, 0.1, 0.2, 0.5, 1.0, 2.0, 3.0, 60.0};
float thrustValues[thrustPoints] = {0, 62, 40, 38, 40, 39, 38, 37, 36};

Adafruit_BNO055 bno = Adafruit_BNO055(55);
Servo tvcServoY, tvcServoX;

const int X_SERVO_MIDDLE = 102;
const int Y_SERVO_MIDDLE = 88;
const float momentArm = 0.35; // meters
const float targetRoll = M_PI/2;
const float targetPitch = 0.0;

// Initial PID gains (modifiable via Serial)
float outerKp = 50;
float outerKi = 20;
float outerKd = 0.0;

float innerKp = 0.1;
float innerKi = 0.0;
float innerKd = 0.002;

DualAxisPID controller(outerKp, outerKi, outerKd, innerKp, innerKi, innerKd, 40.0);  // Filter coeff N;

PIDState rollAnglePID, rollRatePID, pitchAnglePID, pitchRatePID;

unsigned long liftoffTime = 0;
bool launched = false;
unsigned long lastTime = 0;

float getThrustFromCurve(float timeSinceLaunch) {
  if (timeSinceLaunch <= thrustTimes[0]) return thrustValues[0];
  if (timeSinceLaunch >= thrustTimes[thrustPoints - 1]) return thrustValues[thrustPoints - 1];

  for (int i = 0; i < thrustPoints - 1; i++) {
    if (timeSinceLaunch >= thrustTimes[i] && timeSinceLaunch <= thrustTimes[i + 1]) {
      float t0 = thrustTimes[i];
      float t1 = thrustTimes[i + 1];
      float thrust0 = thrustValues[i];
      float thrust1 = thrustValues[i + 1];
      float factor = (timeSinceLaunch - t0) / (t1 - t0);
      return thrust0 + factor * (thrust1 - thrust0);
    }
  }
  return 0.0;
}

//Transformation in rad
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

void bno_ready(){
  Serial.println("Waiting for valid IMU orientation...");

  float roll = 0.0, pitch = 0.0, yaw = 0.0;
  unsigned long waitStart = millis();

  while (true) {
    imu::Quaternion quat = bno.getQuat();
    quaternionToEuler(quat.w(), quat.x(), quat.y(), quat.z(), yaw, pitch, roll);

    // Wait until roll or pitch has moved at least 5 degrees from zero
    if (abs(roll) > 5.0 || abs(pitch) > 5.0) break;

    // Optional: timeout safety
    if (millis() - waitStart > 5000) {
      Serial.println("Timeout: IMU never became ready!");
      break;
    }

    delay(10);
  }

  Serial.println("IMU orientation stabilized!");
}

void handleSerialTuning() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.startsWith("O ")) {
      sscanf(input.c_str(), "O %f %f %f", &outerKp, &outerKi, &outerKd);
      controller.setOuterGains(outerKp, outerKi, outerKd);
      Serial.println("Updated Outer PID");
    } else if (input.startsWith("I ")) {
      sscanf(input.c_str(), "I %f %f %f", &innerKp, &innerKi, &innerKd);
      controller.setInnerGains(innerKp, innerKi, innerKd);
      Serial.println("Updated Inner PID");
    } else {
      Serial.println("Unknown command. Use:\n  O Kp Ki Kd\n  I Kp Ki Kd");
    }
  }
}

float RadToDegrees(float rad){
  return rad * 180/M_PI;
}

void setup() {
  Serial.begin(9600);
  bno.begin();
  delay(1000);
  bno.setExtCrystalUse(true);

  tvcServoY.attach(9);
  tvcServoY.write(Y_SERVO_MIDDLE);
  tvcServoX.attach(10);
  tvcServoX.write(X_SERVO_MIDDLE);

  // Wait for BNO to return a valid quaternion (not all zeros)
  bno_ready();
  lastTime = micros();
  liftoffTime = millis(); // Simulate immediate launch (change for real launch detection)
}

void loop() {
  handleSerialTuning();
  unsigned long now = micros();
  float dt = (now - lastTime) / 1e6;
  if (dt <= 0.0f || dt > 0.1f) dt = 0.01f;
  lastTime = now;

  float flightTime = (millis() - liftoffTime) / 1000.0;
  float thrust = getThrustFromCurve(flightTime);
  if (thrust > 0.0){
    imu::Quaternion quat = bno.getQuat();
    float roll, pitch, yaw;
    quaternionToEuler(quat.w(), quat.x(), quat.y(), quat.z(), yaw, pitch, roll);

    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    float rollRate = gyro.x(); // rad/s
    float pitchRate = gyro.z(); // rad/s

    // === Apply deadband to ignore noise ===
    const float gyroDeadband = 0.5; // rad/s

    if (abs(rollRate) < gyroDeadband) rollRate = 0.0;
    if (abs(pitchRate) < gyroDeadband) pitchRate = 0.0;

    float desiredRollRate = controller.computeOuterLoop(targetRoll, roll, rollAnglePID, dt);
    float desiredPitchRate = controller.computeOuterLoop(targetPitch, pitch, pitchAnglePID, dt);

    float torqueRoll, torquePitch;

    #ifdef BYPASS_INNER_LOOP
      // Bypass inner loop: Scale the desired rate directly to torque
      torqueRoll = desiredRollRate * 0.1;   // 0.1 = gain factor, you can tweak
      torquePitch = desiredPitchRate * 0.1;
    #else
      // Full cascade control: Angle PID + Rate PID
      torqueRoll = controller.computeInnerLoop(desiredRollRate, rollRate, rollRatePID, dt);
      torquePitch = controller.computeInnerLoop(desiredPitchRate, pitchRate, pitchRatePID, dt);
    #endif

    float inputRoll = constrain(torqueRoll / (thrust * momentArm), -1.0, 1.0);
    float gimbalAngleRollRad = asin(inputRoll);

    float inputPitch = constrain(torquePitch / (thrust * momentArm), -1.0, 1.0);
    float gimbalAnglePitchRad = asin(inputPitch);

    float gimbalAngleRollDeg = RadToDegrees(gimbalAngleRollRad);
    float gimbalAnglePitchDeg = RadToDegrees(gimbalAnglePitchRad);

    float ServoRollDeg = gimbal_to_servo(-0.00050, 0.22154, 0.04156, gimbalAngleRollDeg);
    float ServoPitchDeg = gimbal_to_servo(0.00094, 0.1562, -0.0443, gimbalAnglePitchDeg);

    tvcServoY.write(constrain(Y_SERVO_MIDDLE - ServoRollDeg, 58, 118));
    tvcServoX.write(constrain(X_SERVO_MIDDLE + ServoPitchDeg, 72, 132));

    Serial.print("Roll:"); Serial.print(RadToDegrees(roll));
    Serial.print(",Pitch:"); Serial.print(RadToDegrees(pitch));
    Serial.print(",GimbalY:"); Serial.print(gimbalAngleRollDeg);
    Serial.print(",GimbalX:"); Serial.print(gimbalAnglePitchDeg);
    Serial.print(",RollRate:"); Serial.print(rollRate);
    Serial.print(",PitchRate:"); Serial.print(pitchRate);
  }

  Serial.print(",Time:"); Serial.print(flightTime);
  Serial.print(",Thrust:"); Serial.print(thrust);
  Serial.print(",dt:"); Serial.println(dt);
  delay(50);
}
