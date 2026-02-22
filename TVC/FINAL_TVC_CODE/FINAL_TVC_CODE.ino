#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Servo.h>
#include <math.h>
#include <MadgwickAHRS.h>



// IMU and Servo setup
Adafruit_BNO055 bno = Adafruit_BNO055(55);
Servo tvcServoX, tvcServoY;


//Filter setup
Madgwick filter;

// Thrust curve data
const int thrustPoints = 9;
float thrustTimes[thrustPoints] = {0, 0.05, 0.1, 0.2, 0.5, 1.0, 2.0, 3.0, 60.0};
float thrustValues[thrustPoints] = {0, 62, 40, 38, 40, 39, 38, 37, 36};

//Middle Position of X and Y Servo
const int X_SERVO_MIDDLE = 102;
const int Y_SERVO_MIDDLE = 88;

//Set Moment Arm
const float momentArm = 0.35; // meters

//Gain Matrices for State Controller
const float K_Roll[2] = {1.1952, 0.6491};
const float K_Pitch[2] = {1.1952, 0.6491};

//Setpoint Matrices for State Controller
const float Setpoint_Roll[2] = {M_PI/2, 0.0};
const float Setpoint_Pitch[2] = {0.0, 0.0};

//Calculating thrust variables
unsigned long liftoffTime = 0;
bool launched = false;
unsigned long lastTime = 0;

//Set update rate
const unsigned long updateInterval = 10000; // 10 ms = 100 Hz


double gimbal_to_servo(double a, double b, double gimbal_angle_deg){
  return a * gimbal_angle_deg + b;
}

double state_controller(const float K[2], float x[2], const float setpoint[2]){
  float u = 0;
  x[0] -= setpoint[0];
  x[1] -= setpoint[1];
  u = -(K[0] * x[0] + K[1] * x[1]);
  return u;
}

double torque_to_angle(float torque, float thrust){
  double asin_input = constrain(torque / (momentArm * thrust), -1, 1);
  return asin(asin_input);
}

float RadToDegrees(float rad){
  return rad * 180/M_PI;
}

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

void setup() {
  Serial.begin(115200);
  bno.begin();
  delay(1000);
  bno.setExtCrystalUse(true);
  tvcServoY.attach(7);
  tvcServoX.attach(6);
  tvcServoX.write(X_SERVO_MIDDLE);
  tvcServoY.write(Y_SERVO_MIDDLE);
  filter.begin(100);       // sample frequency of the filter
  filter.setInitialRollPitch(M_PI/2, 0.0);

  lastTime = micros();
  liftoffTime = millis(); // Simulate immediate launch (change for real launch detection)
}

void loop() {

    //Getting thrust
  unsigned long now = micros();
  if (now - lastTime >= updateInterval) {
    float dt = (now - lastTime) / 1e6;
    lastTime = now;

    float flightTime = (millis() - liftoffTime) / 1000.0;
    float thrust = getThrustFromCurve(flightTime);

    uint8_t system_cal, gyro_cal, accel_cal, mag_cal = 0;
    bno.getCalibration(&system_cal, &gyro_cal, &accel_cal, &mag_cal);
    Serial.println();
    Serial.print("Calibration: Sys=");
    Serial.print(system_cal);
    Serial.print(" Gyro=");
    Serial.print(gyro_cal);
    Serial.print(" Accel=");
    Serial.print(accel_cal);
    Serial.print(" Mag=");
    Serial.println(mag_cal);

    Serial.println("--");

    //Get the rates:
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    float rollRate = gyro.x() * (M_PI/180); // rad/s
    float pitchRate = gyro.z() * (M_PI/180); // rad/s

    //Get filtered pitch and roll:
    imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

    filter.updateIMU(gyro.x(), gyro.y(), gyro.z(),acc.x(), acc.y(), acc.z());
    float roll = filter.getRollRadians();
    float pitch = filter.getPitchRadians();
    
    //Compute roll torque command:
    float state_vector_roll[2] = {roll, rollRate};
    float roll_torque = state_controller(K_Roll, state_vector_roll, Setpoint_Roll);
    float roll_state_output = torque_to_angle(roll_torque, thrust);
    float gimbalAngleRollDeg = RadToDegrees(roll_state_output);

    //Compute pitch torque command:
    float state_vector_pitch[2] = {pitch, -pitchRate};
    float pitch_torque = state_controller(K_Pitch, state_vector_pitch, Setpoint_Pitch);
    float pitch_state_output = torque_to_angle(pitch_torque, thrust);
    float gimbalAnglePitchDeg = RadToDegrees(pitch_state_output);

    //Convert demanded gimbal angle to servo angle (via CurveFitting)
    float servo_angle_y = gimbal_to_servo(6.0169, 90, gimbalAngleRollDeg);
    float servo_angle_x = gimbal_to_servo(-4.1024, 103.0, gimbalAnglePitchDeg);

    // Map output to servo range (30 degrees in each direction)
    int servoAngleY = constrain(round(servo_angle_y), 57, 118);
    int servoAngleX = constrain(round(servo_angle_x), 72, 132);

    //Write angle to servos
    tvcServoY.write(servoAngleY);
    tvcServoX.write(servoAngleX);

    // Debug
    
    Serial.print("Roll:"); Serial.print(roll * 180 / M_PI);
    Serial.print(",Roll Rate:"); Serial.print(rollRate);
    Serial.print(",StateY:"); Serial.print(roll_state_output);
    Serial.print(",ServoY:"); Serial.print(servoAngleY);
    Serial.print(",gimbalAngleRollDeg: "); Serial.print(gimbalAngleRollDeg);
    Serial.print(",Freq: "); Serial.print(1/dt);
    Serial.print(",Thrust: "); Serial.println(thrust);
    
    
    /*
    Serial.print("Pitch:"); Serial.print(pitch * 180 / M_PI);
    Serial.print(",Pitch Rate:"); Serial.print(-pitchRate);
    Serial.print(",StateX:"); Serial.print(pitch_state_output);
    Serial.print(",ServoX:"); Serial.print(servoAngleX);
    Serial.print(",gimbalAnglePitchDeg: "); Serial.println(gimbalAnglePitchDeg);
    Serial.println(thrust);
    */
  }
  
}