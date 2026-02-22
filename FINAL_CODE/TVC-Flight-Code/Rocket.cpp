#include "Rocket.h"

Rocket::Rocket() : buzzer(5), health(sdCard) {
  // Attach and initialize servos
  Wire.begin();
  Wire.setClock(400000);

  tvcServoY.attach(Y_SERVO_PIN);
  tvcServoX.attach(X_SERVO_PIN);
  parachuteServo.attach(PARACHUTE_SERVO_PIN);

  tvcServoY.write(Y_SERVO_MIDDLE);
  tvcServoX.write(X_SERVO_MIDDLE);
  parachuteServo.write(PARACHUTE_CLOSED);
  
  bno.begin();
  bno.setExtCrystalUse(true);
  bmp.begin_I2C(0x77);
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  filter.begin(100);       // sample frequency of the filter
  filter.setInitialRollPitch(M_PI/2, 0.0);
  buzzer.begin();
  sdCard.begin();
}

void Rocket::padIdle() {
  // Update acceleration for lift off check
  acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
}

void Rocket::ascent() {
  // Calculate flight time in s
  flightTime = (millis() - flightStartTime) / 1000.0f;

  // ----- TVC ALGORITHM (State Controller) -----

  // Read raw sensors
  gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);       // deg/s
  acc  = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);   // m/s^2

  // ---- IMU health ----
  if (isfinite(gyro.x()) && isfinite(gyro.y()) && isfinite(gyro.z()) &&
      isfinite(acc.x())  && isfinite(acc.y())  && isfinite(acc.z())) {
    health.markOK(health.imu);
  } else {
    health.markBAD(health.imu, /*limit*/5, "IMU", "NaN/Inf", flightTime);
  }

  // Filter to get roll/pitch (radians)
  filter.updateIMU(gyro.x(), gyro.y(), gyro.z(), acc.x(), acc.y(), acc.z());
  float roll  = filter.getRollRadians();
  float pitch = filter.getPitchRadians();

  // Rates (rad/s) – mapping as used in FINAL_TVC_CODE
  float rollRate  = gyro.x() * DEG2RAD;
  float pitchRate = gyro.z() * DEG2RAD;

  // Thrust from curve (N)
  float thrust = tvcInterpolateThrust(flightTime);

  // State vectors [angle, rate]; setpoints are [M_PI/2, 0] for roll and [0, 0] for pitch
  float rollTorque  = tvcStateController(TVC_K_ROLL,  roll,  rollRate,  M_PI/2, 0.0f);
  float pitchTorque = tvcStateController(TVC_K_PITCH, pitch, -pitchRate, 0.0f,  0.0f); // note sign for pitch rate

  // Convert torque to demanded gimbal angle (radians -> degrees)
  float rollGimbalRad  = tvcTorqueToAngleRad(rollTorque,  thrust);
  float pitchGimbalRad = tvcTorqueToAngleRad(pitchTorque, thrust);
  float rollGimbalDeg  = rollGimbalRad  * RAD2DEG;
  float pitchGimbalDeg = pitchGimbalRad * RAD2DEG;

  // Convert to servo angles using curve-fit (from FINAL_TVC_CODE)
  float servoY = tvcGimbalToServo(6.0169f,  90.0f,  rollGimbalDeg);
  float servoX = tvcGimbalToServo(-4.1024f, 103.0f, pitchGimbalDeg);

  // Constrain to mechanical limits (approx. ±30° around middle)
  int servoYdeg = (int)constrain(lroundf(servoY),  57, 118);
  int servoXdeg = (int)constrain(lroundf(servoX),  72, 132);

  
  servoYdeg = constrain(2*Y_SERVO_MIDDLE - servoYdeg, 57, 118);
  servoXdeg = constrain(2*X_SERVO_MIDDLE - servoXdeg, 72, 132);

  // Write servos
  if (health.tvcInhibited) {  //Emergency if IMU died
    tvcServoY.write(Y_SERVO_MIDDLE);
    tvcServoX.write(X_SERVO_MIDDLE);
  }
  else{
    tvcServoY.write(servoYdeg);
    tvcServoX.write(servoXdeg);
  }

  // ----- DATA LOGGING -----
  // cache for logging
  lastRoll = roll; lastPitch = pitch;
  lastServoY = servoYdeg; lastServoX = servoXdeg; lastThrust = thrust;

  // --- SLOW PART (25 Hz): baro + logging every 4th tick ---
  static uint8_t slowDiv = 0;
  if (++slowDiv >= 4) {          // 100 / 4 = 25 Hz
    slowDiv = 0;

    float temperature = NAN, pressure = NAN, altitude = NAN;
    if (bmp.performReading()) {               // one baro conversion/read
      temperature = bmp.temperature;
      pressure = bmp.pressure / 100.0f;
      altitude   = bmp.readAltitude(SEA_LEVEL_PRESSURE);
    }

    // Accept sane ranges as "OK" (tuned to my conditions)
    bool baroSane =
        isfinite(temperature) && isfinite(pressure) && isfinite(altitude) &&
        pressure   > 300.0f  && pressure   < 1100.0f &&
        altitude   > -200.0f && altitude   < 5000.0f;

    if (baroSane) {
      health.markOK(health.baro);
    } else {
      health.markBAD(health.baro, /*limit*/3, "BARO", "range/NaN", flightTime);
    }

    sdCard.logData(
      flightTime,
      acc.x(), acc.y(), acc.z(),
      gyro.x(), gyro.y(), gyro.z(),
      lastRoll, lastPitch,
      temperature, pressure, altitude,
      lastServoY, lastServoX, lastThrust
    );

    // Mark SD “alive” (we do timeouts in HealthMonitor::tick)
    health.markOK(health.sd);

    // ===== Periodic health processing =====
    health.tick(flightTime);
  }
}

void Rocket::maxApogee() {
  // Eject Parachute
  parachuteServo.write(PARACHUTE_EJECT);

  // Set tvc servos to middle position
  tvcServoY.write(Y_SERVO_MIDDLE);
  tvcServoX.write(X_SERVO_MIDDLE);
}

void Rocket::descent() {
  // Calculate flight time in s
  flightTime = (millis() - flightStartTime) / 1000.0f;

  // ----- DATA LOGGING -----
  gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);       // deg/s
  acc  = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);   // m/s^2

    // ---- IMU health ----
  if (isfinite(gyro.x()) && isfinite(gyro.y()) && isfinite(gyro.z()) &&
      isfinite(acc.x())  && isfinite(acc.y())  && isfinite(acc.z())) {
    health.markOK(health.imu);
  } else {
    health.markBAD(health.imu, /*limit*/5, "IMU", "NaN/Inf", flightTime);
  }

  float temperature = NAN, pressure = NAN, altitude = NAN;
  if (bmp.performReading()) {
    temperature = bmp.temperature;              // °C
    pressure = bmp.pressure / 100.0f;       // Pa -> hPa
    altitude = bmp.readAltitude(SEA_LEVEL_PRESSURE);
  }
  
  // Accept sane ranges as "OK" (tuned to my conditions)
  bool baroSane =
      isfinite(temperature) && isfinite(pressure) && isfinite(altitude) &&
      pressure   > 300.0f  && pressure   < 1100.0f &&
      altitude   > -200.0f && altitude   < 5000.0f;

  if (baroSane) {
    health.markOK(health.baro);
  } else {
    health.markBAD(health.baro, /*limit*/3, "BARO", "range/NaN", flightTime);
  }

  sdCard.logData(flightTime, acc.x(), acc.y(), acc.z(), gyro.x(), gyro.y(), gyro.z(), 0.0f, 0.0f, temperature, pressure, altitude, 0, 0, 0.0f);

  // Mark SD “alive” (we do timeouts in HealthMonitor::tick)
  health.markOK(health.sd);

  // ===== Periodic health processing =====
  health.tick(flightTime);
}

// ===== TVC State Controller helpers/tables (from FINAL_TVC_CODE) =====
const float Rocket::TVC_T_TBL[TVC_THRUST_POINTS] = {0.0f, 0.05f, 0.1f, 0.2f, 0.5f, 1.0f, 2.0f, 3.0f, 3.3f, 3.6f, 3.9f, 4.35f};
const float Rocket::TVC_F_TBL[TVC_THRUST_POINTS] = {0.0f, 62.0f, 40.0f, 38.0f, 40.0f, 39.0f, 38.0f, 37.0f, 36.0f, 30.0f, 15.0f, 0.0f};

// interpolate between the thrust points
float Rocket::tvcInterpolateThrust(float tSec) const {
  if (tSec <= TVC_T_TBL[0]) return TVC_F_TBL[0];
  for (int i = 0; i < TVC_THRUST_POINTS - 1; ++i) {
    if (tSec <= TVC_T_TBL[i+1]) {
      float t0 = TVC_T_TBL[i], t1 = TVC_T_TBL[i+1];
      float f0 = TVC_F_TBL[i], f1 = TVC_F_TBL[i+1];
      float alpha = (tSec - t0) / (t1 - t0);
      return f0 + alpha * (f1 - f0);
    }
  }
  return TVC_F_TBL[TVC_THRUST_POINTS - 1];
}

// u = -K * (x - r), with x = [angle, rate] (typical LQR Controller)
float Rocket::tvcStateController(const float K[2], float x0, float x1, const float sp0, const float sp1) const {
  float e0 = x0 - sp0;
  float e1 = x1 - sp1;
  return -(K[0]*e0 + K[1]*e1);
}

// torque = Thrust * sin(theta) * momentArm  => theta = asin(torque / (Thrust * momentArm))
float Rocket::tvcTorqueToAngleRad(float torque, float thrust) const {
  if (thrust <= 1e-3f) return 0.0f;
  float x = torque / (TVC_MOMENT_ARM * thrust);
  if (x > 1.0f) x = 1.0f;
  if (x < -1.0f) x = -1.0f;
  return asinf(x);
}

// Linear map from demanded gimbal angle (deg) to servo angle (deg); coefficients from curve-fit
float Rocket::tvcGimbalToServo(float a, float b, float gimbalAngleDeg) const {
  return a * gimbalAngleDeg + b;
}
