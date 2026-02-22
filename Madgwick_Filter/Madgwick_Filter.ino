#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Servo.h>
#include <math.h>
#include <MadgwickAHRS.h>
Madgwick filter;


Adafruit_BNO055 bno = Adafruit_BNO055(55);

double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
double gyroXangle, gyroYangle, gyroZangle;

double dt;
unsigned long millisOld;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  bno.begin();
  bno.setExtCrystalUse(true);
  filter.begin(100);       // sample frequency
  delay(1000);

  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  accX = acc.x();
  accY = acc.y();
  accZ = acc.z();
  
  double roll  = atan2(accY, sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG; //Around X
  double pitch = atan2(accX, accY) * RAD_TO_DEG; //Around Z

  gyroXangle = roll;
  gyroZangle = pitch;

  millisOld = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  uint8_t system_cal, gyro_cal, accel_cal, mag_cal = 0;
  bno.getCalibration(&system_cal, &gyro_cal, &accel_cal, &mag_cal);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  gyroX = gyro.x();
  gyroY = gyro.y();
  gyroZ = gyro.z();
  accX = acc.x();
  accY = acc.y();
  accZ = acc.z();

  double roll  = atan2(accY, sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG; //Around X
  double pitch = atan2(accX, accY) * RAD_TO_DEG; //Around Z

  dt = (millis() - millisOld) / 1000.0; // Calculate delta time
  millisOld = millis();
  
  gyroXangle += gyroX * dt; // Calculate gyro angle without any filter
  gyroZangle += gyroZ * dt;

  filter.updateIMU(
  gyro.x(), gyro.y(), gyro.z(),
  acc.x(), acc.y(), acc.z()
  );

  float rollM = filter.getRoll();
  float pitchM = filter.getPitch();

  //Print the data
  Serial.print(system_cal);
  Serial.print(",");
  Serial.print(gyro_cal);
  Serial.print(",");
  Serial.print(accel_cal);
  Serial.print(",");
  Serial.print(mag_cal);

  Serial.print(","); Serial.print(gyroX);
  Serial.print(","); Serial.print(gyroZ);
  Serial.print(","); Serial.print(gyroY);
  Serial.print(","); Serial.print(accX);
  Serial.print(","); Serial.print(accY);
  Serial.print(","); Serial.print(accZ);
  
  Serial.print(","); Serial.print(roll);
  Serial.print(","); Serial.print(pitch);

  Serial.print(","); Serial.print(gyroXangle);
  Serial.print(","); Serial.print(gyroZangle);

  Serial.print(","); Serial.print(rollM); //X
  Serial.print(","); Serial.println(pitchM); //Z
}
