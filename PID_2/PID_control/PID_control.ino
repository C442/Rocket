#include "PID.h"
#include "Wire.h"
#include <Servo.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// This is basically a copy of yours, but quite heavily modified. Yours was obviously in an early state that is definitvely not up to date anymore
// since I found issues and such. Warning. Code is messy.


// A random link to the adafruit library of the BNo0
// https://github.com/adafruit/Adafruit_BNO055/blob/master/examples/

// IMU and Servo setup
Adafruit_BNO055 bno = Adafruit_BNO055(42);
Servo tvcServo_pitch;
Servo tvcServo_yaw;

// As I will probably have to deal with in the next hours: the IMU will be crucial to decide when we are in what state.
//Luckily, I have teensy so I can upload my failures at least ;)


//values for tuning are Schnuppe, mine won't be used anyways, so it's about understanding what is happenening by implementing it oneself
float kp = 1.0;
float ki = 1.0;
float kd = 1.0;
float setpoint = 90.0;
float angular_velocity = 1.0; // part of the checkout for cascade control. Doesn't work. just to see how it could look

const int X_SERVO_MIDDLE = 102;
const int Y_SERVO_MIDDLE = 88;

PID pid_pitch = PID(kp, kp, kd, setpoint, millis());
PID pid_yaw = PID(kp, kp, kd, setpoint, millis());

PID pitch_rate_target = PID(kp, ki, kd, angular_velocity, millis());
PID yaw_rate_target = PID(kp, ki, kd, angular_velocity, millis());

bool STATE_READY_LAUNCH = false;
bool STATE_LAUNCH = false;
float LAUNCH_THRESHOLD = 3; // ms^2
int consecutive_readings = 0; 
int consecutive_readings_limit = 10;

// I understand the what and the why of this function, but damn I couldn't have come up with this myself...
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
  // put your setup code here, to run once:
  Serial.begin(9600);
  bno.begin();
  delay(1000);
  bno.setExtCrystalUse(true);
  tvcServo_pitch.attach(9);
  tvcServo_yaw.attach(10);
  tvcServo_pitch.write(X_SERVO_MIDDLE);
  tvcServo_yaw.write(Y_SERVO_MIDDLE); // lul that one was missing

  pinMode(13, OUTPUT); // buzzer boy
  pinMode(14, INPUT); // raspy yelling at the teensy

}

void loop() {
  // put your main code here, to run repeatedly:

  //checking if raspy sent a longer lasting signal
  bool val =  digitalRead(14);
  if (val){
        consecutive_readings++;
      } else {
        consecutive_readings = 0;
      }
  if (consecutive_readings >= consecutive_readings_limit){
      STATE_READY_LAUNCH == true;
      consecutive_readings = 0;
    }



  while(STATE_READY_LAUNCH){

    //buzzer all 10 secs, delay(10) => higher sensibility
    float seconds;
    seconds += 0.01;
    if (seconds >= 10 and !STATE_LAUNCH){
      digitalWrite(13, HIGH);
      digitalWrite(13, LOW);
      digitalWrite(13, HIGH);
      digitalWrite(13, LOW);
      seconds = 0;
    }

    //launch condition
    if (!STATE_LAUNCH) {
      digitalWrite(13, HIGH);
      imu::Vector<3> linacc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
      float accel_z = linacc.z(); // we can change that later. Generally, I was inconsistent wit yaw, pitch, roll here
      if (accel_z >= LAUNCH_THRESHOLD){
        consecutive_readings++;
      } else {
        consecutive_readings = 0;
      }
      
    if (consecutive_readings >= consecutive_readings_limit){
      STATE_LAUNCH == true;
    }

    

    while (STATE_LAUNCH){

      //for angle
      imu::Quaternion quat = bno.getQuat();
      imu::Vector<3> eul = quat.toEuler(); // I found this transformation in a forum : https://forum.arduino.cc/t/bno055-quaternion-to-euler/632369/5
      float yaw = eul.x();
      float pitch = eul.y();
      float roll = eul.z();

      //for angle ratem, aka: cascade control
      imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
      float pitch_rate = gyro.y();
      float yaw_rate = gyro.z();

      //for detecting rest
      imu::Vector<3> linacc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL); 

      float target_pitch = pid_pitch.compute(pitch, millis());
      float target_yaw = pid_yaw.compute(yaw, millis());

      float pitch_rate_ = pitch_rate_target.compute(target_pitch, millis());
      float yaw_step_ = yaw_rate_target.compute(target_yaw, millis());


      //Convert demanded gimbal angle to servo angle (via CurveFitting)
      float servo_angle_y = gimbal_to_servo(-0.00050, 0.22154, 0.04156, pitch_rate_);
      float servo_angle_x = gimbal_to_servo(0.00094, 0.1562, -0.0443, yaw_step_);

      tvcServo_pitch.write(X_SERVO_MIDDLE + servo_angle_y);
      tvcServo_yaw.write(Y_SERVO_MIDDLE + servo_angle_x);


      // detecting rest
      if (linacc <= LAUNCH_THRESHOLD){
        consecutive_readings++;
      } else {
        consecutive_readings = 0;
      }
    if (consecutive_readings >= consecutive_readings_limit){
      consecutive_readings = 0;
      break;
    }


      delay(50); //lul, I spent a 15 min wondering if this delay(50) should really be a constant. 
      // My fear of overshooting is real and I wanted to modify the rate of change based on the size of the error to be corrected by tying it to the delay
      // Then I remembered: changing rate of angular velocity? that's cascade control! At 1:21, your brain's just working differently.
      // It's after that realisation that I started adding the idea of the cascade control to the script, and felt hungry, see below:
      // I think I will make myself an omelette now. You want one too?



      }

    //buzzing after rest
    if (STATE_LAUNCH){
      seconds += 0.01;
      if (seconds >= 10 and !STATE_LAUNCH){
        digitalWrite(13, HIGH);
        digitalWrite(13, LOW);
        digitalWrite(13, HIGH);
        digitalWrite(13, LOW);
        seconds = 0;
    }

    }
    delay(10);
    }
  // We are 3:00. The program compiles, but there are certainly logic errors somewhere. I did what I wanted to do. My atonment is however not done yet...
  delay(50);
  }
}
//adding another connection to teensy for reseting program? We are 4:30 and I think I will go to bed now, after pushing this on git