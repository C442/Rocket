#include <Servo.h>

Servo myservo;  // create Servo object to control a servo
int servo_pin = 9;

class ChuteRelease{
  public:
    void lock(){
      myservo.write(lock_position);
    }
    void deploy(){
      myservo.write(open_position);
    }
  private:
    int lock_position = -90;
    int open_position = 90;
};

ChuteRelease chute;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  myservo.attach(servo_pin);

}

void loop() {
  // put your main code here, to run repeatedly:
  while(Serial.available() == 0);
  String input = Serial.readString();
  input.trim();
  if(input == "l"){
    chute.lock();
    Serial.println("Locking Chute...");
  }
  else if(input == "o"){
    chute.deploy();
    Serial.println("Releasing Chute...");
  }
}
