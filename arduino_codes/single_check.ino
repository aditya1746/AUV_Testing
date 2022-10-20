#include <Servo.h>


Servo servoFrontRight;
Servo servoFrontLeft;
Servo servoMiddleRight;
Servo servoMiddleLeft;
Servo servoBackLeft;
Servo servoBackRight;

// ===== for Arduino Mega ===========
byte pin_fr = 4;
byte pin_fl = 6;
byte pin_mr = 5;
byte pin_ml = 3;
byte pin_br = 7;
byte pin_bl = 2;

int pwm_fr=1550 ,pwm_fl=1550,pwm_mr=1550,pwm_ml=1550,pwm_br=1550,pwm_bl=1550;


void setup()
{
  
  servoFrontRight.attach(pin_fr);
  servoFrontLeft.attach(pin_fl);
  servoMiddleRight.attach(pin_mr);
  servoMiddleLeft.attach(pin_ml);
  servoBackRight.attach(pin_br);
  servoBackLeft.attach(pin_bl);
  
  servoFrontRight.writeMicroseconds(1500);
  servoFrontLeft.writeMicroseconds(1500);
  servoMiddleRight.writeMicroseconds(1500);
  servoMiddleLeft.writeMicroseconds(1500);
  servoBackRight.writeMicroseconds(1500);
  servoBackLeft.writeMicroseconds(1500);
  
  delay(7000); // delay to allow the ESC to recognize the stopped signal<br />
}

void MOVE()
{
  servoFrontRight.writeMicroseconds(pwm_fr);
//   servoFrontLeft.writeMicroseconds(pwm_fl);
//   servoMiddleRight.writeMicroseconds(pwm_mr);
//   servoMiddleLeft.writeMicroseconds(pwm_ml);
//   servoBackRight.writeMicroseconds(pwm_br);
//   servoBackLeft.writeMicroseconds(pwm_bl);

    delay(1000);
}

 
void loop()
{
    Serial.println("Inside loop");
    
    MOVE();

    delay(100);

}
