#include <Servo.h>

byte back_left_pin = 6;
byte back_right_pin = 2;
Servo servo_left; 
Servo servo_right;

int signal_l = 1600; // Set signal value, which should be between 1100 and 1900
int signal_r = 1600;
void setup() {
  servo_left.attach(back_left_pin);
  servo_right.attach(back_right_pin);
  Serial.begin(9600);
  
//  servo_left.writeMicroseconds(1500); // send "stop" signal to ESC.
//  servo_right.writeMicroseconds(1500);
  delay(7000); // delay to allow the ESC to recognize the stopped signal
}

void loop() {
 
  Serial.print("signal to left ");
  Serial.println(signal_l);
  Serial.print("signal to right ");
  Serial.println(signal_r);
  servo_left.writeMicroseconds(signal_l); // Send signal to ESC.
  servo_right.writeMicroseconds(signal_r);
  delay(10);
  
//   if(signal_l<1700){
//     signal_l=signal_l +20;
//     signal_r=signal_r +20;
//   }else{
//     servo_left.writeMicroseconds(1500); // Send signal to ESC.
//     servo_right.writeMicroseconds(1500);
//     delay(5000);
  
//     signal_l=1300;
//     signal_r=1300;
//   }

}
