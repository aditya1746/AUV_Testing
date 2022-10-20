#include <Servo.h>
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

#include <Wire.h>
#include "MS5837.h"

MS5837 sensor;

std_msgs::String pub_msg;
std_msgs::Float32 depth_msg;
// std_msgs::String imu_msg;
// std_msgs::String pressure_msg;




Servo servoFrontRight;
Servo servoFrontLeft;
Servo servoMiddleRight;
Servo servoMiddleLeft;
Servo servoBackLeft;
Servo servoBackRight;

// ===== for Arduino Mega ===========
byte pin_fr = 7;
byte pin_fl = 5;
byte pin_mr = 4;
byte pin_ml = 6;
byte pin_br = 3;
byte pin_bl = 2;

ros::NodeHandle nh;

int pwm_fr=1500 ,pwm_fl=1500,pwm_mr=1500,pwm_ml=1500,pwm_br=1500,pwm_bl=1500;
String pwm_value="1500 1500 1500 1500 1500 1500";
// String imu_value="imu value";
// String pressure_value="pressure value";
char pwm[30]= "1500 1500 1500 1500 1500 1500";
// char imu_c[9]= "imu value";
// char pressure_c[14]= "pressure value";


void message7(const std_msgs::String& msg)
{
  //Serial.print("Inside Message7 ");
  //Serial.print(int(msg.data));
  pwm_value= msg.data;

  
  
  pwm_fr= (pwm_value.substring(0,4)).toInt();
  pwm_fl= (pwm_value.substring(5,9)).toInt();
  pwm_mr= (pwm_value.substring(10,14)).toInt();
  pwm_ml= (pwm_value.substring(15,19)).toInt();
  pwm_br= (pwm_value.substring(20,24)).toInt();
  pwm_bl= (pwm_value.substring(25)).toInt();
  
}


//ros::Publisher pub("Verify", &pub_msg);
ros::Publisher pub_depth("Depth", &depth_msg);
ros::Subscriber<std_msgs::String> sub("PWM_VALUE", &message7);

void setup()
{

//  nh.getHardware()->setBaud(/
  nh.initNode();
  nh.subscribe(sub);
  //nh.advertise(pub);
  nh.advertise(pub_depth);
  // nh.advertise(pub_imu);
  // nh.advertise(pub_pressure);
  
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

  Wire.begin();
//
//  while (!sensor.init()) {
//  
//    delay(5000);
//  }
  sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)

  
}

void MOVE()
{
  //servoFrontRight.write(pwm_fr);


  
  servoFrontRight.writeMicroseconds(pwm_fr);
  servoFrontLeft.writeMicroseconds(pwm_fl);
  servoMiddleRight.writeMicroseconds(pwm_mr);
  servoMiddleLeft.writeMicroseconds(pwm_ml);
  servoBackRight.writeMicroseconds(pwm_br);
  servoBackLeft.writeMicroseconds(pwm_bl);

    delay(10);
    pwm_value.toCharArray(pwm, 30);
    pub_msg.data = pwm;
    //pub.publish(&pub_msg);
}

//void IMU_DATA(){
//
//    imu_value.toCharArray(imu_c, 10);
//    imu_msg.data = imu_c;
//    pub_imu.publish(&imu_msg);
//
//
//}

void PRESSURE_DATA(){

  sensor.read();

  float pressure= sensor.pressure();
  float depth = sensor.depth();
 

   depth_msg.data = depth;
   pub_depth.publish(&depth_msg);
   delay(10);

//    pressure_value.toCharArray(pressure_c, 15);
//    pressure_msg.data = pressure_c;
//    pub_pressure.publish(&pressure_msg);

}


 
void loop()
{
//    Serial.println("Inside loop");
//    
     MOVE();

    // IMU_DATA();
    PRESSURE_DATA();


    


    


   nh.spinOnce();

}
