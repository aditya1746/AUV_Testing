#include <Servo.h>
#include "ping1d.h"
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <MPU6050_tockn.h>
//#include <SoftwareSerial.h>

#include <Wire.h>
#include "MS5837.h"

long timer = 0;

MPU6050 mpu6050(Wire);
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
byte arduinoRxPin_R = 19; //Serial1 rx
byte arduinoTxPin_R = 18; //Serial1 tx
byte arduinoRxPin_B = 17; //Serial1 rx
byte arduinoTxPin_B = 16; //Serial1 tx

//SoftwareSerial Serial_Right = SoftwareSerial(arduinoRxPin_R, arduinoTxPin_R);
//SoftwareSerial Serial_Back = SoftwareSerial(arduinoRxPin_B, arduinoTxPin_B);


//Serial 1 is in indefine serial communication pins (19(RX), 18(TX)) && Serial 2 is in indefine serial communication pins (17(RX), 16(TX))
ros::NodeHandle nh;
static Ping1D ping_R { Serial1 };
static Ping1D ping_B { Serial2 };

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


ros::Publisher pub("Verify", &pub_msg);
ros::Publisher pub_depth("Depth", &depth_msg);
ros::Subscriber<std_msgs::String> sub("PWM_VALUE", &message7);

void setup()
{

  
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
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
  Serial1.begin(9600);
  Serial2.begin(9600);
  Serial.begin(115200);   //ek bar dekh lena sonar 115200 braud pe chal raha h and imu 9600 braud rate pe chalta h

  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

//  //initialize sonar right
//  while (!ping_R.initialize()) {
//    Serial.println("\nPing Sonar Right device failed to initialize!");
//    delay(2000);
//  }
//  while (!ping_B.initialize()) {
//    Serial.println("\nPing Sonar Back device failed to initialize!");
//    delay(2000);
//  }


  

  
  while (!sensor.init()) {
    Serial.println("\nPressure device failed to initialize!");
    delay(5000);
  }
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
    pub.publish(&pub_msg);
}

void IMU_DATA(){

      mpu6050.update();
      
//    imu_value.toCharArray(imu_c, 10);
//    imu_msg.data = imu_c;
//    pub_imu.publish(&imu_msg);

      float accX=mpu6050.getAccX();
      Serial.println(" accX "); Serial.print(mpu6050.getAccX());
      float accY=mpu6050.getAccY();
      Serial.println(" accY "); Serial.print(mpu6050.getAccY());
      float accZ=mpu6050.getAccZ();
      Serial.println(" accZ "); Serial.print(mpu6050.getAccZ());

      float gyroX=mpu6050.getGyroX();
      Serial.println(" gyroX "); Serial.print(mpu6050.getGyroX());
      float gyroY=mpu6050.getGyroY();
      Serial.println(" gyroY "); Serial.print(mpu6050.getGyroY());
      float gyroZ=mpu6050.getGyroZ();
      Serial.println(" gyroZ "); Serial.print(mpu6050.getGyroZ());

      float accAngleX=mpu6050.getAccAngleX();
      Serial.println(" accAngleX "); Serial.print(mpu6050.getAccAngleX());
      float accAngleY=mpu6050.getAccAngleY();
      Serial.println(" accAngleY "); Serial.print(mpu6050.getAccAngleY());

      float gyroAngleX=mpu6050.getGyroAngleX();
      Serial.println(" gyroAngleX "); Serial.print(mpu6050.getGyroAngleX());
      float gyroAngleY=mpu6050.getGyroAngleY();
      Serial.println(" gyroAngleY "); Serial.print(mpu6050.getGyroAngleY());
      float gyroAngleZ=mpu6050.getGyroAngleZ();
      Serial.println(" gyroAngleZ "); Serial.print(mpu6050.getGyroAngleZ());

      float angleX=mpu6050.getAngleX();
      Serial.println(" angleX "); Serial.print(mpu6050.getAngleX());
      float angleY=mpu6050.getAngleY();
      Serial.println(" angleY "); Serial.print(mpu6050.getAngleY());
      float angleZ=mpu6050.getAngleZ();
      Serial.println(" angleZ "); Serial.print(mpu6050.getAngleZ());

}

void PRESSURE_DATA(){

  sensor.read();

  float pressure= sensor.pressure();
  Serial.println(" pressure "); Serial.print(sensor.pressure());
  float depth = sensor.depth();
  Serial.println(" depth "); Serial.print(sensor.depth());
 

   depth_msg.data = depth;
   pub_depth.publish(&depth_msg);
   delay(10);

//    pressure_value.toCharArray(pressure_c, 15);
//    pressure_msg.data = pressure_c;
//    pub_pressure.publish(&pressure_msg);

}

void SONAR_DATA(){
    //Right sonar values
    float distance_Right = ping_R.distance();
    Serial.println(" Sonar_distance_Right "); Serial.print(ping_R.distance());
    float confidence_Right = ping_R.confidence();
    Serial.println(" confidence_Right "); Serial.print(ping_R.confidence());

    //Back sonar values
    float distance_Back = ping_B.distance();
    Serial.println(" Sonar_distance_Back "); Serial.print(ping_B.distance());
    float confidence_Back = ping_B.confidence();
    Serial.println(" confidence_Back "); Serial.print(ping_B.confidence());
  }


 
void loop()
{
    Serial.println("Inside loop");
    
     MOVE();

     IMU_DATA();
    PRESSURE_DATA();


    SONAR_DATA();

  delay(10000);
    


   nh.spinOnce();

}
