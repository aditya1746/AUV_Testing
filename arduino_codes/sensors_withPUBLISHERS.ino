#include <Servo.h>
#include "ping1d.h"
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <MPU6050_tockn.h>
// #include <SoftwareSerial.h>

#include <Wire.h>
#include "MS5837.h"

long timer = 0;

MPU6050 mpu6050(Wire);
MS5837 sensor;
int pwmBase_r = 1470, pwmBase_l = 1500;

std_msgs::String pub_msg;
std_msgs::Float32 depth_msg;
std_msgs::Float64 accx_msg;
std_msgs::Float64 accy_msg;
std_msgs::Float64 accz_msg;
std_msgs::Float64 gyrox_msg;
std_msgs::Float64 gyroy_msg;
std_msgs::Float64 gyroz_msg;
std_msgs::Float64 acc_angle_x_msg;
std_msgs::Float64 acc_angle_y_msg;
std_msgs::Float64 gyro_angle_x_msg;
std_msgs::Float64 gyro_angle_y_msg;
std_msgs::Float64 gyro_angle_z_msg;
std_msgs::Float64 angle_x_msg;
std_msgs::Float64 angle_y_msg;
std_msgs::Float64 angle_z_msg;

std_msgs::Float64 distance_r_msg;
std_msgs::Float64 distance_b_msg;
std_msgs::Float64 confidence_r_msg;
std_msgs::Float64 confidence_b_msg;
// std_msgs::String imu_msg;
// std_msgs::String pressure_msg;


Servo servoFrontRight;
Servo servoFrontLeft;
Servo servoMiddleRight;
Servo servoMiddleLeft;
Servo servoBackLeft;
Servo servoBackRight;

// ===== for Arduino Mega ===========
byte pin_fr = 6;
byte pin_fl = 4;
byte pin_mr = 5;
byte pin_ml = 3;
byte pin_br = 7;
byte pin_bl = 2;
byte arduinoRxPin_B = 19; //Serial1 rx
byte arduinoTxPin_B = 18; //Serial1 tx
byte arduinoRxPin_R = 17; //Serial1 rx
byte arduinoTxPin_R = 16; //Serial1 tx

//SoftwareSerial Serial_Right = SoftwareSerial(arduinoRxPin_R, arduinoTxPin_R);
//SoftwareSerial Serial_Back = SoftwareSerial(arduinoRxPin_B, arduinoTxPin_B);

ros::NodeHandle nh;
static Ping1D ping_R { Serial1 };
static Ping1D ping_B { Serial2 };

int pwm_fr = 1500 , pwm_fl = 1500, pwm_mr = 1500, pwm_ml = 1500, pwm_br = 1500, pwm_bl = 1500;
int con_fr = 1, con_fl = 1, con_mr = 1, con_ml = 1, con_br = 1, con_bl = 1;
String pwm_value = "1470 1470 1470 1470 1470 1470";
String pwm_value_car = "1470 1470 1470 1470 ";
String pwm_value_middle = "1470 1470 ";
double AccX, AccY, AccZ, GyroX, GyroY, GyroZ, AccAngleX, AccAngleY, GyroAngleX, GyroAngleY, GyroAngleZ, AngleX, AngleY, AngleZ;
double distance_Right, confidence_Right, distance_Back, confidence_Back;
// String imu_value="imu value";
// String pressure_value="pressure value";
char pwm[30] = "1470 1470 1470 1470 1470 1470";
// char imu_c[9]= "imu value";
// char pressure_c[14]= "pressure value";


void message7(const std_msgs::String& msg)
{
  //Serial.print("Inside Message7 ");
  //Serial.print(int(msg.data));
  pwm_value = msg.data;


  pwm_fr = (pwm_value.substring(0, 4)).toInt();
  pwm_fl = (pwm_value.substring(5, 9)).toInt();
  pwm_mr = (pwm_value.substring(10, 14)).toInt();
  pwm_ml = (pwm_value.substring(15, 19)).toInt();
  pwm_br = (pwm_value.substring(20, 24)).toInt();
  pwm_bl = (pwm_value.substring(25)).toInt();

}

void message_car(const std_msgs::String& msg)
{
  //Serial.print("Inside Message7 ");
  //Serial.print(int(msg.data));
  pwm_value_car = msg.data;


  pwm_fr = (pwm_value_car.substring(0, 4)).toInt();
  pwm_fl = (pwm_value_car.substring(5, 9)).toInt();
  pwm_br = (pwm_value_car.substring(10, 14)).toInt();
  pwm_bl = (pwm_value_car.substring(15, 19)).toInt();

}

void message_middle(const std_msgs::String& msg)
{
  //Serial.print("Inside Message7 ");
  //Serial.print(int(msg.data));
  pwm_value_middle = msg.data;

  pwm_mr = (pwm_value_middle.substring(0, 4)).toInt();
  pwm_ml = (pwm_value_middle.substring(5, 9)).toInt();

}


ros::Publisher pub_accx("accx", &accx_msg);
ros::Publisher pub_accy("accy", &accy_msg);
ros::Publisher pub_accz("accz", &accz_msg);
ros::Publisher pub_gyrox("gyrox", &gyrox_msg);
ros::Publisher pub_gyroy("gyroy", &gyroy_msg);
ros::Publisher pub_gyroz("gyroz", &gyroz_msg);
ros::Publisher pub_acc_angle_x("acc_angle_x", &acc_angle_x_msg);
ros::Publisher pub_acc_angle_y("acc_angle_y", &acc_angle_y_msg);
ros::Publisher pub_gyro_angle_x("gyro_angle_x", &gyro_angle_x_msg);
ros::Publisher pub_gyro_angle_y("gyro_angle_y", &gyro_angle_y_msg);
ros::Publisher pub_gyro_angle_z("gyro_angle_z", &gyro_angle_z_msg);
ros::Publisher pub_angle_x("angle_x", &angle_x_msg);
ros::Publisher pub_angle_y("angle_y", &angle_y_msg);
ros::Publisher pub_angle_z("angle_z", &angle_z_msg);

ros::Publisher pub_dist_r("dist_r", &distance_r_msg);
ros::Publisher pub_dist_b("dist_b", &distance_b_msg);
ros::Publisher pub_con_r("con_r", &confidence_r_msg);
ros::Publisher pub_con_b("con_b", &confidence_b_msg);

ros::Publisher pub("Verify", &pub_msg);
ros::Publisher pub_depth("Depth", &depth_msg);
ros::Subscriber<std_msgs::String> sub("PWM_VALUE", &message7);
ros::Subscriber<std_msgs::String> sub_car("PWM_VALUE_car", &message_car);
ros::Subscriber<std_msgs::String> sub_middle("PWM_VALUE_middle", &message_middle);

//*******************************************************************
//bool ping_Right=ping_R.initialize();
//bool ping_Back=ping_B.initialize();

//*******************************************************************

void setup()
{


  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(sub_car);
  nh.subscribe(sub_middle);
  nh.advertise(pub);
  nh.advertise(pub_depth);

  nh.advertise(pub_accx);
  nh.advertise(pub_accy);
  nh.advertise(pub_accz);
  nh.advertise(pub_gyrox);
  nh.advertise(pub_gyroy);
  nh.advertise(pub_gyroz);
  nh.advertise(pub_acc_angle_x);
  nh.advertise(pub_acc_angle_y);
  nh.advertise(pub_gyro_angle_x);
  nh.advertise(pub_gyro_angle_y);
  nh.advertise(pub_gyro_angle_z);
  nh.advertise(pub_angle_x);
  nh.advertise(pub_angle_y);
  nh.advertise(pub_angle_z);

  nh.advertise(pub_dist_r);
  nh.advertise(pub_dist_b);
  nh.advertise(pub_con_r);
  nh.advertise(pub_con_b);
  // nh.advertise(pub_imu);
  // nh.advertise(pub_pressure);

  servoFrontRight.attach(pin_fr);
  servoFrontLeft.attach(pin_fl);
  servoMiddleRight.attach(pin_mr);
  servoMiddleLeft.attach(pin_ml);
  servoBackRight.attach(pin_br);
  servoBackLeft.attach(pin_bl);

  servoFrontRight.writeMicroseconds(pwmBase_r);
  servoFrontLeft.writeMicroseconds(pwmBase_l);
  servoMiddleRight.writeMicroseconds(pwmBase_r);
  servoMiddleLeft.writeMicroseconds(pwmBase_l);
  servoBackRight.writeMicroseconds(pwmBase_r);
  servoBackLeft.writeMicroseconds(pwmBase_l);

  delay(7000); // delay to allow the ESC to recognize the stopped signal<br />

  Wire.begin();
  Serial1.begin(9600);
  Serial2.begin(9600);
  Serial.begin(115200);   //ek bar dekh lena sonar 115200 braud pe chal raha h and imu 9600 braud rate pe chalta h

  Serial.println("Hellooooooooooooooooooooooooo");


  //  initialize sonar right

//  while (!ping_B.initialize()) {
//    Serial.println("\nPing Sonar Back device failed to initialize!");  //check sonar is running or not
//    delay(2000);
//  }

while (!ping_R.initialize()) {
   Serial.println("\nPing Sonar Right device failed to initialize!");  //check sonar is running or not
   delay(2000);
 }

  while (!sensor.init()) {
    Serial.println("\nPressure device failed to initialize!");
   delay(5000);
   }
  sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)

  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
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


  pwm_value.toCharArray(pwm, 30);
  pub_msg.data = pwm;
  pub.publish(&pub_msg);
}

/*
void ACCX() {
  mpu6050.update();
  AccX = mpu6050.getAccX();
  Serial.println(" accX "); Serial.print(mpu6050.getAccX());
  accx_msg.data = AccX;
  pub_accx.publish(&accx_msg);

}
void ACCY() {
  mpu6050.update();
  AccY = mpu6050.getAccY();
  Serial.println(" accY "); Serial.print(mpu6050.getAccY());
  accy_msg.data = AccY;
  pub_accy.publish(&accy_msg);

}
void ACCZ() {
  mpu6050.update();
  AccZ = mpu6050.getAccZ();
  Serial.println(" accZ "); Serial.print(mpu6050.getAccZ());
  accz_msg.data = AccZ;
  pub_accz.publish(&accz_msg);

}
void GYROX() {
  mpu6050.update();
  GyroX = mpu6050.getGyroX();
  Serial.println(" gyroX "); Serial.print(mpu6050.getGyroX());
  gyrox_msg.data = GyroX;
  pub_gyrox.publish(&gyrox_msg);

}
void GYROY() {
  mpu6050.update();
  GyroY = mpu6050.getGyroY();
  Serial.println(" gyroY "); Serial.print(mpu6050.getGyroY());
  gyroy_msg.data = GyroY;
  pub_gyroy.publish(&gyroy_msg);

}
void GYROZ() {
  mpu6050.update();
  GyroZ = mpu6050.getGyroZ();
  Serial.println(" gyroZ "); Serial.print(mpu6050.getGyroZ());
  gyroz_msg.data = GyroZ;
  pub_gyroz.publish(&gyroz_msg);

}
void ACC_ANGLE_X() {
  mpu6050.update();
  AccAngleX = mpu6050.getAccAngleX();
  Serial.println(" accAngleX "); Serial.print(mpu6050.getAccAngleX());
  acc_angle_x_msg.data = AccAngleX;
  pub_acc_angle_x.publish(&acc_angle_x_msg);

}
void ACC_ANGLE_Y() {
  mpu6050.update();
  AccAngleY = mpu6050.getAccAngleY();
  Serial.println(" accAngleY "); Serial.print(mpu6050.getAccAngleY());
  acc_angle_y_msg.data = AccAngleY;
  pub_acc_angle_y.publish(&acc_angle_y_msg);

}
void GYRO_ANGLE_X() {
  mpu6050.update();
  GyroAngleX = mpu6050.getGyroAngleX();
  Serial.println(" gyroAngleX "); Serial.print(mpu6050.getGyroAngleX());
  gyro_angle_x_msg.data = GyroAngleX;
  pub_gyro_angle_x.publish(&gyro_angle_x_msg);

}
void GYRO_ANGLE_Y() {
  mpu6050.update();
  GyroAngleY = mpu6050.getGyroAngleY();
  Serial.println(" gyroAngleY "); Serial.print(mpu6050.getGyroAngleY());
  gyro_angle_y_msg.data = GyroAngleY;
  pub_gyro_angle_y.publish(&gyro_angle_y_msg);

}
void GYRO_ANGLE_Z() {
  mpu6050.update();
  GyroAngleZ = mpu6050.getGyroAngleZ();
  Serial.println(" gyroAngleZ "); Serial.print(mpu6050.getGyroAngleZ());
  gyro_angle_z_msg.data = GyroAngleZ;
  pub_gyro_angle_z.publish(&gyro_angle_z_msg);

}
*/
void ANGLE_X() {
  mpu6050.update();
  AngleX = mpu6050.getAngleX();
  // Serial.println(" angleX "); Serial.print(mpu6050.getAngleX());
  angle_x_msg.data = AngleX;
  pub_angle_x.publish(&angle_x_msg);

}
void ANGLE_Y() {
  mpu6050.update();
  // Serial.println(" angleY "); Serial.print(mpu6050.getAngleY());
  AngleY = mpu6050.getAngleY();
  angle_y_msg.data = AngleY;
  pub_angle_y.publish(&angle_y_msg);

}
void ANGLE_Z() {
  mpu6050.update();
  AngleZ = mpu6050.getAngleZ();
  // Serial.println(" angleZ "); Serial.print(mpu6050.getAngleZ());
  angle_z_msg.data = AngleZ;
  pub_angle_z.publish(&angle_z_msg);
  // Serial.println(AngleZ);
}

//distance right confidence
void DIST_R() {

  if (ping_R.update()) {
    Serial.print("Distance Right: ");
    distance_Right = ping_R.distance();
    Serial.print(distance_Right);
    confidence_Right = ping_R.confidence();
    Serial.print("\tConfidence: ");
    Serial.println(confidence_Right);

    distance_r_msg.data = distance_Right;
    pub_dist_r.publish(&distance_r_msg);
    confidence_r_msg.data = confidence_Right;
    pub_con_r.publish(&confidence_r_msg);
  } else {
    Serial.println("No update received!");
  }

  
  // Serial.print(" Sonar_distance_Right "); 
  // Serial.println(distance_Right);
  


}

//distance back confidence
void DIST_B() {

  if (ping_B.update()) {
    Serial.print("Distance Back: ");
    distance_Back = ping_B.distance();
    Serial.print(distance_Back);
    confidence_Back = ping_B.confidence();
    Serial.print("\tConfidence: ");
    Serial.println(confidence_Back);

    distance_b_msg.data = distance_Back;
    pub_dist_b.publish(&distance_b_msg);
    confidence_b_msg.data = confidence_Back;
    pub_con_b.publish(&confidence_b_msg);
  } else {
    Serial.println("No update received!");
  }

  
  // Serial.println(" Sonar_distance_Back "); Serial.print(ping_B.distance());
  


}
/*
//Sonar back confidence
void CON_B() {

  confidence_Back = ping_B.confidence();
  Serial.println(" confidence_Back "); Serial.print(ping_B.confidence());
  confidence_b_msg.data = confidence_Back;
  pub_con_b.publish(&confidence_b_msg);


}

//Sonar right confidence
void CON_R() {

  confidence_Right = ping_R.confidence();
  Serial.println(" confidence_Right "); Serial.print(ping_R.confidence());
  confidence_r_msg.data = confidence_Right;
  pub_con_r.publish(&confidence_r_msg);


}
*/
/*void IMU_DATA(){

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

  }*/

void PRESSURE_DATA() {

  if(sensor.init())
  {
    sensor.read();
    float pressure = sensor.pressure();
    // Serial.println(" pressure "); Serial.print(sensor.pressure());
    float depth = sensor.depth();
    // Serial.println(" depth "); Serial.print(depth);


    depth_msg.data = depth;
    pub_depth.publish(&depth_msg);
  }

  //    pressure_value.toCharArray(pressure_c, 15);
  //    pressure_msg.data = pressure_c;
  //    pub_pressure.publish(&pressure_msg);

}

/*
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
*/


void loop()
{
  // Serial.println("Inside loop");

  MOVE();

  // IMU_DATA();
  // Serial.println('Stopped 0');
  PRESSURE_DATA();
  //    ACCX();
    //  ACCY();
  //    ACCZ();
  //    GYROX();
  //    GYROY();
  //    GYROZ();
  //    ACC_ANGLE_X();
    //  ACC_ANGLE_Y();
  //    GYRO_ANGLE_X();
  //    GYRO_ANGLE_Y();
  //    GYRO_ANGLE_Z();

   delay(30);
  //  servoFrontRight.writeMicroseconds(pwmBase_r);
  //  servoFrontLeft.writeMicroseconds(pwmBase_l);
//   servoMiddleRight.writeMicroseconds(pwmBase);
//   servoMiddleLeft.writeMicroseconds(pwmBase);
  //  servoBackRight.writeMicroseconds(pwmBase_r);
  //  servoBackLeft.writeMicroseconds(pwmBase_l);
      // Serial.println('Stopped 1');
      ANGLE_X();
      ANGLE_Y();

  // Serial.println('Stopped 2');
  ANGLE_Z();

  // PRESSURE_DATA();


  //SONAR_DATA();
  DIST_R();
  // CON_R();
  // DIST_B();
  
  //CON_B();


  //delay(2000);


  nh.spinOnce();

}
