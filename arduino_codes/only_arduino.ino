#include <Servo.h>
#include "ping1d.h"
#include <MPU6050_tockn.h>
// #include <SoftwareSerial.h>

#include <Wire.h>
#include "MS5837.h"

long timer = 0;
int start_var = 0;

MPU6050 mpu6050(Wire);
MS5837 sensor;
int pwmBase_r = 1480, pwmBase_l = 1480;

Servo servoFrontRight, servoFrontLeft, servoMiddleRight, servoMiddleLeft, servoBackLeft, servoBackRight;

// ===== for Arduino Mega ===========
byte pin_fr = 6, pin_fl = 4, pin_mr = 5, pin_ml = 3, pin_br = 7, pin_bl = 2;
byte arduinoRxPin_R = 19, arduinoTxPin_R = 18, arduinoRxPin_B = 17, arduinoTxPin_B = 16;

//SoftwareSerial Serial_Right = SoftwareSerial(arduinoRxPin_R, arduinoTxPin_R);
//SoftwareSerial Serial_Back = SoftwareSerial(arduinoRxPin_B, arduinoTxPin_B);

// static Ping1D ping_R { Serial1 };
// static Ping1D ping_B { Serial2 };
// int con_fr = 1, con_fl = 1, con_mr = 1, con_ml = 1, con_br = 1, con_bl = 1;

int pwm_fr = 1500 , pwm_fl = 1500, pwm_mr = 1500, pwm_ml = 1500, pwm_br = 1500, pwm_bl = 1500;

// double AccX, AccY, AccZ, GyroX, GyroY, GyroZ, AccAngleX, AccAngleY, GyroAngleX, GyroAngleY, GyroAngleZ, AngleX, AngleY, AngleZ;
// double distance_Right, confidence_Right, distance_Left, confidence_Left;

int depth_var = 0, roll_var = 0, yaw_var = 0, distr_var = 0, distb_var = 0, pitch_var;
float initial_depth, initial_yaw, initial_roll, initial_pitch, initial_dist_r;
float depth, roll, yaw, pitch;
float prev_err_y, int_err_y, prev_err_r, int_err_r, prev_err_d, int_err_d;
float KP_y = 0.0, KI_y = 0.0, KD_y = 0.0, yaw_desired;
float KP_d = 0.0, KI_d = 0.0, KD_d = 0.0;
float KP_r = 5.0, KI_r = 0.0, KD_r = 0.0, desired_roll;
int startval = 100, MINval = 90, MAXval = 120;
int val_fr = 100, val_fl = 100, val_br 100, val_bl = 100; 

//*******************************************************************
//bool ping_Right=ping_R.initialize();
//bool ping_Back=ping_B.initialize();

//*******************************************************************

void setup()
{
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
  Serial1.begin(9600); Serial2.begin(9600); Serial.begin(115200);

  Serial.println("Hellooooooooooooooooooooooooo");


  //  initialize sonar right
  //   while (!ping_R.initialize()) {
  //     Serial.println("\nPing Sonar Right device failed to initialize!"); //check sonar is running or not
  //     delay(2000);
  //   }
  //   while (!ping_B.initialize()) {
  //     Serial.println("\nPing Sonar Back device failed to initialize!");  //check sonar is running or not
  //     delay(2000);
  //   }

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
  servoFrontRight.writeMicroseconds(pwm_fr);
  servoFrontLeft.writeMicroseconds(pwm_fl);
  servoMiddleRight.writeMicroseconds(pwm_mr);
  servoMiddleLeft.writeMicroseconds(pwm_ml);
  servoBackRight.writeMicroseconds(pwm_br);
  servoBackLeft.writeMicroseconds(pwm_bl);
}

// =========== Pitch ========
void ANGLE_X() {
  mpu6050.update();
  AngleX = mpu6050.getAngleX();
  Serial.println(" angleX "); Serial.print(mpu6050.getAngleX());

  if(pitch_var==0)
  {
    pitch_var = 1;
    initial_pitch = pitch;
  }

  pitch = pitch - initial_pitch;
}

// =========== Roll ========
void ANGLE_Y() {
  mpu6050.update();
  Serial.println(" angleY "); Serial.print(mpu6050.getAngleY());
  AngleY = mpu6050.getAngleY();

  if(roll_var == 0)
  {
    roll_var = 1;
    initial_roll = AngleY;
  }

  roll = roll - initial_roll;
}

// =========== Yaw =========
void ANGLE_Z() {
  mpu6050.update();
  AngleZ = mpu6050.getAngleZ();
  Serial.println(" angleZ "); Serial.print(mpu6050.getAngleZ());

  if(yaw_var == 0)
  {
    yaw_var = 1;
    initial_yaw = AngleZ;
  }

  yaw = yaw - initial_yaw
}

//distance right confidence
void DIST_R() {

  confidence_Right = ping_R.confidence();
  Serial.println(" confidence_Right "); Serial.print(ping_R.confidence());
  distance_Right = ping_R.distance();
  Serial.println(" Sonar_distance_Right "); Serial.print(ping_R.distance());

  if(confidence_Right > 90)
  {
    if(distr_var == 0)
    {
      distr_var = 1;
      initial_dist_r = distance_Right;
    }
  }

  distance_Right = distance_Right - initial_dist_r;
}

//distance back confidence
void DIST_B() {

  confidence_Left = ping_B.confidence();
  Serial.println(" confidence_Back "); Serial.print(ping_B.confidence());
  distance_Left = ping_B.distance();
  Serial.println(" Sonar_distance_Back "); Serial.print(ping_B.distance());
}

//Sonar back confidence
void CON_B() {

  confidence_Left = ping_B.confidence();
  Serial.println(" confidence_Back "); Serial.print(ping_B.confidence());
}

//Sonar right confidence
void CON_R() {

  confidence_Right = ping_R.confidence();
  Serial.println(" confidence_Right "); Serial.print(ping_R.confidence());
}

void PRESSURE_DATA() {

  sensor.read();

  float pressure = sensor.pressure();
  Serial.println(" pressure "); Serial.print(sensor.pressure());
  depth = sensor.depth();
  Serial.println(" depth "); Serial.print(sensor.depth());

  if(depth_var == 0)
  {
    initial_depth = depth;
    depth_var = 1;
  }

  depth = depth - initial_depth;
}

void pid4straight()
{	
	float err_y = yaw - yaw_desired;
	float diff_err_y = err_y - prev_err_y;
	int_err_y += err_y; 
	prev_err_y = err_y;
        
	float correction_y = KP_y*err_y + KD_y*diff_err_y + KI_y*int_err_y;

  val_fr = 100; val_fl = 100; val_br = 100; val_bl = 100;
	
	val_fr -= int(correction_y);  // decreasing the force of front right, if err is +ve
	val_fl += int(correction_y);  // increasing the force of front left, if err is +ve
  val_br -= int(correction_y);
  val_bl += int(correction_y);

	val_fr = max(val_fr, MINval)
	val_fl = max(val_fl, MINval)
	val_br = max(val_br, MINval)
	val_bl = max(val_bl, MINval)
	val_fr = min(val_fr, MAXval)
	val_fl = min(val_fl, MAXval)
	val_br = min(val_br, MAXval)
	val_bl = min(val_bl, MAXval)

	if(val_fr<=MAXval && val_fl<=MAXval && val_br<=MAXval && val_bl<=MAXval)
  {
    pwm_fr = pwmBase_r - val_fr;
    pwm_fl = pwmBase_l + val_fl;
    pwm_br = pwmBase_r - val_br;
    pwm_bl = pwmBase_l + val_bl;
  }
}

void pid4Depth()
{	
	float err_d = desired_depth - depth;
	float diff_err_d = err_d - prev_err_d;
	int_err_d += err_d; 
	prev_err_d = err_d;
        
	float correction_d = KP_d*err_d + KD_d*diff_err_d + KI_d*int_err_d;

  float err_r = desired_roll - roll;
	float diff_err_r = err_r - prev_err_r;
	int_err_r += err_r; 
	prev_err_r = err_r;
        
	float correction_r = KP_r*err_r + KD_r*diff_err_r + KI_r*int_err_r;

  val_mr = 100; val_ml = 100; 
	
	val_mr += int(correction_d);  // decreasing the force of front right, if err is +ve
	val_ml += int(correction_d);  // increasing the force of front left, if err is +ve

  val_mr -= int(correction_r);  // decreasing the force of front right, if err is +ve
	val_ml += int(correction_r);

	val_mr = max(val_mr, MINval)
	val_ml = max(val_ml, MINval)
	val_mr = min(val_mr, MAXval)
	val_ml = min(val_ml, MAXval)

	if(val_mr<=MAXval && val_ml<=MAXval && val_mr<=MAXval && val_ml<=MAXval)
  {
    pwm_mr = pwmBase_r - val_mr;
    pwm_ml = pwmBase_l + val_ml;
  }
}
void loop()
{
  Serial.println("Inside loop");

  pid4straight();
  pid4Depth();

  MOVE();

  PRESSURE_DATA();

  delay(30);
  ANGLE_Y();
  ANGLE_Z();

  //DIST_R();
  //DIST_B();
  //CON_R();
  //CON_B();
  //delay(2000);
}
