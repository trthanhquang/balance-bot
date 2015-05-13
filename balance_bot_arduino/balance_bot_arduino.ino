#include "Wire.h"
#include "SPI.h"
#include "Mirf.h"
#include "nRF24L01.h"
#include "MirfHardwareSpiDriver.h"
#include "I2Cdev.h"
#include "MPU6050.h"

//#include "general_conf.h"
// I/O pins
#define TN1 2
#define TN2 3

#define TN3 7
#define TN4 8

#define ENA 5 //timer1
#define ENB 6 //timer1

#define SERVO 9 //timer0

//Generals
#define pi 3.14159 
#define DEBUG_MODE 1 // comment to deactivate

// System Parameters
#define Gry_offset 321
#define Gyr_Gain 0.00763358
#define Angle_offset -3.95 //2.7
#define RMotor_offset 18
#define LMotor_offset 20

#include <Servo.h> 
Servo myservo;
int servo_pos = 0;

// Sensors & Filter
MPU6050 accelgyro;
float f_angle=0;

// Control Loop
float kp = 30; //30;
float ki = 0.01 ;//0.09;
float kd = 700 ;//700;
#define ERR_SUM_MAX 200
float errSum, dErr, error, lastErr;
double pid_input, pid_output;

double pid_setpoint=-7.5;

// Loop Frequencies Control
unsigned long start_time;
unsigned long prev_control_ms = 0;
unsigned long control_period_ms = 5; //200 Hz
unsigned long prev_sensor_ms = 0;
unsigned long sensor_period_ms = 5; //200 Hz
unsigned long prev_remote_ms = 0;
unsigned long remote_period_ms = 200; //5 Hz

void setup() {
  Wire.begin();
  accelgyro.initialize();

  Serial.begin(115200);
  Serial.setTimeout(1);
  
  pinMode(TN1,OUTPUT);
  pinMode(TN2,OUTPUT);
  pinMode(TN3,OUTPUT);
  pinMode(TN4,OUTPUT);
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  myservo.attach(SERVO);
  
  //Loop Frequencies
  prev_control_ms = millis()+7;
  prev_sensor_ms  = millis();
  prev_remote_ms  = millis()+1;
}

void loop()
{  
  //Read & Filter sensor data
  if((millis() - prev_sensor_ms) >= sensor_period_ms) {
    start_time = millis();
    sensor_loop();
    prev_sensor_ms = start_time;
  }

  //Control
  if((millis() - prev_control_ms) >= control_period_ms) {
    start_time = millis();
    
    if(abs(f_angle)<=45){
      control_loop();
    } else{
      analogWrite(ENA, 0);
      analogWrite(ENB, 0);
    }
    prev_control_ms = start_time;
  }
  
  //Read Joystick command/serial command
  if((millis() - prev_remote_ms) >= remote_period_ms) {
    prev_remote_ms = millis();
    remote_loop();
  }
}

void control_loop(){
  #ifdef DEBUG_MODE
  unsigned long start_us = micros();
  #endif
  
  float dt = (start_time-prev_control_ms);
  //PID
  pid_input = f_angle;
  error = pid_input-pid_setpoint;
  errSum += error * dt;
  //limit errSum:
  errSum = min(ERR_SUM_MAX,errSum);
  errSum = max(-ERR_SUM_MAX,errSum);
  
  dErr = (error - lastErr) / dt;
  pid_output = kp * error + ki * errSum + kd * dErr;
  lastErr = error;

  float motor_cmd = pid_output;

  //send command
  send_motor_cmd(motor_cmd,motor_cmd);
  
  #ifdef DEBUG_MODE
//  Serial.print("  motor_cmd="); Serial.print(motor_cmd);
//  Serial.print("  dt_control="); Serial.print(dt);
//  Serial.print("  kp=");Serial.print(kp);
//  Serial.print("  kd=");Serial.print(kd);
//  Serial.print("  ki=");Serial.print(ki);
//  Serial.print("  errSum=");Serial.print(errSum);

//  int d = micros()-start_us;
  int d =start_time - prev_control_ms;
  Serial.print(" dtc=");Serial.print(d);
  #endif
}

void sensor_loop(){
  #ifdef DEBUG_MODE
  unsigned long start_us = micros();
  #endif
  
  //Read Accel & Gyro (1.9ms)
  int16_t ax, ay, az, gx, gy, gz;
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); //this one take 1.

  //convert to theta & theta_dot (0.2ms)
  float r_angle = (atan2(ay, az) * 180 / pi + Angle_offset);
  float omega =  Gyr_Gain * (gx +  Gry_offset);
  
  //1st order LPF (0.2ms)
  float dt = (start_time-prev_sensor_ms)/1000.0;//sec
  float K = 0.8;
  float A = K / (K + dt);
  f_angle = A * (f_angle + omega * dt) + (1 - A) * r_angle;


  #ifdef DEBUG_MODE
//  Serial.print("  omega="); Serial.print(omega);
//  Serial.print("  r_angle="); Serial.print(r_angle);
//  Serial.print("  f_angle=");Serial.print(f_angle);
//  Serial.print("  dt_sensor_loop="); Serial.print(start_time-prev_sensor_ms);
//  Serial.print("  dts="); Serial.print(micros()-start_us);

//  int d = micros()-start_us;
  int d =start_time - prev_sensor_ms;
  Serial.print("  dts="); Serial.print(d);
  Serial.println();
  #endif  
}

//--------------------------------
//    SUPPORTING FUNCTIONS
void send_motor_cmd(float LOutput, float ROutput){
  if(LOutput > 0){
    digitalWrite(TN1, HIGH);
    digitalWrite(TN2, LOW);
  }
  else if(LOutput < 0){
    digitalWrite(TN1, LOW);
    digitalWrite(TN2, HIGH);
  }
  else{
    digitalWrite(TN1, HIGH);
    digitalWrite(TN2, HIGH);
  }
  if(ROutput > 0){
    digitalWrite(TN3, HIGH);
    digitalWrite(TN4, LOW);
  }
  else if(ROutput < 0){   
    digitalWrite(TN3, LOW);
    digitalWrite(TN4, HIGH);
  }
  else{
    digitalWrite(TN3, HIGH);
    digitalWrite(TN4, HIGH);
  }
  analogWrite(ENA, min(255, abs(LOutput) + LMotor_offset));
  analogWrite(ENB, min(255, abs(ROutput) + RMotor_offset));
}

void read_pid_config(){
  if (Serial.available() > 3) {
    kp = Serial.parseFloat();
    Serial.print("Set kp: ");
    Serial.print(kp, DEC);
    ki = Serial.parseFloat();
    Serial.print("  ki: ");
    Serial.print(ki, DEC);
    kd = Serial.parseFloat();
    Serial.print(" kd: ");
    Serial.println(kd, DEC);
  }
}

void read_servo_config(){
  if (Serial.available() > 0) {
    servo_pos = Serial.parseFloat();
    Serial.print(" servo: ");
    Serial.println(servo_pos, DEC);
    myservo.write(servo_pos);
  }
}

void remote_loop(){
//  read_pid_config(); 
  read_servo_config();
}
