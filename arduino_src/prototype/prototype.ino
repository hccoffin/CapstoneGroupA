#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <BasicLinearAlgebra.h>
#include <Encoder.h>
#include <PIDController.h>

// Setup PWM pin numbers for motors
#define dir_1 2
#define pwm_1 3
#define dir_2 5
#define pwm_2 6

// Setup encoder pin numbers
#define outputA1 18
#define outputB1 19
#define outputA2 20
#define outputB2 21

// Declare IMU instance
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Create Encoder Objects
//Encoder E1(18, 19);
//Encoder E2(20, 21);

PIDController pid;
void setup(void) 
{
  // Initialize PWM pins as outputs
  pinMode(pwm_1,OUTPUT);
  pinMode(dir_1,OUTPUT);
  pinMode(pwm_2,OUTPUT);
  pinMode(dir_2,OUTPUT);
    
  Serial.begin(9600);
  
  delay(100);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);
    
  bno.setExtCrystalUse(true);

  pid.begin();          // initialize the PID instance
  pid.setpoint(0);    // The "goal" the PID controller tries to "reach"
  pid.tune(50, 0, 0);    // Tune the PID, arguments: kP, kI, kD
  pid.limit(-255, 255);    // limit to -255->255
  
  Serial.println("Exiting setup");
    
}

/*
double kp_pitch = 70;
double kd_pitch = 2;
double set_pitch = -0.4;
double set_dpitch = 0;

double kp_wheel = 0;//0.5;
double kd_wheel = 0;//20.0;
long set_wheel = 0.0;
double set_dwheel = 0;

long enc_last = 0;
long t_last = millis();
*/
void loop(void) 
{
 
  // Read Encoder
  //long enc1 = -E1.read();
  //long t = millis();
  //Serial.println(double(t - t_last));
  //long denc1 = double(enc1 - enc_last)/double(t - t_last);
            
  // Read IMU
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  //imu::Vector<3> angVel = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  
  double pitch = euler.z();
  //double dpitch = -angVel.x();
  //double control_sig_pitch = kp_pitch*(set_pitch - pitch) + kd_pitch*(set_dpitch - dpitch);
  //double control_sig_wheel = kp_wheel*(set_wheel - enc1) + kd_wheel*(set_dwheel - denc1);
  //Serial.println(pitch);
  //double control_sig = control_sig_pitch + control_sig_wheel;
  int control_sig = pid.compute(pitch);
  if (control_sig < 0)
  {
    digitalWrite(dir_1, HIGH);
    digitalWrite(dir_2, HIGH);
  }
  else
  {
    digitalWrite(dir_1, LOW);
    digitalWrite(dir_2, LOW);
  }
  
  //int mag = max(min(abs(control_sig), 255), 0);
  int mag = abs(control_sig);
  analogWrite(pwm_1, mag);
  analogWrite(pwm_2, mag);
  //enc_last = enc1;
  //t_last = t;
  //delay(10);
}
