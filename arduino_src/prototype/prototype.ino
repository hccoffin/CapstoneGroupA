#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <BasicLinearAlgebra.h>

// Setup PWM pin numbers for motors
#define dir_1 2
#define pwm_1 3
#define dir_2 5
#define pwm_2 6

// Declare IMU instance
Adafruit_BNO055 bno = Adafruit_BNO055(55);

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
  Serial.println("Checking sensor");
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  Serial.println("Post check");
  delay(1000);
    
  bno.setExtCrystalUse(true);
  Serial.println("Exiting setup");
}


double kp = 40;
double kd = 2;
double pitch_set = 0;
double dpitch_set = 0;

void loop(void) 
{             

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> angVel = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  
  double pitch = euler.z();
  double dpitch = -angVel.x();

  double control_sig = kp*(pitch_set - pitch) + kd*(dpitch_set - dpitch);
  
  
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
  
  int mag = max(min(abs(control_sig), 255), 0);
  Serial.println(mag);
  analogWrite(pwm_1, mag);
  analogWrite(pwm_2, mag);
  
  //delay(10);
}
