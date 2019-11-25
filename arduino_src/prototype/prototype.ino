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
Encoder E1(18, 19);
//Encoder E2(20, 21);

PIDController pid_pitch;
PIDController pid_wheel;

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

  uint8_t sys, gyro, accel, mag;
  sys = gyro = accel = mag = 0;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  Serial.print("System calibration level: ");
  Serial.println(sys);
  Serial.print("Gyro calibration level: ");
  Serial.println(gyro);
  Serial.print("Accel calibration level: ");
  Serial.println(accel);
  Serial.print("Mag calibration level: ");
  Serial.println(mag);

  Serial.println("Setting calibration...");
  adafruit_bno055_offsets_t calib;
  calib.accel_offset_x = 0;
  calib.accel_offset_y = 0;
  calib.accel_offset_z = 0;
  calib.mag_offset_x = 0;
  calib.mag_offset_y = 0;
  calib.mag_offset_z = 0;
  calib.gyro_offset_x = 0;
  calib.gyro_offset_y = 0;
  calib.gyro_offset_z = 0;
  calib.accel_radius = 0;
  calib.mag_radius = 0;
  //bno.setSensorOffsets(calib);
  Serial.println("Didn't actually set calibration");

  delay(1000);
    
  bno.setExtCrystalUse(true);

  pid_pitch.begin();          // initialize the PID instance
  pid_pitch.setpoint(-0.15);    // The "goal" the PID controller tries to "reach"
  pid_pitch.tune(260, 2, 50);    // Tune the PID, arguments: kP, kI, kD
  pid_pitch.limit(-255, 255);    // limit to -255->255

  pid_wheel.begin();          // initialize the PID instance
  pid_wheel.setpoint(0);    // The "goal" the PID controller tries to "reach"
  pid_wheel.tune(0.3, 0, 0);    // Tune the PID, arguments: kP, kI, kD
  pid_wheel.limit(-30, 30);    // limit to -255->255
  
  Serial.println("Exiting setup");
    
}

/*
double kp_pitch = 70;
double kd_pitch = 2;
double set_pitch = -0.4;
double set_dpitch = 0;
*/

double setpoint_alpha = 0.0001;

void loop(void) 
{
  
  // Read Encoder
  long enc1 = E1.read();
            
  // Read IMU
  sensors_event_t orientationData , angVelocityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  //bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  
  //imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  double pitch = orientationData.orientation.z;

  int control_sig = pid_pitch.compute(pitch) + pid_wheel.compute(enc1);
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
  
  int mag = abs(control_sig);
  analogWrite(pwm_1, mag);
  analogWrite(pwm_2, mag);
}
