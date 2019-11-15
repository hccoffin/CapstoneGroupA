#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <BasicLinearAlgebra.h>
#include <Encoder.h>

// Setup PWM pin numbers for motors
#define dir_1 2
#define pwm_1 3
#define dir_2 5
#define pwm_2 6
#define outputA1 18
#define outputB1 19
#define outputA2 20
#define outputB2 21

// Declare IMU instance
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Create Encoder Objects
Encoder E1(18, 19);
Encoder E2(20, 21);

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
//  
//  /* Initialise the sensor */
//  if(!bno.begin())
//  {
//    /* There was a problem detecting the BNO055 ... check your connections */
//    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
//    while(1);
//  delay(1000);
//    
//  bno.setExtCrystalUse(true);
//
//  
//  //digitalWrite(pwm_1,HIGH);
//  //digitalWrite(pwm_2,HIGH);
//  }
    
}

void loop(void) 
{
  /* Get a new sensor event */ 
  sensors_event_t event; 
  bno.getEvent(&event);
                      
  /* Display the floating point data */
  /*Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);
  Serial.println("");*/

  /*
  double pitch = event.orientation.y/45.0; //mapped to -1 to 1
  int pitch_mapped_to_l_speed = max(min(255, (pitch+1)*128), 0);
  int pitch_mapped_to_r_speed = 255 - pitch_mapped_to_l_speed;
  
  analogWrite(dir_1, pitch_mapped_to_l_speed);
  analogWrite(dir_2, pitch_mapped_to_r_speed);
  */
  
  double pitch = event.orientation.y; // -180 to 180
  if (pitch < 0)
  {
    digitalWrite(dir_1, HIGH);
    digitalWrite(dir_2, HIGH);
  }
  else
  {
    digitalWrite(dir_1, LOW);
    digitalWrite(dir_2, LOW);
  }
  int mag = min(max(abs(pitch)*6, 0), 255);
  analogWrite(pwm_1, mag);
  analogWrite(pwm_2, mag);

  // Read Encoder Values
  long enc1 = E1.read();
  long enc2 = E2.read();

  Serial.print(enc1);
  Serial.print(", ");
  Serial.println(enc2);
  delay(10);
}
