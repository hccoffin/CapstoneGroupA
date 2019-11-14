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
#define outputA1 18
#define outputB1 19
#define outputA2 20
#define outputB2 21

volatile int counter1;
volatile int counter2;

// Declare IMU instance
Adafruit_BNO055 bno = Adafruit_BNO055(55);

void encoder1();
void encoder2();

void setup(void) 
{
  // Initialize PWM pins as outputs
  pinMode(pwm_1,OUTPUT);
  pinMode(dir_1,OUTPUT);
  pinMode(pwm_2,OUTPUT);
  pinMode(dir_2,OUTPUT);

  // ENCODER INIT
  pinMode(outputA1,INPUT);
  pinMode(outputB1,INPUT);
  pinMode(outputA2,INPUT);
  pinMode(outputB2,INPUT);

  //attachInterrupt(digitalPinToInterrupt(outputA1), encoder1, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(outputA2), encoder2, CHANGE);
    
  Serial.begin(9600);
  
  // ENCODER INIT PT 2
  counter1 = 0;
  counter2 = 0;
  
  delay(100);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  delay(1000);
    
  bno.setExtCrystalUse(true);

  
  //digitalWrite(pwm_1,HIGH);
  //digitalWrite(pwm_2,HIGH);
  }
  Serial.println("done setup");
    
}

void loop(void) 
{
  /* Get a new sensor event */ 
  Serial.println("In loop");
  sensors_event_t event; 
  bno.getEvent(&event);
  //Serial.print("Position1: ");
  //Serial.println(counter1);
                      
  
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
  //Serial.print("Position2: ");
  //Serial.println(counter2); 
  delay(10);
}
void encoder1() {
  Serial.println("1");
  if (digitalRead(outputB1) != digitalRead(outputA1)) { 
     counter1++;
     } else {
     counter1--;
  }
} 
void encoder2() {
  Serial.println("2");
  if (digitalRead(outputB2) != digitalRead(outputA2)) { 
     counter2++;
     } else {     
     counter2--;
  }
}
