//Libraries for PS4
#include <PS4BT.h>
#include <usbhub.h>

//Don't know why this is needed, but I'm leaving it in here.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#include <SPI.h>
#endif

//Motor Libraries
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

//LCD Libraries
#include <Wire.h>

//Servo Library and info
#include <Adafruit_PWMServoDriver.h>
#define SERVOMIN 200//don't remember what was said in class
#define SERVOMAX 400//don't remember what was said in class
const uint8_t min_position = 0;
const uint8_t max_position = 100;
Adafruit_PWMServoDriver servo_driver = Adafruit_PWMServoDriver();

//Motors
/**
 * (0)-front-(1)
 *    l     r
 *    e     i
 *    f     g
 *    t     h
 *    |     t
 * (2)-back-(3)
 */

Adafruit_MotorShield motor_driver = Adafruit_MotorShield();
Adafruit_DCMotor *front_left = motor_driver.getMotor( 1 );
Adafruit_DCMotor *front_right = motor_driver.getMotor( 2 );
Adafruit_DCMotor *back_left = motor_driver.getMotor( 3 );
Adafruit_DCMotor *back_right = motor_driver.getMotor( 4 );

//create USB instance
USB usb;
//Create BT instance from USB instance
BTD btd( &usb );
//Create DS4 instance from BT instance
PS4BT ps4( &btd );

/**
 * Pin assignments
 */
//button
const uint8_t pin_button = 3;
//laser
const uint8_t pin_laser = 4;

/**
 * Prototypes
 */
void locomotion();
void weapons();
void panels();

/**
 * Flags
 */
bool left_idle = false;
bool right_idle = false;
bool left_max = false;
bool right_max = false;

/**
 * Previous Values
 */
uint8_t prev_right_y = 0;
uint8_t prev_left_y = 0;
uint8_t prev_right_speed;
uint8_t prev_left_speed;

/**
 * Panels
 */
uint8_t hits = 0;
unsigned long long int prev_hit_t;
float speed_ratio = 1.0;

/**
 * Laser
 */ 
bool laser_is_on = false;
unsigned long long int laser_start_t;

/**
 * Cannons
 */
unsigned long long int right_servo_last_t;
unsigned long long int left_servo_last_t;

void setup()
{
  //Fix potential manufacturing defect on Sparkfun USB Hub:
  pinMode( 7, OUTPUT );
  digitalWrite( 7, HIGH );  

  //Halt if usb hasn't started
  if(usb.Init() == -1)
  {
    while(true){};
  }

  //start motor driver
  motor_driver.begin();

  //start servo driver
  servo_driver.begin();
  servo_driver.setPWMFreq(60);
  
  //set button as input
  pinMode(pin_button, INPUT);

  //set laser as output
  pinMode(pin_laser, OUTPUT);
  digitalWrite(pin_laser,LOW);
}

void loop()
{
  usb.Task();

  //if controller is connected, run
  if( ps4.connected() )
  {
    //check all applicable controls
    locomotion();
    weapons();
    panels();
  }
}

void locomotion()
{
  //get controller data
  uint8_t right_y,left_y,right_speed,left_speed;
  bool left_reverse, right_reverse;

  left_y = ps4.getAnalogHat(LeftHatY);
  right_y = ps4.getAnalogHat(RightHatY);
  
  //left side
  if(117 < left_y && left_y < 137)
  {
    if(!left_idle)
    {
      left_idle = true;
      left_max = false;
      left_speed = 0;
      prev_left_speed = 0;
      prev_left_y = 128;
      front_left->run(RELEASE);
      back_left->run(RELEASE);
    }
  }
  else if(left_y < 10)
  {
    if(!left_max)
    {
      left_max = true;
      left_idle = false;
      left_speed = 255;
      prev_left_speed = 255;
      front_left->setSpeed(left_speed*speed_ratio);
      back_left->setSpeed(left_speed*speed_ratio);
      prev_left_y = 0;
      if (left_y < 5)
      {
        front_left->run(FORWARD);
        back_left->run(FORWARD);
      }
    }
  }
  else if(abs((int)left_y - (int)prev_left_y) > 10)
  {
    left_idle = false;
    left_max = false;
    if ( left_y > 137 )
    {
      //Left side is in reverse
      left_speed = map( left_y, 138, 255, 0, 255 );
      front_left->setSpeed(left_speed*speed_ratio);
      back_left->setSpeed(left_speed*speed_ratio);
	    front_left->run(BACKWARD);
      back_left->run(BACKWARD);
    }
    else if ( left_y < 117 )
    {
      //Left side is forward
      left_speed = map( left_y, 116, 0, 0, 255 );
      front_left->setSpeed(left_speed*speed_ratio);
      back_left->setSpeed(left_speed*speed_ratio);
	    front_left->run(FORWARD);
      back_left->run(FORWARD);
    }
    prev_left_y = left_y;
    prev_left_speed = left_speed;
  }
  prev_left_speed = left_speed;
  //right side
  if(117 < right_y && right_y < 137)
  {
    if(!right_idle)
    {
      right_idle = true;
      right_max = false;
      right_speed = 0;
      prev_right_speed = 0;
      prev_right_y = 128;
      front_right->run(RELEASE);
      back_right->run(RELEASE);
    }
  }
  else if(right_y < 10)
  {
    if(!right_max)
    {
      right_max = true;
      right_idle = false;
      right_speed = 255;
      prev_right_speed = 255;
      prev_right_y = 0;
      front_right->setSpeed(right_speed*speed_ratio);
      back_right->setSpeed(right_speed*speed_ratio);
      if (right_y < 5)
      {
        front_right->run(FORWARD);
        back_right->run(FORWARD);
      }
    }
  }
  else if(abs((int)right_y - (int)prev_right_y) > 10)
  {
    right_idle = false;
    right_max = false;
    if ( right_y > 137 )
    {
      //right side is in reverse
      right_speed = map( right_y, 138, 255, 0, 255 );
      front_right->setSpeed(right_speed*speed_ratio);
      back_right->setSpeed(right_speed*speed_ratio);
	    front_right->run(BACKWARD);
      back_right->run(BACKWARD);
    }
    else if ( right_y < 117 )
    {
      //right side is forward
      right_speed = map( right_y, 116, 0, 0, 255 );
      front_right->setSpeed(right_speed*speed_ratio);
      back_right->setSpeed(right_speed*speed_ratio);
	    front_right->run(FORWARD);
      back_right->run(FORWARD);
    }
    prev_right_y = right_y;
    prev_right_speed = right_speed;
  }
}

void weapons()
{
  uint8_t pulse_up = map(30, min_position, max_position, SERVOMIN, SERVOMAX);
  uint8_t pulse_down = map(0, min_position, max_position, SERVOMIN, SERVOMAX);
  //if square -> shoot left cannon
  if(ps4.getButtonClick(SQUARE))
  {
    //to fire -> 60-90
    servo_driver.setPWM(0,0,pulse_up);
    left_servo_last_t = millis();
  }
  if((millis() - left_servo_last_t ) > 1000)
  {
    servo_driver.setPWM(0,0,pulse_down);
  }
  //if circle -> shoot left cannon
  if(ps4.getButtonClick(CIRCLE))
  {
    //to fire -> 60-90
    servo_driver.setPWM(1,0,pulse_down);
    right_servo_last_t = millis();
  }
  if((millis() - right_servo_last_t ) > 1000)
  {
    servo_driver.setPWM(1,0,pulse_up);
  }
  //if triangle -> shoot laser for 1 second (use millis() timers), use flag
  if(ps4.getButtonClick(TRIANGLE))
  {
    laser_is_on = true;
    laser_start_t = millis();
    digitalWrite(pin_laser, LOW);
  }
  //turn laser off if necessary
  if (laser_is_on && (millis() - laser_start_t > 1000))
  {
    digitalWrite(pin_laser, LOW);
    laser_is_on = false;
  }
  
  //if right_trigger -> laser on
}

void panels()
{
  //if panels is hit decrease speed
  if (digitalRead(pin_button))
  {
    hits = 1;
    //cut speed
    speed_ratio = 0.5;
    //reset speeds
    front_right->setSpeed(prev_right_speed*speed_ratio);
    back_right->setSpeed(prev_right_speed*speed_ratio);
    front_left->setSpeed(prev_left_speed*speed_ratio);
    back_left->setSpeed(prev_left_speed*speed_ratio);
    //reset time
    prev_hit_t = millis();
  }
  
  //if there are hits on the vehicle
  if (hits > 0)
  {
    //if 5 seconds have passed
    if ((millis() - prev_hit_t) > 5000)
    {
      hits = 0;
      //restore speed
      speed_ratio = 1.0;
    }
  }
}
