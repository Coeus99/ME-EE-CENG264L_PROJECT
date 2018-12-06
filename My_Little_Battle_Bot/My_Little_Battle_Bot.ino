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
bool left_fired = false;
bool right_fired = false;

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

  //set to default values
  servo_driver.setPWM(0,0,300);
  servo_driver.setPWM(1,0,375);
  
  //set button as input
  pinMode(pin_button, INPUT);

  //set laser as output
  pinMode(pin_laser, OUTPUT);
  digitalWrite(pin_laser,LOW);

  //delete me
  Serial.begin(9600);
}

void loop()
{
  usb.Task();

  //if controller is connected, run
  if( ps4.connected() )
  {
    //check all applicable actions
    locomotion();
    weapons();
    panels();
  }
}

void locomotion()
{
  //some useful variables for setting speeds
  uint8_t right_y,left_y,right_speed,left_speed;
  bool left_reverse, right_reverse;
  
  //get analog stick input
  left_y = ps4.getAnalogHat(LeftHatY);
  right_y = ps4.getAnalogHat(RightHatY);
  
  //left side
  //idle case
  if(117 < left_y && left_y < 137)
  {
    //if the robot was previously idle, do nothing, else make it idle.
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
  //max speed case
  else if(left_y < 10)
  {
    //if the robot was previousy at max speed, do nothing, else make it so.
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
  //every other case
  //if the analog value has changed *significantly, the update the motors accordingly
  //if not, then don't change anything, the user won't notice ;)
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
  //for more information, see left side
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
      right_speed = map( right_y, 138, 255, 0, 255 );
      front_right->setSpeed(right_speed*speed_ratio);
      back_right->setSpeed(right_speed*speed_ratio);
	    front_right->run(BACKWARD);
      back_right->run(BACKWARD);
    }
    else if ( right_y < 117 )
    {
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
  //servo pulse information

  //square shoots left cannon
  if(ps4.getButtonClick(SQUARE))
  {
    if (!left_fired)
    {
      Serial.println("Firing");
      //to fire turn 30 degrees
      //(servo 0 should be on left side)
      servo_driver.setPWM(0,0,400);
      left_fired = true;
    }
    else
    {
      Serial.println("Reloading");
      //reload, turn it back.
      left_fired = false;
      servo_driver.setPWM(0,0,300);
    }
  }

  //circle shoots right cannon
  if(ps4.getButtonClick(CIRCLE))
  {
    if (!right_fired)
    {
      //to fire turn 30 degrees
      //(servo 0 should be on left side)
      servo_driver.setPWM(1,0,250);
      right_fired = true;
    }
    else
    {
      //reload, turn it back.
      right_fired = false;
      servo_driver.setPWM(1,0,375);
    }
  }

  //triangle fires laser for a second
  if(ps4.getButtonClick(TRIANGLE))
  {
    laser_is_on = true;
    laser_start_t = millis();
    digitalWrite(pin_laser, HIGH);
  }
  //if laser has been on or a second, turn it off
  if (laser_is_on && (millis() - laser_start_t > 1000))
  {
    digitalWrite(pin_laser, LOW);
    laser_is_on = false;
  }
  
  //if right_trigger -> laser on (maybe will add), psuedocode:
  //if (laser isn't controlled from triangle)
  //  if(trigger is down)
  //    turn on laser
  //  else
  //    turn off laser
  //(also use laser_is_on state so not digitalWriting constantly)
}

void panels()
{
  //if panels is hit decrease speed
  if (digitalRead(pin_button))
  {
    //got hit
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
      //no hits
      hits = 0;
      //restore speed
      speed_ratio = 1.0;
    }
  }
}
