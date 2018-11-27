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
const uint8_t min_position = 0;//don't remember what was said in class
const uint8_t max_position = 100;//don't remember what was said in class, should be multiple of 10
Adafruit_PWMServoDriver servo_driver = Adafruit_PWMServoDriver();

//Prototypes
void set_motors( uint8_t left_speed, bool left_reverse, uint8_t right_speed, bool right_reverse);

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

void setup()
{
  Serial.begin(9600);
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
}

void loop()
{
  usb.Task();

  //if controller is connected, run
  if( ps4.connected() )
  {
    //check all applicable controls
  }
}
