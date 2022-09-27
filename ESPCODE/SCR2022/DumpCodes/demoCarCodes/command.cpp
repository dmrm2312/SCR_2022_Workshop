#include <Adafruit_PWMServoDriver.h>
#include "command.h"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void CommandServo::setupServo()
{
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ);  
}

//pwm.setPWM(servochannel, on, off) / pwm.writeMicroseconds(channel, micros)
//TREAT MIN AS CW AND MAX AS CCW

/*
 * The servo channel will depend on how the servos are setup in the car (using Adafruit PWM/Servo Motor FeatherWing)
 * Below is the current setup for this code
           FRONT
    servo1      servo0

    servo2      Servo3
           REAR

*/

//Move the car forward
void CommandServo::forward(){
  pwm.setPWM(1, 0, SERVOMAXPWM);
  pwm.setPWM(0, 0, SERVOMINPWM);
  pwm.setPWM(4, 0, SERVOMAXPWM);
  pwm.setPWM(5, 0, SERVOMINPWM);
}


//Move the car backward
void CommandServo::backward(){
  pwm.setPWM(0, 0, SERVOMAXPWM);
  pwm.setPWM(1, 0, SERVOMINPWM);
  pwm.setPWM(5, 0, SERVOMAXPWM);
  pwm.setPWM(4, 0, SERVOMINPWM);
}

void CommandServo::left(){
  pwm.setPWM(0, 0, midpoint[0] + (SERVOMINPWM - midpoint[0]));
  pwm.setPWM(1, 0, midpoint[1] + (SERVOMINPWM - midpoint[1]));
  pwm.setPWM(2, 0, midpoint[2] + (SERVOMINPWM - midpoint[2]));
  pwm.setPWM(3, 0, midpoint[3] + (SERVOMINPWM - midpoint[3]));
}

void CommandServo::right(){
  pwm.setPWM(0, 0, midpoint[0] + (SERVOMAXPWM - midpoint[0]));
  pwm.setPWM(1, 0, midpoint[1] + (SERVOMAXPWM - midpoint[1]));
  pwm.setPWM(2, 0, midpoint[2] + (SERVOMAXPWM - midpoint[2]));
  pwm.setPWM(3, 0, midpoint[3] + (SERVOMAXPWM - midpoint[3]));
}

//Stops the car (MIDPOINT can also be used)
void CommandServo::stopS(){
  pwm.setPWM(1, 0, 0);
  pwm.setPWM(0, 0, 0);
  pwm.setPWM(2, 0, 0);
  pwm.setPWM(3, 0, 0);
}

//f = front; b = back
//cw = clockwise
//ccw = counterclockwise

//Using PWM pulse
//Most of the time (NOT ALWAYS), 290-300 is the midpoint (or stop) -- TEST THIS
//Anything > 290-300 will turn the continous servos ccw
//Anything < 290-300 will turn the continous servos cw
void CommandServo::adjustSpeedPWM(int fcw, int fccw, int bcw, int bccw){
  pwm.setPWM(1, 0, fccw);
  pwm.setPWM(0, 0, fcw);
  pwm.setPWM(2, 0, bccw);
  pwm.setPWM(3, 0, bcw);
}

//Using PWM pulse
//Most of the time (NOT ALWAYS), 1400-1500 is the midpoint (or stop) -- TEST THIS
//Anything > 1400-1500 will turn the continous servos ccw
//Anything < 1400-1500 will turn the continous servos cw
void CommandServo::adjustSpeedMicros(int fcw, int fccw, int bcw, int bccw){
  pwm.writeMicroseconds(0, fccw);
  pwm.writeMicroseconds(1, fcw);
  pwm.writeMicroseconds(2, bccw);
  pwm.writeMicroseconds(3, bcw);
}

void CommandServo::steerPWM(float loopIter){
      pwm.setPWM(4, 0, midpoint[4] + (loopIter * 185));
      //1500 + (loopIter * 600)
}

void CommandServo::steerCalib(int servoNum, int midpoint){
     pwm.setPWM(servoNum, 0, midpoint);
  }
