#include <Adafruit_PWMServoDriver.h>
#include "command.h"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


void CommandServo::setupServo()
{
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ);  
}

void CommandServo::forward(){
  pwm.setPWM(1, 0, SERVOMAX);
  pwm.setPWM(0, 0, SERVOMIN);
  pwm.setPWM(4, 0, SERVOMAX);
  pwm.setPWM(5, 0, SERVOMIN);
}

void CommandServo::backward(){
  pwm.setPWM(0, 0, SERVOMAX);
  pwm.setPWM(1, 0, SERVOMIN);
  pwm.setPWM(5, 0, SERVOMAX);
  pwm.setPWM(4, 0, SERVOMIN);
}

void CommandServo::left(){
  pwm.setPWM(1, 0, SERVOMIN);
  pwm.setPWM(4, 0, SERVOMIN);
  pwm.setPWM(0, 0, SERVOMIN);
  pwm.setPWM(5, 0, SERVOMIN);
}

void CommandServo::right(){
  pwm.setPWM(1, 0, SERVOMAX);
  pwm.setPWM(4, 0, SERVOMAX);
  pwm.setPWM(0, 0, SERVOMAX);
  pwm.setPWM(5, 0, SERVOMAX);
}

void CommandServo::stopS(){
  pwm.setPWM(1, 0, 0);
  pwm.setPWM(0, 0, 0);
  pwm.setPWM(5, 0, 0);
  pwm.setPWM(4, 0, 0);
}
