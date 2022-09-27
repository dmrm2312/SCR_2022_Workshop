#pragma once

#ifndef COMMAND_H
  #define COMMAND_H
#endif

#define SERVOMIN  150 
#define SERVOMAX  600 
#define SERVO_FREQ 50

class CommandServo {
    public:
        CommandServo(){};
        void setupServo();
        void forward();
        void backward();
        void left();
        void right();
        void stopS();
        void loop();
};
