#pragma once

#ifndef COMMAND_H
  #define COMMAND_H
#endif

//PWM SERVO VALUES - Note that the values below varies on the servo you are using
#define MIDPOINTPWM 290
#define MIDPOINTMICROS 1400
#define SERVOMINPWM  205 //For the current purpose of the car, this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAXPWM  375 //For the current purpose of the car, This is the 'maximum' pulse length count (out of 4096)
#define SERVOMINMICROS  1200 
#define SERVOMAXMICROS  1600

#define SERVO_FREQ 50 // Analog servos run at ~50 Hz

class CommandServo {
    public:      
        int midpoint[5];
        CommandServo(){
          for(int i = 0; i < 5; ++i)
            midpoint[i] = MIDPOINTPWM;
          };
        void setupServo();
        void stopS();
        void forward();
        void backward();
        void left();
        void right();
        void adjustSpeedPWM(int fcw, int fccw, int bcw, int bccw);
        void adjustSpeedMicros(int fcw, int fccw, int bcw, int bccw);
        void steerCalib(int servoNum, int midpoint);
        void steerPWM(float loopIter);
};
