#pragma once

#ifndef IMU_H
  #define IMU_H
#endif

#define FILTER_UPDATE_RATE_HZ 100
#define PRINT_EVERY_N_UPDATES 10

class IMUSensor {
    public:
        IMUSensor(){};
        void setupIMU();
        double getYaw();
        double getPitch();
        double getRoll();
        void filterIMU();
};
