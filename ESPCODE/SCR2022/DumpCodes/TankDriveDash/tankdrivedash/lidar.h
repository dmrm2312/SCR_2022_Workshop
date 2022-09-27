#pragma once

#ifndef LIDAR_H
  #define LIDAR_H
#endif

class LidarSensor {
    public:
        LidarSensor(){};
        void setupLidar();
        int getDistance();
        void runLidar();
};
