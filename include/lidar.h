#ifndef LIDAR_H
#define LIDAR_H

#include <vl53l4cx_class.h>

enum DistanceStatus
{
    DISTANCE_OK,
    DISTANCE_INITIAL,
    DISTANCE_NO_DEVICE,
    DISTANCE_RANGING_FAILED,

    DISTANCE_TOO_CLOSE,
    DISTANCE_TOO_FAR,
    DISTANCE_ERROR
};

struct DistanceResults
{
    double distance_mm;
    DistanceStatus distanceStatus;
};

bool initLidarSensor();
bool startSensorRanging();
bool stopSensorRanging();
DistanceResults getSensorDistance();
bool calibrateSensorPart1();
bool calibrateSensorPart2();
void resetLidarBaseline();

#endif