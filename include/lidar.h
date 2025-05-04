#ifndef LIDAR_H
#define LIDAR_H

#include <vl53l8cx.h>

#define VL53L8CX_TARGET_STATUS_RANGE_VALID 5
#define VL53L8CX_TARGET_STATUS_RANGE_VALID_LARGE_PULSE 9
#define VL53L8CX_TARGET_STATUS_RANGE_VALID_NO_PREVIOUS 10

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
bool calibrateSensor();

#endif