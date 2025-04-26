#ifndef LIDAR_H
#define LIDAR_H

#include <vl53l8cx.h>

#define VL53L8CX_TARGET_STATUS_RANGE_VALID 5
#define VL53L8CX_TARGET_STATUS_RANGE_VALID_LARGE_PULSE 9
#define VL53L8CX_TARGET_STATUS_RANGE_VALID_NO_PREVIOUS 10

bool initLidarSensor();
void startSensorRanging();
double getSensorDistancemm();
bool calibrateSensor();

#endif