#include "lidar.h"
#include "log.h"

extern VL53L8CX sensor_vl53l8cx;
extern uint8_t vl_status;

bool initLidarSensor() {
     // Configure VL53L8CX component.
  vl_status = sensor_vl53l8cx.begin();
  if (vl_status != VL53L8CX_STATUS_OK) {
    Serial.print("VL53L8CX begin failed: ");
    Serial.println(vl_status);
    logData("VL53L8CX begin failed:" + vl_status,true);
  } else {
    Serial.println("VL53L8CX begin success");
    logData("VL53L8CX begin success",true);
  }

  uint8_t initTries = 0;
  bool lidarIsAlive = false;


  delay(2000);
    // while (!lidarIsAlive && initTries < 5) {
    //     vl_status = vl53l8cx_is_alive(&Dev, &isAlive);
    //     vl_status = sensor_vl53l8cx.init();
    //     initTries++;
    //     delay(100);
    // }
  logData("About to init sensor..",true);
  vl_status = sensor_vl53l8cx.init();
  if (vl_status != VL53L8CX_STATUS_OK) {
    Serial.print("VL53L8CX init failed: ");
    Serial.println(vl_status);
    logData("VL53L8CX init failed:" + vl_status,true);
    return false;
  } else {
    Serial.println("VL53L8CX init success");
    logData("VL53L8CX init success",true);
    return true;
  }

}