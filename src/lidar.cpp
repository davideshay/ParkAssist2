#include "lidar.h"
#include "log.h"

extern VL53L8CX sensor_vl53l8cx;
extern uint8_t vl_status;
extern uint8_t xtalk_data[VL53L8CX_XTALK_BUFFER_SIZE];
extern bool otaStarted;

void calibrateSensor() {
    logData("Calibrating sensor...",true);
    sensor_vl53l8cx.calibrate_xtalk(5, 16, 600);
    logData("Getting xtalk data...",true);
    sensor_vl53l8cx.get_caldata_xtalk(xtalk_data);
    String msg = "calibrated sensor, xtalk data: ";
    for (size_t i = 0; i < VL53L8CX_XTALK_BUFFER_SIZE; i++)
    {
        msg += String(xtalk_data[i]);
        msg += ",";
    }
//    logData(msg, true);
  
}

void scanBus() {
  byte error, address;
  int nDevices;
  logData("Scanning I2C bus...",true);
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      String msg = "Found I2C device at address 0x";
      if (address<16) {
        msg += "0";
      }
      msg += String(address,HEX);
      logData(msg,true);
      nDevices++;
    }
    else if (error==4) {
      String msg = "Unknown error at address 0x";
      if (address<16) {
        msg += "0";
      }
      msg += String(address,HEX);
      logData(msg,true);
    }    
  }
  if (nDevices == 0) {
    logData("No I2C devices found",true);
  }
  else {
    logData("done",true);
  }
}

bool initLidarSensor() {

  // Initialize I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000); // Set I2C clock speed to 400kHz

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

  if (!otaStarted) {scanBus();}

  uint8_t initTries = 0;
  bool lidarIsAlive = false;


  delay(200);
    // while (!lidarIsAlive && initTries < 5) {
    //     vl_status = vl53l8cx_is_alive(&Dev, &isAlive);
    //     vl_status = sensor_vl53l8cx.init();
    //     initTries++;
    //     delay(100);
    // }
  
  if (otaStarted) {return true;};
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
    if (otaStarted) {return true;};
    calibrateSensor();
    return true;
  }

}