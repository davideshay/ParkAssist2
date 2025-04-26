#include "lidar.h"
#include "log.h"

extern VL53L8CX sensor_vl53l8cx;
extern uint8_t vl_status;
extern bool otaStarted;
extern bool sensorRangingStarted;
extern ParkPreferences parkPreferences;
bool deviceFound = false;

void loadCalibrationData() {
    if (!parkPreferences.calibrationDataSaved) {
        logData("No calibration data saved, starting without calibration", true);
        return;
    }
    uint8_t zero_xtalk_data[VL53L8CX_XTALK_BUFFER_SIZE] = {0};
    if (memcmp(parkPreferences.xtalk_data, zero_xtalk_data, sizeof(zero_xtalk_data)) == 0) {
        logData("Calibration Data saved, but all zeroes. Starting without calibration.", true);
        return;
    }
    if (sensor_vl53l8cx.set_caldata_xtalk(parkPreferences.xtalk_data) == VL53L8CX_STATUS_OK) {
        logData("Sensor calibrated with saved xtalk data...", true);
    } else {
        logData("Failed to set calibration data on sensor", true);
    }
}

bool calibrateSensor() {
    if (!deviceFound) {
      logData("No device found, cannot calibrate sensor",true);
      return false;
    }
    if (sensorRangingStarted) {
        logData("Sensor is already ranging, stopping", true);
        vl_status = sensor_vl53l8cx.stop_ranging();
        if (vl_status != VL53L8CX_STATUS_OK) {
            String msg = "VL53L8CX stop_ranging failed: ";
            msg += vl_status;
            logData(msg,true);
            return false;
        }
        sensorRangingStarted = false;
    }
    logData("Calibrating sensor...",true);
    
    vl_status = sensor_vl53l8cx.calibrate_xtalk(5, 16, 600);
    if (vl_status != VL53L8CX_STATUS_OK) {
        String msg = "VL53L8CX calibrate_xtalk failed: ";
        msg += vl_status;
        logData(msg,true);
        return false;
    }
    logData("Getting xtalk data...",true);
    vl_status = sensor_vl53l8cx.get_caldata_xtalk(parkPreferences.xtalk_data);
    if (vl_status != VL53L8CX_STATUS_OK) {
        String msg = "VL53L8CX get_caldata_xtalk failed: ";
        msg += vl_status;
        logData(msg,true);
        return false;
    }
    logData("Retrieved xtalk data...",true);
    logData("Sensor calibrated and preferences set...",true);
    vl_status = sensor_vl53l8cx.set_caldata_xtalk(parkPreferences.xtalk_data);
    if (vl_status != VL53L8CX_STATUS_OK) {
        String msg = "VL53L8CX set_caldata_xtalk failed: ";
        msg += vl_status;
        logData(msg,true);
        return false;
    } else {
        logData("Sensor calibrated with xtalk data...",true);
        parkPreferences.calibrationDataSaved = true;
        setPreferences();
        return true;
    }
}

bool scanBus() {
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
    Wire.end();
    return false;
  }
  else {
    logData("I2C devices were found, proceeding",true);
    return true;
  }
}

bool initLidarSensor() {

  bool i2c_ok ;
  // Initialize I2C
  i2c_ok = Wire.begin(SDA_PIN, SCL_PIN);
  if (!i2c_ok) {
    Serial.println("I2C initialization failed");
    logData("I2C initialization failed",true);
    return false;
  } else {
    Serial.println("I2C initialization success");
    logData("I2C initialization success",true);
  }
  Wire.setClock(400000); // Set I2C clock speed to 400kHz

  if (!otaStarted) {deviceFound = scanBus();}
  if (!deviceFound) {return false;};

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
    loadCalibrationData();
    return true;
  }

}

void startSensorRanging() {
  vl_status = sensor_vl53l8cx.set_ranging_mode(VL53L8CX_RANGING_MODE_CONTINUOUS);
  vl_status = sensor_vl53l8cx.start_ranging();
    if (vl_status != VL53L8CX_STATUS_OK ) {
      String msg = "VL53L8CX start_ranging failed: ";
      msg += vl_status;
      logData(msg,true);
  } else {
    logData("VL53L8CX start_ranging success",true);
    sensorRangingStarted = true;
  }
}

double getSensorDistancemm() {
  if (!deviceFound) {
    logData("No device found, cannot get distance",true);
    return 10002;
  }
  if (!sensorRangingStarted) {
    startSensorRanging();
  }
  VL53L8CX_ResultsData Results;
  uint8_t NewDataReady = 0;
  do {
    vl_status = sensor_vl53l8cx.check_data_ready(&NewDataReady);
  } while (!NewDataReady);
  if ((!vl_status) && (NewDataReady != 0)) {
    vl_status = sensor_vl53l8cx.get_ranging_data(&Results);
  } else {
    String msg = "VL53L8CX get_ranging_data failed: ";
    msg += vl_status;
    logData(msg,true);
    return 4001;
  }
  double minimumDistancemm = 10000;
  String msg="{ms:";
  msg += esp_millis();
  msg += ",";
  for (size_t i = 0; i < 16; i++)
  {
    if ((Results.distance_mm[i] < minimumDistancemm) && (
      Results.target_status[i] == VL53L8CX_TARGET_STATUS_RANGE_VALID ||
      Results.target_status[i] == VL53L8CX_TARGET_STATUS_RANGE_VALID_LARGE_PULSE ||
      Results.target_status[i] == VL53L8CX_TARGET_STATUS_RANGE_VALID_NO_PREVIOUS 
    )) {
      minimumDistancemm = Results.distance_mm[i];
    }
    msg += "d";
    msg += i;
    msg += ":";
    msg += Results.distance_mm[i];
    msg += ",s";
    msg += i;
    msg += ":";
    msg += Results.target_status[i];
    msg += ",";
  }
  msg += "temp:";
  msg += Results.silicon_temp_degc;
  msg += "}";
  logData(msg,false);
  return minimumDistancemm;
}
