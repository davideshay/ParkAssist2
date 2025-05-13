#include "lidar.h"
#include "log.h"

VL53L4CX sensor_vl53l4cx(&Wire, -1);
extern uint8_t vl_status;
extern bool otaStarted;
extern bool sensorRangingStarted;
extern ParkPreferences parkPreferences;
bool deviceFound = false;

bool loadCalibrationData() {
    if (!parkPreferences.calibrationDataSaved) {
        logData("No calibration data saved, starting without calibration", true);
        return false;
    }
    if (parkPreferences.calData.struct_version == 0) {
        logData("Calibration Data saved, but struct version is zero zeroes. Starting without calibration.", true);
        return false;
    }
    if (sensor_vl53l4cx.VL53L4CX_SetCalibrationData(&parkPreferences.calData) == VL53L4CX_ERROR_NONE) {
        logData("Sensor calibrated with saved xtalk data...", true);
    } else {
        logData("Failed to set calibration data on sensor", true);
        return false;
    }
    return true;
}

void printPrefsData() {
    logData("Preferences version:",true);
    logData(String(parkPreferences.calData.struct_version),true);
    logData("opt center: " + String(parkPreferences.calData.optical_centre.x_centre) + "," + String(parkPreferences.calData.optical_centre.y_centre),true);
}

bool getCalibrationData() {
    logData("Getting calibration data...Before Prefs:",true);
    printPrefsData();
    vl_status = sensor_vl53l4cx.VL53L4CX_GetCalibrationData(&parkPreferences.calData);
    if (vl_status != VL53L4CX_ERROR_NONE) {
        String msg = "VL53L4CX get calibration data failed: ";
        msg += vl_status;
        logData(msg,true);
        return false;
    }
    logData("Put Calibration Data into ParkPreferences. After prefs:",true);
    printPrefsData();
    return true;
}

bool setCalibrationData() {
    logData("Setting calibration data...current prefs are:",true);
    printPrefsData();
    vl_status = sensor_vl53l4cx.VL53L4CX_SetCalibrationData(&parkPreferences.calData);
    if (vl_status != VL53L4CX_ERROR_NONE) {
        String msg = "VL53L8CX Set calibration data failed: ";
        msg += vl_status;
        logData(msg,true);
        return false;
    } else {
        logData("Sensor calibrated with calibration data...",true);
        return true;
    }
}

void saveCalibrationDataToPrefs() {
    parkPreferences.calibrationDataSaved = true;
    setPreferences();
    logData("Calibration data saved to preferences",true);
}

bool calibrateSensorPart1() {
    if (!deviceFound) {
      logData("No device found, cannot calibrate sensor",true);
      return false;
    }
    if (sensorRangingStarted) {
        logData("Sensor is already ranging, stopping", true);
        if (stopSensorRanging()) {
            logData("Sensor stopped ranging successfully", true);
        } else {
            logData("Failed to stop sensor ranging", true);
            return false;          
        }
        sensorRangingStarted = false;
    }
    logData("Calibrating sensor... Set target at 14cm - First doing RefSPAD",true);

    vl_status = sensor_vl53l4cx.VL53L4CX_PerformRefSpadManagement();
    if (vl_status != VL53L4CX_ERROR_NONE) {
        String msg = "VL53L4CX PerformRefSpadManagement failed: ";
        msg += vl_status;
        logData(msg,true);
        return false;
    }
    logData("RefSPAD complete, proceeding to get calibration data...",true);
    delay(300);

    if (!getCalibrationData()) { return false;}

    logData("Got calibration data, now setting it...",true);
    delay(300);
    if (!setCalibrationData()) { return false;}
    delay(300);
    logData("Calibrating refSPAD...now doing offset",true);
    vl_status = sensor_vl53l4cx.VL53L4CX_PerformOffsetPerVcselCalibration(140);
    if (vl_status != VL53L4CX_ERROR_NONE) {
        String msg = "VL53L4CX PerformOffsetCalibration failed: ";
        msg += vl_status;
        logData(msg,true);
        return false;
    }
    delay(300);
    logData("Offset calibration complete, saving calibration data...",true);
    if (!getCalibrationData()) { return false;}
    delay(300);
    if (!setCalibrationData()) { return false;}
    return true;
}

bool calibrateSensorPart2() {
    if (!deviceFound) {
      logData("No device found, cannot calibrate sensor",true);
      return false;
    }

    logData("Calibrating sensor...target should be set to 60cm ... now doing xtalk",true);
    vl_status = sensor_vl53l4cx.VL53L4CX_PerformXTalkCalibration();
    if (vl_status != VL53L4CX_ERROR_NONE) {
        String msg = "VL53L4CX PerformXTalkCalibration failed: ";
        msg += vl_status;
        logData(msg,true);
        return false;
    }
    logData("XTalk calibration complete, proceeding to get calibration data...",true);

    if (!getCalibrationData()) { return false;}
    logData("Got calibration data, now setting it...",true);
    if (!setCalibrationData()) { return false;}
    logData("Calibration data set, now saving to preferences...",true);
    saveCalibrationDataToPrefs();
    return true;
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

     // Configure VL53L4CX component.
  vl_status = sensor_vl53l4cx.begin();
  if (vl_status != VL53L4CX_ERROR_NONE) {
    Serial.print("VL53L4CX begin failed: ");
    Serial.println(vl_status);
    logData("VL53L4CX begin failed:" + vl_status,true);
    return false;
  } else {
    Serial.println("VL53L4CX begin success");
    logData("VL53L4CX begin success",true);
  }

  vl_status = sensor_vl53l4cx.InitSensor(VL53L4CX_DEFAULT_DEVICE_ADDRESS);
  if (vl_status != VL53L4CX_ERROR_NONE) {
    Serial.print("VL53L4CX Init Sensor failed: ");
    Serial.println(vl_status);
    logData("VL53L4CX Init Sensor failed:" + vl_status,true);
    return false;
  } else {
    Serial.println("VL53L4CX Init Sensor success");
    logData("VL53L4CX Init Sensor success",true);
  }

  if (parkPreferences.calibrationDataSaved) {
    logData("VL53L4CX calibration data saved, loading...",true);
    if (!loadCalibrationData()) {
        logData("VL53L4CX load calibration data failed. Proceeding without calibration.",true);
        return false;
    } else {
        logData("VL53L4CX load calibration data success",true);
    }
  } else {
    logData("VL53L4CX no calibration data saved, starting without calibration",true);
  }

  if (!parkPreferences.calibrationDataSaved) {return true;};

  vl_status = sensor_vl53l4cx.VL53L4CX_SetMeasurementTimingBudgetMicroSeconds(100000);
  if (vl_status != VL53L4CX_ERROR_NONE) {
    String msg = "VL53L4CX Set Measurement Timing Budget failed: ";
    msg += vl_status;
    logData(msg,true);
    return false;
  } else {
    logData("VL53L4CX Set Measurement Timing Budget success",true);
  }

  vl_status = sensor_vl53l4cx.VL53L4CX_SetTuningParameter(VL53L4CX_TUNINGPARM_PHASECAL_PATCH_POWER,2);
  if (vl_status != VL53L4CX_ERROR_NONE) {
    String msg = "VL53L4CX Set Tuning Parameter failed: ";
    msg += vl_status;
    logData(msg,true);
    return false;
  } else {
    logData("VL53L4CX Set Tuning Parameter success",true);
  }

  vl_status = sensor_vl53l4cx.VL53L4CX_SmudgeCorrectionEnable(VL53L4CX_SMUDGE_CORRECTION_CONTINUOUS);
  if (vl_status != VL53L4CX_ERROR_NONE) {
    String msg = "VL53L4CX Smudge Correction Enable failed: ";
    msg += vl_status;
    logData(msg,true);
    return false;
  } else {
    logData("VL53L4CX Smudge Correction Enable success",true);
  }

  vl_status = sensor_vl53l4cx.VL53L4CX_SetOffsetCorrectionMode(VL53L4CX_OFFSETCORRECTIONMODE_PERVCSEL);
  if (vl_status != VL53L4CX_ERROR_NONE) {
    String msg = "VL53L4CX Set Offset Correction Mode failed: ";
    msg += vl_status;
    logData(msg,true);
    return false;
  } else {
    logData("VL53L4CX Set Offset Correction Mode success",true);
  }

  if (sensor_vl53l4cx.VL53L4CX_SetXTalkCompensationEnable(1) == VL53L4CX_ERROR_NONE) {
        logData("XTalk compensation enabled", true);
  } else {
        logData("Failed to enable XTalk compensation", true);
        return false;
  }
  logData("Allowing sensor to settle before starting main loop...",true);
  delay(1000);
  logData("Sensor ready for ranging...",true);
  return true;
}

bool startSensorRanging() {
  vl_status = sensor_vl53l4cx.VL53L4CX_SetDistanceMode(VL53L4CX_DISTANCEMODE_MEDIUM);
  if (vl_status != VL53L4CX_ERROR_NONE) {
    String msg = "VL53L4CX Set Distance Mode failed: ";
    msg += vl_status;
    logData(msg,true);
    return false;
  }
  vl_status = sensor_vl53l4cx.VL53L4CX_StartMeasurement();
    if (vl_status != VL53L4CX_ERROR_NONE ) {
      String msg = "VL53L4CX start measurements failed: ";
      msg += vl_status;
      logData(msg,true);
      return false;
  } else {
//    logData("VL53L8CX start_ranging success",true);
    sensorRangingStarted = true;
  }
  return true;
}

bool stopSensorRanging() {
    vl_status = sensor_vl53l4cx.VL53L4CX_StopMeasurement();
    if (vl_status != VL53L4CX_ERROR_NONE) {
        String msg = "VL53L4CX stop measurements failed: ";
        msg += vl_status;
        logData(msg,true);
        return false;
    }
    return true;
}

bool getSingleSensorMeasurement(VL53L4CX_MultiRangingData_t *pMultiRangingData) {
  uint8_t NewDataReady = 0;
  do {
    vl_status = sensor_vl53l4cx.VL53L4CX_GetMeasurementDataReady(&NewDataReady);
  } while (!NewDataReady);
  if ((!vl_status) && (NewDataReady != 0)) {
    vl_status = sensor_vl53l4cx.VL53L4CX_GetMultiRangingData(pMultiRangingData);
    return true;
  } else {
    return false;
  }
}

DistanceResults getSensorDistance() {
  DistanceResults distanceResults = {
    .distance_mm = 0,
    .distanceStatus = DISTANCE_INITIAL
  };
  if (!deviceFound) {
    logData("No device found, cannot get distance",true);
    distanceResults.distanceStatus = DISTANCE_NO_DEVICE;
    return distanceResults;
  }
  if (!sensorRangingStarted) {
    if (!startSensorRanging()) {
      distanceResults.distanceStatus = DISTANCE_RANGING_FAILED;
      return distanceResults;
    };
  }
  VL53L4CX_MultiRangingData_t MultiRangingData;
  VL53L4CX_MultiRangingData_t *pMultiRangingData = &MultiRangingData;
  int numObjectsFound = 0;
  bool getSuccess;
  getSuccess = getSingleSensorMeasurement(pMultiRangingData);
  if (!getSuccess) {
    String msg = "VL53L4CX get single measurement failed - ranging failure";
    msg += vl_status;
    logData(msg,true);
    distanceResults.distanceStatus = DISTANCE_RANGING_FAILED;
    return distanceResults;
  }
  // If this is the first measurement with a wrap-around status, get another measurement
  if (pMultiRangingData->NumberOfObjectsFound > 0 &&
      pMultiRangingData->RangeData[0].RangeStatus == VL53L4CX_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL ) {
      getSuccess = getSingleSensorMeasurement(pMultiRangingData);
      if (!getSuccess) {
        String msg = "VL53L4CX get single measurement failed - ranging failure";
        msg += vl_status;
        logData(msg,true);
        distanceResults.distanceStatus = DISTANCE_RANGING_FAILED;
        return distanceResults;
      }
  }
  numObjectsFound = pMultiRangingData->NumberOfObjectsFound;
  double minimumDistancemm = 10000;
  String msg="{ms:";
  msg += esp_millis();
  msg += ",";
  for (size_t i = 0; i < numObjectsFound; i++)
  {
    if ((pMultiRangingData->RangeData[i].RangeMilliMeter < minimumDistancemm) && (
      pMultiRangingData->RangeData[i].RangeStatus == VL53L4CX_RANGESTATUS_RANGE_VALID ||
      pMultiRangingData->RangeData[i].RangeStatus == VL53L4CX_RANGESTATUS_RANGE_VALID_MERGED_PULSE)
    ) {
      minimumDistancemm = pMultiRangingData->RangeData[i].RangeMilliMeter;
      distanceResults.distanceStatus = DISTANCE_OK;
    }
    msg += "d";
    msg += i;
    msg += ":";
    msg += String(pMultiRangingData->RangeData[i].RangeMilliMeter);
    msg += ",s";
    msg += i;
    msg += ":";
    msg += pMultiRangingData->RangeData[i].RangeStatus;
    msg += ",mi";
    msg +=  i;
    msg += ":";
    msg += String(pMultiRangingData->RangeData[i].RangeMinMilliMeter);
    msg += ",mx";
    msg += i;
    msg += ":";
    msg += String(pMultiRangingData->RangeData[i].RangeMaxMilliMeter);
    msg += ",";
  }
  if (numObjectsFound > 0) {
    msg += "min:";
    msg += String(minimumDistancemm);
    msg += "},";
    logData(msg,false);
  }
  if (distanceResults.distanceStatus == DISTANCE_OK) {
    distanceResults.distance_mm = minimumDistancemm;
  }
  sensor_vl53l4cx.VL53L4CX_ClearInterruptAndStartMeasurement();
  return distanceResults;
}
