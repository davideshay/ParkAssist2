#include <parkassist.h>
#include <log.h>

Preferences externalPrefs;

extern ParkPreferences parkPreferences;
extern CalibrationPreferences calibrationPreferences;

ParkPreferences defaultPreferences = {
  .struct_version = 1, // version of this struct, so we can update it in the future
  .maxCameraCheckMillis = 1000,
  .timeBetweenWifiChecksMillis = 30000,
  .logPort = 44444,
  .secsToResetCarStillPresent = 120,
  .secsToResetAfterCleared = 30,
  .fileLogging = false,
  .netLogging = true,
  .webLogging = true,
  .serialLogging = true,
};

String prefsNamespace = "parkassist";
String prefsKey = "mainprefs1";
//String newPrefsKey = "mainprefs1";
String calPrefsKey = "calprefs";

// utility functions

int64_t esp_millis() {
    return ( (int64_t)(esp_timer_get_time() / 1000));
}

#define RW_MODE false
#define RO_MODE true


void clearPreferences() {
    externalPrefs.begin(prefsNamespace.c_str(), RW_MODE);
    externalPrefs.clear();
    externalPrefs.end();
    logData("Preferences cleared", true);
}

void clearNVSAndReboot() {
    logData("Clearing NVS and Rebooting",true);
    if (nvs_flash_erase() != ESP_OK) {
        logData("Failed to erase NVS", true);
        ESP.restart();
    } else {
        logData("NVS erased", true);
    }
    if (nvs_flash_init() != ESP_OK) {
        logData("Failed to init NVS", true);
        ESP.restart();
    } else {
        logData("NVS initialized", true);
    }
    delay(3000);
    logData("Rebooting...", true);
    ESP.restart();
}

void getPreferences() {
    externalPrefs.begin(prefsNamespace.c_str(), RO_MODE);
    if (externalPrefs.isKey(prefsKey.c_str())) {
        externalPrefs.getBytes(prefsKey.c_str(), &parkPreferences, sizeof(parkPreferences));
        logData("Preferences loaded from external storage.", true);
    } else {
        externalPrefs.end();
        externalPrefs.begin(prefsNamespace.c_str(), RW_MODE);
        parkPreferences = defaultPreferences;
        externalPrefs.putBytes(prefsKey.c_str(), &parkPreferences, sizeof(parkPreferences));
        logData("Preferences not found, created using defaults", true);
    }
    if (externalPrefs.isKey(calPrefsKey.c_str())) {
        externalPrefs.getBytes(calPrefsKey.c_str(), &calibrationPreferences, sizeof(calibrationPreferences));
        logData("Calibration preferences loaded from external storage.", true);
    } else {
        externalPrefs.end();
        externalPrefs.begin(prefsNamespace.c_str(), RW_MODE);
        calibrationPreferences.calibrationDataSaved = false;
        calibrationPreferences.calData.struct_version = 0; // default version
        externalPrefs.putBytes(prefsKey.c_str(), &parkPreferences, sizeof(parkPreferences));
        logData("Calibration preferences not found, using defaults", true);
    }
    externalPrefs.end();
}

void logPrefs(ParkPreferences logPrefs, CalibrationPreferences logCalPrefs) {
    String msg = "Prefs: logport:";
    msg += logPrefs.logPort;
    msg += " secsToResetCarStillPresent:";
    msg += logPrefs.secsToResetCarStillPresent;
    msg += " secsToResetAfterCleared:";
    msg += logPrefs.secsToResetAfterCleared;
    msg += " caldatasaved:";
    msg += logCalPrefs.calibrationDataSaved;
    msg += " calstructver:";
    msg += logCalPrefs.calData.struct_version;
    msg += ",ms between wifi checks:";
    msg += logPrefs.timeBetweenWifiChecksMillis;
    msg += ",spads:";
    msg += logCalPrefs.calData.customer.ref_spad_man__num_requested_ref_spads;
    msg += ",filelogging:";
    msg += logPrefs.fileLogging;
    msg += ",netlogging:";
    msg += logPrefs.netLogging;
    msg += ",weblogging:";
    msg += logPrefs.webLogging;
    msg += ",seriallogging:",
    msg += logPrefs.serialLogging;
    logData(msg, true);   
}


void copyPrefsIntoCalData() {};
void updatePrefsVersion() {};

// void copyPrefsIntoCalData() {
//     logData("Copying preferences into calibration data", true);
//     externalPrefs.begin(prefsNamespace.c_str(), RW_MODE);
//     calibrationPreferences.calibrationDataSaved = true;
//     calibrationPreferences.calData = parkPreferences.calData;
//     externalPrefs.putBytes(calPrefsKey.c_str(),&calibrationPreferences, sizeof(calibrationPreferences));
//     externalPrefs.end();
// }    

// void updatePrefsVersion() {
//     logData("Updating preferences version", true);
//     externalPrefs.begin(prefsNamespace.c_str(), RW_MODE);
//     if (externalPrefs.isKey(prefsKey.c_str())) {
//         ParkPreferences oldPref;
//         NewParkPreferences newPref;
//         externalPrefs.getBytes(prefsKey.c_str(), &oldPref, sizeof(oldPref));
//         newPref.struct_version = 1; // new version
//         newPref.maxCameraCheckMillis = oldPref.maxCameraCheckMillis;
//         newPref.timeBetweenWifiChecksMillis = oldPref.timeBetweenWifiChecksMillis;
//         newPref.logPort = oldPref.logPort;
//         newPref.secsToResetCarStillPresent = 120;
//         newPref.secsToResetAfterCleared = 30;
//         newPref.fileLogging = oldPref.fileLogging;
//         newPref.netLogging = oldPref.netLogging;
//         newPref.webLogging = oldPref.webLogging;
//         newPref.serialLogging = oldPref.serialLogging;
//         externalPrefs.putBytes(newPrefsKey.c_str(), &newPref, sizeof(newPref));
//         logData("Preferences updated to version 1", true);
//     } else {
//         logData("No preferences found to update", true);
//     }
//     externalPrefs.end();
// }


void setPreferences() {
    ParkPreferences toSetPrefs;
    CalibrationPreferences toSetCalPrefs;
    toSetPrefs = parkPreferences;
    toSetCalPrefs = calibrationPreferences;
    getPreferences();
    if (memcmp(&toSetPrefs, &parkPreferences, sizeof(parkPreferences)) != 0) {
        logData("To Set prefs different than current prefs. Opening prefs RW", true);
        externalPrefs.begin(prefsNamespace.c_str(), RW_MODE);
        logData("Preferences to set:", true);
        logPrefs(toSetPrefs,toSetCalPrefs);
        externalPrefs.putBytes(prefsKey.c_str(), &toSetPrefs, sizeof(toSetPrefs));
        externalPrefs.end();
        parkPreferences = toSetPrefs;
        logData("Preferences changed, updated in external storage. Current prefs now:", true);
        logPrefs(parkPreferences,calibrationPreferences);
    } else {
        logData("Preferences to set are the same as current prefs. No changes made.", true);
    }
    if (memcmp(&toSetCalPrefs, &calibrationPreferences, sizeof(calibrationPreferences)) != 0) {
        logData("To Set Calibration prefs different than current calibration prefs. Opening prefs RW", true);
        externalPrefs.begin(prefsNamespace.c_str(), RW_MODE);
        logData("Preferences to set:", true);
        logPrefs(toSetPrefs,toSetCalPrefs);
        externalPrefs.putBytes(calPrefsKey.c_str(), &toSetCalPrefs, sizeof(toSetCalPrefs));
        externalPrefs.end();
        calibrationPreferences = toSetCalPrefs;
        logData("Calibration preferences changed, updated in external storage. Current prefs now:", true);
        logPrefs(parkPreferences,calibrationPreferences);
    } else {
        logData("Calibration Preferences to set are the same as current prefs. No changes made.", true);
    }
}

bool isValidNumber(String str) {
    bool isValid = false;
    for (size_t i = 0; i < str.length(); i++)
    {
        isValid = isDigit(str.charAt(i));
        if (!isValid) {return false;}
    }
    return isValid;
}

bool isValidOnOffToken(String str) {
    return (str == "ON" || str == "OFF" || str == "TRUE" || str == "FALSE" || str == "YES" || str == "NO");
}

bool tokenIsTrue(String str) {
    return (str == "ON" || str == "TRUE" || str == "YES");
}

void setOnePref(String msg) {
    String startToCut="SETPREF";
    if (msg.length() <= startToCut.length()) {
        logData("Invalid preference to set - nothing specified",true);
        return;
    }
    String fullPrefToSet = msg.substring(startToCut.length()+1);
    int spaceAt = fullPrefToSet.indexOf(" ");
    if (spaceAt == -1 || fullPrefToSet.length() == spaceAt) {
        logData("No value to set preference key to",true);
        return;
    }
    fullPrefToSet.toUpperCase();
    String prefToSet = fullPrefToSet.substring(0,spaceAt);
    String valToSet = fullPrefToSet.substring(spaceAt+1);
    if (prefToSet == "LOGPORT") {
        if (!isValidNumber(valToSet)) {
            logData("Log Port specified is not a valid number",true);
            return;
        }
        uint16_t valPort = valToSet.toInt();
        if (valPort > 65534) {
            logData("Log Port is not valid for TCPIP Port",true);
        }
        parkPreferences.logPort = valPort;
        setPreferences();        
    } else if (prefToSet == "SECSRESETPRESENT") {
        if (!isValidNumber(valToSet)) {
            logData("Seconds to Reset specified is not a valid number",true);
            return;
        }
        uint16_t secs = valToSet.toInt();
        if (secs > 3600) {
            logData("Seconds to Reset is too large",true);
        }
        parkPreferences.secsToResetCarStillPresent = secs;
        setPreferences();        
    } else if (prefToSet == "SECSRESETCLEARED") {
        if (!isValidNumber(valToSet)) {
            logData("Seconds to Reset specified is not a valid number",true);
            return;
        }
        uint16_t secs = valToSet.toInt();
        if (secs > 3600) {
            logData("Seconds to Reset is too large",true);
        }
        parkPreferences.secsToResetAfterCleared = secs;
        setPreferences();        
    } else if (prefToSet == "FILELOGGING") {
        if (!isValidOnOffToken(valToSet)) {
            logData("Invalid on/off/true/false/yes/no value for file logging",true);
        }
        parkPreferences.fileLogging = tokenIsTrue(valToSet);
        setPreferences();
    } else if (prefToSet == "WEBLOGGING") {
        if (!isValidOnOffToken(valToSet)) {
            logData("Invalid on/off/true/false/yes/no value for web logging",true);
        }
        parkPreferences.webLogging = tokenIsTrue(valToSet);
        setPreferences();
    } else if (prefToSet == "NETLOGGING") {
        if (!isValidOnOffToken(valToSet)) {
            logData("Invalid on/off/true/false/yes/no value for net logging",true);
        }
        parkPreferences.netLogging = tokenIsTrue(valToSet);
        setPreferences();
    } else if (prefToSet == "SERIALLOGGING") {
        if (!isValidOnOffToken(valToSet)) {
            logData("Invalid on/off/true/false/yes/no value for serial logging",true);
        }
        parkPreferences.serialLogging = tokenIsTrue(valToSet);
        setPreferences();
    } else {
        logData("You are trying to update preference "+prefToSet+" to value:"+valToSet,true);
        logData("Not a valid preference. Choose LOGPORT, SECSRESETPRESENT, SECSRESETCLEARED, FILELOGGING, WEBLOGGING, NETLOGGING, or SERIALLOGGING.",true);
    }
}