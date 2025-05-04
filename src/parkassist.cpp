#include <parkassist.h>


Preferences externalPrefs;

extern ParkPreferences parkPreferences;

ParkPreferences defaultPreferences = {
  .maxCameraCheckMillis = 1000,
  .timeBetweenWifiChecksMillis = 30000,
  .logTarget = IPAddress(10,10,1,136),
  .logPort = 44444,
  .secsToReset = 60,
  .fileLogging = false,
  .netLogging = true,
  .webLogging = true,
  .serialLogging = true,
  .xtalk_data = {0},
  .calibrationDataSaved = false
};

String prefsNamespace = "parkassist";
String prefsKey = "mainprefs";

// utility functions

int64_t esp_millis() {
    return ( (int64_t)(esp_timer_get_time() / 1000));
}

#define RW_MODE false
#define RO_MODE true

void logPrefs(ParkPreferences logPrefs) {
    String msg = "Prefs: logtgt:";
    msg += logPrefs.logTarget.toString();
    msg += " logport:";
    msg += logPrefs.logPort;
    msg += " secsToReset:";
    msg += logPrefs.secsToReset;
    msg += " xtalksaved:";
    msg += logPrefs.calibrationDataSaved;
    msg += " first 2 bytes of xtalk data:";
    msg += logPrefs.xtalk_data[0];
    msg += ",";
    msg += logPrefs.xtalk_data[1];
    logData(msg, true);   
}

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
    externalPrefs.end();
    connectNetLogging();
    logData("Preferences after get function:",true);
    logPrefs(parkPreferences);

}

void setPreferences() {
    ParkPreferences toSetPrefs;
    toSetPrefs = parkPreferences;
    getPreferences();
    if (memcmp(&toSetPrefs, &parkPreferences, sizeof(ParkPreferences)) != 0) {
        logData("To Set prefs different than current prefs. Opening prefs RW", true);
        externalPrefs.begin(prefsNamespace.c_str(), RW_MODE);
        logData("Preferences to set:", true);
        logPrefs(toSetPrefs);
        externalPrefs.putBytes(prefsKey.c_str(), &toSetPrefs, sizeof(toSetPrefs));
        externalPrefs.end();
        parkPreferences = toSetPrefs;
        logData("Preferences changed, updated in external storage. Current prefs now:", true);
        logPrefs(parkPreferences);
    }

}

