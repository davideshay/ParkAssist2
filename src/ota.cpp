#include <ota.h>

int64_t ota_update_millis = 0;
const int64_t time_between_ota_log_millis = 2000;
extern bool otaStarted;

void onOTAStart(NSPrettyOTA::UPDATE_MODE updateMode)
{
    otaStarted = true;
    WebSerial.println("OTA update started");
    if(updateMode == NSPrettyOTA::UPDATE_MODE::FIRMWARE)
        WebSerial.println("Mode: Firmware");
    else if(updateMode == NSPrettyOTA::UPDATE_MODE::FILESYSTEM)
        WebSerial.println("Mode: Filesystem");
}

void onOTAProgress(uint32_t currentSize, uint32_t totalSize)
{
    if (esp_millis() - ota_update_millis < time_between_ota_log_millis) {return;}
    WebSerial.printf("OTA Progress Current: %u bytes, Total: %u bytes\n", currentSize, totalSize);
    ota_update_millis = esp_millis();
}

void onOTAEnd(bool success) {
  if (success) {
    WebSerial.println("OTA update finished successfully!");
  } else {
    WebSerial.println("There was an error during OTA update!");
  }
}
