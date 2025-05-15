#include <wifiota.h>

int64_t ota_update_millis = 0;
const int64_t time_between_ota_log_millis = 2000;
bool otaStarted = false;
PrettyOTA OTAUpdates;

AsyncWebServer serverOTA(80);

void initWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("Connecting");
 
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("MAC address: ");
  Serial.println(WiFi.macAddress());
}

void initOTA() {
  serverOTA.begin();
  Serial.println("OTA HTTP server started");

  serverOTA.begin();
  OTAUpdates.Begin(&serverOTA);
  OTAUpdates.OverwriteAppVersion("1.0.0");
  PRETTY_OTA_SET_CURRENT_BUILD_TIME_AND_DATE();
  OTAUpdates.OnStart(onOTAStart);
  OTAUpdates.OnProgress(onOTAProgress);
  OTAUpdates.OnEnd(onOTAEnd);

  logData("OTA Updates Enabled... pausing 10 seconds to allow updates",true);
  delay(10000);

  logData("OTA Update delay complete, getting preferences and starting logging",true);
}

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
