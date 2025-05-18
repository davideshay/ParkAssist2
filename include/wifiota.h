#ifndef WIFIOTA_H
#define WIFIOTA_H

#include <PrettyOTA.h>
#include <WebSerial.h>
#include <parkassist.h>
#include <WiFi.h>
#include <WiFiClient.h>

void initWifi();
void initOTA();
void onOTAStart(NSPrettyOTA::UPDATE_MODE updateMode);
void onOTAProgress(uint32_t currentSize, uint32_t totalSize);
void onOTAEnd(bool success);

#endif