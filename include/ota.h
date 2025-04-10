#ifndef OTA_H
#define OTA_H

#include <PrettyOTA.h>
#include <WebSerial.h>
#include <parkassist.h>

void onOTAStart(NSPrettyOTA::UPDATE_MODE updateMode);
void onOTAProgress(uint32_t currentSize, uint32_t totalSize);
void onOTAEnd(bool success);

#endif