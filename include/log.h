#ifndef LOG_H
#define LOG_H

#include <Arduino.h>
#include "FS.h"
#include <LittleFS.h>
#include <WebSerial.h>
#include <WiFiClient.h>
#include <parkassist.h>
#include <AsyncUDP.h>

#define FORMAT_LITTLEFS_IF_FAILED true

void disconnectNetLogging();
void connectNetLogging();
void openLogFileAppend();
void openLogFileRead();
void logData(String message, bool includeWeb = true, bool includeMSHeader = true);
void initLogging();
void logCurrentData();

void logDetailData(double od,double cd,float ed, float ed2);
void closeLogFile();
void processConsoleMessage(uint8_t *data, size_t len);

#endif