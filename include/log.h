#ifndef LOG_H
#define LOG_H

#include <Arduino.h>
#include "FS.h"
#include <LittleFS.h>
#include <WebSerial.h>
#include <WiFiClient.h>
#include <parkassist.h>

#define FORMAT_LITTLEFS_IF_FAILED true

void openLogFileAppend();
void openLogFileRead();
void logData(String message, bool includeWeb);
void initLogging();
void logCurrentData();
void logDetailData(double od,double cd,float ed);
void closeLogFile();
void processConsoleMessage(uint8_t *data, size_t len);

#endif