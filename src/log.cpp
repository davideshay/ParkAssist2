#include <log.h>

// Variables from main.cpp

extern stateOpts curState;

extern carInfoStruct currentCar;
extern double currentDistance;
extern distanceEvaluation currentDistanceEvaluation;

// Local file variables for logging

boolean fileLogging = false;
boolean netLogging = true;
boolean webLogging = true;
boolean okToLog = true;

AsyncWebServer serverLog(81);
AsyncWebServer serverLogDetail(83);

int64_t logging_millis = 0;
const int64_t time_between_logs_millis = 500;
WiFiClient logClient;
IPAddress logTarget = IPAddress(10,10,1,136);
File logFile;

void initLogging() {

    serverLog.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/plain", "Hi! This is WebSerial. You can access the WebSerial interface at http://" + WiFi.localIP().toString() + "/webserial");
      });
     
    WebSerial.begin(&serverLog);
    WebSerial.onMessage([](uint8_t *data, size_t len) {
      Serial.printf("Received %lu bytes from WebSerial: ", len);
      Serial.write(data, len);
      Serial.println();
      WebSerial.println("Received Data...");
      String d = "";
      for(size_t i = 0; i < len; i++){
        d += char(data[i]);
      }
      if (d == "move") {
        WebSerial.println("Detected move command, setting to car detected");
        curState = CAR_TYPE_DETECTED;
      } else if (d.startsWith("changeip") && netLogging) {
        String newIPs = d.substring(9);
        WebSerial.println("Changing netlogging IP address to " + newIPs );
        IPAddress newIP;
        newIP.fromString(newIPs);
        logClient.stop();
        if (!logClient.connect(newIP,10000)) {
          WebSerial.println("Error changing netlogging IP address to " + newIPs);
        } else {
          logClient.println("Changed netlogging to this IP address: "+newIPs);
        }
      }
      WebSerial.println(d);
    });
    delay(1000);
    WebSerial.println("Starting Web Serial Log for Park Assist");
  
    if (netLogging) {
      if (!logClient.connect(logTarget, 10000)) {      
        WebSerial.println("Connection to host failed");
      }
      logClient.println("TCP Logging Initiated");
      WebSerial.println("TCP Logging Initiated");
    }  
    serverLog.begin();

    if (fileLogging) {
        if (!LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED)) {
          WebSerial.println("Unable to initialize LittleFS");
          okToLog = false;
        } else {
          LittleFS.remove("/logfile.txt");
          logFile = LittleFS.open("/logfile.txt", FILE_APPEND);
          openLogFileAppend();
        }  
      }
    
        // Handle the download button
    serverLogDetail.on("/download", HTTP_GET, [](AsyncWebServerRequest *request){
        logFile.close();
        openLogFileRead();
        request->send(LittleFS, "/logfile.txt", String(),false);
        logFile.close();
        openLogFileAppend();
    });
  serverLogDetail.begin();
}

void openLogFileAppend() {
    if (fileLogging) {
      logFile = LittleFS.open("/logfile.txt", FILE_APPEND);
      if (!logFile) {
        WebSerial.println("Unable to open log file for writing");
        okToLog = false;
      }  
    }
  }
  
void openLogFileRead() {  
    logFile = LittleFS.open("/logfile.txt", FILE_READ);
    if (!logFile) {
      WebSerial.println("Unable to open log file for reading");
      okToLog = false;
    }
}
  
void logData(String message, bool includeWeb = true) {
    if (webLogging && includeWeb) {WebSerial.println(message);}
    if (fileLogging && okToLog) {logFile.println(message);}
    if (netLogging) {logClient.println(message);}
}
  
void logCurrentData() {
    if (esp_millis() - logging_millis < time_between_logs_millis) {return;}
    boolean carPresent = false;
    if (digitalRead(IR_BREAK_SENSOR) == LOW) {
      carPresent = true;
    }
    String logLine;
    logLine = "cctcm=";
    logLine += currentCar.targetFrontDistanceCm;
    logLine += " dcm=";
    logLine += currentDistance;
    logLine += " dttcm=";
    logLine += currentDistanceEvaluation.cmToTarget;
    logLine += " dttin=";
    logLine += currentDistanceEvaluation.inchesToTarget;
    logLine += " dspdt=";
    logLine += currentDistanceEvaluation.displayDistance;
    logLine += " dspinf=";
    logLine += currentDistanceEvaluation.displayInfinity;
    if (carPresent) {
      logLine += " CARPRES";
    } else {
      logLine += " NOCAR";
    }
    logLine += " cc=";
    logLine += currentDistanceEvaluation.colorCode;
    logLine += " co=";
    logLine += currentDistanceEvaluation.colorOffset;
    logData(logLine,true);
    logging_millis=esp_millis();
  }
  
void logDetailData(double od,double cd,float ed) {
    if (!fileLogging && !webLogging && !netLogging) {return;}
    String logLine;
    logLine = "ms=";
    logLine += esp_millis();
    logLine += " od=";
    logLine += od;
    logLine += " cd=";
    logLine += cd;
    logLine += " ed=";
    logLine += ed;
    logData(logLine,false);
  }

  void closeLogFile() {
    if (fileLogging) {logFile.close();}
  }
  