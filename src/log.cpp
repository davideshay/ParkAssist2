#include <log.h>
#include <AsyncUDP.h>

// Variables from main.cpp

extern stateOpts curState;

extern carInfoStruct currentCar;
extern double currentDistance;
extern distanceEvaluation currentDistanceEvaluation;
extern boolean demoMode;
extern double demoDistance;
extern boolean demoIRBREAK;
extern float currentTemp;
extern ParkPreferences parkPreferences;
extern ParkPreferences defaultPreferences;

// Local file variables for logging

bool okToLog = true;
bool netLoggingStarted = false;

AsyncWebServer serverLog(81);
AsyncWebServer serverLogDetail(83);

int64_t logging_millis = 0;
const int64_t time_between_logs_millis = 500;
AsyncUDP logClientUDP;
File logFile;

void connectNetLogging() {
  bool netOK;
  if (defaultPreferences.netLogging) {
  //  logClientUDP.connect(parkPreferences.logTarget, parkPreferences.logPort);
  //   if (!logClientUDP.connected()) {
  //     logData("UDP connection - failed",true);
  //     return;
  //   }
    netLoggingStarted = true;
    String msg = "UDP Logging Initiated";
    logClientUDP.writeTo(msg.c_str(),msg.length(),parkPreferences.logTarget, parkPreferences.logPort);
    uint16_t bytes_sent = logClientUDP.broadcastTo(msg.c_str(), 44444);
    if (bytes_sent == msg.length()) {
      logData("UDP packet sent successfully",true);
    } else {
      logData("UDP packet send failed",true);
    }
    WebSerial.println("UDP Logging Initiated");  
  }
}  

void disconnectNetLogging() {
  if (defaultPreferences.netLogging) {
//    logClientUDP.stop();
    logClientUDP.close();
  }
}

void initLogging() {

    serverLog.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/plain", "Hi! This is WebSerial. You can access the WebSerial interface at http://" + WiFi.localIP().toString() + "/webserial");
      });
     
    WebSerial.begin(&serverLog);
    WebSerial.onMessage([](uint8_t *data, size_t len) {
        processConsoleMessage(data, len);
    });
    delay(1000);
    WebSerial.println("Starting Web Serial Log for Park Assist");
  
    connectNetLogging();
    serverLog.begin();

    if (parkPreferences.fileLogging) {
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

void processConsoleMessage(uint8_t *data, size_t len) {
    Serial.printf("Received %lu bytes from WebSerial: ", len);
    Serial.write(data, len);
    Serial.println();
    String d;
    for(size_t i = 0; i < len; i++){
      d += char(data[i]);
    }
    d.toUpperCase();
    if (d.startsWith("DEMO")) {
        String newMode = d.substring(5);
        if (newMode == "ON") {
            demoMode = true;
            logData("Demo mode on. Type 'move xx' to move virtual car to a specific distance from the sensor. Type 'irbreak on|off' to set the IR break sensor demo value.",true);
        } else if (newMode == "OFF") {
            demoMode = false;
        } else {
            logData("Invalid demo mode specified. Say 'demo on' or 'demo off'.",true);
        }
    } else if (d.startsWith("IRBREAK")) {
        String newBreak = d.substring(8);
        if (newBreak == "ON") {
            demoIRBREAK = true;
            logData("IR Break Demo Sensor on - demo car breaking the beam, DEMO CAR PRESENT",true);
        } else if (newBreak == "OFF") {
            demoIRBREAK = false;
            logData("IR Break Demo Sensor off - demo car not breaking the beam, DEMO CAR NOT PRESENT",true);
        } else {
            logData("Invalid IR Break Demo sensor value specified. Say 'irbreak on' or 'irbreak off'.",true);
        }
    } else if (d == "START") {
      logData("Detected start command, setting to car detected",true);
      curState = CAR_TYPE_DETECTED;
    } else if (d == "REAL") {
      String msg = "real dist in mm=";
      msg += getSensorDistancemm();
      logData(msg,true);
    } else if (d.startsWith("MOVE")) {
        String newDist = d.substring(5);
        try {
            double newDemoDistance = newDist.toDouble();
            if (newDemoDistance == 0) {
                logData("Demo distance not sent, likely invalid value.",true);  
            } else {
                demoDistance = newDemoDistance;
                logData("Demo distance set to: " + String(demoDistance) + " cm",true);
            }
        } catch (const std::exception& e) {
            logData("Error setting move distance as requested. Use a number.",true);
        }    
    } else if (d.startsWith("CHANGEIP") && parkPreferences.netLogging) {
      String newIPs = d.substring(9);
      logData("Changing netlogging IP address to " + newIPs,true);
      IPAddress newIP;
      newIP.fromString(newIPs);
      disconnectNetLogging();
      parkPreferences.logTarget = newIP;
      setPreferences();
      connectNetLogging();
      logClientUDP.println("Changed netlogging to this IP address: "+newIPs);
    } else if (d == "CLEARPREFS") {
      clearPreferences();
      logData("Cleared all preferences, including logging IP address and port.",true);  
    } else if (d == "GETPREFS") {
      getPreferences();
    } else {
        logData("Invalid command. Try 'demo on' to start demo mode or 'changeip x.x.x.x to change logging ip address.",true);
    }
}

void openLogFileAppend() {
    if (parkPreferences.fileLogging) {
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
    Serial.println(message);
    if (defaultPreferences.webLogging && includeWeb) {WebSerial.println(message);}
    if (parkPreferences.fileLogging && okToLog) {logFile.println(message);}
    if (defaultPreferences.netLogging) {
      // bool netOK =logClientUDP.beginPacket(parkPreferences.logTarget, parkPreferences.logPort);
      // if (!netOK) {
      //   Serial.println("UDP connection - beginpacket - failed");
      //   return;
      // }
      uint16_t bytesSent = logClientUDP.broadcastTo(message.c_str(), 44444);
      if (bytesSent == message.length()) {
        Serial.println("UDP packet sent successfully");
      } else {
        Serial.println("UDP packet send failed");
      }
//      logClientUDP.endPacket();
    }
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
    if (!parkPreferences.fileLogging && !parkPreferences.webLogging && !parkPreferences.netLogging) {return;}
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
    if (parkPreferences.fileLogging) {logFile.close();}
  }
  