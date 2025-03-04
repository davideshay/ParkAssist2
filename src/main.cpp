#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <WiFiClient.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>
#include <ElegantOTA.h>
#include "wificodes.h"

AsyncWebServer server(80);
AsyncWebServer serverl(81);

unsigned long ota_progress_millis = 0;

void onOTAStart() {
  // Log when OTA has started
  Serial.println("OTA update started!");
  // <Add your own code here>
}

void onOTAProgress(size_t current, size_t final) {
  // Log every 1 second
  if (millis() - ota_progress_millis > 1000) {
    ota_progress_millis = millis();
    Serial.printf("OTA Progress Current: %u bytes, Final: %u bytes\n", current, final);
  }
}

void onOTAEnd(bool success) {
  // Log when OTA has finished
  if (success) {
    Serial.println("OTA update finished successfully!");
  } else {
    Serial.println("There was an error during OTA update!");
  }
  // <Add your own code here>
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(4, INPUT);
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");
 
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
 
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hi! This is ElegantOTA AsyncDemo.");
  });

  serverl.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hi! This is WebSerial demo. You can access the WebSerial interface at http://" + WiFi.localIP().toString() + "/webserial");
  });
 
  ElegantOTA.begin(&server);    // Start ElegantOTA
  ElegantOTA.onStart(onOTAStart);
  ElegantOTA.onProgress(onOTAProgress);
  ElegantOTA.onEnd(onOTAEnd);

  server.begin();
  Serial.println("HTTP server started");

  WebSerial.begin(&serverl);
  WebSerial.onMessage([](uint8_t *data, size_t len) {
    Serial.printf("Received %lu bytes from WebSerial: ", len);
    Serial.write(data, len);
    Serial.println();
    WebSerial.println("Received Data...");
    String d = "";
    for(size_t i = 0; i < len; i++){
      d += char(data[i]);
    }
    WebSerial.println(d);
  });

  serverl.begin();

}

void loop() {
    static unsigned long last_print_time = millis();
    ElegantOTA.loop();
    WebSerial.loop();
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN,LOW);
    delay(1000);


    if ((unsigned long)(millis() - last_print_time) > 2000) {
      WebSerial.print(F("IP address: "));
      WebSerial.println(WiFi.localIP());
      WebSerial.printf("Uptime: %lums\n", millis());
      WebSerial.printf("Free heap: %u\n", ESP.getFreeHeap());
      last_print_time = millis();
    }

  // if (digitalRead(4) == LOW) {
  //   digitalWrite(LED_BUILTIN, LOW);  // turn the LED on (HIGH is the voltage level)
  // } else {
  //   digitalWrite(LED_BUILTIN, HIGH);   // turn the LED off by making the voltage LOW
  // }
}

