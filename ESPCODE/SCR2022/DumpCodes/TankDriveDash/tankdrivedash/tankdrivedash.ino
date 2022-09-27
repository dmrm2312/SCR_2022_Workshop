#define WSDEBUG(txt) \
    DEBUGF("ws[%s][%u] " txt "\r\n", server->url(), client->id())

#define WSDEBUGF(txt, ...) \
    DEBUGF("ws[%s][%u] " txt "\r\n", server->url(), client->id(), __VA_ARGS__)
    
#include <Arduino.h>

#include "debug.h"
#include "server.h"
#include "command.h"
#include "imu.h"
#include "lidar.h"
#include <WiFi.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// WiFi AP parameters
char ap_ssid[20] = "NameOfThing: Yeet";
const char ap_password[] = "1234567890";

// WiFi STA parameters
const char sta_ssid[] = "...";
const char sta_password[] = "...";

ESPServer server;
CommandServo servo;
IMUSensor imu;
LidarSensor lidar;

double imuStart;

// Callback: receiving any WebSocket message
void customBinCallback (AsyncWebSocketClient * client, uint64_t len, uint8_t* bytes) {
    DEBUGF("ws RX[%u] binary message of length %llu \r\n", client->id(), len)
    client->binary("I got your binary message");
}

//For handling joystick values
void commandServos(float x, float y){
   Serial.println("in command servo");
  //FORWARD
  if(y > 0){
    Serial.println("in forqard");
    //Straight
    if (x > -0.1 && x < 0.1){
      Serial.println("in forward");
      servo.backward();
    }
    //Forward Left
    if (x <= -0.1 && x >= -1){
      servo.left();
      }
     //Forward Right
     if (x <= 1 && x >= 0.1){
        servo.right();
      }
    }  
  //Backward
  else if(y < 0){
    //Straight
    if (x > -0.1 && x < 0.1){
      servo.forward();
    }
    //Backward Left
    if (x <= -0.1 && x >= -1){
      servo.right();
    }
     //Backward Right
    if (x <= 1 && x >= 0.1){
      servo.left();
    }
  }  
  else{
    servo.stopS();
   }
}
 
//Joystick range is [-1.0, 1.0] for each axis 
//Formatted as [x y]
void handleJoystickCallback(String joyString) {

  //Parse JSON
  int first = joyString.indexOf("\"", 0);
  int second = joyString.indexOf("\"", first+1);
  int third = joyString.indexOf("\"", second+1);
  int fourth = joyString.lastIndexOf("\"");
  joyString = joyString.substring(third+1, fourth);

  int space = joyString.indexOf(' ');
  float joyX = ((joyString.substring(0, space)).toFloat());
  float joyY = ((joyString.substring(space+1, joyString.length())).toFloat());

  Serial.println("Joystick values:");
  Serial.println(joyX);
  Serial.println(joyY);
  
  commandServos(joyX, joyY);
}

void customTxtCallback (AsyncWebSocketClient * client, uint64_t len, uint8_t* str) {
    // Prevent buffer overflow with poorly formatted strings
    str[len] = 0;
    String strn = (char*)(str);

    Serial.print("Got: " + strn);
    handleJoystickCallback(strn);
}

void setup() {
    DEBUG_START;
    
    Serial.begin(115200);
    
    static uint32_t chipId = 0;
    // Equivalent to ESP.getChipId() on ESP8266
    for(int i=0; i<17; i=i+8) {
        chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
    }
    sprintf(ap_ssid, "ESP_%08X", chipId);

    server.setupAP(ap_ssid, ap_password);
    server.setupWS(customBinCallback, customTxtCallback);

    servo.setupServo();
    imu.setupIMU();
    lidar.setupLidar();
    server.start();
}

int count = 0;
int displayIMU = 0;
static int start = 0;

void loop() {
    if((start == 0) && (displayIMU % 100 == 0)){
      imuStart = imu.getYaw();
      start++;
      }
    // toggle LED every 2e9 ms --> 1.024 Hz, 50% duty cycle blink
    digitalWrite(LED_BUILTIN, !!(millis() & (1 << 9)));
    
    imu.filterIMU();
    lidar.runLidar();

    if(displayIMU >= 100){
    String toP2 = "{\"IMU\": \"" + String(imu.getYaw()) + "\"}" ;
    server.broadcast("IMU", toP2);
 
    String toP = "{\"LIDAR\": \"" + String(lidar.getDistance()) + "\"}" ;
    server.broadcast("LIDAR", toP);
    }
    ++displayIMU;

    if(lidar.getDistance() <= 130){
      servo.stopS();
    }
    else{
      servo.backward();
      }

    delay(100);
}
