//Integrated Demo code (ALL Functions)
//Documentation for controlling servo motors is in command.cpp and command.h

#include <Arduino.h>
#include <WiFi.h>
#include <SPI.h>

//DO THIS FIRST" UNCOMMENT the type of steering command that will be used (WARNING: ONLY UNCOMMENT ONE!!!!)
//#define PIVOT
#define DIFFERENTIAL

#include "command.h"
#include "imu.h"
#include "ArnholdMesh.h"
#include "LedPanel.h"

//BEG: Camera and OpenMV PID stuff
//TODO: Put this in command.h and command.cpp
#include <openmvrpc.h>

#define X_MIDDLE 160
#define DIST_MIDDLE 200
// ﻿green: (21, 48, -25, -115, 127, 14)
bool lookForBlob = false;

openmv::rpc_scratch_buffer<256> scratch_buffer; // All RPC objects share this buffer.
openmv::rpc_i2c_master interface(0x12, 100000);

struct results { short cx, cy, ztrans; } tag_result;
unsigned long loopStartTime = millis();
unsigned long elapsedTime = millis();
unsigned long lastMRecv = millis();

// aprilTag/blob rotation variables
const float rotKp = .4;
const float rotKd = .3;
int rotError = 0;
int rotPrevError = 0;
float rotPid = 0;

//aprilTag distance variables
const float distKp = .2;
const float distKd = .4;
int distError = 0;
int distPrevError = 0;
float distPid = 0;

int leftBase = MIDPOINTPWM;
int rightBase = MIDPOINTPWM;
//END: Camera and OpenMV PID stuff

//CONSTANTS for instructing car to go to a specified mm distance (adjust offset)
#define WHEELDIAMETER 70 //diameter of the wheels of the car
#define SERVOREVOL 275 //specifying how many times the PWM signal will be sent to the continous servo motors (alter if needed)
#define OFFSET 3.3 //change this or either SERVOREVOL to properly go to the distance you wanted the car to be

//VARIABLES for TWOAPRILTAGSTARGET
int leftMotor = 0;
int rightMotor = 0;

//CONSTANTS for Mesh
#define   STATION_SSID     "lemur"
#define   STATION_PASSWORD "lemur9473"
#define   MQTT_BROKER      "test.mosquitto.org"
#define   MQTT_TOPIC       "LEMUR"

//VARIABLES for IMU (Orientation + PID)
double currentAngle = 0;
const float imuKp = 0.8;
const float imuKd = .5;
int imuError = 0;
int imuPrevError = 0;
float imuPid = 0;

//VARIABLES for Headers
CommandServo servo;
IMUSensor imu;
ArnholdMesh thisNode;
LedPanel panel(32, 32);

//FUNCTION PROTOTYPES + SHORT DESCRIPTION
//MESH FUNCTIONS
void interpretDashInstructions(String& msg); 
void vehicleInstruction(char instructionType, String instrData, uint32_t from);
void receivedCallback(uint32_t from, String &msg);
void mqttCallback(char* topic, uint8_t* payload, unsigned int length);
void newConnectionCallback(uint32_t nodeId);

//OPENMV OR CAMERA FUNCTIONS
void exe_apriltags(int8_t id1, results &tag_result); //detects april tags using OpenMV cam; gives orientation in x,y,z of tag + rotation
void aprilTagPid(); //PID for car to follow april tags
void exe_blob(results &tag_result); //detects a color blob using OpenMV cam; gives orientation in x,y,z of blob + rotation
void blobPid(); //PID for car to follow color blob 
void aprilTagsTarget(int lSpeed, int rSpeed); //using an outside camera -- THIS IS THE BLIMP'S FUNCTION OR KAMIL's

//IMU FUNCTIONS
void imuPID(); //PID for car to drive straight
void handleOrientationPos(float theta);

//CAR COMMAND or SERVO FUNCTIONS
void pivotCommand(float x, float y); //DASH JOYSTICK: handling joystick control for car with pivot steering
void differentialCommand(float x, float y); //DASH JOYSTICK: handling joystick control for car with differential steering
void calibrateServo(int servoNum, int midpoint); //DASH INPUT: calibrating the midpoint of servo 
void toDistance(float d); //DASH INPUT: instructing car to go to ## mm

void setup() {
  Serial.begin(115200);
  
  servo.setupServo();
  servo.stopS();

  imu.setupIMU();
  panel.beginPanel();
  thisNode.setPanel(&panel);
  static int x = 0;
  while(x <= 1000){
    imu.filterIMU();
    currentAngle = imu.getYaw();
    thisNode.panel->topLeftQuadrant(20,0, 0);
    ++x;
    delay(50);
    }
   thisNode.panel->resetPanel();

   
  interface.begin();
  thisNode.setStationInfo(STATION_SSID, STATION_PASSWORD);
  thisNode.setMQTTInfo(MQTT_BROKER, MQTT_TOPIC);
  thisNode.amIMQTTBridge(true);
  thisNode.init();
}

void loop() {
  thisNode.update();
}

void interpretDashInstructions(String& msg){
  // Deserialize different dash topics as they are received
  String result;
  std::list<uint32_t> nodesToSend;
  Serial.println(msg);
  // If dash topic is Command
  if(thisNode.dashDeserialize(msg, "Command", result, nodesToSend)){
    // Interpret what command on the dash means
    char numChar = result[0];
    String commandContent = result.substring(1);
    if(numChar == 'G'){
      String topo = thisNode.mesh.subConnectionJson();
      thisNode.sendJsonToDash("Debug", topo);
      return;
    }
    else {
      thisNode.sendOverAllFreqs(nodesToSend, result);
    }
  }
  // If the dash topic is MasterJoystick
  else if(thisNode.dashDeserialize(msg, "MasterJoystick", result, nodesToSend)){
     // Interpret what a dash MasterJoystick message means
     String joystickcontrol = "J";
     joystickcontrol += result;
     thisNode.sendOverAllFreqs(nodesToSend, result);
     return;
  }
}

void vehicleInstruction(char instructionType, String instrData, uint32_t from){
  // Do something with the data receieved over the network
  // Based on what the instructionType is, do something different with the instrData
  // instructionType CAN NOT BE:
  // 'B', 'O', 'L', 'R', 'D', 'A', 'g', 'm', 'u'
  //  I am using those within ArnholdMesh for testing purposes
  String payload;
  std::list<uint32_t> nodeList = thisNode.mesh.getNodeList();
  std::list<uint32_t>::iterator it;
  switch(instructionType){
    case 'M':
    {
      lastMRecv = millis();
        int temp = instrData.indexOf('a');
      if(temp == -1)
        return;
      leftMotor = instrData.substring(1,temp).toInt();
      rightMotor = instrData.substring(temp+1).toInt();
      return;
    }
      break;
     case 'r':
        lookForBlob = true;
        return;
        break;
     case 'J':
        {
        int space = instrData.indexOf(' ');
        float joyX = ((instrData.substring(0, space)).toFloat());
        float joyY = ((instrData.substring(space+1, instrData.length())).toFloat());
        #if defined(PIVOT)
          pivotCommand(joyX, joyY);
        #endif 
        #if defined(DIFFERENTIAL)
          differentialCommand(joyX, joyY);
        #endif 
        break;
        }
     case 'S':
        {
        int space = instrData.indexOf(' ');
        int servoNum = ((instrData.substring(0, space)).toInt());
        int midpoint = ((instrData.substring(space+1, instrData.length())).toInt());
        calibrateServo(servoNum, midpoint);
        break;    
        }
     case 'Z':
        {
        int space = instrData.indexOf(' ');
        float theta = ((instrData.substring(0, space)).toFloat());
        handleOrientationPos(theta);
        break;    
        }
     case 'Y':
        {
        int space = instrData.indexOf(' ');
        float distance = ((instrData.substring(0, space)).toFloat());
        toDistance(distance);
        break;    
        }
     default:
      break;
  }
}

// Callback function needed for painlessMesh
void receivedCallback(uint32_t from, String &msg) {
  Serial.printf("startHere: Received from %u msg=%s\n", from, msg.c_str());
  thisNode.lastMsgReceieved = millis();
  thisNode.colorsL[2] = 5;
  thisNode.panel->singleLED(1, thisNode.colorsL[0], thisNode.colorsL[1], thisNode.colorsL[2]);
  thisNode.interpretInstruction(from, msg);
}

// Callback function needed for MQTT
void mqttCallback(char* topic, uint8_t* payload, unsigned int length) {
  char* cleanPayload = (char*)malloc(length+1);
  memcpy(cleanPayload, payload, length);
  cleanPayload[length] = '\0';
  String msg = String(cleanPayload); 
  free(cleanPayload);
  Serial.println(msg);
  interpretDashInstructions(msg);
}

void newConnectionCallback(uint32_t nodeId) {
  Serial.printf("--> startHere: New Connection, nodeId = %u\n", nodeId);
  digitalWrite(LED_BUILTIN, HIGH);
  if(thisNode.bridgeId == 0){
      String askpayload = "g";
      thisNode.sendOverAllFreqs(nodeId, askpayload);
  }
}

//detecting apriltags using OpenMV
void exe_apriltags(int8_t id1, results &tag_result){
    int8_t id_nums[1] = {id1};
    //int8_t id_nums[2] = {0, 4};
    if (interface.call(F("apriltags_cx_cy"), id_nums, sizeof(id_nums), &tag_result, sizeof(tag_result))) {
         if(tag_result.cx != 0){
          Serial.println("April Tag Detected");
         }
         else {
          Serial.println("April Tag Not Detected");
         }
    }
}

//PID for following an apriltag
void aprilTagPid(){
  elapsedTime = loopStartTime - millis();
  loopStartTime = millis();
  // ROT PID
  rotPrevError = rotError;
  rotError = (int)tag_result.cx - X_MIDDLE;
  rotPid = rotKp * rotError + rotKd * ((rotError - rotError) / elapsedTime);
  // rotPid is + when aprilTag is on the right
  // rotPid is - when aprilTag is on the left

  // DIST PID
  distPrevError = distError;
  distError = (int)tag_result.ztrans - DIST_MIDDLE;
  distPid = distKp * distError + distKd * ((distError - distError) / elapsedTime);
  // distPid is + when aprilTag is further than 400 mm
  // distPid is - when aprilTag is less than 400 mm away
}

//detecting color blob using OpenMV
void exe_blob(results &tag_result){
  // ﻿(0, 11, 4, 127, -128, 127) threshold
    int16_t dims[2] = {190, 70};
    if (interface.call(F("color_detection_dist"), dims, sizeof(dims), &tag_result, sizeof(tag_result))) {
         if(tag_result.cx != 0){
          Serial.println("blob Detected");
          if(tag_result.ztrans < 0){
            tag_result.ztrans *= -1;
          }
         }
         else {
          Serial.println("blob Not Detected");
         }
    }
}

//PID for following a color blob
void blobPid(){
  elapsedTime = loopStartTime - millis();
  loopStartTime = millis();
  // ROT PID
  rotPrevError = rotError;
  rotError = (int)tag_result.cx - X_MIDDLE;
  rotPid = rotKp * rotError + rotKd * ((rotError - rotError) / elapsedTime);
  // rotPid is + when blob is on the right
  // rotPid is - when blob is on the left

  // DIST PID
  distPrevError = distError;
  distError = (int)tag_result.ztrans - DIST_MIDDLE;
  distPid = distKp * distError + distKd * ((distError - distError) / elapsedTime);
  // distPid is + when blob is further than 400 mm
  // distPid is - when blob is less than 400 mm away
}

//PID USING IMU
void imuPID() {
   imu.filterIMU();
   imuPrevError = imuError;
   imuError = imu.getYaw() - currentAngle;
   imuPid = imuKp * imuError + imuKd * ((imuError - imuError) / elapsedTime); 
   //negative imuPid if car is leaning more towards the right
   //positive imuPid if car is leaning more towards the left
}

void handleOrientationPos(float theta){
  float altTheta = currentAngle + theta -3;
  if(altTheta > 360)
    altTheta -= 360;

    while((altTheta - 2) - imu.getYaw() <= -1.0 || (altTheta - 2) - imu.getYaw() >=  1.0){
    imu.filterIMU();
    servo.left();
    }

  servo.stopS();
  currentAngle = altTheta;
}

void pivotCommand(float x, float y){
  //y is forward and backward throttle
   //x is left/right throttle
   
   if(x <= -0.2 && x >= -1){
    servo.adjustSpeedPWM(servo.midpoint[0] - 85 * y, servo.midpoint[1] + 85 * y, servo.midpoint[2] - 85 * y, servo.midpoint[3] + 85 * y);
    servo.steerPWM(x);
    }   
   else if(x <= 2 && x >= 0.2){
    servo.adjustSpeedPWM(servo.midpoint[0] - 85 * y, servo.midpoint[1] + 85 * y, servo.midpoint[2] - 85 * y, servo.midpoint[3] + 85 * y);
    servo.steerPWM(x);
    }  
   else
    servo.steerPWM(x);
     servo.adjustSpeedPWM(servo.midpoint[0] - 85 * y, servo.midpoint[1] + 85 * y, servo.midpoint[2] - 85 * y, servo.midpoint[3] + 85 * y);
}

void differentialCommand(float x, float y){
  //y is forward and backward throttle
   //x is left/right throttle
   int xVal = 85 * x;
   if(x <= -0.2 && x >= -1)
    servo.adjustSpeedPWM(servo.midpoint[0] - 85 * y + xVal, servo.midpoint[1] + 85 * y, servo.midpoint[2] - 85 * y  + xVal, servo.midpoint[3] + 85 * y);  
   else if(x <= 2 && x >= 0.2)
    servo.adjustSpeedPWM(servo.midpoint[0] - 85 * y, servo.midpoint[1] + 85 * y + xVal, servo.midpoint[2] - 85 * y, servo.midpoint[3] + 85 * y + xVal);
   else
     servo.adjustSpeedPWM(servo.midpoint[0] - 85 * y, servo.midpoint[1] + 85 * y, servo.midpoint[2] - 85 * y, servo.midpoint[3] + 85 * y);
}

void calibrateServo(int servoNum, int midpoint){
  Serial.println("in calibrate servo");
  for(int i = 0; i < 5; ++i){
    if(i != midpoint)
      continue;
    else{
      servo.steerCalib(servoNum, midpoint);
      servo.midpoint[i] = midpoint;
      }
  }
}

//handling x and y (mm/distance)
void toDistance(float d){
//d should be in mm
  float loopIter = d / (WHEELDIAMETER * PI); 

//OPTIONAL: If the car is not stable to drive straight, define USEIMU and DRIVESTRAIGHTIMU in loop()
  for(int i = 0; i < SERVOREVOL * OFFSET * loopIter; ++i)
    servo.forward();
  
  servo.stopS();
}

void aprilTagsTarget(int lSpeed, int rSpeed){
  int mapLeft = map(lSpeed, -255, 255, SERVOMINPWM, SERVOMAXPWM);
  int mapRight = map(rSpeed, -255, 255, SERVOMAXPWM, SERVOMINPWM);
  servo.adjustSpeedPWM(abs(servo.midpoint[0] - MIDPOINTPWM) + mapRight, abs(servo.midpoint[1] - MIDPOINTPWM) + mapLeft, 
                  abs(servo.midpoint[2] - MIDPOINTPWM) + mapRight, abs(servo.midpoint[3] - MIDPOINTPWM) + mapLeft);
 }
