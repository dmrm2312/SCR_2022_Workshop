#include <WiFi.h>
#include "WebSocketsServer.h" //Make these angle brackets again?
// #include <ESP32Servo.h>

#include "Adafruit_PWMServoDriver.h"
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150
#define SERVOMAX  410
#define SERVO_FREQ 50
#define LOW_DEAD 85
#define HIGH_DEAD 95


// Constants
const char* ssid     = "*****";
const char* password = "****";
String hostname = "ESP_05";
int left_conv;
int right_conv;
int left_flag;
int right_flag;

// Globals
WebSocketsServer webSocket = WebSocketsServer(80);
//WiFiServer server(80); //Necessary?

void setup() {

  // Start Serial port
  Serial.begin(115200);

  // Connect to access point
  Serial.println("Connecting");
  WiFi.setHostname(hostname.c_str()); //define hostname
  WiFi.begin(ssid, password);
  while ( WiFi.status() != WL_CONNECTED ) {
    delay(500);
    Serial.print(".");
  }

  // Print our IP address
  Serial.println("Connected!");
  Serial.print("My IP address: ");
  Serial.println(WiFi.localIP());

  // Start WebSocket server and assign callback
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);

  //Initialize PWM
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
}

void loop() {

  // Look for and handle WebSocket data
  webSocket.loop();
}


// SWAP SERVOMAX AND SERVOMIN BASED ON FORWARD ORIENTATION
void drive(int left, int right) {

  left_flag = 0;
  right_flag = 0;
  if (left >= LOW_DEAD){
    if (left <= HIGH_DEAD){
      pwm.setPWM(1, 0, 0);
      pwm.setPWM(3, 0, 0);
      pwm.setPWM(4, 0, 0);
      pwm.setPWM(6, 0, 0);
      left_flag = 1;
    }
    }
  if (right >= LOW_DEAD){
    if (right <= HIGH_DEAD){
      pwm.setPWM(0, 0, 0);
      pwm.setPWM(2, 0, 0);
      pwm.setPWM(5, 0, 0);
      pwm.setPWM(7, 0, 0);
      right_flag = 1;
    }
    }

  if (right_flag == 0){
    right_conv = map(right, 0, 180, SERVOMAX, SERVOMIN);
    pwm.setPWM(0, 0, right_conv);
    pwm.setPWM(2, 0, right_conv);
    pwm.setPWM(4, 0, right_conv);
    pwm.setPWM(6, 0, right_conv);
  }
  if (left_flag == 0){
    left_conv = map(left, 0, 180, SERVOMIN, SERVOMAX);
    pwm.setPWM(1, 0, left_conv);
    pwm.setPWM(3, 0, left_conv);
    pwm.setPWM(5, 0, left_conv);
    pwm.setPWM(7, 0, left_conv);
  }

}


// Called when receiving any WebSocket message
void onWebSocketEvent(uint8_t num,
                      WStype_t type,
                      uint8_t * payload,
                      size_t length) {

  // Figure out the type of WebSocket event
  switch(type) {

    // Client has disconnected
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      break;

    // New client has connected
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connection from ", num);
        Serial.println(ip.toString());
        break;
      }
      

    case WStype_BIN:
            // DEBUG("On connection #", num)
            Serial.printf("[%u] Connection from ", num);
            // DEBUG("  got binary of length ", length);
            Serial.printf("[%u] Got binary of length ", length);
            for (int i = 0; i < length; i++)
              // DEBUG("    char : ", payload[i]);
              Serial.printf("[%u]     char :  ", payload[i]);

            if (payload[0] == '~') 
              drive(180-payload[1], payload[2]);


    // Echo text message back to client
    case WStype_TEXT:
      Serial.printf("[%u] Text: %s\n", num, payload);
      webSocket.sendTXT(num, payload);

      //...BUT ACTIVATED BY WEB SOCKET TEXT COMMANDS LIKE PAPERBOT
      if (payload[0] == '#') {
          if(payload[1] == 'F') {
            pwm.setPWM(1, 0, SERVOMAX);
            pwm.setPWM(0, 0, SERVOMIN);
            pwm.setPWM(3, 0, SERVOMAX);
            pwm.setPWM(2, 0, SERVOMIN);
          }
          else if(payload[1] == 'B') {
            pwm.setPWM(0, 0, SERVOMAX);
            pwm.setPWM(1, 0, SERVOMIN);
            pwm.setPWM(2, 0, SERVOMAX);
            pwm.setPWM(3, 0, SERVOMIN);
          }
          else if(payload[1] == 'L') {
            for(int i = 0; i < 210; ++i) {
              pwm.setPWM(1, 0, SERVOMIN);
              pwm.setPWM(2, 0, SERVOMIN);
              pwm.setPWM(0, 0, SERVOMIN);
              pwm.setPWM(3, 0, SERVOMIN);
            }
            pwm.setPWM(1, 0, 0);
            pwm.setPWM(0, 0, 0);
            pwm.setPWM(3, 0, 0);
            pwm.setPWM(2, 0, 0);
          }
          else if(payload[1] == 'R') {
            for(int i = 0; i < 250; ++i) {
              pwm.setPWM(1, 0, SERVOMAX);
              pwm.setPWM(2, 0, SERVOMAX);
              pwm.setPWM(0, 0, SERVOMAX);
              pwm.setPWM(3, 0, SERVOMAX);
            }
            pwm.setPWM(1, 0, 0);
            pwm.setPWM(0, 0, 0);
            pwm.setPWM(3, 0, 0);
            pwm.setPWM(2, 0, 0);
          }
          else if (payload[1] == 'S') {
              pwm.setPWM(1, 0, 0);
              pwm.setPWM(0, 0, 0);
              pwm.setPWM(3, 0, 0);
              pwm.setPWM(2, 0, 0);
          }
          else {
              pwm.setPWM(1, 0, 0);
              pwm.setPWM(0, 0, 0);
              pwm.setPWM(3, 0, 0);
              pwm.setPWM(2, 0, 0);
          }
      }
      if (payload[0] == '~') 
        drive(payload[1], payload[2]);
        // Full speed forward payload 0, 0
        // Full speed reverse payload 180, 180
        // Full speed reverse payload 90, 90        

      // 

      break;

    // For everything else: do nothing
    case WStype_ERROR:
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:
    default:
      break;
  }
}
