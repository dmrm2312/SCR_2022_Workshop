#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

#include "debug.h"
#include "server.h"


void defaultBinCallback (AsyncWebSocketClient * client, uint64_t len, uint8_t* bytes) {
    DEBUGF("ws RX[%u] binary message of length %llu \r\n", client->id(), len)
    client->binary("I got your binary message");
}

void defaultTxtCallback (AsyncWebSocketClient * client, uint64_t len, uint8_t* str) {
    // Prevent buffer overflow with poorly formatted strings
    str[len] = 0;

    DEBUGF("ws: RX[%u] text message of length %llu: \r\n  %s \r\n", client->id(), len, (char*)str)
    client->text("I got your text message");
}

ESPServer::ESPServer(int port, const char* wsPath) :
    _httpServer(port), _ws(wsPath) { }

void ESPServer::setupSTA(const char* ssid, const char* password) {
    DEBUG("Connecting to STA");
    WiFi.begin(ssid, password);

    int tries = 0;
    while (WiFi.status() != WL_CONNECTED) {
        if (tries++ > STA_MAXTRIES) {
            DEBUG("  giving up.");
            return;
        }
        delay(500);
        DEBUG("  ... waiting");
    }

    IPAddress myIP = WiFi.localIP();
    DEBUG("STA IP address: ", myIP.toString());
}

void ESPServer::setupAP(const char* ssid, const char* password) {
    DEBUG("Setting up AP");
    WiFi.softAP(ssid, password);

    IPAddress myIP = WiFi.softAPIP();
    DEBUG("AP IP address: ", myIP.toString());
}

void ESPServer::setupWS(ws_callback_t bin, ws_callback_t txt) {
    _binCallback = bin;
    _txtCallback = txt;
}

void ESPServer::start() {
    _ws.onEvent([this](AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type,
             void * arg, uint8_t *data, size_t len) {
        this->_wsEvent(server, client, type, arg, data, len);
    });
    _httpServer.addHandler(&_ws);
    _httpServer.begin();
}

void ESPServer::serve_gz(const char* page, const char* type, const unsigned char* gz, const unsigned int len) {
    _httpServer.on(page, HTTP_GET, [type, gz, len](AsyncWebServerRequest *request) {
        AsyncWebServerResponse *response = request->beginResponse_P(200, type, gz, len);
        response->addHeader("Content-Encoding", "gzip");
        request->send(response);
    });
}

void ESPServer::serve_txt(const char* page, const char* txt) {
    _httpServer.on(page, HTTP_GET, [txt](AsyncWebServerRequest *request) {
        request->send(200, "text/html", txt);
    });
}

void ESPServer::serve_progmem(const char* page, const char* txt) {
    _httpServer.on(page, HTTP_GET, [txt](AsyncWebServerRequest *request) {
        request->send_P(200, "text/html", txt);
    });
}

void ESPServer::broadcast(String topic, String& txt) { 

    Serial.println(txt);
    _ws.textAll(txt);
} ;


void ESPServer::broadcastBinary(String topic, char* bin) { 
    _ws.binaryAll(bin);
} ;




#define WSDEBUG(txt) \
    DEBUGF("ws[%s][%u] " txt "\r\n", server->url(), client->id())

#define WSDEBUGF(txt, ...) \
    DEBUGF("ws[%s][%u] " txt "\r\n", server->url(), client->id(), __VA_ARGS__)

void ESPServer::_wsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, 
             void * arg, uint8_t *data, size_t len) {

    switch (type) {
        case WS_EVT_CONNECT:
            //client connected
            WSDEBUG("connect");
            client->printf("Hello Client %u :)", client->id());
            client->ping();
            break;

        case WS_EVT_DISCONNECT:
            //client disconnected
            WSDEBUG("disconnect");
            break;

        case WS_EVT_ERROR:
            //error was received from the other end
            WSDEBUGF("error(%u): %s", *((uint16_t*) arg), (char*) data);
            break;

        case WS_EVT_PONG:
            //pong message was received (in response to a ping request maybe)
            WSDEBUGF("pong[%u]: %s", len, len ? (char*) data : "");
            break;

        case WS_EVT_DATA:
            //data packet
            AwsFrameInfo * info = (AwsFrameInfo*) arg;

            if (info->final && info->index == 0 && info->len == len) {
                // the whole message is in a single frame and we got all of it's data
                WSDEBUGF("%s-message[%llu]: ", (info->opcode == WS_TEXT) ? "txt" : "bin", info->len);
                if (info->opcode == WS_TEXT)
                    _txtCallback(client, info->len, data);
                else
                    _binCallback(client, info->len, data);

            } else { 
                // the message is comprised of multiple frames or the frame is split into multiple packets
                // TODO: Assemble then call callbacks?  
                // Or maybe have partial callbacks to prevent runaway memory

                if(info->index == 0){
                    if(info->num == 0)
                        WSDEBUGF("%s-message start\n", (info->message_opcode == WS_TEXT)?"text":"binary");
                    WSDEBUGF("frame[%u] start[%llu]\n", info->num, info->len);
                }

                WSDEBUGF("frame[%u] %s[%llu - %llu]: ", info->num, (info->message_opcode == WS_TEXT)?"text":"binary", info->index, info->index + len);
                if(info->message_opcode == WS_TEXT){
                    data[len] = 0;
                    DEBUG((char*)data);
                } else {
                    for(size_t i=0; i < len; i++){
                        DEBUGF("%02x ", data[i]);
                    }
                }

                if((info->index + len) == info->len){
                    WSDEBUGF("frame[%u] end[%llu]\n", info->num, info->len);
                    if(info->final){
                        WSDEBUGF("%s-message end\n", (info->message_opcode == WS_TEXT)?"text":"binary");
                        if(info->message_opcode == WS_TEXT)
                            client->text("I got your text message");
                        else
                            client->binary("I got your binary message");
                    }
                }
            }
        // end case
    }
}
