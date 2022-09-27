#pragma once

#ifndef STA_MAXTRIES
#  define STA_MAXTRIES 10
#endif

#include <ESPAsyncWebServer.h>

typedef void (*ws_callback_t)(AsyncWebSocketClient * , uint64_t , uint8_t * );

void defaultBinCallback (AsyncWebSocketClient* client, uint64_t len, uint8_t* data);
void defaultTxtCallback (AsyncWebSocketClient* client, uint64_t len, uint8_t* str);

static const char defaultWSPath[] = "/ws";

class ESPServer {
    public:
        ESPServer(int port = 80, const char* wsPath = defaultWSPath);
        void setupSTA(const char* ssid, const char* password);
        void setupAP(const char* ssid, const char* password);
        void setupWS(ws_callback_t bin = defaultBinCallback, ws_callback_t txt = defaultTxtCallback);
        void serve_gz(const char* page, const char* type, unsigned const char* gz, const unsigned int len);
        void serve_txt(const char* page, const char* txt);
        void serve_progmem(const char* page, const char* txt);
        void start();
        void broadcast(String topic, String& txt);
        void broadcastBinary(String topic, char* bin);

        void loop();
    protected:
        AsyncWebServer _httpServer;
        AsyncWebSocket _ws;
        ws_callback_t _binCallback;
        ws_callback_t _txtCallback;
        void _wsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type,
             void * arg, uint8_t *data, size_t len);
};
