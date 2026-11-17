#pragma once
// REST API (/api/*) and WebSocket (/ws) — see docs/API.md.
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>

namespace api {
void begin(AsyncWebServer& server, AsyncWebSocket& ws);
void startTelemetryTask(AsyncWebSocket& ws);

// Shared by REST and WebSocket control messages. Returns nullptr on success,
// otherwise an error message. `code` receives an HTTP status.
const char* channelAction(int ch, const char* action, JsonVariantConst body, int& code);
}  // namespace api
