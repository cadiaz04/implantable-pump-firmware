#pragma once
#include <ArduinoJson.h>
#include "pumps/pump_profile.h"

void profileToJson(const PumpProfile& p, JsonObject o);
// Validates as it parses. Returns false with a reason in `err`.
bool profileFromJson(JsonObjectConst o, PumpProfile& p, String& err);

void calibrationToJson(const Calibration& c, JsonObject o);
bool calibrationFromJson(JsonObjectConst o, Calibration& c, String& err);
void fitToJson(const CurveFit& f, JsonObject o);
