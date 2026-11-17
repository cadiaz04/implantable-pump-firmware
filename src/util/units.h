#pragma once
// Unit helpers. Internally all flow is µL/min, volume µL, time s.
constexpr float ulMinToMlMin(float ul) { return ul / 1000.0f; }
constexpr float mlMinToUlMin(float ml) { return ml * 1000.0f; }
constexpr float ulMinToMlHr(float ul) { return ul * 60.0f / 1000.0f; }
constexpr float mlHrToUlMin(float mlhr) { return mlhr * 1000.0f / 60.0f; }
