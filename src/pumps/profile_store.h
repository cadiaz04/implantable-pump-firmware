#pragma once
// =============================================================================
// ProfileStore — pump profiles on LittleFS (/pumps/<id>.json).
//
// Writes are atomic: the JSON is written to <id>.tmp and renamed over the
// old file, so a power cut never leaves a half-written profile. An in-RAM
// index (id, name, type, driver, #calibrations) serves list requests without
// touching flash. Only the storage task writes; any task may read.
// =============================================================================
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <vector>
#include "pumps/pump_profile.h"

struct ProfileSummary {
    String id;
    String name;
    String pumpType;
    DriverType driver = DriverType::NONE;
    int calibrations = 0;
    String activeCalibration;
};

class ProfileStore {
public:
    bool begin();   // mounts LittleFS, creates starter profiles if empty
    std::vector<ProfileSummary> list();
    bool exists(const String& id);
    bool load(const String& id, PumpProfile& out, String* err = nullptr);
    bool save(const PumpProfile& p, String* err = nullptr);
    bool remove(const String& id);
    String uniqueId(const String& name);
    static String pathFor(const String& id) { return "/pumps/" + id + ".json"; }
    static bool validId(const String& id);

private:
    void rebuildIndex();
    void indexPut(const PumpProfile& p);
    SemaphoreHandle_t mtx_ = nullptr;
    std::vector<ProfileSummary> index_;
};

extern ProfileStore g_profiles;
