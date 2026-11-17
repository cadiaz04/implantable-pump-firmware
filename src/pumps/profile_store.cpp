#include "profile_store.h"
#include <ArduinoJson.h>
#include <LittleFS.h>
#include "pumps/profile_json.h"

ProfileStore g_profiles;

namespace {
struct Guard {
    SemaphoreHandle_t m;
    explicit Guard(SemaphoreHandle_t mm) : m(mm) { xSemaphoreTake(m, portMAX_DELAY); }
    ~Guard() { xSemaphoreGive(m); }
};
}  // namespace

bool ProfileStore::validId(const String& id) {
    if (id.isEmpty() || id.length() > 40) return false;
    for (char c : id) {
        if (!(isalnum(static_cast<unsigned char>(c)) || c == '-' || c == '_')) return false;
    }
    return true;
}

bool ProfileStore::begin() {
    mtx_ = xSemaphoreCreateMutex();
    if (!LittleFS.begin(true)) {   // format on first use
        Serial.println("[fs] LittleFS mount failed");
        return false;
    }
    if (!LittleFS.exists("/pumps")) LittleFS.mkdir("/pumps");
    rebuildIndex();
    if (index_.empty()) {
        Serial.println("[fs] no pump profiles: creating starter templates");
        save(starterProfile28byj());
        save(starterProfileNema17());
        save(starterProfileDcDiaphragm());
    }
    return true;
}

void ProfileStore::indexPut(const PumpProfile& p) {
    ProfileSummary s;
    s.id = p.id.c_str();
    s.name = p.name.c_str();
    s.pumpType = p.pumpType.c_str();
    s.driver = p.motor.driverType;
    s.calibrations = static_cast<int>(p.calibrations.size());
    if (const Calibration* c = p.activeCalibration()) s.activeCalibration = c->name.c_str();
    for (auto& e : index_) {
        if (e.id == s.id) {
            e = s;
            return;
        }
    }
    index_.push_back(s);
}

void ProfileStore::rebuildIndex() {
    std::vector<String> ids, stale;
    File dir = LittleFS.open("/pumps");
    if (dir && dir.isDirectory()) {
        for (File f = dir.openNextFile(); f; f = dir.openNextFile()) {
            String n = f.name();
            const int slash = n.lastIndexOf('/');
            if (slash >= 0) n = n.substring(slash + 1);
            if (n.endsWith(".json")) ids.push_back(n.substring(0, n.length() - 5));
            else if (n.endsWith(".tmp")) stale.push_back("/pumps/" + n);
        }
    }
    dir.close();
    for (const auto& path : stale) LittleFS.remove(path);   // leftovers from an interrupted write
    Guard g(mtx_);
    index_.clear();
    for (const auto& id : ids) {
        PumpProfile p;
        String err;
        if (load(id, p, &err)) indexPut(p);
        else Serial.printf("[fs] skipping %s: %s\n", id.c_str(), err.c_str());
    }
}

std::vector<ProfileSummary> ProfileStore::list() {
    Guard g(mtx_);
    return index_;
}

bool ProfileStore::exists(const String& id) {
    Guard g(mtx_);
    for (auto& e : index_)
        if (e.id == id) return true;
    return false;
}

bool ProfileStore::load(const String& id, PumpProfile& out, String* err) {
    if (!validId(id)) {
        if (err) *err = "invalid id";
        return false;
    }
    File f = LittleFS.open(pathFor(id), "r");
    if (!f) {
        if (err) *err = "not found";
        return false;
    }
    JsonDocument doc;
    DeserializationError de = deserializeJson(doc, f);
    f.close();
    if (de) {
        if (err) *err = de.c_str();
        return false;
    }
    String e;
    if (!profileFromJson(doc.as<JsonObjectConst>(), out, e)) {
        if (err) *err = e;
        return false;
    }
    out.id = id.c_str();
    return true;
}

bool ProfileStore::save(const PumpProfile& p, String* err) {
    const String id = p.id.c_str();
    if (!validId(id)) {
        if (err) *err = "invalid id";
        return false;
    }
    JsonDocument doc;
    profileToJson(p, doc.to<JsonObject>());
    const String tmp = "/pumps/" + id + ".tmp";
    File f = LittleFS.open(tmp, "w");
    if (!f) {
        if (err) *err = "cannot open file";
        return false;
    }
    const size_t n = serializeJson(doc, f);
    f.close();
    if (n == 0) {
        LittleFS.remove(tmp);
        if (err) *err = "write failed (filesystem full?)";
        return false;
    }
    if (!LittleFS.rename(tmp, pathFor(id))) {
        // Older LittleFS builds refuse to rename over an existing file.
        LittleFS.remove(pathFor(id));
        if (!LittleFS.rename(tmp, pathFor(id))) {
            if (err) *err = "rename failed";
            return false;
        }
    }
    Guard g(mtx_);
    indexPut(p);
    return true;
}

bool ProfileStore::remove(const String& id) {
    if (!validId(id)) return false;
    const bool ok = LittleFS.remove(pathFor(id));
    Guard g(mtx_);
    for (size_t i = 0; i < index_.size(); ++i) {
        if (index_[i].id == id) {
            index_.erase(index_.begin() + i);
            break;
        }
    }
    return ok;
}

String ProfileStore::uniqueId(const String& name) {
    String base;
    for (char c : name) {
        if (isalnum(static_cast<unsigned char>(c))) base += static_cast<char>(tolower(c));
        else if (!base.isEmpty() && !base.endsWith("-")) base += '-';
        if (base.length() >= 28) break;
    }
    while (base.endsWith("-")) base.remove(base.length() - 1);
    if (base.isEmpty()) base = "pump";
    String id = base;
    for (int i = 2; exists(id) && i < 1000; ++i) id = base + "-" + String(i);
    return id;
}
