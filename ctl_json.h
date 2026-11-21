// JSON handling of the controller usermod that does not need WLED or Arduino: only ArduinoJson 6 (WLED bundles
// 6.18.1; include it before this header). tests/test_json.cpp runs it on a desktop against the same version.
#pragma once
#include <vector>

#include "ctl_logic.h"

namespace ctl {

// Bytes of DynamicJsonDocument for a remote preset list, allocated only while a list is fetched. With presetFilter()
// a preset costs up to ~130 bytes (its key, name, label, an empty playlist object), so the worst case WLED allows,
// 250 presets with 23-character names and labels, fits (tests/test_json.cpp checks it).
constexpr size_t kPresetDocBytes = 36864;

// Filter for presets.json: per preset only the name ("n"), the quick-load label ("ql") and whether it is a playlist.
// "playlist" maps to an EMPTY object, which keeps the key as {} when the value is an object and drops all its members.
// The old filter kept playlist.ps, an array of up to 100 preset ids per playlist, and a few large playlists filled
// the document: deserializeJson failed with NoMemory, the whole list was dropped and downloaded again every 10 s.
inline void presetFilter(JsonDocument& f) {
  JsonObject p = f.createNestedObject("*");
  p["n"] = true;
  p["ql"] = true;
  p.createNestedObject("playlist");
}

// Fill `out` from a (filtered) presets.json object: ids 1..250, name or "Preset <id>", scene flag. P needs
// `uint8_t id`, `char name[N]` and `bool scene`. The order is presets.json's; call orderPresets() afterwards.
template <class P> void presetsFromJson(JsonObjectConst root, std::vector<P>& out) {
  out.clear();
  for (JsonPairConst kv : root) {
    const int id = atoi(kv.key().c_str());
    if (id <= 0 || id > 250) continue;
    JsonObjectConst v = kv.value().as<JsonObjectConst>();
    if (v.isNull()) continue;                  // "0": {} and other non-presets
    P p{};
    p.id = uint8_t(id);
    p.scene = isScene(v["playlist"].is<JsonObjectConst>(), v["ql"] | "");
    const char* n = v["n"] | "";
    if (*n) snprintf(p.name, sizeof(p.name), "%s", n);
    else snprintf(p.name, sizeof(p.name), "Preset %d", id);
    out.push_back(p);
  }
}

}  // namespace ctl
