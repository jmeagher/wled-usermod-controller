// Desktop tests for ctl_json.h against the ArduinoJson version WLED bundles (6.18.1). Build and run with tests/run.sh,
// which fetches that exact header (checksum-verified) when no WLED checkout provides it.
#define ARDUINOJSON_DECODE_UNICODE 0   // as in wled.h
#include "ArduinoJson-v6.18.1.h"

#include <cstdio>
#include <string>
#include <vector>

#include "../ctl_json.h"

static int failures = 0;
#define CHECK(cond) do { if (!(cond)) { std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); failures++; } } while (0)
#define CHECK_EQ(a, b) do { long _a = (long)(a), _b = (long)(b); if (_a != _b) { \
  std::printf("FAIL %s:%d  %s == %s  (%ld vs %ld)\n", __FILE__, __LINE__, #a, #b, _a, _b); failures++; } } while (0)

struct Preset { uint8_t id; char name[24]; bool scene; };

// A presets.json like a busy device's: `playlists` playlists of `steps` entries each (WLED allows 100), plus plain
// presets with a full segment state each, so the unfiltered file is far larger than the document.
static std::string bigPresets(int playlists, int steps, int plain) {
  std::string s = "{\"0\":{}";
  int id = 1;
  for (int p = 0; p < playlists; p++, id++) {
    s += ",\"" + std::to_string(id) + "\":{\"n\":\"Playlist number " + std::to_string(p) + "\",\"on\":true,\"playlist\":{";
    const char* keys[] = {"ps", "dur", "transition"};
    for (int k = 0; k < 3; k++) {
      s += std::string(k ? "," : "") + "\"" + keys[k] + "\":[";
      for (int i = 0; i < steps; i++) s += (i ? "," : "") + std::to_string(100 + i % 50);
      s += "]";
    }
    s += ",\"repeat\":0,\"end\":0,\"r\":false}}";
  }
  for (int p = 0; p < plain; p++, id++) {
    s += ",\"" + std::to_string(id) + "\":{\"n\":\"Plain preset " + std::to_string(p) + "\"";
    if (p % 3 == 0) s += ",\"ql\":\"Q" + std::to_string(p % 10) + "\"";
    s += ",\"on\":true,\"bri\":128,\"seg\":[{\"id\":0,\"start\":0,\"stop\":50,\"col\":[[255,0,0],[0,0,0],[0,0,0]],"
         "\"fx\":9,\"sx\":128,\"ix\":128,\"pal\":0,\"sel\":true,\"rev\":false,\"mi\":false}]}";
  }
  return s + "}";
}

// Regression: with the old filter (playlist.ps kept) a file with big playlists overflowed the 24 KB document.
static void testOldFilterOverflowed() {
  const std::string json = bigPresets(40, 100, 60);
  StaticJsonDocument<256> filter;
  filter["*"]["n"] = true;
  filter["*"]["playlist"]["ps"] = true;
  filter["*"]["ql"] = true;
  DynamicJsonDocument doc(ctl::kPresetDocBytes);
  CHECK(deserializeJson(doc, json, DeserializationOption::Filter(filter)) == DeserializationError::NoMemory);
}

static void testFilterKeepsOnlyWhatIsNeeded() {
  const std::string json = bigPresets(40, 100, 60);
  StaticJsonDocument<256> filter;
  ctl::presetFilter(filter);
  CHECK(!filter.overflowed());
  DynamicJsonDocument doc(ctl::kPresetDocBytes);
  const DeserializationError err = deserializeJson(doc, json, DeserializationOption::Filter(filter));
  CHECK(err == DeserializationError::Ok);
  CHECK(doc.memoryUsage() < ctl::kPresetDocBytes / 2);
  std::vector<Preset> list;
  ctl::presetsFromJson(doc.as<JsonObjectConst>(), list);
  CHECK_EQ(list.size(), 100);
  size_t scenes = ctl::orderPresets(list);
  CHECK_EQ(scenes, 40 + 20);           // every playlist, plus the plain presets with a quick-load label
  CHECK_EQ(list[0].id, 1);
  CHECK(!strcmp(list[0].name, "Playlist number 0"));
  // the playlist arrays are gone, the key remains as an empty object
  CHECK(doc["1"]["playlist"].is<JsonObject>());
  CHECK_EQ(doc["1"]["playlist"].size(), 0);
  CHECK(doc["41"]["seg"].isNull());
}

// The largest list the device allows: 250 presets with names at the 23-character limit, all of them playlists.
static void testWorstCaseFits() {
  std::string s = "{";
  for (int id = 1; id <= 250; id++) {
    s += std::string(id > 1 ? "," : "") + "\"" + std::to_string(id) + "\":{\"n\":\"Name that is 23 chars!!\",\"ql\":\"AB\","
         "\"playlist\":{\"ps\":[1,2,3,4,5,6,7,8,9,10],\"dur\":[1,1,1,1,1,1,1,1,1,1]}}";
  }
  s += "}";
  StaticJsonDocument<256> filter;
  ctl::presetFilter(filter);
  DynamicJsonDocument doc(ctl::kPresetDocBytes);
  CHECK(deserializeJson(doc, s, DeserializationOption::Filter(filter)) == DeserializationError::Ok);
  std::vector<Preset> list;
  ctl::presetsFromJson(doc.as<JsonObjectConst>(), list);
  CHECK_EQ(list.size(), 250);
}

static void testSceneDetection() {
  const char* json = "{\"0\":{},\"1\":{\"n\":\"A\",\"playlist\":{\"ps\":[2],\"end\":0}},\"2\":{\"n\":\"B\",\"ql\":\"x\"},"
                     "\"3\":{\"n\":\"C\",\"playlist\":[1]},\"4\":{},\"5\":{\"ql\":\"\"},\"251\":{\"n\":\"too high\"},"
                     "\"x\":{\"n\":\"not an id\"},\"6\":7}";
  StaticJsonDocument<256> filter;
  ctl::presetFilter(filter);
  DynamicJsonDocument doc(2048);
  CHECK(deserializeJson(doc, json, DeserializationOption::Filter(filter)) == DeserializationError::Ok);
  std::vector<Preset> list;
  ctl::presetsFromJson(doc.as<JsonObjectConst>(), list);
  CHECK_EQ(list.size(), 5);            // 1..5; 0, 251, "x" and the non-object 6 are skipped
  ctl::orderPresets(list);
  CHECK(list[0].id == 1 && list[0].scene);   // playlist object
  CHECK(list[1].id == 2 && list[1].scene);   // quick-load label
  CHECK(list[2].id == 3 && !list[2].scene);  // "playlist" that is not an object is not a playlist
  CHECK(list[3].id == 4 && !list[3].scene && !strcmp(list[3].name, "Preset 4"));
  CHECK(list[4].id == 5 && !list[4].scene);  // an empty label does not count
}

// When even the filtered list does not fit, ArduinoJson keeps what it parsed: the controller shows that part
// (flagged as incomplete) instead of nothing.
static void testNoMemoryKeepsPartialList() {
  const std::string json = bigPresets(0, 0, 100);
  StaticJsonDocument<256> filter;
  ctl::presetFilter(filter);
  DynamicJsonDocument doc(1024);
  CHECK(deserializeJson(doc, json, DeserializationOption::Filter(filter)) == DeserializationError::NoMemory);
  std::vector<Preset> list;
  ctl::presetsFromJson(doc.as<JsonObjectConst>(), list);
  CHECK(list.size() > 3 && list.size() < 100);
  CHECK_EQ(list[0].id, 1);
}

int main() {
  testOldFilterOverflowed();
  testFilterKeepsOnlyWhatIsNeeded();
  testWorstCaseFits();
  testSceneDetection();
  testNoMemoryKeepsPartialList();
  if (failures) { std::printf("%d check(s) failed\n", failures); return 1; }
  std::printf("all JSON tests passed\n");
  return 0;
}
