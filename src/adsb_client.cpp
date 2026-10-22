// Fetch nearby aircraft from adsb.lol's re-api (fallback: airplanes.live) and parse the
// readsb JSON into a vector<Aircraft>.
//
// Memory safety (important on the ESP32): we parse straight from the HTTP stream
// (no full-body String), use an ArduinoJson field filter so only the ~12 fields we
// need are kept, and hard-cap the number of aircraft (ADSB_MAX_AIRCRAFT). The radar
// then keeps only the nearest ~20 for display.
#include "adsb_client.h"
#include <ctype.h>
#include "config.h"
#include "geo.h"           // haversineKm — keep the nearest N aircraft
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>   // v7
#include <esp_heap_caps.h>

// Parse the JSON in PSRAM, not internal RAM. Otherwise the per-poll JSON alloc/free
// churn fragments the internal heap and, after a while, mbedTLS can't find a large
// enough contiguous block for the TLS handshake (-32512), freezing the feed.
struct PsramJsonAllocator : ArduinoJson::Allocator {
    void* allocate(size_t n) override { return heap_caps_malloc(n, MALLOC_CAP_SPIRAM); }
    void  deallocate(void* p) override { heap_caps_free(p); }
    void* reallocate(void* p, size_t n) override { return heap_caps_realloc(p, n, MALLOC_CAP_SPIRAM); }
};
static PsramJsonAllocator s_jsonPsram;

// NetworkClient::readBytes() treats a transient negative TLS read as end-of-input,
// which makes ArduinoJson intermittently report IncompleteInput. Deliberately wrap
// the client without overriding readBytes(): Stream's timed byte reader retries
// temporary no-data reads until the configured timeout.
class ReliableJsonStream : public Stream {
public:
    explicit ReliableJsonStream(Stream& source) : _source(source) {}
    int available() override { return _source.available(); }
    int read() override {
        const int value = _source.read();
        if (value >= 0) ++_bytesRead;
        return value;
    }
    int peek() override { return _source.peek(); }
    void flush() override { _source.flush(); }
    size_t write(uint8_t) override { return 0; }
    size_t bytesRead() const { return _bytesRead; }

private:
    Stream& _source;
    size_t _bytesRead = 0;
};

void AdsbClient::begin(double homeLat, double homeLon, float rangeKm) {
    _lat = homeLat; _lon = homeLon; _rangeKm = rangeKm;
}

void AdsbClient::setApiKey(const char* k) {
    _apiKey = k ? k : "";
    _apiKey.trim();
    _reapiBackoff = false;                 // a new key deserves an immediate try
    const unsigned len = _apiKey.length();
    if (len) snprintf(_keyDesc, sizeof(_keyDesc), "key ...%s", _apiKey.c_str() + (len > 4 ? len - 4 : 0));
    else     snprintf(_keyDesc, sizeof(_keyDesc), "no key");
}

bool AdsbClient::poll(std::vector<Aircraft>& out) {
    if (WiFi.status() != WL_CONNECTED) return false;
    // Counts consecutive polls where the primary answered normally but with no aircraft.
    // Only ever touched from adsb_task, so a plain static is fine.
    static uint32_t s_emptyStreak = 0;

    // Your own receiver wins whenever it is configured and answering: no internet round
    // trip, sub-second data, and it shows whatever your antenna hears.
    if (_localHost.length() && _srcMode != 2) {
        std::vector<Aircraft> localList;
        const bool ok = fetchFrom(_localHost.c_str(), localList, FeedKind::Local);
        if (ok && !localList.empty()) {
            s_emptyStreak = 0;
            _lastWasLocal = true;
            _lastSource = _localHost.c_str();
            out.swap(localList);
            return true;
        }
        // Local-only is a deliberate choice, so honour it even when the answer is "nothing".
        // An empty sky over your own antenna is a real result; quietly topping it up from
        // the APIs would make the receiver look better than it is.
        if (_srcMode == 1) {
            _lastWasLocal = ok;
            if (ok) { out.swap(localList); return true; }
            return false;
        }
    }
    _lastWasLocal = false;
    if (_srcMode == 1) return false;          // local-only with no host configured

    // Public sources in priority order. re-api is only offered while its rate window is
    // open. The gate lives here rather than in the caller so that nothing -- a zoom
    // re-query, a failed local poll, a retry -- can push it past 10 requests/minute.
    const uint32_t nowMs = millis();
    if (_reapiBackoff && nowMs - _reapiBackoffStart >= ADSB_REAPI_BACKOFF_MS)
        _reapiBackoff = false;
    const bool reapiDue = !_reapiBackoff &&
                          (!_reapiArmed || nowMs - _reapiLastMs >= ADSB_REAPI_MIN_INTERVAL_MS);

    struct Src { const char* host; FeedKind kind; };
    Src srcs[2];
    int n = 0;
    if (_publicOrder == 1) {                     // airplanes.live first
        srcs[n++] = { ADSB_FALLBACK_HOST, FeedKind::Point };
        if (reapiDue) srcs[n++] = { ADSB_REAPI_HOST, FeedKind::ReApi };
    } else {                                     // re-api first (default)
        if (reapiDue) srcs[n++] = { ADSB_REAPI_HOST, FeedKind::ReApi };
        srcs[n++] = { ADSB_FALLBACK_HOST, FeedKind::Point };
    }

    // Walk the list until a source returns aircraft.
    //
    // A source *error* always earns a try of the next one. A source that answered cleanly
    // with zero aircraft usually means the sky really is empty -- and that state can hold
    // all night, so probing the fallbacks every poll would sustain extra load on free
    // non-commercial APIs. Probe on the first empty (to catch a feed that has gone silently
    // quiet), then only every ADSB_EMPTY_RECHECK_POLLS. The streak resets whenever any
    // source returns aircraft.
    std::vector<Aircraft> emptyList;
    bool anyOk = false;
    const char* emptyHost = "";
    for (int i = 0; i < n; ++i) {
        if (anyOk && (s_emptyStreak % ADSB_EMPTY_RECHECK_POLLS) != 0) break;

        if (srcs[i].kind == FeedKind::ReApi) {     // the attempt itself counts toward the limit
            _reapiArmed = true;
            _reapiLastMs = nowMs;
        }
        std::vector<Aircraft> list;
        const bool ok = fetchFrom(srcs[i].host, list, srcs[i].kind);

        if (srcs[i].kind == FeedKind::ReApi && !ok &&
            (_lastHttpCode == 429 || _lastHttpCode == 403 || _lastHttpCode == 401)) {
            _reapiBackoff = true;
            _reapiBackoffStart = nowMs;
            ADSB_LOG("[adsb] %s refused (HTTP %d, sent %s)%s; using the other feed for %us\n",
                          ADSB_REAPI_HOST, _lastHttpCode, _keyDesc,
                          _lastHttpCode == 429 ? "" :
                          _apiKey.length() ? " -- check the adsb.lol API key on the config page"
                                           : " -- no API key set, and this network is not a known feeder",
                          (unsigned)(ADSB_REAPI_BACKOFF_MS / 1000));
        }

        if (ok && !list.empty()) {
            s_emptyStreak = 0;
            _lastSource = srcs[i].host;
            out.swap(list);
            return true;
        }
        if (ok && !anyOk) {
            anyOk = true;
            emptyHost = srcs[i].host;
            emptyList.swap(list);
        }
    }

    if (anyOk) {
        ++s_emptyStreak;
        _lastSource = emptyHost;
        out.swap(emptyList);
        return true;
    }
    return false;
}

bool AdsbClient::fetchFrom(const char* host, std::vector<Aircraft>& out, FeedKind kind) {
    const bool local = (kind == FeedKind::Local);
    _lastHttpCode = 0;
    char url[192];
    if (local) {
        // tar1090/readsb serve the whole picture the receiver hears -- there is no radius
        // query, so the nearest-N cull during parse does that job instead. Plain HTTP: it
        // is your own LAN, and TLS to a Pi would buy a handshake every poll for nothing.
        snprintf(url, sizeof(url), "http://%s%s", host, ADSB_LOCAL_PATH);
    } else {
        double nm = ceil((double)_rangeKm * 0.539957);              // km -> nautical miles (rounded up)
        if (nm < 1.0) nm = 1.0;
        if (kind == FeedKind::ReApi)
            snprintf(url, sizeof(url), "https://%s/?circle=%.5f,%.5f,%.0f", host, _lat, _lon, nm);
        else
            snprintf(url, sizeof(url), "https://%s/v2/point/%.4f/%.4f/%.0f", host, _lat, _lon, nm);
    }

    bool hostChanged = (_lastHost == nullptr || strcmp(_lastHost, host) != 0);
    if (hostChanged) {
        _http.end();
        if (!local) _client.setInsecure();
        _http.setReuse(true);
        _http.setConnectTimeout(local ? 2500 : 6000);   // a box on the LAN answers fast or not at all
        _http.setTimeout(local ? 4000 : 8000);
        _lastHost = host;
    }

    if (!_http.begin(local ? (WiFiClient &)_plain : (WiFiClient &)_client, url)) {
        ADSB_LOG("[adsb] begin failed (%s)\n", host);
        if (local) _plain.stop(); else _client.stop();
        return false;
    }
    _http.setUserAgent(ADSB_USER_AGENT);
    // begin() clears custom headers, so this is added fresh on every request.
    if (kind == FeedKind::ReApi && _apiKey.length()) _http.addHeader("x-api-key", _apiKey);
    _http.addHeader("Accept", "application/json");

    const int code = _http.GET();
    _lastHttpCode = code;
    if (code != 200) {
        char tls[128] = "";
        const int tlsCode = _client.lastError(tls, sizeof(tls));
        ADSB_LOG("[adsb] HTTP %d (%s) tls=%d '%s' heap=%u largest=%u psram=%u\n",
                      code, host, tlsCode, tls,
                      (unsigned)ESP.getFreeHeap(),
                      (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
                      (unsigned)ESP.getFreePsram());
        if (kind == FeedKind::ReApi) ADSB_LOG("[adsb]   that re-api request was sent with %s\n", _keyDesc);
        _http.end();
        _client.stop();
        return false;
    }

    // Only keep the fields we use -> much smaller parsed document.
    JsonDocument filter(&s_jsonPsram);
    const char* keys[] = { "ac", "aircraft" };
    const char* flds[] = { "hex", "flight", "t", "lat", "lon", "alt_baro", "alt_geom",
                           "track", "true_heading", "gs", "baro_rate",
                           "squawk", "seen_pos", "dbFlags" };
    for (const char* k : keys)
        for (const char* f : flds)
            filter[k][0][f] = true;

    JsonDocument doc(&s_jsonPsram);
    const int expectedBytes = _http.getSize();
    NetworkClient& responseStream = _http.getStream();
    ReliableJsonStream jsonStream(responseStream);
    DeserializationError err = deserializeJson(doc, jsonStream,
                                               DeserializationOption::Filter(filter));
    if (err) {
        ADSB_LOG("[adsb] JSON parse failed (%s): %s; expected=%d read=%u available=%d connected=%d\n",
                      host, err.c_str(), expectedBytes, (unsigned)jsonStream.bytesRead(),
                      responseStream.available(), responseStream.connected());
        _http.end();
        _client.stop();
        return false;
    }
    _http.end();
    _client.stop();

    JsonArrayConst arr = doc["ac"].as<JsonArrayConst>();
    if (arr.isNull()) arr = doc["aircraft"].as<JsonArrayConst>();
    if (arr.isNull()) return false;

    // Keep the ADSB_MAX_AIRCRAFT *nearest* aircraft
    std::vector<Aircraft> tmp;
    std::vector<float>     dist;             // parallel array: km from home for each kept aircraft
    tmp.reserve(ADSB_MAX_AIRCRAFT);
    dist.reserve(ADSB_MAX_AIRCRAFT);
    const uint32_t now = millis();
    for (JsonObjectConst a : arr) {
        if (a["lat"].isNull() || a["lon"].isNull()) continue;   // need a position
        const double lat = a["lat"].as<double>();
        const double lon = a["lon"].as<double>();

        // Robust altitude parsing: preference alt_baro, fallback to alt_geom
        bool onGround = false;
        float altFt = 0.0f;
        if (a["alt_baro"].is<const char*>()) {
            const char* s = a["alt_baro"].as<const char*>();
            if (s && (strcasecmp(s, "ground") == 0)) onGround = true;
        }
        if (!onGround) {
            if (!a["alt_baro"].isNull())     altFt = a["alt_baro"].as<float>();
            else if (!a["alt_geom"].isNull()) altFt = a["alt_geom"].as<float>();
        }

        if (_hideGround && onGround) continue;
        // optional filters (applied before the cap, so slots only go to matching aircraft)
        if (_minAltFt > 0.0f && (onGround || altFt < _minAltFt)) continue;
        if (_milOnly && (((a["dbFlags"] | 0u) & 0x1) == 0)) continue;

        const float d = (float)geo::haversineKm(_lat, _lon, lat, lon);

        // nearest-N gate: if the buffer is full and this one isn't closer than the farthest kept,
        // drop it now — before any string allocation.
        int farIdx = -1;
        if ((int)tmp.size() >= ADSB_MAX_AIRCRAFT) {
            farIdx = 0;
            for (int i = 1; i < (int)dist.size(); ++i) if (dist[i] > dist[farIdx]) farIdx = i;
            if (d >= dist[farIdx]) continue;
        }

        Aircraft ac;
        ac.hex = (const char*)(a["hex"] | "");
        if (ac.hex.length() == 0) continue;
        ac.flight = String((const char*)(a["flight"] | "")); ac.flight.trim();
        ac.type   = (const char*)(a["t"] | "");
        {
            const char *cat = (const char*)(a["category"] | "");
            ac.emitter[0] = cat[0] ? (char)toupper((unsigned char)cat[0]) : 0;
            ac.emitter[1] = (cat[0] && cat[1]) ? cat[1] : 0;
            ac.emitter[2] = 0;
        }
        ac.lat = lat; ac.lon = lon;
        ac.onGround = onGround;
        ac.altBaro  = altFt;
        ac.track    = !a["track"].isNull() ? a["track"].as<float>() : (!a["true_heading"].isNull() ? a["true_heading"].as<float>() : NAN);
        ac.gs       = !a["gs"].isNull() ? a["gs"].as<float>() : NAN;
        ac.baroRate = !a["baro_rate"].isNull() ? a["baro_rate"].as<float>() : NAN;
        if (a["squawk"].is<const char*>()) ac.squawk = atoi(a["squawk"].as<const char*>());
        else if (!a["squawk"].isNull())    ac.squawk = a["squawk"].as<int>();
        else                               ac.squawk = -1;
        ac.seenPos  = a["seen_pos"] | 0;
        ac.military = ((a["dbFlags"] | 0u) & 0x1) != 0;
        ac.lastUpdateMs = now;

        if (farIdx >= 0) { tmp[farIdx] = std::move(ac); dist[farIdx] = d; }   // replace the farthest kept
        else             { tmp.push_back(std::move(ac)); dist.push_back(d); }
    }

    out.swap(tmp);
    _lastOkMs = now;
    return true;
}
