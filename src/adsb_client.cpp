// Fetch nearby aircraft from airplanes.live and adsb.fi (order set on the config page),
// or from a receiver on the local network, and parse the
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
//
// It also enforces an overall deadline: once it passes, reads report "no data", the
// parse fails with IncompleteInput, and the poll moves on to the next feed instead of
// waiting out a server that is sending its reply a trickle at a time.
class ReliableJsonStream : public Stream {
public:
    ReliableJsonStream(Stream& source, uint32_t deadlineMs) : _source(source), _deadline(deadlineMs) {}
    int available() override { return expiredNow() ? 0 : _source.available(); }
    int read() override {
        if (expiredNow()) return -1;
        const int value = _source.read();
        if (value >= 0) ++_bytesRead;
        return value;
    }
    int peek() override { return expiredNow() ? -1 : _source.peek(); }
    bool expired() const { return _expired; }
    void flush() override { _source.flush(); }
    size_t write(uint8_t) override { return 0; }
    size_t bytesRead() const { return _bytesRead; }

private:
    bool expiredNow() {
        if (!_expired && (int32_t)(millis() - _deadline) >= 0) _expired = true;
        return _expired;
    }
    Stream& _source;
    uint32_t _deadline;
    bool _expired = false;
    size_t _bytesRead = 0;
};

// Each ordering of the internet feeds. Index = setPublicOrder() value.
static const uint8_t kOrders[][AdsbClient::SRC_COUNT] = {
    { AdsbClient::SRC_AIRPLANES, AdsbClient::SRC_ADSBFI },
    { AdsbClient::SRC_ADSBFI,    AdsbClient::SRC_AIRPLANES },
};
static const char *const kOrderLabels[] = {
    "airplanes.live, adsb.fi",
    "adsb.fi, airplanes.live",
};
static const char *const kSrcHost[AdsbClient::SRC_COUNT] = { ADSB_AIRPLANES_HOST, ADSB_FI_HOST };

int AdsbClient::orderCount() { return (int)(sizeof(kOrders) / sizeof(kOrders[0])); }
const char *AdsbClient::orderLabel(int o) { return (o >= 0 && o < orderCount()) ? kOrderLabels[o] : "?"; }

// Retry-After is either delay-seconds or an HTTP-date. Only the seconds form is parsed;
// a date (or anything else) returns 0 and the caller uses ADSB_429_DEFAULT_S.
static const char *fmt_dur(uint32_t s, char *buf, size_t cap) {
    if (s >= 3600)    snprintf(buf, cap, "%uh%02um", (unsigned)(s / 3600), (unsigned)((s % 3600) / 60));
    else if (s >= 60) snprintf(buf, cap, "%um%02us", (unsigned)(s / 60), (unsigned)(s % 60));
    else              snprintf(buf, cap, "%us", (unsigned)s);
    return buf;
}

static uint32_t parse_retry_after(const String &v) {
    if (v.length() == 0 || v.length() > 9) return 0;
    for (size_t i = 0; i < v.length(); ++i) if (!isdigit((unsigned char)v[i])) return 0;
    return (uint32_t)v.toInt();
}

void AdsbClient::begin(double homeLat, double homeLon, float rangeKm) {
    _lat = homeLat; _lon = homeLon; _rangeKm = rangeKm;
}

bool AdsbClient::poll(std::vector<Aircraft>& out) {
    if (WiFi.status() != WL_CONNECTED) return false;
    // Counts consecutive polls where the primary answered normally but with no aircraft.
    // Only ever touched from adsb_task, so a plain static is fine.
    static uint32_t s_emptyStreak = 0;

    // Your own receiver wins whenever it is configured and answering: no internet round
    // trip, sub-second data, and it shows whatever your antenna hears.
    if (_localHost.length() && _srcMode != 2 && _srcMode != 3) {
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

    // Public sources, in the order chosen on the config page.
    const uint32_t nowMs = millis();

    // Release any 429 holds whose Retry-After has run out.
    for (int s = 0; s < SRC_COUNT; ++s)
        if (_hold[s] && nowMs - _holdStart[s] >= _holdDur[s]) {
            _hold[s] = false;
            ADSB_LOG("[adsb] %s: hold over, using it again\n", kSrcHost[s]);
        }

    struct Src { const char* host; FeedKind kind; int id; };
    Src srcs[SRC_COUNT];
    int n = 0;
    for (int k = 0; k < SRC_COUNT; ++k) {
        const int s = kOrders[_publicOrder][k];
        if (_hold[s]) continue;                              // told to wait (429)
        const FeedKind kind = (s == SRC_ADSBFI) ? FeedKind::AdsbFi : FeedKind::Point;
        srcs[n++] = { kSrcHost[s], kind, s };
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

        std::vector<Aircraft> list;
        const bool ok = fetchFrom(srcs[i].host, list, srcs[i].kind);

        if (!ok && _lastHttpCode == 429) {
            // Rate limited: wait exactly as long as the feed asks, then use it again.
            uint32_t sec = _lastRetryAfterS ? _lastRetryAfterS : ADSB_429_DEFAULT_S;
            if (sec < 1) sec = 1;
            if (sec > ADSB_429_MAX_S) sec = ADSB_429_MAX_S;
            _hold[srcs[i].id] = true;
            _holdStart[srcs[i].id] = nowMs;
            _holdDur[srcs[i].id] = sec * 1000UL;
            char d[16];
            if (_lastRetryAfterS)
                ADSB_LOG("[adsb] %s rate-limited (HTTP 429, Retry-After %u s); holding off %s, using the other feed\n",
                         srcs[i].host, (unsigned)_lastRetryAfterS, fmt_dur(sec, d, sizeof(d)));
            else
                ADSB_LOG("[adsb] %s rate-limited (HTTP 429, no Retry-After); holding off %s, using the other feed\n",
                         srcs[i].host, fmt_dur(sec, d, sizeof(d)));
        } else if (!ok && (_lastHttpCode == 403 || _lastHttpCode == 401)) {
            // Refused outright. Retrying every poll only deepens a block.
            _hold[srcs[i].id] = true;
            _holdStart[srcs[i].id] = nowMs;
            _holdDur[srcs[i].id] = ADSB_REFUSED_HOLD_S * 1000UL;
            char d[16];
            ADSB_LOG("[adsb] %s refused us (HTTP %d); holding off %s, using the other feed\n",
                     srcs[i].host, _lastHttpCode, fmt_dur(ADSB_REFUSED_HOLD_S, d, sizeof(d)));
        }

        if (ok && !list.empty()) {
            s_emptyStreak = 0;
            _lastSource = srcs[i].host;
            out.swap(list);
            return true;
        }
        if (ok && list.empty() && (s_emptyStreak % ADSB_EMPTY_RECHECK_POLLS) == 0)
            ADSB_LOG("[adsb] %s answered with 0 aircraft\n", srcs[i].host);
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

    // Internet-first auto: no internet feed answered (each one errored, or is on hold),
    // so fall back to the receiver on the local network. An internet feed that answered
    // with an empty sky is a real answer and does not come here.
    if (_srcMode == 3 && _localHost.length()) {
        std::vector<Aircraft> localList;
        if (fetchFrom(_localHost.c_str(), localList, FeedKind::Local)) {
            if (!_lastWasLocal)
                ADSB_LOG("[adsb] no internet feed answered; using the local receiver %s\n",
                         _localHost.c_str());
            _lastWasLocal = true;
            _lastSource = _localHost.c_str();
            out.swap(localList);
            return true;
        }
    }
    return false;
}

bool AdsbClient::fetchFrom(const char* host, std::vector<Aircraft>& out, FeedKind kind) {
    const bool local = (kind == FeedKind::Local);
    _lastHttpCode = 0;
    _lastTooSlow = false;
    const uint32_t t0 = millis();
    // Record the duration on every return path.
    struct FetchTimer {
        uint32_t &dst; uint32_t t0;
        ~FetchTimer() { dst = millis() - t0; }
    } fetchTimer{ _lastFetchMs, t0 };
    char url[192];
    if (local) {
        // tar1090/readsb serve the whole picture the receiver hears -- there is no radius
        // query, so the nearest-N cull during parse does that job instead. Plain HTTP: it
        // is your own LAN, and TLS to a Pi would buy a handshake every poll for nothing.
        snprintf(url, sizeof(url), "http://%s%s", host, ADSB_LOCAL_PATH);
    } else {
        double nm = ceil((double)_rangeKm * 0.539957);              // km -> nautical miles (rounded up)
        if (nm < 1.0) nm = 1.0;
        if (kind == FeedKind::AdsbFi)
            snprintf(url, sizeof(url), "https://%s/api/v3/lat/%.4f/lon/%.4f/dist/%.0f",
                     host, _lat, _lon, nm > 250.0 ? 250.0 : nm);
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

    if (!local) _client.setHandshakeTimeout(ADSB_TLS_HANDSHAKE_S);   // default is 120 s
    if (!_http.begin(local ? (WiFiClient &)_plain : (WiFiClient &)_client, url)) {
        ADSB_LOG("[adsb] begin failed (%s)\n", host);
        if (local) _plain.stop(); else _client.stop();
        return false;
    }
    _http.setUserAgent(ADSB_USER_AGENT);
    _http.addHeader("Accept", "application/json");
    // adsb.fi sends HTTP/1.1 replies chunked, and we parse straight from the raw stream,
    // which still carries the chunk-size lines: ArduinoJson read the leading "1f4a" as the
    // number 1, returned Ok, and the reply had no aircraft list. Asking for HTTP/1.0 makes
    // the server send a plain body. (useHTTP10 also flips setReuse, so set it every time.)
    _http.useHTTP10(kind == FeedKind::AdsbFi);
    static const char *kWantHeaders[] = { "Retry-After" };
    _http.collectHeaders(kWantHeaders, 1);

    const int code = _http.GET();
    _lastHttpCode = code;
    _lastRetryAfterS = (code == 429) ? parse_retry_after(_http.header("Retry-After")) : 0;
    if (code != 200) {
        char tls[128] = "";
        const int tlsCode = _client.lastError(tls, sizeof(tls));
        ADSB_LOG("[adsb] HTTP %d (%s) after %u ms, tls=%d '%s' heap=%u largest=%u psram=%u\n",
                      code, host, (unsigned)(millis() - t0), tlsCode, tls,
                      (unsigned)ESP.getFreeHeap(),
                      (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
                      (unsigned)ESP.getFreePsram());
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
    ReliableJsonStream jsonStream(responseStream, t0 + ADSB_FETCH_DEADLINE_MS);
    DeserializationError err = deserializeJson(doc, jsonStream,
                                               DeserializationOption::Filter(filter));
    if (err && jsonStream.expired()) {
        _lastTooSlow = true;
        ADSB_LOG("[adsb] %s too slow: gave up after %u ms with %u of %d bytes\n",
                 host, (unsigned)(millis() - t0), (unsigned)jsonStream.bytesRead(), expectedBytes);
        _http.end();
        _client.stop();
        return false;
    }
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
    if (arr.isNull()) {
        ADSB_LOG("[adsb] %s answered %u bytes but no 'ac'/'aircraft' list (in %u ms)\n",
                 host, (unsigned)jsonStream.bytesRead(), (unsigned)(millis() - t0));
        return false;
    }

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
