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
#include <WiFiUdp.h>
#include <esp_random.h>

// Minimal DNS A-record lookup, straight to the network's DNS server. lwIP's own resolver
// returns a single address (and caches it), so it cannot tell us about the other servers
// behind a name. Returns the number of IPv4 addresses written to `out`, or -1.
static int dns_query_a(const char *name, const IPAddress &server, uint32_t *out, int max) {
    uint8_t q[300];
    size_t n = 0;
    const uint16_t id = (uint16_t)esp_random();
    q[n++] = id >> 8; q[n++] = id & 0xFF;
    q[n++] = 0x01; q[n++] = 0x00;                  // standard query, recursion desired
    q[n++] = 0; q[n++] = 1;                        // one question
    memset(q + n, 0, 6); n += 6;                   // no answer/authority/additional
    for (const char *p = name; *p;) {
        const char *dot = strchr(p, '.');
        const size_t len = dot ? (size_t)(dot - p) : strlen(p);
        if (len == 0 || len > 63 || n + len + 6 > sizeof(q)) return -1;
        q[n++] = (uint8_t)len;
        memcpy(q + n, p, len); n += len;
        p += len;
        if (*p == '.') ++p;
    }
    q[n++] = 0;
    q[n++] = 0; q[n++] = 1;                        // QTYPE  A
    q[n++] = 0; q[n++] = 1;                        // QCLASS IN

    WiFiUDP udp;
    if (!udp.begin(0)) return -1;
    if (!udp.beginPacket(server, 53)) { udp.stop(); return -1; }
    udp.write(q, n);
    if (!udp.endPacket()) { udp.stop(); return -1; }
    uint8_t r[512];
    int len = 0;
    const uint32_t t0 = millis();
    while (millis() - t0 < 1500) {
        if (udp.parsePacket() > 0) { len = udp.read(r, sizeof(r)); break; }
        delay(10);
    }
    udp.stop();
    if (len < 12 || r[0] != q[0] || r[1] != q[1] || (r[3] & 0x0F) != 0) return -1;

    const int qd = (r[4] << 8) | r[5];
    const int an = (r[6] << 8) | r[7];
    int i = 12;
    auto skipName = [&](int &at) -> bool {
        while (at < len) {
            const uint8_t b = r[at];
            if (b == 0) { ++at; return true; }
            if ((b & 0xC0) == 0xC0) { at += 2; return true; }   // compression pointer
            at += b + 1;
        }
        return false;
    };
    for (int k = 0; k < qd; ++k) { if (!skipName(i)) return -1; i += 4; }
    int cnt = 0;
    for (int k = 0; k < an && i < len; ++k) {
        if (!skipName(i) || i + 10 > len) break;
        const uint16_t type = (r[i] << 8) | r[i + 1];
        const uint16_t rdl  = (r[i + 8] << 8) | r[i + 9];
        i += 10;
        if (i + rdl > len) break;
        if (type == 1 && rdl == 4 && cnt < max)
            out[cnt++] = (uint32_t)IPAddress(r[i], r[i + 1], r[i + 2], r[i + 3]);
        i += rdl;
    }
    return cnt;
}

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

// Every ordering of the three feeds. Index = setPublicOrder() value; 0 and 1 match the
// old two-feed settings so a saved choice keeps its meaning.
static const uint8_t kOrders[][AdsbClient::SRC_COUNT] = {
    { AdsbClient::SRC_REAPI,     AdsbClient::SRC_AIRPLANES, AdsbClient::SRC_ADSBFI },
    { AdsbClient::SRC_AIRPLANES, AdsbClient::SRC_REAPI,     AdsbClient::SRC_ADSBFI },
    { AdsbClient::SRC_ADSBFI,    AdsbClient::SRC_REAPI,     AdsbClient::SRC_AIRPLANES },
    { AdsbClient::SRC_REAPI,     AdsbClient::SRC_ADSBFI,    AdsbClient::SRC_AIRPLANES },
    { AdsbClient::SRC_AIRPLANES, AdsbClient::SRC_ADSBFI,    AdsbClient::SRC_REAPI },
    { AdsbClient::SRC_ADSBFI,    AdsbClient::SRC_AIRPLANES, AdsbClient::SRC_REAPI },
};
static const char *const kOrderLabels[] = {
    "adsb.lol, airplanes.live, adsb.fi",
    "airplanes.live, adsb.lol, adsb.fi",
    "adsb.fi, adsb.lol, airplanes.live",
    "adsb.lol, adsb.fi, airplanes.live",
    "airplanes.live, adsb.fi, adsb.lol",
    "adsb.fi, airplanes.live, adsb.lol",
};
static const char *const kSrcHost[AdsbClient::SRC_COUNT] = { ADSB_REAPI_HOST, ADSB_FALLBACK_HOST, ADSB_FI_HOST };

int AdsbClient::orderCount() { return (int)(sizeof(kOrders) / sizeof(kOrders[0])); }
const char *AdsbClient::orderLabel(int o) { return (o >= 0 && o < orderCount()) ? kOrderLabels[o] : "?"; }
bool AdsbClient::reapiFirst() const { return kOrders[_publicOrder][0] == SRC_REAPI; }

// Retry-After is either delay-seconds or an HTTP-date. Only the seconds form is parsed;
// a date (or anything else) returns 0 and the caller uses ADSB_429_DEFAULT_S.
static uint32_t parse_retry_after(const String &v) {
    if (v.length() == 0 || v.length() > 9) return 0;
    for (size_t i = 0; i < v.length(); ++i) if (!isdigit((unsigned char)v[i])) return 0;
    return (uint32_t)v.toInt();
}

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
    if (_reapiBackoff && nowMs - _reapiBackoffStart >= _reapiBackoffDur)
        _reapiBackoff = false;
    const bool reapiDue = !_reapiBackoff &&
                          (!_reapiArmed || nowMs - _reapiLastMs >= ADSB_REAPI_MIN_INTERVAL_MS);

    // Release any 429 holds whose Retry-After has run out.
    for (int s = 0; s < SRC_COUNT; ++s)
        if (_hold[s] && nowMs - _holdStart[s] >= _holdDur[s]) {
            _hold[s] = false;
            ADSB_LOG("[adsb] %s: Retry-After hold over, using it again\n", kSrcHost[s]);
        }

    struct Src { const char* host; FeedKind kind; int id; };
    Src srcs[SRC_COUNT];
    int n = 0;
    for (int k = 0; k < SRC_COUNT; ++k) {
        const int s = kOrders[_publicOrder][k];
        if (_hold[s]) continue;                              // told to wait (429)
        if (s == SRC_REAPI && !reapiDue) continue;           // rate gate / refusal backoff
        const FeedKind kind = (s == SRC_REAPI) ? FeedKind::ReApi
                            : (s == SRC_ADSBFI) ? FeedKind::AdsbFi : FeedKind::Point;
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

        if (srcs[i].kind == FeedKind::ReApi) {     // the attempt itself counts toward the limit
            _reapiArmed = true;
            _reapiLastMs = nowMs;
        }
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
            if (_lastRetryAfterS)
                ADSB_LOG("[adsb] %s rate-limited (HTTP 429, Retry-After %u s); holding off %u s, using the other feeds\n",
                         srcs[i].host, (unsigned)_lastRetryAfterS, (unsigned)sec);
            else
                ADSB_LOG("[adsb] %s rate-limited (HTTP 429, no Retry-After); holding off %u s, using the other feeds\n",
                         srcs[i].host, (unsigned)sec);
        } else if (srcs[i].kind == FeedKind::ReApi && !ok &&
            (_lastHttpCode == 403 || _lastHttpCode == 401)) {
            _reapiBackoff = true;
            _reapiBackoffStart = nowMs;
            _reapiBackoffDur = ADSB_REAPI_BACKOFF_MS;
            ADSB_LOG("[adsb] %s refused (HTTP %d, sent %s)%s; using the other feed for %us\n",
                          ADSB_REAPI_HOST, _lastHttpCode, _keyDesc,
                          _apiKey.length() ? " -- check the adsb.lol API key on the config page"
                                           : " -- no API key set, and this network is not a known feeder",
                          (unsigned)(ADSB_REAPI_BACKOFF_MS / 1000));
        } else if (srcs[i].kind == FeedKind::ReApi && !ok && !reapiHasGoodIp(nowMs)) {
            // Connection error, TLS failure or a reply slower than ADSB_FETCH_DEADLINE_MS.
            // These come in runs, and each one costs a timeout before the other feed is
            // tried -- so give re-api a short rest rather than paying that every poll.
            _reapiBackoff = true;
            _reapiBackoffStart = nowMs;
            _reapiBackoffDur = ADSB_REAPI_SLOW_BACKOFF_MS;
            ADSB_LOG("[adsb] %s (%s) %s after %u ms; using the other feed for %us\n",
                     ADSB_REAPI_HOST, _reapiIp, _lastTooSlow ? "too slow" : "failed",
                     (unsigned)_lastFetchMs, (unsigned)(ADSB_REAPI_SLOW_BACKOFF_MS / 1000));
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

void AdsbClient::refreshReapiIps(uint32_t now) {
    if (_reapiNIps && now - _reapiIpsAt < ADSB_REAPI_IP_REFRESH_MS) return;
    _reapiIpsAt = now;
    uint32_t ips[kReapiMaxIps];
    int n = -1;
    const IPAddress dns = WiFi.dnsIP(0);
    if ((uint32_t)dns != 0) n = dns_query_a(ADSB_REAPI_HOST, dns, ips, kReapiMaxIps);
    if (n <= 0) {                                   // fall back to the system resolver
        IPAddress ip;
        if (WiFi.hostByName(ADSB_REAPI_HOST, ip) == 1) { ips[0] = (uint32_t)ip; n = 1; }
    }
    if (n <= 0) return;                             // keep whatever list we had

    // Carry "bad" marks over for servers still in the new list.
    bool bad[kReapiMaxIps] = {};
    uint32_t since[kReapiMaxIps] = {};
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < _reapiNIps; ++j)
            if (ips[i] == _reapiIps[j]) { bad[i] = _reapiBad[j]; since[i] = _reapiBadSince[j]; }
    const bool changed = (n != _reapiNIps) || memcmp(ips, _reapiIps, n * sizeof(uint32_t)) != 0;
    memcpy(_reapiIps, ips, n * sizeof(uint32_t));
    memcpy(_reapiBad, bad, sizeof(bad));
    memcpy(_reapiBadSince, since, sizeof(since));
    _reapiNIps = n;
    if (_reapiCur >= n) _reapiCur = 0;
    if (changed) {
        char list[8 * 17] = "";
        for (int i = 0; i < n; ++i) {
            strncat(list, " ", sizeof(list) - strlen(list) - 1);
            strncat(list, IPAddress(_reapiIps[i]).toString().c_str(), sizeof(list) - strlen(list) - 1);
        }
        ADSB_LOG("[adsb] %s servers:%s\n", ADSB_REAPI_HOST, list);
    }
}

// Prefer the server that last worked; skip any marked bad until their time is up.
int AdsbClient::pickReapiIp(uint32_t now) {
    if (!_reapiNIps) return -1;
    for (int k = 0; k < _reapiNIps; ++k) {
        const int i = (_reapiCur + k) % _reapiNIps;
        if (_reapiBad[i] && now - _reapiBadSince[i] >= ADSB_REAPI_IP_BAD_MS) _reapiBad[i] = false;
        if (!_reapiBad[i]) { _reapiCur = i; return i; }
    }
    // Every server is marked bad: give the next one another chance rather than none.
    _reapiCur = (_reapiCur + 1) % _reapiNIps;
    _reapiBad[_reapiCur] = false;
    return _reapiCur;
}

bool AdsbClient::reapiHasGoodIp(uint32_t now) const {
    for (int i = 0; i < _reapiNIps; ++i)
        if (!_reapiBad[i] || now - _reapiBadSince[i] >= ADSB_REAPI_IP_BAD_MS) return true;
    return false;
}

void AdsbClient::markReapiIp(bool ok) {
    const int i = _reapiCurIdx;
    _reapiCurIdx = -1;
    if (i < 0 || i >= _reapiNIps) return;
    if (ok) { _reapiBad[i] = false; return; }
    _reapiBad[i] = true;
    _reapiBadSince[i] = millis();
    _reapiCur = (i + 1) % _reapiNIps;              // next request goes elsewhere
    ADSB_LOG("[adsb] re-api server %s not answering; skipping it for %u min\n",
             _reapiIp, (unsigned)(ADSB_REAPI_IP_BAD_MS / 60000));
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
        if (kind == FeedKind::ReApi)
            snprintf(url, sizeof(url), "https://%s/?circle=%.5f,%.5f,%.0f", host, _lat, _lon, nm);
        else if (kind == FeedKind::AdsbFi)
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
    _reapiCurIdx = -1;
    if (kind == FeedKind::ReApi) {
        // Connect to a specific re-api server ourselves (TLS still uses the host name for
        // SNI), so a dead one can be skipped. HTTPClient sees the open connection and
        // reuses it (setReuse(true) above) instead of resolving the name again.
        const uint32_t now = millis();
        refreshReapiIps(now);
        const int idx = pickReapiIp(now);
        if (idx >= 0) {
            const IPAddress ip(_reapiIps[idx]);
            snprintf(_reapiIp, sizeof(_reapiIp), "%s", ip.toString().c_str());
            _reapiCurIdx = idx;
            _client.stop();
            _client.setConnectionTimeout(6000);
            if (!_client.connect(ip, 443, host, nullptr, nullptr, nullptr)) {
                _lastHttpCode = -1;
                ADSB_LOG("[adsb] could not connect to re-api server %s after %u ms\n",
                         _reapiIp, (unsigned)(millis() - t0));
                markReapiIp(false);
                _client.stop();
                return false;
            }
        } else {
            snprintf(_reapiIp, sizeof(_reapiIp), "DNS-fail");   // fall back to connect-by-name
        }
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
        if (kind == FeedKind::ReApi) {
            ADSB_LOG("[adsb]   that re-api request went to %s, sent with %s\n", _reapiIp, _keyDesc);
            if (code < 0) markReapiIp(false);     // transport failure: try another server
            else          markReapiIp(true);      // the server answered (an HTTP error is not its fault)
        }
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
        if (kind == FeedKind::ReApi) markReapiIp(false);
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
    if (kind == FeedKind::ReApi) markReapiIp(true);

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
