#pragma once
// Fetches nearby aircraft from adsb.lol's re-api (fallback: airplanes.live) and parses
// the readsb JSON into a vector<Aircraft>. See docs/DATA_SOURCE.md.
#include <vector>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include "aircraft.h"

// Feed log lines: Serial, (P4) the USB console, and the /log web page. See sg_log.h.
#include "sg_log.h"
#define ADSB_LOG(...) sg_log(__VA_ARGS__)

class AdsbClient {
public:
    // What the feed is actually being asked for. Exposed because the displayed centre and
    // the queried centre are two different things, and when they disagree the scope looks
    // empty for no visible reason -- /diag reports both so the disagreement is checkable.
    double queryLat() const { return _lat; }
    double queryLon() const { return _lon; }
    ~AdsbClient() { _http.end(); }
    void begin(double homeLat, double homeLon, float rangeKm);
    void setHome(double lat, double lon) { _lat = lat; _lon = lon; }
    void setRange(float km) { _rangeKm = km; }
    void setHideGround(bool h) { _hideGround = h; }   // skip on-ground aircraft during parse
    void setMinAltFt(float ft) { _minAltFt = ft; }    // skip aircraft below this altitude (0 = off)
    void setMilitaryOnly(bool m) { _milOnly = m; }    // keep only military-flagged aircraft

    // A dump1090/readsb/tar1090 receiver on the local network. When set it is polled
    // first and the public APIs become the fallback -- your own antenna, no internet.
    // Empty string disables it.
    void setLocalHost(const char* h) { _localHost = h ? h : ""; }
    // 0 = auto (local first, public feeds if it does not answer)
    // 1 = local only  -- shows exactly what your antenna hears, never masked by the APIs
    // 2 = public only -- ignore the receiver even when it is up
    void setSourceMode(int m) { _srcMode = m; }
    int  sourceMode() const { return _srcMode; }
    const char* localHost() const { return _localHost.c_str(); }

    // adsb.lol API key, sent as x-api-key on re-api requests. Empty = no header (re-api
    // then only answers when the request comes from a network that feeds adsb.lol).
    void setApiKey(const char* k);
    // "key ...abcd" (last four characters) or "no key" -- for logs, never the key itself.
    const char* apiKeyDesc() const { return _keyDesc; }
    bool hasApiKey() const { return _apiKey.length() > 0; }

    // Internet feeds, and the order they are tried in (config page "Internet source order").
    // Orders 0 and 1 keep their old meaning (adsb.lol / airplanes.live first) with adsb.fi
    // appended. Each feed keeps its own limits whatever its position: re-api is still
    // gated to ADSB_REAPI_MIN_INTERVAL_MS, and any feed that answers 429 is held off.
    enum Source { SRC_REAPI = 0, SRC_AIRPLANES = 1, SRC_ADSBFI = 2, SRC_COUNT = 3 };
    static int         orderCount();
    static const char *orderLabel(int order);
    void setPublicOrder(int o) { _publicOrder = (o >= 0 && o < orderCount()) ? o : 0; }
    int  publicOrder() const { return _publicOrder; }
    bool reapiFirst() const;                   // decides the poll cadence
    bool lastPollWasLocal() const { return _lastWasLocal; }
    // Host that supplied the last successful poll (for the serial log).
    const char* lastSource() const { return _lastSource; }

    // Fetch + parse. Returns true on success and fills `out` (replaces contents).
    // On failure, leaves `out` untouched and returns false (caller keeps last good).
    bool poll(std::vector<Aircraft>& out);

    uint32_t lastOkMs() const { return _lastOkMs; }
    // How long the most recent single-host request took, in ms (for the log).
    uint32_t lastFetchMs() const { return _lastFetchMs; }
    // Server address the last re-api request resolved to ("?" if unknown), for the log.
    const char* lastReapiIp() const { return _reapiIp; }

private:
    // How a host is queried.
    //   Point -- HTTPS /v2/point/{lat}/{lon}/{nm}       (airplanes.live)
    //   ReApi -- HTTPS /?circle={lat},{lon},{nm}         (re-api.adsb.lol)
    //   Local -- plain HTTP aircraft.json on your own LAN receiver
    //   AdsbFi -- HTTPS /api/v3/lat/{lat}/lon/{lon}/dist/{nm} (opendata.adsb.fi)
    enum class FeedKind { Point, ReApi, Local, AdsbFi };

    // One host, one attempt. Sets _lastHttpCode.
    bool fetchFrom(const char* host, std::vector<Aircraft>& out, FeedKind kind);

    double _lat = 0, _lon = 0;
    float  _rangeKm = 15.0f;
    bool   _hideGround = false;
    float  _minAltFt = 0.0f;
    bool   _milOnly = false;
    uint32_t _lastOkMs = 0;

    WiFiClientSecure _client;
    WiFiClient       _plain;          // local receiver: no TLS, it is on your own LAN
    HTTPClient _http;
    const char* _lastHost = nullptr;
    String _localHost;
    String _apiKey;
    char   _keyDesc[16] = "no key";
    int    _publicOrder = 0;
    uint32_t _lastRetryAfterS = 0;       // Retry-After of the last 429 (0 = none sent)
    // Per-feed hold after HTTP 429, honouring Retry-After.
    bool     _hold[SRC_COUNT] = {};
    uint32_t _holdStart[SRC_COUNT] = {};
    uint32_t _holdDur[SRC_COUNT] = {};
    int    _srcMode = 0;
    bool   _lastWasLocal = false;
    const char* _lastSource = "";
    int    _lastHttpCode = 0;

    // re-api rate gate. Plain unsigned millis() deltas, so they survive the 49-day wrap.
    bool     _reapiArmed = false;        // false until the first attempt
    uint32_t _reapiLastMs = 0;           // millis() of the last attempt (success or not)
    bool     _reapiBackoff = false;      // refused with 429/403: wait ADSB_REAPI_BACKOFF_MS
    uint32_t _reapiBackoffStart = 0;
    uint32_t _reapiBackoffDur = 0;       // how long this backoff lasts
    uint32_t _lastFetchMs = 0;
    bool     _lastTooSlow = false;       // the last request hit ADSB_FETCH_DEADLINE_MS
    char     _reapiIp[16] = "?";

    // re-api server pool: every IPv4 address behind ADSB_REAPI_HOST.
    static const int kReapiMaxIps = 8;
    uint32_t _reapiIps[kReapiMaxIps] = {};
    bool     _reapiBad[kReapiMaxIps] = {};
    uint32_t _reapiBadSince[kReapiMaxIps] = {};
    int      _reapiNIps = 0;
    int      _reapiCur = 0;              // server currently preferred
    int      _reapiCurIdx = -1;          // server used by the request in flight (-1 = by name)
    uint32_t _reapiIpsAt = 0;
    void refreshReapiIps(uint32_t now);
    int  pickReapiIp(uint32_t now);
    void markReapiIp(bool ok);
    bool reapiHasGoodIp(uint32_t now) const;
};
