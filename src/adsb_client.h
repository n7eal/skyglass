#pragma once
// Fetches nearby aircraft from airplanes.live and adsb.fi (or a local receiver) and parses
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

    // Internet feeds, and the order they are tried in (config page "Internet source order").
    //   0 = airplanes.live, then adsb.fi   (default)
    //   1 = adsb.fi, then airplanes.live
    // A feed that answers 429 is held off for its Retry-After and the other one is used.
    enum Source { SRC_AIRPLANES = 0, SRC_ADSBFI = 1, SRC_COUNT = 2 };
    static int         orderCount();
    static const char *orderLabel(int order);
    void setPublicOrder(int o) { _publicOrder = (o >= 0 && o < orderCount()) ? o : 0; }
    int  publicOrder() const { return _publicOrder; }
    bool lastPollWasLocal() const { return _lastWasLocal; }
    // Host that supplied the last successful poll (for the serial log).
    const char* lastSource() const { return _lastSource; }

    // Fetch + parse. Returns true on success and fills `out` (replaces contents).
    // On failure, leaves `out` untouched and returns false (caller keeps last good).
    bool poll(std::vector<Aircraft>& out);

    uint32_t lastOkMs() const { return _lastOkMs; }
    // How long the most recent single-host request took, in ms (for the log).
    uint32_t lastFetchMs() const { return _lastFetchMs; }

private:
    // How a host is queried.
    //   Point -- HTTPS /v2/point/{lat}/{lon}/{nm}       (airplanes.live)
    //   Local -- plain HTTP aircraft.json on your own LAN receiver
    //   AdsbFi -- HTTPS /api/v3/lat/{lat}/lon/{lon}/dist/{nm} (opendata.adsb.fi)
    enum class FeedKind { Point, Local, AdsbFi };

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

    uint32_t _lastFetchMs = 0;
    bool     _lastTooSlow = false;       // the last request hit ADSB_FETCH_DEADLINE_MS
};
