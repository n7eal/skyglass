#pragma once
// Fetches nearby aircraft from adsb.lol's re-api (fallback: airplanes.live) and parses
// the readsb JSON into a vector<Aircraft>. See docs/DATA_SOURCE.md.
#include <vector>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include "aircraft.h"

// Feed log lines. On the P4 the Serial log never reaches the USB port, so mirror them
// through the ROM printf (the path the RESTART banners already use) -- otherwise a
// failing feed on that board is invisible. The board flag is a -D build flag.
#if defined(BOARD_WAVESHARE_P4_LCD_4C)
#  include <esp_rom_sys.h>
#  define ADSB_LOG(...) do { Serial.printf(__VA_ARGS__); esp_rom_printf(__VA_ARGS__); } while (0)
#else
#  define ADSB_LOG(...) Serial.printf(__VA_ARGS__)
#endif

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

    // Order of the internet feeds (config page "Internet source order").
    //   0 = re-api.adsb.lol, then airplanes.live   (default)
    //   1 = airplanes.live, then re-api.adsb.lol
    // Each feed keeps its own limits whatever its position: re-api is still gated to
    // ADSB_REAPI_MIN_INTERVAL_MS. Add new orders here as more sources arrive.
    void setPublicOrder(int o) { _publicOrder = o; }
    int  publicOrder() const { return _publicOrder; }
    bool lastPollWasLocal() const { return _lastWasLocal; }
    // Host that supplied the last successful poll (for the serial log).
    const char* lastSource() const { return _lastSource; }

    // Fetch + parse. Returns true on success and fills `out` (replaces contents).
    // On failure, leaves `out` untouched and returns false (caller keeps last good).
    bool poll(std::vector<Aircraft>& out);

    uint32_t lastOkMs() const { return _lastOkMs; }

private:
    // How a host is queried.
    //   Point -- HTTPS /v2/point/{lat}/{lon}/{nm}       (airplanes.live)
    //   ReApi -- HTTPS /?circle={lat},{lon},{nm}         (re-api.adsb.lol)
    //   Local -- plain HTTP aircraft.json on your own LAN receiver
    enum class FeedKind { Point, ReApi, Local };

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
    int    _srcMode = 0;
    bool   _lastWasLocal = false;
    const char* _lastSource = "";
    int    _lastHttpCode = 0;

    // re-api rate gate. Plain unsigned millis() deltas, so they survive the 49-day wrap.
    bool     _reapiArmed = false;        // false until the first attempt
    uint32_t _reapiLastMs = 0;           // millis() of the last attempt (success or not)
    bool     _reapiBackoff = false;      // refused with 429/403: wait ADSB_REAPI_BACKOFF_MS
    uint32_t _reapiBackoffStart = 0;
};
