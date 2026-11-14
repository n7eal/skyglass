# Data source — ADS-B feed

## Primary: airplanes.live (free, no key)
Independent ADS-B/MLAT aggregator. **Educational / non-commercial use only** — fits this project. Be a good citizen: poll every 1–2 s max, send a descriptive `User-Agent`.

### Endpoint (position + radius)
```
GET https://api.airplanes.live/v2/point/{lat}/{lon}/{radius_nm}
```
- `lat`, `lon`: decimal degrees (our home coords).
- `radius_nm`: nautical miles (max 250). Convert from our range: `nm = km * 0.539957`.

### Response (readsb format)
JSON object; the aircraft list is under key **`ac`** (older/raw readsb files use `aircraft` — handle both). Each entry includes (keys omitted when unavailable):

| Field          | Meaning                                  | Use |
|----------------|------------------------------------------|-----|
| `hex`          | 24-bit ICAO id (may start with `~`)      | stable key / de-dupe |
| `flight`       | callsign (8 chars)                       | label |
| `lat`,`lon`    | position, decimal degrees                | project to screen |
| `alt_baro`     | barometric altitude (ft) or `"ground"`   | altitude color |
| `track`        | ground track, ° from true N             | glyph rotation |
| `true_heading` | heading, ° (fallback when no `track`)   | glyph rotation |
| `gs`           | ground speed (kt)                        | detail card |
| `baro_rate`    | vertical rate (fpm, ±)                   | V/S arrow |
| `squawk`       | transponder code                         | emergency detect (7500/7600/7700) |
| `seen_pos`     | seconds since last position fix          | stale/expiry + trail |
| `t` / `type`   | aircraft type (e.g. B738) when present   | detail card |
| `dbFlags`      | bitfield (military, etc.)                | "interesting" alerts |

### Fallback: adsb.fi
Same aircraft format (`ac` list). `GET https://opendata.adsb.fi/api/v3/lat/{lat}/lon/{lon}/dist/{radius_nm}` (max 250 NM, no key, 1 request/s; personal non-commercial use, credit adsb.fi). Used automatically when airplanes.live errors, times out or answers 429; the config page can swap which of the two is tried first. The reply is chunked under HTTP/1.1, so the client requests HTTP/1.0.

### Not used: OpenSky
Now requires OAuth2 client-credentials and has tighter anonymous limits — awkward for an always-on embedded device. Keep as a documented alternative only.

## On-device math (implemented in src/geo.h)
For each aircraft, given home `(lat0, lon0)` and range `R_km` (outer ring):
1. **Distance** via haversine (km).
2. **Bearing** from home to aircraft (° from N, clockwise).
3. **Project** to screen: `r_px = (dist_km / R_km) * R_px_outer`; with north-up,
   `x = cx + r_px * sin(bearing)`, `y = cy - r_px * cos(bearing)`.
4. Drop aircraft beyond `R_km` (or clamp to the rim with a "beyond range" marker).
