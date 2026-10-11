#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <mutex>

namespace CloudAirmass {

constexpr double Pi = 3.14159265358979323846;
constexpr double SiderealPerSolar = 1.00273790935;

inline double Altitude(double latitudeDeg, double declinationDeg, double hourAngleHours) {
    const double lat = latitudeDeg * Pi / 180., dec = declinationDeg * Pi / 180., ha = hourAngleHours * 15. * Pi / 180.;
    const double s = std::sin(lat) * std::sin(dec) + std::cos(lat) * std::cos(dec) * std::cos(ha);
    return std::asin(std::max(-1., std::min(1., s))) * 180. / Pi;
}

inline double FromAltitude(double altitudeDeg) {
    if (!std::isfinite(altitudeDeg) || altitudeDeg < 5. || altitudeDeg > 90.)
        return std::numeric_limits<double>::quiet_NaN();
    return 1. / (std::sin(altitudeDeg * Pi / 180.) + .50572 * std::pow(altitudeDeg + 6.07995, -1.6364));
}

inline bool Normalizes(double magPerAirmass, double airmass) {
    return magPerAirmass > 0. && magPerAirmass <= 1. && std::isfinite(airmass) && airmass >= 1.;
}

inline double ZenithFactor(double magPerAirmass, double airmass) {
    if (!Normalizes(magPerAirmass, airmass))
        return 1.;
    return std::pow(10., .4 * magPerAirmass * (airmass - 1.));
}

} // namespace CloudAirmass

class CloudPointingCache {
    mutable std::mutex mx;
    bool haveEquatorial = false, haveLatitude = false;
    int64_t equatorialMs = 0, lastReadingMs = 0;
    double ra = 0., dec = 0., lst = 0., latitude = 0.;
    int altitudes = 0;
    int64_t altitudeMs[2] = {};
    double altitude[2] = {};
    double heldAirmass = 0.;
    int64_t heldAirmassMs = 0;
    double altitudeLocked(int64_t nowMs, bool* equatorial = nullptr) const {
        if (equatorial) *equatorial = false;
        if (haveEquatorial && haveLatitude && std::llabs(nowMs - equatorialMs) <= EquatorialValidForMs) {
            if (equatorial) *equatorial = true;
            const double sidereal = lst + (double) (nowMs - equatorialMs) / 3600000. * CloudAirmass::SiderealPerSolar;
            return CloudAirmass::Altitude(latitude, dec, sidereal - ra);
        }
        if (altitudes == 0 || std::llabs(nowMs - altitudeMs[altitudes - 1]) > ValidForMs)
            return std::numeric_limits<double>::quiet_NaN();
        if (altitudes == 1) return altitude[0];
        const double span = (double) (altitudeMs[1] - altitudeMs[0]);
        const double at = (double) std::min<int64_t>(nowMs, altitudeMs[1] + QueryAfterMs) - (double) altitudeMs[0];
        return altitude[0] + (altitude[1] - altitude[0]) * at / span;
    }
public:
    static constexpr int64_t QueryAfterMs = 60000;
    static constexpr int64_t ValidForMs = 300000;
    static constexpr int64_t HoldMs = 600000;
    static constexpr int64_t EquatorialValidForMs = ValidForMs + HoldMs;
    void RecordEquatorial(int64_t tMs, double raHours, double decDeg, double lstHours) {
        if (!std::isfinite(raHours) || !std::isfinite(decDeg) || !std::isfinite(lstHours) || std::fabs(decDeg) > 90.) return;
        std::lock_guard<std::mutex> lock(mx);
        haveEquatorial = true; equatorialMs = tMs; ra = raHours; dec = decDeg; lst = lstHours;
        lastReadingMs = std::max(lastReadingMs, tMs);
    }
    void RecordAltitude(int64_t tMs, double altDeg) {
        if (!std::isfinite(altDeg) || altDeg < -90. || altDeg > 90.) return;
        std::lock_guard<std::mutex> lock(mx);
        // Keep the two newest readings in time order; an out-of-order reading starts over.
        if (altitudes > 0 && tMs <= altitudeMs[altitudes - 1]) altitudes = 0;
        else if (altitudes == 2) { altitudeMs[0] = altitudeMs[1]; altitude[0] = altitude[1]; altitudes = 1; }
        altitudeMs[altitudes] = tMs; altitude[altitudes] = altDeg; ++altitudes;
        lastReadingMs = std::max(lastReadingMs, tMs);
    }
    void RecordLatitude(double latDeg) {
        if (!std::isfinite(latDeg) || std::fabs(latDeg) > 90.) return;
        std::lock_guard<std::mutex> lock(mx);
        haveLatitude = true; latitude = latDeg;
    }
    bool HaveLatitude() const { std::lock_guard<std::mutex> lock(mx); return haveLatitude; }
    bool QueryDue(int64_t nowMs) const {
        std::lock_guard<std::mutex> lock(mx);
        return !lastReadingMs || nowMs - lastReadingMs > QueryAfterMs || (haveEquatorial && !haveLatitude);
    }
    void Clear() {
        std::lock_guard<std::mutex> lock(mx);
        haveEquatorial = haveLatitude = false; altitudes = 0; lastReadingMs = 0; heldAirmassMs = 0;
    }

    void ClearPointing() {
        std::lock_guard<std::mutex> lock(mx);
        haveEquatorial = false; altitudes = 0; lastReadingMs = 0; heldAirmassMs = 0;
    }
    double AltitudeAt(int64_t nowMs) const {
        std::lock_guard<std::mutex> lock(mx);
        return altitudeLocked(nowMs);
    }
    double AirmassAt(int64_t nowMs) {
        std::lock_guard<std::mutex> lock(mx);
        bool equatorial = false;
        const double airmass = CloudAirmass::FromAltitude(altitudeLocked(nowMs, &equatorial));
        if (std::isfinite(airmass)) {
            heldAirmass = airmass; heldAirmassMs = equatorial ? 0 : nowMs;
            return airmass;
        }
        if (heldAirmassMs && nowMs >= heldAirmassMs && nowMs - heldAirmassMs <= HoldMs)
            return heldAirmass;
        return std::numeric_limits<double>::quiet_NaN();
    }
};
