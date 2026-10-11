/*
 *  star_shape.h
 *  PHD Guiding
 *
 *  Star shape from brightness-weighted second moments, and the elongated-candidate rule used by
 *  star auto-selection (Ticket 006). Header-only and free of wxWidgets so it can be unit tested.
 *
 *  This source code is distributed under the following "BSD" license
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *    Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *    Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *    Neither the name of Craig Stark, Stark Labs nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef STAR_SHAPE_H_INCLUDED
#define STAR_SHAPE_H_INCLUDED

#include <algorithm>
#include <cmath>
#include <vector>

// Brightness-weighted second moments of a star's pixels about its centroid.
struct StarMoments
{
    double w = 0.0;
    double sxx = 0.0;
    double syy = 0.0;
    double sxy = 0.0;

    void Add(double dx, double dy, double weight)
    {
        w += weight;
        sxx += weight * dx * dx;
        syy += weight * dy * dy;
        sxy += weight * dx * dy;
    }
};

struct StarShape
{
    // sqrt(lambda_min / lambda_max) of the moment matrix: 1 for a round star, smaller when elongated.
    // 0 when unavailable.
    double axisRatio = 0.0;
    // Direction of the major axis in degrees, -90..90, measured from the image +x axis towards +y
    // (image rows, so clockwise on screen).
    double angleDeg = 0.0;
};

inline StarShape ShapeFromMoments(const StarMoments& m)
{
    StarShape shape;
    if (!(m.w > 0.0))
        return shape;
    double const xx = m.sxx / m.w;
    double const yy = m.syy / m.w;
    double const xy = m.sxy / m.w;
    double const half = 0.5 * (xx + yy);
    double const disc = std::sqrt(std::max(0.0, 0.25 * (xx - yy) * (xx - yy) + xy * xy));
    double const lmax = half + disc;
    double const lmin = std::max(0.0, half - disc);
    if (!(lmax > 0.0) || !std::isfinite(lmax))
        return shape;
    shape.axisRatio = std::sqrt(lmin / lmax);
    shape.angleDeg = 0.5 * std::atan2(2.0 * xy, xx - yy) * 180.0 / 3.14159265358979323846;
    return shape;
}

// Elongated-candidate rule for star auto-selection: a candidate is rejected when its axis ratio is below
// MinRoundnessPct of the median of the field's usable candidates. Guiding itself never uses it.
namespace StarRoundness
{
const int DefaultMinPct = 75;
const int MinPct = 50;
const int MaxPct = 95;
// Without enough measured stars the field median is not trusted; typical round stars measure about 0.8.
const double FallbackMedian = 0.80;
const unsigned int MinStarsForMedian = 5;

inline int ClampPct(int pct)
{
    return std::min(MaxPct, std::max(MinPct, pct));
}

struct Candidate
{
    double axisRatio;
    bool found;
    bool saturated;
    double snr;
};

// Median axis ratio of the candidates that were found, are not saturated and reach minSNR; the fallback
// with fewer than MinStarsForMedian of them.
inline double FieldMedian(const std::vector<Candidate>& candidates, double minSNR, unsigned int *used = nullptr)
{
    std::vector<double> q;
    for (const Candidate& c : candidates)
        if (c.found && !c.saturated && c.snr >= minSNR && c.axisRatio > 0.0)
            q.push_back(c.axisRatio);
    if (used)
        *used = (unsigned int) q.size();
    if (q.size() < MinStarsForMedian)
        return FallbackMedian;
    std::sort(q.begin(), q.end());
    size_t const n = q.size();
    return n % 2 ? q[n / 2] : 0.5 * (q[n / 2 - 1] + q[n / 2]);
}

// A field's median roundness and the number of usable stars it came from. A median of 0 means the field was not
// measured (a subframe, which AutoFind cannot search); with fewer than MinStarsForMedian stars it is the fallback.
struct Field
{
    double median = 0.0;
    unsigned int stars = 0;

    bool Measured() const { return median > 0.0; }
    bool Fallback() const { return stars < MinStarsForMedian; }
};

inline double Threshold(double fieldMedian, int minRoundnessPct)
{
    return ClampPct(minRoundnessPct) / 100.0 * fieldMedian;
}

// An unavailable axis ratio (0) is never rejected.
inline bool IsElongated(double axisRatio, double threshold)
{
    return axisRatio > 0.0 && axisRatio < threshold;
}
} // namespace StarRoundness

#endif /* STAR_SHAPE_H_INCLUDED */
