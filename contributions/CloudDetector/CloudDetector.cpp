/* CloudDetector.cpp
 *
 * Scene-quality / cloud detection -- see CloudDetector.h and cloud-detection-design.md.
 * Kept dependency-light (standard C++ plus the pure-preprocessor CloudDetectorConfig.h) so it
 * builds and unit-tests standalone outside PHD2.
 *
 * Copyright (c) 2026 Leo Shatz
 * All rights reserved.
 */

#include "CloudDetector.h"
#include "CloudDetectorConfig.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <exception>
#include <limits>

namespace {

// Local aliases for the tuning block (shadow-mode starting points -- refine from logged
// real-session/replay data before using the telemetry for automated decisions).
constexpr int   kShortK           = CONFIG_CLOUD_SHORT_K;
constexpr int   kBaselineDecimMs  = CONFIG_CLOUD_BASELINE_DECIM_MS;
constexpr int   kMinBaseEntries   = CONFIG_CLOUD_MIN_BASE_ENTRIES;
constexpr int   kWarmupMs         = CONFIG_CLOUD_WARMUP_MS;
constexpr int   kLossRunTrip      = CONFIG_CLOUD_LOSS_RUN_TRIP;
constexpr int   kSuspectQuietMs   = CONFIG_CLOUD_SUSPECT_QUIET_MS;
constexpr int   kRecoverHoldMs    = CONFIG_CLOUD_RECOVER_HOLD_MS;
constexpr int   kMediumWindowBase = CONFIG_CLOUD_MEDIUM_WINDOW_MS;
constexpr float kRecoverMargin    = CONFIG_CLOUD_RECOVER_MARGIN;
constexpr float kRecoverCap       = CONFIG_CLOUD_RECOVER_CAP;
constexpr float kRecoverSnrMargin = CONFIG_CLOUD_RECOVER_SNR_MARGIN;
constexpr float kAnchorDownPctMin = CONFIG_CLOUD_ANCHOR_DOWN_PCT_MIN;
constexpr float kSlowRatio        = CONFIG_CLOUD_SLOW_RATIO;
constexpr int   kSlowDwellMs      = CONFIG_CLOUD_SLOW_DWELL_MS;
constexpr float kScoreBandK       = CONFIG_CLOUD_SCORE_BAND_K;
constexpr float kScoreBandMin     = CONFIG_CLOUD_SCORE_BAND_MIN;
constexpr int   kVariabilityK     = CONFIG_CLOUD_VARIABILITY_K;
constexpr float kVariabilitySigmaK = CONFIG_CLOUD_VARIABILITY_SIGMA_K;
constexpr float kVariabilityRecover = CONFIG_CLOUD_VARIABILITY_RECOVER_FACTOR;
constexpr float kMassVarFracMin   = CONFIG_CLOUD_MASS_VARIABILITY_FRAC_MIN;
constexpr float kSnrVarDbMin      = CONFIG_CLOUD_SNR_VARIABILITY_DB_MIN;
constexpr int64_t kStuckMs        = CONFIG_CLOUD_STUCK_MS;
constexpr float kStaticMadFrac    = CONFIG_CLOUD_STATIC_MAD_FRAC;
constexpr int   kPeriodicLogMs    = CONFIG_CLOUD_PERIODIC_LOG_MS;
constexpr float kEps              = 1e-6f;

const char* StateName(SceneState s)
{
    switch (s) {
    case SceneState::Warmup:   return "WARMUP";
    case SceneState::Clear:    return "CLEAR";
    case SceneState::Suspect:  return "SUSPECT";
    case SceneState::Obscured: return "OBSCURED";
    }
    return "?";
}

} // namespace

float CloudHighTailContrast(const unsigned short *imageData, int frameWidth, int frameHeight,
                            int x, int y, int width, int height) noexcept
{
    if (!imageData || frameWidth <= 0 || frameHeight <= 0 || width <= 0 || height <= 0)
        return -1.f;

    // Reject dimensions whose row addressing cannot be represented by size_t. This is mostly a
    // guard against corrupt metadata; real guide frames are many orders of magnitude smaller.
    if ((size_t) frameWidth > std::numeric_limits<size_t>::max() / (size_t) frameHeight)
        return -1.f;

    const int left = std::max(0, x);
    const int top = std::max(0, y);
    // Widen before adding: a malformed ROI near INT_MAX must not invoke signed overflow.
    const int64_t right64 = (int64_t) x + (int64_t) width;
    const int64_t bottom64 = (int64_t) y + (int64_t) height;
    const int right = (int) std::max<int64_t>(0, std::min<int64_t>(frameWidth, right64));
    const int bottom = (int) std::max<int64_t>(0, std::min<int64_t>(frameHeight, bottom64));
    if (left >= right || top >= bottom)
        return -1.f;

    constexpr size_t kMaxSamples = 131072;
    const uint64_t sampleWidth = (uint64_t) (right - left);
    const uint64_t sampleHeight = (uint64_t) (bottom - top);
    const uint64_t area = sampleWidth * sampleHeight;
    int stride = std::max(1, (int) std::sqrt((double) area / (double) kMaxSamples));
    while (((sampleWidth + (uint64_t) stride - 1) / (uint64_t) stride) *
               ((sampleHeight + (uint64_t) stride - 1) / (uint64_t) stride) >
           (uint64_t) kMaxSamples)
        ++stride;

    // Camera/output BPP is not a universal numeric scale. Derive histogram quantization from
    // the sampled values and convert the result back to their original ADU scale.
    unsigned short maxSample = 0;
    size_t sampleCount = 0;
    for (int row = top; row < bottom; row += stride)
    {
        const unsigned short *pixels = imageData + (size_t) row * frameWidth;
        for (int col = left; col < right; col += stride)
        {
            maxSample = std::max(maxSample, pixels[col]);
            ++sampleCount;
        }
    }
    if (!sampleCount)
        return -1.f;

    constexpr int kBinCount = 4096;
    int shift = 0;
    while (((unsigned int) maxSample >> shift) >= kBinCount)
        ++shift;

    std::array<unsigned int, kBinCount> hist = {};
    for (int row = top; row < bottom; row += stride)
    {
        const unsigned short *pixels = imageData + (size_t) row * frameWidth;
        for (int col = left; col < right; col += stride)
            ++hist[pixels[col] >> shift];
    }

    const size_t medianCount = (sampleCount + 1) / 2;
    const size_t percentileCount = (sampleCount * 995 + 999) / 1000;
    size_t cumulative = 0;
    int medianBin = 0;
    bool haveMedian = false;
    for (int bin = 0; bin < kBinCount; ++bin)
    {
        cumulative += hist[bin];
        if (!haveMedian && cumulative >= medianCount)
        {
            medianBin = bin;
            haveMedian = true;
        }
        if (cumulative >= percentileCount)
            return (float) ((bin - medianBin) << shift);
    }
    return (float) ((kBinCount - 1 - medianBin) << shift);
}

// ---------------------------------------------------------------------------
// Ring
// ---------------------------------------------------------------------------

void CloudDetector::Ring::push(float v)
{
    buf[head] = v;
    head = (head + 1) % kCap;
    if (n < kCap)
        n++;
}

static float MedianOf(float* tmp, int n)
{
    std::sort(tmp, tmp + n);
    return tmp[n / 2];
}

bool CloudDetector::Ring::median(float& out) const
{
    if (n == 0)
        return false;
    float tmp[kCap];
    for (int i = 0; i < n; ++i) tmp[i] = buf[i];
    out = MedianOf(tmp, n);
    return true;
}

bool CloudDetector::Ring::medianMad(float& med, float& mad) const
{
    if (n == 0)
        return false;
    float tmp[kCap];
    for (int i = 0; i < n; ++i) tmp[i] = buf[i];
    med = MedianOf(tmp, n);
    for (int i = 0; i < n; ++i) tmp[i] = std::fabs(buf[i] - med);
    mad = MedianOf(tmp, n);
    return true;
}

bool CloudDetector::Ring::lastMedian(int k, float& out) const
{
    if (n < k)
        return false;
    float tmp[kCap];
    for (int i = 0; i < k; ++i)
        tmp[i] = buf[(head - 1 - i + kCap * 2) % kCap];
    out = MedianOf(tmp, k);
    return true;
}

bool CloudDetector::Ring::lastMedianMad(int k, float& med, float& mad) const
{
    if (n < k)
        return false;
    float tmp[kCap];
    for (int i = 0; i < k; ++i)
        tmp[i] = buf[(head - 1 - i + kCap * 2) % kCap];
    med = MedianOf(tmp, k);
    for (int i = 0; i < k; ++i)
        tmp[i] = std::fabs(buf[(head - 1 - i + kCap * 2) % kCap] - med);
    mad = MedianOf(tmp, k);
    return true;
}

// ---------------------------------------------------------------------------
// CloudDetector
// ---------------------------------------------------------------------------

CloudDetector::CloudDetector()
{
    applySensitivityLocked();   // ctor: no other thread can observe this yet
}

// Recompute every working threshold from m_sensitivityPct. Recovery thresholds are DERIVED from
// the trip thresholds rather than fixed, so `recover > trip` holds at every sensitivity -- with a
// fixed 0.85 recovery factor, 100% sensitivity tripped mass at 0.8625 and allowed recovery at
// 0.85, i.e. a "recovered" frame still satisfied the trip predicate and re-tripped immediately.
void CloudDetector::applySensitivityLocked()
{
    // f < 1 = less sensitive (thresholds farther below baseline, longer window); > 1 = more.
    const float f = 0.85f + 0.30f * (float)m_sensitivityPct / 100.f;   // 0.85 .. 1.15
    m_fastMassRatio   = std::min(0.92f, CONFIG_CLOUD_FAST_MASS_RATIO * f);
    m_fastBrightRatio = std::min(0.92f, CONFIG_CLOUD_FAST_BRIGHT_RATIO * f);
    m_medMassRatio    = std::min(0.92f, CONFIG_CLOUD_MED_MASS_RATIO * f);
    m_medBrightRatio  = std::min(0.92f, CONFIG_CLOUD_MED_BRIGHT_RATIO * f);
    m_medFeatureRatio = std::min(0.92f, CONFIG_CLOUD_MED_FEATURE_RATIO * f);
    m_medSnrDropDb    = CONFIG_CLOUD_MED_SNR_DROP_DB / f;
    m_slowRatio       = std::min(0.96f, kSlowRatio * f);
    m_mediumWindowMs  = std::max(12000, std::min(40000, (int)(kMediumWindowBase / f)));

    // recover = min(cap, trip + margin) for "ratio below trips" channels; for SNR (a DROP, so the
    // comparison is inverted) recover = trip - margin. Both directions end strictly stricter.
    m_recMassRatio    = std::min(kRecoverCap, m_medMassRatio + kRecoverMargin);
    m_recFeatureRatio = std::min(kRecoverCap, m_medFeatureRatio + kRecoverMargin);
    m_recSnrDropDb    = std::max(0.5f, m_medSnrDropDb - kRecoverSnrMargin);
}

void CloudDetector::SetSensitivityPct(int pct) noexcept
{
    try {
        pct = std::max(0, std::min(100, pct));
        std::lock_guard<std::mutex> lk(m_mx);
        if (pct == m_sensitivityPct)
            return;
        m_sensitivityPct = pct;
        applySensitivityLocked();
        // Thresholds that moved under a running state machine would compare new limits against dwell
        // timers and a reference accumulated under the old ones. Start over instead.
        resetLocked("sensitivity change");
    }
    catch (const std::exception& e) {
        containException("SetSensitivityPct", e.what());
    }
    catch (...) {
        containException("SetSensitivityPct", "non-standard exception");
    }
}

void CloudDetector::SetEnabled(bool on) noexcept
{
    try {
        std::lock_guard<std::mutex> lk(m_mx);
        if (m_enabled == on)
            return;
        m_enabled = on;
        // Reset on BOTH edges. Disabling must not leave a latched Obscured verdict readable by
        // IsClear() -- the state machine stops running, so nothing would ever clear it again.
        resetLocked(on ? "enabled" : "disabled");
    }
    catch (const std::exception& e) {
        containException("SetEnabled", e.what());
    }
    catch (...) {
        containException("SetEnabled", "non-standard exception");
    }
}

void CloudDetector::SetLogger(std::function<void(const std::string&)> logger) noexcept
{
    try {
        std::lock_guard<std::mutex> lk(m_mx);
        m_log = std::move(logger);
    }
    catch (const std::exception& e) {
        containException("SetLogger", e.what());
    }
    catch (...) {
        containException("SetLogger", "non-standard exception");
    }
}

void CloudDetector::Reset(const char* reason) noexcept
{
    try {
        std::lock_guard<std::mutex> lk(m_mx);
        resetLocked(reason);
    }
    catch (const std::exception& e) {
        containException("Reset", e.what());
    }
    catch (...) {
        containException("Reset", "non-standard exception");
    }
}

void CloudDetector::ReportFault(const char* operation, const char* detail) noexcept
{
    containException(operation, detail);
}

bool CloudDetector::IsClear() const noexcept
{
    try {
        std::lock_guard<std::mutex> lk(m_mx);
        return m_state != SceneState::Obscured;
    }
    catch (...) {
        // Cloud detection is advisory inside PHD2. If its mutex itself cannot be read, do not
        // manufacture a stale not-clear verdict that could remain latched forever.
        return true;
    }
}

SceneState CloudDetector::GetState() const noexcept
{
    try {
        std::lock_guard<std::mutex> lk(m_mx);
        return m_state;
    }
    catch (...) {
        return SceneState::Warmup;
    }
}

SceneTelemetry CloudDetector::GetTelemetry() const noexcept
{
    try {
        std::lock_guard<std::mutex> lk(m_mx);
        return m_tele;
    }
    catch (...) {
        SceneTelemetry unavailable;
        unavailable.healthy = false;
        return unavailable;
    }
}

// Follow the view UP at once; drift DOWN no faster than kAnchorDownPctMin per minute. The
// asymmetry is the whole point: an unconstrained reference adapts to a fade and normalizes it
// away, while a permanently lower but stable scene can still become nominal after roughly an
// observing-scale interval rather than requiring a restart.
void CloudDetector::Anchor::update(float v, int64_t t)
{
    if (!valid()) {
        seed(v, t);
        return;
    }
    if (v >= value) {
        value = v;
    }
    else {
        // Cap the elapsed term so the first update after a long input gap cannot carry
        // authorization for one huge drop -- exactly the poisoning this reference prevents.
        const int64_t dtMs = std::max<int64_t>(0, std::min<int64_t>(4 * kBaselineDecimMs, t - tMs));
        // Use magnitude rather than `value` itself so signed quality scores (negative FWHM) slew
        // in the correct direction too. The candidate magnitude avoids an artificially tiny rate
        // when the old standard is close to zero.
        const float scale = std::max({std::fabs(value), std::fabs(v), kEps});
        const float maxDrop = scale * (kAnchorDownPctMin / 100.f) * ((float)dtMs / 60000.f);
        value = std::max(v, value - maxDrop);
    }
    tMs = t;
}

CloudDetector::Identity CloudDetector::IdentityOf(const SceneSample& s)
{
    Identity id;
    id.exposureMs = (s.exposureMs > 0) ? s.exposureMs : -1;
    id.gain   = s.gain;
    id.bitDepth = s.bitDepth;
    id.mode   = s.mode;
    id.frameW = s.frameW; id.frameH = s.frameH;
    id.roiX = s.roiX; id.roiY = s.roiY; id.roiW = s.roiW; id.roiH = s.roiH;
    id.sourceGen = s.sourceGen;
    id.valid = true;
    return id;
}

const char* CloudDetector::IdentityDelta(const Identity& p, const Identity& c)
{
    if (!p.valid)
        return nullptr;
    // A field unknown on either side cannot evidence a change (an unreadable gain must not look
    // like a gain change every frame).
    auto changed = [](int a, int b) { return a >= 0 && b >= 0 && a != b; };
    if (changed(p.exposureMs, c.exposureMs)) return "exposure change";
    if (changed(p.gain, c.gain))             return "gain change";
    if (changed(p.bitDepth, c.bitDepth))     return "bit-depth change";
    if (changed(p.mode, c.mode))             return "detection mode change";
    if (changed(p.frameW, c.frameW) || changed(p.frameH, c.frameH))
        return "frame geometry change";
    if (changed(p.roiX, c.roiX) || changed(p.roiY, c.roiY) ||
        changed(p.roiW, c.roiW) || changed(p.roiH, c.roiH))
        return "ROI change";
    if (p.sourceGen != c.sourceGen)          return "source change";
    return nullptr;
}

void CloudDetector::clearStateLocked() noexcept
{
    m_baseMass.clear(); m_baseBright.clear(); m_baseSnr.clear();
    m_baseScore.clear(); m_baseFeatures.clear();
    m_shortMass.clear(); m_shortBright.clear(); m_shortSnr.clear();
    m_shortScore.clear(); m_shortFeatures.clear();
    m_refMass.clear(); m_refBright.clear(); m_refSnr.clear();
    m_refScore.clear(); m_refFeatures.clear();
    m_anchorBright.clear(); m_anchorMass.clear(); m_anchorSnr.clear();
    m_anchorScore.clear(); m_anchorFeatures.clear();
    m_seenMass = m_seenSnr = m_seenScore = m_seenFeatures = false;
    m_lastBaselinePushMs = 0;
    m_lastAnchorUpdateMs = 0;
    m_clearAccumMs = 0;
    m_prevSampleMs = 0;
    m_mediumTripSinceMs = 0;
    m_slowTripSinceMs = 0;
    m_recoverSinceMs = 0;
    m_suspectQuietSinceMs = 0;
    m_lossRun = 0;
    m_sawGoodDetection = false;
    m_lossLogged = false;
    m_lastPeriodicLogMs = 0;     // else the heartbeat stays silent for up to a minute after a reset
    m_state = SceneState::Warmup;
    m_stateSinceMs = 0;
    m_tele = SceneTelemetry{};
    m_tele.exceptionCount = m_exceptionCount;
    m_tele.loggerExceptionCount = m_loggerExceptionCount;
}

void CloudDetector::resetLocked(const char* reason)
{
    clearStateLocked();
    char buf[160];
    std::snprintf(buf, sizeof(buf), "cloud: reset (%s) -> warm-up", reason ? reason : "?");
    logLocked(buf);
}

void CloudDetector::noteExceptionLocked(bool loggerFailure) noexcept
{
    if (m_exceptionCount != std::numeric_limits<unsigned>::max())
        ++m_exceptionCount;
    if (loggerFailure && m_loggerExceptionCount != std::numeric_limits<unsigned>::max())
        ++m_loggerExceptionCount;
    m_tele.exceptionCount = m_exceptionCount;
    m_tele.loggerExceptionCount = m_loggerExceptionCount;
}

void CloudDetector::containException(const char* operation, const char* detail) noexcept
{
    try {
        std::lock_guard<std::mutex> lk(m_mx);
        noteExceptionLocked(false);
        clearStateLocked();
        m_tele.healthy = false;

        // Keep diagnostics best-effort. logLocked contains and disables a failing callback, so
        // reporting the original failure cannot recursively escape into guiding.
        char buf[320];
        std::snprintf(buf, sizeof(buf), "cloud: contained exception in %s (%s); detector reset to warm-up",
                      operation ? operation : "?", detail ? detail : "unknown");
        logLocked(buf);
        m_tele.healthy = false; // a logger failure must not hide the calculation failure
    }
    catch (...) {
        // A mutex/system failure leaves no safe state to mutate. Public entry points still honor
        // their no-throw contract; getters fall back to Warmup/unhealthy if they cannot lock.
    }
}

void CloudDetector::transitionLocked(SceneState next, const char* why, int64_t tMs)
{
    if (m_state == next)
        return;
    char buf[384];
    std::snprintf(buf, sizeof(buf),
        "cloud: %s -> %s (%s) mass=%.2f massVar=%.2f bright=%.2f "
        "snrDrop=%.1fdB snrVar=%.2f score%+.2f feat=%.2f "
        "slow[bright=%.2f mass=%.2f feat=%.2f] loss=%d",
        StateName(m_state), StateName(next), why ? why : "",
        m_tele.massRatio, m_tele.massScatterFactor, m_tele.brightRatio, m_tele.snrDropDb,
        m_tele.snrScatterFactor,
        m_tele.scoreDelta, m_tele.featureRatio,
        m_tele.slowBrightRatio, m_tele.slowMassRatio, m_tele.slowFeatureRatio,
        m_tele.lossRun);
    logLocked(buf);
    m_state = next;
    m_stateSinceMs = tMs;
    m_mediumTripSinceMs = 0;
    m_slowTripSinceMs = 0;
    m_recoverSinceMs = 0;
    m_suspectQuietSinceMs = 0;
    m_tele.staticObstruction = false;
}

void CloudDetector::Feed(const SceneSample& s) noexcept
{
    try {
        feedLocked(s);
    }
    catch (const std::exception& e) {
        containException("Feed", e.what());
    }
    catch (...) {
        containException("Feed", "non-standard exception");
    }
}

void CloudDetector::feedLocked(const SceneSample& s)
{
    std::lock_guard<std::mutex> lk(m_mx);

    // Acquisition discontinuities invalidate everything built from earlier frames, so track the
    // identity tuple unconditionally (even while disabled) and reset on any change.
    const Identity cur = IdentityOf(s);
    const char* discontinuity = IdentityDelta(m_ident, cur);
    m_ident = cur;
    if (!m_enabled)
        return;
    if (discontinuity)
        resetLocked(discontinuity);

    const int64_t t = s.tMs;
    if (t <= 0) {
        logLocked("cloud: ignored sample with invalid timestamp");
        return;
    }
    if (m_prevSampleMs > 0 && t < m_prevSampleMs)
        resetLocked("timestamp moved backwards");

    // A malformed upstream metric must be treated as unavailable, never inserted into a sorted
    // ring. NaN in particular does not provide the strict weak ordering std::sort requires and
    // can poison every subsequent ratio and JSON/HUD conversion.
    const bool validMass = s.detected && std::isfinite(s.mass) && s.mass > 0.f;
    const bool validFeatures = s.detected && s.features > 0;
    const bool validSnr = s.detected && std::isfinite(s.snr);
    const bool validScore = s.detected && std::isfinite(s.score);
    float brightCeil = -1.f;
    if (std::isfinite(s.brightCeil) && s.brightCeil >= 0.f) {
        const double normalized = s.brightExposureMs > 0
                                      ? (double) s.brightCeil * (1000.0 / (double) s.brightExposureMs)
                                      : (double) s.brightCeil;
        if (std::isfinite(normalized) && normalized <= std::numeric_limits<float>::max())
            brightCeil = (float) normalized;
    }

    // ---- loss bookkeeping (must precede the windows: it decides clear-eligibility) ----
    if (s.detected) {
        m_lossRun = 0;
        m_sawGoodDetection = true;
        m_lossLogged = false;
    }
    else if (m_sawGoodDetection) {
        if (m_lossRun < std::numeric_limits<int>::max())
            ++m_lossRun;
        // Target loss is the detection-hold state machine's business, not the scene monitor's.
        // Say so once per episode so a log reader can tell "we lost the target" from "the sky
        // went bad" -- previously both arrived as a cloud verdict.
        if (!m_lossLogged && m_lossRun >= kLossRunTrip) {
            m_lossLogged = true;
            char buf[200];
            std::snprintf(buf, sizeof(buf),
                "cloud: target lost (%d frames, detection hold owns this) scene=%s bright=%.2f",
                m_lossRun, StateName(m_state), m_tele.brightRatio);
            logLocked(buf);
        }
    }

    // Does this frame look like the clear, locked view the monitor is trying to characterize?
    const bool clearEligible = s.detected && s.stableLock && m_lossRun == 0 &&
                               (validMass || validFeatures || validSnr || validScore);

    // ---- short (fast) windows: the sky AS IT IS, unconditional ----
    // These drive the trips, so they must include the bad frames -- a blackout is undetected by
    // definition and still has to be measurable here. Zero is a REAL brightness reading (an
    // all-black ROI); only a negative means "no stats this frame". Rejecting zero discarded the
    // one measurement a total blackout produces, so with detection lost at the same moment --
    // which is what a blackout causes -- every detection channel was silenced and the brightness
    // window still held pre-blackout samples, leaving nothing able to trip.
    if (brightCeil >= 0.f)
        m_shortBright.push(brightCeil);
    if (validMass)     { m_shortMass.push(s.mass);                    m_seenMass = true; }
    if (validFeatures) { m_shortFeatures.push((float) s.features);   m_seenFeatures = true; }
    if (validSnr)      { m_shortSnr.push(s.snr);                      m_seenSnr = true; }
    if (validScore)    { m_shortScore.push(s.score);                  m_seenScore = true; }

    // ---- reference windows: what CLEAR looks like, clear-eligible frames only ----
    // Baseline entries and anchor updates snapshot from THESE, never from the trip windows above.
    // The two jobs disagree -- a trip window must include the bad frames, a reference window must
    // exclude them -- so they cannot share a ring: gating only the write moment still snapshotted
    // a median of mixed history whenever an eligible frame arrived among unstable ones.
    if (clearEligible) {
        if (brightCeil >= 0.f) m_refBright.push(brightCeil);
        if (validMass)     m_refMass.push(s.mass);
        if (validFeatures) m_refFeatures.push((float) s.features);
        if (validSnr)      m_refSnr.push(s.snr);
        if (validScore)    m_refScore.push(s.score);
    }

    // Frames without a current detection carry no fresh detection-derived telemetry; their short
    // windows simply hold whatever the last detected frames left there. Voting on that stale data
    // would let an old value speak for a frame that never produced one, so the detection channels
    // are silent whenever the target is missing. brightCeil is measured from the pixels regardless
    // of detection, so it keeps voting -- and it is the channel that matters during an occultation.
    const bool detChannelsLive = (m_lossRun == 0);

    // ---- baselines: accumulate only while presumed clear (Warmup/Clear); frozen otherwise ----
    const bool accumulating = (m_state == SceneState::Warmup || m_state == SceneState::Clear);
    if (accumulating) {
        // Warm-up may only accumulate across frames that actually look like a clear, locked view.
        // Counting arbitrary background (bright sky, no target) let warm-up "establish" a
        // baseline while the target was already lost, then publish Clear and trip on the very
        // next frame. Unusable frames PAUSE the clock rather than resetting it, so ordinary
        // dropouts merely delay arming.
        if (m_prevSampleMs > 0 && clearEligible)
            m_clearAccumMs += std::min<int64_t>(t - m_prevSampleMs, 3000);

        // Snapshots come from the REFERENCE rings (clear-eligible history only), so an eligible
        // frame arriving among unstable ones contributes its own reading, not a median of its
        // unstable neighbours. The decimation clock also only advances on eligible frames now:
        // with the write gated but the clock free-running, an ineligible frame at the decimation
        // boundary would silently skip that baseline entry.
        if (clearEligible &&
            (m_lastBaselinePushMs == 0 || t - m_lastBaselinePushMs >= kBaselineDecimMs)) {
            m_lastBaselinePushMs = t;
            float v;
            if (m_refBright.lastMedian(kShortK, v))
                m_baseBright.push(v);
            if (m_refMass.lastMedian(kShortK, v))
                m_baseMass.push(v);
            if (m_refSnr.lastMedian(kShortK, v))   m_baseSnr.push(v);
            if (m_refScore.lastMedian(kShortK, v)) m_baseScore.push(v);
            if (m_refFeatures.lastMedian(kShortK, v))
                m_baseFeatures.push(v);
        }
    }

    // Once armed, the protected standards have their own lifetime and update policy. They are
    // deliberately independent of the rolling baseline and state label: improvements become the
    // new standard without resetting through Warmup, while a persistently lower stable view slews
    // them down at only CONFIG_CLOUD_ANCHOR_DOWN_PCT_MIN. Continuing the slow slew in Suspect or
    // Obscured is what eventually permits a permanent benign change to become the new normal.
    if (m_state != SceneState::Warmup && clearEligible &&
        (m_lastAnchorUpdateMs == 0 || t - m_lastAnchorUpdateMs >= kBaselineDecimMs)) {
        m_lastAnchorUpdateMs = t;
        float v;
        if (m_refBright.lastMedian(kShortK, v))   m_anchorBright.update(v, t);
        if (m_refMass.lastMedian(kShortK, v))     m_anchorMass.update(v, t);
        if (m_refSnr.lastMedian(kShortK, v))      m_anchorSnr.update(v, t);
        if (m_refScore.lastMedian(kShortK, v))    m_anchorScore.update(v, t);
        if (m_refFeatures.lastMedian(kShortK, v)) m_anchorFeatures.update(v, t);
    }
    m_prevSampleMs = t;

    // ---- telemetry (recent scene vs the protected clear-sky standard) ----
    float shortBright = 0.f, shortMass = 0.f, shortFeatures = 0.f;
    m_tele.massRatio = detChannelsLive && m_shortMass.lastMedian(kShortK, shortMass)
                           ? m_anchorMass.ratio(shortMass) : -1.f;
    m_tele.brightRatio = m_shortBright.lastMedian(kShortK, shortBright)
                             ? m_anchorBright.ratio(shortBright) : -1.f;
    m_tele.featureRatio = detChannelsLive && m_shortFeatures.lastMedian(kShortK, shortFeatures)
                              ? m_anchorFeatures.ratio(shortFeatures) : -1.f;
    m_tele.lossRun      = m_lossRun;
    float shortSnr = 0.f;
    const bool haveSnr = detChannelsLive && m_anchorSnr.valid() &&
                         m_shortSnr.lastMedian(kShortK, shortSnr);
    m_tele.snrDropDb = haveSnr ? -m_anchorSnr.delta(shortSnr) : 0.f;
    float shortScore = 0.f, scoreNoiseMed = 0.f, baseScoreMad = 0.f;
    const bool haveScore = detChannelsLive &&
                            m_anchorScore.valid() &&
                            m_shortScore.lastMedian(kShortK, shortScore) &&
                            m_baseScore.medianMad(scoreNoiseMed, baseScoreMad);
    m_tele.scoreDelta = haveScore ? m_anchorScore.delta(shortScore) : 0.f;

    // Cloud often first appears as broad waves around an otherwise similar average star mass.
    // Measure recent window amplitude (MAD), not only frame-to-frame change: real cloud waves can
    // evolve gradually enough that every individual step looks harmless. The absolute floor keeps
    // very stable cameras from magnifying insignificant numerical ripple.
    float recentMassMed = 0.f, recentMassMad = 0.f, clearMassMed = 0.f, clearMassMad = 0.f;
    const bool haveMassScatter = detChannelsLive &&
                                 m_shortMass.lastMedianMad(kVariabilityK, recentMassMed, recentMassMad) &&
                                 m_baseMass.medianMad(clearMassMed, clearMassMad) &&
                                 std::fabs(clearMassMed) > kEps;
    const float massScatterBand = haveMassScatter
                                      ? std::max(kVariabilitySigmaK * clearMassMad,
                                                 kMassVarFracMin * std::fabs(clearMassMed))
                                      : 0.f;
    m_tele.massScatterFactor = haveMassScatter && massScatterBand > kEps
                                   ? recentMassMad / massScatterBand : -1.f;

    float recentSnrMed = 0.f, recentSnrMad = 0.f, clearSnrMed = 0.f, clearSnrMad = 0.f;
    const bool haveSnrScatter = detChannelsLive &&
                                m_shortSnr.lastMedianMad(kVariabilityK, recentSnrMed, recentSnrMad) &&
                                m_baseSnr.medianMad(clearSnrMed, clearSnrMad);
    const float snrScatterBand = haveSnrScatter
                                     ? std::max(kVariabilitySigmaK * clearSnrMad, kSnrVarDbMin)
                                     : 0.f;
    m_tele.snrScatterFactor = haveSnrScatter && snrScatterBand > kEps
                                  ? recentSnrMad / snrScatterBand : -1.f;

    // ---- slow-haze ratios against the clear anchors ----
    float shortBrightNow = 0.f, shortMassNow = 0.f, shortFeatNow = 0.f;
    const bool haveBrightNow = m_shortBright.lastMedian(kShortK, shortBrightNow);
    const bool haveMassNow   = detChannelsLive && m_shortMass.lastMedian(kShortK, shortMassNow);
    const bool haveFeatNow   = detChannelsLive && m_shortFeatures.lastMedian(kShortK, shortFeatNow);
    m_tele.slowBrightRatio  = haveBrightNow ? m_anchorBright.ratio(shortBrightNow) : -1.f;
    m_tele.slowMassRatio    = haveMassNow   ? m_anchorMass.ratio(shortMassNow)     : -1.f;
    m_tele.slowFeatureRatio = haveFeatNow   ? m_anchorFeatures.ratio(shortFeatNow) : -1.f;

    if (m_state == SceneState::Warmup) {
        // Arming gate. Time and a brightness ring alone are not evidence of a clear view: the
        // CURRENT frame must be a stable, detected lock, and every channel this mode actually
        // produces must have enough baseline to both vote and recover. Arming without them
        // published Clear over an already-lost target.
        const bool haveBaselines =
            m_baseBright.n >= kMinBaseEntries &&
            (!m_seenMass     || m_baseMass.n     >= kMinBaseEntries) &&
            (!m_seenSnr      || m_baseSnr.n      >= kMinBaseEntries) &&
            (!m_seenScore    || m_baseScore.n    >= kMinBaseEntries) &&
            (!m_seenFeatures || m_baseFeatures.n >= kMinBaseEntries);
        const bool stableNow = s.detected && s.stableLock && m_lossRun == 0;
        if (m_clearAccumMs >= kWarmupMs && haveBaselines && stableNow) {
            transitionLocked(SceneState::Clear, "baseline established", t);
            // Seed every protected standard from the reference we just certified as clear.
            float v;
            if (m_baseBright.median(v))   m_anchorBright.seed(v, t);
            if (m_seenMass && m_baseMass.median(v))         m_anchorMass.seed(v, t);
            if (m_seenSnr && m_baseSnr.median(v))           m_anchorSnr.seed(v, t);
            if (m_seenScore && m_baseScore.median(v))       m_anchorScore.seed(v, t);
            if (m_seenFeatures && m_baseFeatures.median(v)) m_anchorFeatures.seed(v, t);
            m_lastAnchorUpdateMs = t;
        }
        m_tele.state = m_state;
        m_tele.severity = 0.f;
        m_tele.stateSinceMs = m_stateSinceMs;
        return;
    }

    // ---- channel trips (a channel without data never votes) ----
    // NOTE there is deliberately no loss-run trip here. A miss run says the detector lost the
    // target; it says nothing about transmission, and PHD2's established lost-star path already
    // owns it. Promoting it to a scene verdict adds no evidence, only this monitor's much stricter
    // recovery latch. Real transmission collapse still fast-trips on mass or brightCeil below.
    const bool massFast   = m_tele.massRatio   >= 0.f && m_tele.massRatio   < m_fastMassRatio;
    const bool brightFast = m_tele.brightRatio >= 0.f && m_tele.brightRatio < m_fastBrightRatio;
    const float scoreBand = std::max(kScoreBandK * baseScoreMad, kScoreBandMin);
    const bool massLevelMed = m_tele.massRatio  >= 0.f && m_tele.massRatio    < m_medMassRatio;
    const bool brightMed  = m_tele.brightRatio  >= 0.f && m_tele.brightRatio  < m_medBrightRatio;
    const bool featureMed = m_tele.featureRatio >= 0.f && m_tele.featureRatio < m_medFeatureRatio;
    const bool snrLevelMed = haveSnr && m_tele.snrDropDb > m_medSnrDropDb;
    const bool massVariable = m_tele.massScatterFactor > 1.f;
    const bool snrVariable  = m_tele.snrScatterFactor > 1.f;
    const bool massMed    = massLevelMed || massVariable;
    const bool snrMed     = snrLevelMed || snrVariable;
    const bool scoreMed   = haveScore && m_tele.scoreDelta < -scoreBand;
    const int  votes      = (massMed ? 1 : 0) + (brightMed ? 1 : 0) + (featureMed ? 1 : 0) +
                            (snrMed ? 1 : 0) + (scoreMed ? 1 : 0);
    // Mass/SNR (and any future feature channel) are primary evidence. FWHM and sparse-field image
    // contrast are useful corroboration, but their normal noise must never publish HAZE alone.
    const bool primaryEvidence = massMed || snrMed || featureMed;
    const bool corroborated = primaryEvidence && votes >= 2;

    // A contrast collapse can fast-trip by itself only when the target is missing (physical
    // blackout). With a stable detection it is noisy supporting evidence and must be corroborated
    // through the sustained 2-of-N path below.
    const bool fastTrip = massFast || (!s.detected && brightFast);

    // ---- slow (gradual haze) vote, measured against the anchors rather than the baseline ----
    // Requires the detection-INDEPENDENT brightness channel plus one photometric channel. That
    // pairing keeps target-shape changes from voting by themselves: the detection-independent
    // brightness channel must fall along with mass or a future feature channel.
    const bool slowBright = m_tele.slowBrightRatio  >= 0.f && m_tele.slowBrightRatio  < m_slowRatio;
    const bool slowMass   = m_tele.slowMassRatio    >= 0.f && m_tele.slowMassRatio    < m_slowRatio;
    const bool slowFeat   = m_tele.slowFeatureRatio >= 0.f && m_tele.slowFeatureRatio < m_slowRatio;
    const bool slowVote   = slowBright && (slowMass || slowFeat);

    // Recovery mirrors the 2-of-N trip rule. Requiring every channel to remain above its recovery
    // threshold made a single noisy FWHM or contrast sample reset the hold forever. Contrast is
    // deliberately excluded here when the target is stably detected; it has already served its
    // blackout role, and genuine cloud degradation remains represented by mass/SNR/score/features.
    const bool massNotRecovered = (m_tele.massRatio >= 0.f && m_tele.massRatio < m_recMassRatio) ||
                                  m_tele.massScatterFactor > kVariabilityRecover;
    const bool featureNotRecovered = m_tele.featureRatio >= 0.f &&
                                     m_tele.featureRatio < m_recFeatureRatio;
    const bool snrNotRecovered = (haveSnr && m_tele.snrDropDb > m_recSnrDropDb) ||
                                 m_tele.snrScatterFactor > kVariabilityRecover;
    const bool scoreNotRecovered = haveScore && m_tele.scoreDelta < -0.5f * scoreBand;
    const int recoveryBadChannels = (massNotRecovered ? 1 : 0) + (featureNotRecovered ? 1 : 0) +
                                    (snrNotRecovered ? 1 : 0) + (scoreNotRecovered ? 1 : 0);

    // Severity: how far the worst channel sits past its MEDIUM threshold, 0..1.
    auto excess = [](float ratio, float thr) {
        return (ratio >= 0.f && thr > kEps && ratio < thr) ? std::min(1.f, (thr - ratio) / thr) : 0.f;
    };
    float sev = std::max({ excess(m_tele.massRatio, m_medMassRatio),
                           excess(m_tele.brightRatio, m_medBrightRatio),
                           excess(m_tele.featureRatio, m_medFeatureRatio) });
    if (snrMed)   sev = std::max(sev, std::min(1.f, (m_tele.snrDropDb - m_medSnrDropDb) / 6.f + 0.3f));
    if (scoreMed) sev = std::max(sev, 0.3f);
    auto scatterExcess = [](float factor) {
        // Crossing the variability band begins at zero severity and ramps deliberately slowly:
        // twice the band is only 25%, not the former 50% cliff.
        return factor > 1.f ? std::min(1.f, (factor - 1.f) / 4.f) : 0.f;
    };
    sev = std::max(sev, scatterExcess(m_tele.massScatterFactor));
    sev = std::max(sev, scatterExcess(m_tele.snrScatterFactor));
    sev = std::max(sev, excess(m_tele.slowBrightRatio, m_slowRatio));
    m_tele.severity = (primaryEvidence || slowVote || fastTrip) ? sev : 0.f;

    switch (m_state) {
    case SceneState::Clear:
    case SceneState::Suspect: {
        if (fastTrip) {
            transitionLocked(SceneState::Obscured,
                massFast ? "mass collapse" : brightFast ? "brightness collapse" : "detection lost", t);
            break;
        }
        if (corroborated) {
            if (m_mediumTripSinceMs == 0)
                m_mediumTripSinceMs = t;
            if (t - m_mediumTripSinceMs >= m_mediumWindowMs) {
                transitionLocked(SceneState::Obscured, "sustained degradation (2-of-N)", t);
                break;
            }
        }
        else {
            m_mediumTripSinceMs = 0;
        }
        // Gradual fade: the anchors have opened a sustained gap the rolling baseline cannot see.
        if (slowVote) {
            if (m_slowTripSinceMs == 0)
                m_slowTripSinceMs = t;
            if (t - m_slowTripSinceMs >= kSlowDwellMs) {
                transitionLocked(SceneState::Obscured, "slow haze (anchor)", t);
                break;
            }
        }
        else {
            m_slowTripSinceMs = 0;
        }
        if (primaryEvidence || slowVote) {
            m_suspectQuietSinceMs = 0;
            if (m_state == SceneState::Clear) {
                // Preserve the running vote windows across the Clear->Suspect edge: neither timer
                // may restart just because the state label changed (a slow fade would otherwise
                // re-arm its dwell on the very edge it caused, and never reach it).
                const int64_t medSince = m_mediumTripSinceMs;
                const int64_t slowSince = m_slowTripSinceMs;
                transitionLocked(SceneState::Suspect,
                    slowVote && !primaryEvidence ? "slow fade against anchor" : "primary channel degraded", t);
                m_mediumTripSinceMs = medSince;
                m_slowTripSinceMs = slowSince;
            }
        }
        else if (m_state == SceneState::Suspect) {
            if (m_suspectQuietSinceMs == 0)
                m_suspectQuietSinceMs = t;
            else if (t - m_suspectQuietSinceMs >= kSuspectQuietMs)
                transitionLocked(SceneState::Clear, "channels recovered", t);
        }
        break;
    }

    case SceneState::Obscured: {
        const bool recovered = s.detected && s.stableLock && m_lossRun == 0 &&
                               !massFast && recoveryBadChannels < 2 && !slowVote;
        if (recovered) {
            if (m_recoverSinceMs == 0)
                m_recoverSinceMs = t;
            else if (t - m_recoverSinceMs >= kRecoverHoldMs) {
                transitionLocked(SceneState::Clear, "view recovered", t);
                break;
            }
        }
        else {
            m_recoverSinceMs = 0;
        }
        // Static-obstruction escalation: long obscuration with a flat (non-cloud-like)
        // brightness signature -- a branch / dew never fluctuates the way moving cloud does.
        if (!m_tele.staticObstruction && t - m_stateSinceMs >= kStuckMs) {
            float med = 0.f, mad = 0.f;
            if (m_shortBright.medianMad(med, mad) && med > kEps && mad / med < kStaticMadFrac) {
                m_tele.staticObstruction = true;
                logLocked("cloud: prolonged STATIC obstruction (branch/dew?), user attention needed");
            }
        }
        break;
    }

    default:
        break;
    }

    m_tele.state = m_state;
    m_tele.stateSinceMs = m_stateSinceMs;

    // Shadow-mode heartbeat while degraded.
    if (m_state != SceneState::Clear && m_log &&
        (m_lastPeriodicLogMs == 0 || t - m_lastPeriodicLogMs >= kPeriodicLogMs)) {
        m_lastPeriodicLogMs = t;
        char buf[384];
        std::snprintf(buf, sizeof(buf),
            "cloud: %s sev=%.2f mass=%.2f massVar=%.2f bright=%.2f "
            "snrDrop=%.1fdB snrVar=%.2f score%+.2f feat=%.2f "
            "slow[bright=%.2f mass=%.2f feat=%.2f] loss=%d recBad=%d slowVote=%d",
            StateName(m_state), m_tele.severity, m_tele.massRatio, m_tele.massScatterFactor,
            m_tele.brightRatio, m_tele.snrDropDb, m_tele.snrScatterFactor,
            m_tele.scoreDelta, m_tele.featureRatio,
            m_tele.slowBrightRatio, m_tele.slowMassRatio, m_tele.slowFeatureRatio, m_lossRun,
            recoveryBadChannels, slowVote ? 1 : 0);
        logLocked(buf);
    }
    else if (m_state == SceneState::Clear) {
        m_lastPeriodicLogMs = 0;
    }
}

void CloudDetector::logLocked(const char* msg) noexcept
{
    if (!m_log)
        return;

    try {
        m_log(std::string(msg ? msg : ""));
    }
    catch (...) {
        // Logging is diagnostic only. Disable a callback after its first exception so every
        // subsequent guide frame does not pay for and rethrow the same broken dependency.
        m_log = nullptr;
        noteExceptionLocked(true);
    }
}
