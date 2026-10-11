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
#include <iomanip>
#include <locale>
#include <sstream>

namespace {

// Local aliases for the tuning block (shadow-mode starting points -- refine from logged
// real-session/replay data before using the telemetry for automated decisions).
constexpr int   kShortK           = CONFIG_CLOUD_SHORT_K;
constexpr int   kBaselineDecimMs  = CONFIG_CLOUD_BASELINE_DECIM_MS;
constexpr int   kMinBaseEntries   = CONFIG_CLOUD_MIN_BASE_ENTRIES;
constexpr int   kWarmupMs         = CONFIG_CLOUD_WARMUP_MS;
constexpr int   kLossRunTrip      = CONFIG_CLOUD_LOSS_RUN_TRIP;
constexpr int   kSuspectQuietMs   = CONFIG_CLOUD_SUSPECT_QUIET_MS;
constexpr float kRecoverMargin    = CONFIG_CLOUD_RECOVER_MARGIN;
constexpr float kRecoverCap       = CONFIG_CLOUD_RECOVER_CAP;
constexpr float kRecoverSnrMargin = CONFIG_CLOUD_RECOVER_SNR_MARGIN;
constexpr float kAnchorDownPctMin = CONFIG_CLOUD_ANCHOR_DOWN_PCT_MIN;
constexpr float kSlowRatio        = CONFIG_CLOUD_SLOW_RATIO;
constexpr float kScoreBandK       = CONFIG_CLOUD_SCORE_BAND_K;
constexpr float kScoreBandMin     = CONFIG_CLOUD_SCORE_BAND_MIN;
constexpr int   kVariabilityK     = CONFIG_CLOUD_VARIABILITY_K;
constexpr int   kJoinBeforeK      = CONFIG_CLOUD_EXPOSURE_JOIN_BEFORE_K;
constexpr float kVariabilitySigmaK = CONFIG_CLOUD_VARIABILITY_SIGMA_K;
constexpr float kVariabilityRecover = CONFIG_CLOUD_VARIABILITY_RECOVER_FACTOR;
constexpr float kMassVarFracMin   = CONFIG_CLOUD_MASS_VARIABILITY_FRAC_MIN;
constexpr float kSnrVarDbMin      = CONFIG_CLOUD_SNR_VARIABILITY_DB_MIN;
constexpr int64_t kStuckMs        = CONFIG_CLOUD_STUCK_MS;
constexpr float kStaticMadFrac    = CONFIG_CLOUD_STATIC_MAD_FRAC;
constexpr int64_t kAlternateBaselineMs = CONFIG_CLOUD_ALTERNATE_BASELINE_MS;
constexpr float kAlternateRelMad       = CONFIG_CLOUD_ALTERNATE_REL_MAD;
constexpr float kAlternateSnrMadDb     = CONFIG_CLOUD_ALTERNATE_SNR_MAD_DB;
constexpr float kAlternateMinSnrDb     = CONFIG_CLOUD_ALTERNATE_MIN_SNR_DB;
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
// Recovery trend
// ---------------------------------------------------------------------------

void CloudDetector::Trend::push(float value, int64_t t)
{
    if (n > 0 && t - times[(head - 1 + kCap) % kCap] < CONFIG_CLOUD_TREND_SAMPLE_MS)
        return;
    values[head] = value;
    times[head] = t;
    head = (head + 1) % kCap;
    if (n < kCap) ++n;
}

bool CloudDetector::Trend::settled(int64_t t, float fractionFloor, float absoluteFloor,
                                  float clearMad, bool& improving, bool* ready) const
{
    improving = false;
    if (ready) *ready = false;
    static_assert(CONFIG_CLOUD_TREND_SAMPLE_MS > 0, "trend cadence must be positive");
    static_assert(CONFIG_CLOUD_RECOVERY_TREND_MS > 0 &&
                  CONFIG_CLOUD_RECOVERY_TREND_MS / CONFIG_CLOUD_TREND_SAMPLE_MS + 2 <= kCap,
                  "recovery trend window must fit the time ring");
    if (n < 2) return false;
    const int64_t cadence = times[(head - 1 + kCap) % kCap] - times[(head - 2 + kCap) % kCap];
    const int64_t window = std::max<int64_t>(CONFIG_CLOUD_RECOVERY_TREND_MS, 4 * cadence);
    const int64_t start = t - window;
    const int64_t middle = t - window / 2;
    Ring early, late;
    bool fullWindow = false;
    // Include one observation on/before the boundary so non-dividing frame cadences work.
    for (int i = 0; i < n; ++i) {
        const int index = (head - 1 - i + kCap) % kCap;
        if (times[index] <= middle) early.push(values[index]);
        else late.push(values[index]);
        if (times[index] <= start) { fullWindow = true; break; }
    }
    // Slow exposures extend the window to supply at least two samples in each half.
    if (!fullWindow || early.n < 2 || late.n < 2)
        return false;
    if (ready) *ready = true;
    float before = 0.f, after = 0.f;
    early.median(before); late.median(after);
    const double noiseBand = CONFIG_CLOUD_TREND_NOISE_K * (double) clearMad /
                             std::sqrt((double) std::min(early.n, late.n));
    const double band = std::max({ (double) absoluteFloor,
                                  fractionFloor * std::fabs((double) before), noiseBand });
    const double change = (double) after - before;
    improving = change > band;
    return std::fabs(change) <= band;
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
    m_medMassRatio    = std::min(0.92f, CONFIG_CLOUD_MED_MASS_RATIO * f);
    m_medBrightRatio  = std::min(0.92f, CONFIG_CLOUD_MED_BRIGHT_RATIO * f);
    m_medFeatureRatio = std::min(0.92f, CONFIG_CLOUD_MED_FEATURE_RATIO * f);
    m_medSnrDropDb    = CONFIG_CLOUD_MED_SNR_DROP_DB / f;
    m_slowRatio       = std::min(0.96f, kSlowRatio * f);

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
        logReplayLocked("sensitivity");
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
        logReplayLocked("enabled");
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
        logReplayLocked("attach");
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
        logReplayLocked("reset");
        resetLocked(reason);
    }
    catch (const std::exception& e) {
        containException("Reset", e.what());
    }
    catch (...) {
        containException("Reset", "non-standard exception");
    }
}

void CloudDetector::StartNewSegment(const char* reason) noexcept
{
    try {
        std::lock_guard<std::mutex> lk(m_mx);
        logReplayLocked("segment");
        resetLocked(reason, true, true);
    }
    catch (const std::exception& e) {
        containException("StartNewSegment", e.what());
    }
    catch (...) {
        containException("StartNewSegment", "non-standard exception");
    }
}

void CloudDetector::ResumeAfterMotion(const char* reason) noexcept
{
    try {
        std::lock_guard<std::mutex> lk(m_mx);
        logReplayLocked("resume");
        resumeAfterMotionLocked(reason);
    }
    catch (const std::exception& e) {
        containException("ResumeAfterMotion", e.what());
    }
    catch (...) {
        containException("ResumeAfterMotion", "non-standard exception");
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

int64_t CloudDetector::sampleGapLimitLocked(int currentExposureMs) const
{
    return std::max<int64_t>(CONFIG_CLOUD_SAMPLE_GAP_MIN_MS,
        (int64_t) CONFIG_CLOUD_SAMPLE_GAP_EXPOSURES * std::max(currentExposureMs, m_prevExposureMs) +
            CONFIG_CLOUD_GUIDE_WAIT_ALLOWANCE_MS);
}

bool CloudDetector::recoveryHoldCompleteLocked(int64_t t, int64_t& sinceMs, int holdMs)
{
    static_assert(CONFIG_CLOUD_RECOVER_MIN_SAMPLES >= 2, "recovery needs distinct observations");
    if (sinceMs == 0) {
        sinceMs = t;
        m_recoverySamples = 1;
    }
    else if (m_recoverySamples < CONFIG_CLOUD_RECOVER_MIN_SAMPLES) {
        ++m_recoverySamples;
    }
    return m_recoverySamples >= CONFIG_CLOUD_RECOVER_MIN_SAMPLES && t - sinceMs >= holdMs;
}

SceneTelemetry CloudDetector::GetTelemetry(int64_t nowMs) const noexcept
{
    try {
        std::lock_guard<std::mutex> lk(m_mx);
        SceneTelemetry snapshot = m_tele;
        if (nowMs > 0 && (m_prevSampleMs == 0 || nowMs < m_prevSampleMs ||
                         nowMs - m_prevSampleMs > sampleGapLimitLocked())) {
            snapshot.fresh = false;
            snapshot.recoverySettled = false;
            snapshot.clearingTrend = false;
        }
        return snapshot;
    }
    catch (...) {
        SceneTelemetry unavailable;
        unavailable.healthy = false;
        return unavailable;
    }
}

// Follow the view UP at once; drift DOWN no faster than kAnchorDownPctMin per minute, or never
// when `mayFall` is false. The asymmetry is the whole point: an unconstrained reference adapts to
// a fade and normalizes it away, while a permanently lower but stable scene can still become
// nominal after roughly an observing-scale interval rather than requiring a restart.
void CloudDetector::Anchor::update(float v, int64_t t, bool mayFall)
{
    if (!valid()) {
        seed(v, t);
        return;
    }
    if (v >= value) {
        value = v;
        tMs = t;
    }
    else if (mayFall) {
        slideToward(v, t);
    }
    else {
        tMs = t;
    }
}

void CloudDetector::Anchor::slideToward(float v, int64_t t)
{
    if (valid() && v < value) {
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
    id.autoExposure = s.brightExposureMs > 0 ? (s.exposureMs <= 0 ? 1 : 0) : -1;
    id.gain   = s.gain;
    id.bitDepth = s.bitDepth;
    id.mode   = s.mode;
    id.frameW = s.frameW; id.frameH = s.frameH;
    id.roiX = s.roiX; id.roiY = s.roiY; id.roiW = s.roiW; id.roiH = s.roiH;
    id.sourceGen = s.sourceGen;
    if (s.mode == 0) {
        id.zenith = CloudAirmass::Normalizes(s.extinctionMagPerAirmass, s.airmass) ? 1 : 0;
        id.airmass = id.zenith ? s.airmass : -1.f;
    }
    id.tMs = s.tMs;
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
    // Stellar mode compares mass per second and joins it across a change of the fixed exposure (feedLocked).
    if (changed(p.exposureMs, c.exposureMs) && !(p.mode == 0 && c.mode == 0)) return "exposure change";
    if (changed(p.autoExposure, c.autoExposure)) return "auto-exposure mode change";
    if (changed(p.gain, c.gain))             return "gain change";
    if (changed(p.bitDepth, c.bitDepth))     return "bit-depth change";
    if (changed(p.mode, c.mode))             return "detection mode change";
    if (changed(p.frameW, c.frameW) || changed(p.frameH, c.frameH))
        return "frame geometry change";
    if (changed(p.roiX, c.roiX) || changed(p.roiY, c.roiY) ||
        changed(p.roiW, c.roiW) || changed(p.roiH, c.roiH))
        return "ROI change";
    if (p.sourceGen != c.sourceGen)          return "source change";
    // Corrected and uncorrected stellar photometry differ by the whole zenith correction, so references
    // learned on one cannot measure the other (Ticket 005 #8).
    if (changed(p.zenith, c.zenith))         return "air mass availability change";
    if (p.zenith == 1 && c.zenith == 1) {
        const double minutes = std::max(0., (double) (c.tMs - p.tMs) / 60000.);
        if (std::fabs(c.airmass - p.airmass) > CONFIG_CLOUD_AIRMASS_JUMP + CONFIG_CLOUD_AIRMASS_JUMP_PER_MIN * minutes)
            return "air mass jump";
    }
    return nullptr;
}

void CloudDetector::clearStateLocked(bool preserveDecline) noexcept
{
    if (++m_referenceGeneration == 0)
        ++m_referenceGeneration;
    m_identBeforeReset = true;
    m_baseMass.clear(); m_baseBright.clear(); m_baseSnr.clear();
    m_baseScore.clear(); m_baseFeatures.clear();
    m_shortMass.clear(); m_shortBright.clear(); m_shortSnr.clear();
    m_shortScore.clear(); m_shortFeatures.clear();
    m_shortEnsemble.clear();
    m_trendMass.clear(); m_trendSnr.clear(); m_trendFeatures.clear(); m_trendEnsemble.clear();
    m_refMass.clear(); m_refBright.clear(); m_refSnr.clear();
    m_refScore.clear(); m_refFeatures.clear();
    m_anchorBright.clear(); m_anchorMass.clear(); m_anchorSnr.clear(); m_snrSessionBest.clear();
    m_anchorEnsemble.clear(); m_baseEnsemble.clear();
    m_anchorScore.clear(); m_anchorFeatures.clear();
    m_anchorAirmass = -1.f;
    m_seenMass = m_seenSnr = m_seenScore = m_seenFeatures = false;
    m_lastBaselinePushMs = 0;
    m_lastAnchorUpdateMs = 0;
    m_clearAccumMs = 0;
    m_prevExposureMs = 0;
    m_prevClearEligible = false;
    m_massJoin = 1.;
    m_joinBefore = m_joinEnsembleBefore = -1.f;
    m_joinHold = 0;
    m_motionMass = m_motionEnsemble = m_motionScatter = -1.f;
    m_motionMs = 0;
    m_joinMass.clear(); m_joinEnsemble.clear();
    m_snrRelearning = false;
    // m_lastEnsembleMs survives: a reset does not change whether the guider supplies a multi-star ratio.
    m_clipBits = 0; m_clipFrames = 0; m_lastClipMs = 0;
    if (m_primaryHeldOut) {
        m_primaryHeldOut = false;
        logLocked("cloud: guide star back in the mass channels (detector reset)");
    }
    m_unclippedRun.clear(); m_unclippedLevel = -1.f;
    m_joinClipped = m_joinDeferred = false;
    m_deferredFromMs = m_deferredToMs = 0;
    m_prevSampleMs = 0;
    m_alternateSinceMs = 0;
    m_suspectQuietSinceMs = 0;
    m_lossRun = 0;
    m_sawGoodDetection = false;
    m_lossLogged = false;
    m_lastPeriodicLogMs = 0;     // else the heartbeat stays silent for up to a minute after a reset
    m_faultRecoverySamples = 0;
    m_faultRecoveryLastMs = 0;
    m_state = SceneState::Warmup;
    m_obscuredRecovery = false;
    m_stateSinceMs = 0;
    if (preserveDecline) m_massDecline.resume(true); else m_massDecline.reset();
    m_skyStability.reset();
    m_tele = SceneTelemetry{};
    m_tele.massDeclineLatched = m_massDecline.latched;
    m_tele.massDeclineUsesEnsemble = m_massDecline.usesEnsemble;
    m_tele.exceptionCount = m_exceptionCount;
    m_tele.loggerExceptionCount = m_loggerExceptionCount;
    m_tele.referenceGeneration = m_referenceGeneration;
    m_tele.faultRecoverySamples = m_faultRecoverySamples;
    m_tele.faultAutoRetries = m_faultAutoRetries;
    m_tele.faultRetryLimit = CONFIG_CLOUD_FAULT_MAX_AUTO_RETRIES;
}

void CloudDetector::resetLocked(const char* reason, bool resetFaultRetries, bool preserveDecline)
{
    if (resetFaultRetries)
        m_faultAutoRetries = 0;
    clearStateLocked(preserveDecline);
    char buf[160];
    std::snprintf(buf, sizeof(buf), "cloud: reset (%s) -> warm-up", reason ? reason : "?");
    logLocked(buf);
}

void CloudDetector::resumeAfterMotionLocked(const char* reason) noexcept
{
    // Keep the learned clear-scene baselines/anchors and the latched state. In particular, a
    // dither must not turn a genuine pre-existing Obscured verdict into Warmup/Clear. Only the
    // evidence that can have been contaminated by deliberate motion or reacquisition is stale.
    // The primary's level before the motion is kept for an exposure change on the next frames.
    float level = 0.f, mad = 0.f;
    if ((m_shortMass.lastMedianMad(kJoinBeforeK, level, mad) || m_shortMass.lastMedianMad(kShortK, level, mad)) &&
        level > 0.f) {
        m_motionMass = level;
        m_motionScatter = mad / level;
        m_motionEnsemble =
            (m_shortEnsemble.lastMedian(kJoinBeforeK, level) || m_shortEnsemble.lastMedian(kShortK, level)) && level > 0.f
                ? level
                : -1.f;
        m_motionMs = m_prevSampleMs;
    }
    m_shortMass.clear(); m_shortBright.clear(); m_shortSnr.clear();
    m_shortScore.clear(); m_shortFeatures.clear();
    m_shortEnsemble.clear();
    m_trendMass.clear(); m_trendSnr.clear(); m_trendFeatures.clear(); m_trendEnsemble.clear();
    m_refMass.clear(); m_refBright.clear(); m_refSnr.clear();
    m_refScore.clear(); m_refFeatures.clear();
    m_unclippedRun.clear();
    m_lastBaselinePushMs = 0;
    m_lastAnchorUpdateMs = 0;
    m_prevExposureMs = 0;
    m_prevClearEligible = false;
    m_prevSampleMs = 0;         // do not count the suspended interval as detector dwell time
    m_alternateSinceMs = 0;
    m_suspectQuietSinceMs = 0;
    m_lossRun = 0;
    m_sawGoodDetection = false;
    m_lossLogged = false;
    m_lastPeriodicLogMs = 0;
    m_faultRecoverySamples = 0;
    m_faultRecoveryLastMs = 0;

    m_massDecline.resume();
    m_skyStability.resume();
    m_tele.skyStabilityReady = m_tele.skyImproving = m_tele.skyStable = false;
    SceneTelemetry resumed;
    resumed.massDeclineLatched = m_massDecline.latched;
    resumed.massDeclineUsesEnsemble = m_massDecline.usesEnsemble;
    resumed.state = m_state;
    resumed.primarySaturated = m_tele.primarySaturated;
    resumed.primaryHeldOut = m_primaryHeldOut;
    resumed.healthy = m_tele.healthy;
    resumed.exceptionCount = m_exceptionCount;
    resumed.loggerExceptionCount = m_loggerExceptionCount;
    resumed.staticObstruction = m_tele.staticObstruction;
    resumed.stateSinceMs = m_stateSinceMs;
    resumed.referenceGeneration = m_referenceGeneration;
    resumed.faultRecoverySamples = m_faultRecoverySamples;
    resumed.faultAutoRetries = m_faultAutoRetries;
    resumed.faultRetryLimit = CONFIG_CLOUD_FAULT_MAX_AUTO_RETRIES;
    m_tele = resumed;

    char buf[192];
    std::snprintf(buf, sizeof(buf), "cloud: resume after motion (%s); transient evidence cleared",
                  reason ? reason : "?");
    logLocked(buf);
}

// Stellar mode, change of the fixed guide exposure (Ticket 005). The references, verdict, decline and sky stability
// continue on per-second mass. S/N does not scale with exposure, so its channel restarts and stays optional until its
// standard and trend have refilled. The primary's next kShortK masses, or kJoinBeforeK when its last frames scatter
// widely, are held out of the mass channels to measure its step at the new exposure, against its last frames or,
// right after a dither, its level before the motion.
void CloudDetector::beginExposureJoinLocked(int fromMs, int toMs, int64_t t)
{
    m_shortSnr.clear(); m_refSnr.clear(); m_baseSnr.clear(); m_trendSnr.clear();
    m_anchorSnr.clear(); m_snrSessionBest.clear();
    m_snrRelearning = m_seenSnr;
    float v = 0.f, mad = 0.f, scatter = -1.f;
    const char* against = "";
    if ((m_shortMass.lastMedianMad(kJoinBeforeK, v, mad) || m_shortMass.lastMedianMad(kShortK, v, mad)) && v > 0.f) {
        m_joinBefore = v;
        scatter = mad / v;
        m_joinEnsembleBefore =
            (m_shortEnsemble.lastMedian(kJoinBeforeK, v) || m_shortEnsemble.lastMedian(kShortK, v)) && v > 0.f ? v : -1.f;
    }
    else if (m_motionMass > 0.f && m_motionMs > 0 && t - m_motionMs <= CONFIG_CLOUD_EXPOSURE_JOIN_MOTION_MS) {
        m_joinBefore = m_motionMass;
        m_joinEnsembleBefore = m_motionEnsemble;
        scatter = m_motionScatter;
        against = " against the level before the motion";
    }
    else
        m_joinBefore = m_joinEnsembleBefore = -1.f;
    m_joinHold = scatter > CONFIG_CLOUD_EXPOSURE_JOIN_NOISY_MAD ? kJoinBeforeK : kShortK;
    m_motionMass = m_motionEnsemble = m_motionScatter = -1.f;
    m_joinMass.clear(); m_joinEnsemble.clear();
    m_joinFromMs = fromMs; m_joinToMs = toMs;
    // Whether the primary clipped in its last 9 frames before the change; its first frames after are added as they come.
    // Held out (clipping, no multi-star ratio), it has no recent mass to join: it joins once unclipped for 9 frames.
    const bool recentClip = CONFIG_CLOUD_CLIP_HOLD_MS > 0 && m_lastClipMs > 0 && t - m_lastClipMs < CONFIG_CLOUD_CLIP_HOLD_MS;
    m_joinClipped = m_clipBits != 0 || recentClip;
    m_unclippedRun.clear();
    if (m_joinDeferred) m_deferredToMs = toMs;
    else if (m_joinBefore <= 0.f && m_joinClipped) {
        m_joinDeferred = true;
        m_deferredFromMs = fromMs;
        m_deferredToMs = toMs;
    }
    char joins[64] = "no recent primary mass to join";
    if (m_joinBefore > 0.f)
        std::snprintf(joins, sizeof(joins), "primary mass joins after %d frames", m_joinHold);
    else if (m_joinDeferred)
        std::snprintf(joins, sizeof(joins), "the primary clips: it joins after %d unclipped frames", kJoinBeforeK);
    char buf[192];
    std::snprintf(buf, sizeof(buf), "cloud: exposure change %d -> %d ms; references kept, S/N relearns, %s%s", fromMs, toMs,
                  joins, against);
    logLocked(buf);
}

// Joins the primary's per-second mass to the new exposure at the step of its first frames, and returns the current
// frame's mass at the joined scale. The held frames enter the trip window at that scale. The step stands as sky when
// the multi-star ratio moved with it, or, without a multi-star ratio, when the camera cannot give it at this exposure
// ratio: between linear (1) and fully clipped (from/to), widened by the aperture's margin, within the cap.
float CloudDetector::finishExposureJoinLocked(float current)
{
    float after = 0.f, ensembleAfter = 0.f;
    const bool haveAfter = m_joinMass.median(after) && after > 0.f;
    const bool haveEnsemble = m_joinEnsembleBefore > 0.f && m_joinEnsemble.median(ensembleAfter) && ensembleAfter > 0.f;
    const double step = haveAfter ? (double) after / m_joinBefore : 1.;
    const double ensembleStep = haveEnsemble ? (double) ensembleAfter / m_joinEnsembleBefore : 1.;
    const double clipped = (double) m_joinFromMs / (double) m_joinToMs;
    const double lowest =
        std::max(1. - CONFIG_CLOUD_EXPOSURE_JOIN_CAP, std::min(1., clipped) - CONFIG_CLOUD_EXPOSURE_JOIN_MARGIN);
    const double highest =
        std::min(1. + CONFIG_CLOUD_EXPOSURE_JOIN_CAP, std::max(1., clipped) + CONFIG_CLOUD_EXPOSURE_JOIN_MARGIN);
    // Ticket 005 follow-up. Without a multi-star ratio around the change, a primary that clipped measured the camera,
    // not the sky: its join waits until it has been unclipped for 9 frames and 5 min (finishDeferredJoinLocked). A
    // primary that did not clip is linear, so only the aperture's margin is the camera's. With a multi-star ratio the
    // join is as before: the median outvotes a clipped primary, and joining it at the multi-star step instead changed
    // a confirmation on 10-11 October (doc/005-repro/saturation).
    const bool clippedJoin = m_joinClipped;
    const double low = clippedJoin ? lowest : std::max(1. - CONFIG_CLOUD_EXPOSURE_JOIN_CAP, 1. - CONFIG_CLOUD_EXPOSURE_JOIN_MARGIN);
    const double high = clippedJoin ? highest : std::min(1. + CONFIG_CLOUD_EXPOSURE_JOIN_CAP, 1. + CONFIG_CLOUD_EXPOSURE_JOIN_MARGIN);
    const char* sky = nullptr;
    bool defer = false;
    if (!haveAfter || !std::isfinite(step))
        sky = "no primary mass";
    else if (haveEnsemble && std::fabs(ensembleStep - 1.) > CONFIG_CLOUD_EXPOSURE_JOIN_ENSEMBLE_TOLERANCE)
        sky = "the multi-star ratio moved";
    else if (clippedJoin && !haveEnsemble)
        defer = true;
    else if (!haveEnsemble && (step < low || step > high))
        sky = "more than the camera gives";
    const bool join = sky == nullptr && !defer;
    const double factor = join ? 1. / step : 1.;
    m_massJoin *= factor;
    if (!defer)
        for (int i = m_joinMass.n - 1; i >= 1; --i)
            m_shortMass.push((float) (m_joinMass.buf[(m_joinMass.head - 1 - i + 2 * Ring::kCap) % Ring::kCap] * factor));
    char ensemble[48] = ", multi-star unavailable";
    if (haveEnsemble)
        std::snprintf(ensemble, sizeof(ensemble), ", multi-star %+.1f%%", 100. * (ensembleStep - 1.));
    char buf[256];
    if (defer) {
        if (!m_joinDeferred) m_deferredFromMs = m_joinFromMs;
        m_joinDeferred = true;
        m_deferredToMs = m_joinToMs;
        std::snprintf(buf, sizeof(buf), "cloud: exposure %d -> %d ms join deferred: the primary clipped (step %+.1f%%%s); it "
                      "joins after %d unclipped frames", m_joinFromMs, m_joinToMs, 100. * (step - 1.), ensemble, kJoinBeforeK);
    }
    else if (join)
        std::snprintf(buf, sizeof(buf), "cloud: exposure %d -> %d ms joined: primary mass x%.3f (step %+.1f%%%s, %d frames%s)",
                      m_joinFromMs, m_joinToMs, factor, 100. * (step - 1.), ensemble, m_joinMass.n,
                      clippedJoin ? "; the primary clipped" : "");
    else
        std::snprintf(buf, sizeof(buf), "cloud: exposure %d -> %d ms not joined: x%.3f stands (step %+.1f%%%s; %s)",
                      m_joinFromMs, m_joinToMs, factor, 100. * (step - 1.), ensemble, sky);
    logLocked(buf);
    m_joinBefore = m_joinEnsembleBefore = -1.f;
    m_joinMass.clear(); m_joinEnsemble.clear();
    m_joinClipped = false;
    return (float) (current * factor);
}

// A join deferred because the primary clipped and no multi-star ratio stood in (Ticket 005 follow-up): once it has
// been unclipped for 9 frames at the new exposure, its level joins the level of its last 9 unclipped frames before the
// change. Both are unclipped, so the camera gives only the aperture's margin; a larger step stands as sky.
float CloudDetector::finishDeferredJoinLocked(float current, float level)
{
    m_joinDeferred = false;
    const double step = m_unclippedLevel > 0.f ? (double) level / m_unclippedLevel : -1.;
    const bool join = step > 0. && std::isfinite(step) && std::fabs(step - 1.) <= CONFIG_CLOUD_EXPOSURE_JOIN_MARGIN;
    const double factor = join ? 1. / step : 1.;
    m_massJoin *= factor;
    for (int i = 0; i < m_unclippedRun.n; ++i) m_unclippedRun.buf[i] = (float) (m_unclippedRun.buf[i] * factor);
    char buf[256];
    if (step <= 0.)
        std::snprintf(buf, sizeof(buf), "cloud: exposure %d -> %d ms deferred join: no unclipped level before the change; "
                      "x1.000 stands", m_deferredFromMs, m_deferredToMs);
    else
        std::snprintf(buf, sizeof(buf), "cloud: exposure %d -> %d ms %s: primary mass x%.3f (step %+.1f%% against its last "
                      "unclipped level, %d frames)", m_deferredFromMs, m_deferredToMs, join ? "joined after clipping" :
                      "not joined after clipping, the step stands", factor, 100. * (step - 1.), kJoinBeforeK);
    logLocked(buf);
    return (float) (current * factor);
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
        logReplayLocked("fault");
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
        "cloud: %s -> %s (%s) mass=%.2f massVar=%.2f bright=%.2f ensemble=%.2f "
        "snrDrop=%.1fdB snrVar=%.2f score%+.2f feat=%.2f "
        "slow[bright=%.2f mass=%.2f feat=%.2f] loss=%d",
        StateName(m_state), StateName(next), why ? why : "",
        m_tele.massRatio, m_tele.massScatterFactor, m_tele.brightRatio, m_tele.ensembleRatio,
        m_tele.snrDropDb,
        m_tele.snrScatterFactor,
        m_tele.scoreDelta, m_tele.featureRatio,
        m_tele.slowBrightRatio, m_tele.slowMassRatio, m_tele.slowFeatureRatio,
        m_tele.lossRun);
    logLocked(buf);
    m_state = next;
    if (next == SceneState::Obscured) m_obscuredRecovery = true;
    else if (next == SceneState::Clear) m_obscuredRecovery = false;
    m_stateSinceMs = tMs;
    m_alternateSinceMs = 0;
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

void CloudDetector::feedLocked(const SceneSample& recorded)
{
    std::lock_guard<std::mutex> lk(m_mx);
    logReplayLocked("feed", &recorded);

    // Stellar photometry is evaluated at zenith-equivalent flux, so extinction while a target rises or
    // sets is neither clearing nor haze. The replay record keeps the raw values and the correction inputs.
    SceneSample s = recorded;
    const double zenith = s.mode == 0 ? CloudAirmass::ZenithFactor(s.extinctionMagPerAirmass, s.airmass) : 1.;
    if (zenith != 1.) {
        if (std::isfinite(s.mass) && s.mass > 0.f) s.mass = (float) (s.mass * zenith);
        if (std::isfinite(s.brightCeil) && s.brightCeil > 0.f) s.brightCeil = (float) (s.brightCeil * zenith);
    }
    // A fixed exposure's stellar mass is compared per second, as brightCeil is, so the references survive a change of
    // the guide exposure (Ticket 005). The guider already normalizes automatic exposure; other modes relearn instead.
    if (s.mode == 0 && s.exposureMs > 0 && std::isfinite(s.mass) && s.mass > 0.f)
        s.mass = (float) (s.mass * (1000. / s.exposureMs));

    // Relearn acquisition references while preserving normalized stellar slope segments. A change of the fixed
    // exposure in stellar mode keeps them (beginExposureJoinLocked).
    const Identity cur = IdentityOf(s);
    // The first frame after a reset is not compared with the air mass of the frame before it: the references already
    // relearn, and a second reset would only restart the warm-up (Ticket 005 follow-up). Later frames are compared.
    Identity prev = m_ident;
    if (m_identBeforeReset) { prev.zenith = -1; prev.airmass = -1.f; }
    const char* discontinuity = IdentityDelta(prev, cur);
    if (m_identBeforeReset && !discontinuity && m_enabled) {
        if (const char* skipped = IdentityDelta(m_ident, cur)) {
            auto airmass = [](const Identity& id, char* out, size_t n) {
                if (id.zenith == 1) std::snprintf(out, n, "%.3f", id.airmass); else std::snprintf(out, n, "unknown");
            };
            char before[16], after[16], buf[192];
            airmass(m_ident, before, sizeof(before));
            airmass(cur, after, sizeof(after));
            std::snprintf(buf, sizeof(buf), "cloud: air mass %s -> %s across the reset is not an %s: the references "
                          "already relearn", before, after, skipped);
            logLocked(buf);
        }
    }
    const bool preserveDecline = m_ident.valid && m_ident.mode == 0 && cur.mode == 0;
    const bool exposureChange = preserveDecline && m_ident.exposureMs > 0 && cur.exposureMs > 0 &&
                                m_ident.exposureMs != cur.exposureMs;
    const int previousExposureMs = m_ident.exposureMs;
    m_ident = cur;
    if (!m_enabled)
        return;
    const int64_t t = s.tMs;
    if (!m_tele.healthy) {
        static_assert(CONFIG_CLOUD_FAULT_RECOVERY_SAMPLES >= 2, "fault recovery needs fresh samples");
        static_assert(CONFIG_CLOUD_FAULT_MAX_AUTO_RETRIES > 0, "fault recovery needs a retry budget");
        if (m_faultAutoRetries >= CONFIG_CLOUD_FAULT_MAX_AUTO_RETRIES)
            return;
        if (discontinuity) {
            m_faultRecoverySamples = 0;
            m_faultRecoveryLastMs = t;
            m_tele.faultRecoverySamples = 0;
            return;
        }
        if (t <= m_faultRecoveryLastMs) {
            if (t < m_faultRecoveryLastMs) {
                m_faultRecoverySamples = 0;
                m_faultRecoveryLastMs = t;
                m_tele.faultRecoverySamples = 0;
            }
            return;
        }
        m_faultRecoveryLastMs = t;
        const bool autoExposure = s.exposureMs <= 0 && s.brightExposureMs > 0;
        const bool recoverySample = t > 0 && s.detected && s.stableLock &&
            std::isfinite(s.brightCeil) && s.brightCeil >= 0.f &&
            ((std::isfinite(s.mass) && s.mass > 0.f) || s.features > 0 ||
             (!autoExposure && std::isfinite(s.snr)));
        if (!recoverySample) {
            m_faultRecoverySamples = 0;
            m_tele.faultRecoverySamples = 0;
            return;
        }
        ++m_faultRecoverySamples;
        m_tele.faultRecoverySamples = m_faultRecoverySamples;
        if (m_faultRecoverySamples < CONFIG_CLOUD_FAULT_RECOVERY_SAMPLES)
            return;
        ++m_faultAutoRetries;
        resetLocked("automatic fault recovery", false);
    }
    if (discontinuity)
        resetLocked(discontinuity, true, preserveDecline);
    else if (exposureChange)
        beginExposureJoinLocked(previousExposureMs, cur.exposureMs, t);

    if (t <= 0) {
        logLocked("cloud: ignored sample with invalid timestamp");
        return;
    }
    if (m_prevSampleMs > 0 && t == m_prevSampleMs)
        return; // A replayed frame must not advance a median or a dwell timer twice.
    if (m_prevSampleMs > 0 && t < m_prevSampleMs)
        resetLocked("timestamp moved backwards");
    m_identBeforeReset = false;   // this frame is the first that the references learn after any reset
    // brightExposureMs carries the real cadence even when exposureMs is suppressed for auto exposure.
    const int actualExposureMs = std::max(0, std::max(s.exposureMs, s.brightExposureMs));
    const int64_t gapLimit = sampleGapLimitLocked(actualExposureMs);
    if (m_prevSampleMs > 0 && t - m_prevSampleMs > gapLimit)
        resumeAfterMotionLocked("input gap");
    m_prevExposureMs = actualExposureMs;
    if (s.mode == 0 && m_massJoin != 1. && std::isfinite(s.mass) && s.mass > 0.f)
        s.mass = (float) (s.mass * m_massJoin);

    // A malformed upstream metric must be treated as unavailable, never inserted into a sorted
    // ring. NaN in particular does not provide the strict weak ordering std::sort requires and
    // can poison every subsequent ratio and JSON/HUD conversion.
    const bool validMass = s.detected && std::isfinite(s.mass) && s.mass > 0.f;
    const bool validFeatures = s.detected && s.features > 0;
    const bool autoExposure = s.exposureMs <= 0 && s.brightExposureMs > 0;
    const bool validSnr = s.detected && !autoExposure && std::isfinite(s.snr);
    const bool validScore = s.detected && std::isfinite(s.score);
    const bool validEnsemble = s.detected && s.ensembleStars > 0 &&
                               std::isfinite(s.ensembleRatio) && s.ensembleRatio > 0.f;
    // ---- a clipping primary (Ticket 005 follow-up) ----
    if (s.mode == 0 && validMass) {
        m_clipBits = ((m_clipBits << 1) | (s.primarySaturated ? 1u : 0u)) & ((1u << kJoinBeforeK) - 1u);
        if (m_clipFrames < kJoinBeforeK) ++m_clipFrames;
        if (s.primarySaturated) m_lastClipMs = t;
    }
    int clipped = 0;
    for (unsigned b = m_clipBits; b; b >>= 1) clipped += (int) (b & 1u);
    const bool recentClip = CONFIG_CLOUD_CLIP_HOLD_MS > 0 && m_lastClipMs > 0 && t - m_lastClipMs < CONFIG_CLOUD_CLIP_HOLD_MS;
    m_tele.primarySaturated = s.mode == 0 && m_clipFrames > 0 ? (float) clipped / (float) m_clipFrames : -1.f;
    // A primary that clipped in its last 9 frames or the last 5 min enters no loss or trend channel while multi-star is
    // not in use: its deficit is the camera's. Multi-star is in use while the option is on and a ratio came in the last
    // 30 min, kept across resets: a warm-up has no ratio while the per-star references relearn. The multi-star median
    // then outvotes the primary, and nothing changes. The state changes only on frames with a primary mass, so the
    // debug log has one line when the primary is held out and one when it comes back.
    if (validEnsemble) m_lastEnsembleMs = t;
    const bool multiStarInUse = validEnsemble ||
        (s.ensembleConfigured && m_lastEnsembleMs > 0 && t >= m_lastEnsembleMs &&
         t - m_lastEnsembleMs < CONFIG_CLOUD_MULTISTAR_RECENT_MS);
    if (s.mode == 0 && validMass) {
        const bool out = (clipped > 0 || recentClip) && !multiStarInUse;
        if (out != m_primaryHeldOut) {
            m_primaryHeldOut = out;
            char buf[200];
            if (out)
                std::snprintf(buf, sizeof(buf), "cloud: guide star held out of the mass channels: it clipped (%d of its last %d "
                              "frames) and no multi-star ratio stands in", clipped, m_clipFrames);
            else if (multiStarInUse)
                std::snprintf(buf, sizeof(buf), "cloud: guide star back in the mass channels (a multi-star ratio stands in)");
            else
                std::snprintf(buf, sizeof(buf), "cloud: guide star back in the mass channels (unclipped for %d frames and %d min)",
                              kJoinBeforeK, (int) (CONFIG_CLOUD_CLIP_HOLD_MS / 60000));
            logLocked(buf);
        }
    }
    const bool primaryOut = s.mode == 0 && validMass && m_primaryHeldOut;
    m_tele.primaryHeldOut = m_primaryHeldOut;

    // The first frames at a new stellar exposure measure the primary's step. Meanwhile its mass enters no mass
    // channel, so the windows keep describing the frames before the change; then the new exposure is joined.
    bool massHeld = false;
    if (m_joinBefore > 0.f && validMass) {
        if (s.primarySaturated) m_joinClipped = true;
        m_joinMass.push(s.mass);
        if (validEnsemble) m_joinEnsemble.push(s.ensembleRatio);
        if (m_joinMass.n < m_joinHold) massHeld = true;
        else s.mass = finishExposureJoinLocked(s.mass);
    }
    // The run of unclipped frames; after 9, a deferred join is finished and the unclipped level follows them.
    if (s.mode == 0 && validMass && !massHeld) {
        if (s.primarySaturated) m_unclippedRun.clear();
        else m_unclippedRun.push(s.mass);
        float level = 0.f;
        if (clipped == 0 && !recentClip && m_unclippedRun.lastMedian(kJoinBeforeK, level) && level > 0.f) {
            if (m_joinDeferred) {
                const double before = m_massJoin;
                s.mass = finishDeferredJoinLocked(s.mass, level);
                level = (float) (level * (m_massJoin / before));
            }
            m_unclippedLevel = level;
        }
    }
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
    m_tele.fresh = validMass || validSnr || validFeatures || validEnsemble || brightCeil >= 0.f;

    // Missing is not another vote for the last reading. Clear both transient histories,
    // retain the certified baseline, and require a fresh short window on reacquisition.
    if (!validMass || primaryOut) { m_shortMass.clear(); m_refMass.clear(); }
    if (!validSnr) { m_shortSnr.clear(); m_refSnr.clear(); }
    if (!validScore) { m_shortScore.clear(); m_refScore.clear(); }
    if (!validFeatures) { m_shortFeatures.clear(); m_refFeatures.clear(); }
    if (brightCeil < 0.f) { m_shortBright.clear(); m_refBright.clear(); }
    if (!validEnsemble) m_shortEnsemble.clear();

    // These raw standards do not follow the protected anchors upward during clearing.
    // Interrupted stable photometry must refill the time window before it can certify recovery.
    if (massHeld) {}
    else if (clearEligible && validMass && !primaryOut) m_trendMass.push(s.mass, t); else m_trendMass.clear();
    if (clearEligible && validSnr) m_trendSnr.push(s.snr, t); else m_trendSnr.clear();
    if (clearEligible && validFeatures) m_trendFeatures.push((float) s.features, t); else m_trendFeatures.clear();
    if (clearEligible && validEnsemble) m_trendEnsemble.push(s.ensembleRatio, t); else m_trendEnsemble.clear();

    if (!massHeld) {
        m_massDecline.update(t, s.mode == 0 ? CONFIG_CLOUD_MASS_DECLINE_PCT_PER_MINUTE : 0.f, s.cloudConfigGeneration,
                             clearEligible && validMass && !primaryOut ? s.mass : -1.f,
                             clearEligible && validEnsemble ? s.ensembleRatio : -1.f);
        m_skyStability.update(t, clearEligible && s.mode == 0 && validMass && !primaryOut ? s.mass : -1.f,
                             clearEligible && s.mode == 0 && validEnsemble ? s.ensembleRatio : -1.f, gapLimit);
    }
    m_tele.skyStabilityApplicable = s.mode == 0;
    m_tele.airmass = s.mode == 0 && std::isfinite(s.airmass) && s.airmass >= 1.f ? s.airmass : -1.f;
    m_tele.skyStabilityReady = m_skyStability.ready;
    m_tele.skyImproving = m_skyStability.improving;
    m_tele.skyStable = m_skyStability.stable;
    m_tele.skyRiseLoss = m_skyStability.ready ? m_skyStability.riseLoss : -1.f;
    m_tele.massDeclineRate = m_massDecline.rate;
    m_tele.massDeclineRatio = m_massDecline.ratio;
    m_tele.massDeclineLatched = m_massDecline.latched;
    m_tele.massDeclineUsesEnsemble = m_massDecline.usesEnsemble;

    // ---- short (fast) windows: the sky AS IT IS, unconditional ----
    // These drive the trips, so they must include the bad frames -- a blackout is undetected by
    // definition and still has to be measurable here. Zero is a REAL brightness reading (an
    // all-black ROI); only a negative means "no stats this frame". Rejecting zero discarded the
    // one measurement a total blackout produces, so with detection lost at the same moment --
    // which is what a blackout causes -- every detection channel was silenced and the brightness
    // window still held pre-blackout samples, leaving nothing able to trip.
    if (brightCeil >= 0.f)
        m_shortBright.push(brightCeil);
    if (validMass) m_seenMass = true;
    if (validMass && !massHeld && !primaryOut) m_shortMass.push(s.mass);
    if (validFeatures) { m_shortFeatures.push((float) s.features);   m_seenFeatures = true; }
    if (validSnr)      { m_shortSnr.push(s.snr);                      m_seenSnr = true; }
    if (validScore)    { m_shortScore.push(s.score);                  m_seenScore = true; }
    if (validEnsemble) m_shortEnsemble.push(s.ensembleRatio);

    // ---- reference windows: what CLEAR looks like, clear-eligible frames only ----
    // Baseline entries and anchor updates snapshot from THESE, never from the trip windows above.
    // The two jobs disagree -- a trip window must include the bad frames, a reference window must
    // exclude them -- so they cannot share a ring: gating only the write moment still snapshotted
    // a median of mixed history whenever an eligible frame arrived among unstable ones.
    if (clearEligible) {
        if (brightCeil >= 0.f) m_refBright.push(brightCeil);
        if (validMass && !massHeld && !primaryOut) m_refMass.push(s.mass);
        if (validFeatures) m_refFeatures.push((float) s.features);
        if (validSnr)      m_refSnr.push(s.snr);
        if (validScore)    m_refScore.push(s.score);
    }

    // Frames without a current detection carry no fresh detection-derived telemetry; their short
    // windows simply hold whatever the last detected frames left there. Voting on that stale data
    // would let an old value speak for a frame that never produced one, so the detection channels
    // are silent whenever the target is missing. brightCeil is measured from the pixels regardless
    // of detection, so it keeps voting -- and it is the channel that matters during an occultation.
    const bool detChannelsLive = s.detected;

    // ---- baselines: accumulate only while presumed clear (Warmup/Clear); frozen otherwise ----
    const bool accumulating = (m_state == SceneState::Warmup || m_state == SceneState::Clear);
    if (accumulating) {
        // Warm-up may only accumulate across frames that actually look like a clear, locked view.
        // Counting arbitrary background (bright sky, no target) let warm-up "establish" a
        // baseline while the target was already lost, then publish Clear and trip on the very
        // next frame. Unusable frames PAUSE the clock rather than resetting it, so ordinary
        // dropouts merely delay arming.
        if (m_prevSampleMs > 0 && clearEligible && m_prevClearEligible)
            m_clearAccumMs = std::min<int64_t>(kWarmupMs,
                m_clearAccumMs + std::min<int64_t>(t - m_prevSampleMs, 3000));

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
            if (!massHeld && !primaryOut && m_refMass.lastMedian(kShortK, v))
                m_baseMass.push(v);
            if (m_refSnr.lastMedian(kShortK, v))   m_baseSnr.push(v);
            if (m_refScore.lastMedian(kShortK, v)) m_baseScore.push(v);
            if (m_refFeatures.lastMedian(kShortK, v))
                m_baseFeatures.push(v);
            if (validEnsemble && m_shortEnsemble.lastMedian(kShortK, v))
                m_baseEnsemble.push(v);
        }
    }

    // Once armed, the protected standards have their own lifetime and update policy. They are
    // deliberately independent of the rolling baseline and state label: improvements become the
    // new standard without resetting through Warmup. Star mass and peak brightness measure haze, so
    // they do not slew down with time (Ticket 005 #3): a slide of CONFIG_CLOUD_ANCHOR_DOWN_PCT_MIN let a
    // steady 40% haze read 30% after 50 minutes, absorbed haze building at up to that rate, and made the
    // non-sliding multi-star ratio look like lone evidence, so a stable hazy sky was certified as a
    // new baseline. Air mass, their main benign drift, is normalized to zenith in stellar mode. The
    // configured extinction can be wrong for the night, so they may fall by what an error of
    // CONFIG_CLOUD_ANCHOR_EXTINCTION_TOLERANCE explains over the air-mass change since the previous
    // update; near the meridian that is nothing. Frames without a known air mass or extinction, and
    // other modes, keep the slide. S/N, FWHM and contrast still slew down slowly, since background,
    // seeing and focus change them without cloud. A loss confined to the guide optics, such as dew,
    // now persists until guiding restarts.
    if (m_state != SceneState::Warmup && clearEligible &&
        (m_lastAnchorUpdateMs == 0 || t - m_lastAnchorUpdateMs >= kBaselineDecimMs)) {
        m_lastAnchorUpdateMs = t;
        float v;
        const bool zenithNormalized = s.mode == 0 && CloudAirmass::Normalizes(s.extinctionMagPerAirmass, s.airmass);
        const float extinctionFall = zenithNormalized && m_anchorAirmass >= 1.f
            ? (float) std::pow(10., -.4 * CONFIG_CLOUD_ANCHOR_EXTINCTION_TOLERANCE * std::fabs(s.airmass - m_anchorAirmass))
            : 1.f;
        m_anchorAirmass = zenithNormalized ? s.airmass : -1.f;
        // A standard that never falls must not rise to the noise envelope (review after Ticket 005 #7): following
        // the 3-frame median at once ratcheted it up on a noisy star, so a clear sky with 4% frame noise read 0.86
        // after 150 minutes. Normalized standards rise only to the clear baseline ring's median, about two
        // minutes of Warmup/Clear frames; the haze reading still follows the current level at once. They rise
        // only in Clear: the ring is frozen otherwise, and its pre-haze median would undo the extinction-error
        // fall during a long haze.
        // Stellar frames without a known air mass keep the time-based slide but rise the same way, only to the
        // clear ring's median in Clear: rising to every 3-frame median ratcheted them up there too (mean 0.95,
        // 0.90 after 150 minutes at 4% frame noise). Other detection modes keep the original rule.
        float baseline = 0.f;
        const bool risesFromBaseline = m_state == SceneState::Clear;
        const bool stellarSlide = s.mode == 0 && !zenithNormalized;
        if (m_refBright.lastMedian(kShortK, v)) {
            if (stellarSlide && m_anchorBright.valid()) {
                m_anchorBright.slideToward(v, t);
                if (risesFromBaseline && m_baseBright.median(baseline)) m_anchorBright.update(baseline, t, false);
            }
            else if (!zenithNormalized) m_anchorBright.update(v, t, true);
            else {
                if (risesFromBaseline && m_baseBright.median(baseline)) m_anchorBright.update(baseline, t, false);
                m_anchorBright.fallToward(v, extinctionFall);
            }
        }
        if (m_refMass.lastMedian(kShortK, v)) {
            if (stellarSlide && m_anchorMass.valid()) {
                m_anchorMass.slideToward(v, t);
                if (risesFromBaseline && m_baseMass.median(baseline)) m_anchorMass.update(baseline, t, false);
            }
            else if (!zenithNormalized) m_anchorMass.update(v, t, true);
            else {
                if (risesFromBaseline && m_baseMass.median(baseline)) m_anchorMass.update(baseline, t, false);
                m_anchorMass.fallToward(v, extinctionFall);
            }
        }
        // The multi-star clear standard follows the mass standard's rule: seeded from the clear ring once it
        // holds kMinBaseEntries readings, rising only to that ring's median in Clear, and with a known air
        // mass falling only by the extinction allowance. Without air mass it does not fall at all: the per-star
        // references it corrects never slid, and a slide would let a steady multi-star haze become the clear
        // level and release a latched decline (finding #3 again).
        if (validEnsemble && m_shortEnsemble.lastMedian(kShortK, v)) {
            if (!m_anchorEnsemble.valid()) {
                if (m_state == SceneState::Clear && m_baseEnsemble.n >= kMinBaseEntries && m_baseEnsemble.median(baseline))
                    m_anchorEnsemble.seed(baseline, t);
            }
            else {
                if (risesFromBaseline && m_baseEnsemble.median(baseline)) m_anchorEnsemble.update(baseline, t, false);
                if (zenithNormalized) m_anchorEnsemble.fallToward(v, extinctionFall);
            }
        }
        if (m_refSnr.lastMedian(kShortK, v)) {
            m_anchorSnr.update(v, t, true);
            m_snrSessionBest.update(v, t, false);
        }
        if (m_refScore.lastMedian(kShortK, v))    m_anchorScore.update(v, t, true);
        if (m_refFeatures.lastMedian(kShortK, v)) m_anchorFeatures.update(v, t, true);
    }
    m_prevSampleMs = t;
    m_prevClearEligible = clearEligible;

    // ---- telemetry (recent scene vs the protected clear-sky standard) ----
    float shortBright = 0.f, shortMass = 0.f, shortFeatures = 0.f;
    m_tele.massRatio = detChannelsLive && m_shortMass.lastMedian(kShortK, shortMass)
                           ? m_anchorMass.ratio(shortMass) : -1.f;
    m_tele.brightRatio = m_shortBright.lastMedian(kShortK, shortBright)
                             ? m_anchorBright.ratio(shortBright) : -1.f;
    m_tele.featureRatio = detChannelsLive && m_shortFeatures.lastMedian(kShortK, shortFeatures)
                              ? m_anchorFeatures.ratio(shortFeatures) : -1.f;
    float shortEnsemble = 0.f;
    m_tele.ensembleRatioRaw = validEnsemble && m_shortEnsemble.lastMedian(kShortK, shortEnsemble)
                                  ? shortEnsemble : -1.f;
    // Relative to the multi-star clear standard, at most 1: a reading above it is no loss. Before the standard
    // exists the raw ratio stands in, as before it was introduced.
    auto ensembleLevel = [this](float raw) {
        return raw >= 0.f && m_anchorEnsemble.valid() && m_anchorEnsemble.value > kEps
            ? std::min(1.f, raw / m_anchorEnsemble.value) : raw;
    };
    m_tele.ensembleRatio = ensembleLevel(m_tele.ensembleRatioRaw);
    m_tele.ensembleStars = validEnsemble ? s.ensembleStars : 0;
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
            (!m_seenMass     || m_baseMass.n     >= kMinBaseEntries || primaryOut) &&
            (!m_seenSnr      || m_baseSnr.n      >= kMinBaseEntries) &&
            (!m_seenScore    || m_baseScore.n    >= kMinBaseEntries) &&
            (!m_seenFeatures || m_baseFeatures.n >= kMinBaseEntries);
        const bool stableNow = clearEligible && brightCeil >= 0.f &&
            (!m_seenMass || validMass) && (!m_seenSnr || validSnr) &&
            (!m_seenScore || validScore) && (!m_seenFeatures || validFeatures);
        if (m_clearAccumMs >= kWarmupMs && haveBaselines && stableNow) {
            transitionLocked(m_massDecline.latched ? SceneState::Suspect : SceneState::Clear,
                             "baseline established", t);
            // Seed every protected standard from the reference we just certified as clear.
            float v;
            if (m_baseBright.median(v))   m_anchorBright.seed(v, t);
            if (m_seenMass && m_baseMass.median(v))         m_anchorMass.seed(v, t);
            if (m_seenSnr && m_baseSnr.median(v)) {
                m_anchorSnr.seed(v, t);
                m_snrSessionBest.seed(v, t);
            }
            if (m_seenScore && m_baseScore.median(v))       m_anchorScore.seed(v, t);
            if (m_seenFeatures && m_baseFeatures.median(v)) m_anchorFeatures.seed(v, t);
            m_lastAnchorUpdateMs = t;
            m_anchorAirmass = s.mode == 0 && CloudAirmass::Normalizes(s.extinctionMagPerAirmass, s.airmass) ? s.airmass : -1.f;
        }
        m_tele.state = m_state;
        m_tele.severity = m_massDecline.latched && m_massDecline.ratio >= 0.f ?
            std::max(0.f, std::min(1.f, 1.f - m_massDecline.ratio)) : 0.f;
        if (m_massDecline.latched && m_massDecline.ratio < 0.f) m_tele.fresh = false;
        m_tele.stateSinceMs = m_stateSinceMs;
        return;
    }

    // Missing channels never vote; target loss alone is not cloud evidence.
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
    const float ensembleTripRatio = std::isfinite(s.ensembleTripRatio)
                                        ? std::max(0.2f, std::min(0.98f, s.ensembleTripRatio)) : 0.78f;
    const bool ensembleMed = m_tele.ensembleRatio >= 0.f && m_tele.ensembleRatio < ensembleTripRatio;
    const int  votes      = (massMed ? 1 : 0) + (brightMed ? 1 : 0) + (featureMed ? 1 : 0) +
                            (snrMed ? 1 : 0) + (scoreMed ? 1 : 0) +
                            (ensembleMed ? 1 : 0);
    // FWHM and image contrast can corroborate a photometric level loss. A single variable
    // channel plus those noisy supporting measurements warrants Suspect, not Obscured.
    // Ticket 005 #7: in stellar mode a lower or noisier S/N while the star's flux and the multi-star ratio
    // hold is a brighter or variable background (moonrise, twilight, stray light), not lost light. It is
    // then not evidence by itself, so it can neither raise Suspect nor certify an alternate baseline;
    // it still corroborates mass or multi-star evidence below.
    const bool stellar = s.mode == 0;
    // "Holds" uses the healthy-photometry bar of the decline release: within 5% of the clear standard. A
    // flux loss of 11%, under the 25% mass vote, still lets S/N stand as evidence, so that loss is reported.
    // The bar compares nine-frame medians: three frames of a noisy star fall below 95% often enough to let
    // an S/N drop count as evidence again. Until nine frames exist the three-frame ratio stands in.
    float steadyMass = 0.f, steadyEnsemble = 0.f;
    const float steadyMassRatio = detChannelsLive && m_shortMass.lastMedian(kVariabilityK, steadyMass)
                                      ? m_anchorMass.ratio(steadyMass) : m_tele.massRatio;
    const bool ensembleSteady = m_tele.ensembleRatio < 0.f ||
        (m_shortEnsemble.lastMedian(kVariabilityK, steadyEnsemble) ? ensembleLevel(steadyEnsemble) : m_tele.ensembleRatio) >= .95f;
    const bool photometryHolds = m_seenMass && steadyMassRatio >= .95f && !massMed && ensembleSteady;
    // A held-out primary says nothing about the flux, so an S/N drop is not lost light by itself then either.
    const bool snrAlone = stellar && (photometryHolds || primaryOut);
    const bool primaryEvidence = massMed || (snrMed && !snrAlone) || featureMed || ensembleMed;
    const bool levelEvidence = massLevelMed || snrLevelMed || featureMed || ensembleMed;
    const int photometricVotes = (massMed ? 1 : 0) + (snrMed ? 1 : 0) +
                                 (featureMed ? 1 : 0) + (ensembleMed ? 1 : 0);
    const bool corroborated = photometricVotes >= 2 || (levelEvidence && votes >= 2);

    // A recovery quorum needs current primary measurements, not simply an absence of bad
    // votes. Optional FWHM/contrast and a temporarily absent ensemble do not become mandatory.
    const bool freshPrimary = s.detected && s.stableLock &&
        (m_seenMass || m_seenSnr || m_seenFeatures) &&
        (!m_seenMass || m_tele.massRatio >= 0.f || primaryOut) && (!m_seenSnr || haveSnr || m_snrRelearning) &&
        (!m_seenFeatures || m_tele.featureRatio >= 0.f);
    // A retained Clear label is not a current verdict while primary photometry is missing
    // or refilling. During target loss, fresh image contrast still describes physical blackout.
    m_tele.fresh = s.detected ? freshPrimary : m_tele.brightRatio >= 0.f;
    bool settled = freshPrimary, recoverySafe = freshPrimary, improving = false, rising = false;
    if (m_seenMass && !primaryOut) {
        const bool quiet = m_trendMass.settled(t, CONFIG_CLOUD_TREND_MASS_FRAC, 0.f, clearMassMad, rising);
        settled = settled && quiet; recoverySafe = recoverySafe && (quiet || rising); improving = improving || rising;
    }
    if (m_seenSnr) {
        bool snrReady = false;
        const bool quiet = m_trendSnr.settled(t, 0.f, CONFIG_CLOUD_TREND_SNR_DB, clearSnrMad, rising, &snrReady);
        // S/N restarted at an exposure change neither blocks nor certifies recovery until it has refilled.
        if (m_snrRelearning && snrReady && m_anchorSnr.valid())
            m_snrRelearning = false;
        if (!m_snrRelearning) {
            settled = settled && quiet; recoverySafe = recoverySafe && (quiet || rising); improving = improving || rising;
        }
    }
    // Feature-only modes need a plateau too, without making terrain changes constrain stellar S/N.
    if (!m_seenMass && !m_seenSnr && m_seenFeatures) {
        float med = 0.f, mad = 0.f;
        m_baseFeatures.medianMad(med, mad);
        const bool quiet = m_trendFeatures.settled(t, CONFIG_CLOUD_TREND_MASS_FRAC, 0.f, mad, rising);
        settled = settled && quiet; recoverySafe = recoverySafe && (quiet || rising); improving = improving || rising;
    }
    if (validEnsemble) {
        bool ready = false;
        const bool quiet = m_trendEnsemble.settled(t, CONFIG_CLOUD_TREND_MASS_FRAC, 0.f, 0.f, rising, &ready);
        // An optional star ensemble can veto a measured trend, but intermittent availability
        // must not make its missing history a permanent requirement for primary-star recovery.
        if (ready) {
            settled = settled && quiet;
            recoverySafe = recoverySafe && (quiet || rising);
        }
        improving = improving || rising;
    }
    m_tele.recoverySettled = settled;
    m_tele.clearingTrend = improving;

    // ---- slow (gradual haze) vote, measured against the anchors rather than the baseline ----
    // Requires the detection-INDEPENDENT brightness channel plus one photometric channel. That
    // pairing is what keeps legitimate target evolution out of it: a partial eclipse fades mass
    // and feature count by geometry while peak surface brightness holds, so geometry alone can
    // never satisfy this vote.
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
    const bool ensembleNotRecovered = m_tele.ensembleRatio >= 0.f &&
                                      m_tele.ensembleRatio < std::min(0.99f, ensembleTripRatio + 0.08f);
    const int recoveryBadChannels = (massNotRecovered ? 1 : 0) + (featureNotRecovered ? 1 : 0) +
                                    (snrNotRecovered ? 1 : 0) + (scoreNotRecovered ? 1 : 0) +
                                    (ensembleNotRecovered ? 1 : 0);

    const float obscuredHaze = CONFIG_CLOUD_OBSCURED_HAZE_PERCENT / 100.f;
    auto loss = [](float ratio) {
        return ratio >= 0.f ? std::max(0.f, std::min(1.f, 1.f - ratio)) : 0.f;
    };
    float certifiedMass = 0.f, currentMass = 0.f, currentEnsemble = 0.f;
    const bool healthyPrimary = freshPrimary && !primaryEvidence && !slowVote &&
        !massNotRecovered && !snrNotRecovered &&
        m_baseMass.median(certifiedMass) && certifiedMass > 0.f &&
        m_refMass.median(currentMass) && currentMass >= certifiedMass * .95f &&
        (!m_massDecline.usesEnsemble ||
         (validEnsemble && m_shortEnsemble.median(currentEnsemble) && ensembleLevel(currentEnsemble) >= .95f));
    if (m_massDecline.releaseIfHealthy(healthyPrimary, CONFIG_CLOUD_DECLINE_HEALTHY_RECOVERY_MS)) {
        logLocked("cloud: decline reference released after sustained healthy photometry");
        m_tele.massDeclineLatched = false;
        m_tele.massDeclineRatio = m_tele.massDeclineRate = -1.f;
    }
    const bool declineEvidence = m_massDecline.latched && !m_massDecline.recoveryAllowed();
    const float massLoss = loss(m_tele.ensembleRatio >= 0.f ? m_tele.ensembleRatio : m_tele.massRatio);
    const float declineLoss = m_massDecline.latched ? loss(m_massDecline.ratio) : 0.f;
    const float sceneLoss = !s.detected ? loss(m_tele.brightRatio) :
        (!m_seenMass && slowVote ? std::min(loss(m_tele.featureRatio), loss(m_tele.brightRatio)) : 0.f);
    const float attenuation = std::max({ massLoss, declineLoss, sceneLoss });
    // Variability remains a haze warning, but cannot imply total signal loss.
    auto scatterSeverity = [](float factor) {
        return factor > 1.f ? std::min(0.5f, (factor - 1.f) / 4.f) : 0.f;
    };
    float qualitySeverity = std::max(scatterSeverity(m_tele.massScatterFactor),
                                     scatterSeverity(m_tele.snrScatterFactor));
    const float backgroundSeverity = snrLevelMed
        ? std::min(0.5f, (m_tele.snrDropDb - m_medSnrDropDb) / 6.f + 0.3f) : 0.f;
    qualitySeverity = std::max(qualitySeverity, backgroundSeverity);
    const bool evidence = primaryEvidence || slowVote || declineEvidence || sceneLoss > 0.f;
    // Stellar haze is lost light only (Ticket 005 #7): Cloud Guard adds it up over each frame, so the
    // quality warning is published separately instead of standing in for attenuation.
    m_tele.severity = evidence ? (stellar ? attenuation : std::max(attenuation, qualitySeverity)) : 0.f;
    // The background part of the warning also compares S/N with the segment's best, at half the S/N-drop
    // vote threshold: a moonrise of about 3 dB over half an hour is followed by m_anchorSnr's slide and
    // would otherwise never show. It informs; it is not evidence and does not enter haze.
    const float sessionSnrDrop = haveSnr && m_snrSessionBest.valid() ? -m_snrSessionBest.delta(shortSnr) : 0.f;
    const float sessionBackground = sessionSnrDrop > .5f * m_medSnrDropDb
        ? std::min(0.5f, (sessionSnrDrop - .5f * m_medSnrDropDb) / 6.f + 0.3f) : 0.f;
    const float warning = std::max(qualitySeverity, sessionBackground);
    m_tele.qualityWarning = stellar ? warning : -1.f;
    m_tele.qualityWarningSnrDrop = stellar && warning > 0.f &&
        std::max(backgroundSeverity, sessionBackground) >= std::max(scatterSeverity(m_tele.massScatterFactor),
                                                                    scatterSeverity(m_tele.snrScatterFactor));
    const bool obscured = evidence && attenuation >= obscuredHaze &&
                          (s.detected ? freshPrimary : m_tele.brightRatio >= 0.f);
    if (m_massDecline.latched && m_massDecline.ratio < 0.f)
        m_tele.fresh = false;

    float alternateMassMed = 0.f, alternateMassMad = 0.f;
    float alternateBrightMed = 0.f, alternateBrightMad = 0.f;
    float alternateSnrMed = 0.f, alternateSnrMad = 0.f;
    float alternateEnsembleMed = 0.f, alternateEnsembleMad = 0.f;
    const bool stableMass = !m_seenMass ||
        (detChannelsLive && m_shortMass.lastMedianMad(kVariabilityK, alternateMassMed, alternateMassMad) &&
         alternateMassMed > kEps && alternateMassMad / alternateMassMed <= kAlternateRelMad);
    const bool stableBright =
        m_shortBright.lastMedianMad(kVariabilityK, alternateBrightMed, alternateBrightMad) &&
        alternateBrightMed > kEps && alternateBrightMad / alternateBrightMed <= kAlternateRelMad;
    const bool stableSnr = !m_seenSnr ||
        (detChannelsLive && m_shortSnr.lastMedianMad(kVariabilityK, alternateSnrMed, alternateSnrMad) &&
         alternateSnrMed >= kAlternateMinSnrDb && alternateSnrMad <= kAlternateSnrMadDb);
    const bool stableEnsemble = !validEnsemble ||
        (m_shortEnsemble.lastMedianMad(kVariabilityK, alternateEnsembleMed, alternateEnsembleMad) &&
         alternateEnsembleMed > kEps && alternateEnsembleMad / alternateEnsembleMed <= kAlternateRelMad);
    const bool stableAlternative = settled && clearEligible && brightCeil >= 0.f &&
                                   !massVariable && !snrVariable && stableMass && stableBright &&
                                   stableSnr && stableEnsemble;
    auto alternateCertified = [&]() {
        if (!stableAlternative || m_massDecline.latched || obscured) {
            m_alternateSinceMs = 0;
            return false;
        }
        if (m_alternateSinceMs == 0)
            m_alternateSinceMs = t;
        else if (t - m_alternateSinceMs >= kAlternateBaselineMs) {
            logLocked("cloud: stable alternate baseline certified");
            resetLocked("stable alternate baseline");
            return true;
        }
        return false;
    };

    switch (m_state) {
    case SceneState::Clear:
    case SceneState::Suspect: {
        if (obscured) {
            transitionLocked(SceneState::Obscured, "near-total measured signal loss", t);
            break;
        }
        if (primaryEvidence || slowVote || declineEvidence || sceneLoss > 0.f) {
            m_suspectQuietSinceMs = 0;
            if (m_state == SceneState::Clear)
                transitionLocked(SceneState::Suspect,
                    declineEvidence ? "sustained mass decline accumulating haze" : "measured degradation", t);
            if (primaryEvidence && !corroborated && !slowVote && !declineEvidence) {
                if (alternateCertified()) return;
            } else {
                m_alternateSinceMs = 0;
            }
        }
        else if (m_state == SceneState::Suspect) {
            const bool ready = m_obscuredRecovery ? settled && recoveryBadChannels < 2 : recoverySafe;
            if (!ready || !m_massDecline.recoveryAllowed()) {
                m_suspectQuietSinceMs = 0;
                if (alternateCertified()) return;
                break;
            }
            if (recoveryHoldCompleteLocked(t, m_suspectQuietSinceMs, kSuspectQuietMs)) {
                m_massDecline.recovered();
                m_tele.massDeclineLatched = false;
                m_tele.massDeclineRatio = -1.f;
                transitionLocked(SceneState::Clear, "channels recovered", t);
            } else if (alternateCertified()) {
                return;
            }
        }
        break;
    }

    case SceneState::Obscured: {
        if (freshPrimary && m_tele.fresh && attenuation < obscuredHaze * .9f) {
            transitionLocked(SceneState::Suspect, "usable signal restored; qualifying clear view", t);
            break;
        }
        // Static-obstruction escalation: long obscuration with a flat (non-cloud-like)
        // brightness signature -- a branch / dew never fluctuates the way moving cloud does.
        if (!m_tele.staticObstruction && t - m_stateSinceMs >= kStuckMs) {
            float med = 0.f, mad = 0.f;
            if (brightCeil >= 0.f && m_shortBright.medianMad(med, mad) && med > kEps && mad / med < kStaticMadFrac) {
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
            "cloud: %s sev=%.2f mass=%.2f massVar=%.2f bright=%.2f ensemble=%.2f "
            "snrDrop=%.1fdB snrVar=%.2f score%+.2f feat=%.2f "
            "slow[bright=%.2f mass=%.2f feat=%.2f] loss=%d recBad=%d slowVote=%d settled=%d clearing=%d",
            StateName(m_state), m_tele.severity, m_tele.massRatio, m_tele.massScatterFactor,
            m_tele.brightRatio, m_tele.ensembleRatio, m_tele.snrDropDb, m_tele.snrScatterFactor,
            m_tele.scoreDelta, m_tele.featureRatio,
            m_tele.slowBrightRatio, m_tele.slowMassRatio, m_tele.slowFeatureRatio, m_lossRun,
            recoveryBadChannels, slowVote ? 1 : 0, settled ? 1 : 0, improving ? 1 : 0);
        logLocked(buf);
    }
    else if (m_state == SceneState::Clear) {
        m_lastPeriodicLogMs = 0;
    }
}

void CloudDetector::logReplayLocked(const char* event, const SceneSample* s) noexcept
{
    ++m_replaySequence;
    if (!m_log)
        return;
    try {
        std::ostringstream out;
        out.imbue(std::locale::classic());
        out << std::setprecision(std::numeric_limits<float>::max_digits10)
            << "cloud: replay v=1 seq=" << m_replaySequence << " event=" << event
            << " enabled=" << m_enabled << " sensitivity=" << m_sensitivityPct
            << " referenceGeneration=" << m_referenceGeneration;
        if (s) {
            out << " tMs=" << s->tMs << " detected=" << s->detected << " stableLock=" << s->stableLock
                << " score=" << s->score << " snr=" << s->snr << " mass=" << s->mass
                << " features=" << s->features << " ensembleRatio=" << s->ensembleRatio
                << " ensembleStars=" << s->ensembleStars << " ensembleTripRatio=" << s->ensembleTripRatio
                << " massDeclinePctPerMinute=" << CONFIG_CLOUD_MASS_DECLINE_PCT_PER_MINUTE
                << " cloudConfigGeneration=" << s->cloudConfigGeneration
                << " brightCeil=" << s->brightCeil << " brightExposureMs=" << s->brightExposureMs
                << " airmass=" << s->airmass << " extinction=" << s->extinctionMagPerAirmass
                << " exposureMs=" << s->exposureMs << " gain=" << s->gain << " bitDepth=" << s->bitDepth
                << " frameW=" << s->frameW << " frameH=" << s->frameH
                << " roiX=" << s->roiX << " roiY=" << s->roiY << " roiW=" << s->roiW << " roiH=" << s->roiH
                << " sourceGen=" << s->sourceGen << " mode=" << s->mode << " saturated=" << s->primarySaturated
                << " ensembleConfigured=" << s->ensembleConfigured;
        }
        logLocked(out.str().c_str());
    }
    catch (...) {
        m_log = nullptr;
        noteExceptionLocked(true);
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
