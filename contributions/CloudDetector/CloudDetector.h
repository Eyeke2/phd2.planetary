/* CloudDetector.h
 *
 * Scene-quality / cloud detection, adapted from the HM SceneQualityMonitor reference design.
 *
 * Detects clouds, haze, tree branches, dew -- anything degrading the view -- from the
 * per-frame telemetry the active PHD2 detector already produces, plus a detection-independent
 * image high-tail contrast. Publishes a tri-state (CLEAR / SUSPECT / OBSCURED) and severity for
 * PHD2's informational guide-window HUD.
 *
 * SCOPE: the public PHD2 integration supports ordinary guide-star tracking only. Planetary cloud
 * detection belongs to HM and is deliberately excluded by Guider::IsCloudDetectionActive().
 *
 * WHAT THIS DOES NOT DO: target loss or guide-frame rejection. A miss run means the detector
 * lost its target; existing PHD2 star-loss, mass-change and SNR logic continues to own that
 * decision. lossRun is telemetry only.
 *
 * Wiring: fed once per live star frame by GuiderMultiStar and consumed by the Guider paint path.
 * Deliberately dependency-light (standard C++ plus CloudDetectorConfig.h and
 * an injectable logger) so it compiles and unit-tests standalone with explicit timestamps.
 * Thresholds are shadow-mode starting points: tests pin the state machine and hysteresis, not
 * whether the values suit every real cloudy session.
 *
 * Copyright (c) 2026 Leo Shatz
 * All rights reserved.
 */
#pragma once

#include <cstdint>
#include <functional>
#include <mutex>
#include <string>

// One sample per live detection frame. Reprocessing the same frame for UI interaction must not
// feed a second sample because that would double-count it.
struct SceneSample {
    int64_t tMs = 0;           // ObjTracker::FrameNowMs() -- sim-aware, so replays tune offline
    bool    detected = false;
    // Whether tracking is stable enough for this sample to update the clear reference.
    bool    stableLock = false;
    float   score = 0.f;       // Guide-star quality: negative FWHM
    float   snr = 0.f;         // dB (callers must convert linear S/N before feeding)
    float   mass = 0.f;        // Integrated flux
    int     features = 0;      // Reserved; 0 in the public star-only integration
    // Normalized multi-star evidence; 1 is nominal and negative is unavailable.
    float   ensembleRatio = -1.f;
    int     ensembleStars = 0;
    float   ensembleTripRatio = 0.78f;
    // Image P99.5-minus-median contrast -- valid even when NOT detected, which is what makes it the
    // channel that still measures the sky during a total occultation.
    // NEGATIVE = unavailable (no stats this frame). ZERO IS A VALID READING: an all-black ROI is
    // exactly what a blackout looks like, and discarding it leaves the brightness window holding
    // the last clear samples with nothing able to trip. Default is -1, not 0, so a sample that
    // never sets this field reads as "no data" rather than as a blackout.
    float   brightCeil = -1.f;
    // When positive, brightCeil is normalized to a one-second exposure before entering the
    // detector windows. This is deliberately separate from exposureMs: auto exposure must not
    // reset the baseline on every adjustment, but its raw image contrast still needs scaling.
    int     brightExposureMs = 0;

    // ---- acquisition identity ----
    // Any change here means the frames after it come from a DIFFERENT image regime, so every
    // baseline, anchor, window and dwell timer built from the frames before it is meaningless.
    // The monitor compares the whole tuple and resets on any change: a discontinuity must never
    // be classified as weather. Unknown/unavailable fields (-1) never trigger a reset, so a
    // value that cannot be read is simply not a discontinuity source.
    int     exposureMs = 0;    // current guide exposure (<= 0 = unknown)
    int     gain = -1;         // camera gain; -1 = unreadable (0 is a VALID gain)
    int     bitDepth = -1;     // declared camera/output depth; -1 = unknown
    int     frameW = -1;       // frame geometry as processed by the detector
    int     frameH = -1;
    int     roiX = -1, roiY = -1, roiW = -1, roiH = -1;   // effective detection ROI
    unsigned sourceGen = 0;    // camera/source generation: tracker re-init, source change
    int     mode = 0;          // DetectionMode as int (avoid a TrackerUI.h dependency)
};

enum class SceneState {
    Warmup,     // baseline not yet trustworthy -- no verdicts (IsClear() == true)
    Clear,
    Suspect,    // a primary target channel degraded -- baseline FROZEN, no action yet
    Obscured,
};

// Robust signal above the local background, in the input image's original ADU scale.
// The percentile rejects isolated hot pixels; adaptive histogram quantization supports both
// unscaled 8-bit samples in a ushort buffer and values expanded/integrated into 16-bit range.
float CloudHighTailContrast(const unsigned short *imageData, int frameWidth, int frameHeight,
                            int x, int y, int width, int height) noexcept;

// HUD / logging snapshot. Ratios are shortMedian / protected clear standard (1.0 = nominal); negative
// ratio means the channel has no data yet (undetected / warm-up).
struct SceneTelemetry {
    SceneState state = SceneState::Warmup;
    // False means an exception escaped the detector's calculation path and was contained. The
    // state is reset to Warmup in that case so stale Clear/Obscured evidence is never published.
    bool healthy = true;
    // Lifetime counters survive state resets and make contained failures visible to RPC clients.
    unsigned exceptionCount = 0;
    unsigned loggerExceptionCount = 0;
    float severity = 0.f;        // 0..1, max normalized excess over the trip thresholds
    float massRatio = -1.f;
    float brightRatio = -1.f;
    float snrDropDb = 0.f;       // protected clear standard - current (positive = degraded)
    float scoreDelta = 0.f;      // current - protected clear standard; negative = degraded
    float featureRatio = -1.f;
    float ensembleRatio = -1.f;
    int   ensembleStars = 0;
    // Recent robust window scatter divided by the learned clear-scatter trip band. Values above 1
    // mean the normally flat mass/SNR trace has developed significant waves. -1 = unavailable.
    float massScatterFactor = -1.f;
    float snrScatterFactor = -1.f;
    // Slow-haze ratios against the non-poisoning clear anchor (see CONFIG_CLOUD_SLOW_RATIO).
    // These are what catch a gradual fade; the rolling-baseline ratios above cannot, by
    // construction, because the baseline adapts to the fade. -1 = no anchor yet.
    float slowBrightRatio = -1.f;
    float slowMassRatio = -1.f;
    float slowFeatureRatio = -1.f;
    // Consecutive undetected frames. TELEMETRY ONLY: target loss belongs to PHD2's existing
    // lost-star behavior, which already owns it. The monitor never turns a miss run
    // into a scene verdict -- "the target is gone" and "the sky went bad" are different claims
    // and a detector that conflates them re-reports one failure as two.
    int   lossRun = 0;
    bool  staticObstruction = false;  // OBSCURED long + static signature (branch/dew, not cloud)
    int64_t stateSinceMs = 0;
};

class CloudDetector {
public:
    CloudDetector();

    // ---- configuration (GUI thread) ----
    // 0..100; 50 = the documented defaults. Higher = trips earlier (thresholds closer to
    // baseline, shorter medium window). Maps to the future Advanced Sequence Options slider.
    void SetSensitivityPct(int pct) noexcept;
    // Disabled: Feed() still tracks acquisition identity for a clean later enable, but no state
    // machine runs. BOTH edges reset, so disabling while Obscured immediately publishes warm-up
    // and IsClear() == true: stopping the state machine while a stale not-clear verdict stays
    // latched would leave a stale verdict published by a monitor that is no longer running.
    void SetEnabled(bool on) noexcept;
    // Shadow-mode logger (state transitions + a periodic line while degraded). Nullable.
    void SetLogger(std::function<void(const std::string&)> logger) noexcept;

    // ---- data path (any thread; internally locked) ----
    void Feed(const SceneSample& s) noexcept;
    // Hard reset (session start, source change...). Exposure / detection-mode changes are
    // detected inside Feed and reset automatically.
    void Reset(const char* reason) noexcept;
    // Resume after dither/settle without discarding learned clear-sky references.
    void ResumeAfterMotion(const char* reason) noexcept;
    // Integration code can route an exception raised while assembling a sample through the same
    // fail-open health path as an exception raised by the detector itself.
    void ReportFault(const char* operation, const char* detail) noexcept;

    // ---- consumers (any thread) ----
    // True in Warmup/Clear/Suspect; only a latched OBSCURED reports not-clear.
    bool IsClear() const noexcept;
    SceneState GetState() const noexcept;
    SceneTelemetry GetTelemetry() const noexcept;

private:
    // Fixed-capacity ring with on-demand robust stats (tiny N -- copy+sort is fine).
    struct Ring {
        static constexpr int kCap = 64;
        float buf[kCap];
        int   n = 0, head = 0;
        void  clear() { n = 0; head = 0; }
        void  push(float v);
        bool  median(float& out) const;          // false when empty
        bool  medianMad(float& med, float& mad) const;
        bool  lastMedian(int k, float& out) const;   // median of the k newest entries
        bool  lastMedianMad(int k, float& med, float& mad) const;
    };

    // A non-poisoning "what did clear look like" standard for ONE channel. Higher is always
    // better. It follows sustained-window improvements promptly, but slews down only at the
    // bounded rate in CONFIG_CLOUD_ANCHOR_DOWN_PCT_MIN. Explicit validity permits signed quality
    // scores such as negative FWHM.
    struct Anchor {
        float value = 0.f;
        int64_t tMs = 0;         // when `value` was last updated (rate limiting is per elapsed time)
        bool established = false;
        void clear() { value = 0.f; tMs = 0; established = false; }
        bool valid() const { return established; }
        void seed(float v, int64_t t) { value = v; tMs = t; established = true; }
        void update(float v, int64_t t);
        // sample / anchor, or -1 when the anchor is not established.
        float ratio(float v) const { return valid() && value > 1e-6f ? (v / value) : -1.f; }
        float delta(float v) const { return valid() ? (v - value) : 0.f; }
    };

    void  resetLocked(const char* reason);
    void  resumeAfterMotionLocked(const char* reason) noexcept;
    void  clearStateLocked() noexcept;
    void  containException(const char* operation, const char* detail) noexcept;
    void  noteExceptionLocked(bool loggerFailure) noexcept;
    void  feedLocked(const SceneSample& sample);
    void  transitionLocked(SceneState next, const char* why, int64_t tMs);
    void  logLocked(const char* msg) noexcept;
    void  applySensitivityLocked();

    mutable std::mutex m_mx;
    std::function<void(const std::string&)> m_log;
    unsigned m_exceptionCount = 0;
    unsigned m_loggerExceptionCount = 0;

    bool m_enabled = true;
    int  m_sensitivityPct = 50;
    // Sensitivity-scaled working thresholds (recomputed in applySensitivityLocked).
    float m_fastMassRatio, m_fastBrightRatio;
    float m_medMassRatio, m_medSnrDropDb, m_medFeatureRatio, m_medBrightRatio;
    // Recovery thresholds DERIVED from the trip thresholds above, so hysteresis points the right
    // way at every sensitivity (see CONFIG_CLOUD_RECOVER_MARGIN).
    float m_recMassRatio, m_recFeatureRatio, m_recSnrDropDb;
    float m_slowRatio;
    int   m_mediumWindowMs;

    // Baselines: decimated (one entry / kBaselineDecimMs) rings of CLEAR-state samples.
    Ring m_baseMass, m_baseBright, m_baseSnr, m_baseScore, m_baseFeatures;
    int64_t m_lastBaselinePushMs = 0;
    int64_t m_clearAccumMs = 0;        // CLEAR-eligible data accumulated since reset (warm-up)
    int64_t m_prevSampleMs = 0;

    // Short (fast) windows: newest raw samples; detection channels only accept detected frames.
    // These measure the sky AS IT IS and must stay unconditional -- they drive the trips, and a
    // blackout (undetected by definition) has to be measurable through them.
    Ring m_shortMass, m_shortBright, m_shortSnr, m_shortScore, m_shortFeatures;
    Ring m_shortEnsemble;

    // Reference windows: the same channels, but accepting ONLY clear-eligible frames (detected,
    // stable-locked, no loss run). Baseline entries and anchor updates are snapshotted from these.
    // Two separate sets are needed because the two jobs disagree: a trip window must include the
    // bad frames, a reference window must exclude them. Gating only the WRITE MOMENT against the
    // shared short rings is not enough -- the value written is a median, so an eligible frame
    // arriving among unstable ones still snapshotted a median made of the unstable ones.
    Ring m_refMass, m_refBright, m_refSnr, m_refScore, m_refFeatures;

    // Which channels this mode actually produces. A channel that has never carried data is not
    // "applicable" and must neither block the warm-up gate nor vote.
    bool m_seenMass = false, m_seenSnr = false, m_seenScore = false, m_seenFeatures = false;

    // Protected clear-sky standards. Improvements are adopted promptly; lower stable values are
    // admitted at a 30-60 minute scale even while degraded, so a permanent benign scene change
    // can eventually become the new normal without erasing the best view after a short reset.
    Anchor m_anchorBright, m_anchorMass, m_anchorSnr, m_anchorScore, m_anchorFeatures;
    int64_t m_lastAnchorUpdateMs = 0;
    int64_t m_slowTripSinceMs = 0;     // 0 = slow vote not currently held

    // State machine.
    SceneState m_state = SceneState::Warmup;
    int64_t m_stateSinceMs = 0;
    int64_t m_mediumTripSinceMs = 0;   // 0 = medium vote not currently held
    int64_t m_recoverSinceMs = 0;      // 0 = recovery condition not currently held
    int64_t m_suspectQuietSinceMs = 0; // 0 = a channel is still tripped while in Suspect
    int     m_lossRun = 0;
    bool    m_sawGoodDetection = false;
    bool    m_lossLogged = false;      // one "target lost" line per loss episode
    int64_t m_lastPeriodicLogMs = 0;

    // Acquisition identity of the previous sample (see SceneSample). Any change resets.
    struct Identity {
        int exposureMs = -1, gain = -1, bitDepth = -1, mode = -1;
        int frameW = -1, frameH = -1;
        int roiX = -1, roiY = -1, roiW = -1, roiH = -1;
        unsigned sourceGen = 0;
        bool valid = false;
    } m_ident;
    // Names the first field that differs, or nullptr when the tuple is unchanged. Unknown (-1)
    // fields on either side are skipped: "cannot be read" is not a discontinuity.
    static const char* IdentityDelta(const Identity& prev, const Identity& cur);
    static Identity IdentityOf(const SceneSample& s);

    SceneTelemetry m_tele;
};
