/* CloudDetector.h
 *
 * Scene-quality / cloud detection.
 *
 * Detects clouds, haze, tree branches, dew, anything degrading the view
 *
 * WHAT THIS DOES NOT DO: target loss or guide-frame rejection. A miss run means the detector
 * lost its target; existing PHD2 star-loss, mass-change and SNR logic continues to own that
 * decision. lossRun is telemetry only.
 *
 * Copyright (c) 2026 Leo Shatz
 * All rights reserved.
 */
#pragma once

#include "CloudAirmass.h"
#include "MassDeclineDetector.h"
#include "SkyStabilityDetector.h"
#include <algorithm>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>

struct SceneSample {
    int64_t tMs = 0;
    bool    detected = false;
    bool    stableLock = false;
    float   score = 0.f;
    float   snr = 0.f;
    float   mass = 0.f;
    int     features = 0;
    float   ensembleRatio = -1.f;
    int     ensembleStars = 0;
    float   ensembleTripRatio = 0.78f;
    bool    primarySaturated = false;
    bool    ensembleConfigured = false;
    unsigned cloudConfigGeneration = 0; // restart rate evidence after explicit configuration edits
    float   brightCeil = -1.f;
    int     brightExposureMs = 0;
    float   airmass = -1.f;
    float   extinctionMagPerAirmass = 0.f;
    int     exposureMs = 0;    // fixed exposure; 0 with brightExposureMs > 0 means automatic
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
    Suspect,    // partial degradation; consumers apply their haze threshold
    Obscured,   // near-total measured signal loss
};

float CloudHighTailContrast(const unsigned short *imageData, int frameWidth, int frameHeight,
                            int x, int y, int width, int height) noexcept;

struct SceneTelemetry {
    SceneState state = SceneState::Warmup;
    bool healthy = true;
    unsigned exceptionCount = 0;
    unsigned loggerExceptionCount = 0;
    float severity = 0.f;
    float qualityWarning = -1.f;
    bool qualityWarningSnrDrop = false;
    float massRatio = -1.f;
    float brightRatio = -1.f;
    float snrDropDb = 0.f;
    float scoreDelta = 0.f;
    float featureRatio = -1.f;
    float ensembleRatio = -1.f;
    float ensembleRatioRaw = -1.f;
    int   ensembleStars = 0;
    float primarySaturated = -1.f;
    bool  primaryHeldOut = false;
    float massDeclineRate = -1.f;
    float massDeclineRatio = -1.f;
    bool massDeclineLatched = false;
    bool massDeclineUsesEnsemble = false;
    float massScatterFactor = -1.f;
    float snrScatterFactor = -1.f;
    float slowBrightRatio = -1.f;
    float slowMassRatio = -1.f;
    float slowFeatureRatio = -1.f;
    int   lossRun = 0;
    bool  staticObstruction = false;
    int64_t stateSinceMs = 0;
    float airmass = -1.f;
    bool skyStabilityApplicable = false;
    bool skyStabilityReady = false, skyImproving = false, skyStable = false;
    float skyRiseLoss = -1.f;
    bool recoverySettled = false;
    bool clearingTrend = false;
    bool fresh = false;
    unsigned referenceGeneration = 0;
    int faultRecoverySamples = 0;
    int faultAutoRetries = 0;
    int faultRetryLimit = 0;
};

class CloudDetector {
public:
    CloudDetector();

    void SetSensitivityPct(int pct) noexcept;
    void SetEnabled(bool on) noexcept;
    void SetLogger(std::function<void(const std::string&)> logger) noexcept;

    void Feed(const SceneSample& s) noexcept;
    void Reset(const char* reason) noexcept;
    void StartNewSegment(const char* reason) noexcept;
    void ResumeAfterMotion(const char* reason) noexcept;
    void ReportFault(const char* operation, const char* detail) noexcept;

    bool IsClear() const noexcept;
    SceneState GetState() const noexcept;
    SceneTelemetry GetTelemetry(int64_t nowMs = 0) const noexcept;

private:
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

    struct Trend {
        static constexpr int kCap = 64;
        float values[kCap];
        int64_t times[kCap];
        int n = 0, head = 0;
        void clear() { n = 0; head = 0; }
        void push(float value, int64_t t);
        bool settled(int64_t t, float fractionFloor, float absoluteFloor, float clearMad,
                     bool& improving, bool* ready = nullptr) const;
    };

    struct Anchor {
        float value = 0.f;
        int64_t tMs = 0;
        bool established = false;
        void clear() { value = 0.f; tMs = 0; established = false; }
        bool valid() const { return established; }
        void seed(float v, int64_t t) { value = v; tMs = t; established = true; }
        void update(float v, int64_t t, bool mayFall);
        void slideToward(float v, int64_t t);
        void fallToward(float v, float floorFactor) { if (valid() && v < value) value = std::max(v, value * floorFactor); }
        float ratio(float v) const { return valid() && value > 1e-6f ? (v / value) : -1.f; }
        float delta(float v) const { return valid() ? (v - value) : 0.f; }
    };

    void  resetLocked(const char* reason, bool resetFaultRetries = true, bool preserveDecline = false);
    void  resumeAfterMotionLocked(const char* reason) noexcept;
    void  beginExposureJoinLocked(int fromMs, int toMs, int64_t t);
    float finishExposureJoinLocked(float current);
    float finishDeferredJoinLocked(float current, float level);
    void  clearStateLocked(bool preserveDecline = false) noexcept;
    void  containException(const char* operation, const char* detail) noexcept;
    void  noteExceptionLocked(bool loggerFailure) noexcept;
    void  feedLocked(const SceneSample& recorded);
    void  transitionLocked(SceneState next, const char* why, int64_t tMs);
    void  logLocked(const char* msg) noexcept;
    void  logReplayLocked(const char* event, const SceneSample* sample = nullptr) noexcept;
    void  applySensitivityLocked();
    int64_t sampleGapLimitLocked(int currentExposureMs = 0) const;
    bool recoveryHoldCompleteLocked(int64_t t, int64_t& sinceMs, int holdMs);

    mutable std::mutex m_mx;
    std::function<void(const std::string&)> m_log;
    uint64_t m_replaySequence = 0;
    unsigned m_exceptionCount = 0;
    unsigned m_loggerExceptionCount = 0;
    unsigned m_referenceGeneration = 0;
    int m_faultRecoverySamples = 0;
    int m_faultAutoRetries = 0;
    int64_t m_faultRecoveryLastMs = 0;

    bool m_enabled = true;
    int  m_sensitivityPct = 50;
    float m_medMassRatio, m_medSnrDropDb, m_medFeatureRatio, m_medBrightRatio;
    float m_recMassRatio, m_recFeatureRatio, m_recSnrDropDb;
    float m_slowRatio;

    Ring m_baseMass, m_baseBright, m_baseSnr, m_baseScore, m_baseFeatures;
    int64_t m_lastBaselinePushMs = 0;
    int64_t m_clearAccumMs = 0;        // CLEAR-eligible data accumulated since reset (warm-up)
    int64_t m_prevSampleMs = 0;
    int m_prevExposureMs = 0;
    bool m_prevClearEligible = false;
    double m_massJoin = 1.;
    float m_joinBefore = -1.f, m_joinEnsembleBefore = -1.f;
    int m_joinHold = 0;
    float m_motionMass = -1.f, m_motionEnsemble = -1.f, m_motionScatter = -1.f;
    int64_t m_motionMs = 0;
    int m_joinFromMs = 0, m_joinToMs = 0;
    Ring m_joinMass, m_joinEnsemble;
    bool m_snrRelearning = false;
    unsigned m_clipBits = 0;
    int m_clipFrames = 0;
    int64_t m_lastClipMs = 0;
    int64_t m_lastEnsembleMs = 0;
    bool m_primaryHeldOut = false;
    Ring m_unclippedRun;
    float m_unclippedLevel = -1.f;
    bool m_joinClipped = false, m_joinDeferred = false;
    int m_deferredFromMs = 0, m_deferredToMs = 0;

    Ring m_shortMass, m_shortBright, m_shortSnr, m_shortScore, m_shortFeatures;
    Ring m_shortEnsemble;
    Ring m_baseEnsemble;   // 3-frame multi-star medians of Warmup/Clear frames, for its clear standard
    Trend m_trendMass, m_trendSnr, m_trendFeatures, m_trendEnsemble;

    Ring m_refMass, m_refBright, m_refSnr, m_refScore, m_refFeatures;

    bool m_seenMass = false, m_seenSnr = false, m_seenScore = false, m_seenFeatures = false;

    Anchor m_anchorBright, m_anchorMass, m_anchorSnr, m_anchorScore, m_anchorFeatures;
    Anchor m_snrSessionBest;
    Anchor m_anchorEnsemble;
    int64_t m_lastAnchorUpdateMs = 0;
    float m_anchorAirmass = -1.f;

    // State machine.
    SceneState m_state = SceneState::Warmup;
    bool m_obscuredRecovery = false;
    int64_t m_stateSinceMs = 0;
    int m_recoverySamples = 0;
    int64_t m_alternateSinceMs = 0;
    int64_t m_suspectQuietSinceMs = 0;
    int     m_lossRun = 0;
    bool    m_sawGoodDetection = false;
    bool    m_lossLogged = false;
    int64_t m_lastPeriodicLogMs = 0;

    struct Identity {
        int exposureMs = -1, autoExposure = -1, gain = -1, bitDepth = -1, mode = -1;
        int frameW = -1, frameH = -1;
        int roiX = -1, roiY = -1, roiW = -1, roiH = -1;
        unsigned sourceGen = 0;
        int zenith = -1;
        float airmass = -1.f;
        int64_t tMs = 0;
        bool valid = false;
    } m_ident;
    bool m_identBeforeReset = false;
    static const char* IdentityDelta(const Identity& prev, const Identity& cur);
    static Identity IdentityOf(const SceneSample& s);

    MassDeclineDetector m_massDecline;
    SkyStabilityDetector m_skyStability;
    SceneTelemetry m_tele;
};
