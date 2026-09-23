#include "../contributions/CloudDetector/CloudDetector.h"
#include "../contributions/CloudDetector/CloudDetectorConfig.h"

#include <algorithm>
#include <cstdlib>
#include <climits>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>
#include <map>
#include <sstream>
#include <locale>
#include <random>

namespace {

void Require(bool condition, const char *message)
{
    if (!condition)
    {
        std::cerr << "cloud detector test failed: " << message << '\n';
        throw std::runtime_error(message);
    }
}

SceneSample ClearSample(int64_t tMs)
{
    SceneSample sample;
    sample.tMs = tMs;
    sample.detected = true;
    sample.stableLock = true;
    sample.score = -3.0f; // negative Gaussian-equivalent FWHM in star mode
    sample.snr = 20.0f;
    sample.mass = 100.0f;
    sample.brightCeil = 100.0f;
    sample.brightExposureMs = 1000;
    sample.exposureMs = 1000;
    sample.gain = 50;
    sample.bitDepth = 8;
    sample.frameW = 640;
    sample.frameH = 480;
    sample.mode = 0;

    return sample;
}

int64_t Arm(CloudDetector& detector)
{
    int64_t t = 1000;
    for (int i = 0; i < 16; ++i, t += 2000)
        detector.Feed(ClearSample(t));
    Require(detector.GetState() == SceneState::Clear, "warm-up did not establish a clear baseline");
    return t;
}

void TargetLossIsNotCloud()
{
    CloudDetector detector;
    int64_t t = Arm(detector);
    for (int i = 0; i < 5; ++i, t += 2000)
    {
        SceneSample sample = ClearSample(t);
        sample.detected = false;
        detector.Feed(sample);
    }
    Require(detector.GetState() == SceneState::Clear, "target loss was classified as cloud");
    Require(detector.GetTelemetry().lossRun == 5, "target loss telemetry was not retained");
}

void BiasedDarkFrameTripsContrastChannel()
{
    CloudDetector detector;
    int64_t t = Arm(detector);
    for (int i = 0; i < 3; ++i, t += 2000)
    {
        SceneSample sample = ClearSample(t);
        sample.detected = false;
        // The image feeder reports P99.5-minus-median, so a physical dark frame's
        // non-zero sensor bias has already been removed before it reaches the detector.
        sample.brightCeil = 2.0f;
        detector.Feed(sample);
    }
    Require(detector.GetState() == SceneState::Obscured, "dark frame did not trip the contrast channel");
    Require(detector.GetTelemetry().brightRatio < 0.05f, "dark-frame contrast ratio was unexpectedly high");
}

void DetectedContrastCollapseNeedsCorroboration()
{
    CloudDetector detector;
    int64_t t = Arm(detector);
    for (int i = 0; i < 20; ++i, t += 2000)
    {
        SceneSample sample = ClearSample(t);
        sample.brightCeil = 40.f;
        detector.Feed(sample);
        Require(detector.GetState() == SceneState::Clear,
                "stable target plus contrast-only collapse published a cloud verdict");
        Require(detector.GetTelemetry().severity == 0.f,
                "contrast-only noise published non-zero haze severity");
    }
}

void StablePhotometryIgnoresFwhmJitter()
{
    CloudDetector detector;
    int64_t t = Arm(detector);
    for (int i = 0; i < 20; ++i, t += 2000)
    {
        SceneSample sample = ClearSample(t);
        // This is well beyond the score channel's 0.08-pixel absolute band and used to produce
        // the characteristic false HAZE 30% even though mass and SNR were perfectly flat.
        sample.score = i % 2 == 0 ? -3.20f : -3.35f;
        detector.Feed(sample);
        Require(detector.GetState() == SceneState::Clear,
                "FWHM-only jitter published haze over stable mass/SNR");
        Require(detector.GetTelemetry().severity == 0.f,
                "FWHM-only jitter published non-zero haze severity");
    }
}

void StableMassRippleStaysClear()
{
    CloudDetector detector;
    int64_t t = Arm(detector);
    const float ripple[] = { 99.f, 100.f, 101.f };
    for (int i = 0; i < 24; ++i, t += 2000)
    {
        SceneSample sample = ClearSample(t);
        sample.mass = ripple[i % 3];
        detector.Feed(sample);
        Require(detector.GetState() == SceneState::Clear,
                "normal small mass ripple was classified as haze");
    }
    const SceneTelemetry telemetry = detector.GetTelemetry();
    Require(telemetry.massScatterFactor >= 0.f && telemetry.massScatterFactor < 1.f,
            "normal mass ripple exceeded the learned variability band");
}

void GentleMassStepIsNotErraticCloud()
{
    CloudDetector detector;
    int64_t t = Arm(detector);
    for (int i = 0; i < 20; ++i, t += 2000)
    {
        SceneSample sample = ClearSample(t);
        sample.mass = 92.f;
        detector.Feed(sample);
        Require(detector.GetState() == SceneState::Clear,
                "a gentle stable mass step was mistaken for erratic cloud");
    }
    Require(detector.GetTelemetry().severity == 0.f,
            "a gentle stable mass step published haze severity");
}

void ModerateMassWavesHaveProportionalHaze()
{
    CloudDetector detector;
    int64_t t = Arm(detector);
    const float waves[] = { 93.f, 100.f, 107.f };
    for (int i = 0; i < 20; ++i, t += 2000)
    {
        SceneSample sample = ClearSample(t);
        sample.mass = waves[i % 3];
        detector.Feed(sample);
    }
    const SceneTelemetry telemetry = detector.GetTelemetry();
    Require(telemetry.massScatterFactor > 1.f,
            "moderate mass waves did not cross the variability band");
    Require(telemetry.severity > 0.f && telemetry.severity < 0.35f,
            "moderate mass waves jumped to an excessive haze percentage");
    Require(detector.GetState() != SceneState::Obscured,
            "moderate mass waves bypassed corroboration");
}

void SlowBroadMassWavesPublishHaze()
{
    CloudDetector detector;
    int64_t t = Arm(detector);
    const float waves[] = { 88.f, 91.f, 94.f, 97.f, 100.f, 103.f, 106.f, 109.f, 112.f,
                            109.f, 106.f, 103.f, 100.f, 97.f, 94.f, 91.f };
    bool sawHaze = false;
    for (int i = 0; i < 32; ++i, t += 2000)
    {
        SceneSample sample = ClearSample(t);
        sample.mass = waves[i % 16];
        detector.Feed(sample);
        sawHaze = sawHaze || detector.GetState() == SceneState::Suspect;
        Require(detector.GetState() != SceneState::Obscured,
                "slow mass variability alone bypassed corroboration");
    }
    Require(sawHaze,
            "broad mass waves were missed because their frame-to-frame steps were small");
}

void ModestCorrelatedFadeStartsWithLowHaze()
{
    CloudDetector detector;
    int64_t t = Arm(detector);
    for (int i = 0; i < 20; ++i, t += 2000)
    {
        SceneSample sample = ClearSample(t);
        sample.mass = 85.f;
        sample.brightCeil = 85.f;
        detector.Feed(sample);
    }
    const SceneTelemetry telemetry = detector.GetTelemetry();
    Require(detector.GetState() == SceneState::Suspect,
            "a sustained 15-percent correlated fade did not publish haze");
    Require(telemetry.severity > 0.f && telemetry.severity < 0.16f,
            "a modest correlated fade published disproportionate haze severity");
}

void SustainedModestCorrelatedFadeRemainsHaze()
{
    CloudDetector detector;
    int64_t t = Arm(detector);
    for (int i = 0; i < 40; ++i, t += 2000)
    {
        SceneSample sample = ClearSample(t);
        sample.mass = 85.f;
        sample.brightCeil = 85.f;
        detector.Feed(sample);
    }
    Require(detector.GetState() == SceneState::Suspect,
            "duration alone escalated modest haze to Obscured");
}

void DeeperSlowFadeDoesNotJumpToFiftyPercent()
{
    CloudDetector detector;
    int64_t t = Arm(detector);
    for (int i = 0; i < 20; ++i, t += 2000)
    {
        SceneSample sample = ClearSample(t);
        sample.mass = 78.f;
        sample.brightCeil = 78.f;
        detector.Feed(sample);
    }
    const SceneTelemetry telemetry = detector.GetTelemetry();
    Require(detector.GetState() == SceneState::Suspect,
            "a deeper correlated fade did not publish a suspect state");
    Require(telemetry.severity > 0.f && telemetry.severity < 0.23f,
            "a barely tripped slow fade jumped to an excessive haze percentage");
}

void ErraticMassWavesPublishHaze()
{
    CloudDetector detector;
    int64_t t = Arm(detector);
    const float waves[] = { 85.f, 100.f, 115.f };
    bool sawHaze = false;
    for (int i = 0; i < 30; ++i, t += 2000)
    {
        SceneSample sample = ClearSample(t);
        // Mean and recent three-sample median remain nominal. Only the wave amplitude changes,
        // matching a physical guiding graph whose horizontal mass trace becomes erratic in cloud.
        sample.mass = waves[i % 3];
        detector.Feed(sample);
        sawHaze = sawHaze || detector.GetState() == SceneState::Suspect;
        Require(detector.GetState() != SceneState::Obscured,
                "mass variability alone bypassed the corroboration required for obscured");
    }
    const SceneTelemetry telemetry = detector.GetTelemetry();
    Require(sawHaze, "large mass waves did not publish haze");
    Require(telemetry.massRatio > 0.95f,
            "mass-wave test accidentally depended on a mean transmission fade");
    Require(telemetry.massScatterFactor > 1.f,
            "large mass waves did not exceed the learned variability band");
}

void VariabilityWithSupportingNoiseRemainsSuspect()
{
    for (bool varySnr : { false, true }) {
        CloudDetector detector;
        int64_t t = Arm(detector);
        const unsigned generation = detector.GetTelemetry().referenceGeneration;
        bool sawWarning = false;
        for (int i = 0; i < 400; ++i, t += 3400) {
            auto s = ClearSample(t);
            s.mass = varySnr ? 100.f : 94.f + 4.f * (i % 3 - 1);
            s.snr = varySnr ? 20.f + 0.6f * (i % 3 - 1) : 19.85f;
            s.score = -3.2f;
            s.brightCeil = 75.f + 10.f * (i % 3);
            s.ensembleStars = 3;
            s.ensembleRatio = 0.95f;
            detector.Feed(s);
            const auto tele = detector.GetTelemetry();
            sawWarning = sawWarning || tele.state == SceneState::Suspect;
            Require(tele.state != SceneState::Obscured,
                    "one variable channel plus FWHM/contrast noise escalated to Obscured");
            Require(tele.referenceGeneration == generation,
                    "active variability was normalized by replacing the clear reference");
            Require(tele.severity < 0.3f,
                    "FWHM imposed a 30 percent haze floor without photometric level loss");
        }
        Require(sawWarning && detector.GetState() == SceneState::Suspect,
                "ongoing variability must retain its haze warning");
    }
}

void FadedPhotometryStillAcceptsSupportingVotes()
{
    for (bool useContrast : { false, true }) {
        CloudDetector detector;
        int64_t t = Arm(detector);
        for (int i = 0; i < 30; ++i, t += 3400) {
            auto s = ClearSample(t);
            s.mass = 65.f;
            if (useContrast) s.brightCeil = 70.f;
            else s.score = -3.5f;
            detector.Feed(s);
        }
        Require(detector.GetState() == SceneState::Suspect,
                "partial corroborated loss must remain Suspect");
    }
}

void CorroboratedMassAndSnrWavesRemainHaze()
{
    CloudDetector detector;
    int64_t t = Arm(detector);
    const float massWaves[] = { 85.f, 100.f, 115.f };
    const float snrWaves[] = { 18.f, 20.f, 22.f };
    for (int i = 0; i < 30; ++i, t += 2000)
    {
        SceneSample sample = ClearSample(t);
        sample.mass = massWaves[i % 3];
        sample.snr = snrWaves[i % 3];
        detector.Feed(sample);
    }
    const SceneTelemetry telemetry = detector.GetTelemetry();
    Require(telemetry.massRatio > 0.95f && telemetry.snrDropDb < 0.5f,
            "variability test accidentally depended on a mean fade");
    Require(telemetry.massScatterFactor > 1.f && telemetry.snrScatterFactor > 1.f,
            "correlated mass/SNR waves did not exceed their variability bands");
    Require(detector.GetState() == SceneState::Suspect,
            "variability alone must remain Suspect");
}

void StarSnrAndFwhmDriveSustainedVote()
{
    CloudDetector detector;
    int64_t t = Arm(detector);
    for (int i = 0; i < 16; ++i, t += 2000)
    {
        SceneSample sample = ClearSample(t);
        sample.snr = 15.0f;
        sample.score = -5.0f; // broader FWHM is a lower generic quality score
        detector.Feed(sample);
    }
    Require(detector.GetState() == SceneState::Suspect, "SNR/FWHM changes must remain Suspect");
    Require(detector.GetTelemetry().snrDropDb > 3.0f, "star SNR drop was not reported");
    Require(detector.GetTelemetry().scoreDelta < -1.0f, "star FWHM score drop was not reported");
}

void ExposureChangeResetsBaseline()
{
    CloudDetector detector;
    int64_t t = Arm(detector);
    SceneSample sample = ClearSample(t);
    sample.exposureMs = 2000;
    detector.Feed(sample);
    Require(detector.GetState() == SceneState::Warmup, "exposure change did not reset the baseline");
}

void ImageContrastHandlesCameraScalesAndHotPixels()
{
    const int width = 49;
    const int height = 49;
    std::vector<unsigned short> image8(width * height, 20);
    for (int i = 0; i < 20; ++i)
        image8[i] = 200;

    const float contrast8 = CloudHighTailContrast(image8.data(), width, height, 0, 0, width, height);
    Require(contrast8 == 180.f, "unscaled 8-bit contrast was incorrect");

    std::vector<unsigned short> image16(image8.size());
    for (size_t i = 0; i < image8.size(); ++i)
        image16[i] = (unsigned short) (image8[i] * 257U);
    const float contrast16 = CloudHighTailContrast(image16.data(), width, height, 0, 0, width, height);
    const float scale = contrast16 / contrast8;
    Require(scale > 256.f && scale < 258.f, "expanded 8-bit contrast did not preserve its scale");

    std::vector<unsigned short> hotPixelFrame(width * height, 20);
    hotPixelFrame[0] = 255;
    const float hotPixelContrast = CloudHighTailContrast(hotPixelFrame.data(), width, height, 0, 0, width, height);
    Require(hotPixelContrast == 0.f, "an isolated hot pixel was mistaken for image signal");
}

void BitDepthChangeResetsBaseline()
{
    CloudDetector detector;
    int64_t t = Arm(detector);
    SceneSample sample = ClearSample(t);
    sample.bitDepth = 16;
    detector.Feed(sample);
    Require(detector.GetState() == SceneState::Warmup, "bit-depth change did not reset the baseline");
}

void AutoExposureKeepsContrastOnOneScale()
{
    CloudDetector detector;
    int64_t t = 1000;
    for (int i = 0; i < 16; ++i, t += 2000)
    {
        SceneSample sample = ClearSample(t);
        sample.exposureMs = 0; // auto exposure: not part of acquisition identity
        sample.brightExposureMs = 2000;
        sample.brightCeil = 200.f;
        detector.Feed(sample);
    }
    Require(detector.GetState() == SceneState::Clear, "auto-exposure baseline did not arm");

    for (int i = 0; i < 6; ++i, t += 2000)
    {
        SceneSample sample = ClearSample(t);
        sample.exposureMs = 0;
        sample.brightExposureMs = 1000;
        sample.brightCeil = 100.f;
        detector.Feed(sample);
    }
    Require(detector.GetState() == SceneState::Clear, "auto-exposure change altered normalized contrast");
}

void BetterClearViewRaisesProtectedStandardWithoutReset()
{
    CloudDetector detector;
    int64_t t = 1000;
    for (int i = 0; i < 16; ++i, t += 2000)
    {
        SceneSample sample = ClearSample(t);
        sample.mass = 90.f;
        sample.snr = 18.f;
        sample.score = -4.f;
        sample.brightCeil = 200.f; // cloud sparkle inflated the learned contrast
        detector.Feed(sample);
    }
    Require(detector.GetState() == SceneState::Clear, "contaminated reference did not arm");

    bool sawObscured = false;
    for (int i = 0; i < 8; ++i, t += 2000)
    {
        SceneSample sample = ClearSample(t);
        sample.mass = 110.f;
        sample.snr = 21.f;   // improved along with mass, as in the simulator log
        sample.score = -3.f;
        // The simulator's cloud sparkle makes contrast alternate across the old reference;
        // relearning must follow sustained target improvement, not require a continuous dip.
        sample.brightCeil = i % 2 == 0 ? 100.f : 240.f;
        detector.Feed(sample);
        sawObscured = sawObscured || detector.GetState() == SceneState::Obscured;
    }
    Require(!sawObscured, "a better clear view was classified as obscured");
    Require(detector.GetState() != SceneState::Warmup,
            "a better view erased the certified standard and restarted warm-up");

    // Returning to the old, poorer view must now compare against the better standard rather than
    // silently rebuilding a lower baseline.
    for (int i = 0; i < 3; ++i, t += 2000)
    {
        SceneSample sample = ClearSample(t);
        sample.mass = 90.f;
        sample.snr = 18.f;
        sample.score = -4.f;
        sample.brightCeil = 200.f;
        detector.Feed(sample);
    }
    const SceneTelemetry telemetry = detector.GetTelemetry();
    Require(telemetry.massRatio < 0.9f, "better mass standard was allowed to fall immediately");
    Require(telemetry.snrDropDb > 2.5f, "better SNR standard was not retained");
    Require(telemetry.scoreDelta < -0.8f, "better FWHM standard was not retained");
}

void HealthyAug31TelemetryStaysClear()
{
    CloudDetector detector;
    int64_t t = Arm(detector);
    for (int i = 0; i < 20; ++i, t += 2000)
    {
        SceneSample sample = ClearSample(t);
        sample.mass = 97.4f;
        sample.brightCeil = 96.7f;
        sample.snr = 19.944f;
        sample.score = -3.32f;
        sample.ensembleRatio = 0.981f;
        sample.ensembleStars = 3;
        detector.Feed(sample);
        Require(detector.GetState() == SceneState::Clear,
                "healthy Aug 31 photometry was classified as cloud");
    }
}

void StableNearTotalLossCannotRequalify()
{
    CloudDetector detector;
    int64_t t = Arm(detector);
    const unsigned referenceGeneration = detector.GetTelemetry().referenceGeneration;

    auto feedLower = [&]() {
        SceneSample sample = ClearSample(t);
        sample.mass = 6.f;
        sample.brightCeil = 10.f;
        sample.snr = 18.f;
        sample.score = -4.f;
        detector.Feed(sample);
        t += 2000;
    };

    for (int i = 0; i < 45 / 2; ++i)
        feedLower();
    Require(detector.GetState() != SceneState::Clear,
            "lower conditions bypassed alternate-baseline qualification");

    for (int i = 0; i < 4 * 60 / 2; ++i)
        feedLower();
    Require(detector.GetState() == SceneState::Obscured,
            "near-total stable loss was incorrectly accepted as clear");
    Require(detector.GetTelemetry().referenceGeneration == referenceGeneration,
            "near-total loss replaced the certified reference");
    Require(detector.GetTelemetry().massRatio < 0.10f,
            "near-total loss was normalized away");
}

void VariableObstructionCannotRequalify()
{
    CloudDetector detector;
    int64_t t = Arm(detector);

    for (int i = 0; i < 6 * 60 / 2; ++i, t += 2000)
    {
        SceneSample sample = ClearSample(t);
        sample.mass = i % 2 == 0 ? 8.f : 42.f;
        sample.brightCeil = i % 2 == 0 ? 10.f : 45.f;
        sample.snr = i % 2 == 0 ? 16.f : 23.f;
        detector.Feed(sample);
    }
    Require(detector.GetState() != SceneState::Clear,
            "variable obstruction was accepted as an alternate baseline");
}

void NoisyContrastAndFwhmDoNotBlockRecovery()
{
    CloudDetector detector;
    int64_t t = Arm(detector);
    for (int i = 0; i < 3; ++i, t += 2000)
    {
        SceneSample sample = ClearSample(t);
        sample.detected = false;
        sample.brightCeil = 2.f;
        detector.Feed(sample);
    }
    Require(detector.GetState() != SceneState::Clear, "test blackout did not latch obscured");

    bool sawClearRecovery = false;
    for (int i = 0; i < 45; ++i, t += 2000)
    {
        SceneSample sample = ClearSample(t);
        sample.score = -3.12f; // one mildly degraded target-derived channel
        sample.brightCeil = i % 2 == 0 ? 40.f : 160.f;
        detector.Feed(sample);
        if (i < 30)
            Require(detector.GetState() != SceneState::Clear,
                    "recovery skipped the full photometric trend window");
        sawClearRecovery = sawClearRecovery || detector.GetState() == SceneState::Clear;
    }
    Require(sawClearRecovery,
            "noisy contrast/FWHM prevented a healthy detection quorum from recovering");

    for (int i = 0; i < 20; ++i, t += 2000)
    {
        SceneSample sample = ClearSample(t);
        sample.score = -3.12f;
        sample.brightCeil = i % 2 == 0 ? 40.f : 160.f;
        detector.Feed(sample);
        Require(detector.GetState() != SceneState::Obscured,
                "detected contrast jitter re-latched obscured without corroboration");
    }
}

void GuidingSessionResetRelearnsCurrentClearView()
{
    CloudDetector detector;
    int64_t t = Arm(detector);

    for (int i = 0; i < 3; ++i, t += 2000)
    {
        SceneSample sample = ClearSample(t);
        sample.detected = false;
        sample.brightCeil = 2.f;
        detector.Feed(sample);
    }
    Require(detector.GetState() == SceneState::Obscured,
            "test cloud did not latch obscured before the session restart");

    // Guider::StartGuiding invokes this hard reset. A long stopped interval and a different
    // stable star/field must establish a fresh standard instead of recovering against the old
    // cloud-era session statistics.
    t += 60 * 60 * 1000;
    detector.Reset("guiding started");
    const SceneTelemetry resetTelemetry = detector.GetTelemetry();
    Require(resetTelemetry.state == SceneState::Warmup,
            "new guiding session retained the obscured verdict");
    Require(resetTelemetry.severity == 0.f && resetTelemetry.massRatio < 0.f &&
                resetTelemetry.slowMassRatio < 0.f && resetTelemetry.lossRun == 0,
            "new guiding session retained stale detector telemetry");

    for (int i = 0; i < 16; ++i, t += 2000)
    {
        SceneSample sample = ClearSample(t);
        sample.mass = 60.f;
        sample.brightCeil = 60.f;
        sample.snr = 12.f;
        detector.Feed(sample);
    }
    Require(detector.GetState() == SceneState::Clear,
            "new guiding session did not learn the current stable clear view");
}

void MotionResumeClearsTransientEvidenceButKeepsBaseline()
{
    CloudDetector detector;
    int64_t t = Arm(detector);

    detector.ResumeAfterMotion("dither settled");
    SceneTelemetry telemetry = detector.GetTelemetry();
    Require(telemetry.state == SceneState::Clear,
            "motion resume discarded the established clear verdict");
    Require(telemetry.severity == 0.f && telemetry.massRatio < 0.f &&
                telemetry.brightRatio < 0.f && telemetry.lossRun == 0,
            "motion resume retained transient detector evidence");

    // Three fresh collapsed-mass samples must still fast-trip. If ResumeAfterMotion accidentally
    // performed a hard reset, the detector would instead be back in Warmup with no anchor.
    for (int i = 0; i < 3; ++i, t += 2000)
    {
        SceneSample sample = ClearSample(t);
        sample.mass = 5.f;
        detector.Feed(sample);
    }
    Require(detector.GetState() == SceneState::Obscured,
            "motion resume discarded the learned clear-sky baseline");

    detector.ResumeAfterMotion("second dither settled");
    telemetry = detector.GetTelemetry();
    Require(telemetry.state == SceneState::Obscured,
            "motion resume cleared a genuine obscured latch");
    Require(telemetry.severity == 0.f && telemetry.massRatio < 0.f,
            "motion resume retained the pre-motion trip window");

    for (int i = 0; i < 45; ++i, t += 2000)
        detector.Feed(ClearSample(t));
    Require(detector.GetState() == SceneState::Clear,
            "obscured latch could not recover from fresh post-motion samples");
}

void OptionalMultiStarEvidenceCorroboratesPrimaryFade()
{
    CloudDetector detector;
    int64_t t = Arm(detector);
    for (int i = 0; i < 18; ++i, t += 2000)
    {
        SceneSample sample = ClearSample(t);
        sample.mass = 70.f;
        sample.ensembleRatio = 0.60f;
        sample.ensembleStars = 4;
        detector.Feed(sample);
    }
    const SceneTelemetry telemetry = detector.GetTelemetry();
    Require(telemetry.ensembleStars == 4 && telemetry.ensembleRatio < 0.7f,
            "multi-star evidence was not published");
    Require(detector.GetState() == SceneState::Suspect,
            "multi-star consensus did not corroborate a sustained primary fade");
}

void ThrowingLoggerIsContainedAndDisabled()
{
    CloudDetector detector;
    int calls = 0;
    detector.SetLogger([&calls](const std::string&) {
        ++calls;
        throw std::runtime_error("injected logger failure");
    });

    detector.Reset("logger fault injection");
    SceneTelemetry telemetry = detector.GetTelemetry();
    Require(telemetry.healthy, "a diagnostic logger failure invalidated detector calculations");
    Require(telemetry.exceptionCount == 1, "logger exception was not counted");
    Require(telemetry.loggerExceptionCount == 1, "logger exception subtype was not counted");
    Require(calls == 1, "throwing logger was not called exactly once");

    Arm(detector);
    Require(calls == 1, "throwing logger was not disabled after its first exception");
    telemetry = detector.GetTelemetry();
    Require(telemetry.healthy, "detector did not remain healthy after logger containment");
    Require(telemetry.exceptionCount == 1 && telemetry.loggerExceptionCount == 1,
            "exception counters did not survive normal feeds");
}

void NonFiniteSamplesCannotPoisonTelemetry()
{
    CloudDetector detector;
    int64_t t = Arm(detector);

    for (int i = 0; i < 8; ++i, t += 2000)
    {
        SceneSample malformed = ClearSample(t);
        malformed.mass = std::numeric_limits<float>::quiet_NaN();
        malformed.snr = std::numeric_limits<float>::infinity();
        malformed.score = -std::numeric_limits<float>::infinity();
        malformed.brightCeil = std::numeric_limits<float>::quiet_NaN();
        detector.Feed(malformed);
    }

    const SceneTelemetry telemetry = detector.GetTelemetry();
    Require(telemetry.healthy, "non-finite optional metrics faulted the detector");
    Require(detector.GetState() == SceneState::Clear, "non-finite metrics changed the scene verdict");
    Require(std::isfinite(telemetry.severity), "non-finite severity escaped sanitization");
    Require(std::isfinite(telemetry.snrDropDb), "non-finite SNR telemetry escaped sanitization");
    Require(std::isfinite(telemetry.scoreDelta), "non-finite score telemetry escaped sanitization");
    Require(telemetry.massRatio < 0.f || std::isfinite(telemetry.massRatio),
            "non-finite mass ratio escaped sanitization");
    Require(telemetry.brightRatio < 0.f || std::isfinite(telemetry.brightRatio),
            "non-finite brightness ratio escaped sanitization");
}

void BackwardTimestampResetsInsteadOfUnderflowing()
{
    CloudDetector detector;
    const int64_t next = Arm(detector);
    detector.Feed(ClearSample(next - 10000));
    Require(detector.GetState() == SceneState::Warmup,
            "backward timestamp did not reset timing-dependent detector state");
    Require(detector.GetTelemetry().healthy, "backward timestamp was treated as an exception");
}

void ContrastRejectsOverflowingRoi()
{
    const unsigned short image[] = { 1, 2, 3, 4 };
    Require(CloudHighTailContrast(image, 2, 2, INT_MAX, INT_MAX, INT_MAX, INT_MAX) < 0.f,
            "overflowing positive ROI was not rejected");
    Require(CloudHighTailContrast(image, 2, 2, INT_MIN, INT_MIN, INT_MAX, INT_MAX) < 0.f,
            "overflowing negative ROI was not rejected");
}

void ReportedIntegrationFaultFailsOpenUntilReset()
{
    CloudDetector detector;
    Arm(detector);

    detector.ReportFault("fault injection", "test exception");
    SceneTelemetry telemetry = detector.GetTelemetry();
    Require(!telemetry.healthy, "reported integration fault did not mark the detector unhealthy");
    Require(telemetry.exceptionCount == 1, "reported integration fault was not counted");
    Require(detector.GetState() == SceneState::Warmup, "reported fault left stale detector state");
    Require(detector.IsClear(), "reported fault did not fail open");

    detector.Reset("test recovery");
    telemetry = detector.GetTelemetry();
    Require(telemetry.healthy, "explicit reset did not recover detector health");
    Require(telemetry.exceptionCount == 1, "explicit reset erased lifetime exception count");
}

void MissingChannelsCannotRepeatVotes()
{
    CloudDetector detector;
    int64_t t = Arm(detector);
    for (int i = 0; i < 10; ++i, t += 2000) {
        SceneSample s = ClearSample(t);
        s.mass = 70.f; s.snr = 15.f;
        detector.Feed(s);
    }
    Require(detector.GetState() == SceneState::Suspect, "partial medium vote did not become suspect");
    for (int i = 0; i < 20; ++i, t += 2000) {
        SceneSample s = ClearSample(t);
        s.mass = s.snr = s.score = std::numeric_limits<float>::quiet_NaN();
        s.brightCeil = -1.f;
        detector.Feed(s);
        Require(detector.GetState() == SceneState::Suspect,
                "missing data completed a trip or cleared a suspect verdict");
        Require(detector.GetTelemetry().massRatio < 0.f && detector.GetTelemetry().brightRatio < 0.f,
                "missing channel reused its old median");
    }
}

void GradualClearingWaitsForPlateau()
{
    for (int duration : { 120, 240, 600 }) {
        CloudDetector detector;
        int64_t t = Arm(detector);
        for (int i = 0; i < 3; ++i, t += 2000) {
            SceneSample s = ClearSample(t);
            s.detected = false; s.brightCeil = 0.f;
            detector.Feed(s);
        }
        Require(detector.GetState() != SceneState::Clear, "ramp setup failed to obscure");
        for (int elapsed = 0; elapsed < duration; elapsed += 2, t += 2000) {
            SceneSample s = ClearSample(t);
            const float progress = static_cast<float>(elapsed) / duration;
            s.mass = 55.f + 60.f * progress;
            s.snr = 16.f + 7.f * progress;
            detector.Feed(s);
            Require(detector.GetState() != SceneState::Clear,
                    "continuing one-to-ten-minute clearing ramp released the cloud hold");
        }
        for (int i = 0; i < 60; ++i, t += 2000) {
            SceneSample s = ClearSample(t);
            s.mass = 115.f; s.snr = 23.f;
            s.score = -3.12f; s.brightCeil = i % 2 ? 40.f : 160.f;
            detector.Feed(s);
        }
        Require(detector.GetState() == SceneState::Clear, "settled clear plateau failed to release hold");
    }
}

void AlternateBaselineCannotAcceptContinuingClearing()
{
    CloudDetector detector;
    int64_t t = Arm(detector);
    for (int i = 0; i < 120; ++i, t += 2000) {
        SceneSample s = ClearSample(t);
        s.mass = 6.f + 6.f * i / 120.f;
        s.brightCeil = 10.f + 10.f * i / 120.f;
        s.snr = 17.f + 2.f * i / 120.f;
        detector.Feed(s);
        if (i >= 3)
            Require(detector.GetState() == SceneState::Obscured,
                    "alternate-baseline path bypassed the clearing trend gate");
    }
    for (int i = 0; i < 100; ++i, t += 2000) {
        SceneSample s = ClearSample(t);
        s.mass = 12.f; s.brightCeil = 20.f; s.snr = 19.f;
        detector.Feed(s);
    }
    Require(detector.GetState() != SceneState::Clear, "deep residual attenuation was normalized as clear");
}

int64_t Blackout(CloudDetector& detector)
{
    int64_t t = Arm(detector);
    for (int i = 0; i < 3; ++i, t += 2000) {
        auto s = ClearSample(t);
        s.detected = false; s.brightCeil = 0.f;
        detector.Feed(s);
    }
    Require(detector.GetState() == SceneState::Obscured, "blackout setup failed");
    return t;
}

void MissingEvidenceAndGapRestartRecovery()
{
    for (bool gap : { false, true }) {
        CloudDetector detector;
        int64_t t = Blackout(detector);
        for (int i = 0; i < 32; ++i, t += 2000) detector.Feed(ClearSample(t));
        Require(detector.GetTelemetry().recoverySettled && detector.GetState() != SceneState::Clear,
                "expected plateau qualification before the recovery hold completes");
        if (gap) {
            t += 120000;
        } else {
            for (int i = 0; i < 15; ++i, t += 2000) {
                auto s = ClearSample(t);
                s.mass = s.snr = std::numeric_limits<float>::quiet_NaN();
                detector.Feed(s);
                Require(!detector.GetTelemetry().recoverySettled && detector.GetState() != SceneState::Clear,
                        "missing primary evidence released a hold");
            }
        }
        for (int i = 0; i < 30; ++i, t += 2000) {
            detector.Feed(ClearSample(t));
            Require(detector.GetState() != SceneState::Clear,
                    "interrupted recovery failed to refill its time window");
        }
        for (int i = 0; i < 15; ++i, t += 2000) detector.Feed(ClearSample(t));
        Require(detector.GetState() == SceneState::Clear, "fresh recovery after interruption failed");
    }
}

void StalledFeedExpiresWithoutErasingVerdict()
{
    CloudDetector detector;
    int64_t t = Arm(detector);
    Require(detector.GetTelemetry(t).fresh, "recent frame was marked stale");
    auto stale = detector.GetTelemetry(t + 60000);
    Require(!stale.fresh && stale.state == SceneState::Clear && !stale.recoverySettled,
            "stalled stream advertised fresh Clear or erased its remembered verdict");
    detector.Reset("blackout freshness test");
    t = Blackout(detector);
    stale = detector.GetTelemetry(t + 60000);
    Require(!stale.fresh && stale.state == SceneState::Obscured && !detector.IsClear(),
            "snapshot expiry must not release an obscured latch");
    detector.ResumeAfterMotion("test");
    Require(!detector.GetTelemetry(t).fresh, "motion resume advertised evidence before any new frame");
    detector.Reset("invalid primary freshness");
    t = Arm(detector);
    auto unavailable = ClearSample(t);
    unavailable.mass = std::numeric_limits<float>::quiet_NaN();
    detector.Feed(unavailable);
    Require(!detector.GetTelemetry(t).fresh && detector.GetState() == SceneState::Clear,
            "missing learned photometry advertised a cached Clear verdict as current");
}

void DuplicatesCannotFillWindowsAndOptionalChannelsStayOptional()
{
    CloudDetector detector;
    int64_t t = Blackout(detector);
    auto s = ClearSample(t);
    detector.Feed(s);
    for (int i = 0; i < 500; ++i) detector.Feed(s);
    Require(detector.GetTelemetry().massRatio < 0.f && !detector.GetTelemetry().recoverySettled,
            "duplicate timestamp filled fresh short/trend windows");
    t += 2000;
    for (int i = 0; i < 45; ++i, t += 2000) {
        s = ClearSample(t);
        s.brightCeil = -1.f;
        s.score = std::numeric_limits<float>::quiet_NaN();
        if (i % 3) { s.ensembleStars = 4; s.ensembleRatio = 1.f; }
        detector.Feed(s);
    }
    Require(detector.GetState() == SceneState::Clear,
            "missing optional FWHM/contrast blocked fresh mass/SNR recovery");
}

void SlowAndIrregularCadencesCanRecover()
{
    for (int interval : { 700, 3000, 15000, 30000 }) {
        CloudDetector detector;
        int64_t t = 1000;
        auto sampleAt = [&](int64_t when) {
            auto s = ClearSample(when);
            s.exposureMs = 0; // auto-exposure uses brightExposureMs for cadence, not identity
            s.brightExposureMs = interval;
            s.brightCeil = 100.f * interval / 1000.f;
            return s;
        };
        for (int i = 0; i < 80; ++i, t += interval) detector.Feed(sampleAt(t));
        Require(detector.GetState() == SceneState::Clear, "cadence fixture failed to arm");
        for (int i = 0; i < 3; ++i, t += interval) {
            auto s = sampleAt(t);
            s.detected = false; s.brightCeil = 0.f;
            detector.Feed(s);
        }
        for (int elapsed = 0; elapsed < 240000; elapsed += interval, t += interval)
            detector.Feed(sampleAt(t));
        Require(detector.GetState() == SceneState::Clear && detector.GetTelemetry(t).fresh,
                "slow/non-dividing cadence could not complete a settled recovery");
    }
}

void SmallPhotometricNoiseDoesNotBlockPlateau()
{
    CloudDetector detector;
    int64_t t = Blackout(detector);
    for (int i = 0; i < 75; ++i, t += 2000) {
        auto s = ClearSample(t);
        s.mass += static_cast<float>((i * 7) % 5 - 2) * 0.4f;
        s.snr += static_cast<float>((i * 3) % 5 - 2) * 0.04f;
        detector.Feed(s);
    }
    Require(detector.GetState() == SceneState::Clear && !detector.GetTelemetry().clearingTrend,
            "small stationary mass/SNR noise blocked recovery");
}

void SuspectAcceptsImprovingRecoveryAndEnsembleHistoryIsFresh()
{
    CloudDetector detector;
    int64_t t = Arm(detector);
    for (int i = 0; i < 3; ++i, t += 2000) {
        auto s = ClearSample(t);
        s.mass = 70.f;
        detector.Feed(s);
    }
    Require(detector.GetState() == SceneState::Suspect, "single mass channel did not become suspect");
    bool recovered = false;
    for (int i = 0; i < 90; ++i, t += 2000) {
        auto s = ClearSample(t);
        s.mass = 75.f + 40.f * i / 90.f;
        detector.Feed(s);
        recovered = recovered || detector.GetState() == SceneState::Clear;
    }
    Require(recovered, "Suspect remained latched while every primary channel was stable or improving");

    detector.Reset("mixed recovery directions");
    t = Arm(detector);
    for (int i = 0; i < 3; ++i, t += 2000) {
        auto s = ClearSample(t);
        s.mass = 70.f;
        detector.Feed(s);
    }
    for (int i = 0; i < 90; ++i, t += 2000) {
        auto s = ClearSample(t);
        s.mass = 75.f + 40.f * i / 90.f;
        s.snr = 20.f - 1.5f * i / 90.f;
        detector.Feed(s);
        Require(detector.GetState() != SceneState::Clear,
                "an improving channel hid deterioration in another recovery channel");
    }

    detector.Reset("ensemble dropout");
    t = Arm(detector);
    for (int i = 0; i < 3; ++i, t += 2000) {
        auto s = ClearSample(t);
        s.ensembleStars = 4; s.ensembleRatio = 0.6f;
        detector.Feed(s);
    }
    detector.Feed(ClearSample(t)); t += 2000;
    auto s = ClearSample(t);
    s.ensembleStars = 4; s.ensembleRatio = 1.f;
    detector.Feed(s);
    Require(detector.GetTelemetry().ensembleRatio < 0.f,
            "returning ensemble mixed old obscured history with a fresh sample");
}

void PersistentCalculationFaultExhaustsAutomaticRetries()
{
    CloudDetector detector;
    int64_t t = Arm(detector);
    detector.ReportFault("test", "sample assembly");
    for (int retry = 0; retry < CONFIG_CLOUD_FAULT_MAX_AUTO_RETRIES; ++retry) {
        for (int i = 0; i < CONFIG_CLOUD_FAULT_RECOVERY_SAMPLES; ++i, t += 2000)
            detector.Feed(ClearSample(t));
        Require(detector.GetTelemetry().healthy, "automatic fault retry did not start");
        detector.ReportFault("test", "persistent sample assembly");
    }
    for (int i = 0; i < 30; ++i, t += 2000)
        detector.Feed(ClearSample(t));
    Require(!detector.GetTelemetry().healthy && !detector.GetTelemetry(t).fresh &&
            detector.GetState() == SceneState::Warmup, "persistent fault bypassed the retry limit");
    Require(detector.GetTelemetry().exceptionCount == CONFIG_CLOUD_FAULT_MAX_AUTO_RETRIES + 1,
            "persistent fault diagnostics were not retained");
    Require(detector.GetTelemetry().faultAutoRetries == CONFIG_CLOUD_FAULT_MAX_AUTO_RETRIES,
            "persistent fault retry count was not published");
    detector.Reset("operator recovery");
    Require(detector.GetTelemetry().healthy && detector.GetTelemetry().faultAutoRetries == 0,
            "explicit reset did not restore the retry budget");
}

void NormalGuideCadenceAllowsMovementWaits()
{
    for (int exposure : { 1000, 2000, 3500, 5000 }) {
        CloudDetector detector;
        int64_t t = 1000;
        int frame = 0;
        auto cycleMs = [&]() { return exposure + (frame++ % 4) * 1500; };
        auto sampleAt = [&](int64_t when) {
            auto s = ClearSample(when);
            s.exposureMs = exposure;
            s.brightExposureMs = exposure;
            s.brightCeil = 100.f * exposure / 1000.f;
            return s;
        };
        for (int i = 0; i < 60; ++i) {
            detector.Feed(sampleAt(t));
            const int cycle = cycleMs();
            Require(detector.GetTelemetry(t + cycle - 1).fresh,
                    "ordinary guide-movement wait expired a fresh frame");
            t += cycle;
        }
        Require(detector.GetState() == SceneState::Clear, "normal cadence with movement waits did not arm");
        for (int i = 0; i < 3; ++i) {
            auto s = sampleAt(t);
            s.detected = false; s.brightCeil = 0.f;
            detector.Feed(s);
            t += cycleMs();
        }
        Require(detector.GetState() != SceneState::Clear, "waiting between exposures prevented blackout detection");
        const int64_t rampStart = t;
        for (; t - rampStart < 180000; t += cycleMs()) {
            auto s = sampleAt(t);
            const float progress = static_cast<float>(t - rampStart) / 180000.f;
            s.mass = 55.f + 60.f * progress;
            s.snr = 16.f + 7.f * progress;
            detector.Feed(s);
            Require(detector.GetState() != SceneState::Clear,
                    "movement waits allowed Clear during a continuing three-minute clearing ramp");
        }
        const int64_t plateauStart = t;
        int64_t lastFrame = t;
        for (; t - plateauStart < 150000; t += cycleMs()) {
            auto s = sampleAt(t);
            s.mass = 115.f; s.snr = 23.f;
            detector.Feed(s);
            lastFrame = t;
        }
        Require(detector.GetState() == SceneState::Clear, "realistic cadence could not complete plateau recovery");
        const int64_t gap = std::max<int64_t>(CONFIG_CLOUD_SAMPLE_GAP_MIN_MS,
            (int64_t) CONFIG_CLOUD_SAMPLE_GAP_EXPOSURES * exposure + CONFIG_CLOUD_GUIDE_WAIT_ALLOWANCE_MS);
        Require(detector.GetTelemetry(lastFrame + gap).fresh && !detector.GetTelemetry(lastFrame + gap + 1).fresh,
                "exposure-plus-wait freshness boundary was not enforced");
        auto s = sampleAt(lastFrame + gap + 1);
        s.mass = 115.f; s.snr = 23.f;
        detector.Feed(s);
        Require(!detector.GetTelemetry().recoverySettled && detector.GetTelemetry().massRatio < 0.f,
                "true input gap failed to discard the previous trend and short window");
    }
}

void RecoveryRequiresThreeFreshObservations()
{
    CloudDetector detector;
    int64_t t = Blackout(detector);
    detector.ResumeAfterMotion("sparse recovery observation test");
    auto sampleAt = [&](int64_t when) {
        auto s = ClearSample(when);
        s.brightExposureMs = 30000;
        s.brightCeil = 3000.f;
        return s;
    };
    bool qualified = false;
    for (int i = 0; i < 10; ++i, t += 30000) {
        detector.Feed(sampleAt(t));
        if (detector.GetTelemetry().recoverySettled) { qualified = true; break; }
    }
    Require(qualified && detector.GetState() != SceneState::Clear, "sparse plateau did not start recovery hold");
    for (int i = 0; i < 20; ++i) detector.Feed(sampleAt(t));
    t += 30000;
    detector.Feed(sampleAt(t));
    Require(detector.GetState() != SceneState::Clear,
            "elapsed hold time or duplicate frames substituted for a third fresh recovery observation");
    t += 30000;
    detector.Feed(sampleAt(t));
    Require(detector.GetState() == SceneState::Clear, "three stable fresh observations failed to complete recovery");
}

void ReferenceGenerationTracksHardResets()
{
    CloudDetector detector;
    int64_t t = Arm(detector);
    const unsigned initial = detector.GetTelemetry().referenceGeneration;

    detector.ResumeAfterMotion("same target");
    Require(detector.GetTelemetry().referenceGeneration == initial,
            "motion resume invalidated external clear references");

    detector.Reset("new session");
    const unsigned reset = detector.GetTelemetry().referenceGeneration;
    Require(reset != initial, "hard reset did not invalidate external clear references");

    t = Arm(detector);
    auto changed = ClearSample(t);
    changed.exposureMs = 2000;
    detector.Feed(changed);
    Require(detector.GetTelemetry().referenceGeneration != reset,
            "acquisition identity reset did not invalidate external clear references");
}

void StableSingleChannelSuspectRequalifies()
{
    {
        CloudDetector detector;
        int64_t t = Arm(detector);
        const unsigned generation = detector.GetTelemetry().referenceGeneration;
        for (int i = 0; i < 100; ++i, t += 2000) {
            auto s = ClearSample(t);
            s.mass = 70.f;
            detector.Feed(s);
        }
        Require(detector.GetState() == SceneState::Clear,
                "stable low mass-only evidence retained Suspect");
        Require(detector.GetTelemetry().referenceGeneration != generation,
                "stable low mass-only evidence did not requalify its reference");
    }

    {
        CloudDetector detector;
        int64_t t = Arm(detector);
        const unsigned generation = detector.GetTelemetry().referenceGeneration;
        bool referenceChanged = false;
        for (int i = 0; i < 110; ++i, t += 2000) {
            auto s = ClearSample(t);
            s.ensembleStars = 4;
            s.ensembleRatio = referenceChanged ? 1.f : 0.65f;
            detector.Feed(s);
            referenceChanged = detector.GetTelemetry().referenceGeneration != generation;
        }
        Require(referenceChanged && detector.GetState() == SceneState::Clear,
                "stable low ensemble-only evidence retained Suspect after its reference reset");
    }
}

void VariableSingleChannelSuspectDoesNotRequalify()
{
    CloudDetector detector;
    int64_t t = Arm(detector);
    const unsigned generation = detector.GetTelemetry().referenceGeneration;
    for (int i = 0; i < 120; ++i, t += 2000) {
        auto s = ClearSample(t);
        s.mass = i % 2 ? 68.f : 88.f;
        detector.Feed(s);
    }
    Require(detector.GetState() == SceneState::Suspect,
            "variable single-channel evidence was accepted as clear");
    Require(detector.GetTelemetry().referenceGeneration == generation,
            "variable single-channel evidence replaced the clear reference");
}

void ObscuredRecoveryTimersDoNotCancelEachOther()
{
    CloudDetector detector;
    int64_t t = Blackout(detector);
    const unsigned generation = detector.GetTelemetry().referenceGeneration;
    bool referenceChanged = false;
    for (int i = 0; i < 100; ++i, t += 2000) {
        auto s = ClearSample(t);
        s.mass = 82.f;
        s.ensembleStars = 4;
        s.ensembleRatio = referenceChanged ? 1.f : (i % 6 >= 3 ? 0.858f : 0.862f);
        detector.Feed(s);
        referenceChanged = detector.GetTelemetry().referenceGeneration != generation;
    }
    Require(detector.GetState() == SceneState::Clear,
            "normal and alternate Obscured recovery timers canceled each other");
    Require(referenceChanged,
            "stable alternate recovery did not replace the reference");
}

void AutoExposureDoesNotTreatSnrScalingAsCloud()
{
    CloudDetector detector;
    int64_t t = 1000;
    for (int i = 0; i < 16; ++i, t += 2000) {
        auto s = ClearSample(t);
        s.exposureMs = 0;
        detector.Feed(s);
    }
    Require(detector.GetState() == SceneState::Clear, "auto-exposure baseline did not arm");

    for (int i = 0; i < 40; ++i, t += 2000) {
        const int exposure = 1000 + i * 100;
        auto s = ClearSample(t);
        s.exposureMs = 0;
        s.brightExposureMs = exposure;
        s.brightCeil = 100.f * exposure / 1000.f;
        s.snr = 20.f - 5.f * i / 39.f;
        detector.Feed(s);
    }
    Require(detector.GetState() == SceneState::Clear,
            "exposure-only SNR scaling produced a cloud warning");

    for (int i = 0; i < 3; ++i, t += 2000) {
        auto s = ClearSample(t);
        s.exposureMs = 0;
        s.brightExposureMs = 5000;
        s.brightCeil = 150.f;
        s.mass = 5.f;
        s.snr = 15.f;
        detector.Feed(s);
    }
    Require(detector.GetState() == SceneState::Obscured,
            "auto-exposure SNR suppression hid a genuine mass/brightness fade");
}

void TransientCalculationFaultRecoversFromFreshSamples()
{
    CloudDetector detector;
    int64_t t = Arm(detector);
    detector.ReportFault("test", "transient");
    Require(!detector.GetTelemetry().healthy, "fault injection did not mark telemetry unhealthy");

    for (int i = 0; i < 2; ++i, t += 2000)
        detector.Feed(ClearSample(t));
    Require(!detector.GetTelemetry().healthy && detector.GetTelemetry().faultRecoverySamples == 2,
            "fault recovery did not require three fresh samples");

    detector.Feed(ClearSample(t));
    Require(detector.GetTelemetry().healthy && detector.GetState() == SceneState::Warmup,
            "transient fault did not begin bounded automatic recovery");
    for (int i = 0; i < 16; ++i, t += 2000)
        detector.Feed(ClearSample(t));
    Require(detector.GetState() == SceneState::Clear,
            "automatic fault recovery did not rebuild a clear reference");
    Require(detector.GetTelemetry().exceptionCount == 1,
            "automatic fault recovery erased diagnostics");
    Require(detector.GetTelemetry().faultAutoRetries == 1,
            "automatic fault recovery retry was not published");
}

void ActiveMassVariabilityCannotRequalify()
{
    CloudDetector detector;
    int64_t t = Arm(detector);
    const unsigned generation = detector.GetTelemetry().referenceGeneration;
    const float masses[] = { 96.f, 100.f, 104.f };
    for (int i = 0; i < 180; ++i, t += 2000) {
        auto s = ClearSample(t);
        s.mass = masses[i % 3];
        detector.Feed(s);
    }
    Require(detector.GetState() == SceneState::Suspect,
            "active low-amplitude mass variability was accepted as clear");
    Require(detector.GetTelemetry().referenceGeneration == generation,
            "active low-amplitude mass variability replaced the reference");
}

void VariableEnsembleCannotRequalify()
{
    CloudDetector detector;
    int64_t t = Arm(detector);
    const unsigned generation = detector.GetTelemetry().referenceGeneration;
    const float ratios[] = { 0.45f, 0.60f, 0.75f };
    for (int i = 0; i < 180; ++i, t += 2000) {
        auto s = ClearSample(t);
        s.ensembleStars = 3;
        s.ensembleRatio = ratios[i % 3];
        detector.Feed(s);
    }
    Require(detector.GetState() == SceneState::Suspect,
            "variable ensemble evidence was accepted as clear");
    Require(detector.GetTelemetry().referenceGeneration == generation,
            "variable ensemble evidence replaced the reference");
}

void FaultRecoveryQualificationRestartsOnDiscontinuity()
{
    CloudDetector detector;
    int64_t t = Arm(detector);
    detector.ReportFault("test", "transient");
    const unsigned faultGeneration = detector.GetTelemetry().referenceGeneration;
    detector.Feed(ClearSample(t)); t += 2000;
    Require(detector.GetTelemetry().faultRecoverySamples == 1,
            "fault recovery qualification did not begin");

    detector.ResumeAfterMotion("test motion");
    Require(detector.GetTelemetry().faultRecoverySamples == 0 &&
            detector.GetTelemetry().referenceGeneration == faultGeneration,
            "motion resume retained unsafe fault qualification state");

    auto changed = ClearSample(t);
    changed.gain = 20;
    detector.Feed(changed); t += 2000;
    Require(detector.GetTelemetry().faultRecoverySamples == 0,
            "acquisition change counted toward fault recovery");
    changed.tMs = t;
    detector.Feed(changed); t += 2000;
    Require(detector.GetTelemetry().faultRecoverySamples == 1,
            "fresh qualification did not restart after acquisition change");
}

void ReplayLogReproducesDetector()
{
    CloudDetector original, replay;
    std::vector<std::string> records;
    std::vector<SceneTelemetry> expected;
    original.SetLogger([&](const std::string& line) {
        if (line.find("cloud: replay ") == 0)
            records.push_back(line);
    });
    expected.push_back(original.GetTelemetry());
    auto step = [&](const std::function<void()>& action) {
        const size_t before = records.size();
        action();
        Require(records.size() == before + 1, "replay must record exactly one external operation");
        expected.push_back(original.GetTelemetry());
    };
    step([&] { original.Reset("replay test"); });
    int64_t t = 1000;
    for (int i = 0; i < 220; ++i, t += 3400) {
        if (i == 60) step([&] { original.ResumeAfterMotion("dither"); });
        if (i == 100) step([&] { original.StartNewSegment("next target"); });
        if (i == 90) step([&] { original.SetSensitivityPct(73); });
        if (i == 130) step([&] { original.SetEnabled(false); });
        if (i == 134) step([&] { original.SetEnabled(true); });
        if (i == 170) step([&] { original.ReportFault("test", "transient"); });
        auto s = ClearSample(t);
        s.mass = i >= 30 && i < 90 ? 3.1234567f : 100.123459f;
        s.snr = 20.1234567f;
        s.score = -3.1234567f;
        s.brightCeil = i >= 30 && i < 90 ? 20.1234567f : 100.123459f;
        s.ensembleRatio = i >= 30 && i < 90 ? 0.05f : 0.987654328f;
        s.ensembleStars = 4;

        s.cloudConfigGeneration = i >= 120 ? 2 : 1;
        s.ensembleTripRatio = 0.765432119f;
        s.roiX = 11; s.roiY = 17; s.roiW = 99; s.roiH = 101;
        s.sourceGen = i >= 190 ? 18 : 17;
        if (i == 50) { s.detected = false; s.stableLock = false; s.brightCeil = 0.f; }
        if (i == 51) s.brightCeil = -1.f;
        if (i == 52) s.tMs -= 3400;
        step([&] { original.Feed(s); });
    }
    uint64_t sequence = 0;
    bool sawObscured = false, sawFault = false;
    for (size_t i = 0; i < records.size(); ++i) {
        std::istringstream line(records[i]);
        line.imbue(std::locale::classic());
        std::map<std::string, std::string> fields;
        std::string token;
        while (line >> token) {
            const auto split = token.find('=');
            if (split != std::string::npos)
                fields[token.substr(0, split)] = token.substr(split + 1);
        }
        auto integer = [&](const char* key) { return std::stoll(fields.at(key)); };
        auto scalar = [&](const char* key) {
            std::istringstream value(fields.at(key));
            value.imbue(std::locale::classic());
            float result = 0.f;
            value >> result;
            Require(!value.fail(), "invalid replay scalar");
            return result;
        };
        Require(integer("v") == 1 && (uint64_t) integer("seq") == ++sequence,
                "replay version or operation sequence changed");
        const auto& event = fields.at("event");
        if (event == "attach") {
            replay.SetEnabled(integer("enabled") != 0);
            replay.SetSensitivityPct((int) integer("sensitivity"));
        }
        else if (event == "reset") replay.Reset("replay");
        else if (event == "segment") replay.StartNewSegment("replay");
        else if (event == "resume") replay.ResumeAfterMotion("replay");
        else if (event == "enabled") replay.SetEnabled(integer("enabled") != 0);
        else if (event == "sensitivity") replay.SetSensitivityPct((int) integer("sensitivity"));
        else if (event == "fault") replay.ReportFault("replay", "fault");
        else {
            Require(event == "feed", "unknown replay operation");
            SceneSample s;
            s.tMs = integer("tMs");
            s.detected = integer("detected") != 0;
            s.stableLock = integer("stableLock") != 0;
            s.score = scalar("score"); s.snr = scalar("snr"); s.mass = scalar("mass");
            s.features = (int) integer("features");
            s.ensembleRatio = scalar("ensembleRatio");
            s.ensembleStars = (int) integer("ensembleStars");
            s.ensembleTripRatio = scalar("ensembleTripRatio");

            s.cloudConfigGeneration = (unsigned) integer("cloudConfigGeneration");

            s.brightCeil = scalar("brightCeil");
            s.brightExposureMs = (int) integer("brightExposureMs");
            s.exposureMs = (int) integer("exposureMs");
            s.gain = (int) integer("gain"); s.bitDepth = (int) integer("bitDepth");
            s.frameW = (int) integer("frameW"); s.frameH = (int) integer("frameH");
            s.roiX = (int) integer("roiX"); s.roiY = (int) integer("roiY");
            s.roiW = (int) integer("roiW"); s.roiH = (int) integer("roiH");
            s.sourceGen = (unsigned) integer("sourceGen"); s.mode = (int) integer("mode");
            Require(s.snr == 20.1234567f && s.score == -3.1234567f &&
                    (s.ensembleRatio == 0.987654328f || s.ensembleRatio == 0.05f) && s.ensembleTripRatio == 0.765432119f,
                    "replay rounded photometric inputs");
            replay.Feed(s);
        }
        const auto actual = replay.GetTelemetry();
        const auto& want = expected[i];
        Require(actual.state == want.state && actual.severity == want.severity &&
                actual.massDeclineRate == want.massDeclineRate && actual.massDeclineRatio == want.massDeclineRatio &&
                actual.massDeclineLatched == want.massDeclineLatched && actual.massDeclineUsesEnsemble == want.massDeclineUsesEnsemble &&
                actual.massRatio == want.massRatio && actual.brightRatio == want.brightRatio &&
                actual.snrDropDb == want.snrDropDb && actual.scoreDelta == want.scoreDelta &&
                actual.massScatterFactor == want.massScatterFactor && actual.snrScatterFactor == want.snrScatterFactor &&
                actual.referenceGeneration == want.referenceGeneration && actual.healthy == want.healthy &&
                actual.recoverySettled == want.recoverySettled && actual.clearingTrend == want.clearingTrend &&
                actual.stateSinceMs == want.stateSinceMs && actual.fresh == want.fresh &&
                actual.faultAutoRetries == want.faultAutoRetries && actual.lossRun == want.lossRun,
                "replayed detector diverged from recorded run");
        sawObscured = sawObscured || actual.state == SceneState::Obscured;
        sawFault = sawFault || !actual.healthy;
    }
    Require(sawObscured && sawFault, "replay did not exercise cloud and fault recovery");
}




void UnconfirmedSlopeSurvivesAcquisitionAndMotionSegments()
{
    CloudDetector detector;
    int64_t t = 1000;
    for (int segment = 0; segment < 3; ++segment) {
        if (segment % 2 == 0) detector.StartNewSegment("next target");
        else detector.ResumeAfterMotion("dither settled");
        t += 600000;
        for (int i = 0; i < 25; ++i, t += 4000) {
            auto s = ClearSample(t);
            const float scale = segment < 2 ? 100.f : segment < 4 ? 1000.f : 300.f;
            const float start = segment % 2 == 0 ? 1.f : .952f;
            s.mass = scale * start * (1.f - i * .002f);
            s.exposureMs = segment < 2 ? 1000 : 2000;
            s.cloudConfigGeneration = segment / 2;
            detector.Feed(s);
            Require(detector.GetState() != SceneState::Obscured, "short shallow segments implied total loss");
        }
    }
    const auto result = detector.GetTelemetry();
    Require(result.massDeclineLatched && result.state == SceneState::Suspect && result.severity > .1f,
            "acquisition warmups and dithers erased unconfirmed joined slope evidence");
    auto mode = ClearSample(t); mode.mode = 1;
    detector.Feed(mode);
    Require(!detector.GetTelemetry().massDeclineLatched, "non-stellar mode reused stellar transparency");
}

void ShortSlopeSegmentsFormOneObservedTrend()
{
    for (bool multi : {false, true}) {
        MassDeclineDetector decline;
        int64_t t = 1000;
        const float scales[] = {100.f, 3000.f, .4f};
        for (float scale : scales) {
            decline.resume(true);
            t += 900000;
            for (int i = 0; i < 25; ++i, t += 4000) {
                const float mass = scale * (1.f - i * .002f);
                decline.update(t, 1.f, 0, multi ? 100.f : mass, multi ? mass : -1.f);
            }
        }
        Require(decline.latched && decline.ratio < .9f && decline.ratio > .84f,
                "short declining segments did not accumulate across target joins");
        Require(decline.rate > 2.f && decline.rate < 4.f,
                "unobserved wall time diluted or inflated the stitched slope");
    }
}

void SameAcquisitionLossCompoundsAndRecoveryReducesIt()
{
    MassDeclineDetector decline;
    int64_t t = 1000;
    auto feed = [&](float value) { decline.update(t, 1.f, 0, 100.f, value); t += 4000; };
    for (int i = 0; i < 65; ++i) feed(1.f);
    for (int i = 0; i <= 60; ++i) feed(1.f - i / 600.f);
    for (int i = 0; i < 3; ++i) feed(.9f);
    Require(decline.latched && std::fabs(decline.ratio - .9f) < .005f, "first measured loss");
    decline.resume(); t += 3600000;
    for (int i = 0; i < 20; ++i) feed(.9f);
    for (int i = 0; i <= 60; ++i) feed(.9f - i * .0015f);
    for (int i = 0; i < 3; ++i) feed(.81f);
    Require(std::fabs(decline.ratio - .81f) < .005f, "comparable ten-percent losses must compound to nineteen percent");
    decline.resume(); t += 600000;
    for (int i = 0; i < 3; ++i) feed(1.f);
    Require(decline.recoveryAllowed() && std::fabs(decline.ratio - 1.f) < .005f,
            "recovery while unobserved was normalized away");
    decline.resume(true);
    for (int i = 0; i < 30; ++i, t += 4000) decline.update(t, 1.f, 1, 500.f, -1.f);
    Require(!decline.usesEnsemble && !decline.latched && decline.ratio < 0.f,
            "replacement source published incomparable confirmed loss");
}

void JoinsPreservePendingConfirmationWithoutCountingGaps()
{
    MassDeclineDetector decline;
    int64_t t = 1000;
    for (int i = 0; i < 51; ++i, t += 4000) decline.update(t, 1.f, 0, 100.f - i * .15f, -1.f);
    Require(decline.rate > 1.f && !decline.latched, "test needs an unconfirmed eligible slope");
    decline.resume(true); t += 3600000;
    for (int i = 0; i < 3; ++i, t += 4000) decline.update(t, 1.f, 1, 1000.f - i * 1.5f, -1.f);
    Require(!decline.latched, "gap or joining samples completed confirmation without observed time");
    for (int i = 3; i < 14; ++i, t += 4000) decline.update(t, 1.f, 1, 1000.f - i * 1.5f, -1.f);
    Require(decline.latched, "joining erased the pending slope and confirmation evidence");
}

void FlatSegmentsAndInvalidJoinSamplesCannotCreateSlope()
{
    MassDeclineDetector decline;
    int64_t t = 1000;
    for (int segment = 0; segment < 20; ++segment) {
        decline.resume(true); t += 600000;
        const float value = segment % 2 ? .001f : 100000.f;
        for (int i = 0; i < 22; ++i, t += 4000) {
            const float mass = i == 0 ? value * 100.f : value;
            decline.update(t, 1.f, segment, mass, -1.f);
            for (int duplicate = 0; duplicate < 8; ++duplicate)
                decline.update(t, 1.f, segment, mass, -1.f);
            Require(!decline.latched, "flat joins, an initial outlier or duplicates fabricated a slope");
        }
    }
    Require(decline.rate >= 0.f && decline.rate < .001f, "normalized flat segments did not produce a flat virtual slope");
}

void AcquisitionChangesRequalifyConfirmedHaze()
{
    CloudDetector detector;
    int64_t t = Arm(detector);
    auto feed = [&](float mass, int exposure, unsigned generation = 0) {
        auto s = ClearSample(t); s.mass = mass; s.exposureMs = exposure; s.cloudConfigGeneration = generation;
        detector.Feed(s); t += 4000;
    };
    for (int i = 0; i < 65; ++i) feed(100.f, 1000);
    for (int i = 0; i <= 60; ++i) feed(100.f - i / 6.f, 1000);
    for (int i = 0; i < 3; ++i) feed(90.f, 1000);
    const float before = detector.GetTelemetry().severity;
    Require(before > .09f && before < .11f, "test did not accumulate initial haze");
    feed(180.f, 2000);
    Require(detector.GetState() == SceneState::Warmup && !detector.GetTelemetry().massDeclineLatched &&
            detector.GetTelemetry().massDeclineRatio < 0.f, "exposure change published incomparable haze");
    for (int i = 0; i < 40; ++i) feed(180.f, 2000);
    Require(detector.GetState() == SceneState::Clear && detector.GetTelemetry().severity == 0.f,
            "old confirmed haze survived fresh exposure qualification");
    detector.StartNewSegment("guiding restarted on next target"); t += 600000;
    for (int i = 0; i < 40; ++i) feed(7000.f, 2000, 1);
    Require(detector.GetState() == SceneState::Clear && detector.GetTelemetry().severity == 0.f,
            "old confirmed haze held a newly qualified target");
    for (int i = 0; i <= 60; ++i) feed(7000.f - i * (700.f / 60.f), 2000, 1);
    for (int i = 0; i < 3; ++i) feed(6300.f, 2000, 1);
    Require(detector.GetTelemetry().massDeclineLatched && std::fabs(detector.GetTelemetry().severity - .1f) < .01f,
            "fresh decline did not qualify in the new target");
    detector.SetEnabled(false); detector.SetEnabled(true);
    Require(!detector.GetTelemetry().massDeclineLatched, "explicit detector reset retained the old transparency history");
}


void BlackoutRecoveryDuringGapReleasesObscured()
{
    for (bool multi : {false, true}) {
        CloudDetector detector;
        int64_t t = Arm(detector);
        auto feed = [&](float level, bool available = true) {
            auto sample = ClearSample(t);
            sample.mass = multi ? 100.f : level;
            sample.ensembleRatio = multi && available ? level / 100.f : -1.f;
            sample.ensembleStars = multi && available ? 3 : 0;
            sample.detected = sample.stableLock = multi || available;
            detector.Feed(sample); t += 4000;
        };
        for (int i = 0; i < 65; ++i) feed(100.f);
        for (int i = 0; i <= 300; ++i) feed(100.f - 95.f * i / 300.f);
        for (int i = 0; i < 10; ++i) feed(5.f);
        Require(detector.GetState() == SceneState::Obscured && detector.GetTelemetry().massDeclineLatched,
                "blackout recovery test did not qualify obscuration");
        for (int i = 0; i < 30; ++i) feed(100.f, false);
        Require(!detector.GetTelemetry().fresh, "missing source published fresh haze");
        for (int i = 0; i < 45; ++i) feed(100.f);
        Require(detector.GetState() == SceneState::Clear && !detector.GetTelemetry().massDeclineLatched,
                "clear sky after dropout remained Obscured beyond three minutes");
    }
}

void HealthyPhotometryCanReleaseAnObsoleteDeclineReference()
{
    for (float plateau : {1.f, .88f}) {
        CloudDetector detector;
        int64_t t = Arm(detector);
        auto feed = [&](float level) {
            auto sample = ClearSample(t); sample.ensembleRatio = level; sample.ensembleStars = 3;
            detector.Feed(sample); t += 4000;
        };
        for (int i = 0; i < 65; ++i) feed(1.08f);
        for (int i = 0; i <= 100; ++i) feed(1.08f - (1.08f - plateau) * i / 100.f);
        Require(detector.GetTelemetry().massDeclineLatched, "test did not establish a decline reference");
        for (int i = 0; i < 180; ++i) feed(plateau);
        if (plateau == 1.f)
            Require(detector.GetState() == SceneState::Clear && !detector.GetTelemetry().massDeclineLatched,
                    "healthy photometry could not release an obsolete high reference");
        else
            Require(detector.GetState() == SceneState::Suspect && detector.GetTelemetry().massDeclineLatched,
                    "bounded recovery normalized away a genuinely degraded plateau");
    }
}

void NoisyFlatNightsWithDropoutsDoNotAccumulateHaze()
{
    for (double sigma : {.03, .06}) {
        double declineMinutes = 0., hazeMinutes = 0.;
        for (int night = 0; night < 10; ++night) {
            std::mt19937 rng(5000u + night);
            std::normal_distribution<double> noise(0., sigma);
            std::uniform_real_distribution<double> random(0., 1.);
            CloudDetector detector;
            for (int64_t t = 1000; t < 8LL * 3600000; t += 2000) {
                auto sample = ClearSample(t);
                sample.detected = sample.stableLock = random(rng) >= (sigma == .03 ? 1./60 : 1./300);
                sample.mass = (float) (100. * std::exp(noise(rng)));
                detector.Feed(sample);
                const auto result = detector.GetTelemetry();
                if (result.massDeclineLatched && result.state != SceneState::Clear) declineMinutes += 1./30;
                if (result.fresh && result.state != SceneState::Warmup && result.severity >= .3f) hazeMinutes += 1./30;
            }
        }
        Require(sigma != .03 || declineMinutes <= 50., "3% flat-sky noise caused prolonged decline latches");
        Require(hazeMinutes <= (sigma == .03 ? 0. : 10.), "flat-sky dropouts manufactured actionable haze");
    }
}

void SustainedMassDeclineAccumulatesHaze()
{
    for (bool multi : {false, true}) {
        CloudDetector detector;
        int64_t t = Arm(detector);
        auto feed = [&](float level, bool valid = true) {
            auto sample = ClearSample(t);
            // Fixed brightness and SNR deliberately cannot corroborate the legacy level trip.
            sample.mass = multi ? 100.f : level;
            sample.ensembleRatio = multi && valid ? level / 100.f : -1.f;
            sample.ensembleStars = multi && valid ? 3 : 0;
            detector.Feed(sample); t += 4000;
        };
        for (int i = 0; i < 50; ++i) feed(100.f);
        for (int i = 0; i < 100; ++i) feed(100.f - i * 0.12f);
        Require(detector.GetState() == SceneState::Suspect && detector.GetTelemetry().massDeclineLatched,
                "sustained shallow decline must report Suspect");
        Require(detector.GetTelemetry().massDeclineUsesEnsemble == multi, "wrong decline evidence source");
        for (int i = 0; i < 400; ++i) feed(88.f);
        Require(detector.GetState() == SceneState::Suspect,
                "degraded plateau erased accumulated haze");
        detector.ResumeAfterMotion("dither");
        Require(detector.GetTelemetry().massDeclineLatched, "dither erased confirmed decline hold");
        if (multi) {
            for (int i = 0; i < 100; ++i) feed(100.f, false);
            Require(detector.GetState() == SceneState::Suspect,
                    "missing ensemble was replaced by a healthy primary during recovery");
        }
        for (int i = 0; i < 50; ++i) feed(100.f);
        Require(detector.GetState() == SceneState::Clear && !detector.GetTelemetry().massDeclineLatched,
                "restored settled mass did not release decline hold");
    }
}

void MassDeclineRejectsNoiseStepsAndDiscontinuities()
{
    MassDeclineDetector decline;
    int64_t t = 1000;
    for (int i = 0; i < 400; ++i, t += 3500) {
        const float mass = i % 41 == 0 ? 60.f : 100.f + (i % 7 - 3) * 0.7f;
        decline.update(t, 1.f, 0, mass, -1.f);
        Require(!decline.latched, "flat noise/outliers caused decline trip");
    }
    for (int i = 0; i < 120; ++i, t += 3500) {
        decline.update(t, 1.f, 0, 92.f, -1.f);
        Require(!decline.latched, "isolated mass step caused slope trip");
    }
    decline.reset();
    for (int i = 0; i < 140; ++i, t += 3500) {
        decline.update(t, 1.f, 0, 100.f - i * 0.015f, -1.f);
        Require(!decline.latched, "subthreshold decline tripped");
    }
    decline.reset();
    for (int i = 0; i < 100; ++i, t += 3500) {
        if (i == 40) decline.resume();
        decline.update(t, 1.f, 0, i < 40 ? 100.f : 92.f, -1.f);
        Require(!decline.latched, "dither step contaminated trend history");
    }
    decline.reset();
    for (int i = 0; i < 120; ++i, t += 3500) {
        decline.update(t, 1.f, 0, 100.f - i * 0.12f, 1.f);
        Require(!decline.latched, "primary-star anomaly overrode stable multi-star evidence");
    }
}

void MassDeclineSettingsAndIdentityJoinEvidence()
{
    CloudDetector detector;
    int64_t t = Arm(detector);
    for (int i = 0; i < 130; ++i, t += 3500) {
        auto s = ClearSample(t);  s.mass = 100.f - i * .1f;
        detector.Feed(s);
    }
    Require(detector.GetTelemetry().massDeclineLatched, "test did not arm decline hold");
    auto s = ClearSample(t);  s.mass = 85.f; s.exposureMs = 2000;
    detector.Feed(s);
    Require(detector.GetState() == SceneState::Warmup && !detector.GetTelemetry().massDeclineLatched &&
            detector.GetTelemetry().massDeclineRatio < 0.f,
            "new acquisition reused incomparable confirmed loss");
    MassDeclineDetector decline;
    for (int i = 0; i < 130; ++i, t += 3500) decline.update(t, 1.f, 0, 100.f - i * .1f, -1.f);
    Require(decline.latched, "rate component did not latch");
    decline.update(t, 1.f, 1, 85.f, -1.f);
    Require(!decline.latched && decline.ratio < 0, "configuration edit reused incomparable confirmed loss");
    for (float disabled : {0.f, -1.f, std::numeric_limits<float>::infinity()}) {
        for (int i = 0; i < 130; ++i, t += 3500) decline.update(t, disabled, 1, 100.f - i * .1f, -1.f);
        Require(!decline.latched, "disabled/invalid decline setting tripped");
    }
}

void MassDeclineHandlesUnavailableAndDuplicateSamples()
{
    CloudDetector detector;
    int64_t t = Arm(detector);
    for (int i = 0; i < 120; ++i, t += 3500) {
        auto s = ClearSample(t);  s.mass = 100.f - i * .1f;
        if (i % 2 == 0) s.stableLock = false;
        detector.Feed(s);
        for (int j = 0; j < 5; ++j) detector.Feed(s);
        Require(!detector.GetTelemetry().massDeclineLatched, "duplicates or unstable evidence filled decline window");
    }
    detector.Reset("new session");
    for (int i = 0; i < 150; ++i, t += 3500) {
        auto s = ClearSample(t);
        // Auto-exposure input mass is already exposure-normalized by the integration.
        s.exposureMs = 0; s.brightExposureMs = i % 2 ? 2000 : 1000;
        s.brightCeil *= s.brightExposureMs / 1000.f;
        s.snr = i % 2 ? 20.f : 17.f;
        detector.Feed(s);
        Require(!detector.GetTelemetry().massDeclineLatched, "auto-exposure/SNR change tripped mass decline");
    }
}

void HazeTracksMeasuredLossAndRecovery()
{
    for (bool multi : { false, true }) {
        CloudDetector detector;
        int64_t t = Arm(detector);
        auto feed = [&](float level, bool available = true) {
            auto s = ClearSample(t);

            s.mass = multi ? 100.f : level;
            s.ensembleStars = multi && available ? 3 : 0;
            s.ensembleRatio = multi && available ? level / 100.f : -1.f;
            detector.Feed(s); t += 4000;
        };
        for (int i = 0; i < 50; ++i) feed(100.f);
        for (int i = 0; i < 100; ++i) feed(100.f - i * .12f);
        auto warning = detector.GetTelemetry();
        Require(warning.state == SceneState::Suspect && warning.massDeclineLatched,
                "shallow sustained fade did not start haze tracking");
        Require(warning.severity > .10f && warning.severity < .14f,
                "accumulated haze did not match measured mass loss");
        for (int i = 0; i < 500; ++i) feed(88.f);
        auto plateau = detector.GetTelemetry();
        Require(plateau.state == SceneState::Suspect && std::fabs(plateau.severity - warning.severity) < .01f,
                "flat degraded mass accumulated invented haze or lost its reference");
        for (int i = 0; i < 100; ++i) feed(88.f - i * .6f);
        auto deep = detector.GetTelemetry();
        Require(deep.state == SceneState::Suspect && deep.severity > .68f && deep.severity < .75f,
                "continuing slope did not accumulate proportional haze");
        for (int i = 0; i < 60; ++i) feed(5.f);
        Require(detector.GetState() == SceneState::Obscured && detector.GetTelemetry().severity > .9f,
                "near-total accumulated loss failed to obscure");
        if (multi) {
            for (int i = 0; i < 60; ++i) feed(100.f, false);
            Require(!detector.GetTelemetry().fresh && detector.GetState() == SceneState::Obscured,
                    "missing triggering ensemble falsely certified recovery");
        }
        for (int i = 0; i < 20; ++i) feed(60.f);
        auto recovering = detector.GetTelemetry();
        Require(recovering.state == SceneState::Suspect && recovering.severity > .37f && recovering.severity < .42f,
                "returning signal did not reduce haze and release Obscured to Suspect");
        for (int i = 0; i < 100; ++i) feed(100.f);
        Require(detector.GetState() == SceneState::Clear && !detector.GetTelemetry().massDeclineLatched,
                "restored signal did not complete clear qualification");
    }
}

void ObscuredRequiresHighAttenuationAtEverySensitivity()
{
    for (int sensitivity : { 0, 50, 100 }) {
        for (bool multi : { false, true }) {
            CloudDetector detector;
            detector.SetSensitivityPct(sensitivity);
            int64_t t = Arm(detector);
            auto feed = [&](float mass, float ensemble) {
                auto s = ClearSample(t);
                s.mass = mass;
                s.ensembleStars = multi ? 3 : 0;
                s.ensembleRatio = multi ? ensemble : -1.f;
                s.brightCeil = 50.f;
                s.snr = 10.f;
                s.score = -6.f;
                detector.Feed(s); t += 2000;
            };
            for (int i = 0; i < 100; ++i) feed(40.f, .40f);
            Require(detector.GetState() == SceneState::Suspect && detector.GetTelemetry().severity < .65f,
                    "partial loss with multiple bad channels escalated to Obscured");
            if (multi) {
                for (int i = 0; i < 20; ++i) feed(1.f, 1.f);
                Require(detector.GetState() != SceneState::Obscured,
                        "single-star anomaly overrode a healthy measured ensemble");
            }
            for (int i = 0; i < 3; ++i) feed(5.f, .05f);
            Require(detector.GetState() == SceneState::Obscured,
                    "severe loss did not obscure at this sensitivity");
        }
    }
}

void VariabilityCannotImplyTotalLoss()
{
    CloudDetector detector;
    int64_t t = Arm(detector);
    for (int i = 0; i < 300; ++i, t += 2000) {
        auto s = ClearSample(t);
        s.mass = i % 3 == 0 ? 1.f : i % 3 == 1 ? 100.f : 199.f;
        s.snr = i % 3 == 0 ? 1.f : i % 3 == 1 ? 20.f : 39.f;
        s.score = -8.f;
        s.brightCeil = 10.f;
        detector.Feed(s);
        Require(detector.GetState() != SceneState::Obscured && detector.GetTelemetry().severity <= .5f,
                "variability or supporting-channel noise was mistaken for total signal loss");
    }
}


} // namespace

void SkyStabilityRequiresSustainedRiseAndPlateau()
{
    for (bool multi : { false, true }) {
        SkyStabilityDetector sky;
        int64_t t = 1000;
        auto feed = [&](float level) {
            sky.update(t, multi ? 100.f : level, multi ? level / 100.f : -1.f, 15000);
            t += 2000;
        };
        for (int i = 0; i < 100; ++i) feed(100.f);
        Require(sky.ready && sky.stable && !sky.improving, "flat startup did not qualify");
        for (int i = 0; i < 200; ++i) feed(100.f + i * .2f);
        Require(sky.ready && sky.improving && !sky.stable, "sustained rise was accepted as settled");
        for (int i = 0; i < 100; ++i) feed(140.f);
        Require(sky.ready && sky.stable && !sky.improving, "plateau failed to qualify");
        sky.reset();
        for (int i = 0; i < 100; ++i) feed(100.f);
        for (int i = 0; i < 120; ++i) {
            feed(120.f);
            Require(!sky.improving, "isolated step became a sustained rise");
        }
        sky.update(t + 30000, multi ? 100.f : 120.f, multi ? 1.2f : -1.f, 15000);
        Require(!sky.ready, "long gap retained stability qualification");
    }
    SkyStabilityDetector sky;
    for (int i = 0; i < 100; ++i) sky.update(1000 + i * 2000, 100.f, -1.f, 15000);
    sky.update(201000, 100.f, 1.f, 15000);
    Require(!sky.ready, "primary-to-ensemble change joined unrelated levels");
    for (int i = 0; i < 100; ++i) sky.update(203000 + i * 2000, 100.f, 1.f, 15000);
    sky.update(403000, 200.f, -1.f, 15000);
    Require(!sky.ready && !sky.improving, "missing ensemble used a primary bridge");
    sky.reset();
    for (int i = 0; i < 30; ++i) sky.update(1000 + i * 20000, 100.f, -1.f, 45000);
    Require(sky.ready && sky.stable, "slow exposures never qualified stability");
    sky.reset();
    unsigned seed = 12345;
    for (int i = 0; i < 24 * 60 * 20; ++i) {
        seed = seed * 1664525u + 1013904223u;
        const float noise = ((seed >> 8) / 16777215.f - .5f) * 3.f;
        sky.update(1000 + i * 3000, 100.f + noise, -1.f, 15000);
        Require(!sky.improving, "ordinary flat-sky noise became a sustained rise");
    }
}

void SkyStabilityDoesNotTreatScatterAsAPlateau()
{
    for (bool multi : {false, true}) {
        SkyStabilityDetector sky;
        for (int i = 0; i <= 90; ++i) {
            const float base = i <= 30 ? 100.f : i <= 60 ? 104.f : 108.f;
            const float mass = base + (i % 3 - 1) * 10.f;
            sky.update(1000 + i * 2000, multi ? 100.f : mass, multi ? mass / 100.f : -1.f, 15000);
        }
        Require(sky.ready && !sky.stable, "scatter hid an eight percent change in level");
        for (int i = 91; i <= 200; ++i) {
            const float mass = 108.f + (i % 3 - 1);
            sky.update(1000 + i * 2000, multi ? 100.f : mass, multi ? mass / 100.f : -1.f, 15000);
        }
        Require(sky.ready && sky.stable, "bounded noise prevented a true plateau from settling");
    }
}

void SkyStabilityRetainsOnlyCompatibleObservedHistory()
{
    for (bool explicitResume : {false, true}) {
        SkyStabilityDetector sky;
        int64_t t = 1000;
        auto feed = [&](float mass = 100.f) { sky.update(t, mass, -1.f, 15000); t += 2000; };
        for (int i = 0; i < 100; ++i) feed();
        Require(sky.ready && sky.stable, "initial stability window incomplete");
        if (explicitResume) sky.resume();
        t += 45000;
        for (int i = 0; i < 15; ++i) {
            feed(); Require(!sky.ready, "gap reused ready flag without fresh reacquisition");
        }
        feed();
        Require(sky.ready && sky.stable, "short same-source gap lost the observed window");
        sky.resume(); t += 20000;
        for (int i = 0; i < 16; ++i) feed(50.f);
        Require(sky.ready && !sky.stable, "gap normalized away an actual brightness change");
        t += 121000;
        for (int i = 0; i < 16; ++i) feed(50.f);
        Require(!sky.ready, "long gap reused stale history");
        sky.reset();
        for (int i = 0; i < 31; ++i) feed();
        sky.resume(); t += 100000;
        for (int i = 0; i < 31; ++i) feed();
        Require(!sky.ready, "missing time completed a partially observed window");
        for (int i = 0; i < 31; ++i) feed();
        Require(sky.ready && sky.stable, "observed segments failed to complete the window");
    }
    CloudDetector detector;
    int64_t t = 1000;
    for (int i = 0; i < 100; ++i, t += 2000) detector.Feed(ClearSample(t));
    detector.ResumeAfterMotion("test dither"); t += 30000;
    for (int i = 0; i < 16; ++i, t += 2000) detector.Feed(ClearSample(t));
    Require(detector.GetTelemetry().skyStable, "integration discarded dither history");
    SceneSample changed = ClearSample(t); changed.gain += 10;
    detector.Feed(changed);
    Require(!detector.GetTelemetry().skyStabilityReady, "acquisition change reused history");
    changed.tMs += 2000; changed.mode = 1;
    detector.Feed(changed);
    Require(!detector.GetTelemetry().skyStabilityApplicable, "non-stellar mode advertised stellar recovery");
}

void SkyStabilityRejectsNoisyFlatNights()
{
    for (double noise : { .03, .06, .10 }) {
        for (unsigned seed = 0; seed < 10; ++seed) {
            std::mt19937 gen(seed);
            std::normal_distribution<double> random(0, noise);
            SkyStabilityDetector sky;
            int consecutive = 0;
            for (int i = 0; i < 8 * 3600 / 3; ++i) {
                sky.update(1000LL + i * 3000, (float)(100 * (1 + random(gen))), -1.f, 15000);
                consecutive = sky.ready && sky.improving ? consecutive + 1 : 0;
                Require(consecutive < 6, "flat noisy telemetry confirmed a clearing event");
            }
        }
    }
}

namespace {

struct PortableNoise {
    std::mt19937 gen;
    explicit PortableNoise(unsigned seed) : gen(seed) {}
    double next() {
        double sum = 0;
        for (int i = 0; i < 4; ++i) sum += gen() / 4294967296.0 - .5;
        return sum * std::sqrt(3.0);
    }
};

bool BlocksAreStable(float a, float b, float c, float ripple)
{
    SkyStabilityDetector sky;
    for (int i = 0; i <= 90; ++i) {
        const float level = i <= 30 ? a : i <= 60 ? b : c;
        sky.update(1000 + i * 2000, level * (1.f + ripple * (i % 3 - 1)), -1.f, 15000);
    }
    Require(sky.ready, "three observed blocks were not ready");
    return sky.stable;
}

void SlowNoisyTrendIsNeverStable(int direction)
{
    SkyStabilityDetector sky;
    for (int i = 0; i < 400; ++i) {
        const double minutes = i < 100 ? 0. : (i - 100) * 2000 / 60000.;
        const double level = 100. * (1 + direction * .015 * minutes) * (1 + .03 * (i % 3 - 1));
        sky.update(1000 + i * 2000LL, (float) level, -1.f, 15000);
        if (i >= 195) Require(sky.ready && !sky.stable, "slow trend inside the widened band was reported stable");
    }
    for (unsigned seed = 0; seed < 20; ++seed) {
        PortableNoise noise(seed + 50);
        SkyStabilityDetector noisy;
        int run = 0;
        for (int i = 0; i < 344; ++i) {
            const double minutes = i < 172 ? 0. : (i - 172) * 3.5 / 60.;
            noisy.update(1000 + i * 3500LL, (float) (100000. * (1 + direction * .03 * minutes) * (1 + .02 * noise.next())),
                         -1.f, 15000);
            if (i < 172 + 52) continue;
            run = noisy.ready && noisy.stable ? run + 1 : 0;
            Require(run * 3500 < 120000, "continuing noisy trend qualified as a stable plateau");
        }
    }
}

}

void SkyStabilityNoisyPlateauQualifiesWithinMinutes()
{
    for (double sigma : { .03, .04 }) {
        int prompt = 0;
        for (unsigned seed = 0; seed < 10; ++seed) {
            PortableNoise noise(seed);
            SkyStabilityDetector sky;
            int run = 0;
            double releasedMin = -1;
            for (int i = 0; releasedMin < 0 && i * 3500LL < 60 * 60000; ++i) {
                const int64_t t = 1000 + i * 3500LL;
                sky.update(t, (float) (100000. * (1 + sigma * noise.next())), -1.f, 15000);
                run = sky.ready && sky.stable ? run + 1 : 0;
                if (run * 3500 >= 120000) releasedMin = t / 60000.;
            }
            Require(releasedMin >= 0, "flat noisy plateau never reached 120 stable seconds within an hour");
            prompt += releasedMin <= 20;
        }
        Require(prompt >= 8, "flat noisy plateau usually took over twenty minutes to qualify");
    }
}

void SkyStabilitySlowNoisyRiseIsNeverStable()
{
    SlowNoisyTrendIsNeverStable(1);
}

void SkyStabilitySlowNoisyFallIsNeverStable()
{
    SlowNoisyTrendIsNeverStable(-1);
}

void SkyStabilityQuietPlateauKeepsTwoPercentBand()
{
    for (float ripple : { 0.f, .01f }) {
        Require(BlocksAreStable(100.f, 100.f, 100.f, ripple), "quiet flat plateau did not qualify");
        Require(BlocksAreStable(100.f, 101.8f, 100.f, ripple), "quiet spread inside two percent did not qualify");
        Require(BlocksAreStable(100.f, 100.9f, 101.8f, ripple), "quiet drift inside two percent did not qualify");
        Require(!BlocksAreStable(100.f, 102.2f, 100.f, ripple), "quiet photometry widened the two percent band");
        Require(!BlocksAreStable(100.f, 101.1f, 102.2f, ripple), "quiet drift beyond two percent qualified");
    }
    Require(BlocksAreStable(100.f, 103.5f, 100.f, .05f), "measured scatter did not widen the band");
    Require(!BlocksAreStable(100.f, 104.5f, 100.f, .10f), "scatter widened the band beyond four percent");
}

void SkyStabilityClearingOverridesWideBand()
{
    int improvingFrames = 0;
    for (double sigma : { .03, .05, .08 }) {
        for (double rate : { .03, .06, .12 }) {
            for (unsigned seed = 0; seed < 10; ++seed) {
                PortableNoise noise(seed + 900);
                SkyStabilityDetector sky;
                for (int i = 0; i < 400; ++i) {
                    const double minutes = i < 100 ? 0. : (i - 100) * 2. / 60.;
                    sky.update(1000 + i * 2000LL, (float) (100000. * (1 + rate * minutes) * (1 + sigma * noise.next())),
                               -1.f, 15000);
                    improvingFrames += sky.improving;
                    Require(!(sky.improving && sky.stable), "clearing was reported stable inside a wide band");
                }
            }
        }
    }
    Require(improvingFrames > 1000, "wide-band ramps never exercised clearing");
}

int main()
{
    int failed = 0;
    try { SkyStabilityNoisyPlateauQualifiesWithinMinutes(); } catch (const std::exception&) { std::cerr << "FAIL SkyStabilityNoisyPlateauQualifiesWithinMinutes\n"; ++failed; }
    try { SkyStabilitySlowNoisyRiseIsNeverStable(); } catch (const std::exception&) { std::cerr << "FAIL SkyStabilitySlowNoisyRiseIsNeverStable\n"; ++failed; }
    try { SkyStabilitySlowNoisyFallIsNeverStable(); } catch (const std::exception&) { std::cerr << "FAIL SkyStabilitySlowNoisyFallIsNeverStable\n"; ++failed; }
    try { SkyStabilityQuietPlateauKeepsTwoPercentBand(); } catch (const std::exception&) { std::cerr << "FAIL SkyStabilityQuietPlateauKeepsTwoPercentBand\n"; ++failed; }
    try { SkyStabilityClearingOverridesWideBand(); } catch (const std::exception&) { std::cerr << "FAIL SkyStabilityClearingOverridesWideBand\n"; ++failed; }
    try { SkyStabilityRetainsOnlyCompatibleObservedHistory(); } catch (const std::exception&) { std::cerr << "FAIL SkyStabilityRetainsOnlyCompatibleObservedHistory\n"; ++failed; }
    try { SkyStabilityDoesNotTreatScatterAsAPlateau(); } catch (const std::exception&) { std::cerr << "FAIL SkyStabilityDoesNotTreatScatterAsAPlateau\n"; ++failed; }
    try { SkyStabilityRejectsNoisyFlatNights(); } catch (const std::exception&) { std::cerr << "FAIL SkyStabilityRejectsNoisyFlatNights\n"; ++failed; }
    try { SkyStabilityRequiresSustainedRiseAndPlateau(); } catch (const std::exception&) { std::cerr << "FAIL SkyStabilityRequiresSustainedRiseAndPlateau\n"; ++failed; }
    try { VariabilityCannotImplyTotalLoss(); } catch (const std::exception&) { std::cerr << "FAIL VariabilityCannotImplyTotalLoss\n"; ++failed; }
    try { ObscuredRequiresHighAttenuationAtEverySensitivity(); } catch (const std::exception&) { std::cerr << "FAIL ObscuredRequiresHighAttenuationAtEverySensitivity\n"; ++failed; }
    try { HazeTracksMeasuredLossAndRecovery(); } catch (const std::exception&) { std::cerr << "FAIL HazeTracksMeasuredLossAndRecovery\n"; ++failed; }
    try { UnconfirmedSlopeSurvivesAcquisitionAndMotionSegments(); } catch (const std::exception&) { std::cerr << "FAIL UnconfirmedSlopeSurvivesAcquisitionAndMotionSegments\n"; ++failed; }
    try { ShortSlopeSegmentsFormOneObservedTrend(); } catch (const std::exception&) { std::cerr << "FAIL ShortSlopeSegmentsFormOneObservedTrend\n"; ++failed; }
    try { SameAcquisitionLossCompoundsAndRecoveryReducesIt(); } catch (const std::exception&) { std::cerr << "FAIL SameAcquisitionLossCompoundsAndRecoveryReducesIt\n"; ++failed; }
    try { JoinsPreservePendingConfirmationWithoutCountingGaps(); } catch (const std::exception&) { std::cerr << "FAIL JoinsPreservePendingConfirmationWithoutCountingGaps\n"; ++failed; }
    try { FlatSegmentsAndInvalidJoinSamplesCannotCreateSlope(); } catch (const std::exception&) { std::cerr << "FAIL FlatSegmentsAndInvalidJoinSamplesCannotCreateSlope\n"; ++failed; }
    try { AcquisitionChangesRequalifyConfirmedHaze(); } catch (const std::exception&) { std::cerr << "FAIL AcquisitionChangesRequalifyConfirmedHaze\n"; ++failed; }
    try { BlackoutRecoveryDuringGapReleasesObscured(); } catch (const std::exception&) { std::cerr << "FAIL BlackoutRecoveryDuringGapReleasesObscured\n"; ++failed; }
    try { HealthyPhotometryCanReleaseAnObsoleteDeclineReference(); } catch (const std::exception&) { std::cerr << "FAIL HealthyPhotometryCanReleaseAnObsoleteDeclineReference\n"; ++failed; }
    try { NoisyFlatNightsWithDropoutsDoNotAccumulateHaze(); } catch (const std::exception&) { std::cerr << "FAIL NoisyFlatNightsWithDropoutsDoNotAccumulateHaze\n"; ++failed; }
    try { SustainedMassDeclineAccumulatesHaze(); } catch (const std::exception&) { std::cerr << "FAIL SustainedMassDeclineAccumulatesHaze\n"; ++failed; }
    try { MassDeclineRejectsNoiseStepsAndDiscontinuities(); } catch (const std::exception&) { std::cerr << "FAIL MassDeclineRejectsNoiseStepsAndDiscontinuities\n"; ++failed; }
    try { MassDeclineSettingsAndIdentityJoinEvidence(); } catch (const std::exception&) { std::cerr << "FAIL MassDeclineSettingsAndIdentityJoinEvidence\n"; ++failed; }
    try { MassDeclineHandlesUnavailableAndDuplicateSamples(); } catch (const std::exception&) { std::cerr << "FAIL MassDeclineHandlesUnavailableAndDuplicateSamples\n"; ++failed; }
    try { ReplayLogReproducesDetector(); } catch (const std::exception&) { std::cerr << "FAIL ReplayLogReproducesDetector\n"; ++failed; }
    try { VariabilityWithSupportingNoiseRemainsSuspect(); } catch (const std::exception&) { std::cerr << "FAIL VariabilityWithSupportingNoiseRemainsSuspect\n"; ++failed; }
    try { FadedPhotometryStillAcceptsSupportingVotes(); } catch (const std::exception&) { std::cerr << "FAIL FadedPhotometryStillAcceptsSupportingVotes\n"; ++failed; }
    try { MissingChannelsCannotRepeatVotes(); } catch (const std::exception&) { std::cerr << "FAIL MissingChannelsCannotRepeatVotes\n"; ++failed; }
    try { GradualClearingWaitsForPlateau(); } catch (const std::exception&) { std::cerr << "FAIL GradualClearingWaitsForPlateau\n"; ++failed; }
    try { AlternateBaselineCannotAcceptContinuingClearing(); } catch (const std::exception&) { std::cerr << "FAIL AlternateBaselineCannotAcceptContinuingClearing\n"; ++failed; }
    try { MissingEvidenceAndGapRestartRecovery(); } catch (const std::exception&) { std::cerr << "FAIL MissingEvidenceAndGapRestartRecovery\n"; ++failed; }
    try { StalledFeedExpiresWithoutErasingVerdict(); } catch (const std::exception&) { std::cerr << "FAIL StalledFeedExpiresWithoutErasingVerdict\n"; ++failed; }
    try { DuplicatesCannotFillWindowsAndOptionalChannelsStayOptional(); } catch (const std::exception&) { std::cerr << "FAIL DuplicatesCannotFillWindowsAndOptionalChannelsStayOptional\n"; ++failed; }
    try { SlowAndIrregularCadencesCanRecover(); } catch (const std::exception&) { std::cerr << "FAIL SlowAndIrregularCadencesCanRecover\n"; ++failed; }
    try { SmallPhotometricNoiseDoesNotBlockPlateau(); } catch (const std::exception&) { std::cerr << "FAIL SmallPhotometricNoiseDoesNotBlockPlateau\n"; ++failed; }
    try { SuspectAcceptsImprovingRecoveryAndEnsembleHistoryIsFresh(); } catch (const std::exception&) { std::cerr << "FAIL SuspectAcceptsImprovingRecoveryAndEnsembleHistoryIsFresh\n"; ++failed; }
    try { PersistentCalculationFaultExhaustsAutomaticRetries(); } catch (const std::exception&) { std::cerr << "FAIL PersistentCalculationFaultExhaustsAutomaticRetries\n"; ++failed; }
    try { NormalGuideCadenceAllowsMovementWaits(); } catch (const std::exception&) { std::cerr << "FAIL NormalGuideCadenceAllowsMovementWaits\n"; ++failed; }
    try { RecoveryRequiresThreeFreshObservations(); } catch (const std::exception&) { std::cerr << "FAIL RecoveryRequiresThreeFreshObservations\n"; ++failed; }
    try { ReferenceGenerationTracksHardResets(); } catch (const std::exception&) { std::cerr << "FAIL ReferenceGenerationTracksHardResets\n"; ++failed; }
    try { StableSingleChannelSuspectRequalifies(); } catch (const std::exception&) { std::cerr << "FAIL StableSingleChannelSuspectRequalifies\n"; ++failed; }
    try { VariableSingleChannelSuspectDoesNotRequalify(); } catch (const std::exception&) { std::cerr << "FAIL VariableSingleChannelSuspectDoesNotRequalify\n"; ++failed; }
    try { ActiveMassVariabilityCannotRequalify(); } catch (const std::exception&) { std::cerr << "FAIL ActiveMassVariabilityCannotRequalify\n"; ++failed; }
    try { VariableEnsembleCannotRequalify(); } catch (const std::exception&) { std::cerr << "FAIL VariableEnsembleCannotRequalify\n"; ++failed; }
    try { ObscuredRecoveryTimersDoNotCancelEachOther(); } catch (const std::exception&) { std::cerr << "FAIL ObscuredRecoveryTimersDoNotCancelEachOther\n"; ++failed; }
    try { AutoExposureDoesNotTreatSnrScalingAsCloud(); } catch (const std::exception&) { std::cerr << "FAIL AutoExposureDoesNotTreatSnrScalingAsCloud\n"; ++failed; }
    try { TransientCalculationFaultRecoversFromFreshSamples(); } catch (const std::exception&) { std::cerr << "FAIL TransientCalculationFaultRecoversFromFreshSamples\n"; ++failed; }
    try { FaultRecoveryQualificationRestartsOnDiscontinuity(); } catch (const std::exception&) { std::cerr << "FAIL FaultRecoveryQualificationRestartsOnDiscontinuity\n"; ++failed; }
    try { ImageContrastHandlesCameraScalesAndHotPixels(); } catch (const std::exception&) { std::cerr << "FAIL ImageContrastHandlesCameraScalesAndHotPixels\n"; ++failed; }
    try { TargetLossIsNotCloud(); } catch (const std::exception&) { std::cerr << "FAIL TargetLossIsNotCloud\n"; ++failed; }
    try { BiasedDarkFrameTripsContrastChannel(); } catch (const std::exception&) { std::cerr << "FAIL BiasedDarkFrameTripsContrastChannel\n"; ++failed; }
    try { DetectedContrastCollapseNeedsCorroboration(); } catch (const std::exception&) { std::cerr << "FAIL DetectedContrastCollapseNeedsCorroboration\n"; ++failed; }
    try { StablePhotometryIgnoresFwhmJitter(); } catch (const std::exception&) { std::cerr << "FAIL StablePhotometryIgnoresFwhmJitter\n"; ++failed; }
    try { HealthyAug31TelemetryStaysClear(); } catch (const std::exception&) { std::cerr << "FAIL HealthyAug31TelemetryStaysClear\n"; ++failed; }
    try { StableMassRippleStaysClear(); } catch (const std::exception&) { std::cerr << "FAIL StableMassRippleStaysClear\n"; ++failed; }
    try { GentleMassStepIsNotErraticCloud(); } catch (const std::exception&) { std::cerr << "FAIL GentleMassStepIsNotErraticCloud\n"; ++failed; }
    try { ModerateMassWavesHaveProportionalHaze(); } catch (const std::exception&) { std::cerr << "FAIL ModerateMassWavesHaveProportionalHaze\n"; ++failed; }
    try { SlowBroadMassWavesPublishHaze(); } catch (const std::exception&) { std::cerr << "FAIL SlowBroadMassWavesPublishHaze\n"; ++failed; }
    try { ModestCorrelatedFadeStartsWithLowHaze(); } catch (const std::exception&) { std::cerr << "FAIL ModestCorrelatedFadeStartsWithLowHaze\n"; ++failed; }
    try { SustainedModestCorrelatedFadeRemainsHaze(); } catch (const std::exception&) { std::cerr << "FAIL SustainedModestCorrelatedFadeRemainsHaze\n"; ++failed; }
    try { DeeperSlowFadeDoesNotJumpToFiftyPercent(); } catch (const std::exception&) { std::cerr << "FAIL DeeperSlowFadeDoesNotJumpToFiftyPercent\n"; ++failed; }
    try { ErraticMassWavesPublishHaze(); } catch (const std::exception&) { std::cerr << "FAIL ErraticMassWavesPublishHaze\n"; ++failed; }
    try { CorroboratedMassAndSnrWavesRemainHaze(); } catch (const std::exception&) { std::cerr << "FAIL CorroboratedMassAndSnrWavesRemainHaze\n"; ++failed; }
    try { StarSnrAndFwhmDriveSustainedVote(); } catch (const std::exception&) { std::cerr << "FAIL StarSnrAndFwhmDriveSustainedVote\n"; ++failed; }
    try { ExposureChangeResetsBaseline(); } catch (const std::exception&) { std::cerr << "FAIL ExposureChangeResetsBaseline\n"; ++failed; }
    try { BitDepthChangeResetsBaseline(); } catch (const std::exception&) { std::cerr << "FAIL BitDepthChangeResetsBaseline\n"; ++failed; }
    try { AutoExposureKeepsContrastOnOneScale(); } catch (const std::exception&) { std::cerr << "FAIL AutoExposureKeepsContrastOnOneScale\n"; ++failed; }
    try { BetterClearViewRaisesProtectedStandardWithoutReset(); } catch (const std::exception&) { std::cerr << "FAIL BetterClearViewRaisesProtectedStandardWithoutReset\n"; ++failed; }
    try { StableNearTotalLossCannotRequalify(); } catch (const std::exception&) { std::cerr << "FAIL StableNearTotalLossCannotRequalify\n"; ++failed; }
    try { VariableObstructionCannotRequalify(); } catch (const std::exception&) { std::cerr << "FAIL VariableObstructionCannotRequalify\n"; ++failed; }
    try { NoisyContrastAndFwhmDoNotBlockRecovery(); } catch (const std::exception&) { std::cerr << "FAIL NoisyContrastAndFwhmDoNotBlockRecovery\n"; ++failed; }
    try { GuidingSessionResetRelearnsCurrentClearView(); } catch (const std::exception&) { std::cerr << "FAIL GuidingSessionResetRelearnsCurrentClearView\n"; ++failed; }
    try { MotionResumeClearsTransientEvidenceButKeepsBaseline(); } catch (const std::exception&) { std::cerr << "FAIL MotionResumeClearsTransientEvidenceButKeepsBaseline\n"; ++failed; }
    try { OptionalMultiStarEvidenceCorroboratesPrimaryFade(); } catch (const std::exception&) { std::cerr << "FAIL OptionalMultiStarEvidenceCorroboratesPrimaryFade\n"; ++failed; }
    try { ThrowingLoggerIsContainedAndDisabled(); } catch (const std::exception&) { std::cerr << "FAIL ThrowingLoggerIsContainedAndDisabled\n"; ++failed; }
    try { NonFiniteSamplesCannotPoisonTelemetry(); } catch (const std::exception&) { std::cerr << "FAIL NonFiniteSamplesCannotPoisonTelemetry\n"; ++failed; }
    try { BackwardTimestampResetsInsteadOfUnderflowing(); } catch (const std::exception&) { std::cerr << "FAIL BackwardTimestampResetsInsteadOfUnderflowing\n"; ++failed; }
    try { ContrastRejectsOverflowingRoi(); } catch (const std::exception&) { std::cerr << "FAIL ContrastRejectsOverflowingRoi\n"; ++failed; }
    try { ReportedIntegrationFaultFailsOpenUntilReset(); } catch (const std::exception&) { std::cerr << "FAIL ReportedIntegrationFaultFailsOpenUntilReset\n"; ++failed; }
    if (failed) return 1;
    std::cout << "cloud detector tests passed\n";
    return 0;
}
