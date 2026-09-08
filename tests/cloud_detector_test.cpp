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

namespace {

void Require(bool condition, const char *message)
{
    if (!condition)
    {
        std::cerr << "cloud detector test failed: " << message << '\n';
        std::exit(1);
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
    Require(telemetry.severity > 0.f && telemetry.severity < 0.10f,
            "a modest correlated fade published disproportionate haze severity");
}

void SustainedModestCorrelatedFadeObscures()
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
    Require(detector.GetState() == SceneState::Obscured,
            "a sustained correlated fade never advanced from haze to obscured");
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
    Require(telemetry.severity > 0.f && telemetry.severity < 0.15f,
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

void CorroboratedMassAndSnrWavesObscure()
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
    Require(detector.GetState() == SceneState::Obscured,
            "corroborated mass/SNR variability did not latch obscured");
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
    Require(detector.GetState() == SceneState::Obscured, "star SNR/FWHM degradation did not latch obscured");
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

void StableAlternateViewRequalifies()
{
    CloudDetector detector;
    int64_t t = Arm(detector);
    const unsigned referenceGeneration = detector.GetTelemetry().referenceGeneration;

    auto feedLower = [&]() {
        SceneSample sample = ClearSample(t);
        sample.mass = 80.f;
        sample.brightCeil = 80.f;
        sample.snr = 16.f;
        sample.score = -4.f;
        detector.Feed(sample);
        t += 2000;
    };

    for (int i = 0; i < 4 * 60 / 2; ++i)
        feedLower();
    Require(detector.GetState() == SceneState::Clear,
            "stable alternate view did not establish a new baseline");
    Require(detector.GetTelemetry().referenceGeneration != referenceGeneration,
            "alternate baseline did not invalidate external clear references");
    Require(detector.GetTelemetry().massRatio > 0.98f,
            "new baseline did not represent the stable alternate view");
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
    Require(detector.GetState() == SceneState::Obscured, "test blackout did not latch obscured");

    bool sawClearRecovery = false;
    for (int i = 0; i < 45; ++i, t += 2000)
    {
        SceneSample sample = ClearSample(t);
        sample.score = -3.12f; // one mildly degraded target-derived channel
        sample.brightCeil = i % 2 == 0 ? 40.f : 160.f;
        detector.Feed(sample);
        if (i < 30)
            Require(detector.GetState() == SceneState::Obscured,
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
        sample.mass = 40.f;
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
    Require(detector.GetState() == SceneState::Obscured,
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
        Require(detector.GetState() == SceneState::Obscured, "ramp setup failed to obscure");
        for (int elapsed = 0; elapsed < duration; elapsed += 2, t += 2000) {
            SceneSample s = ClearSample(t);
            const float progress = static_cast<float>(elapsed) / duration;
            s.mass = 55.f + 60.f * progress;
            s.snr = 16.f + 7.f * progress;
            detector.Feed(s);
            Require(detector.GetState() == SceneState::Obscured,
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
    Require(detector.GetState() == SceneState::Clear, "settled alternate baseline could not requalify");
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
        Require(detector.GetTelemetry().recoverySettled && detector.GetState() == SceneState::Obscured,
                "expected plateau qualification before the recovery hold completes");
        if (gap) {
            t += 120000;
        } else {
            for (int i = 0; i < 15; ++i, t += 2000) {
                auto s = ClearSample(t);
                s.mass = s.snr = std::numeric_limits<float>::quiet_NaN();
                detector.Feed(s);
                Require(!detector.GetTelemetry().recoverySettled && detector.GetState() == SceneState::Obscured,
                        "missing primary evidence released a hold");
            }
        }
        for (int i = 0; i < 30; ++i, t += 2000) {
            detector.Feed(ClearSample(t));
            Require(detector.GetState() == SceneState::Obscured,
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
        Require(detector.GetState() == SceneState::Obscured, "waiting between exposures prevented blackout detection");
        const int64_t rampStart = t;
        for (; t - rampStart < 180000; t += cycleMs()) {
            auto s = sampleAt(t);
            const float progress = static_cast<float>(t - rampStart) / 180000.f;
            s.mass = 55.f + 60.f * progress;
            s.snr = 16.f + 7.f * progress;
            detector.Feed(s);
            Require(detector.GetState() == SceneState::Obscured,
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
    Require(qualified && detector.GetState() == SceneState::Obscured, "sparse plateau did not start recovery hold");
    for (int i = 0; i < 20; ++i) detector.Feed(sampleAt(t));
    t += 30000;
    detector.Feed(sampleAt(t));
    Require(detector.GetState() == SceneState::Obscured,
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
        s.mass = 30.f;
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

} // namespace

int main()
{
    MissingChannelsCannotRepeatVotes();
    GradualClearingWaitsForPlateau();
    AlternateBaselineCannotAcceptContinuingClearing();
    MissingEvidenceAndGapRestartRecovery();
    StalledFeedExpiresWithoutErasingVerdict();
    DuplicatesCannotFillWindowsAndOptionalChannelsStayOptional();
    SlowAndIrregularCadencesCanRecover();
    SmallPhotometricNoiseDoesNotBlockPlateau();
    SuspectAcceptsImprovingRecoveryAndEnsembleHistoryIsFresh();
    PersistentCalculationFaultExhaustsAutomaticRetries();
    NormalGuideCadenceAllowsMovementWaits();
    RecoveryRequiresThreeFreshObservations();
    ReferenceGenerationTracksHardResets();
    StableSingleChannelSuspectRequalifies();
    VariableSingleChannelSuspectDoesNotRequalify();
    ActiveMassVariabilityCannotRequalify();
    VariableEnsembleCannotRequalify();
    ObscuredRecoveryTimersDoNotCancelEachOther();
    AutoExposureDoesNotTreatSnrScalingAsCloud();
    TransientCalculationFaultRecoversFromFreshSamples();
    FaultRecoveryQualificationRestartsOnDiscontinuity();
    ImageContrastHandlesCameraScalesAndHotPixels();
    TargetLossIsNotCloud();
    BiasedDarkFrameTripsContrastChannel();
    DetectedContrastCollapseNeedsCorroboration();
    StablePhotometryIgnoresFwhmJitter();
    StableMassRippleStaysClear();
    GentleMassStepIsNotErraticCloud();
    ModerateMassWavesHaveProportionalHaze();
    SlowBroadMassWavesPublishHaze();
    ModestCorrelatedFadeStartsWithLowHaze();
    SustainedModestCorrelatedFadeObscures();
    DeeperSlowFadeDoesNotJumpToFiftyPercent();
    ErraticMassWavesPublishHaze();
    CorroboratedMassAndSnrWavesObscure();
    StarSnrAndFwhmDriveSustainedVote();
    ExposureChangeResetsBaseline();
    BitDepthChangeResetsBaseline();
    AutoExposureKeepsContrastOnOneScale();
    BetterClearViewRaisesProtectedStandardWithoutReset();
    StableAlternateViewRequalifies();
    NoisyContrastAndFwhmDoNotBlockRecovery();
    GuidingSessionResetRelearnsCurrentClearView();
    MotionResumeClearsTransientEvidenceButKeepsBaseline();
    OptionalMultiStarEvidenceCorroboratesPrimaryFade();
    ThrowingLoggerIsContainedAndDisabled();
    NonFiniteSamplesCannotPoisonTelemetry();
    BackwardTimestampResetsInsteadOfUnderflowing();
    ContrastRejectsOverflowingRoi();
    ReportedIntegrationFaultFailsOpenUntilReset();
    std::cout << "cloud detector tests passed\n";
    return 0;
}
