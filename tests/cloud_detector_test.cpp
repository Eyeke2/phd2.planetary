#include "../contributions/CloudDetector/CloudDetector.h"

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
    for (int i = 0; i < 8; ++i, t += 2000)
    {
        SceneSample sample = ClearSample(t);
        sample.score = -3.12f; // one mildly degraded target-derived channel
        sample.brightCeil = i % 2 == 0 ? 40.f : 160.f;
        detector.Feed(sample);
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

    for (int i = 0; i < 8; ++i, t += 2000)
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

} // namespace

int main()
{
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
