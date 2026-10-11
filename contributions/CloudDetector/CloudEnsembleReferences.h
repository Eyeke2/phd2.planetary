// Per-star clear references for the optional multi-star ensemble ratio. Shared by the PHD2 multi-star
// integration and offline replay, so the learning and selection policy is tested once.
#pragma once
#include "CloudDetector.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

class CloudEnsembleReferences {
    struct Reference {
        int keyX = 0, keyY = 0;
        float anchor = 0.f;
        std::vector<float> warmup;
    };
    std::vector<Reference> references;
    unsigned settingsGeneration = ~0u, referenceGeneration = ~0u;
    Reference& find(int keyX, int keyY) {
        for (Reference& r : references)
            if (r.keyX == keyX && r.keyY == keyY) return r;
        references.push_back(Reference{});
        references.back().keyX = keyX; references.back().keyY = keyY;
        return references.back();
    }
public:
    // Mass per second for a fixed exposure and as MassChecker normalizes it for automatic exposure, at zenith.
    struct Observation { int keyX, keyY; double mass; };
    static constexpr int LearningObservations = 8;
    static float Median(std::vector<float> values) {
        if (values.empty()) return -1.f;
        const size_t middle = values.size() / 2;
        std::nth_element(values.begin(), values.begin() + middle, values.end());
        float result = values[middle];
        if ((values.size() & 1U) == 0) result = .5f * (result + *std::max_element(values.begin(), values.begin() + middle));
        return result;
    }
    static bool MayLearn(const SceneTelemetry& telemetry) {
        return telemetry.state == SceneState::Warmup || telemetry.state == SceneState::Clear;
    }
    // A reference must not capture a transient: on Oct 6 a 30-second dip in the first frames of a target set
    // the references 15% low for the whole target. Eight observations are accepted only when their scatter is
    // small and their two halves agree; otherwise the oldest is dropped and learning continues. The primary star
    // provides the evidence meanwhile, so imaging does not wait for the ensemble.
    static bool Agree(const std::vector<float>& values) {
        const float level = Median(values);
        std::vector<float> deviations;
        for (float v : values) deviations.push_back(std::fabs(v - level));
        const float mad = Median(deviations);
        const auto half = values.begin() + values.size() / 2;
        const float early = Median(std::vector<float>(values.begin(), half));
        const float late = Median(std::vector<float>(half, values.end()));
        // Evenly spaced values differ by exactly 2 MAD between halves, so 1.5 MAD rejects ramps above 4%.
        return mad <= .05f * level && std::fabs(late - early) <= std::max(.02f * level, 1.5f * mad);
    }
    // A new acquisition or settings generation relearns every star.
    void Sync(unsigned settings, unsigned reference) {
        if (settings == settingsGeneration && reference == referenceGeneration) return;
        settingsGeneration = settings; referenceGeneration = reference;
        references.clear();
    }
    // A contained failure keeps the star identities but relearns their references.
    void Invalidate() {
        for (Reference& r : references) { r.anchor = 0.f; r.warmup.clear(); }
    }
    // Lower is preferred: learned references in learning order, then partly learned ones, then unseen stars.
    // Per-frame peaks reorder stars of similar brightness, and a substitute without a reference cannot learn
    // outside Warmup/Clear, so reshuffling left frames without an ensemble ratio.
    size_t Rank(int keyX, int keyY) const {
        for (size_t i = 0; i < references.size(); ++i)
            if (references[i].keyX == keyX && references[i].keyY == keyY)
                return references[i].anchor > 0.f ? i : references.size() + i;
        return 2 * references.size();
    }
    // Median ratio of observed stars with learned references, or -1 with fewer than minStars of them.
    // A reference is the median of eight agreeing observations made while learning is allowed; an observation
    // outside Warmup/Clear restarts that star's learning.
    float Evaluate(const std::vector<Observation>& stars, bool mayLearn, int minStars, int& used) {
        std::vector<float> ratios;
        for (const Observation& star : stars) {
            Reference& r = find(star.keyX, star.keyY);
            if (!std::isfinite(star.mass) || star.mass <= 0.) continue;
            if (r.anchor <= 0.f) {
                if (!mayLearn) r.warmup.clear();
                else {
                    r.warmup.push_back((float) star.mass);
                    if ((int) r.warmup.size() >= LearningObservations) {
                        if (Agree(r.warmup)) { r.anchor = Median(r.warmup); r.warmup.clear(); }
                        else r.warmup.erase(r.warmup.begin());
                    }
                }
            }
            if (r.anchor > 0.f) ratios.push_back((float) (star.mass / r.anchor));
        }
        used = (int) ratios.size();
        return used >= minStars && used > 0 ? Median(std::move(ratios)) : -1.f;
    }
};
