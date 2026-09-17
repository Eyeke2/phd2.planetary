#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

class MassDeclineDetector {
    struct Window {
        static constexpr int Capacity = 128;
        float values[Capacity] = {};
        int64_t times[Capacity] = {};
        int count = 0, head = 0;
        void clear() { count = head = 0; }
        void push(float value, int64_t t) {
            if (count && t - times[(head + Capacity - 1) % Capacity] < 2000) return;
            values[head] = value; times[head] = t;
            head = (head + 1) % Capacity;
            count = std::min(count + 1, Capacity);
        }
        static float median(float* values, int count) {
            std::sort(values, values + count);
            return values[count / 2];
        }
        float recent() const {
            if (count < 3) return -1.f;
            float last[3];
            for (int i = 0; i < 3; ++i) last[i] = values[(head + Capacity - 1 - i) % Capacity];
            return median(last, 3);
        }
        // Three one-minute blocks reject spikes and isolated steps. Actual median sample times
        // determine %/minute, so irregular guide cadence does not change the threshold's meaning.
        bool estimate(int64_t t, float threshold, float& rate, float& reference, bool& sustained) const {
            float blocks[3][Capacity], offsets[3][Capacity];
            int sizes[3] = {};
            bool full = false;
            for (int i = 0; i < count; ++i) {
                const int index = (head + Capacity - 1 - i) % Capacity;
                const int64_t age = t - times[index];
                const int block = age < 60000 ? 2 : age < 120000 ? 1 : 0;
                blocks[block][sizes[block]] = values[index];
                offsets[block][sizes[block]++] = (float) age;
                if (age >= 180000) { full = true; break; }
            }
            if (!full || sizes[0] < 6 || sizes[1] < 6 || sizes[2] < 6) return false;
            float levels[3], ages[3], scatter[3];
            for (int b = 0; b < 3; ++b) {
                levels[b] = median(blocks[b], sizes[b]);
                ages[b] = median(offsets[b], sizes[b]);
                for (int i = 0; i < sizes[b]; ++i) blocks[b][i] = std::fabs(blocks[b][i] - levels[b]);
                scatter[b] = median(blocks[b], sizes[b]);
            }
            reference = levels[0];
            const float minutes = (ages[0] - ages[2]) / 60000.f;
            if (reference <= 0.f || minutes <= 0.f) return false;
            rate = std::max(0.f, 100.f * (levels[0] - levels[2]) / reference / minutes);
            const float noise = 3.f * std::max(scatter[0], scatter[2]) /
                                std::sqrt((float) std::min(sizes[0], sizes[2]));
            sustained = levels[0] - levels[2] > std::max(reference * 0.01f, noise);
            for (int b = 0; b < 2; ++b) {
                const float legMinutes = (ages[b] - ages[b + 1]) / 60000.f;
                sustained = sustained && levels[b] - levels[b + 1] >
                    reference * (threshold / 100.f) * legMinutes * 0.5f;
            }
            return true;
        }
    } history, input;
    float threshold = 0.f, reference = -1.f, candidateReference = -1.f;
    double level = 1., segmentCarry = 1., segmentReference = -1.;
    unsigned generation = 0;
    int64_t observedMs = 1000, candidateSince = 0, lastSampleMs = 0, lastSeenMs = 0, healthySince = 0;
    bool sourceChosen = false;
public:
    float rate = -1.f;       // %/minute of observed time; -1 while unavailable or collecting
    float ratio = -1.f;      // stitched transparency / retained pre-decline transparency
    bool latched = false, usesEnsemble = false;
    void reset() { *this = MassDeclineDetector{}; }
    void resume(bool newSource = false) {
        input.clear(); lastSampleMs = 0;
        if (newSource) {
            healthySince = 0;
            if (latched) {
                history.clear(); observedMs = 1000; candidateSince = 0;
                latched = false; reference = candidateReference = -1.f; level = 1.;
            }
            segmentCarry = level; segmentReference = -1.; sourceChosen = false;
        }
        rate = ratio = -1.f;
    }
    void recovered() { reset(); }
    bool recoveryAllowed() const { return !latched || ratio >= 0.98f; }
    bool releaseIfHealthy(bool healthy, int64_t holdMs) {
        if (!latched) { healthySince = 0; return false; }
        if (ratio < 0.f || rate < 0.f) return false;
        if (!healthy || rate >= threshold) { healthySince = 0; return false; }
        if (!healthySince) healthySince = observedMs;
        if (observedMs - healthySince < holdMs) return false;
        recovered(); return true;
    }
    void update(int64_t t, float requested, unsigned configGeneration, float mass, float multi) {
        if (!std::isfinite(requested) || requested < 0.f || requested > 20.f) requested = 0.f;
        if (requested != threshold || (lastSeenMs && t < lastSeenMs)) {
            reset(); threshold = requested; generation = configGeneration;
        }
        if (threshold == 0.f || t <= 0 || t == lastSeenMs) return;
        lastSeenMs = t;
        if (configGeneration != generation) {
            resume(true); generation = configGeneration;
        }
        const bool haveMass = std::isfinite(mass) && mass > 0.f;
        const bool haveMulti = std::isfinite(multi) && multi > 0.f;
        if ((!sourceChosen && (haveMass || haveMulti)) || (!usesEnsemble && haveMulti)) {
            resume(true); usesEnsemble = haveMulti; sourceChosen = true;
        }
        if (!sourceChosen || !(usesEnsemble ? haveMulti : haveMass)) {
            resume(); return;
        }
        if (lastSampleMs && t - lastSampleMs < 2000) return;
        const int64_t dt = lastSampleMs ? t - lastSampleMs : 0;
        lastSampleMs = t;
        const bool observing = input.count >= 3;
        const float raw = usesEnsemble ? multi : mass;
        input.push(raw, t);
        const float current = input.recent();
        if (current <= 0.f) return;
        if (segmentReference <= 0.) {
            segmentReference = current;
        } else if (observing) {
            observedMs += dt;
        }
        const double next = segmentCarry * (current / segmentReference);
        if (!std::isfinite(next) || next <= 0. || next > 1.e30) { resume(); return; }
        level = next;
        history.push((float) (segmentCarry * (raw / segmentReference)), observedMs);
        float before = -1.f;
        bool sustained = false;
        rate = -1.f;
        const bool ready = history.estimate(observedMs, threshold, rate, before, sustained);
        if (!latched) {
            if (!ready || !sustained || rate < threshold) {
                candidateSince = 0;
            } else if (!candidateSince) {
                candidateSince = observedMs; candidateReference = before;
            } else if (observedMs - candidateSince >= 30000) {
                latched = true; reference = candidateReference;
            }
        }
        ratio = latched ? (float) (level / reference) : -1.f;
    }
};
