// Optional sustained star-mass decline detector. Independent of adapting clear anchors.
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
    } primary, ensemble;
    float threshold = 0.f, reference = -1.f, candidateReference = -1.f;
    unsigned generation = 0;
    int64_t candidateSince = 0;
    bool candidateEnsemble = false;
public:
    float rate = -1.f;       // positive %/minute loss; -1 means the three-minute window is not ready
    float ratio = -1.f;      // current / frozen pre-decline reference while latched
    bool latched = false, usesEnsemble = false;
    void reset() { *this = MassDeclineDetector{}; }
    void resume() {
        primary.clear(); ensemble.clear(); candidateSince = 0;
        rate = ratio = -1.f; // preserve a confirmed hold and its pre-decline reference across dither
    }
    void recovered() { latched = false; reference = ratio = -1.f; candidateSince = 0; }
    bool recoveryAllowed() const { return !latched || ratio >= 0.98f; }
    void update(int64_t t, float requested, unsigned configGeneration, float mass, float multi) {
        if (!std::isfinite(requested) || requested < 0.f || requested > 20.f) requested = 0.f;
        if (requested != threshold || configGeneration != generation) {
            reset(); threshold = requested; generation = configGeneration;
        }
        if (threshold == 0.f) return;
        const bool haveMass = std::isfinite(mass) && mass > 0.f;
        const bool haveMulti = std::isfinite(multi) && multi > 0.f;
        if (haveMass) primary.push(mass, t); else primary.clear();
        if (haveMulti) ensemble.push(multi, t); else ensemble.clear();
        // Never compare a primary-star value with a normalized ensemble reference. A missing
        // triggering channel cannot certify recovery; it must return with fresh measurements.
        const bool useMulti = latched ? usesEnsemble : haveMulti;
        const Window& source = useMulti ? ensemble : primary;
        float before = -1.f;
        bool sustained = false;
        rate = -1.f;
        const bool ready = source.estimate(t, threshold, rate, before, sustained);
        if (!latched) {
            if (!ready || !sustained || rate < threshold) {
                candidateSince = 0;
            } else if (!candidateSince || candidateEnsemble != useMulti) {
                candidateSince = t; candidateReference = before; candidateEnsemble = useMulti;
            } else if (t - candidateSince >= 30000) {
                latched = true; usesEnsemble = useMulti; reference = candidateReference;
            }
        }
        ratio = latched && source.recent() > 0.f ? source.recent() / reference : -1.f;
        if (!latched) usesEnsemble = useMulti;
    }
};
