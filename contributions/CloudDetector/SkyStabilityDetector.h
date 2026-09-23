#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

class SkyStabilityDetector {
    static constexpr int Capacity = 128;
    float values[Capacity] = {};
    int64_t times[Capacity] = {};
    int count = 0, head = 0;
    bool chosen = false, ensemble = false, paused = false;
    int64_t cadenceMs = 2000, recoverySinceMs = 0;
    static float median(float* v, int n) {
        std::sort(v, v + n);
        return v[n / 2];
    }
public:
    bool ready = false, improving = false, stable = false;
    void reset() { *this = SkyStabilityDetector{}; }
    void resume() { paused = true; ready = improving = stable = false; }
    void update(int64_t t, float mass, float multi, int64_t gapLimit) {
        ready = improving = stable = false;
        if (t <= 0) return;
        const bool haveMass = std::isfinite(mass) && mass > 0.f;
        const bool haveMulti = std::isfinite(multi) && multi > 0.f;
        if ((!chosen && (haveMass || haveMulti)) || (chosen && !ensemble && haveMulti)) {
            reset(); chosen = true; ensemble = haveMulti;
        }
        if (!chosen || !(ensemble ? haveMulti : haveMass)) { resume(); return; }
        if (count) {
            const int64_t dt = t - times[(head + Capacity - 1) % Capacity];
            if (dt < 0 || dt > std::max<int64_t>(120000, gapLimit)) {
                count = head = 0; recoverySinceMs = 0;
            } else if (paused || dt > gapLimit) {
                for (int i = 0; i < count; ++i)
                    times[(head + Capacity - 1 - i) % Capacity] += dt;
                recoverySinceMs = t;
            } else if (dt < 2000) return;
            else cadenceMs = dt;
        }
        paused = false;
        values[head] = ensemble ? multi : mass; times[head] = t;
        head = (head + 1) % Capacity; count = std::min(count + 1, Capacity);
        if (count < 2) return;
        if (recoverySinceMs && t - recoverySinceMs < std::max<int64_t>(30000, 6 * cadenceMs)) return;
        const int64_t blockMs = std::max<int64_t>(60000, 6 * cadenceMs);
        float blocks[3][Capacity]; int sizes[3] = {};
        bool full = false;
        for (int i = 0; i < count; ++i) {
            const int index = (head + Capacity - 1 - i) % Capacity;
            const int64_t age = t - times[index];
            const int b = age < blockMs ? 2 : age < 2 * blockMs ? 1 : 0;
            blocks[b][sizes[b]++] = values[index];
            if (age >= 3 * blockMs) { full = true; break; }
        }
        if (!full || sizes[0] < 6 || sizes[1] < 6 || sizes[2] < 6) return;
        float levels[3], scatter[3];
        for (int b = 0; b < 3; ++b) {
            levels[b] = median(blocks[b], sizes[b]);
            for (int i = 0; i < sizes[b]; ++i) blocks[b][i] = std::fabs(blocks[b][i] - levels[b]);
            scatter[b] = median(blocks[b], sizes[b]);
        }
        const float noise = 8.f * std::max({scatter[0], scatter[1], scatter[2]}) /
            std::sqrt((float) std::min({sizes[0], sizes[1], sizes[2]}));
        const float base = levels[0];
        ready = true;
        improving = levels[2] - base > std::max(base * .03f, noise) &&
            levels[1] - levels[0] > base * .005f && levels[2] - levels[1] > base * .005f;
        const float band = std::max(base * .02f, std::min(base * .04f, noise));
        const float first = levels[1] - levels[0], second = levels[2] - levels[1];
        const bool trend = std::fabs(levels[2] - levels[0]) > base * .02f && first * second > 0.f &&
            std::min(std::fabs(first), std::fabs(second)) > base * .005f;
        stable = !improving && !trend && std::max({levels[0], levels[1], levels[2]}) -
            std::min({levels[0], levels[1], levels[2]}) <= band;
    }
};
