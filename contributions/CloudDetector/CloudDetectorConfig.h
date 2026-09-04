/*
 * Cloud detector tuning defaults used by the PHD2 integration.
 *
 * These values are the shadow-mode starting points documented in
 * cloud-detection-design.md.  They intentionally live beside the standalone
 * detector so the state machine remains independent of wxWidgets and OpenCV.
 */
#pragma once

#define CONFIG_CLOUD_SHORT_K 3
#define CONFIG_CLOUD_BASELINE_DECIM_MS 2000
#define CONFIG_CLOUD_MIN_BASE_ENTRIES 8
#define CONFIG_CLOUD_WARMUP_MS 20000
#define CONFIG_CLOUD_LOSS_RUN_TRIP 3
#define CONFIG_CLOUD_SUSPECT_QUIET_MS 10000
#define CONFIG_CLOUD_RECOVER_HOLD_MS 10000
#define CONFIG_CLOUD_MEDIUM_WINDOW_MS 25000

#define CONFIG_CLOUD_FAST_MASS_RATIO 0.50f
#define CONFIG_CLOUD_FAST_BRIGHT_RATIO 0.55f
#define CONFIG_CLOUD_MED_MASS_RATIO 0.75f
#define CONFIG_CLOUD_MED_BRIGHT_RATIO 0.80f
#define CONFIG_CLOUD_MED_FEATURE_RATIO 0.60f
#define CONFIG_CLOUD_MED_SNR_DROP_DB 3.0f

#define CONFIG_CLOUD_RECOVER_MARGIN 0.08f
#define CONFIG_CLOUD_RECOVER_CAP 0.98f
#define CONFIG_CLOUD_RECOVER_SNR_MARGIN 0.75f
// A certified clear-sky standard may improve promptly, but can fall only very slowly.  At this
// slew rate a persistent 10-20% reduction takes roughly 30-60 minutes to become the new normal.
#define CONFIG_CLOUD_ANCHOR_DOWN_PCT_MIN 0.33f
#define CONFIG_CLOUD_SLOW_RATIO 0.88f
#define CONFIG_CLOUD_SLOW_DWELL_MS 60000
#define CONFIG_CLOUD_SCORE_BAND_K 3.0f
#define CONFIG_CLOUD_SCORE_BAND_MIN 0.08f
// A clear guide star normally produces a narrow, nearly horizontal mass/SNR trace. Moving cloud
// often appears first as broad waves rather than as a large mean fade, so compare recent robust
// window scatter (MAD) with the learned clear scatter. This remains sensitive when a wave evolves
// over several frames and each individual frame-to-frame change is small. Absolute floors prevent
// a nearly noiseless baseline from magnifying insignificant ripple.
#define CONFIG_CLOUD_VARIABILITY_K 9
#define CONFIG_CLOUD_VARIABILITY_SIGMA_K 4.0f
#define CONFIG_CLOUD_VARIABILITY_RECOVER_FACTOR 0.75f
#define CONFIG_CLOUD_MASS_VARIABILITY_FRAC_MIN 0.03f
#define CONFIG_CLOUD_SNR_VARIABILITY_DB_MIN 0.50f
#define CONFIG_CLOUD_STUCK_MS 600000
#define CONFIG_CLOUD_STATIC_MAD_FRAC 0.05f
#define CONFIG_CLOUD_ALTERNATE_BASELINE_MS 60000
#define CONFIG_CLOUD_ALTERNATE_REL_MAD 0.05f
#define CONFIG_CLOUD_ALTERNATE_SNR_MAD_DB 0.75f
#define CONFIG_CLOUD_ALTERNATE_MIN_SNR_DB 15.0f
#define CONFIG_CLOUD_PERIODIC_LOG_MS 60000
