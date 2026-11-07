// Copyright 2026 Future Proof Homes
//
// Compile-time proof that the channel-count constants introduced in app_conf.h
// reproduce the values the 2-mic production firmware used before the change,
// so main.c, the speaker pipeline and platform_init.c generate the same code
// for satellite1_firmware_fixed_delay. Compiled twice: with the production
// 2-mic board definitions and with the four-mic pipeline definitions.

#include "app_conf.h"

#if EXPECT_MICS == 2
/* Before this change every one of these uses was appconfAUDIO_PIPELINE_CHANNELS
 * (= MIC_ARRAY_CONFIG_MIC_COUNT = 2), and lane 1 read frame index 3. */
_Static_assert(appconfAUDIO_PIPELINE_CHANNELS == 2, "production mic count");
_Static_assert(appconfAUDIO_PIPELINE_MIC_CHANNELS == appconfAUDIO_PIPELINE_CHANNELS, "mic gain loop length");
_Static_assert(appconfAUDIO_PIPELINE_REF_CHANNELS == appconfAUDIO_PIPELINE_CHANNELS, "reference frame size / mic offset");
_Static_assert(appconfI2S_FRAME_CHANNELS == appconfAUDIO_PIPELINE_CHANNELS, "I2S frame layout");
_Static_assert(appconfAUDIO_SPK_CHANNELS == appconfAUDIO_PIPELINE_CHANNELS, "speaker pipeline frame");
_Static_assert(appconfAUDIO_PIPELINE_PROC_CHANNELS == 2, "frame_data_t.samples width");
_Static_assert(appconfAUDIO_PIPELINE_OUT_IDX_AGC == 0, "lane 0 source index");
_Static_assert(appconfAUDIO_PIPELINE_OUT_IDX_NS == 3, "lane 1 source index (was the literal 3)");
_Static_assert(3 + appconfARRAY_DIAG_SERVICER_COUNT == 3, "device-control servicer count unchanged");
#elif EXPECT_MICS == 4
_Static_assert(appconfAUDIO_PIPELINE_MIC_CHANNELS == 4, "four-mic capture");
_Static_assert(appconfAUDIO_PIPELINE_REF_CHANNELS == 2, "stereo reference still delivered");
_Static_assert(appconfI2S_FRAME_CHANNELS == 2, "I2S stays stereo");
_Static_assert(appconfAUDIO_PIPELINE_PROC_CHANNELS == 4, "all AEC outputs carried to tile 0");
_Static_assert(appconfAUDIO_PIPELINE_OUT_IDX_NS == 5, "NS audio lives in reference slot 1");
_Static_assert(3 + appconfARRAY_DIAG_SERVICER_COUNT == 4, "diagnostics servicer registered");
#else
#error "set EXPECT_MICS"
#endif

int app_conf_equivalence_checked;
