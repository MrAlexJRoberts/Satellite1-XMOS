##******************************************
## Create fixed_delay AEC+IC+NS+AGC 
##   2 mic input channels
#    2 reference input channels
##******************************************

add_library(fixed_delay_aec_ic_ns_agc_2mic_2ref INTERFACE)
target_sources(fixed_delay_aec_ic_ns_agc_2mic_2ref
    INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/fixed_delay/audio_pipeline_t0.c
        ${CMAKE_CURRENT_LIST_DIR}/fixed_delay/audio_pipeline_t1.c
        ${CMAKE_CURRENT_LIST_DIR}/fixed_delay/aec/aec_process_frame_1thread.c
)
target_include_directories(fixed_delay_aec_ic_ns_agc_2mic_2ref
    INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/fixed_delay
)
target_link_libraries(fixed_delay_aec_ic_ns_agc_2mic_2ref
    INTERFACE
        voice_compat
        core::general
        rtos::freertos
        rtos::sw_services::generic_pipeline
        fwk_voice::aec
        fwk_voice::agc
        fwk_voice::ic
        fwk_voice::ns
        fwk_voice::vnr
)

##******************************************
## Create ADEC "prevarch" AEC+IC+NS+AGC 
##   2 mic input channels
#    2 reference input channels
##******************************************

add_library(adec_aec_ic_ns_agc_2mic_2ref INTERFACE)
target_sources(adec_aec_ic_ns_agc_2mic_2ref
    INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/adec/audio_pipeline_t0.c
        ${CMAKE_CURRENT_LIST_DIR}/adec/audio_pipeline_t1.c
        ${CMAKE_CURRENT_LIST_DIR}/adec/stage1/delay_buffer.c
        ${CMAKE_CURRENT_LIST_DIR}/adec/stage1/stage_1.c
        ${CMAKE_CURRENT_LIST_DIR}/adec/aec/aec_process_frame_1thread.c
)
target_include_directories(adec_aec_ic_ns_agc_2mic_2ref
    INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/adec
        ${CMAKE_CURRENT_LIST_DIR}/adec/aec
        ${CMAKE_CURRENT_LIST_DIR}/adec/stage1
)
target_link_libraries(adec_aec_ic_ns_agc_2mic_2ref
    INTERFACE
        voice_compat
        core::general
        rtos::freertos
        rtos::sw_services::generic_pipeline
        fwk_voice::adec
        fwk_voice::aec
        fwk_voice::agc
        fwk_voice::ic
        fwk_voice::ns
        fwk_voice::vnr
)

##******************************************
## Create ADEC altarch AEC+IC+NS+AGC 
##   2 mic input channels
#    2 reference input channels
##******************************************

add_library(adec_altarch_aec_ic_ns_agc_2mic_2ref INTERFACE)
target_sources(adec_altarch_aec_ic_ns_agc_2mic_2ref
    INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/adec_alt_arch/audio_pipeline_t0.c
        ${CMAKE_CURRENT_LIST_DIR}/adec_alt_arch/audio_pipeline_t1.c
        ${CMAKE_CURRENT_LIST_DIR}/adec_alt_arch/stage1/delay_buffer.c
        ${CMAKE_CURRENT_LIST_DIR}/adec_alt_arch/stage1/stage_1.c
        ${CMAKE_CURRENT_LIST_DIR}/adec_alt_arch/aec/aec_process_frame_1thread.c
)
target_include_directories(adec_altarch_aec_ic_ns_agc_2mic_2ref
    INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/adec_alt_arch
        ${CMAKE_CURRENT_LIST_DIR}/adec_alt_arch/aec
        ${CMAKE_CURRENT_LIST_DIR}/adec_alt_arch/stage1
)
target_link_libraries(adec_altarch_aec_ic_ns_agc_2mic_2ref
    INTERFACE
        voice_compat
        core::general
        rtos::freertos
        rtos::sw_services::generic_pipeline
        fwk_voice::adec
        fwk_voice::aec
        fwk_voice::agc
        fwk_voice::ic
        fwk_voice::ns
        fwk_voice::vnr
)

##******************************************
## Create empty audio pipeline
##******************************************

add_library(empty_2mic_2ref INTERFACE)
target_sources(empty_2mic_2ref
    INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/empty/audio_pipeline_t0.c
        ${CMAKE_CURRENT_LIST_DIR}/empty/audio_pipeline_t1.c
)
target_include_directories(empty_2mic_2ref
    INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/empty
)
target_link_libraries(empty_2mic_2ref
    INTERFACE
        core::general
        rtos::freertos
        rtos::sw_services::generic_pipeline
)

##*********************************************
## Create aliases for sln_voice example designs
##*********************************************

add_library(fph::ffva::ap::adec_altarch ALIAS adec_altarch_aec_ic_ns_agc_2mic_2ref)
add_library(fph::ffva::ap::fixed_delay ALIAS fixed_delay_aec_ic_ns_agc_2mic_2ref)
add_library(fph::ffva::ap::adec ALIAS adec_aec_ic_ns_agc_2mic_2ref)
add_library(fph::ffva::ap::empty ALIAS empty_2mic_2ref)

##******************************************
## Four-microphone fixed_delay variants (experimental, separate targets)
##   4 mic input channels (fph::ffva::satellite1_4mic board)
##   4Y x 1X AEC with a mono (L+R)/2 reference
##   tile 0 array front end -> IC -> NS -> AGC, unchanged ESP32 output
## The production fixed_delay pipeline above is not affected.
##******************************************

set(SAT1_ARRAY_DEV_TAP OFF CACHE BOOL
    "Compile the development audio tap into the four-mic pipelines (never for release builds)")

set(FIXED_DELAY_ARRAY_DIR ${CMAKE_CURRENT_LIST_DIR}/fixed_delay_array)
set(ARRAY_LIB_DIR ${CMAKE_CURRENT_LIST_DIR}/../array)

function(fph_add_fixed_delay_array_pipeline NAME ALIAS FRONTEND_MODE)
    add_library(${NAME} INTERFACE)
    target_sources(${NAME}
        INTERFACE
            ${FIXED_DELAY_ARRAY_DIR}/audio_pipeline_t0.c
            ${FIXED_DELAY_ARRAY_DIR}/audio_pipeline_t1.c
            ${FIXED_DELAY_ARRAY_DIR}/array_diag.c
            ${FIXED_DELAY_ARRAY_DIR}/aec/aec_schedule_array.c
            # Same single-thread AEC frame driver as the 2-mic pipeline; it is
            # generic over the configured channel counts.
            ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/fixed_delay/aec/aec_process_frame_1thread.c
            ${ARRAY_LIB_DIR}/array_reference.c
            ${ARRAY_LIB_DIR}/array_select.c
            ${ARRAY_LIB_DIR}/array_stats.c
            ${ARRAY_LIB_DIR}/beamformer.c
            ${ARRAY_LIB_DIR}/crossfade.c
            ${ARRAY_LIB_DIR}/fractional_delay.c
    )
    target_include_directories(${NAME}
        INTERFACE
            ${CMAKE_CURRENT_FUNCTION_LIST_DIR}
            ${FIXED_DELAY_ARRAY_DIR}
            ${FIXED_DELAY_ARRAY_DIR}/aec
            ${ARRAY_LIB_DIR}
    )
    target_link_libraries(${NAME}
        INTERFACE
            voice_compat
            core::general
            rtos::freertos
            rtos::sw_services::generic_pipeline
            fwk_voice::aec
            fwk_voice::agc
            fwk_voice::ic
            fwk_voice::ns
            fwk_voice::vnr
    )
    # fwk_voice v0.9.0 lib_aec is an INTERFACE library: its sources compile
    # inside the firmware executable with these definitions, so the AEC state
    # holds four microphone channels and one reference channel.
    target_compile_definitions(${NAME}
        INTERFACE
            AEC_MAX_Y_CHANNELS=4
            AEC_MAX_X_CHANNELS=1
            appconfAUDIO_PIPELINE_PROC_CHANNELS=4
            appconfARRAY_FRONTEND_MODE=${FRONTEND_MODE}
            appconfARRAY_DIAG_SERVICER_COUNT=1
            appconfARRAY_DEV_TAP=$<BOOL:${SAT1_ARRAY_DEV_TAP}>
    )
    add_library(fph::ffva::ap::${ALIAS} ALIAS ${NAME})
endfunction()

# Four-mic baseline: two IC paths (E/W, N/S), best output VNR selected.
fph_add_fixed_delay_array_pipeline(fixed_delay_array_pairsel_4mic_1ref fixed_delay_4mic 1)
# Four-mic beamformer: 8 fixed delay-and-sum beams feeding IC.
fph_add_fixed_delay_array_pipeline(fixed_delay_array_beam_4mic_1ref fixed_delay_4mic_bf 2)
