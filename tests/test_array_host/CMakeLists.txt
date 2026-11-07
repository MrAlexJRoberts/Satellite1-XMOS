# Host (x86/ARM Linux or macOS) unit tests for the Satellite1 microphone-array
# code. These run without the XMOS XTC toolchain: lib_xcore_math is built with
# its portable C reference implementation and fwk_voice's lib_aec is compiled
# from source with the same AEC_MAX_* channel configuration the firmware uses.
#
# What this does and does not prove:
#   * It proves the AEC/array C code is memory-safe and numerically correct for
#     the configured channel counts, using the same library sources.
#   * It does NOT prove xcore timing, VPU behaviour, RTOS scheduling or memory
#     fit on the XU316. Those need an XTC build (-report) and on-device runs.
#
#   cmake -S tests/test_array_host -B build_host && cmake --build build_host
#   ctest --test-dir build_host --output-on-failure
cmake_minimum_required(VERSION 3.21)
project(sat1_array_host_tests C CXX)
enable_testing()

set(REPO_ROOT ${CMAKE_CURRENT_LIST_DIR}/../..)
set(FW_ROOT ${REPO_ROOT}/satellite-xmos-firmware)
set(VOICE_ROOT ${REPO_ROOT}/modules/voice/modules)
set(XMATH_ROOT ${REPO_ROOT}/modules/core/modules/xcore_math/lib_xcore_math/lib_xcore_math)

set(CMAKE_C_STANDARD 11)

## lib_xcore_math, portable reference implementation
add_subdirectory(${XMATH_ROOT} ${CMAKE_BINARY_DIR}/lib_xcore_math)
target_link_libraries(lib_xcore_math PUBLIC m)

## Array front-end library (pure C; identical sources to the firmware)
set(ARRAY_SRC_DIR ${FW_ROOT}/audio_pipelines/array)
add_library(sat1_array STATIC
    ${ARRAY_SRC_DIR}/array_reference.c
    ${ARRAY_SRC_DIR}/array_stats.c
    ${ARRAY_SRC_DIR}/fractional_delay.c
    ${ARRAY_SRC_DIR}/beamformer.c
    ${ARRAY_SRC_DIR}/array_select.c
    ${ARRAY_SRC_DIR}/crossfade.c
)
target_include_directories(sat1_array PUBLIC ${ARRAY_SRC_DIR})
target_compile_options(sat1_array PRIVATE -O2 -g -Wall -Wextra -Werror -Wconversion -Wno-sign-conversion)
target_link_libraries(sat1_array PUBLIC m)

add_executable(test_array_frontend test_array_frontend.c)
target_link_libraries(test_array_frontend PRIVATE sat1_array)
target_compile_options(test_array_frontend PRIVATE -O2 -g -Wall -Wextra -Werror)
add_test(NAME array_frontend COMMAND test_array_frontend)

## AEC with a given (Y, X) channel configuration.
## lib_aec's state layout depends on AEC_MAX_Y_CHANNELS / AEC_MAX_X_CHANNELS, so
## every configuration is a separate executable, exactly as on the firmware.
set(AEC_SOURCES
    ${VOICE_ROOT}/lib_aec/src/aec_impl.c
    ${VOICE_ROOT}/lib_aec/src/aec_l2_impl.c
    ${VOICE_ROOT}/lib_aec/src/aec_priv_impl.c
    ${VOICE_ROOT}/lib_aec/src/aec_process_frame.c
    ${VOICE_ROOT}/lib_aec/src/aec_schedule_defaults.c
    # The firmware's own single-thread frame driver (shared by all fixed_delay
    # variants) is what runs on the device, so test that, not only the library one.
    ${FW_ROOT}/audio_pipelines/reference/fixed_delay/aec/aec_process_frame_1thread.c
    ${FW_ROOT}/audio_pipelines/reference/fixed_delay_array/aec/aec_schedule_array.c
)

function(add_aec_test NAME Y X)
    add_executable(${NAME} test_aec_array.c ${AEC_SOURCES})
    target_include_directories(${NAME} PRIVATE
        ${VOICE_ROOT}/lib_aec/api
        ${VOICE_ROOT}/lib_aec/src
        ${FW_ROOT}/audio_pipelines/reference/fixed_delay_array/aec)
    target_compile_definitions(${NAME} PRIVATE
        AEC_MAX_Y_CHANNELS=${Y}
        AEC_MAX_X_CHANNELS=${X})
    target_compile_options(${NAME} PRIVATE -O2 -g -Wno-unused-variable -Wno-unused-but-set-variable)
    target_link_libraries(${NAME} PRIVATE lib_xcore_math m)
    add_test(NAME ${NAME} COMMAND ${NAME})
endfunction()

add_aec_test(test_aec_2y2x 2 2)   # production configuration
add_aec_test(test_aec_4y1x 4 1)   # four-mic configuration

## Firmware pipeline sources, type-checked on the host against the real
## fwk_voice / lib_xcore_math / application headers (see fw_stubs/README.md).
## Object libraries only: compiling is the test.
set(FW_CHECK_INCLUDES
    ${CMAKE_CURRENT_LIST_DIR}/fw_stubs
    ${REPO_ROOT}/modules/core/modules/multitile_support/api
    ${CMAKE_CURRENT_LIST_DIR}/fw_stubs/next
    ${FW_ROOT}/src
    ${FW_ROOT}/src/control
    ${FW_ROOT}/bsp_config/SATELLITE1
    ${FW_ROOT}/audio_pipelines
    ${FW_ROOT}/audio_pipelines/reference
    ${FW_ROOT}/audio_pipelines/reference/fixed_delay_array
    ${FW_ROOT}/audio_pipelines/reference/fixed_delay_array/aec
    ${ARRAY_SRC_DIR}
    ${VOICE_ROOT}/lib_aec/api
    ${VOICE_ROOT}/lib_aec/src
    ${VOICE_ROOT}/lib_ic/api
    ${VOICE_ROOT}/lib_vnr/api
    ${VOICE_ROOT}/lib_ns/api
    ${VOICE_ROOT}/lib_agc/api
    ${XMATH_ROOT}/api
    ${REPO_ROOT}/modules/fph/rtos_device_control/api
    ${REPO_ROOT}/modules/rtos/modules/sw_services/generic_pipeline/api
)
set(FW_CHECK_DEFS
    AEC_MAX_Y_CHANNELS=4
    AEC_MAX_X_CHANNELS=1
    appconfAUDIO_PIPELINE_PROC_CHANNELS=4
    appconfARRAY_DIAG_SERVICER_COUNT=1
    appconfDEVICE_CTRL_SPI=1
    appconfPIPELINE_BYPASS=0
    appconfI2S_AUDIO_SAMPLE_RATE=48000
    MIC_ARRAY_CONFIG_MIC_COUNT=4
    MIC_ARRAY_CONFIG_SAMPLES_PER_FRAME=240
    MIC_ARRAY_CONFIG_MCLK_FREQ=24576000
    MIC_ARRAY_CONFIG_PDM_FREQ=3072000
)
set(FW_CHECK_FLAGS -Wall -Wextra -Wno-unused-parameter -Wno-sign-compare -Wno-expansion-to-defined
    -Werror=implicit-function-declaration -Werror=incompatible-pointer-types
    -Werror=int-conversion -Werror=return-type)

function(add_fw_check NAME TILE MODE TAP)
    set(SRCS ${FW_ROOT}/audio_pipelines/reference/fixed_delay_array/audio_pipeline_t${TILE}.c)
    if(TILE EQUAL 0)
        list(APPEND SRCS ${FW_ROOT}/audio_pipelines/reference/fixed_delay_array/array_diag.c)
    endif()
    add_library(${NAME} OBJECT ${SRCS})
    target_include_directories(${NAME} PRIVATE ${FW_CHECK_INCLUDES})
    target_compile_definitions(${NAME} PRIVATE ${FW_CHECK_DEFS}
        THIS_XCORE_TILE=${TILE}
        appconfARRAY_FRONTEND_MODE=${MODE}
        appconfARRAY_DEV_TAP=${TAP})
    target_compile_options(${NAME} PRIVATE ${FW_CHECK_FLAGS})
endfunction()

add_fw_check(fw_check_t1          1 1 0)
add_fw_check(fw_check_t0_pairsel  0 1 0)
add_fw_check(fw_check_t0_beam     0 2 0)
add_fw_check(fw_check_t0_pair_tap 0 1 1)
add_fw_check(fw_check_t0_beam_tap 0 2 1)

## app_conf.h constant equivalence (compile-time assertions).
add_library(app_conf_check_2mic OBJECT test_app_conf_equivalence.c)
target_include_directories(app_conf_check_2mic PRIVATE ${FW_CHECK_INCLUDES})
target_compile_definitions(app_conf_check_2mic PRIVATE EXPECT_MICS=2
    MIC_ARRAY_CONFIG_MIC_COUNT=2 MIC_ARRAY_CONFIG_SAMPLES_PER_FRAME=240
    MIC_ARRAY_CONFIG_MCLK_FREQ=24576000 MIC_ARRAY_CONFIG_PDM_FREQ=3072000)
add_library(app_conf_check_4mic OBJECT test_app_conf_equivalence.c)
target_include_directories(app_conf_check_4mic PRIVATE ${FW_CHECK_INCLUDES})
target_compile_definitions(app_conf_check_4mic PRIVATE EXPECT_MICS=4 ${FW_CHECK_DEFS})
