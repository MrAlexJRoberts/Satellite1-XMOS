
# MICS:
# North: 0 (pin 0 falling edge)
# South: 1 (pin 1 falling edge)
# East:  4 (pin 0 rising edge )
# West:  5 (pin 1 rising edge )
# pins 2 & 3 are not used
#
# Hardware (Satellite1-Hardware, HAT rev4.1/rev6.1): PORT_PDM_DATA is
# XS1_PORT_4D. Bit 0 = X1D16 = MIC_DATA_1 (MK3 SEL=GND, MK4 SEL=VDD), bit 1 =
# X1D17 = MIC_DATA_2 (MK1 SEL=GND, MK2 SEL=VDD). MK1..MK4 sit on a 32.08 mm
# radius circle at 90 degree spacing, so East/West and North/South are each a
# diametrically opposite pair 64.16 mm apart. See
# audio_pipelines/array/array_config.h for the full derivation.

# Creates a Satellite1 board-support target.
#   NAME:        target name (an fph::ffva:: alias is created from ALIAS)
#   ALIAS:       alias suffix, e.g. "satellite1"
#   MIC_COUNT:   number of PDM microphones decoded
#   MIC_MAPPING: PDM input index per output channel
function(fph_add_satellite1_board NAME ALIAS MIC_COUNT MIC_MAPPING)
    add_library(${NAME} INTERFACE)
    target_sources(${NAME}
        INTERFACE
            ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/platform/app_pll_ctrl.c
            ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/platform/driver_instances.c
            ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/platform/platform_init.c
            ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/platform/platform_start.c
    )
    target_include_directories(${NAME}
        INTERFACE
            ${CMAKE_CURRENT_FUNCTION_LIST_DIR}
    )
    target_link_libraries(${NAME}
        INTERFACE
            core::general
            rtos::freertos
            rtos::drivers::general
            rtos::drivers::dfu_image
            fph::qspi_flash_ext
            fph::rtos_mic_array
            fph::i2s_sync
            fph::rtos_ws2812
    )
    target_compile_options(${NAME}
        INTERFACE
            ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/SATELLITE1.xn
    )
    target_link_options(${NAME}
        INTERFACE
            ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/SATELLITE1.xn
    )
    target_compile_definitions(${NAME}
        INTERFACE
            XCOREAI_EXPLORER=1
            PLATFORM_SUPPORTS_TILE_0=1
            PLATFORM_SUPPORTS_TILE_1=1
            PLATFORM_SUPPORTS_TILE_2=0
            PLATFORM_SUPPORTS_TILE_3=0
            USB_TILE_NO=0
            USB_TILE=tile[USB_TILE_NO]

            MIC_ARRAY_CONFIG_MCLK_FREQ=24576000
            MIC_ARRAY_CONFIG_PDM_FREQ=3072000
            MIC_ARRAY_CONFIG_SAMPLES_PER_FRAME=240
            MIC_ARRAY_CONFIG_USE_DDR=1
            MIC_ARRAY_CONFIG_MIC_INPUT=8
            MIC_ARRAY_CONFIG_MIC_COUNT=${MIC_COUNT}
            MIC_ARRAY_CONFIG_INPUT_MAPPING={${MIC_MAPPING}}

            MIC_ARRAY_CONFIG_CLOCK_BLOCK_A=XS1_CLKBLK_1
            MIC_ARRAY_CONFIG_CLOCK_BLOCK_B=XS1_CLKBLK_2
            MIC_ARRAY_CONFIG_PORT_MCLK=PORT_MCLK_IN
            MIC_ARRAY_CONFIG_PORT_PDM_CLK=PORT_PDM_CLK
            MIC_ARRAY_CONFIG_PORT_PDM_DATA=PORT_PDM_DATA
    )
    add_library(fph::ffva::${ALIAS} ALIAS ${NAME})
endfunction()

# Production board: East as first and West as second mic.
fph_add_satellite1_board(fph_ffva_board_support_satellite1 satellite1 2 "4, 5")

# Four-mic board (experimental targets only). Channels 0/1 keep the production
# East/West pair; 2/3 add North/South. lib_mic_array cost at 3.072 MHz PDM with
# the PDM rx ISR: 22.0 MIPS for 2 mics, 43.7 MIPS for 4 (lib_mic_array
# resource_usage.rst), on the dedicated appconfPDM_MIC_IO_CORE thread.
fph_add_satellite1_board(fph_ffva_board_support_satellite1_4mic satellite1_4mic 4 "4, 5, 0, 1")
