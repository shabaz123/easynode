cmake_minimum_required(VERSION 3.12)
include(pico_sdk_import.cmake)
project(easynode)

pico_sdk_init()


add_subdirectory(pico-lorawan pico-lorawan/lib/LoRaMac-node)
add_subdirectory(no-OS-FatFS-SD-SPI-RPi-Pico/FatFs_SPI)

# rest of your project
add_executable(easynode
    main.c
    hw_config.c
)

target_include_directories(easynode PUBLIC
        pico-lorawan/lib/LoRaMac-node/src/system
        no-OS-FatFS-SD-SPI-RPi-Pico/FatFs_SPI/include
        no-OS-FatFS-SD-SPI-RPi-Pico/FatFs_SPI/sd_driver
        no-OS-FatFS-SD-SPI-RPi-Pico/FatFs_SPI/ff14a/source
        )

target_link_libraries(easynode pico_lorawan hardware_adc pico_stdlib hardware_clocks
        FatFs_SPI hardware_i2c hardware_dma hardware_spi
        )

# enable usb output, disable uart output
pico_enable_stdio_usb(easynode 1)
pico_enable_stdio_uart(easynode 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(easynode)
