cmake_minimum_required(VERSION 3.13)

# WICHTIG: Board MUSS Pico W sein!
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pico SDK einbinden
# Versuche PICO_SDK_PATH, sonst lokale Datei
if(DEFINED ENV{PICO_SDK_PATH})
    include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
elseif(EXISTS ${CMAKE_CURRENT_LIST_DIR}/pico_sdk_import.cmake)
    include(${CMAKE_CURRENT_LIST_DIR}/pico_sdk_import.cmake)
else()
    message(FATAL_ERROR "PICO_SDK_PATH not set and pico_sdk_import.cmake not found!")
endif()

project(encoder_calibration C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Pico SDK initialisieren
pico_sdk_init()

# Executable
add_executable(encoder_calibration
    src/main.cpp
    src/encoder/as5048a.cpp
    src/encoder/filament_tracker.cpp
    src/adc/hall_sensor.cpp
    src/gatt/encoder_service.cpp
    src/gatt.cpp
)

# Include Directories
target_include_directories(encoder_calibration PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
)

# Libraries - WICHTIG: threadsafe_background mit lwIP deaktiviert
target_link_libraries(encoder_calibration
    pico_stdlib
    pico_cyw43_arch_threadsafe_background  # BLE im Background Thread
    pico_btstack_ble
    pico_btstack_cyw43
    hardware_spi
    hardware_gpio
    hardware_adc
)

# USB und BLE Output
pico_enable_stdio_usb(encoder_calibration 1)
pico_enable_stdio_uart(encoder_calibration 0)

# Bluetooth aktivieren + Poll Mode
target_compile_definitions(encoder_calibration PRIVATE
    CYW43_ENABLE_BLUETOOTH=1
    CYW43_HOST_NAME="Encoder-PicoW"
    CYW43_LWIP=0  # Kein WiFi/lwIP
)

# GATT Database generieren (WICHTIG!)
pico_btstack_make_gatt_header(encoder_calibration PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src/encoder.gatt)

# UF2 Output erstellen
pico_add_extra_outputs(encoder_calibration)

# Binary Info (erscheint in picotool)
pico_set_program_name(encoder_calibration "Encoder Calibration")
pico_set_program_version(encoder_calibration "1.0.0")
pico_set_program_description(encoder_calibration "AS5048A Encoder for Filament Calibration")
pico_set_program_url(encoder_calibration "https://github.com/yourusername/encoder-calibration")
