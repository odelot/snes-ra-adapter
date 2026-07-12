cmake_minimum_required(VERSION 3.13)

if(PICO_BUILD)
    # RP2350B (Pico 2 com pacote QFN-80 e 48 GPIOs)
    set(PICO_PLATFORM rp2350-arm-s)
    set(PICO_BOARD pico2)
    set(NAME snes-pico-firmware)

    include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)
    include(${PICO_SDK_PATH}/tools/CMakeLists.txt)

    project(${NAME} C CXX ASM)
    set(CMAKE_C_STANDARD 11)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_BUILD_TYPE Release)

    pico_sdk_init()

    add_subdirectory(src)
endif()

if(UNIT_TESTS_BUILD)
    add_subdirectory(tests)

    enable_testing()
    add_test(NAME tests COMMAND unit_tests)
endif()
