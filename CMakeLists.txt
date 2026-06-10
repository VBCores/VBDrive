cmake_minimum_required(VERSION 3.22)

include(ExternalProject)

set(CMAKE_C_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 20)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME VBDrive)

# VBBoot is built as a separate firmware image and then merged with VBDrive.
# These cache options are passed through to the bootloader sub-build below.
set(BOOTLOADER_FDCAN_ID_FORMAT "extended" CACHE STRING "VBBoot FDCAN ID format: standard or extended")
set_property(CACHE BOOTLOADER_FDCAN_ID_FORMAT PROPERTY STRINGS standard extended)
if(NOT BOOTLOADER_FDCAN_ID_FORMAT MATCHES "^(standard|extended)$")
    message(FATAL_ERROR "BOOTLOADER_FDCAN_ID_FORMAT must be 'standard' or 'extended'")
endif()
option(BOOTLOADER_USE_EEPROM_CONFIG "Load VBBoot CAN node id and rates from VBDrive EEPROM config" ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

enable_language(C ASM)

project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

add_executable(${CMAKE_PROJECT_NAME})
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
    OUTPUT_NAME ${CMAKE_PROJECT_NAME}
    SUFFIX ".elf"
)
add_compile_definitions(
    -DUSE_HAL_DRIVER
    -DSTM32G
    -DSTM32G4
    -DARM_MATH_CM4
)

add_subdirectory(cmake/stm32cubemx)

target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
)

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    "LINKER:-Map=${CMAKE_PROJECT_NAME}.map"
)

file(GLOB_RECURSE USER_SOURCES "App/*.cpp")
file(GLOB_RECURSE LIBVOLTBRO_SOURCES "Drivers/libvoltbro/*.cpp" "Drivers/libvoltbro/*.c")
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${USER_SOURCES}
    ${LIBVOLTBRO_SOURCES}
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    App
    Drivers/libcxxcanard
    Drivers/libcxxcanard/libs
    Drivers/libvoltbro
    Drivers/CMSIS/DSP/Include
    Drivers/Third-Party
)

add_subdirectory(Drivers/libcxxcanard)
target_link_libraries(libcxxcanard PUBLIC stm32cubemx)
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    libcxxcanard
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/DSP/Lib/GCC/libarm_cortexM4lf_math.a
)
include(Drivers/cyphal_types/cyphal_types.cmake)
cyphal_add_dsdl_types(${CMAKE_PROJECT_NAME}
    TYPE_REPOS
        ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/cyphal_types/public_regulated_data_types
        ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/cyphal_types/voltbro
)

add_compile_options(
    -mfloat-abi=hard   # Use hardware FPU
    -mfpu=fpv4-sp-d16  # For Cortex-M4
)

target_compile_definitions(${CMAKE_PROJECT_NAME} PUBLIC
    $<$<CONFIG:RelWithDebInfo>: -DMONITOR -DFOC_PROFILE>
    $<$<CONFIG:Debug>: -DMONITOR -DDEBUG -DFOC_PROFILE>
)
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    $<$<CONFIG:Release>: -O3 -flto -fno-math-errno -fno-signaling-nans -fomit-frame-pointer>
    $<$<CONFIG:RelWithDebInfo>: -O3 -g2 -flto -fno-math-errno -fno-signaling-nans>
    $<$<CONFIG:Debug>: -Og -g3>
)

if (NOT CMAKE_BUILD_TYPE MATCHES Debug)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    $<$<CONFIG:Release>:-flto>
    $<$<CONFIG:RelWithDebInfo>:-flto>
)

set(
    CMAKE_EXE_LINKER_FLAGS
    "${CMAKE_EXE_LINKER_FLAGS} -mfloat-abi=hard -mfpu=fpv4-sp-d16"
)

add_custom_command(
    POST_BUILD
    TARGET ${CMAKE_PROJECT_NAME}
    COMMAND arm-none-eabi-objcopy -O ihex ${CMAKE_PROJECT_NAME}.elf ${CMAKE_PROJECT_NAME}.hex
)

# VBBoot defines its own `stm32cubemx` target, so it cannot be added with
# add_subdirectory() into the same CMake project. ExternalProject keeps it in
# a nested build directory while still making its hex/bin visible as byproducts.
set(VBBOOT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/VBBoot)
set(VBBOOT_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/VBBoot)
set(VBBOOT_HEX ${VBBOOT_BINARY_DIR}/VBBoot.hex)
set(VBBOOT_BIN ${VBBOOT_BINARY_DIR}/VBBoot.bin)
set(VBDRIVE_HEX ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex)
set(VBDRIVE_FULL_HEX ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}_full.hex)

if(NOT EXISTS ${VBBOOT_SOURCE_DIR}/CMakeLists.txt)
    message(FATAL_ERROR "VBBoot submodule is not initialized at ${VBBOOT_SOURCE_DIR}")
endif()

ExternalProject_Add(VBBoot_firmware
    SOURCE_DIR ${VBBOOT_SOURCE_DIR}
    BINARY_DIR ${VBBOOT_BINARY_DIR}
    CMAKE_GENERATOR "${CMAKE_GENERATOR}"
    CMAKE_ARGS
        -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
        -DCMAKE_TOOLCHAIN_FILE=${VBBOOT_SOURCE_DIR}/cmake/gcc-arm-none-eabi.cmake
        -DBOOTLOADER_FDCAN_ID_FORMAT=${BOOTLOADER_FDCAN_ID_FORMAT}
        -DBOOTLOADER_USE_EEPROM_CONFIG=${BOOTLOADER_USE_EEPROM_CONFIG}
    BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --clean-first
    INSTALL_COMMAND ""
    BUILD_ALWAYS TRUE
    BUILD_BYPRODUCTS
        ${VBBOOT_HEX}
        ${VBBOOT_BIN}
    USES_TERMINAL_CONFIGURE TRUE
    USES_TERMINAL_BUILD TRUE
)

# Produce the user-facing image: bootloader at 0x08000000 and VBDrive app at
# 0x08003000. The script validates both input hex ranges before concatenating
# their Intel HEX records and writing a single EOF record.
add_custom_command(
    OUTPUT ${VBDRIVE_FULL_HEX}
    COMMAND ${CMAKE_COMMAND}
        -DBOOT_HEX=${VBBOOT_HEX}
        -DAPP_HEX=${VBDRIVE_HEX}
        -DOUT_HEX=${VBDRIVE_FULL_HEX}
        -DBOOT_START=0x08000000
        -DBOOT_END=0x08003000
        -DAPP_START=0x08003000
        -DAPP_END=0x08020000
        -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/merge_vbdrive_hex.cmake
    DEPENDS
        ${CMAKE_PROJECT_NAME}
        VBBoot_firmware
        ${VBDRIVE_HEX}
        ${VBBOOT_HEX}
    COMMENT "Merging VBBoot and VBDrive hex files"
    VERBATIM
)

add_custom_target(${CMAKE_PROJECT_NAME}_full_hex ALL
    DEPENDS ${VBDRIVE_FULL_HEX}
)
