cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 20)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME VBDrive)

include("cmake/gcc-arm-none-eabi.cmake")

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

enable_language(C ASM)

project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

add_executable(${CMAKE_PROJECT_NAME})
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
    OUTPUT_NAME ${CMAKE_PROJECT_NAME}
    SUFFIX ".elf"
)
add_subdirectory(cmake/stm32cubemx)

target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/DSP/Lib/GCC/libarm_cortexM4lf_math.a
)

target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
)

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    "LINKER:-Map=${CMAKE_PROJECT_NAME}.map"
)

file(GLOB_RECURSE USER_SOURCES "App/*.cpp")
file(GLOB_RECURSE LIBCXXCANARD_SOURCES "Drivers/libcxxcanard/cyphal/*.cpp" "Drivers/libcxxcanard/libs/*.c")
file(GLOB_RECURSE LIBVOLTBRO_SOURCES "Drivers/libvoltbro/*.cpp" "Drivers/libvoltbro/*.c")
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${USER_SOURCES}
    ${LIBVOLTBRO_SOURCES}
    ${LIBCXXCANARD_SOURCES}
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    App
    Drivers/libcxxcanard
    Drivers/libcxxcanard/libs
    Drivers/libvoltbro
    Drivers/CMSIS/DSP/Include
    Drivers/cyphal_types
)

if(CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -g3")
    target_compile_definitions(${CMAKE_PROJECT_NAME} PUBLIC
        -DMONITOR
    )
endif()
if(CMAKE_BUILD_TYPE MATCHES Release)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -g1")
endif()
if(CMAKE_BUILD_TYPE MATCHES Debug)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Og -g3")
    target_compile_definitions(${CMAKE_PROJECT_NAME} PUBLIC
        -DDEBUG
        -DMONITOR
    )
endif()

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    -DLCM
    -DUSE_HAL_DRIVER
    -DSTM32_G
    -DARM_MATH_CM4
    -D__FPU_PRESENT
)

add_compile_options(
    -mfloat-abi=hard   # Use hardware FPU
    -mfpu=fpv4-sp-d16  # For Cortex-M4
)
set(
    CMAKE_EXE_LINKER_FLAGS
    "${CMAKE_EXE_LINKER_FLAGS} -mfloat-abi=hard -mfpu=fpv4-sp-d16 -u _printf_float"
)

add_custom_command(
    POST_BUILD
    TARGET ${CMAKE_PROJECT_NAME}
    COMMAND arm-none-eabi-objcopy -O ihex ${CMAKE_PROJECT_NAME}.elf ${CMAKE_PROJECT_NAME}.hex
)
