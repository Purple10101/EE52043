# Command-line build of the servo motion controller, matching the STM32CubeIDE Debug configuration
# (same sources, defines, include paths, MCU flags and linker script). The CubeIDE project files
# are kept, so the project can still be opened in CubeIDE.
#
#   cmake -S firmware -B firmware/build -G Ninja -DCMAKE_TOOLCHAIN_FILE=firmware/cmake/arm-none-eabi.cmake
#   cmake --build firmware/build
#
# Output: firmware/build/servo_motion_controller.{elf,bin,hex}

cmake_minimum_required(VERSION 3.20)

if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/arm-none-eabi.cmake")
endif()

project(servo_motion_controller C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_EXECUTABLE_SUFFIX ".elf")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()
# CubeIDE Debug build is -O0 -g3. Timing of the 1 ms control loop was tuned against it.
set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_ASM_FLAGS_DEBUG "-g3")

file(GLOB CORE_SOURCES CONFIGURE_DEPENDS Core/Src/*.c)
file(GLOB HAL_SOURCES CONFIGURE_DEPENDS Drivers/STM32F4xx_HAL_Driver/Src/*.c)

add_executable(${PROJECT_NAME}
    ${CORE_SOURCES}
    ${HAL_SOURCES}
    Core/Startup/startup_stm32f446retx.s)

target_include_directories(${PROJECT_NAME} PRIVATE
    Core/Inc
    Drivers/STM32F4xx_HAL_Driver/Inc
    Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32F4xx/Include
    Drivers/CMSIS/Include)

target_compile_definitions(${PROJECT_NAME} PRIVATE
    USE_HAL_DRIVER
    STM32F446xx
    $<$<CONFIG:Debug>:DEBUG>)

# The play build (firmware/README.md): every base/elbow target approached from below. Build it in its own
# directory: cmake -S firmware -B firmware/build_play -G Ninja -DSCARA_PLAY=ON
option(SCARA_PLAY "approach base/elbow targets from one side (PLAY_APPROACH_DEFAULT=1)" OFF)
if(SCARA_PLAY)
    target_compile_definitions(${PROJECT_NAME} PRIVATE PLAY_APPROACH_DEFAULT=1)
endif()

set(MCU_FLAGS -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)

target_compile_options(${PROJECT_NAME} PRIVATE
    ${MCU_FLAGS}
    --specs=nano.specs
    -ffunction-sections
    -fdata-sections
    -Wall
    # GCC 14 makes this an error; CubeIDE's GCC only warns (potentiometers.c's HAL_ADC_Start_DMA call)
    -Wno-error=incompatible-pointer-types
    $<$<COMPILE_LANGUAGE:ASM>:-x assembler-with-cpp>)

target_link_options(${PROJECT_NAME} PRIVATE
    ${MCU_FLAGS}
    -T${CMAKE_CURRENT_SOURCE_DIR}/STM32F446RETX_FLASH.ld
    --specs=nosys.specs
    --specs=nano.specs
    -static
    -Wl,-Map=${PROJECT_NAME}.map
    -Wl,--gc-sections
    -Wl,--print-memory-usage
    -Wl,--start-group -lc -lm -Wl,--end-group)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>
    BYPRODUCTS ${PROJECT_NAME}.bin ${PROJECT_NAME}.hex ${PROJECT_NAME}.map)
