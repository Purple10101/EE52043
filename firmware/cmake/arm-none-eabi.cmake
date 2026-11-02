# Cross-compile toolchain for the STM32F446RE (Cortex-M4F) using GNU Arm Embedded.
# Finds arm-none-eabi-gcc on PATH, or in the default winget / Arm installer locations.

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

file(GLOB _ARM_HINTS
    "C:/Program Files (x86)/Arm GNU Toolchain arm-none-eabi/*/bin"
    "C:/Program Files/Arm GNU Toolchain arm-none-eabi/*/bin"
    "C:/Program Files (x86)/GNU Arm Embedded Toolchain/*/bin"
    "C:/Program Files/Arm/GNU Toolchain mingw-w64-x86_64-arm-none-eabi/bin")

find_program(ARM_GCC arm-none-eabi-gcc HINTS ${_ARM_HINTS} REQUIRED)
get_filename_component(_ARM_BIN "${ARM_GCC}" DIRECTORY)

set(CMAKE_C_COMPILER   "${_ARM_BIN}/arm-none-eabi-gcc.exe")
set(CMAKE_ASM_COMPILER "${_ARM_BIN}/arm-none-eabi-gcc.exe")
set(CMAKE_OBJCOPY      "${_ARM_BIN}/arm-none-eabi-objcopy.exe" CACHE FILEPATH "")
set(CMAKE_SIZE         "${_ARM_BIN}/arm-none-eabi-size.exe" CACHE FILEPATH "")

if(NOT EXISTS "${CMAKE_C_COMPILER}")
    # Non-Windows install: no .exe suffix.
    set(CMAKE_C_COMPILER   "${_ARM_BIN}/arm-none-eabi-gcc")
    set(CMAKE_ASM_COMPILER "${_ARM_BIN}/arm-none-eabi-gcc")
    set(CMAKE_OBJCOPY      "${_ARM_BIN}/arm-none-eabi-objcopy" CACHE FILEPATH "" FORCE)
    set(CMAKE_SIZE         "${_ARM_BIN}/arm-none-eabi-size" CACHE FILEPATH "" FORCE)
endif()

# No host executables can run, so test-compile a static library instead.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
