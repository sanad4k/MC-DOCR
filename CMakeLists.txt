cmake_minimum_required(VERSION 3.15)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/arm-none-eabi-gcc.cmake)

project(OC_Relay VERSION 0.0 LANGUAGES C ASM)

# MCU Configuration
set(MCU_MODEL STM32F401xE)
set(MCU_ARCH cortex-m4)

# Paths
set(CMSIS_DEVICE_DIR ${CMAKE_SOURCE_DIR}/cmsis_device_f4)
set(CMSIS_CORE_DIR ${CMAKE_SOURCE_DIR}/CMSIS_5/CMSIS/Core)
set(HAL_DIR ${CMAKE_SOURCE_DIR}/stm32f4xx-hal-driver)

# Include directories
include_directories(
    ${CMSIS_CORE_DIR}/Include
    ${CMSIS_DEVICE_DIR}/Include
    ${HAL_DIR}/Inc
    ${CMAKE_SOURCE_DIR}/Inc
)

# Compiler definitions
add_definitions(-D${MCU_MODEL})
add_definitions(-DUSE_HAL_DRIVER)

# Compiler flags
set(COMMON_FLAGS "-mcpu=${MCU_ARCH} -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fdata-sections -ffunction-sections")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${COMMON_FLAGS} -Wall")
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} ${COMMON_FLAGS} -x assembler-with-cpp")

# Adjust the RAM and FLASH size according to the MCU
# Some common examples are listed here

# STM32F401xC (256KB Flash, 64KB RAM)
# FLASH: LENGTH = 256K, RAM: LENGTH = 64K, CCMRAM: Not available

# STM32F401xE (512KB Flash, 96KB RAM)
# FLASH: LENGTH = 512K, RAM: LENGTH = 96K, CCMRAM: Not available

# STM32F405/407 (1MB Flash, 128KB RAM, 64KB CCM)
# FLASH: LENGTH = 1024K, RAM: LENGTH = 128K, CCMRAM: LENGTH = 64K

# STM32F411xE (512KB Flash, 128KB RAM)
# FLASH: LENGTH = 512K, RAM: LENGTH = 128K, CCMRAM: Not available

# STM32F427/429 (2MB Flash, 192KB RAM, 64KB CCM)
# FLASH: LENGTH = 2048K, RAM: LENGTH = 192K, CCMRAM: LENGTH = 64K

# STM32F446 (512KB Flash, 128KB RAM)
# FLASH: LENGTH = 512K, RAM: LENGTH = 128K, CCMRAM: Not available


# Linker flags
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F4xx_FLASH.ld)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T${LINKER_SCRIPT} -Wl,--gc-sections -Wl,-Map=${PROJECT_NAME}.map --specs=nosys.specs --specs=nano.specs")
# Source files
file(GLOB_RECURSE HAL_SOURCES "${HAL_DIR}/Src/*.c")
list(FILTER HAL_SOURCES EXCLUDE REGEX ".*template.*")
list(FILTER HAL_SOURCES EXCLUDE REGEX ".*_template.*")
file(GLOB APP_SOURCES "${CMAKE_SOURCE_DIR}/Src/*.c")

# Startup file
set(STARTUP_FILE "${CMSIS_DEVICE_DIR}/Source/Templates/gcc/startup_stm32f401xe.s")

# Build executable
add_executable(${PROJECT_NAME}.elf
    ${APP_SOURCES}
    ${HAL_SOURCES}
    ${CMSIS_DEVICE_DIR}/Source/Templates/system_stm32f4xx.c
    ${STARTUP_FILE}
)
target_link_libraries(${PROJECT_NAME}.elf PRIVATE c m nosys)
# Generate hex and bin files
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.bin
    COMMENT "Generating hex and bin files"
)
