# CROSS COMPILER SETTING
SET(CMAKE_SYSTEM_NAME Generic)
CMAKE_MINIMUM_REQUIRED (VERSION 3.10.0)

# THE VERSION NUMBER
SET (Tutorial_VERSION_MAJOR 1)
SET (Tutorial_VERSION_MINOR 0)

# ENABLE ASM
ENABLE_LANGUAGE(ASM)

SET(CMAKE_STATIC_LIBRARY_PREFIX)
SET(CMAKE_STATIC_LIBRARY_SUFFIX)

SET(CMAKE_EXECUTABLE_LIBRARY_PREFIX)
SET(CMAKE_EXECUTABLE_LIBRARY_SUFFIX)

# CURRENT DIRECTORY
SET(ProjDirPath ${CMAKE_CURRENT_SOURCE_DIR})

SET(EXECUTABLE_OUTPUT_PATH ${ProjDirPath}/${CMAKE_BUILD_TYPE})
SET(LIBRARY_OUTPUT_PATH ${ProjDirPath}/${CMAKE_BUILD_TYPE})


project(flexcan_ping_pong_buffer_transfer)

set(MCUX_BUILD_TYPES debug release ddr_debug ddr_release flash_debug flash_release)

set(MCUX_SDK_PROJECT_NAME flexcan_ping_pong_buffer_transfer.elf)

if (NOT DEFINED SdkRootDirPath)
    SET(SdkRootDirPath ${ProjDirPath}/../../../../../..)
endif()

include(${ProjDirPath}/flags.cmake)

include(${ProjDirPath}/config.cmake)

add_executable(${MCUX_SDK_PROJECT_NAME} 
"${ProjDirPath}/../flexcan_ping_pong_buffer_transfer.c"
"${ProjDirPath}/../pin_mux.c"
"${ProjDirPath}/../pin_mux.h"
"${ProjDirPath}/../board.c"
"${ProjDirPath}/../board.h"
"${ProjDirPath}/../clock_config.c"
"${ProjDirPath}/../clock_config.h"
"${ProjDirPath}/../fsl_iomuxc.h"
"${ProjDirPath}/../empty_rsc_table.c"
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
    ${ProjDirPath}/..
)

set(CMAKE_MODULE_PATH
    ${SdkRootDirPath}/devices/MIMX8ML8/drivers
    ${SdkRootDirPath}/devices/MIMX8ML8
    ${SdkRootDirPath}/devices/MIMX8ML8/utilities
    ${SdkRootDirPath}/components/uart
    ${SdkRootDirPath}/components/serial_manager
    ${SdkRootDirPath}/components/lists
    ${SdkRootDirPath}/CMSIS/Core/Include
)

# include modules
include(driver_igpio_MIMX8ML8)

include(driver_flexcan_MIMX8ML8)

include(driver_clock_MIMX8ML8)

include(driver_common_MIMX8ML8)

include(driver_rdc_MIMX8ML8)

include(driver_audiomix_MIMX8ML8)

include(device_MIMX8ML8_CMSIS_MIMX8ML8)

include(utility_debug_console_MIMX8ML8)

include(component_iuart_adapter_MIMX8ML8)

include(component_serial_manager_MIMX8ML8)

include(component_lists_MIMX8ML8)

include(component_serial_manager_uart_MIMX8ML8)

include(driver_iuart_MIMX8ML8)

include(device_MIMX8ML8_startup_MIMX8ML8)

include(utility_assert_MIMX8ML8)

include(CMSIS_Include_core_cm_MIMX8ML8)

include(utilities_misc_utilities_MIMX8ML8)

include(device_MIMX8ML8_system_MIMX8ML8)


IF(NOT DEFINED TARGET_LINK_SYSTEM_LIBRARIES)  
    SET(TARGET_LINK_SYSTEM_LIBRARIES "-lm -lc -lgcc -lnosys")  
ENDIF()  

TARGET_LINK_LIBRARIES(${MCUX_SDK_PROJECT_NAME} PRIVATE -Wl,--start-group)

target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE ${TARGET_LINK_SYSTEM_LIBRARIES})

TARGET_LINK_LIBRARIES(${MCUX_SDK_PROJECT_NAME} PRIVATE -Wl,--end-group)

ADD_CUSTOM_COMMAND(TARGET ${MCUX_SDK_PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_OBJCOPY}
-Obinary ${EXECUTABLE_OUTPUT_PATH}/${MCUX_SDK_PROJECT_NAME} ${EXECUTABLE_OUTPUT_PATH}/flexcan_ping_pong_buffer_transfer.bin)

