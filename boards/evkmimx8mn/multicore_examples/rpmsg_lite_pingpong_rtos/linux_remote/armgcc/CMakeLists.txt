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


project(rpmsg_lite_pingpong_rtos_linux_remote)

set(MCUX_BUILD_TYPES debug release ddr_debug ddr_release flash_debug flash_release)

set(MCUX_SDK_PROJECT_NAME rpmsg_lite_pingpong_rtos_linux_remote.elf)

if (NOT DEFINED SdkRootDirPath)
    SET(SdkRootDirPath ${ProjDirPath}/../../../../../..)
endif()

include(${ProjDirPath}/flags.cmake)

include(${ProjDirPath}/config.cmake)

add_executable(${MCUX_SDK_PROJECT_NAME} 
"${ProjDirPath}/../main_remote.c"
"${ProjDirPath}/../FreeRTOSConfig.h"
"${ProjDirPath}/../pin_mux.c"
"${ProjDirPath}/../pin_mux.h"
"${ProjDirPath}/../rpmsg_config.h"
"${ProjDirPath}/../rsc_table.c"
"${ProjDirPath}/../rsc_table.h"
"${ProjDirPath}/../remoteproc.h"
"${ProjDirPath}/../board.c"
"${ProjDirPath}/../board.h"
"${ProjDirPath}/../clock_config.c"
"${ProjDirPath}/../clock_config.h"
"${ProjDirPath}/../fsl_iomuxc.h"
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
    ${ProjDirPath}/..
)

set(CMAKE_MODULE_PATH
    ${SdkRootDirPath}/middleware/multicore
    ${SdkRootDirPath}/rtos/freertos/freertos-kernel
    ${SdkRootDirPath}/devices/MIMX8MN6/drivers
    ${SdkRootDirPath}/devices/MIMX8MN6
    ${SdkRootDirPath}/devices/MIMX8MN6/utilities
    ${SdkRootDirPath}/components/uart
    ${SdkRootDirPath}/components/serial_manager
    ${SdkRootDirPath}/components/lists
    ${SdkRootDirPath}/CMSIS/Core/Include
)

# include modules
include(middleware_multicore_rpmsg_lite_imx8mn_m7_freertos_MIMX8MN6)

include(middleware_multicore_rpmsg_lite_MIMX8MN6)

include(middleware_multicore_rpmsg_lite_freertos_MIMX8MN6)

include(middleware_freertos-kernel_heap_4_MIMX8MN6)

include(driver_clock_MIMX8MN6)

include(driver_mu_MIMX8MN6)

include(middleware_freertos-kernel_MIMX8MN6)

include(driver_common_MIMX8MN6)

include(driver_rdc_MIMX8MN6)

include(device_MIMX8MN6_CMSIS_MIMX8MN6)

include(utility_debug_console_MIMX8MN6)

include(component_iuart_adapter_MIMX8MN6)

include(component_serial_manager_MIMX8MN6)

include(component_lists_MIMX8MN6)

include(component_serial_manager_uart_MIMX8MN6)

include(driver_iuart_MIMX8MN6)

include(device_MIMX8MN6_startup_MIMX8MN6)

include(utility_assert_MIMX8MN6)

include(middleware_freertos-kernel_extension_MIMX8MN6)

include(CMSIS_Include_core_cm_MIMX8MN6)

include(utilities_misc_utilities_MIMX8MN6)

include(device_MIMX8MN6_system_MIMX8MN6)


IF(NOT DEFINED TARGET_LINK_SYSTEM_LIBRARIES)  
    SET(TARGET_LINK_SYSTEM_LIBRARIES "-lm -lc -lgcc -lnosys")  
ENDIF()  

TARGET_LINK_LIBRARIES(${MCUX_SDK_PROJECT_NAME} PRIVATE -Wl,--start-group)

target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE ${TARGET_LINK_SYSTEM_LIBRARIES})

TARGET_LINK_LIBRARIES(${MCUX_SDK_PROJECT_NAME} PRIVATE -Wl,--end-group)

ADD_CUSTOM_COMMAND(TARGET ${MCUX_SDK_PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_OBJCOPY}
-Obinary ${EXECUTABLE_OUTPUT_PATH}/${MCUX_SDK_PROJECT_NAME} ${EXECUTABLE_OUTPUT_PATH}/rpmsg_lite_pingpong_rtos_linux_remote.bin)

