include_guard()
message("component_wm8524_adapter component is included.")

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/port/wm8524/fsl_codec_wm8524_adapter.c
)


target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/port/wm8524
    ${CMAKE_CURRENT_LIST_DIR}/port
)


include(driver_wm8524_MIMX8MN6)

include(driver_codec_MIMX8MN6)

