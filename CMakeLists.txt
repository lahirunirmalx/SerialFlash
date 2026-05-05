idf_component_register(
    SRCS
        "SerialFlashChip.cpp"
        "SerialFlashDirectory.cpp"
    INCLUDE_DIRS
        "include"
    REQUIRES
        driver
        esp_common
        esp_hw_support
        heap
        log
        freertos
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-unused-parameter
)
