idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES serial_flash log freertos
)
