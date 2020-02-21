set(COMPONENT_SRCS main.c mspi.c)
set(COMPONENT_ADD_INCLUDEDIRS "")
set(COMPONENT_REQUIRES freertos spi_flash)

register_component()