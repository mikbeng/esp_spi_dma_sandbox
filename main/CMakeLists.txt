set(COMPONENT_SRCS main.c my_spi.c)
set(COMPONENT_ADD_INCLUDEDIRS "")
set(COMPONENT_REQUIRES freertos spi_flash)

register_component()