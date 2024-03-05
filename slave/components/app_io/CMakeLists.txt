set(COMPONENT_ADD_INCLUDEDIRS .)
set(COMPONENT_SRCS  "app_io.c" "lpf.c")
set(COMPONENT_REQUIRES  driver app_audio network app_flash min app_mqtt app_ota utilities)
register_component()
