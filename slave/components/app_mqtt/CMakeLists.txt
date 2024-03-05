set(COMPONENT_ADD_INCLUDEDIRS .)
set(COMPONENT_SRCS  "app_mqtt.c")
set(COMPONENT_REQUIRES  driver mqtt network app_flash app_ota app_io utilities)
register_component()
