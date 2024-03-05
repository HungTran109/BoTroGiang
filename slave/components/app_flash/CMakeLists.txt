set(COMPONENT_ADD_INCLUDEDIRS .)
set(COMPONENT_SRCS  "app_flash.c")
set(COMPONENT_REQUIRES  driver nvs_flash app_audio)
register_component()
