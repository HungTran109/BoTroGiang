set(COMPONENT_ADD_INCLUDEDIRS .)

set(COMPONENT_SRCS  "app_mdns.c")

set(COMPONENT_ADD_INCLUDEDIRS "include")

set(COMPONENT_REQUIRES driver mdns)

register_component()
