set(COMPONENT_ADD_INCLUDEDIRS .)

set(COMPONENT_SRCS  "network.c")

set(COMPONENT_ADD_INCLUDEDIRS "include")

set(COMPONENT_REQUIRES driver esp_eth app_mdns modem)

register_component()
