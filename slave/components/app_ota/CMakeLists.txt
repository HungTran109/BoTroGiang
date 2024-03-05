set(COMPONENT_ADD_INCLUDEDIRS .)

set(COMPONENT_SRCS  "app_ota.c")

set(COMPONENT_REQUIRES  esp_http_client esp_https_ota main min)

register_component()
