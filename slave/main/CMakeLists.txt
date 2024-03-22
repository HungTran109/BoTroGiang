set(COMPONENT_SRCS "main.c" "pcf8575.c" "kt0656m.c")
set(COMPONENT_ADD_INCLUDEDIRS .)
set(COMPONENT_REQUIRES driver nvs_flash audio_pipeline audio_stream esp_peripherals audio_board audio_hal mqtt mdns bluetooth_service
    esp_https_ota app_update app_ota app_mdns network modem app_ota app_flash app_sntp app_io app_mqtt tcp_console playlist app_uhf)

register_component()
