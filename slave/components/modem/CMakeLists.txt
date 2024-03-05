set(COMPONENT_ADD_INCLUDEDIRS .)

set(COMPONENT_SRCS  "src/esp_modem.c"
                    "src/esp_modem_dce_service"
					"src/ec2x.c")

set(COMPONENT_ADD_INCLUDEDIRS "include")

set(COMPONENT_REQUIRES driver main utilities app_io)

register_component()
