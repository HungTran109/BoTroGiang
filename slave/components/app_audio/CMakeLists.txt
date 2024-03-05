set(COMPONENT_ADD_INCLUDEDIRS .)
set(COMPONENT_SRCS  "app_audio.c")
set(COMPONENT_REQUIRES  driver audio_pipeline audio_stream app_flash app_io)
register_component()
