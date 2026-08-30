set(proto_sources "main.c")
set(proto_private_requires
  driver
  esp_driver_i2s
  esp_lcd_co5300
  esp_timer
  waveshare__qmi8658
)

if(GRANOLA_PROTO_AUDIO)
  list(APPEND proto_sources "audio_stream.c")
  list(APPEND proto_private_requires espressif__esp_codec_dev)
endif()

idf_component_register(
  SRCS ${proto_sources}
  INCLUDE_DIRS "."
  PRIV_REQUIRES ${proto_private_requires}
)

if(GRANOLA_PROTO_AUDIO)
  target_compile_definitions(${COMPONENT_LIB} PRIVATE GRANOLA_PROTO_AUDIO=1)
endif()
