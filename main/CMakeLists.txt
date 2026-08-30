idf_component_register(
  SRCS "main.c"
  INCLUDE_DIRS "."
  PRIV_REQUIRES driver esp_lcd_co5300 esp_timer waveshare__qmi8658
)
