idf_component_register(SRCS "main.cpp" "xl9555.c" "myiic.c" "camera.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES esp_lvgl_port lvgl esp32-camera esp_timer)