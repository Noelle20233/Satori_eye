idf_component_register(SRCS "main.cpp" "xl9555.c" "myiic.c" "camera.c" "face_detect.cpp"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES esp_lvgl_port lvgl esp32-camera esp_timer esp_driver_uart esp-dl human_face_detect)