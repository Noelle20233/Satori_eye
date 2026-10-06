set(INC_DIRS
    "."
    "app"
    "board"
    "ui"
    "pipeline"
    "service"
    "drivers"
    "vision")

set(SRCS
    "main.cpp"
    "app/app_context.cpp"
    "app/app.cpp"
    "board/board_display.cpp"
    "ui/preview_ui.cpp"
    "pipeline/camera_preview.cpp"
    "service/panel_guard.cpp"
    "service/console.cpp"
    "service/keys.cpp"
    "drivers/camera.c"
    "drivers/xl9555.c"
    "drivers/myiic.c"
    "vision/face_detect.cpp"
    "vision/face_tracker.cpp")

idf_component_register(SRCS ${SRCS}
                    INCLUDE_DIRS ${INC_DIRS}
                    PRIV_REQUIRES esp_lvgl_port lvgl esp32-camera esp_timer esp_driver_uart esp-dl human_face_detect)
