#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

static constexpr const char *TAG = "MAIN";

class Application {
public:
  void init() {
    ESP_LOGI(TAG, "Initializing application...");

    // 初始化GPIO
    gpio_set_direction(GPIO_NUM_1, gpio_mode_t::GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_1, 1);
  }

  void run() {
    while (true) {
      loop();

      vTaskDelay(pdMS_TO_TICKS(500));
    }
  }

private:
  int status = 1;
  void loop() {
    // 主循环代码
    if (status == 0) {
      status = 1;
    } else {
      status = 0;
    }
    gpio_set_level(GPIO_NUM_1, status);
    return;
  }
};

extern "C" void app_main() {
  Application app;

  app.init();
  app.run();
}