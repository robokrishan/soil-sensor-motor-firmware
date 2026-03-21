#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_log.h"

static const char *TAG = "DEPLOY";

// Command inputs
#define EXTEND_BUTTON_GPIO   GPIO_NUM_1
#define RETRACT_BUTTON_GPIO  GPIO_NUM_2

// Limit switches
#define LIMIT_BOTTOM_GPIO    GPIO_NUM_6
#define LIMIT_TOP_GPIO       GPIO_NUM_7

// Stepper driver
#define STEPPER_DIR_GPIO     GPIO_NUM_8
#define STEPPER_STEP_GPIO    GPIO_NUM_9
#define STEPPER_SLEEP_GPIO   GPIO_NUM_14
#define DRIVER_ENABLE_GPIO   GPIO_NUM_15

#define STEP_DELAY_US   1000
#define WAKE_DELAY_MS   2
#define POLL_DELAY_MS   20

static volatile bool halt = false;

static void wake_driver(void) {
    gpio_set_level(DRIVER_ENABLE_GPIO, 0);
    gpio_set_level(STEPPER_SLEEP_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(WAKE_DELAY_MS));
}

static void sleep_driver(void) {
    gpio_set_level(STEPPER_SLEEP_GPIO, 0);
    gpio_set_level(DRIVER_ENABLE_GPIO, 1);
}

static inline void step_once(void) {
    gpio_set_level(STEPPER_STEP_GPIO, 1);
    esp_rom_delay_us(STEP_DELAY_US);
    gpio_set_level(STEPPER_STEP_GPIO, 0);
    esp_rom_delay_us(STEP_DELAY_US);
}

static void extend(void) {
    if (gpio_get_level(LIMIT_BOTTOM_GPIO) == 0) {
        ESP_LOGI(TAG, "Cannot extend: bottom limit already active.");
        return;
    }

    halt = false;
    wake_driver();

    // Swap if direction is reversed
    gpio_set_level(STEPPER_DIR_GPIO, 1);

    ESP_LOGI(TAG, "Extending...");

    int step_counter = 0;

    while (!halt) {
        if (gpio_get_level(LIMIT_BOTTOM_GPIO) == 0) {
            halt = true;
            break;
        }

        step_once();
        step_counter++;

        if (step_counter >= 100) {
            step_counter = 0;
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    sleep_driver();
    ESP_LOGI(TAG, "Extension stopped.");

    vTaskDelay(pdMS_TO_TICKS(10));
}

static void retract(void) {
    if (gpio_get_level(LIMIT_TOP_GPIO) == 0) {
        ESP_LOGI(TAG, "Cannot retract: top limit already active.");
        return;
    }

    halt = false;
    wake_driver();

    // Swap if direction is reversed
    gpio_set_level(STEPPER_DIR_GPIO, 0);

    ESP_LOGI(TAG, "Retracting...");

    int step_counter = 0;

    while (!halt) {
        if (gpio_get_level(LIMIT_TOP_GPIO) == 0) {
            halt = true;
            break;
        }

        step_once();
        step_counter++;

        if (step_counter >= 100) {
            step_counter = 0;
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    sleep_driver();
    ESP_LOGI(TAG, "Retraction stopped.");
}

static void init_gpio(void) {
    gpio_config_t out_conf = {
        .pin_bit_mask =
            (1ULL << STEPPER_DIR_GPIO) |
            (1ULL << STEPPER_STEP_GPIO) |
            (1ULL << STEPPER_SLEEP_GPIO) |
            (1ULL << DRIVER_ENABLE_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&out_conf));

    gpio_config_t in_conf = {
        .pin_bit_mask =
            (1ULL << EXTEND_BUTTON_GPIO) |
            (1ULL << RETRACT_BUTTON_GPIO) |
            (1ULL << LIMIT_BOTTOM_GPIO) |
            (1ULL << LIMIT_TOP_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&in_conf));

    gpio_set_level(STEPPER_STEP_GPIO, 0);
    gpio_set_level(STEPPER_DIR_GPIO, 0);
    sleep_driver();
}

static void button_task(void *arg) {
    int last_extend_state = 1;
    int last_retract_state = 1;

    ESP_LOGI(TAG, "System ready.");
    ESP_LOGI(TAG, "GP1 -> pull LOW to extend");
    ESP_LOGI(TAG, "GP2 -> pull LOW to retract");

    while (1) {
        int extend_state = gpio_get_level(EXTEND_BUTTON_GPIO);
        int retract_state = gpio_get_level(RETRACT_BUTTON_GPIO);

        if ((last_extend_state == 1) && (extend_state == 0)) {
            ESP_LOGI(TAG, "Extend command triggered from GP1.");
            extend();
        }

        if ((last_retract_state == 1) && (retract_state == 0)) {
            ESP_LOGI(TAG, "Retract command triggered from GP2.");
            retract();
        }

        last_extend_state = extend_state;
        last_retract_state = retract_state;

        vTaskDelay(pdMS_TO_TICKS(POLL_DELAY_MS));
    }
}

void app_main(void) {
    init_gpio();
    xTaskCreate(button_task, "button_task", 4096, NULL, 5, NULL);
}