#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_log.h"

static const char *TAG = "main";

// --------------------------------------------------
// Pin definitions
// --------------------------------------------------
#define LIMIT_TOP_GPIO       GPIO_NUM_7
#define LIMIT_BOTTOM_GPIO    GPIO_NUM_6

#define STEPPER_DIR_GPIO     GPIO_NUM_8
#define STEPPER_STEP_GPIO    GPIO_NUM_9
#define STEPPER_SLEEP_GPIO   GPIO_NUM_14
#define DRIVER_ENABLE_GPIO   GPIO_NUM_15

#define EXTEND_BUTTON_GPIO   GPIO_NUM_1
#define RETRACT_BUTTON_GPIO  GPIO_NUM_2

// --------------------------------------------------
// Timing
// --------------------------------------------------
#define STEP_DELAY_US        1000
#define BUTTON_POLL_MS       20
#define BUTTON_DEBOUNCE_MS   30
#define MOTION_LOOP_DELAY_MS 10
#define WAKE_DELAY_MS        2

// --------------------------------------------------
// Motion state
// --------------------------------------------------
typedef enum {
    MOTION_IDLE = 0,
    MOTION_EXTEND,
    MOTION_RETRACT
} motion_state_t;

static volatile bool halt = false;
static volatile bool top_limit_triggered = false;
static volatile bool bottom_limit_triggered = false;
static volatile motion_state_t motion_state = MOTION_IDLE;

static TaskHandle_t motion_task_handle = NULL;

// --------------------------------------------------
// ISR functions
// --------------------------------------------------
static void IRAM_ATTR top_limit_isr(void *arg)
{
    halt = true;
    top_limit_triggered = true;
}

static void IRAM_ATTR bottom_limit_isr(void *arg)
{
    halt = true;
    bottom_limit_triggered = true;
}

// --------------------------------------------------
// Stepper helper functions
// --------------------------------------------------
static void wake_driver(void)
{
    gpio_set_level(STEPPER_SLEEP_GPIO, 1);
    gpio_set_level(DRIVER_ENABLE_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(WAKE_DELAY_MS));
}

static void sleep_driver(void)
{
    gpio_set_level(STEPPER_SLEEP_GPIO, 0);
    gpio_set_level(DRIVER_ENABLE_GPIO, 1);
}

static inline void step_once(void)
{
    gpio_set_level(STEPPER_STEP_GPIO, 1);
    esp_rom_delay_us(STEP_DELAY_US);
    gpio_set_level(STEPPER_STEP_GPIO, 0);
    esp_rom_delay_us(STEP_DELAY_US);
}

static void stop_motion(const char *msg)
{
    motion_state = MOTION_IDLE;
    sleep_driver();
    ESP_LOGI(TAG, "%s", msg);
}

// --------------------------------------------------
// GPIO setup
// --------------------------------------------------
static void init_gpio(void)
{
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
            (1ULL << LIMIT_TOP_GPIO) |
            (1ULL << LIMIT_BOTTOM_GPIO) |
            (1ULL << EXTEND_BUTTON_GPIO) |
            (1ULL << RETRACT_BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&in_conf));

    gpio_set_level(STEPPER_STEP_GPIO, 0);
    gpio_set_level(STEPPER_DIR_GPIO, 0);
    sleep_driver();

    ESP_ERROR_CHECK(gpio_set_intr_type(LIMIT_TOP_GPIO, GPIO_INTR_NEGEDGE));
    ESP_ERROR_CHECK(gpio_set_intr_type(LIMIT_BOTTOM_GPIO, GPIO_INTR_NEGEDGE));

    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(LIMIT_TOP_GPIO, top_limit_isr, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(LIMIT_BOTTOM_GPIO, bottom_limit_isr, NULL));
}

// --------------------------------------------------
// Button task
// --------------------------------------------------
static void button_task(void *arg)
{
    int last_extend = 1;
    int last_retract = 1;

    ESP_LOGI(TAG, "System ready.");
    ESP_LOGI(TAG, "GP1 -> pull LOW to extend");
    ESP_LOGI(TAG, "GP2 -> pull LOW to retract");
    ESP_LOGI(TAG, "Initial states: GP1=%d GP2=%d BOT=%d TOP=%d",
             gpio_get_level(EXTEND_BUTTON_GPIO),
             gpio_get_level(RETRACT_BUTTON_GPIO),
             gpio_get_level(LIMIT_BOTTOM_GPIO),
             gpio_get_level(LIMIT_TOP_GPIO));

    while (1) {
        int extend_now = gpio_get_level(EXTEND_BUTTON_GPIO);
        int retract_now = gpio_get_level(RETRACT_BUTTON_GPIO);

        if ((last_extend == 1) && (extend_now == 0)) {
            vTaskDelay(pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS));
            if (gpio_get_level(EXTEND_BUTTON_GPIO) == 0) {
                if (gpio_get_level(LIMIT_BOTTOM_GPIO) == 0) {
                    ESP_LOGI(TAG, "Cannot extend: bottom limit already active.");
                } else {
                    halt = false;
                    bottom_limit_triggered = false;
                    motion_state = MOTION_EXTEND;
                    ESP_LOGI(TAG, "Extend requested.");
                    if (motion_task_handle != NULL) {
                        xTaskNotifyGive(motion_task_handle);
                    }
                }

                while (gpio_get_level(EXTEND_BUTTON_GPIO) == 0) {
                    vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_MS));
                }
            }
        }

        if ((last_retract == 1) && (retract_now == 0)) {
            vTaskDelay(pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS));
            if (gpio_get_level(RETRACT_BUTTON_GPIO) == 0) {
                if (gpio_get_level(LIMIT_TOP_GPIO) == 0) {
                    ESP_LOGI(TAG, "Cannot retract: top limit already active.");
                } else {
                    halt = false;
                    top_limit_triggered = false;
                    motion_state = MOTION_RETRACT;
                    ESP_LOGI(TAG, "Retract requested.");
                    if (motion_task_handle != NULL) {
                        xTaskNotifyGive(motion_task_handle);
                    }
                }

                while (gpio_get_level(RETRACT_BUTTON_GPIO) == 0) {
                    vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_MS));
                }
            }
        }

        last_extend = extend_now;
        last_retract = retract_now;

        vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_MS));
    }
}

// --------------------------------------------------
// Motion task
// --------------------------------------------------
static void motion_task(void *arg)
{
    while (1) {
        // Sleep here until a button press requests motion
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (motion_state == MOTION_EXTEND) {
            wake_driver();
            gpio_set_level(STEPPER_DIR_GPIO, 1);
            ESP_LOGI(TAG, "Extending...");
        } else if (motion_state == MOTION_RETRACT) {
            wake_driver();
            gpio_set_level(STEPPER_DIR_GPIO, 0);
            ESP_LOGI(TAG, "Retracting...");
        } else {
            continue;
        }

        while (motion_state != MOTION_IDLE) {
            if (motion_state == MOTION_EXTEND) {
                if (halt || gpio_get_level(LIMIT_BOTTOM_GPIO) == 0) {
                    stop_motion("Extension stopped.");
                    break;
                }
                step_once();
            } else if (motion_state == MOTION_RETRACT) {
                if (halt || gpio_get_level(LIMIT_TOP_GPIO) == 0) {
                    stop_motion("Retraction stopped.");
                    break;
                }
                step_once();
            }

            if (top_limit_triggered) {
                top_limit_triggered = false;
                ESP_LOGI(TAG, "ISR: Top limit triggered.");
            }

            if (bottom_limit_triggered) {
                bottom_limit_triggered = false;
                ESP_LOGI(TAG, "ISR: Bottom limit triggered.");
            }

            vTaskDelay(pdMS_TO_TICKS(MOTION_LOOP_DELAY_MS));
        }
    }
}

// --------------------------------------------------
// Main entry
// --------------------------------------------------
void app_main(void)
{
    init_gpio();

    xTaskCreate(button_task, "button_task", 4096, NULL, 3, NULL);
    xTaskCreate(motion_task, "motion_task", 4096, NULL, 3, &motion_task_handle);
}