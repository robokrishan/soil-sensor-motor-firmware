#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_log.h"

static const char *TAG = "DEPLOY";

// --------------------------------------------------
// Pin definitions
// --------------------------------------------------
#define PWM_INPUT_GPIO       GPIO_NUM_1

#define LIMIT_TOP_GPIO       GPIO_NUM_7
#define LIMIT_BOTTOM_GPIO    GPIO_NUM_6

#define STEPPER_DIR_GPIO     GPIO_NUM_8
#define STEPPER_STEP_GPIO    GPIO_NUM_9
#define STEPPER_SLEEP_GPIO   GPIO_NUM_14
#define DRIVER_ENABLE_GPIO   GPIO_NUM_15

// --------------------------------------------------
// Timing / thresholds
// --------------------------------------------------
#define STEP_DELAY_US          1000
#define WAKE_DELAY_MS          2
#define MOTION_LOOP_DELAY_MS   10

#define PWM_HIGH_THRESHOLD_US  1700   // UP -> RETRACT
#define PWM_LOW_THRESHOLD_US   1300   // DOWN -> EXTEND
#define PWM_TIMEOUT_US         50000  // 50 ms
#define PWM_POLL_MS            20

// --------------------------------------------------
// Motion state
// --------------------------------------------------
typedef enum {
    MOTION_IDLE = 0,
    MOTION_EXTEND,
    MOTION_RETRACT
} motion_state_t;

typedef enum {
    SWITCH_UNKNOWN = 0,
    SWITCH_UP_RETRACT,
    SWITCH_DOWN_EXTEND,
    SWITCH_MID_UNKNOWN,
    SWITCH_NO_SIGNAL
} switch_state_t;

// --------------------------------------------------
// Shared state
// --------------------------------------------------
static volatile bool top_limit_triggered = false;
static volatile bool bottom_limit_triggered = false;
static volatile motion_state_t motion_state = MOTION_IDLE;

static TaskHandle_t motion_task_handle = NULL;

// PWM shared state
static volatile int64_t rise_time_us = 0;
static volatile uint32_t pulse_width_us = 0;
static volatile int64_t last_update_us = 0;

// --------------------------------------------------
// ISR functions
// --------------------------------------------------
static void IRAM_ATTR top_limit_isr(void *arg)
{
    top_limit_triggered = true;
}

static void IRAM_ATTR bottom_limit_isr(void *arg)
{
    bottom_limit_triggered = true;
}

static void IRAM_ATTR pwm_input_isr(void *arg)
{
    int level = gpio_get_level(PWM_INPUT_GPIO);
    int64_t now = esp_timer_get_time();

    if (level == 1) {
        rise_time_us = now;
    } else {
        if (rise_time_us > 0) {
            pulse_width_us = (uint32_t)(now - rise_time_us);
            last_update_us = now;
        }
    }
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
// PWM decode helper
// --------------------------------------------------
static switch_state_t get_switch_state(uint32_t *width_out)
{
    int64_t now = esp_timer_get_time();
    uint32_t width = pulse_width_us;

    if (width_out != NULL) {
        *width_out = width;
    }

    if ((now - last_update_us) > PWM_TIMEOUT_US) {
        return SWITCH_NO_SIGNAL;
    }

    if (width < PWM_LOW_THRESHOLD_US) {
        return SWITCH_UP_RETRACT;
    } else if (width > PWM_HIGH_THRESHOLD_US) {
        return SWITCH_DOWN_EXTEND;
    } else {
        return SWITCH_MID_UNKNOWN;
    }
}

// --------------------------------------------------
// GPIO setup
// --------------------------------------------------
static void init_gpio(void)
{
    // Outputs
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

    // Limit switches (with pull-ups)
    gpio_config_t limit_conf = {
        .pin_bit_mask =
            (1ULL << LIMIT_TOP_GPIO) |
            (1ULL << LIMIT_BOTTOM_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&limit_conf));

    // PWM input (no pull-up/down)
    gpio_config_t pwm_conf = {
        .pin_bit_mask = (1ULL << PWM_INPUT_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    ESP_ERROR_CHECK(gpio_config(&pwm_conf));

    gpio_set_level(STEPPER_STEP_GPIO, 0);
    gpio_set_level(STEPPER_DIR_GPIO, 0);
    sleep_driver();

    // Install ISR service once
    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    // Limit switch interrupts
    ESP_ERROR_CHECK(gpio_set_intr_type(LIMIT_TOP_GPIO, GPIO_INTR_NEGEDGE));
    ESP_ERROR_CHECK(gpio_set_intr_type(LIMIT_BOTTOM_GPIO, GPIO_INTR_NEGEDGE));
    ESP_ERROR_CHECK(gpio_isr_handler_add(LIMIT_TOP_GPIO, top_limit_isr, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(LIMIT_BOTTOM_GPIO, bottom_limit_isr, NULL));

    // PWM interrupt
    ESP_ERROR_CHECK(gpio_isr_handler_add(PWM_INPUT_GPIO, pwm_input_isr, NULL));
}

// --------------------------------------------------
// PWM command task
// --------------------------------------------------
static void pwm_command_task(void *arg)
{
    switch_state_t last_state = SWITCH_UNKNOWN;
    uint32_t last_logged_width = 0;

    ESP_LOGI(TAG, "System ready.");
    ESP_LOGI(TAG, "Reading PWM on GPIO1.");

    while (1) {
        uint32_t width = 0;
        switch_state_t state = get_switch_state(&width);

        // Log when width meaningfully changes
        if (state != SWITCH_NO_SIGNAL && width != last_logged_width) {
            last_logged_width = width;

            const char *label = "UNKNOWN";

            if (state == SWITCH_UP_RETRACT) {
                label = "UP (RETRACT)";
            } else if (state == SWITCH_DOWN_EXTEND) {
                label = "DOWN (EXTEND)";
            } else if (state == SWITCH_MID_UNKNOWN) {
                label = "MID / UNKNOWN";
            }

            ESP_LOGI(TAG, "PWM: %" PRIu32 " us -> %s", width, label);
        }

        if (state != last_state) {
            last_state = state;

            if (state == SWITCH_UP_RETRACT) {
                if (gpio_get_level(LIMIT_TOP_GPIO) == 0) {
                    ESP_LOGI(TAG, "Already fully retracted (top limit active).");
                    motion_state = MOTION_IDLE;
                    sleep_driver();
                } else {
                    top_limit_triggered = false;
                    bottom_limit_triggered = false;
                    motion_state = MOTION_RETRACT;
                    ESP_LOGI(TAG, "Retract requested from PWM.");
                    if (motion_task_handle != NULL) {
                        xTaskNotifyGive(motion_task_handle);
                    }
                }
            }
            else if (state == SWITCH_DOWN_EXTEND) {
                if (gpio_get_level(LIMIT_BOTTOM_GPIO) == 0) {
                    ESP_LOGI(TAG, "Already fully extended (bottom limit active).");
                    motion_state = MOTION_IDLE;
                    sleep_driver();
                } else {
                    top_limit_triggered = false;
                    bottom_limit_triggered = false;
                    motion_state = MOTION_EXTEND;
                    ESP_LOGI(TAG, "Extend requested from PWM.");
                    if (motion_task_handle != NULL) {
                        xTaskNotifyGive(motion_task_handle);
                    }
                }
            }
            else if (state == SWITCH_NO_SIGNAL) {
                ESP_LOGW(TAG, "No PWM signal detected.");
                motion_state = MOTION_IDLE;
                sleep_driver();
            }
            else {
                ESP_LOGW(TAG, "PWM in mid/unknown range. Stopping motion.");
                motion_state = MOTION_IDLE;
                sleep_driver();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(PWM_POLL_MS));
    }
}

// --------------------------------------------------
// Motion task
// --------------------------------------------------
static void motion_task(void *arg)
{
    while (1) {
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
                if (gpio_get_level(LIMIT_BOTTOM_GPIO) == 0) {
                    if (bottom_limit_triggered) {
                        bottom_limit_triggered = false;
                        ESP_LOGI(TAG, "ISR: Bottom limit triggered.");
                    }
                    stop_motion("Extension stopped.");
                    break;
                }
                step_once();
            }
            else if (motion_state == MOTION_RETRACT) {
                if (gpio_get_level(LIMIT_TOP_GPIO) == 0) {
                    if (top_limit_triggered) {
                        top_limit_triggered = false;
                        ESP_LOGI(TAG, "ISR: Top limit triggered.");
                    }
                    stop_motion("Retraction stopped.");
                    break;
                }
                step_once();
            }

            // Use a real RTOS tick to avoid watchdog issues
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

    xTaskCreate(pwm_command_task, "pwm_command_task", 4096, NULL, 3, NULL);
    xTaskCreate(motion_task, "motion_task", 4096, NULL, 3, &motion_task_handle);
}