#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pwm_ctrl.h"
#include "rls.h"

#define SAMPLE_TIME_MS 20U
#define SAMPLE_TIME_S ((float)SAMPLE_TIME_MS / 1000.0f)

#define PPR 8341.0f
#define TWO_PI_F 6.28318530717958647692f
#define INPUT_HOLD_SAMPLES 40U

static const char *TAG = "main_rls";

/* Escalones persistentes para excitar la dinámica del motor sin invertirlo. */
static const uint8_t input_sequence[] = {30, 55, 75, 40, 85, 60, 35, 70};



static float encoder_speed_rad_s(int current_count, int previous_count)
{
    const int delta_count = current_count - previous_count;
    return ((float)delta_count * TWO_PI_F) /
           (PPR * SAMPLE_TIME_S);
}

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(5000)); 
    motor_t motor = {0};
    const motor_config_t motor_config = MOTOR_CONFIG_DEFAULT();
    rls_t rls;

    ESP_ERROR_CHECK(motor_init(&motor, &motor_config));
    ESP_ERROR_CHECK(rls_init(&rls, 0.995f, 1000.0f));

    int previous_count = 0;
    ESP_ERROR_CHECK(motor_get_count(&motor, &previous_count));

    float previous_y = 0.0f;
    bool has_previous_sample = false;
    size_t input_index = 0;
    uint32_t samples_at_input = 0;
    TickType_t last_wake_time = xTaskGetTickCount();

    while (true) {
        const uint8_t duty_percent = input_sequence[input_index];
        /* Esta entrada permanece aplicada durante el intervalo k-1 -> k. */
        const float applied_u = (float)duty_percent / 100.0f;
        ESP_ERROR_CHECK(motor_set_speed(&motor, duty_percent));

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(SAMPLE_TIME_MS));

        int current_count = 0;
        esp_err_t err = motor_get_count(&motor, &current_count);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "No se pudo leer PCNT: %s", esp_err_to_name(err));
            continue;
        }

        const float current_y = encoder_speed_rad_s(current_count, previous_count);
        previous_count = current_count;

        if (has_previous_sample) {
            /* phi = [y(k-1), u(k-1)]^T. */
            const float phi[RLS_PARAMETER_COUNT] = {previous_y, applied_u};
            float predicted_y = 0.0f;
            float prediction_error = 0.0f;

            err = rls_update(&rls, phi, current_y,
                             &predicted_y, &prediction_error);
            if (err == ESP_OK) {
                ESP_ERROR_CHECK(rls_print_parameters(&rls, current_y, predicted_y));
            } else {
                ESP_LOGW(TAG, "Actualización RLS descartada: %s", esp_err_to_name(err));
            }
        } else {
            has_previous_sample = true;
        }

        previous_y = current_y;

        ++samples_at_input;
        if (samples_at_input >= INPUT_HOLD_SAMPLES) {
            samples_at_input = 0;
            input_index = (input_index + 1U) %
                          (sizeof(input_sequence) / sizeof(input_sequence[0]));
            //ESP_LOGI(TAG, "Nuevo escalón PWM: %u%%", input_sequence[input_index]);
        }
    }
}
