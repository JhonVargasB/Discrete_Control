#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pwm_ctrl.h"

#define ENCODER_COUNTS_PER_REVOLUTION 8341.0f
#define TWO_PI_F 6.28318530717958647692f
#define TEST_PWM_PERCENT 15
#define MEASUREMENT_PERIOD_MS 200U
#define SAMPLES_PER_DIRECTION 3U

static const char *TAG = "main_pwm";

void app_main(void)
{
    motor_t motor = {0};
    const motor_config_t motor_config = MOTOR_CONFIG_DEFAULT_A();

    ESP_ERROR_CHECK(motor_init(&motor, &motor_config));
    ESP_ERROR_CHECK(motor_stop(&motor));
    ESP_ERROR_CHECK(motor_clear_count(&motor));

    ESP_LOGI(TAG, "Prueba corta de sentido del motor A");
    vTaskDelay(pdMS_TO_TICKS(1000));

    const int test_speeds[] = {
        TEST_PWM_PERCENT,
        -TEST_PWM_PERCENT,
    };

    for (unsigned direction = 0; direction < 2U; ++direction)
    {
        const int test_speed = test_speeds[direction];
        ESP_ERROR_CHECK(motor_clear_count(&motor));
        ESP_ERROR_CHECK(motor_set_speed(&motor, test_speed));

        int previous_count = 0;

        for (unsigned sample = 0; sample < SAMPLES_PER_DIRECTION; ++sample)
        {
            vTaskDelay(pdMS_TO_TICKS(MEASUREMENT_PERIOD_MS));

            int current_count = 0;
            ESP_ERROR_CHECK(motor_get_count(&motor, &current_count));

            const int delta_count = current_count - previous_count;
            const float theta =
                ((float)current_count * TWO_PI_F) /
                ENCODER_COUNTS_PER_REVOLUTION;
            const float delta_theta =
                ((float)delta_count * TWO_PI_F) /
                ENCODER_COUNTS_PER_REVOLUTION;

            ESP_LOGI(TAG,
                     "PWM=%+4d%%, conteo=%+8d, theta=%+.6f rad, delta_theta=%+.6f rad",
                     test_speed,
                     current_count,
                     theta,
                     delta_theta);

            previous_count = current_count;
        }

        ESP_ERROR_CHECK(motor_stop(&motor));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(TAG, "Prueba terminada; motor detenido");

    while (true)
    {
        vTaskDelay(portMAX_DELAY);
    }
}
