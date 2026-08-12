#include <stdio.h>
#include "pwm_ctrl.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

static const char *TAG = "main";
uint32_t conteo = 0;
static const motor_config_t motor_config = MOTOR_CONFIG_DEFAULT();
motor_t motor = {0};
// Prototipo de tareas
void vTaskMotor(void *pvParameters);
void vTaskEncoder(void *pvParameters);

void app_main(void)
{
    ESP_ERROR_CHECK(motor_init(&motor, &motor_config));
    xTaskCreate(vTaskMotor, "Motor Task", 2048, NULL, 1, NULL);
    xTaskCreate(vTaskEncoder, "Encoder Task", 2048, NULL, 1, NULL);
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}



// Defincion de tareas
void vTaskMotor(void *pvParameters)
{
    // Variables globlales
    uint8_t speed = 80;
    // Bucle infinito
    while (1)
    {
        //ESP_LOGI(TAG, "Setting speed to %d", speed);
        ESP_ERROR_CHECK(motor_set_speed(&motor, speed));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void vTaskEncoder(void *pvParameters)
{
    int count = 0;
    while (1)
    {
        ESP_ERROR_CHECK(motor_get_count(&motor, &count));
        ESP_LOGI(TAG, "Encoder count: %d", count);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
