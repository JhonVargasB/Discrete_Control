#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "pwm_ctrl.h"

#include "lqi.h"

static const char *TAG = "main_lqi";

#define REFERENCE_TASK_PERIOD_MS 100U
#define INITIAL_REFERENCE_RAD 0.0f

static const motor_config_t motor_config = MOTOR_CONFIG_DEFAULT_A();
static const lqi_config_t lqi_config = LQI_CONFIG_DEFAULT();
static motor_t motor = {0};

static void latch_safety_fault(float theta, float omega)
{
    motor_stop(&motor);
    ESP_LOGE(TAG,
             "Fallo de seguridad: theta=%.4f rad, omega=%.4f rad/s",
             theta,
             omega);
    ESP_LOGE(TAG, "Motor bloqueado; reinicia para repetir la prueba");

    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void latch_reference_fault(float reference)
{
    motor_stop(&motor);
    ESP_LOGE(TAG, "Referencia invalida: %.4f rad", reference);
    ESP_LOGE(TAG, "Motor bloqueado; reinicia para continuar");

    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static lqi_controller_t lqi_controller = {0};
static QueueHandle_t reference_queue = NULL;

static void reference_task(void *argument)
{
    (void)argument;

    float reference = INITIAL_REFERENCE_RAD;
    TickType_t last_wake_time = xTaskGetTickCount();

    while (true)
    {
        /*
         * Reemplazar aqui "reference" con el resultado de la tarea
         * que calcule la trayectoria o el objetivo deseado.
         */

        /* La cola siempre conserva la referencia mas reciente. */
        xQueueOverwrite(reference_queue, &reference);

        vTaskDelayUntil(&last_wake_time,
                        pdMS_TO_TICKS(REFERENCE_TASK_PERIOD_MS));
    }
}

static void lqi_control_task(void *argument)
{
    (void)argument;

    /* Espera la primera referencia antes de habilitar el control. */
    float initial_reference = 0.0f;
    xQueueReceive(reference_queue, &initial_reference, portMAX_DELAY);
    if (lqi_controller_set_reference(&lqi_controller,
                                     initial_reference) != ESP_OK)
    {
        latch_reference_fault(initial_reference);
    }

    int previous_count = 0;
    ESP_ERROR_CHECK(motor_get_count(&motor, &previous_count));

    uint32_t sample_number = 0;
    TickType_t last_wake_time = xTaskGetTickCount();

    while (true)
    {
        vTaskDelayUntil(&last_wake_time,
                        pdMS_TO_TICKS(lqi_controller.config.sample_time_ms));

        /* Si hay una referencia nueva, reemplaza la anterior. */
        float received_reference = 0.0f;
        if (xQueueReceive(reference_queue, &received_reference, 0) == pdTRUE)
        {
            if (lqi_controller_set_reference(&lqi_controller,
                                             received_reference) != ESP_OK)
            {
                latch_reference_fault(received_reference);
            }
        }

        int current_count = 0;
        /* Obtiene una muestra del encoder y detiene el motor si falla. */
        esp_err_t err = motor_get_count(&motor, &current_count);
        if (err != ESP_OK)
        {
            motor_stop(&motor);
            ESP_LOGE(TAG, "No se pudo leer el encoder: %s", esp_err_to_name(err));
            latch_safety_fault(lqi_controller.variables.x.theta,
                               lqi_controller.variables.signals.omega_measured);
        }

        const float theta = encoder_position_rad(&lqi_controller.config,
                                                 current_count);
        const float measured_omega =
            encoder_speed_rad_s(&lqi_controller.config,
                                current_count,
                                previous_count);
        previous_count = current_count;

        err = lqi_controller_step(&lqi_controller,
                                  theta,
                                  measured_omega);
        if (err != ESP_OK)
        {
            latch_safety_fault(theta, measured_omega);
        }

        ESP_ERROR_CHECK(
            motor_set_speed(
                &motor,
                lqi_controller.variables.signals.pwm_percent));

        if (sample_number % TELEMETRY_DECIMATION == 0U)
        {
            const lqi_variables_t *lqi = &lqi_controller.variables;
            const float elapsed_time =
                (float)(sample_number + 1U) *
                ((float)lqi_controller.config.sample_time_ms / 1000.0f);
            printf("LQI: %.3f,%.4f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d,%d\n",
                   elapsed_time,
                   lqi->signals.r,
                   lqi->x.theta,
                   lqi->signals.omega_measured,
                   lqi->x.omega,
                   lqi->signals.e,
                   lqi->x.v,
                   lqi->proposed.u_prop,
                   lqi->signals.u,
                   lqi->signals.pwm_percent,
                   lqi->signals.anti_windup_active ? 1 : 0,
                   lqi->signals.position_reached ? 1 : 0);
        }

        ++sample_number;
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(motor_init(&motor, &motor_config));
    ESP_ERROR_CHECK(motor_stop(&motor));
    ESP_ERROR_CHECK(lqi_controller_init(&lqi_controller, &lqi_config));

    reference_queue = xQueueCreate(1, sizeof(float));
    ESP_ERROR_CHECK(reference_queue == NULL ? ESP_ERR_NO_MEM : ESP_OK);

    ESP_LOGI(TAG, "Control LQI y generador de referencia iniciados");
    ESP_LOGI(TAG,             "Formato: LQI: t,r,theta,omega_delta,omega_f,error,v,u_raw,u_cmd,pwm,aw,ok");

    ESP_ERROR_CHECK(
        xTaskCreate(reference_task,
                    "reference_task",
                    2048,
                    NULL,
                    4,
                    NULL) == pdPASS
            ? ESP_OK
            : ESP_ERR_NO_MEM);

    ESP_ERROR_CHECK(
        xTaskCreate(lqi_control_task,
                    "lqi_control_task",
                    4096,
                    NULL,
                    5,
                    NULL) == pdPASS
            ? ESP_OK
            : ESP_ERR_NO_MEM);
}
