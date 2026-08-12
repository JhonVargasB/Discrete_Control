#include <ctype.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lqi.h"
#include "pwm_ctrl.h"
#include "robotica.h"
#include "shared.h"

#define HALF_PI_RAD 1.57079632679f
#define DEG_TO_RAD_F 0.01745329251994329577f
#define UART_QUEUE_LENGTH 20U
#define USB_LINE_LENGTH 128U

/* Los encoders aumentan en el sentido opuesto al angulo del modelo 2R. */
#define MOTOR_A_ENCODER_TO_MODEL_SIGN -1.0f
#define MOTOR_B_ENCODER_TO_MODEL_SIGN -1.0f

SemaphoreHandle_t xSemaphore1 = NULL;

static const char *TAG = "main_lqi2";

motor_t motor1 = {0};
motor_t motor2 = {0};
lqi_controller_t lqi1 = {0};
lqi_controller_t lqi2 = {0};

brazo_t brazo1 = {
    .art1.a = 8.3f,
    .art2.a = 10.1f,
};

static const motor_config_t motor1_config = MOTOR_CONFIG_DEFAULT_A();
static const motor_config_t motor2_config = MOTOR_CONFIG_DEFAULT_B();

static const lqi_config_t lqi1_config = LQI_CONFIG_WITH_GAINS(8.5621f, 0.3637f, 1.9635f);
static const lqi_config_t lqi2_config = LQI_CONFIG_WITH_GAINS(8.5621f, 0.3637f, 1.9635f);

typedef struct
{
    unsigned motor_id;
    float elapsed_time;
    float r;
    float theta;
    float omega_measured;
    float omega;
    float e;
    float v;
    float u_prop;
    float u;
    int pwm_percent;
    bool anti_windup_active;
    bool position_reached;
} telemetry_message_t;

typedef enum
{
    REFERENCE_MODE_CI = 0,
    REFERENCE_MODE_CD
} reference_mode_t;

typedef struct
{
    reference_mode_t mode;
    float value1;
    float value2;
} usb_reference_message_t;

// Prototipo de funciones de aplicacion
static void stop_lqi(motor_t *motor, unsigned motor_id, const char *reason, esp_err_t error);
static void run_lqi_control(motor_t *motor, lqi_controller_t *lqi, QueueHandle_t reference_queue, unsigned motor_id);
static void gpio_init(void);
// Prototipo de tareas
void vTaskReference(void *pvParameter);
void vTask_Lqi1(void *pvParameter);
void VTask_Lqi2(void *pvParameter);
void vTaskUart(void *pvParameter);
void vTaskUSB(void *pvParameter);
void vTaskReset(void *pvParameter);
// Protoripo de rutinas de interrupcion

void IRAM_ATTR gpio_isr_handler(void *arg);

// Recursos RTOS
QueueHandle_t r_Queue_1 = NULL;
QueueHandle_t r_Queue_2 = NULL;
QueueHandle_t uart_Queue = NULL;
QueueHandle_t usb_reference_Queue = NULL;

static volatile float desired_px = 18.4f;
static volatile float desired_py = 0.0f;
static volatile reference_mode_t active_reference_mode = REFERENCE_MODE_CD;

void app_main(void)
{
    xSemaphore1 = xSemaphoreCreateBinary();
    gpio_init();
    // Incializacion de hardaware motor1
    ESP_ERROR_CHECK(motor_init(&motor1, &motor1_config));
    ESP_ERROR_CHECK(motor_stop(&motor1));
    ESP_ERROR_CHECK(motor_clear_count(&motor1));
    ESP_ERROR_CHECK(lqi_controller_init(&lqi1, &lqi1_config));

    // Inicializaciond de hardawre motor2
    ESP_ERROR_CHECK(motor_init(&motor2, &motor2_config));
    ESP_ERROR_CHECK(motor_stop(&motor2));
    ESP_ERROR_CHECK(motor_clear_count(&motor2));
    ESP_ERROR_CHECK(lqi_controller_init(&lqi2, &lqi2_config));

    // Inicialzicion de recursos de RTOS
    r_Queue_1 = xQueueCreate(1, sizeof(float));
    r_Queue_2 = xQueueCreate(1, sizeof(float));
    uart_Queue = xQueueCreate(UART_QUEUE_LENGTH, sizeof(telemetry_message_t));
    usb_reference_Queue = xQueueCreate(1, sizeof(usb_reference_message_t));

    // Creacion de tareas
    xTaskCreate(vTaskReference, "reference_task", 4096, NULL, 4, NULL);
    xTaskCreate(vTask_Lqi1, "lqi1_task", 4096, NULL, 5, NULL);
    xTaskCreate(VTask_Lqi2, "lqi2_task", 4096, NULL, 5, NULL);
    xTaskCreate(vTaskUart, "uart_task", 4096, NULL, 3, NULL);
    xTaskCreate(vTaskUSB, "usb_task", 4096, NULL, 3, NULL);
    xTaskCreate(vTaskReset, "rst_task", 4096, NULL, 3, NULL);
}

// Tareas ISR

void IRAM_ATTR gpio_isr_handler(void *arg)
{
    gpio_intr_disable(PIN_RESET);
    xSemaphoreGiveFromISR(xSemaphore1, NULL);

    // Tarea Breve
}

// Definicion de tareas
void vTask_Lqi1(void *pvParameter)
{
    run_lqi_control(&motor1, &lqi1, r_Queue_1, 1U);
}

void VTask_Lqi2(void *pvParameter)
{
    run_lqi_control(&motor2, &lqi2, r_Queue_2, 2U);
}

void vTaskReference(void *argument)
{
    (void)argument;

    usb_reference_message_t request;

    while (true)
    {
        if (xQueueReceive(usb_reference_Queue,
                          &request,
                          portMAX_DELAY) != pdTRUE)
        {
            continue;
        }

        float joint_angles[2] = {0.0f, 0.0f};
        float desired_position[2] = {0.0f, 0.0f};

        if (request.mode == REFERENCE_MODE_CI)
        {
            desired_position[0] = request.value1;
            desired_position[1] = request.value2;

            const int elbow_configuration =
                desired_position[1] < 0.0f ? -1 : 1;

            if (!i_cinematica(&brazo1,
                              desired_position[0],
                              desired_position[1],
                              elbow_configuration,
                              joint_angles))
            {
                ESP_LOGW(TAG,
                         "CI fuera del alcance: px=%.4f, py=%.4f",
                         desired_position[0],
                         desired_position[1]);
                continue;
            }
        }
        else
        {
            joint_angles[0] = request.value1 * DEG_TO_RAD_F;
            joint_angles[1] = request.value2 * DEG_TO_RAD_F;

            if (!d_cinematica(&brazo1,
                              joint_angles[0],
                              joint_angles[1],
                              desired_position))
            {
                ESP_LOGW(TAG, "No se pudo calcular la cinematica directa");
                continue;
            }
        }

        const float theta1 = joint_angles[0];
        const float theta2 = joint_angles[1];

        if (!isfinite(theta1) || !isfinite(theta2) ||
            fabsf(theta1) > HALF_PI_RAD ||
            fabsf(theta2) > HALF_PI_RAD)
        {
            ESP_LOGW(TAG,
                     "Angulos fuera de [-pi/2, pi/2]: theta1=%.4f, theta2=%.4f",
                     theta1,
                     theta2);
            continue;
        }

        /* Motor B controla theta1; motor A controla theta2. */
        const float motor_b_reference =
            theta1 / MOTOR_B_ENCODER_TO_MODEL_SIGN;
        const float motor_a_reference =
            theta2 / MOTOR_A_ENCODER_TO_MODEL_SIGN;

        desired_px = desired_position[0];
        desired_py = desired_position[1];
        active_reference_mode = request.mode;
        xQueueOverwrite(r_Queue_2, &motor_b_reference);
        xQueueOverwrite(r_Queue_1, &motor_a_reference);

        ESP_LOGI(TAG,
                 "%s aplicada: theta1(B)=%.4f, theta2(A)=%.4f, px=%.4f, py=%.4f",
                 request.mode == REFERENCE_MODE_CI ? "CI" : "CD",
                 theta1,
                 theta2,
                 desired_position[0],
                 desired_position[1]);
    }
}

void vTaskUart(void *pvParameter)
{
    (void)pvParameter;

    telemetry_message_t telemetry;
    telemetry_message_t motor_a = {0};
    telemetry_message_t motor_b = {0};
    bool motor_a_updated = false;
    bool motor_b_updated = false;

    while (true)
    {
        if (xQueueReceive(uart_Queue, &telemetry, portMAX_DELAY) == pdTRUE)
        {
            if (telemetry.motor_id == 1U)
            {
                motor_a = telemetry;
                motor_a_updated = true;
            }
            else if (telemetry.motor_id == 2U)
            {
                motor_b = telemetry;
                motor_b_updated = true;
            }

            if (motor_a_updated && motor_b_updated)
            {
                /* Motor B = theta1; motor A = theta2. Angulos en radianes. */
                const float theta1_reference =
                    MOTOR_B_ENCODER_TO_MODEL_SIGN * motor_b.r;
                const float theta2_reference =
                    MOTOR_A_ENCODER_TO_MODEL_SIGN * motor_a.r;
                const float theta1_measured =
                    MOTOR_B_ENCODER_TO_MODEL_SIGN * motor_b.theta;
                const float theta2_measured =
                    MOTOR_A_ENCODER_TO_MODEL_SIGN * motor_a.theta;
                float measured_position[2];

                if (!d_cinematica(&brazo1,
                                  theta1_measured,
                                  theta2_measured,
                                  measured_position))
                {
                    motor_a_updated = false;
                    motor_b_updated = false;
                    continue;
                }

                printf("ARM,%s,%.4f,%.4f,%.4f,%.4f,%.6f,%.6f,%.6f,%.6f\n",
                       active_reference_mode == REFERENCE_MODE_CI ? "CI" : "CD",
                       desired_px,
                       desired_py,
                       measured_position[0],
                       measured_position[1],
                       theta1_reference,
                       theta2_reference,
                       theta1_measured,
                       theta2_measured);

                motor_a_updated = false;
                motor_b_updated = false;
            }
        }
    }
}

void vTaskUSB(void *pvParameter)
{
    (void)pvParameter;

    char line[USB_LINE_LENGTH];

    while (true)
    {
        if (fgets(line, sizeof(line), stdin) == NULL)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        size_t length = strlen(line);
        if (length == 0U || line[length - 1U] != '\n')
        {
            int character;
            do
            {
                character = getchar();
            } while (character != '\n' && character != EOF);

            ESP_LOGW(TAG, "Dato USB descartado: falta terminador CRLF");
            continue;
        }

        line[--length] = '\0';
        if (length > 0U && line[length - 1U] == '\r')
        {
            line[--length] = '\0';
        }

        for (size_t index = 0U; index < length; ++index)
        {
            if (line[index] == ',')
            {
                line[index] = ' ';
            }
        }

        char command[3] = {0};
        usb_reference_message_t request = {0};
        char extra_character;
        const int parsed_fields = sscanf(line,
                                         " %2s %f %f %c",
                                         command,
                                         &request.value1,
                                         &request.value2,
                                         &extra_character);

        command[0] = (char)toupper((unsigned char)command[0]);
        command[1] = (char)toupper((unsigned char)command[1]);

        if (strcmp(command, "CI") == 0)
        {
            request.mode = REFERENCE_MODE_CI;
        }
        else if (strcmp(command, "CD") == 0)
        {
            request.mode = REFERENCE_MODE_CD;
        }
        else
        {
            ESP_LOGW(TAG,
                     "Comando desconocido; use CI px py o CD theta1 theta2");
            continue;
        }

        if (parsed_fields != 3 ||
            !isfinite(request.value1) ||
            !isfinite(request.value2))
        {
            ESP_LOGW(TAG,
                     "Dato USB invalido; use CI px py o CD theta1 theta2");
            continue;
        }

        if (request.mode == REFERENCE_MODE_CD &&
            (fabsf(request.value1) > 90.0f ||
             fabsf(request.value2) > 90.0f))
        {
            ESP_LOGW(TAG, "CD rechazado: ambos angulos deben estar entre -90 y 90 grados");
            continue;
        }

        if (usb_reference_Queue != NULL)
        {
            xQueueOverwrite(usb_reference_Queue, &request);
            ESP_LOGI(TAG, "USB recibido: %s %.4f %.4f",
                     request.mode == REFERENCE_MODE_CI ? "CI" : "CD",
                     request.value1,
                     request.value2);
        }
    }
}

void vTaskReset(void *pvParameter)
{

    while (1)
    {
        // Intentar tomar el semaforo
        if (xSemaphoreTake(xSemaphore1, portMAX_DELAY) == pdTRUE)
        {
            gpio_set_level(PIN_LED, 1);
            ESP_LOGI(TAG, "eventProcess: se ha generado un eevnto");
            const float reference1 = 0.0f;
            const float reference2 = 0.0f;
            desired_px = brazo1.art1.a + brazo1.art2.a;
            desired_py = 0.0f;
            active_reference_mode = REFERENCE_MODE_CD;
            xQueueOverwrite(r_Queue_1, &reference1);
            xQueueOverwrite(r_Queue_2, &reference2);
            vTaskDelay(pdMS_TO_TICKS(200));
            gpio_set_level(PIN_LED, 0);
            gpio_intr_enable(PIN_RESET);
        }

        // Procesar y seguir con la aplicacion
    }
}
// Definicion de funciones de aplciacion

static void gpio_init(void)
{
    gpio_config_t i_Config = {
        .pin_bit_mask = (1ULL << PIN_RESET),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE};

    gpio_config(&i_Config);

    gpio_config_t o_Config = {
        .pin_bit_mask = (1ULL << PIN_LED),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};

    gpio_config(&i_Config);
    gpio_config(&o_Config);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_RESET, gpio_isr_handler, NULL);
}

static void stop_lqi(motor_t *motor, unsigned motor_id, const char *reason, esp_err_t error)
{
    motor_stop(motor);
    ESP_LOGE(TAG, "Motor %u bloqueado: %s (%s)", motor_id, reason, esp_err_to_name(error));

    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void run_lqi_control(motor_t *motor,
                            lqi_controller_t *lqi,
                            QueueHandle_t reference_queue,
                            unsigned motor_id)
{
    lqi_variables_t *variables = &lqi->variables;

    float reference = 0.0f;
    xQueueReceive(reference_queue, &reference, portMAX_DELAY);

    esp_err_t err = lqi_controller_set_reference(lqi, reference);
    if (err != ESP_OK)
    {
        stop_lqi(motor, motor_id, "referencia invalida", err);
    }

    int previous_count = 0;
    err = motor_get_count(motor, &previous_count);
    if (err != ESP_OK)
    {
        stop_lqi(motor, motor_id, "lectura inicial del encoder", err);
    }

    uint32_t sample_number = 0;
    TickType_t last_wake_time = xTaskGetTickCount();

    while (true)
    {
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(lqi->config.sample_time_ms));

        float received_reference = 0.0f;

        if (xQueueReceive(reference_queue, &received_reference, 0) == pdTRUE)
        {
            err = lqi_controller_set_reference(lqi, received_reference);
            if (err != ESP_OK)
            {
                stop_lqi(motor, motor_id, "referencia invalida", err);
            }
        }

        int current_count = 0;
        // Obtener el contador pcnt
        err = motor_get_count(motor, &current_count);

        if (err != ESP_OK)
        {
            stop_lqi(motor, motor_id, "lectura del encoder", err);
        }
        // el estado se convierte de Rev a Rad
        const float theta = encoder_position_rad(&lqi->config,
                                                 current_count);
        // Velocidad angular
        const float measured_omega = encoder_speed_rad_s(&lqi->config,
                                                         current_count,
                                                         previous_count);

        previous_count = current_count;

        err = lqi_controller_step(lqi, theta, measured_omega);
        if (err != ESP_OK)
        {
            stop_lqi(motor, motor_id, "variables LQI invalidas", err);
        }
        // Establecer señal de  control obtenia del ciclo anterior
        /* Limites relativos al angulo fisico inicial de 90 grados. */
        if ((theta >= HALF_PI_RAD && variables->signals.pwm_percent > 0) ||
            (theta <= -HALF_PI_RAD && variables->signals.pwm_percent < 0))
        {
            variables->signals.pwm_percent = 0;
            variables->signals.u_applied = 0.0f;
            lqi->actuator.applied_control = 0.0f;
        }

        err = motor_set_speed(motor, variables->signals.pwm_percent);

        if (err != ESP_OK)
        {
            stop_lqi(motor, motor_id, "salida PWM", err);
        }
        // Mandar los datos a una cola par leugo procesarlos
        if (sample_number % TELEMETRY_DECIMATION == 0U)
        {
            const telemetry_message_t telemetry = {
                .motor_id = motor_id,
                .elapsed_time =
                    (float)(sample_number + 1U) *
                    ((float)lqi->config.sample_time_ms / 1000.0f),
                .r = variables->signals.r,
                .theta = variables->x.theta,
                .omega_measured = variables->signals.omega_measured,
                .omega = variables->x.omega,
                .e = variables->signals.e,
                .v = variables->x.v,
                .u_prop = variables->proposed.u_prop,
                .u = variables->signals.u,
                .pwm_percent = variables->signals.pwm_percent,
                .anti_windup_active = variables->signals.anti_windup_active,
                .position_reached = variables->signals.position_reached,
            };

            /* Si la UART esta ocupada se descarta esta muestra de telemetria. */
            xQueueSend(uart_Queue, &telemetry, 0);
        }
        ++sample_number;
    }
}
