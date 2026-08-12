#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "driver/pulse_cnt.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pwm_ctrl.h"

#define SAMPLE_TIME_MS 20U
#define SAMPLE_TIME_S ((float)SAMPLE_TIME_MS / 1000.0f)
#define ENCODER_COUNTS_PER_REVOLUTION 8341.0f
#define TWO_PI_F 6.28318530717958647692f
#define PI_F (TWO_PI_F / 2.0f)

#define LQI_K_THETA 8.5621f
#define LQI_K_OMEGA 0.3637f
#define LQI_K_I 1.9635f

#define CONTROL_MIN (-0.15f)
#define CONTROL_MAX 0.15f
#define CONTROL_SLEW_PER_SAMPLE 0.02f
#define REVERSAL_HOLD_SAMPLES 5U

#define POSITION_ENTER_TOL_RAD 0.010f
#define POSITION_EXIT_TOL_RAD 0.020f
#define SPEED_TOL_RAD_S 0.05f
#define SETTLED_DWELL_SAMPLES 5U

#define VELOCITY_FILTER_ALPHA 0.60f
#define POSITION_SAFETY_LIMIT_RAD 3.50f
#define SPEED_SAFETY_LIMIT_RAD_S 3.0f

#define FIRST_ENDPOINT_SAMPLES 750U
#define FULL_TRAVEL_SAMPLES 1000U
#define RETURN_ZERO_SAMPLES 750U
#define TELEMETRY_DECIMATION 5U

static const char *TAG = "main_lqi";

typedef struct
{
    float reference_rad;
    uint32_t hold_samples;
} reference_step_t;

/* Trayectoria: -pi -> +pi -> -pi -> 0. */
static const reference_step_t reference_sequence[] = {
    {-PI_F / 2, FIRST_ENDPOINT_SAMPLES},
    {PI_F / 2, FULL_TRAVEL_SAMPLES},
    {-PI_F, FULL_TRAVEL_SAMPLES},
    {0.0f, RETURN_ZERO_SAMPLES},
};

typedef struct
{
    /* v(k) = v(k-1) + e(k), sin multiplicar por Ts. */
    float integral_error;
} lqi_controller_t;

typedef struct
{
    bool reached;
    uint32_t dwell_samples;
} position_gate_t;

typedef struct
{
    float applied_control;
    uint32_t reversal_hold_samples;
    bool reversing;
} safe_actuator_t;

typedef struct
{
    float error;
    float unsaturated_control;
    float requested_control;
    bool anti_windup_active;
} lqi_result_t;

static float clampf(float value, float minimum, float maximum)
{
    if (value > maximum)
    {
        return maximum;
    }
    if (value < minimum)
    {
        return minimum;
    }
    return value;
}

static float move_towards(float value, float target, float maximum_step)
{
    if (target > value + maximum_step)
    {
        return value + maximum_step;
    }
    if (target < value - maximum_step)
    {
        return value - maximum_step;
    }
    return target;
}

static int control_sign(float control)
{
    if (control > 0.0001f)
    {
        return 1;
    }
    if (control < -0.0001f)
    {
        return -1;
    }
    return 0;
}

static float lqi_state_feedback(float theta, float omega, float integral_error)
{
    return -LQI_K_THETA * theta -
           LQI_K_OMEGA * omega +
           LQI_K_I * integral_error;
}

static lqi_result_t lqi_update(lqi_controller_t *controller, float reference, float theta, float omega);
static bool update_position_gate(position_gate_t *gate,
                                 float error,
                                 float omega);
     
static float update_safe_actuator(safe_actuator_t *actuator,
                                  float requested_control,
                                  bool force_stop);


static esp_err_t lqi_math_self_test(void);



static float encoder_position_rad(int count)
{
    return ((float)count * TWO_PI_F) / ENCODER_COUNTS_PER_REVOLUTION;
}

static float encoder_speed_rad_s(int current_count, int previous_count)
{
    const int delta_count = current_count - previous_count;
    return ((float)delta_count * TWO_PI_F) /
           (ENCODER_COUNTS_PER_REVOLUTION * SAMPLE_TIME_S);
}

static void latch_safety_fault(float theta, float omega)
{
    setSpeed(0);
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

static void finish_test(void)
{
    setSpeed(0);
    ESP_LOGI(TAG, "Prueba LQI finalizada; motor detenido");
    ESP_LOGI(TAG, "Reinicia el ESP32 para repetirla");

    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    motor_t motor = {0};
    lqi_controller_t controller = {0};
    position_gate_t position_gate = {0};
    safe_actuator_t actuator = {0};

    ESP_ERROR_CHECK(lqi_math_self_test());
    ESP_LOGI(TAG, "Autoprueba matematica y de seguridad superada");

    motor_init();
    pcnt_init(&motor);
    setSpeed(0);

    int previous_count = 0;
    ESP_ERROR_CHECK(
        pcnt_unit_get_count(motor.g_pcnt1_unit, &previous_count));

    float filtered_omega = 0.0f;
    size_t reference_index = 0;
    uint32_t samples_at_reference = 0;
    uint32_t sample_number = 0;
    TickType_t last_wake_time = xTaskGetTickCount();

    ESP_LOGI(TAG,
             "Prueba segura: referencias -pi, +pi, -pi, 0 rad");
    ESP_LOGI(TAG,
             "Duraciones: 15 s al primer extremo, 20 s por recorrido completo y 15 s a cero");
    ESP_LOGI(TAG,
             "Formato: LQI: t,r,theta,omega_delta,omega_f,error,v,u_raw,u_cmd,pwm,aw,ok");

    while (true)
    {
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(SAMPLE_TIME_MS));

        int current_count = 0;
        esp_err_t err =
            pcnt_unit_get_count(motor.g_pcnt1_unit, &current_count);
        if (err != ESP_OK)
        {
            setSpeed(0);
            ESP_LOGE(TAG, "No se pudo leer el encoder: %s",
                     esp_err_to_name(err));
            finish_test();
        }

        const float theta = encoder_position_rad(current_count);
        const float omega_delta =
            encoder_speed_rad_s(current_count, previous_count);
        previous_count = current_count;
        filtered_omega = VELOCITY_FILTER_ALPHA * filtered_omega +
                         (1.0f - VELOCITY_FILTER_ALPHA) * omega_delta;

        if (!isfinite(theta) || !isfinite(omega_delta) ||
            !isfinite(filtered_omega) ||
            fabsf(theta) > POSITION_SAFETY_LIMIT_RAD ||
            fabsf(omega_delta) > SPEED_SAFETY_LIMIT_RAD_S)
        {
            latch_safety_fault(theta, omega_delta);
        }

        const reference_step_t *current_step =
            &reference_sequence[reference_index];
        const float reference = current_step->reference_rad;
        const float position_error = reference - theta;
        const bool position_reached =
            update_position_gate(&position_gate,
                                 position_error,
                                 filtered_omega);
        const bool inside_braking_band =
            fabsf(position_error) <= POSITION_EXIT_TOL_RAD;

        lqi_result_t result = {
            .error = position_error,
            .unsaturated_control =
                lqi_state_feedback(theta,
                                   filtered_omega,
                                   controller.integral_error),
            .requested_control = 0.0f,
            .anti_windup_active =
                position_reached || inside_braking_band,
        };

        if (!position_reached && !inside_braking_band)
        {
            result = lqi_update(&controller,
                                reference,
                                theta,
                                filtered_omega);

            /* No permite torque que aleje el motor de la referencia. */
            if (result.requested_control * position_error < 0.0f)
            {
                result.requested_control = 0.0f;
                result.anti_windup_active = true;
            }
        }

        const float applied_control =
            update_safe_actuator(&actuator,
                                 result.requested_control,
                                 position_reached || inside_braking_band);
        const int pwm_percent =
            (int)lroundf(100.0f * applied_control);
        setSpeed(pwm_percent);
        actuator.applied_control = (float)pwm_percent / 100.0f;

        if (sample_number % TELEMETRY_DECIMATION == 0U)
        {
            const float elapsed_time = (float)sample_number * SAMPLE_TIME_S;
            printf("LQI: %.3f,%.4f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d,%d\n",
                   elapsed_time,
                   reference,
                   theta,
                   omega_delta,
                   filtered_omega,
                   result.error,
                   controller.integral_error,
                   result.unsaturated_control,
                   result.requested_control,
                   pwm_percent,
                   result.anti_windup_active ? 1 : 0,
                   position_reached ? 1 : 0);
        }

        ++sample_number;
        ++samples_at_reference;
        if (samples_at_reference >= current_step->hold_samples)
        {
            samples_at_reference = 0;
            if (reference_index + 1U >=
                sizeof(reference_sequence) /
                    sizeof(reference_sequence[0]))
            {
                finish_test();
            }

            ++reference_index;
            ESP_LOGI(TAG,
                     "Nueva referencia: %.3f rad durante %.1f s",
                     reference_sequence[reference_index].reference_rad,
                     (float)reference_sequence[reference_index].hold_samples *
                         SAMPLE_TIME_S);
        }
    }
}




    static lqi_result_t lqi_update(lqi_controller_t *controller,
                                   float reference,
                                   float theta,
                                   float omega)
{
    lqi_result_t result = {0};
    result.error = reference - theta;

    const float candidate_integral =
        controller->integral_error + result.error;
    result.unsaturated_control =
        lqi_state_feedback(theta, omega, candidate_integral);

    /*
     * Proyecta el estado integral al intervalo que puede producir el
     * actuador. Asi se conserva v(k)=v(k-1)+e(k) sin windup.
     */
    const float state_feedback =
        LQI_K_THETA * theta + LQI_K_OMEGA * omega;
    const float integral_min =
        (CONTROL_MIN + state_feedback) / LQI_K_I;
    const float integral_max =
        (CONTROL_MAX + state_feedback) / LQI_K_I;
    const float limited_integral =
        clampf(candidate_integral, integral_min, integral_max);

    result.anti_windup_active =
        fabsf(limited_integral - candidate_integral) > 1e-6f;
    controller->integral_error = limited_integral;

    result.requested_control =
        lqi_state_feedback(theta, omega, controller->integral_error);
    result.requested_control =
        clampf(result.requested_control, CONTROL_MIN, CONTROL_MAX);
    return result;
}


                            
static bool update_position_gate(position_gate_t *gate,
                                 float error,
                                 float omega)
{
    if (gate->reached)
    {
        if (fabsf(error) >= POSITION_EXIT_TOL_RAD)
        {
            gate->reached = false;
            gate->dwell_samples = 0;
        }
        return gate->reached;
    }

    if (fabsf(error) <= POSITION_ENTER_TOL_RAD &&
        fabsf(omega) <= SPEED_TOL_RAD_S)
    {
        ++gate->dwell_samples;
        if (gate->dwell_samples >= SETTLED_DWELL_SAMPLES)
        {
            gate->reached = true;
        }
    }
    else
    {
        gate->dwell_samples = 0;
    }

    return gate->reached;
}



static float update_safe_actuator(safe_actuator_t *actuator,
                                  float requested_control,
                                  bool force_stop)
{
    if (force_stop)
    {
        actuator->applied_control = 0.0f;
        actuator->reversal_hold_samples = 0;
        actuator->reversing = false;
        return 0.0f;
    }

    requested_control =
        clampf(requested_control, CONTROL_MIN, CONTROL_MAX);

    if (actuator->reversal_hold_samples > 0U)
    {
        --actuator->reversal_hold_samples;
        actuator->applied_control = 0.0f;
        actuator->reversing = true;
        return 0.0f;
    }

    const int applied_sign = control_sign(actuator->applied_control);
    const int requested_sign = control_sign(requested_control);
    if (applied_sign != 0 && requested_sign != 0 &&
        applied_sign != requested_sign)
    {
        actuator->applied_control =
            move_towards(actuator->applied_control,
                         0.0f,
                         CONTROL_SLEW_PER_SAMPLE);
        actuator->reversing = true;

        if (control_sign(actuator->applied_control) == 0)
        {
            actuator->applied_control = 0.0f;
            actuator->reversal_hold_samples = REVERSAL_HOLD_SAMPLES;
        }
        return actuator->applied_control;
    }

    actuator->reversing = false;
    actuator->applied_control =
        move_towards(actuator->applied_control,
                     requested_control,
                     CONTROL_SLEW_PER_SAMPLE);
    return actuator->applied_control;
}



static esp_err_t lqi_math_self_test(void)
{
    const float tolerance = 1e-5f;
    if (fabsf(lqi_state_feedback(0.0f, 0.0f, 0.0f)) > tolerance ||
        fabsf(lqi_state_feedback(0.1f, 0.0f, 0.0f) + 0.85621f) > tolerance ||
        fabsf(lqi_state_feedback(0.0f, 1.0f, 0.0f) + 0.3637f) > tolerance ||
        fabsf(lqi_state_feedback(0.0f, 0.0f, 0.1f) - 0.19635f) > tolerance)
    {
        return ESP_FAIL;
    }

    position_gate_t gate = {0};
    for (uint32_t i = 0; i < SETTLED_DWELL_SAMPLES; ++i)
    {
        update_position_gate(&gate, 0.0f, 0.0f);
    }
    if (!gate.reached)
    {
        return ESP_FAIL;
    }

    safe_actuator_t actuator = {0};
    const float first_step =
        update_safe_actuator(&actuator, CONTROL_MAX, false);
    const float reversal_step =
        update_safe_actuator(&actuator, CONTROL_MIN, false);
    if (fabsf(first_step - CONTROL_SLEW_PER_SAMPLE) > tolerance ||
        fabsf(reversal_step) > tolerance || !actuator.reversing)
    {
        return ESP_FAIL;
    }

    return ESP_OK;
}

