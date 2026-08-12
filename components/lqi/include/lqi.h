#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct
{
    float theta;
    float omega;
    float v;
} lqi_states_t;

typedef struct
{
    float r;
    float e;
    float omega_measured;
    float u;
    float u_applied;
    int pwm_percent;
    bool integrator_frozen;
    bool anti_windup_active;
    bool position_reached;
} lqi_signals_t;

typedef struct
{
    float v_prop;
    float u_prop;
} lqi_proposed_signals_t;

typedef struct
{
    lqi_states_t x;
    lqi_signals_t signals;
    lqi_proposed_signals_t proposed;
} lqi_variables_t;

/*
 * bool reached
 * uint32_t dwell_samples
 * */
typedef struct
{
    bool reached;
    uint32_t dwell_samples;
} position_gate_t;

/*
 * float applied_control
 * uint32_t reversal_hold_samples
 * bool reversing
 * */
typedef struct
{
    float applied_control;
    uint32_t reversal_hold_samples;
    bool reversing;
} safe_actuator_t;

/** Parametros de diseno y seguridad propios de un controlador LQI. */
typedef struct
{
    float k_theta;
    float k_omega;
    float ki;

    uint32_t sample_time_ms;
    float encoder_counts_per_revolution;

    float control_min;
    float control_max;
    float control_slew_per_sample;
    uint32_t reversal_hold_samples;

    float position_enter_tolerance_rad;
    float position_exit_tolerance_rad;
    float speed_enter_tolerance_rad_s;
    float speed_exit_tolerance_rad_s;
    uint32_t settled_dwell_samples;

    float velocity_filter_alpha;
    float reference_safety_limit_rad;
    float position_safety_limit_rad;
    float speed_safety_limit_rad_s;
} lqi_config_t;

/** Crea una configuracion completa indicando las tres ganancias LQI. */
#define LQI_CONFIG_WITH_GAINS(K_THETA_VALUE,      \
                              K_OMEGA_VALUE,      \
                              KI_VALUE)           \
    {                                             \
        .k_theta = (K_THETA_VALUE),               \
        .k_omega = (K_OMEGA_VALUE),               \
        .ki = (KI_VALUE),                         \
        .sample_time_ms = 20U,                    \
        .encoder_counts_per_revolution = 8341.0f, \
        .control_min = -0.15f,                    \
        .control_max = 0.15f,                     \
        .control_slew_per_sample = 0.04f,         \
        .reversal_hold_samples = 5U,              \
        .position_enter_tolerance_rad = 0.035f,   \
        .position_exit_tolerance_rad = 0.040f,    \
        .speed_enter_tolerance_rad_s = 0.04f,     \
        .speed_exit_tolerance_rad_s = 0.10f,      \
        .settled_dwell_samples = 5U,              \
        .velocity_filter_alpha = 0.60f,           \
        .reference_safety_limit_rad = 3.30f,      \
        .position_safety_limit_rad = 3.50f,       \
        .speed_safety_limit_rad_s = 4.0f,         \
    }

/** Valores usados hasta ahora por el controlador LQI. */
#define LQI_CONFIG_DEFAULT() \
    LQI_CONFIG_WITH_GAINS(8.5621f, 0.3637f, 1.9635f)

/** Instancia completa e independiente de un controlador LQI. */
typedef struct
{
    lqi_config_t config;
    lqi_variables_t variables;
    position_gate_t position_gate;
    safe_actuator_t actuator;
    bool reference_initialized;
} lqi_controller_t;

#define TELEMETRY_DECIMATION 5U

/** Inicializa en cero todos los estados y señales de una instancia LQI. */
esp_err_t lqi_controller_init(lqi_controller_t *controller,
                              const lqi_config_t *config);

/**
 * Asigna una referencia segura. Si cambia, reinicia la deteccion de
 * posicion alcanzada sin borrar el estado integral.
 */
esp_err_t lqi_controller_set_reference(lqi_controller_t *controller,
                                       float reference_rad);

/**
 * Ejecuta un periodo completo del controlador usando theta y la velocidad
 * medida. El PWM resultante queda en variables.signals.pwm_percent.
 */
esp_err_t lqi_controller_step(lqi_controller_t *controller,
                              float theta,
                              float measured_omega);

/**
 * Calcula v_prop y u_prop, actualiza el integrador con anti-windup y
 * obtiene u. Si freeze_integrator es true, conserva v y entrega u = 0.
 * El llamador debe actualizar signals.e antes de invocar esta funcion.
 */
void lqi_update(lqi_variables_t *lqi,
                const lqi_config_t *config,
                bool freeze_integrator);

/**
 * Determina si la posicion fue alcanzada usando tolerancia de entrada,
 * velocidad baja, tiempo de permanencia e histeresis de salida.
 */
bool update_position_gate(position_gate_t *gate,
                          const lqi_config_t *config,
                          float error,
                          float omega);

/**
 * Limita la accion, aplica una rampa de PWM y evita inversiones bruscas.
 * Devuelve la accion normalizada que realmente puede aplicarse al motor.
 */
float update_safe_actuator(safe_actuator_t *actuator,
                           const lqi_config_t *config,
                           float requested_control,
                           bool force_stop);

/** Convierte el conteo acumulado del encoder a posicion angular [rad]. */
float encoder_position_rad(const lqi_config_t *config, int count);

/** Calcula la velocidad entre dos muestras consecutivas [rad/s]. */
float encoder_speed_rad_s(const lqi_config_t *config,
                          int current_count,
                          int previous_count);
