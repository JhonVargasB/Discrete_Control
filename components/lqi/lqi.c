#include <math.h>
#include <string.h>

#include "lqi.h"

#define REFERENCE_CHANGE_EPSILON_RAD 1e-6f
#define TWO_PI_F 6.28318530717958647692f

/* Restringe value al intervalo cerrado [minimum, maximum]. */
static float clampf(float value, float minimum, float maximum)
{
    if (value > maximum)
        return maximum;

    if (value < minimum)
        return minimum;

    return value;
}

/* Comprueba que la configuracion pueda usarse sin divisiones invalidas. */
static bool is_valid_config(const lqi_config_t *config)
{
    if (config == NULL ||
        !isfinite(config->k_theta) ||
        !isfinite(config->k_omega) ||
        !isfinite(config->ki) || config->ki <= 0.0f ||
        config->sample_time_ms == 0U ||
        !isfinite(config->encoder_counts_per_revolution) ||
        config->encoder_counts_per_revolution <= 0.0f ||
        !isfinite(config->control_min) ||
        !isfinite(config->control_max) ||
        config->control_min < -1.0f ||
        config->control_min >= 0.0f ||
        config->control_max <= 0.0f ||
        config->control_max > 1.0f ||
        config->control_min >= config->control_max ||
        !isfinite(config->control_slew_per_sample) ||
        config->control_slew_per_sample <= 0.0f ||
        !isfinite(config->position_enter_tolerance_rad) ||
        !isfinite(config->position_exit_tolerance_rad) ||
        config->position_enter_tolerance_rad < 0.0f ||
        config->position_enter_tolerance_rad >=
            config->position_exit_tolerance_rad ||
        !isfinite(config->speed_enter_tolerance_rad_s) ||
        !isfinite(config->speed_exit_tolerance_rad_s) ||
        config->speed_enter_tolerance_rad_s < 0.0f ||
        config->speed_enter_tolerance_rad_s >=
            config->speed_exit_tolerance_rad_s ||
        config->settled_dwell_samples == 0U ||
        !isfinite(config->velocity_filter_alpha) ||
        config->velocity_filter_alpha < 0.0f ||
        config->velocity_filter_alpha >= 1.0f ||
        !isfinite(config->reference_safety_limit_rad) ||
        !isfinite(config->position_safety_limit_rad) ||
        !isfinite(config->speed_safety_limit_rad_s) ||
        config->reference_safety_limit_rad <= 0.0f ||
        config->position_safety_limit_rad <
            config->reference_safety_limit_rad ||
        config->speed_safety_limit_rad_s <= 0.0f ||
        config->position_exit_tolerance_rad >
            config->position_safety_limit_rad ||
        config->speed_exit_tolerance_rad_s >
            config->speed_safety_limit_rad_s)
    {
        return false;
    }

    return true;
}

/* Acerca value a target sin cambiar mas de maximum_step por llamada. */
static float move_towards(float value, float target, float maximum_step)
{
    if (target > value + maximum_step)
        return value + maximum_step;

    if (target < value - maximum_step)
        return value - maximum_step;

    return target;
}

/* Devuelve -1, 0 o +1 para detectar cambios de direccion del control. */
static int control_sign(float control)
{
    if (control > 0.0001f)
        return 1;

    if (control < -0.0001f)
        return -1;

    return 0;
}

/* Aplica u = -K_theta*theta - K_omega*omega + Ki*v. */
static float calculate_lqi_control(const lqi_states_t *x,
                                   const lqi_config_t *config)
{
    return -config->k_theta * x->theta -
           config->k_omega * x->omega +
           config->ki * x->v;
}

/*
 * Ejecuta una actualizacion del LQI. Primero calcula las variables
 * propuestas; despues acepta o limita v_prop, salvo que el integrador
 * deba permanecer congelado porque la posicion ya fue alcanzada.
 */
void lqi_update(lqi_variables_t *lqi,
                const lqi_config_t *config,
                bool freeze_integrator)
{
    lqi->proposed.v_prop = lqi->x.v + lqi->signals.e; // v_prop = v + e

    lqi_states_t proposed_states = lqi->x;
    proposed_states.v = lqi->proposed.v_prop;
    lqi->proposed.u_prop = calculate_lqi_control(&proposed_states,
                                                 config);

    /* Al alcanzar la posicion se conserva v y no se aplica control. */
    lqi->signals.integrator_frozen = freeze_integrator;
    lqi->signals.anti_windup_active = false;
    lqi->signals.u = 0.0f;

    if (lqi->signals.integrator_frozen)
    {
        return;
    }

    /*
     * Proyecta el estado integral al intervalo que puede producir el
     * actuador. Asi se conserva v(k)=v(k-1)+e(k) sin windup.
     */
    const float state_feedback =
        config->k_theta * lqi->x.theta +
        config->k_omega * lqi->x.omega;

    const float integral_min =
        (config->control_min + state_feedback) / config->ki;
    const float integral_max =
        (config->control_max + state_feedback) / config->ki;

    const float limited_v = clampf(lqi->proposed.v_prop, integral_min, integral_max);

    lqi->signals.anti_windup_active = fabsf(limited_v - lqi->proposed.v_prop) > 1e-6f;
    lqi->x.v = limited_v;

    lqi->signals.u = calculate_lqi_control(&lqi->x, config);
    lqi->signals.u = clampf(lqi->signals.u,
                            config->control_min,
                            config->control_max);
}

/*
 * Usa dos tolerancias para evitar conmutaciones: una menor para entrar
 * al estado reached y otra mayor para salir. Tambien exige velocidad
 * baja durante varias muestras consecutivas.
 */
bool update_position_gate(position_gate_t *gate,
                          const lqi_config_t *config,
                          float error,
                          float omega)
{
    if (gate->reached)
    {
        if (fabsf(error) >= config->position_exit_tolerance_rad ||
            fabsf(omega) >= config->speed_exit_tolerance_rad_s)
        {
            gate->reached = false;
            gate->dwell_samples = 0;
        }
        return gate->reached;
    }

    if (fabsf(error) <= config->position_enter_tolerance_rad &&
        fabsf(omega) <= config->speed_enter_tolerance_rad_s)
    {
        ++gate->dwell_samples;
        if (gate->dwell_samples >= config->settled_dwell_samples)
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

/*
 * Convierte la solicitud del LQI en una accion segura: satura el rango,
 * limita la variacion entre muestras y obliga a pasar por cero antes de
 * invertir el sentido del motor.
 */
float update_safe_actuator(safe_actuator_t *actuator,
                           const lqi_config_t *config,
                           float requested_control,
                           bool force_stop)
{
    /* Una posicion alcanzada exige PWM cero inmediatamente. */
    if (force_stop)
    {
        actuator->applied_control = 0.0f;
        actuator->reversal_hold_samples = 0;
        actuator->reversing = false;
        return 0.0f;
    }
    /* Limita nuevamente la entrada para proteger la interfaz del actuador. */
    requested_control = clampf(requested_control,
                               config->control_min,
                               config->control_max);

    /* Mantiene PWM cero durante la pausa previa al cambio de sentido. */
    if (actuator->reversal_hold_samples > 0U)
    {
        --actuator->reversal_hold_samples;
        actuator->applied_control = 0.0f;
        actuator->reversing = true;
        return 0.0f;
    }

    /* Detecta si la solicitud pretende invertir el sentido actual. */
    const int applied_sign = control_sign(actuator->applied_control);
    const int requested_sign = control_sign(requested_control);

    if (applied_sign != 0 && requested_sign != 0 &&
        applied_sign != requested_sign)
    {
        /* Antes de invertir, reduce gradualmente el control hasta cero. */
        actuator->applied_control = move_towards(actuator->applied_control,
                                                 0.0f,
                                                 config->control_slew_per_sample);
        actuator->reversing = true;
        if (control_sign(actuator->applied_control) == 0)
        {
            actuator->applied_control = 0.0f;
            actuator->reversal_hold_samples =
                config->reversal_hold_samples;
        }
        return actuator->applied_control;
    }
    /* Sin inversion pendiente, sigue la solicitud usando la rampa. */
    actuator->reversing = false;
    actuator->applied_control =
        move_towards(actuator->applied_control,
                     requested_control,
                     config->control_slew_per_sample);
    return actuator->applied_control;
}

/* Convierte las cuentas acumuladas en posicion angular. */
float encoder_position_rad(const lqi_config_t *config, int count)
{
    return ((float)count * TWO_PI_F) /
           config->encoder_counts_per_revolution;
}

/* Calcula velocidad por diferencia finita usando el periodo de muestreo. */
float encoder_speed_rad_s(const lqi_config_t *config,
                          int current_count,
                          int previous_count)
{
    const int delta_count = current_count - previous_count;
    const float sample_time_s =
        (float)config->sample_time_ms / 1000.0f;
    return ((float)delta_count * TWO_PI_F) /
           (config->encoder_counts_per_revolution * sample_time_s);
}

esp_err_t lqi_controller_init(lqi_controller_t *controller,
                              const lqi_config_t *config)
{
    if (controller == NULL || !is_valid_config(config))
    {
        return ESP_ERR_INVALID_ARG;
    }

    const lqi_config_t config_copy = *config;
    memset(controller, 0, sizeof(*controller));
    controller->config = config_copy;
    return ESP_OK;
}

esp_err_t lqi_controller_set_reference(lqi_controller_t *controller, float reference_rad)
{
    if (controller == NULL || !isfinite(reference_rad) ||
        fabsf(reference_rad) >
            controller->config.reference_safety_limit_rad)
    {
        return ESP_ERR_INVALID_ARG;
    }

    lqi_variables_t *variables = &controller->variables;

    if (!controller->reference_initialized || fabsf(reference_rad - variables->signals.r) > REFERENCE_CHANGE_EPSILON_RAD)
    {
        controller->position_gate.reached = false;   // La posicion no fue alcanzada
        controller->position_gate.dwell_samples = 0; // reiniciamos la cuenta de position_gate
        variables->signals.position_reached = false;
        variables->signals.integrator_frozen = false;
    }

    variables->signals.r = reference_rad;
    controller->reference_initialized = true;
    return ESP_OK;
}

esp_err_t lqi_controller_step(lqi_controller_t *controller, float theta, float measured_omega)
{
    // Validacion de las entradas
    if (controller == NULL)
        return ESP_ERR_INVALID_ARG;

    if (!controller->reference_initialized)
        return ESP_ERR_INVALID_STATE;

    if (!isfinite(theta) || !isfinite(measured_omega) ||
        fabsf(theta) > controller->config.position_safety_limit_rad ||
        fabsf(measured_omega) >
            controller->config.speed_safety_limit_rad_s)
    {
        return ESP_ERR_INVALID_ARG;
    }

    lqi_variables_t *variables = &controller->variables;

    variables->x.theta = theta;
    variables->signals.omega_measured = measured_omega;
    // Filtro pasabajas para la velocidad angular
    variables->x.omega =
        controller->config.velocity_filter_alpha * variables->x.omega +
        (1.0f - controller->config.velocity_filter_alpha) *
            measured_omega;
    if (!isfinite(variables->x.omega))
        return ESP_ERR_INVALID_ARG;
    // Calculo del error
    variables->signals.e = variables->signals.r - variables->x.theta;
    // Se llego a la posicion deseada?
    variables->signals.position_reached = update_position_gate(&controller->position_gate,
                                                               &controller->config,
                                                               variables->signals.e,
                                                               variables->x.omega);
    // Actualiza u[k] y v[k]
    lqi_update(variables,
               &controller->config,
               variables->signals.position_reached);

    /* Proteccion conservadora adicional a la ley LQI. evita que se aleje  */
    if (!variables->signals.integrator_frozen && variables->signals.u * variables->signals.e < 0.0f)
    {
        variables->signals.u = 0.0f;
    }
    // Limitamos el u aplciado, contra cambio de direccion y valores extremos
    variables->signals.u_applied = update_safe_actuator(&controller->actuator,
                                                        &controller->config,
                                                        variables->signals.u,
                                                        variables->signals.position_reached);
    // Valor del pwm actualziado
    variables->signals.pwm_percent = (int)lroundf(100.0f * variables->signals.u_applied);
    // seañl de control final , limpida y filtrada
    controller->actuator.applied_control = (float)variables->signals.pwm_percent / 100.0f;
    //Actualiza el valor de u
    variables->signals.u_applied = controller->actuator.applied_control;

    return ESP_OK;
}
