#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "shared.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /** Configuracion de hardware asignada a una instancia de motor. */
    typedef struct
    {
        gpio_num_t pwm_gpio;
        gpio_num_t direction_gpio_1;
        gpio_num_t direction_gpio_2;
        gpio_num_t encoder_gpio_a;
        gpio_num_t encoder_gpio_b;

        ledc_mode_t ledc_mode;
        ledc_timer_t ledc_timer;
        ledc_channel_t ledc_channel;
        ledc_timer_bit_t ledc_duty_resolution;
        uint32_t pwm_frequency_hz;

        int pcnt_low_limit;
        int pcnt_high_limit;
        uint32_t pcnt_glitch_filter_ns;
    } motor_config_t;

    /** Estado y recursos privados de una instancia de motor. */
    typedef struct
    {
        motor_config_t config;

        pcnt_unit_handle_t pcnt_unit;
        pcnt_channel_handle_t pcnt_channel_a;
        pcnt_channel_handle_t pcnt_channel_b;

        int current_direction_sign;
        bool pwm_initialized;
        bool encoder_initialized;
    } motor_t;

/** Configuracion correspondiente al motor A de la placa actual.
 * PWMA GPIO_NUM_2
 * AIN2 GPIO_NUM_3
 * AIN1 GPIO_NUM_4
 * C1-VERDE  GPIO_NUM_44
 * C2-AMARILLO GPIO_NUM_7
*/

#define MOTOR_CONFIG_DEFAULT_A()                   \
    {                                              \
        .pwm_gpio = PIN_PWM_A,                    \
        .direction_gpio_1 = PIN_DIR_A_1,           \
        .direction_gpio_2 = PIN_DIR_A_2,            \
        .encoder_gpio_a = PIN_ENCODER_A_C1,              \
        .encoder_gpio_b = PIN_ENCODER_A_C2,              \
        .ledc_mode = LEDC_LOW_SPEED_MODE,          \
        .ledc_timer = LEDC_TIMER_0,                \
        .ledc_channel = LEDC_CHANNEL_0,            \
        .ledc_duty_resolution = LEDC_TIMER_10_BIT, \
        .pwm_frequency_hz = 60000U,                \
        .pcnt_low_limit = -30000,                  \
        .pcnt_high_limit = 30000,                  \
        .pcnt_glitch_filter_ns = 2000U,            \
    }

/** Configuracion correspondiente al motor B de la placa actual.
 * PWMB GPIO_NUM_43
 * BIN2 GPIO_NUM_6
 * BIN1 GPIO_NUM_5
 * C1-VERDE  GPIO_NUM_8
 * C2-AMARILLO GPIO_NUM_9
*/

#define MOTOR_CONFIG_DEFAULT_B()                   \
    {                                              \
        .pwm_gpio = PIN_PWM_B,                    \
        .direction_gpio_1 = PIN_DIR_B_1,           \
        .direction_gpio_2 = PIN_DIR_B_2,            \
        .encoder_gpio_a = PIN_ENCODER_B_C2,              \
        .encoder_gpio_b = PIN_ENCODER_B_C1,             \
        .ledc_mode = LEDC_LOW_SPEED_MODE,          \
        .ledc_timer = LEDC_TIMER_0,                \
        .ledc_channel = LEDC_CHANNEL_1,            \
        .ledc_duty_resolution = LEDC_TIMER_10_BIT, \
        .pwm_frequency_hz = 60000U,                \
        .pcnt_low_limit = -30000,                  \
        .pcnt_high_limit = 30000,                  \
        .pcnt_glitch_filter_ns = 2000U,            \
    }

    /**
     * Inicializa el PWM, el puente H y un PCNT independiente para el motor.
     * motor debe comenzar inicializado en cero. Dos motores pueden compartir
     * ledc_timer si usan la misma frecuencia y resolucion, pero cada uno debe
     * tener su propio ledc_channel, pines y recursos PCNT.
     */
    esp_err_t motor_init(motor_t *motor, const motor_config_t *config);

    /** Aplica velocidad firmada en porcentaje, saturada al intervalo [-100, 100]. */
    esp_err_t motor_set_speed(motor_t *motor, int speed_percent);

    /** Coloca PWM y pines de direccion en cero. */
    esp_err_t motor_stop(motor_t *motor);

    /** Lee el conteo acumulado del encoder asociado a esta instancia. */
    esp_err_t motor_get_count(const motor_t *motor, int *count);

    /** Reinicia en cero el contador del encoder asociado a esta instancia. */
    esp_err_t motor_clear_count(motor_t *motor);

    /** Libera la unidad PCNT y detiene la salida PWM de esta instancia. */
    esp_err_t motor_deinit(motor_t *motor);

#ifdef __cplusplus
}
#endif
