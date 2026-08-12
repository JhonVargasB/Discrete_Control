#include <string.h>

#include "esp_rom_sys.h"
#include "pwm_ctrl.h"

static esp_err_t validate_config(const motor_config_t *config)
{
    if (config == NULL ||
        !GPIO_IS_VALID_OUTPUT_GPIO(config->pwm_gpio) ||
        !GPIO_IS_VALID_OUTPUT_GPIO(config->direction_gpio_1) ||
        !GPIO_IS_VALID_OUTPUT_GPIO(config->direction_gpio_2) ||
        !GPIO_IS_VALID_GPIO(config->encoder_gpio_a) ||
        !GPIO_IS_VALID_GPIO(config->encoder_gpio_b) ||
        (unsigned)config->ledc_mode >= LEDC_SPEED_MODE_MAX ||
        (unsigned)config->ledc_timer >= LEDC_TIMER_MAX ||
        (unsigned)config->ledc_channel >= LEDC_CHANNEL_MAX ||
        config->ledc_duty_resolution < LEDC_TIMER_1_BIT ||
        config->ledc_duty_resolution >= LEDC_TIMER_BIT_MAX ||
        config->pwm_frequency_hz == 0U ||
        config->pcnt_low_limit >= 0 ||
        config->pcnt_high_limit <= 0 ||
        config->pcnt_low_limit >= config->pcnt_high_limit ||
        config->pcnt_glitch_filter_ns == 0U)
    {
        return ESP_ERR_INVALID_ARG;
    }

    const gpio_num_t assigned_pins[] = {
        config->pwm_gpio,
        config->direction_gpio_1,
        config->direction_gpio_2,
        config->encoder_gpio_a,
        config->encoder_gpio_b,
    };
    for (unsigned i = 0; i < 5U; ++i)
    {
        for (unsigned j = i + 1U; j < 5U; ++j)
        {
            if (assigned_pins[i] == assigned_pins[j])
            {
                return ESP_ERR_INVALID_ARG;
            }
        }
    }

    return ESP_OK;
}

static uint32_t maximum_ledc_duty(ledc_timer_bit_t resolution)
{
    return (1UL << (uint32_t)resolution) - 1UL;
}

static esp_err_t apply_pwm_percent(motor_t *motor, uint8_t duty_percent)
{
    if (motor == NULL || !motor->pwm_initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }

    const uint32_t maximum_duty =
        maximum_ledc_duty(motor->config.ledc_duty_resolution);
    const uint32_t duty =
        ((uint32_t)duty_percent * maximum_duty) / 100U;

    esp_err_t err = ledc_set_duty(motor->config.ledc_mode,
                                  motor->config.ledc_channel,
                                  duty);
    if (err != ESP_OK)
    {
        return err;
    }

    return ledc_update_duty(motor->config.ledc_mode,
                            motor->config.ledc_channel);
}

static esp_err_t set_bridge_direction(motor_t *motor, int direction_sign)
{
    int level_1 = direction_sign > 0 ? 1 : 0;
    int level_2 = direction_sign > 0 ? 0 : 1;

    esp_err_t err = gpio_set_level(motor->config.direction_gpio_1,
                                   level_1);
    if (err != ESP_OK)
    {
        return err;
    }

    return gpio_set_level(motor->config.direction_gpio_2, level_2);
}

static esp_err_t init_pwm(motor_t *motor)
{
    const ledc_timer_config_t timer_config = {
        .speed_mode = motor->config.ledc_mode,
        .duty_resolution = motor->config.ledc_duty_resolution,
        .timer_num = motor->config.ledc_timer,
        .freq_hz = motor->config.pwm_frequency_hz,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    esp_err_t err = ledc_timer_config(&timer_config);
    if (err != ESP_OK)
    {
        return err;
    }

    const ledc_channel_config_t channel_config = {
        .gpio_num = motor->config.pwm_gpio,
        .speed_mode = motor->config.ledc_mode,
        .channel = motor->config.ledc_channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = motor->config.ledc_timer,
        .duty = 0,
        .hpoint = 0,
    };

    err = ledc_channel_config(&channel_config);
    if (err != ESP_OK)
    {
        return err;
    }
    motor->pwm_initialized = true;

    const gpio_config_t direction_config = {
        .pin_bit_mask =
            (1ULL << motor->config.direction_gpio_1) |
            (1ULL << motor->config.direction_gpio_2),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    err = gpio_config(&direction_config);
    if (err != ESP_OK)
    {
        return err;
    }

    return motor_stop(motor);
}

static esp_err_t init_encoder(motor_t *motor)
{
    const pcnt_unit_config_t unit_config = {
        .low_limit = motor->config.pcnt_low_limit,
        .high_limit = motor->config.pcnt_high_limit,
        .intr_priority = 0,
        .flags.accum_count = true,
    };

    esp_err_t err = pcnt_new_unit(&unit_config, &motor->pcnt_unit);
    if (err != ESP_OK)
    {
        return err;
    }

    err = pcnt_unit_add_watch_point(motor->pcnt_unit,
                                    motor->config.pcnt_high_limit);
    if (err != ESP_OK)
    {
        return err;
    }

    err = pcnt_unit_add_watch_point(motor->pcnt_unit,
                                    motor->config.pcnt_low_limit);
    if (err != ESP_OK)
    {
        return err;
    }

    const pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = motor->config.pcnt_glitch_filter_ns,
    };
    err = pcnt_unit_set_glitch_filter(motor->pcnt_unit,
                                      &filter_config);
    if (err != ESP_OK)
    {
        return err;
    }

    const pcnt_chan_config_t channel_a_config = {
        .edge_gpio_num = motor->config.encoder_gpio_a,
        .level_gpio_num = motor->config.encoder_gpio_b,
    };
    const pcnt_chan_config_t channel_b_config = {
        .edge_gpio_num = motor->config.encoder_gpio_b,
        .level_gpio_num = motor->config.encoder_gpio_a,
    };

    err = pcnt_new_channel(motor->pcnt_unit,
                           &channel_a_config,
                           &motor->pcnt_channel_a);
    if (err != ESP_OK)
    {
        return err;
    }

    err = pcnt_new_channel(motor->pcnt_unit,
                           &channel_b_config,
                           &motor->pcnt_channel_b);
    if (err != ESP_OK)
    {
        return err;
    }

    err = pcnt_channel_set_edge_action(
        motor->pcnt_channel_a,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    if (err != ESP_OK)
    {
        return err;
    }

    err = pcnt_channel_set_level_action(
        motor->pcnt_channel_a,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    if (err != ESP_OK)
    {
        return err;
    }

    err = pcnt_channel_set_edge_action(
        motor->pcnt_channel_b,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    if (err != ESP_OK)
    {
        return err;
    }

    err = pcnt_channel_set_level_action(
        motor->pcnt_channel_b,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    if (err != ESP_OK)
    {
        return err;
    }

    err = pcnt_unit_enable(motor->pcnt_unit);
    if (err != ESP_OK)
    {
        return err;
    }

    err = pcnt_unit_clear_count(motor->pcnt_unit);
    if (err != ESP_OK)
    {
        return err;
    }

    err = pcnt_unit_start(motor->pcnt_unit);
    if (err != ESP_OK)
    {
        return err;
    }

    motor->encoder_initialized = true;
    return ESP_OK;
}

esp_err_t motor_init(motor_t *motor, const motor_config_t *config)
{
    if (motor == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (motor->pwm_initialized ||
        motor->encoder_initialized ||
        motor->pcnt_unit != NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = validate_config(config);
    if (err != ESP_OK)
    {
        return err;
    }

    memset(motor, 0, sizeof(*motor));
    motor->config = *config;

    err = init_pwm(motor);
    if (err != ESP_OK)
    {
        motor_deinit(motor);
        return err;
    }

    err = init_encoder(motor);
    if (err != ESP_OK)
    {
        motor_deinit(motor);
        return err;
    }

    return ESP_OK;
}

esp_err_t motor_set_speed(motor_t *motor, int speed_percent)
{
    if (motor == NULL || !motor->pwm_initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (speed_percent > 100)
    {
        speed_percent = 100;
    }
    else if (speed_percent < -100)
    {
        speed_percent = -100;
    }

    if (speed_percent == 0)
    {
        return motor_stop(motor);
    }

    const int requested_direction_sign = speed_percent > 0 ? 1 : -1;
    if (motor->current_direction_sign != 0 &&
        requested_direction_sign != motor->current_direction_sign)
    {
        esp_err_t err = apply_pwm_percent(motor, 0);
        if (err != ESP_OK)
        {
            return err;
        }

        err = gpio_set_level(motor->config.direction_gpio_1, 0);
        if (err != ESP_OK)
        {
            return err;
        }
        err = gpio_set_level(motor->config.direction_gpio_2, 0);
        if (err != ESP_OK)
        {
            return err;
        }
        esp_rom_delay_us(20);
    }

    esp_err_t err = set_bridge_direction(motor,
                                         requested_direction_sign);
    if (err != ESP_OK)
    {
        return err;
    }
    motor->current_direction_sign = requested_direction_sign;

    const uint8_t duty_percent = (uint8_t)(
        speed_percent > 0 ? speed_percent : -speed_percent);
    return apply_pwm_percent(motor, duty_percent);
}

esp_err_t motor_stop(motor_t *motor)
{
    if (motor == NULL || !motor->pwm_initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t first_error = apply_pwm_percent(motor, 0);
    esp_err_t err = gpio_set_level(motor->config.direction_gpio_1, 0);
    if (first_error == ESP_OK && err != ESP_OK)
    {
        first_error = err;
    }

    err = gpio_set_level(motor->config.direction_gpio_2, 0);
    if (first_error == ESP_OK && err != ESP_OK)
    {
        first_error = err;
    }

    motor->current_direction_sign = 0;
    return first_error;
}

esp_err_t motor_get_count(const motor_t *motor, int *count)
{
    if (motor == NULL || count == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!motor->encoder_initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }

    return pcnt_unit_get_count(motor->pcnt_unit, count);
}

esp_err_t motor_clear_count(motor_t *motor)
{
    if (motor == NULL || !motor->encoder_initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }

    return pcnt_unit_clear_count(motor->pcnt_unit);
}

esp_err_t motor_deinit(motor_t *motor)
{
    if (motor == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t first_error = ESP_OK;
    if (motor->pwm_initialized)
    {
        first_error = motor_stop(motor);
        esp_err_t err = ledc_stop(motor->config.ledc_mode,
                                  motor->config.ledc_channel,
                                  0);
        if (first_error == ESP_OK && err != ESP_OK)
        {
            first_error = err;
        }
    }

    if (motor->pcnt_unit != NULL)
    {
        pcnt_unit_stop(motor->pcnt_unit);
        pcnt_unit_disable(motor->pcnt_unit);

        if (motor->pcnt_channel_a != NULL)
        {
            pcnt_del_channel(motor->pcnt_channel_a);
        }
        if (motor->pcnt_channel_b != NULL)
        {
            pcnt_del_channel(motor->pcnt_channel_b);
        }

        esp_err_t err = pcnt_del_unit(motor->pcnt_unit);
        if (first_error == ESP_OK && err != ESP_OK)
        {
            first_error = err;
        }
    }

    memset(motor, 0, sizeof(*motor));
    return first_error;
}
