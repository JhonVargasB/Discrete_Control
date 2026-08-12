#include "rls.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "dspm_mult.h"
#include "dsps_add.h"
#include "dsps_dotprod.h"
#include "dsps_mulc.h"
#include "dsps_sub.h"

#define RLS_VECTOR_LENGTH 2
#define RLS_MATRIX_LENGTH 4
#define RLS_STEP 1

//Funcion para verficar si los valores son finitos, para evitar errores de overflow
static bool rls_values_are_finite(const float *values, size_t length)
{
    for (size_t i = 0; i < length; ++i) {
        if (!isfinite(values[i])) {
            return false;
        }
    }
    return true;
}

esp_err_t rls_init(rls_t *rls, float lambda, float initial_covariance)
{
    if (rls == NULL || !isfinite(lambda) || lambda <= 0.0f || lambda > 1.0f ||
        !isfinite(initial_covariance) || initial_covariance <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    const rls_t initialized = {
        .theta = {0.0f, 0.0f},
        .P = {initial_covariance, 0.0f, 0.0f, initial_covariance},
        .lambda = lambda,
    };
    *rls = initialized;
    return ESP_OK;
}



esp_err_t rls_update(rls_t *rls,
                     const float phi[RLS_PARAMETER_COUNT],
                     float measured_output,
                     float *predicted_output,
                     float *prediction_error)
{
    if (rls == NULL || phi == NULL || predicted_output == NULL || prediction_error == NULL ||
        !isfinite(measured_output) || !isfinite(rls->lambda) ||
        rls->lambda <= 0.0f || rls->lambda > 1.0f ||
        !rls_values_are_finite(phi, RLS_VECTOR_LENGTH) ||
        !rls_values_are_finite(rls->theta, RLS_VECTOR_LENGTH) ||
        !rls_values_are_finite(rls->P, RLS_MATRIX_LENGTH)) {
        return ESP_ERR_INVALID_ARG;
    }

    float y_hat = 0.0f;
    esp_err_t err = dsps_dotprod_f32(phi, rls->theta, &y_hat, RLS_VECTOR_LENGTH);
    if (err != ESP_OK) {
        return err;
    }
    if (!isfinite(y_hat)) {
        return ESP_ERR_INVALID_STATE;
    }

    const float prediction_err = measured_output - y_hat;
    if (!isfinite(prediction_err)) {
        return ESP_ERR_INVALID_STATE;
    }

    /* P: 2x2, phi: 2x1, p_phi: 2x1. */
    float p_phi[RLS_VECTOR_LENGTH];
    err = dspm_mult_f32(rls->P, phi, p_phi, 2, 2, 1);
    if (err != ESP_OK) {
        return err;
    }

    float phi_t_p_phi = 0.0f;
    err = dsps_dotprod_f32(phi, p_phi, &phi_t_p_phi, RLS_VECTOR_LENGTH);
    if (err != ESP_OK) {
        return err;
    }

    const float denominator = rls->lambda + phi_t_p_phi;
    if (!isfinite(denominator) || fabsf(denominator) < RLS_DEN_MIN) {
        return ESP_ERR_INVALID_STATE;
    }

    const float inverse_denominator = 1.0f / denominator;
    const float inverse_lambda = 1.0f / rls->lambda;

    /* K: 2x1. */
    float gain[RLS_VECTOR_LENGTH];
    err = dsps_mulc_f32(p_phi, gain, RLS_VECTOR_LENGTH,
                        inverse_denominator, RLS_STEP, RLS_STEP);
    if (err != ESP_OK) {
        return err;
    }

    float parameter_correction[RLS_VECTOR_LENGTH];
    float theta_new[RLS_VECTOR_LENGTH];
    err = dsps_mulc_f32(gain, parameter_correction, RLS_VECTOR_LENGTH,
                        prediction_err, RLS_STEP, RLS_STEP);
    if (err != ESP_OK) {
        return err;
    }
    err = dsps_add_f32(rls->theta, parameter_correction, theta_new,
                       RLS_VECTOR_LENGTH, RLS_STEP, RLS_STEP, RLS_STEP);
    if (err != ESP_OK) {
        return err;
    }

    /* phi^T: 1x2, P: 2x2, phi_t_P: 1x2. */
    float phi_t_P[RLS_VECTOR_LENGTH];
    err = dspm_mult_f32(phi, rls->P, phi_t_P, 1, 2, 2);
    if (err != ESP_OK) {
        return err;
    }

    /* K: 2x1, phi_t_P: 1x2, covariance_correction: 2x2. */
    float covariance_correction[RLS_MATRIX_LENGTH];
    err = dspm_mult_f32(gain, phi_t_P, covariance_correction, 2, 1, 2);
    if (err != ESP_OK) {
        return err;
    }

    float covariance_difference[RLS_MATRIX_LENGTH];
    float P_new[RLS_MATRIX_LENGTH];
    err = dsps_sub_f32(rls->P, covariance_correction, covariance_difference,
                       RLS_MATRIX_LENGTH, RLS_STEP, RLS_STEP, RLS_STEP);
    if (err != ESP_OK) {
        return err;
    }
    err = dsps_mulc_f32(covariance_difference, P_new, RLS_MATRIX_LENGTH,
                        inverse_lambda, RLS_STEP, RLS_STEP);
    if (err != ESP_OK) {
        return err;
    }

    const float off_diagonal = 0.5f * (P_new[1] + P_new[2]);
    P_new[1] = off_diagonal;
    P_new[2] = off_diagonal;

    if (!rls_values_are_finite(gain, RLS_VECTOR_LENGTH) ||
        !rls_values_are_finite(theta_new, RLS_VECTOR_LENGTH) ||
        !rls_values_are_finite(P_new, RLS_MATRIX_LENGTH)) {
        return ESP_ERR_INVALID_STATE;
    }

    memcpy(rls->theta, theta_new, sizeof(theta_new));
    memcpy(rls->P, P_new, sizeof(P_new));
    *predicted_output = y_hat;
    *prediction_error = prediction_err;
    return ESP_OK;
}

esp_err_t rls_print_parameters(const rls_t *rls,
                               float measured_output,
                               float predicted_output)
{
    if (rls == NULL || !isfinite(measured_output) || !isfinite(predicted_output) ||
        !rls_values_are_finite(rls->theta, RLS_VECTOR_LENGTH)) {
        return ESP_ERR_INVALID_ARG;
    }

    printf("RLS: y=%.6f, y'=%.6f, a=%.6f, b=%.6f\n",
           measured_output,
           predicted_output,
           rls->theta[0],
           rls->theta[1]);
    return ESP_OK;
}
