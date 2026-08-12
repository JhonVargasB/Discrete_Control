#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RLS_PARAMETER_COUNT 2
#define RLS_DEN_MIN 1e-8f

typedef struct
{
    /* theta = [a, b]^T for y(k) = a*y(k-1) + b*u(k-1). */
    float theta[RLS_PARAMETER_COUNT];

    /* 2x2 covariance matrix in row-major order: P00, P01, P10, P11. */
    float P[RLS_PARAMETER_COUNT * RLS_PARAMETER_COUNT];

    float lambda;
} rls_t;

esp_err_t rls_init(rls_t *rls, float lambda, float initial_covariance);

esp_err_t rls_update(rls_t *rls,
                     const float phi[RLS_PARAMETER_COUNT],
                     float measured_output,
                     float *predicted_output,
                     float *prediction_error);

/** Imprime la salida medida y, la predicción y', y los parámetros a y b. */
esp_err_t rls_print_parameters(const rls_t *rls,
                               float measured_output,
                               float predicted_output);

#ifdef __cplusplus
}
#endif
