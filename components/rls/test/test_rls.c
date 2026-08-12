#include <math.h>

#include "dspm_mult.h"
#include "rls.h"
#include "unity.h"

#define TEST_TOLERANCE 1e-4f

static void reference_update(rls_t *rls,
                             const float phi[2],
                             float measured_output,
                             float *predicted_output,
                             float *prediction_error)
{
    const float y_hat = phi[0] * rls->theta[0] + phi[1] * rls->theta[1];
    const float error = measured_output - y_hat;
    const float p_phi[2] = {
        rls->P[0] * phi[0] + rls->P[1] * phi[1],
        rls->P[2] * phi[0] + rls->P[3] * phi[1],
    };
    const float denominator = rls->lambda + phi[0] * p_phi[0] + phi[1] * p_phi[1];
    const float gain[2] = {
        p_phi[0] / denominator,
        p_phi[1] / denominator,
    };
    const float phi_t_P[2] = {
        phi[0] * rls->P[0] + phi[1] * rls->P[2],
        phi[0] * rls->P[1] + phi[1] * rls->P[3],
    };
    float P_new[4] = {
        (rls->P[0] - gain[0] * phi_t_P[0]) / rls->lambda,
        (rls->P[1] - gain[0] * phi_t_P[1]) / rls->lambda,
        (rls->P[2] - gain[1] * phi_t_P[0]) / rls->lambda,
        (rls->P[3] - gain[1] * phi_t_P[1]) / rls->lambda,
    };

    rls->theta[0] += gain[0] * error;
    rls->theta[1] += gain[1] * error;
    const float off_diagonal = 0.5f * (P_new[1] + P_new[2]);
    P_new[1] = off_diagonal;
    P_new[2] = off_diagonal;
    for (int i = 0; i < 4; ++i) {
        rls->P[i] = P_new[i];
    }
    *predicted_output = y_hat;
    *prediction_error = error;
}

TEST_CASE("ESP-DSP matrix multiplication produces expected 2x2 result", "[rls][esp-dsp]")
{
    const float A[4] = {1.0f, 2.0f, 3.0f, 4.0f};
    const float B[4] = {5.0f, 6.0f, 7.0f, 8.0f};
    float C[4] = {0};

    TEST_ASSERT_EQUAL(ESP_OK, dspm_mult_f32(A, B, C, 2, 2, 2));
    TEST_ASSERT_FLOAT_WITHIN(TEST_TOLERANCE, 19.0f, C[0]);
    TEST_ASSERT_FLOAT_WITHIN(TEST_TOLERANCE, 22.0f, C[1]);
    TEST_ASSERT_FLOAT_WITHIN(TEST_TOLERANCE, 43.0f, C[2]);
    TEST_ASSERT_FLOAT_WITHIN(TEST_TOLERANCE, 50.0f, C[3]);
}

TEST_CASE("ESP-DSP RLS matches scalar reference over multiple updates", "[rls][esp-dsp]")
{
    static const float regressors[][2] = {
        {0.0f, 1.0f},
        {0.2f, 1.0f},
        {0.7f, 0.8f},
        {1.1f, 0.5f},
        {1.0f, -0.2f},
        {0.65f, -0.5f},
    };
    static const float outputs[] = {0.2f, 0.36f, 0.65f, 0.87f, 0.76f, 0.455f};

    rls_t dsp_rls;
    rls_t reference_rls;
    TEST_ASSERT_EQUAL(ESP_OK, rls_init(&dsp_rls, 0.98f, 1000.0f));
    TEST_ASSERT_EQUAL(ESP_OK, rls_init(&reference_rls, 0.98f, 1000.0f));

    for (size_t sample = 0; sample < sizeof(outputs) / sizeof(outputs[0]); ++sample) {
        float dsp_prediction = 0.0f;
        float dsp_error = 0.0f;
        float reference_prediction = 0.0f;
        float reference_error = 0.0f;

        TEST_ASSERT_EQUAL(ESP_OK,
                          rls_update(&dsp_rls, regressors[sample], outputs[sample],
                                     &dsp_prediction, &dsp_error));
        reference_update(&reference_rls, regressors[sample], outputs[sample],
                         &reference_prediction, &reference_error);

        TEST_ASSERT_FLOAT_WITHIN(TEST_TOLERANCE, reference_prediction, dsp_prediction);
        TEST_ASSERT_FLOAT_WITHIN(TEST_TOLERANCE, reference_error, dsp_error);
        for (int i = 0; i < 2; ++i) {
            TEST_ASSERT_FLOAT_WITHIN(TEST_TOLERANCE, reference_rls.theta[i], dsp_rls.theta[i]);
        }
        for (int i = 0; i < 4; ++i) {
            TEST_ASSERT_FLOAT_WITHIN(TEST_TOLERANCE, reference_rls.P[i], dsp_rls.P[i]);
        }
    }
}
