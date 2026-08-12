#include <stdio.h>
#include "robotica.h"
#include <math.h>
#include "stdbool.h"

bool i_cinematica(const brazo_t *brazo, float px, float py, int codo, float result[2])
{
    if (brazo == NULL || result == NULL)
        return false;

    const float a1 = brazo->art1.a;
    const float a2 = brazo->art2.a;

    if (a1 <= 0.0f || a2 <= 0.0f)
        return false;

    /* Aplicación de la ley de cosenos */
    float cos_theta2 = (px * px + py * py - a1 * a1 - a2 * a2) /
                       (2.0f * a1 * a2);

    /*
     * Si cos(theta2) está muy alejado del intervalo [-1,1],
     * el punto no pertenece al espacio alcanzable.
     */
    const float tolerancia = 1.0e-6f;

    if (cos_theta2 > 1.0f + tolerancia ||
        cos_theta2 < -1.0f - tolerancia)
    {
        return false;
    }

    /*
     * Corrige pequeños errores numéricos cerca de los límites
     * para evitar sqrtf() de un número negativo.
     */
    if (cos_theta2 > 1.0f)
        cos_theta2 = 1.0f;
    else if (cos_theta2 < -1.0f)
        cos_theta2 = -1.0f;

    float sin_theta2 = sqrtf(
        fmaxf(0.0f, 1.0f - cos_theta2 * cos_theta2));

    /* Selección entre las dos soluciones posibles */
    if (codo < 0)
        sin_theta2 = -sin_theta2;

    const float theta2 = atan2f(sin_theta2, cos_theta2);

    const float theta1 = atan2f(py, px) - atan2f(a2 * sin_theta2, a1 + a2 * cos_theta2);

    result[0] = theta1;
    result[1] = theta2;

    return true;
}

bool d_cinematica(const brazo_t *brazo,
                  float theta1,
                  float theta2,
                  float result[2])
{
    if (brazo == NULL || result == NULL ||
        !isfinite(theta1) || !isfinite(theta2))
    {
        return false;
    }

    const float a1 = brazo->art1.a;
    const float a2 = brazo->art2.a;

    if (a1 <= 0.0f || a2 <= 0.0f)
    {
        return false;
    }

    result[0] = a1 * cosf(theta1) +
                a2 * cosf(theta1 + theta2);
    result[1] = a1 * sinf(theta1) +
                a2 * sinf(theta1 + theta2);

    return isfinite(result[0]) && isfinite(result[1]);
}
