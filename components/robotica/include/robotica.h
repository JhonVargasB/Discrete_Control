
#pragma once

#include <stdbool.h>

typedef struct
{
    float a;
    float alpha;
    float b;
    float theta;
} articulacion_t;

//sacara1
typedef struct
{
    articulacion_t art1;
    articulacion_t art2;
} brazo_t;

bool i_cinematica(const brazo_t *brazo,
                  float px,
                  float py,
                  int codo,
                  float result[2]);

bool d_cinematica(const brazo_t *brazo,
                  float theta1,
                  float theta2,
                  float result[2]);
