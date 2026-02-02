#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef void (*State)(uint8_t);

void sync_state(uint8_t event);
void size_state(uint8_t event);
void addr_state(uint8_t event);
void data_state(uint8_t event);

float thr_inv[6][6] = {
    {  0.000545f,  0.5f,       -0.000003f, -0.000022f, -0.000023f, -0.003693f },
    { -0.000545f,  0.5f,        0.000003f,  0.000022f,  0.000023f,  0.003693f },
    { -0.359017f,  0.0f,        0.349225f,  2.318034f,  2.383222f,  0.0f      },
    {  0.348197f,  0.0f,        0.357989f,  2.318034f, -2.383222f,  0.0f      },
    { -0.348197f,  0.0f,        0.357989f, -2.318034f, -2.383222f,  0.0f      },
    {  0.359017f,  0.0f,        0.349225f, -2.318034f,  2.383222f,  0.0f      }
};

