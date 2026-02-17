#ifndef THRUST_CTRL_H
#define THRUST_CTRL_H

#include "dataPacket.h"
#define N (6)

void THRUST_CTRL(void* params);
void THRUST_UART_CONS(void* params);
void vecmult(const float mat[N][N], float *vec, float *out);
void ctrl_allocation(uint8_t *u, float *f);

#endif // THRUST_CTRL_H
