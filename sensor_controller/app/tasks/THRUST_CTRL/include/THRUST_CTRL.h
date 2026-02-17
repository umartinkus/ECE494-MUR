#ifndef THRUST_CTRL_H
#define THRUST_CTRL_H

#include "dataPacket.h"
#define N (6)

void THRUST_CTRL(void* params);
void THRUST_UART_CONS(void* params);
void vecmult(const double mat[N][N], double *vec, double *out);
void ctrl_allocation(double *u, double *f);

#endif // THRUST_CTRL_H
