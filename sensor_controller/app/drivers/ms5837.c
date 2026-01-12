#include "ms5837.h"

int getDepth(int pressure) {
    return pressure / 1000;
}