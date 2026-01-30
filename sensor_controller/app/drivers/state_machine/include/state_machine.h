#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "dataPacket.h"

typedef void (*State)(uint8_t);

void sync_state(uint8_t event);
void size_state(uint8_t event);
void addr_state(uint8_t event);
void data_state(uint8_t event);

