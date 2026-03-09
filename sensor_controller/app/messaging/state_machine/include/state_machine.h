#pragma once
#include "freertos/idf_additions.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef void (*State)(uint8_t, QueueHandle_t);

void sync_state(uint8_t event, QueueHandle_t queue);
void size_state(uint8_t event, QueueHandle_t queue);
void addr_state(uint8_t event, QueueHandle_t queue);
void data_state(uint8_t event, QueueHandle_t queue);

extern float thr_inv[6][6];
extern State state;
