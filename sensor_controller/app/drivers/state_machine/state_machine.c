#include "state_machine.h"
#include "include/state_machine.h"

static State state;
static bool sync_recieved = false;
static uartPacket_t packet_buf = {0};
static size_t position = 0;

void sync_state(uint8_t event) {
  if (sync_recieved && event == START_FRAMEL) {
    state = size_state;
  } else if (event == START_FRAMEH) {
    sync_recieved = true;
  }
}

void size_state(uint8_t event) {
  packet_buf.data_size = event;
  state = addr_state;
}

void addr_state(uint8_t event) {
  packet_buf.device_address = event;
  state = data_state;
}

void data_state(uint8_t event) {
  if (position < packet_buf.data_size) {
    packet_buf.data[position++] = event;
  } else {
    state = sync_state;
    sync_recieved = false;
    position = 0;
  }
}
