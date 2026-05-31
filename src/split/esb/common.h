/*
 * Copyright (c) 2025 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr/sys/ring_buffer.h>
#include <zephyr/device.h>
#include <zmk/split/transport/types.h>

#define IS_CENTRAL IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
#define IS_PERIPHERAL !IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)

#if IS_CENTRAL
#define PERIPHERAL_COUNT CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS
#else
#define PERIPHERAL_COUNT 1
#define PERIPHERAL_ID CONFIG_ZMK_SPLIT_ESB_PERIPHERAL_ID
#endif

#define POSITION_STATE_DATA_LEN 16

enum peripheral_slot_state {
    PERIPHERAL_DOWN,
    PERIPHERAL_UP,
};

struct peripheral_slot_context {
    enum peripheral_slot_state state;
    uint8_t position_state[POSITION_STATE_DATA_LEN];
};

void schedule_thread_info_check_work(int interval_ms);
// void check_stack_usage(struct k_thread *thread, const char *name);
void print_reset_reason(void);
ssize_t get_payload_data_size_cmd(int _type);
ssize_t get_payload_data_size_evt(int _type);
bool change_position_state(uint8_t position, bool is_pressed, uint8_t *position_state);
void init_slot(struct peripheral_slot_context *slot);