/*
 * Copyright (c) 2025 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr/sys/ring_buffer.h>
#include <zephyr/device.h>

#include <zmk/split/transport/types.h>
#include "app_esb.h"

#define TIMEOUT_MS CONFIG_ZMK_SPLIT_ESB_KEYBOARD_EVENT_TIMEOUT_MS

struct zmk_split_transport_buffer {
    uint8_t type;
    uint8_t data[CONFIG_ESB_MAX_PAYLOAD_LENGTH];
};

struct esb_data_envelope {
    uint32_t timestamp;
    uint8_t source;
    union {
        struct zmk_split_transport_peripheral_event event;
        struct zmk_split_transport_central_command command;
        struct zmk_split_transport_buffer buf;
    };
} __packed;

struct payload_buffer {
    uint8_t cnt;
    uint32_t nonce;
    uint8_t body[CONFIG_ESB_MAX_PAYLOAD_LENGTH - 5];
    uint8_t length;
} __packed;

// #define ESB_MSG_EXTRA_SIZE (sizeof(struct esb_msg_prefix) + sizeof(struct esb_msg_postfix))

typedef void (*zmk_split_esb_process_tx_callback_t)(void);

struct zmk_split_esb_async_state {
    atomic_t state;

    zmk_split_esb_process_tx_callback_t process_tx_callback;

    struct k_work_delayable restart_rx_work;
    struct k_work *process_tx_work;
    const struct gpio_dt_spec *dir_gpio;

};

void zmk_split_esb_cb(app_esb_event_t *event, struct zmk_split_esb_async_state *state);

ssize_t get_payload_data_size_evt(const struct zmk_split_transport_peripheral_event *evt);
ssize_t get_payload_data_size_cmd(const struct zmk_split_transport_central_command *cmd);
ssize_t get_payload_data_size_buf(const struct zmk_split_transport_buffer *buf);
ssize_t get_payload_data_size_max(bool is_cmd);

int service_init();

uint32_t get_nonce();
int process_payload(char* data, size_t length, uint32_t nonce);

uint32_t get_u32_le(const uint8_t *src);
void put_u32_le(uint8_t *dst, uint32_t val);