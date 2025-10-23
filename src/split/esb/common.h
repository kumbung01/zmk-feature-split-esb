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

typedef enum zmk_split_transport_peripheral_event_type zmk_split_transport_buffer_type;
#define ZMK_DATA_SIZE (size_t)(sizeof(struct zmk_split_transport_peripheral_event) > sizeof(struct zmk_split_transport_central_command) ? \
                               sizeof(struct zmk_split_transport_peripheral_event) : \
                               sizeof(struct zmk_split_transport_central_command))

struct zmk_split_transport_buffer {
    zmk_split_transport_buffer_type type;
    uint8_t data[ZMK_DATA_SIZE - sizeof(zmk_split_transport_buffer_type)];
};

_Static_assert(sizeof(struct zmk_split_transport_buffer) == ZMK_DATA_SIZE,
               "zmk_split_transport_buffer size mismatch");

struct esb_data_envelope {
    uint32_t timestamp;
    uint8_t source;
    union {
        struct zmk_split_transport_peripheral_event event;
        struct zmk_split_transport_central_command command;
        struct zmk_split_transport_buffer buf;
    };
} __packed;

struct payload_header {
    uint8_t count;
    uint32_t nonce;
} __packed;

#define HEADER_SIZE sizeof(struct payload_header)

struct payload_buffer {
    struct payload_header header;
    uint8_t body[CONFIG_ESB_MAX_PAYLOAD_LENGTH - HEADER_SIZE];
} __packed;

_Static_assert(sizeof(struct payload_buffer) == CONFIG_ESB_MAX_PAYLOAD_LENGTH,
               "zmk_split_transport_buffer size mismatch");

// #define ESB_MSG_EXTRA_SIZE (sizeof(struct esb_msg_prefix) + sizeof(struct esb_msg_postfix))

typedef void (*zmk_split_esb_process_tx_callback_t)(void);
struct zmk_split_esb_async_state {
    atomic_t state;

    zmk_split_esb_process_tx_callback_t process_tx_callback;

    struct k_work_delayable restart_rx_work;
    struct k_work *process_tx_work;
    const struct gpio_dt_spec *dir_gpio;

    union {
        struct zmk_split_transport_central *central_transport;
        struct zmk_split_transport_peripheral *peripheral_transport;
    };
};

void zmk_split_esb_cb(app_esb_event_t *event, struct zmk_split_esb_async_state *state);

ssize_t get_payload_data_size_evt(enum zmk_split_transport_peripheral_event_type _type);
ssize_t get_payload_data_size_cmd(enum zmk_split_transport_central_command_type _type);
ssize_t get_payload_data_size_buf(zmk_split_transport_buffer_type _type, bool is_cmd);
ssize_t get_payload_data_size_max(bool is_cmd);

int service_init();

uint32_t get_nonce();
int process_payload(char* data, size_t length, uint32_t nonce);

int handle_packet(struct zmk_split_esb_async_state* state);
void reset_buffers();

uint32_t get_u32_le(const uint8_t *src);
void put_u32_le(uint8_t *dst, uint32_t val);