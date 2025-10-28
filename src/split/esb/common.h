/*
 * Copyright (c) 2025 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr/sys/ring_buffer.h>
#include <zephyr/device.h>
#include "app_esb.h"
#include <zmk/split/transport/types.h>

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
#define RX_MSGQ_SIZE CONFIG_ZMK_SPLIT_ESB_EVENT_BUFFER_ITEMS
#define TX_MSGQ_SIZE CONFIG_ZMK_SPLIT_ESB_CMD_BUFFER_ITEMS
#else
#define RX_MSGQ_SIZE CONFIG_ZMK_SPLIT_ESB_CMD_BUFFER_ITEMS
#define TX_MSGQ_SIZE CONFIG_ZMK_SPLIT_ESB_EVENT_BUFFER_ITEMS
#endif

#define IS_CENTRAL IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
#define IS_PERIPHERAL !IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)

#define PERIPHERAL_ID CONFIG_ZMK_SPLIT_ESB_PERIPHERAL_ID

#define TIMEOUT_MS CONFIG_ZMK_SPLIT_ESB_KEYBOARD_EVENT_TIMEOUT_MS
#define SAFE_DIV(x, y) ((x) / (y) > 0 ? (x) / (y) : 1)


extern struct k_msgq rx_msgq;

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
    uint8_t source;
    union {
        struct zmk_split_transport_peripheral_event event;
        struct zmk_split_transport_central_command command;
        struct zmk_split_transport_buffer buf;
    };
};

struct payload_header {
    uint8_t type;
} __packed;

#define HEADER_SIZE sizeof(struct payload_header)

struct payload_buffer {
    struct payload_header header;
    uint8_t body[CONFIG_ESB_MAX_PAYLOAD_LENGTH - HEADER_SIZE];
} __packed;

_Static_assert(sizeof(struct payload_buffer) == CONFIG_ESB_MAX_PAYLOAD_LENGTH,
               "zmk_split_transport_buffer size mismatch");

typedef int (*zmk_split_transport_handler)(struct esb_data_envelope*);
typedef int (*get_data_size)(int);

typedef void (*zmk_split_esb_process_tx_callback_t)(void);
struct zmk_split_esb_ops {
    struct k_work *tx_work;
    struct k_work *rx_work;

    get_data_size get_data_size_rx;
    get_data_size get_data_size_tx;

    zmk_split_transport_handler event_handler;
};

extern struct zmk_split_esb_ops *esb_ops;

ssize_t get_payload_data_size_evt(enum zmk_split_transport_peripheral_event_type _type);
ssize_t get_payload_data_size_cmd(enum zmk_split_transport_central_command_type _type);


int service_init();

uint32_t get_nonce();
int process_payload(char* data, size_t length, uint32_t nonce);

int handle_packet();
void reset_buffers();

int tx_msgq_init(int *type_to_idx);
int put_tx_data(void *ptr);
void *get_next_tx_data();
void requeue_tx_data(void *ptr);
int tx_alloc(void **ptr);
int rx_alloc(void **ptr);
void tx_free(void *ptr);
void rx_free(void *ptr);