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

#define TIMEOUT_MS CONFIG_ZMK_SPLIT_ESB_KEYBOARD_EVENT_TIMEOUT_MS

extern struct k_work_q esb_work_q;
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
    int64_t timestamp;
    uint8_t source;
    union {
        struct zmk_split_transport_peripheral_event event;
        struct zmk_split_transport_central_command command;
        struct zmk_split_transport_buffer buf;
    };
};

struct payload_header {
    uint8_t type;
    // uint32_t nonce;
} __packed;

#define HEADER_SIZE sizeof(struct payload_header)

struct payload_buffer {
    struct payload_header header;
    uint8_t body[CONFIG_ESB_MAX_PAYLOAD_LENGTH - HEADER_SIZE];
} __packed;

_Static_assert(sizeof(struct payload_buffer) == CONFIG_ESB_MAX_PAYLOAD_LENGTH,
               "zmk_split_transport_buffer size mismatch");

// #define ESB_MSG_EXTRA_SIZE (sizeof(struct esb_msg_prefix) + sizeof(struct esb_msg_postfix))

typedef int (*zmk_split_transport_data_handler)(struct esb_data_envelope*);
typedef int (*get_data_size)(int);

typedef void (*zmk_split_esb_process_tx_callback_t)(void);
struct zmk_split_esb_async_state {
    atomic_t state;

    zmk_split_esb_process_tx_callback_t process_tx_callback;

    struct k_work_delayable restart_rx_work;
    struct k_work *process_tx_work;
    const struct gpio_dt_spec *dir_gpio;

    get_data_size get_data_size_rx;
    get_data_size get_data_size_tx;

    zmk_split_transport_data_handler handler;
};

void zmk_split_esb_cb(app_esb_event_t *event, struct zmk_split_esb_async_state *state);

ssize_t get_payload_data_size_evt(enum zmk_split_transport_peripheral_event_type _type);
ssize_t get_payload_data_size_cmd(enum zmk_split_transport_central_command_type _type);


int service_init();

uint32_t get_nonce();
int process_payload(char* data, size_t length, uint32_t nonce);

int handle_packet(struct zmk_split_esb_async_state* state);
void reset_buffers();


uint32_t get_u32_le(const uint8_t *src);
void put_u32_le(uint8_t *dst, uint32_t val);


int tx_msgq_init(struct k_msgq *msgqs[], size_t _count, const int* type_to_idx, int* _idx_to_type);
struct k_msgq *tx_msgq_ready(int *_type);
int tx_alloc(void **ptr);
int rx_alloc(void **ptr);
void tx_free(void *ptr);
void rx_free(void *ptr);
void set_thread_id(k_tid_t thread);

void set_tx_queued(bool _queued);
bool is_tx_queued();