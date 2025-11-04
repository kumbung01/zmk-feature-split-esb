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
#define TX_MSGQ_SIZE CONFIG_ZMK_SPLIT_ESB_CMD_BUFFER_ITEMS
#else
#define TX_MSGQ_SIZE CONFIG_ZMK_SPLIT_ESB_EVENT_BUFFER_ITEMS
#endif

#define RX_MSGQ_SIZE CONFIG_ESB_RX_FIFO_SIZE

#define IS_CENTRAL IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
#define IS_PERIPHERAL !IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
#if IS_CENTRAL
#define PERIPHERAL_COUNT CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS
#define TX_POWER_INIT (0)
#define SLEEP_DELAY 0
#else
#define TX_POWER_INIT (-4)
#define PERIPHERAL_ID CONFIG_ZMK_SPLIT_ESB_PERIPHERAL_ID
#define SLEEP_DELAY (PERIPHERAL_ID * (RETRANSMIT_DELAY / 2))
#endif

#define RETRANSMIT_DELAY CONFIG_ZMK_SPLIT_ESB_PROTO_TX_RETRANSMIT_DELAY
#define RETRANSMIT_COUNT CONFIG_ZMK_SPLIT_ESB_PROTO_TX_RETRANSMIT_COUNT
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ESB_BITRATE_2MBPS)
#define BITRATE ESB_BITRATE_2MBPS
#else
#define BITRATE ESB_BITRATE_1MBPS
#endif

#define TIMEOUT_MS CONFIG_ZMK_SPLIT_ESB_KEYBOARD_EVENT_TIMEOUT_MS
#define SAFE_DIV(x, y) ((x) / (y) > 0 ? (x) / (y) : 1)
#if IS_CENTRAL
    #define ENABLED_PIPES GENMASK(PERIPHERAL_COUNT, 0)
#else
    #define ENABLED_PIPES BIT(PERIPHERAL_ID)
#endif

#define PIPE_TO_SOURCE(pipe)   (pipe - 1)
#define SOURCE_TO_PIPE(source) (source + 1)
extern const char *ACTIVE_STATE_CHAR[];
extern struct k_msgq rx_msgq;
#define POSITION_STATE_DATA_LEN 16
typedef enum zmk_split_transport_peripheral_event_type zmk_split_transport_buffer_type;
#define ZMK_DATA_SIZE (size_t)(sizeof(struct zmk_split_transport_peripheral_event) > sizeof(struct zmk_split_transport_central_command) ? \
                               sizeof(struct zmk_split_transport_peripheral_event) : \
                               sizeof(struct zmk_split_transport_central_command))


struct zmk_split_transport_buffer {
    zmk_split_transport_buffer_type type;
    union {
        uint8_t data[ZMK_DATA_SIZE - sizeof(zmk_split_transport_buffer_type)];
        int8_t rssi; 
    };
};

_Static_assert(sizeof(struct zmk_split_transport_buffer) == ZMK_DATA_SIZE,
               "zmk_split_transport_buffer size mismatch");

enum zmk_split_transport_central_command_type_proprietary {
    ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_RSSI = 4,
} __packed;

enum zmk_split_transport_peripheral_event_type_proprietary {
    ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_TX_POWER_CHANGED = 4,
} __packed;

struct esb_data_envelope {
    uint8_t source;
    struct esb_payload *payload;
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
typedef void (*esb_op)(int timeout_us);
typedef ssize_t (*get_data_size)(int type);
typedef ssize_t (*packet_maker)(struct esb_data_envelope *env, struct payload_buffer *buf);

typedef void (*zmk_split_esb_process_tx_callback_t)(void);
struct zmk_split_esb_ops {
    esb_op rx_op;
    esb_op tx_op;

    get_data_size get_data_size_rx;
    get_data_size get_data_size_tx;

    packet_maker packet_make;
    zmk_split_transport_handler event_handler;
};

extern struct zmk_split_esb_ops *esb_ops;

ssize_t get_payload_data_size_evt(enum zmk_split_transport_peripheral_event_type _type);
ssize_t get_payload_data_size_cmd(enum zmk_split_transport_central_command_type _type);

int enqueue_event(uint8_t source, struct zmk_split_transport_buffer *event);


int service_init();

uint32_t get_nonce();
int process_payload(char* data, size_t length, uint32_t nonce);

int make_packet(struct esb_payload *payload);
int handle_packet();
void reset_buffers();

int tx_msgq_init(int *type_to_idx);
int put_tx_data(void *ptr);
void *get_next_tx_data();
size_t get_rx_data_count();
int put_rx_data(void *ptr);
int tx_alloc(void **ptr);
void tx_free(void *ptr);
size_t get_tx_count();
void check_stack_usage(struct k_thread *thread, const char *name, int64_t *before, int duration);
power_set_t check_rssi(int rssi);
void print_reset_reason(void);
ssize_t make_packet_default(struct esb_data_envelope *env, struct payload_buffer *buf);
int get_bit_count(uint8_t x);