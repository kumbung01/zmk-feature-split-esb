/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef __APP_ESB_H
#define __APP_ESB_H

#include <zephyr/kernel.h>
#include <esb.h>
#include <zmk/split/transport/types.h>

#define PERIPHERAL_SLEEP_TIMEOUT 600000

#define PIPE_TO_SOURCE(pipe) (pipe)
#define SOURCE_TO_PIPE(source) (source)
#define HEADER (0xab)

extern const char *ACTIVE_STATE_CHAR[];

typedef struct zmk_split_transport_central_command zmk_cmd;
typedef struct zmk_split_transport_peripheral_event zmk_evt;

typedef struct {
    uint8_t type;
    uint8_t data[16];
} zmk_buf;

union zmk_data {
    zmk_cmd cmd;
    zmk_evt evt;
    zmk_buf buf;
};

struct esb_data {
    int source;
    uint8_t type;
    uint8_t data[32];
};

struct esb_simple_addr {
    uint8_t base_0[4];
    uint8_t base_1[4];
    uint8_t prefix[8];
};

typedef int (*esb_rx_handler)(int source, uint8_t *data, uint8_t size);
typedef ssize_t (*get_data_size)(int type);
typedef void (*esb_op)(void);
typedef void (*esb_op_1)(void *data);
typedef void (*esb_op_2)(void *data1, void *data2);

struct esb_context {
    esb_op tx_op;
    esb_op rx_op;
    esb_op_1 sync_op;

    get_data_size tx_size;
    get_data_size rx_size;

    esb_rx_handler rx_handler;
};

extern struct esb_context *esb_ctx;

int zmk_split_esb_init(void);
int send_data(uint8_t source, uint8_t type, void *data);
int handle_data(void);

#endif
