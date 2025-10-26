/*
 * Copyright (c) 2025 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include "common.h"

#include <zephyr/sys/crc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
#include <zmk/split/transport/central.h>
#else
#include <zmk/split/transport/peripheral.h>
#endif
#include <esb.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_SPLIT_ESB_LOG_LEVEL);


K_MEM_SLAB_DEFINE(tx_slab, sizeof(struct esb_data_envelope), TX_MSGQ_SIZE, 4);
struct k_msgq **tx_msgq;
size_t tx_msgq_cnt;
K_MEM_SLAB_DEFINE(rx_slab, sizeof(struct esb_payload), RX_MSGQ_SIZE, 4);
K_MSGQ_DEFINE(rx_msgq, sizeof(void*), RX_MSGQ_SIZE, 4);


ssize_t get_payload_data_size_cmd(enum zmk_split_transport_central_command_type _type) {
    switch (_type) {
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_POLL_EVENTS:
        return 0;
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_INVOKE_BEHAVIOR:
        return sizeof(((struct zmk_split_transport_central_command*)0)->data.invoke_behavior);
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_SET_PHYSICAL_LAYOUT:
        return sizeof(((struct zmk_split_transport_central_command*)0)->data.set_physical_layout);
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_SET_HID_INDICATORS:
        return sizeof(((struct zmk_split_transport_central_command*)0)->data.set_hid_indicators);
    default:
        return -ENOTSUP;
    }
}

ssize_t get_payload_data_size_evt(enum zmk_split_transport_peripheral_event_type _type) {
    switch (_type) {
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_INPUT_EVENT:
        return sizeof(((struct zmk_split_transport_peripheral_event*)0)->data.input_event);
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_KEY_POSITION_EVENT:
        return sizeof(((struct zmk_split_transport_peripheral_event*)0)->data.key_position_event);
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_SENSOR_EVENT:
        return sizeof(((struct zmk_split_transport_peripheral_event*)0)->data.sensor_event);
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_BATTERY_EVENT:
        return sizeof(((struct zmk_split_transport_peripheral_event*)0)->data.battery_event);
    default:
        return -ENOTSUP;
    }
}


ssize_t get_payload_data_size_buf(zmk_split_transport_buffer_type _type, bool is_cmd) {
    if (is_cmd) {
        return get_payload_data_size_cmd((enum zmk_split_transport_central_command_type)_type);
    } else {
        return get_payload_data_size_evt((enum zmk_split_transport_peripheral_event_type)_type);
    }
}


struct k_work_q esb_work_q;
K_THREAD_STACK_DEFINE(esb_work_q_stack, 1300);

int service_init(void) {
    static const struct k_work_queue_config queue_config = {
        .name = "Split Peripheral Notification Queue"};
    k_work_queue_start(&esb_work_q, esb_work_q_stack, K_THREAD_STACK_SIZEOF(esb_work_q_stack),
                       5, &queue_config);

    return 0;
}


void zmk_split_esb_cb(app_esb_event_t *event, struct zmk_split_esb_async_state *state) {
    switch(event->evt_type) {
        case APP_ESB_EVT_TX_SUCCESS:
            break;
        case APP_ESB_EVT_TX_FAIL:
            break;
        case APP_ESB_EVT_RX:
            // LOG_DBG("RX + %3d and now buffer is %3d", received, ring_buf_size_get(state->rx_buf));
            if (state->process_tx_callback) {
                state->process_tx_callback();
            } 
            
            else if (state->process_tx_work) {
                k_work_submit_to_queue(&esb_work_q, state->process_tx_work);
            }

            break;
        default:
            LOG_ERR("Unknown APP ESB event!");
            break;
    }
}


// encrption
static uint32_t nonce = 0;
#define ENCRYPTION_KEY CONFIG_ZMK_SPLIT_ESB_ENCRYPTION_KEY
int process_payload(char* data, size_t length, uint32_t nonce) {
    uint32_t key_len = sizeof(ENCRYPTION_KEY);
    for (size_t i = 0; i < length; i++) {
        data[i] ^= (ENCRYPTION_KEY[i % key_len] ^ ((uint8_t*)&nonce)[i % 4]);
    }

    return 0;
}

uint32_t get_nonce() {
    return nonce++;
}

uint32_t get_u32_le(const uint8_t *src) {
    return ((uint32_t)src[0])       |
           ((uint32_t)src[1] << 8)  |
           ((uint32_t)src[2] << 16) |
           ((uint32_t)src[3] << 24);
}

void put_u32_le(uint8_t *dst, uint32_t v) {
    dst[0] = (uint8_t)(v & 0xFF);
    dst[1] = (uint8_t)((v >> 8) & 0xFF);
    dst[2] = (uint8_t)((v >> 16) & 0xFF);
    dst[3] = (uint8_t)((v >> 24) & 0xFF);
}

void reset_buffers() {
    void *data;

    while (k_msgq_num_used_get(&tx_msgq) > 0) {
        if (k_msgq_get(&tx_msgq, &data, K_NO_WAIT) == 0) {
            k_mem_slab_free(&tx_slab, data);
        }
    }

    while (k_msgq_num_used_get(&rx_msgq) > 0) {
        if (k_msgq_get(&rx_msgq, &data, K_NO_WAIT) == 0) {
            k_mem_slab_free(&rx_slab, data);
        }
    }
}


int handle_packet(struct zmk_split_esb_async_state* state, bool is_cmd) {
    int handled = 0;

    while (true) {
        struct esb_payload *rx_payload = NULL;
        int err = k_msgq_get(&rx_msgq, &rx_payload, K_NO_WAIT);
        if (err < 0) {
            LOG_DBG("k_msgq_get failed (err %d)", err);
            break;
        }

        struct payload_buffer *buf = (struct payload_buffer*)(rx_payload->data);
        uint8_t *data = buf->body;
        int type = buf->header.type;
        size_t length = rx_payload->length - HEADER_SIZE;
        size_t offset = 0;

        ssize_t data_size = get_payload_data_size_buf(type, is_cmd);
        if (data_size < 0) {
            LOG_ERR("Unknown event type %d", type);
            break;
        }
        
        size_t count = length / data_size;
        struct esb_data_envelope env = { .buf.type = type, 
                                         .source = rx_payload->pipe,
                                        };

        for (size_t i = 0; i < count; ++i) {
            if (length < data_size + offset) {
                LOG_ERR("Payload too small for event type %d", type);
                break;
            }

            memcpy(env.buf.data, &data[offset], data_size);
            offset += data_size;

            err = state->handler(&env);
            if (err < 0) {
                LOG_ERR("zmk handler failed(%d)", err);
                continue;
            }

            handled++;
        }

        k_mem_slab_free(&rx_slab, (void *)rx_payload);
    }

    return handled;
}

struct k_msgq* get_msgq(struct k_msgq **msgqs, size_t cnt, int* _type) {
    if (!msgqs || cnt == 0 || !_type) {
        return NULL;
    }

    for (int i = 0; i < cnt; ++i) {
        if (msgqs[i] == NULL)
            continue;

        if (k_msgq_num_used_get(msgqs[i]) > 0) {
            *_type = i;
            return msgqs[i];
        }

    }

    return NULL;
}