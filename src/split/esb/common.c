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


K_MEM_SLAB_DEFINE_STATIC(tx_slab, sizeof(struct esb_data_envelope), TX_MSGQ_SIZE, 4);
K_MEM_SLAB_DEFINE_STATIC(rx_slab, sizeof(struct esb_payload), RX_MSGQ_SIZE, 4);
K_MSGQ_DEFINE(rx_msgq, sizeof(void*), RX_MSGQ_SIZE, 4);

K_MSGQ_DEFINE(msgq0, sizeof(void*), TX_MSGQ_SIZE, 4);
K_MSGQ_DEFINE(msgq1, sizeof(void*), TX_MSGQ_SIZE, 4);
K_MSGQ_DEFINE(msgq2, sizeof(void*), TX_MSGQ_SIZE, 4);
K_MSGQ_DEFINE(msgq3, sizeof(void*), TX_MSGQ_SIZE, 4);
static struct k_msgq* tx_msgq[] = {&msgq0, &msgq1, &msgq2, &msgq3};
static int idx_to_type[ARRAY_SIZE(tx_msgq)];
static const size_t tx_msgq_cnt = ARRAY_SIZE(tx_msgq);


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

#if IS_PERIPHERAL
struct k_work_q esb_work_q;
K_THREAD_STACK_DEFINE(esb_work_q_stack, 1300);

int service_init(void) {
    static const struct k_work_queue_config queue_config = {
        .name = "Split Peripheral Notification Queue"};
    k_work_queue_start(&esb_work_q, esb_work_q_stack, K_THREAD_STACK_SIZEOF(esb_work_q_stack),
                       5, &queue_config);

    return 0;
}
#endif

static size_t tx_fail_count = 0;
void zmk_split_esb_cb(app_esb_event_t *event, struct zmk_split_esb_async_state *state) {
    switch(event->evt_type) {
#if IS_CENTRAL
        case APP_ESB_EVT_TX_SUCCESS:
            // k_work_submit(state->central_tx_work);
            break;
        case APP_ESB_EVT_TX_FAIL:
            // esb_pop_tx();
            // k_work_submit(state->central_tx_work);
            break;
        case APP_ESB_EVT_RX:
            k_sem_give(&rx_sem);
            break;
#else // IS_PERIPHERAL
        case APP_ESB_EVT_TX_SUCCESS:
            k_sem_give(&tx_sem);
            tx_fail_count = 0;
            break;
        case APP_ESB_EVT_TX_FAIL:
            if (tx_fail_count++ > 0) {
                tx_fail_count = 0;
                esb_pop_tx();
                k_sem_give(&tx_sem);
            }
            esb_start_tx();
            break;
        case APP_ESB_EVT_RX:
            k_work_submit_to_queue(&esb_work_q, state->peripheral_rx_work);
            break;
#endif
        default:
            LOG_ERR("Unknown APP ESB event!");
            break;
    }
}

#if 0
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

    for (int i = 0; i < tx_msgq_cnt; ++i) {
        while (k_msgq_get(tx_msgq->msgqs[i], &data, K_NO_WAIT) == 0) {
            tx_free(data);
        }
    }

    while (k_msgq_get(&rx_msgq, &data, K_NO_WAIT) == 0) {
        rx_free(data);
    }
}
#endif


size_t handle_packet(struct zmk_split_esb_async_state* state) {
    size_t handled = 0;
    struct esb_payload *rx_payload = NULL;
    int err = k_msgq_get(&rx_msgq, &rx_payload, K_NO_WAIT);
    if (err < 0) {
        LOG_DBG("k_msgq_get failed (err %d)", err);
        return 0;
    }

    struct payload_buffer *buf = (struct payload_buffer*)(rx_payload->data);
    uint8_t *data = buf->body;
    int type = buf->header.type;
    size_t length = rx_payload->length - HEADER_SIZE;
    size_t offset = 0;
    size_t source = rx_payload->pipe;
#if IS_CENTRAL
    if (source >= CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS)
#else
    if (source != PERIPHERAL_ID)
#endif
    {
        LOG_WRN("invalid source (%u)", source);
        goto CLEANUP;
    }

    ssize_t data_size = state->get_data_size_rx(type);
    if (data_size < 0) {
        LOG_WRN("Unknown event type %d", type);
        goto CLEANUP;
    }
    
    size_t count = length / data_size;
    if (count * data_size != length) {
        LOG_WRN("data_size * count != length");
        goto CLEANUP;
    }

    struct esb_data_envelope env = { .buf.type = type, 
                                        .source = source,
                                    };

    for (size_t i = 0; i < count; ++i) {
        if (length < data_size + offset) {
            LOG_WRN("Payload too small for event type %d", type);
            break;
        }

        memcpy(env.buf.data, &data[offset], data_size);
        offset += data_size;

        err = state->handler(&env);
        if (err < 0) {
            LOG_WRN("zmk handler failed(%d)", err);
        }
        
        handled++;
    }

    if (handled == 0)
        handled = 1; // to prevent from tx app execute

CLEANUP:
    rx_free(rx_payload);
    return handled;
}

int tx_msgq_init(int *type_to_idx) {
    __ASSERT(type_to_idx != NULL, "type_to_idx must not NULL");

    for (int i = 0; i < tx_msgq_cnt; ++i) {
        int idx = type_to_idx[i];
        __ASSERT(idx >= 0 && idx < tx_msgq_cnt, "idx out of valid range");
        idx_to_type[idx] = i;
    }

    return 0;
}

struct k_msgq *tx_msgq_ready(int *_type) {
    __ASSERT(_type != NULL, "_type must not NULL");

    for (int i = 0; i < tx_msgq_cnt; ++i) {
        if (k_msgq_num_used_get(tx_msgq[i]) > 0) {
            *_type = idx_to_type[i];
            return tx_msgq[i];
        }
    }

    return NULL;
}

struct k_msgq *get_tx_msgq(size_t idx) {
    __ASSERT(idx >= 0 && idx < tx_msgq_cnt, "idx out of valid range");

    return tx_msgq[idx];
}

int tx_alloc(void **ptr) {
    return k_mem_slab_alloc(&tx_slab, ptr, K_NO_WAIT);
}

void tx_free(void *ptr) {
    k_mem_slab_free(&tx_slab, ptr);
}

int rx_alloc(void **ptr) {
    return k_mem_slab_alloc(&rx_slab, ptr, K_NO_WAIT);
}

void rx_free(void *ptr) {
    k_mem_slab_free(&rx_slab, ptr);
}

static atomic_t is_queued = ATOMIC_INIT(0);
void set_tx_queued(bool _queued) {
    atomic_set(&is_queued, _queued ? 1 : 0);
}

bool is_tx_queued() {
    return atomic_get(&is_queued) ? true : false;
}