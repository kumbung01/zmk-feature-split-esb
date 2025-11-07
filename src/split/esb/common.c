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
#include <zephyr/drivers/hwinfo.h>
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_SPLIT_ESB_LOG_LEVEL);


K_MEM_SLAB_DEFINE_STATIC(tx_slab, sizeof(struct esb_data_envelope), TX_MSGQ_SIZE, 4);
K_MSGQ_DEFINE(tx_msgq, sizeof(void*), TX_MSGQ_SIZE, 4);
#if CONFIG_LOG
const char *ACTIVE_STATE_CHAR[] = {"ACTIVE", "IDLE", "SLEEP"};
const char *TX_POWER_CHAR[] = {"OK", "UP", "DOWN"};
#endif

struct zmk_split_esb_ops *esb_ops;

ssize_t get_payload_data_size_cmd(enum zmk_split_transport_central_command_type _type) {
    ssize_t size = -1;

    switch (_type) {
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_POLL_EVENTS:
        size = 0;
        break;
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_INVOKE_BEHAVIOR:
        size = sizeof(((struct zmk_split_transport_central_command*)0)->data.invoke_behavior);
        break;
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_SET_PHYSICAL_LAYOUT:
        size = sizeof(((struct zmk_split_transport_central_command*)0)->data.set_physical_layout);
        break;
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_SET_HID_INDICATORS:
        size = sizeof(((struct zmk_split_transport_central_command*)0)->data.set_hid_indicators);
        break;
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_RSSI:
        size = 1;
        break;
    default:
        size = -ENOTSUP;
        break;
    }

    LOG_DBG("cmd type (%d) size (%d)", _type, size);

    return size;
}


ssize_t get_payload_data_size_evt(enum zmk_split_transport_peripheral_event_type _type) {
    ssize_t size = -1;

    switch (_type) {
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_INPUT_EVENT:
        size = sizeof(((struct zmk_split_transport_peripheral_event*)0)->data.input_event);
        break;
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_KEY_POSITION_EVENT:
        size = POSITION_STATE_DATA_LEN;
        break;
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_SENSOR_EVENT:
        size = sizeof(((struct zmk_split_transport_peripheral_event*)0)->data.sensor_event);
        break;
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_BATTERY_EVENT:
        size = sizeof(((struct zmk_split_transport_peripheral_event*)0)->data.battery_event);
        break;
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_RSSI_REQUEST:
        size = 0;
        break;
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_RSSI_TAKE:
        size = 0;
        break;
    default:
        size = -ENOTSUP;
        break;
    }

    LOG_DBG("evt type (%d) size (%d)", _type, size);

    return size;
}

power_set_t check_rssi(int rssi) {
    if (rssi < RSSI_BASELINE - 4) {
        return POWER_UP;
    }
    else if (rssi > RSSI_BASELINE + 4) {
        return POWER_DOWN;
    }
    else {
        return POWER_OK;
    }
}

// static size_t tx_fail_count = 0;


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

int enqueue_event(uint8_t source, struct zmk_split_transport_buffer *buf) {
    LOG_DBG("sending packet type (%d)", buf->type);

    struct esb_data_envelope *env;
    int ret = tx_alloc(&env);
    if (ret < 0) {
        LOG_WRN("k_mem_slab_alloc failed (err %d)", ret);
        return -ENOMEM;
    }

    env->buf = *buf;
    env->source = source;
    // env->timestamp = k_uptime_get();

    ret = put_tx_data(env);
    if (ret < 0) {
        LOG_WRN("k_msgq_put failed (err %d)", ret);
        tx_free(env);
        return ret;
    }

    return 0;
}

ssize_t make_packet_default(struct esb_data_envelope *env, struct payload_buffer *buf) {
    ssize_t data_size = esb_ops->get_data_size_tx(buf->header.type);
    if (data_size < 0) {
        return data_size;
    }

    LOG_DBG("adding type %u size %u to packet", buf->header.type, data_size);

    memcpy(buf->body, env->buf.data, data_size);

    return data_size;
}

int make_packet(struct esb_payload *payload) {
    struct payload_buffer *buf = (struct payload_buffer *)payload->data;
    const size_t body_size = sizeof(buf->body);
    payload->noack = !CONFIG_ZMK_SPLIT_ESB_PROTO_TX_ACK;

    struct esb_data_envelope *env = get_next_tx_data();
    if (env == NULL) {
        return -ENODATA;
    }

    buf->header.type = env->buf.type;
    ssize_t data_size = esb_ops->packet_make(env, buf);
    if (data_size < 0) {
        tx_free(env);
        return -ENOTSUP;
    }

    payload->pipe = SOURCE_TO_PIPE(env->source); // use the source as the ESB pipe number

    tx_free(env);

    // write header and length
    payload->length = data_size + HEADER_SIZE;

    return 0;
}

int handle_packet() {
    struct esb_payload rx_payload;
    int err = esb_read_rx_payload(&rx_payload);
    if (err) {
        LOG_DBG("esb_read_returned (%d)", err);
        return err;
    }

    LOG_DBG("rx_payload pipe %d", rx_payload.pipe);

    struct payload_buffer *buf = (struct payload_buffer*)(rx_payload.data);
    uint8_t *data = buf->body;
    int type = buf->header.type;
    size_t length = rx_payload.length - HEADER_SIZE;

    ssize_t data_size = esb_ops->get_data_size_rx(type);
    if (data_size < 0) {
        LOG_WRN("Unknown event type %d", type);
        return -ENOTSUP;
    }

    struct esb_data_envelope env = { .buf.type = type, 
                                     .source = PIPE_TO_SOURCE(rx_payload.pipe),
                                     .payload = &rx_payload,
                                    };

    memcpy(env.buf.data, data, data_size);

    err = esb_ops->event_handler(&env);
    if (err < 0) {
        LOG_WRN("zmk handler failed(%d)", err);
    }

    return err;
}

void *get_next_tx_data() {
    void *ptr = NULL;
    int err = k_msgq_get(&tx_msgq, &ptr, K_NO_WAIT);
    if (err) {
        LOG_DBG("queue is empty.");
    }

    return ptr;
}

int put_tx_data(void *ptr) {
    __ASSERT(ptr != NULL, "ptr must not null");
    return k_msgq_put(&tx_msgq, &ptr, K_NO_WAIT);
}

int tx_alloc(void **ptr) {
    return k_mem_slab_alloc(&tx_slab, ptr, K_NO_WAIT);
}

void tx_free(void *ptr) {
    k_mem_slab_free(&tx_slab, ptr);
}

size_t get_tx_count() {
    return k_mem_slab_num_used_get(&tx_slab);
}

void check_stack_usage(struct k_thread *thread, const char *name, int64_t *before, int duration)
{
    size_t unused_stack;
    int64_t now = k_uptime_get();
    if (now - *before < duration) {
        return;
    }

    *before = now;

    int ret = k_thread_stack_space_get(thread, &unused_stack);
    if (ret == 0) {
        size_t total_size = thread->stack_info.size; 
        size_t used_stack = total_size - unused_stack;

        LOG_WRN("\"%s\": Max Used %zu bytes / Total %zu bytes (%u%%)",
               name, 
               used_stack, 
               total_size,
               (uint32_t)(used_stack * 100.0 / total_size));

    } else {
        LOG_WRN("Error: Failed to get stack space for %s (ret: %d)", name, ret);
    }
}

void print_reset_reason(void)
{
    uint32_t cause = 0;
    int ret = hwinfo_get_reset_cause(&cause);

    if (ret < 0) {
        LOG_ERR("Failed to get reset cause (err %d)", ret);
        return;
    }

    LOG_INF("Reset cause raw value: 0x%08x", cause);

    if (cause & RESET_PIN) {
        LOG_INF("Reset reason: Pin reset");
    }
    if (cause & RESET_SOFTWARE) {
        LOG_INF("Reset reason: Software reset");
    }
    if (cause & RESET_WATCHDOG) {
        LOG_INF("Reset reason: Watchdog");
    }
    if (cause & RESET_BROWNOUT) {
        LOG_INF("Reset reason: Brownout / Power failure");
    }
    if (cause & RESET_POR) {
        LOG_INF("Reset reason: Power-on reset");
    }

    hwinfo_clear_reset_cause();
}

inline int get_bit_count(uint8_t x) {
    x = x - ((x >> 1) & 0x55);
    x = (x & 0x33) + ((x >> 2) & 0x33);
    x = (x + (x >> 4)) & 0x0F;
    
    return x;
}