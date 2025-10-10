/*
 * Copyright (c) 2025 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include "common.h"

#include <zephyr/sys/crc.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <esb.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_SPLIT_ESB_LOG_LEVEL);

K_MSGQ_DEFINE(tx_msgq, sizeof(struct esb_data_envelope), CONFIG_ZMK_SPLIT_ESB_PROTO_MSGQ_ITEMS, 4);
K_MSGQ_DEFINE(rx_msgq, sizeof(struct esb_payload), CONFIG_ZMK_SPLIT_ESB_PROTO_MSGQ_ITEMS, 4);

ssize_t get_payload_data_size_cmd(const struct zmk_split_transport_central_command *cmd) {
    switch (cmd->type) {
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_POLL_EVENTS:
        return 0;
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_INVOKE_BEHAVIOR:
        return sizeof(cmd->data.invoke_behavior);
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_SET_PHYSICAL_LAYOUT:
        return sizeof(cmd->data.set_physical_layout);
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_SET_HID_INDICATORS:
        return sizeof(cmd->data.set_hid_indicators);
    default:
        return -ENOTSUP;
    }
}

ssize_t get_payload_data_size_evt(const struct zmk_split_transport_peripheral_event *evt) {
    switch (evt->type) {
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_INPUT_EVENT:
        return sizeof(evt->data.input_event);
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_KEY_POSITION_EVENT:
        return sizeof(evt->data.key_position_event);
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_SENSOR_EVENT:
        return sizeof(evt->data.sensor_event);
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_BATTERY_EVENT:
        return sizeof(evt->data.battery_event);
    default:
        return -ENOTSUP;
    }
}

struct k_work_q esb_work_q;
K_THREAD_STACK_DEFINE(esb_work_q_stack, 1300);

int service_init(void) {
    static const struct k_work_queue_config queue_config = {
        .name = "Split Peripheral Notification Queue"};
    k_work_queue_start(&esb_work_q, esb_work_q_stack, K_THREAD_STACK_SIZEOF(esb_work_q_stack),
                       3, &queue_config);

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