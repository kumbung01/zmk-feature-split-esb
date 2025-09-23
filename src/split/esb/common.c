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

K_MSGQ_DEFINE(rx_msgq, sizeof(struct esb_event_envelope), CONFIG_ZMK_SPLIT_ESB_PROTO_MSGQ_ITEMS, 4);

void zmk_split_esb_cb(app_esb_event_t *event, struct zmk_split_esb_async_state *state) {
    switch(event->evt_type) {
        case APP_ESB_EVT_TX_SUCCESS:
            // LOG_DBG("ESB TX sent");
            pull_packet_from_tx_msgq();
            break;
        case APP_ESB_EVT_TX_FAIL:
            // LOG_WRN("ESB TX failed");
            pull_packet_from_tx_msgq();
            break;
        case APP_ESB_EVT_RX:
            int ret = k_msgq_put(&rx_msgq, event->payload->data, K_NO_WAIT);
            if (ret) {
                LOG_WRN("rx msgq put fail(%d)", ret);
                break;
            }

            // LOG_DBG("RX + %3d and now buffer is %3d", received, ring_buf_size_get(state->rx_buf));
            if (state->process_tx_callback) {
                state->process_tx_callback();
            } 
            /*
            else if (state->process_tx_work) {
                k_work_submit(state->process_tx_work);
            }*/

            break;
        default:
            LOG_ERR("Unknown APP ESB event!");
            break;
    }
}
