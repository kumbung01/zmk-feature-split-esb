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

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_SPLIT_ESB_LOG_LEVEL);

void zmk_split_esb_async_tx(struct zmk_split_esb_async_state *state) {
    size_t tx_buf_len = ring_buf_size_get(state->tx_buf);
    if (!tx_buf_len) {
        return;
    }
    // LOG_DBG("tx_buf_len %d", tx_buf_len);

    uint8_t *buf;
    uint32_t claim_len = ring_buf_get_claim(state->tx_buf, &buf, tx_buf_len);
    // LOG_DBG("tx_buf_len: %d, claim_len: %d", tx_buf_len, claim_len);

    if (claim_len <= 0) {
        return;
    }
    // LOG_HEXDUMP_DBG(buf, claim_len, "buf");

    static app_esb_data_t my_data;
    my_data.data = buf;
    my_data.len = claim_len;

    int ret = 0;
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ESB_PROTO_TX_ACK)
    ret = app_esb_send(&my_data);
#else
    ret = app_esb_send_noack(&my_data);
#endif

    LOG_DBG("ESB TX Buf finish %d", claim_len);
    ring_buf_get_finish(state->tx_buf, claim_len);
}

void zmk_split_esb_cb(app_esb_event_t *event, struct zmk_split_esb_async_state *state) {
    switch(event->evt_type) {
        case APP_ESB_EVT_TX_SUCCESS:
            LOG_DBG("ESB TX Done %d", event->data_length);
            // ring_buf_get_finish(state->tx_buf, event->data_length);
            if (!ring_buf_is_empty(state->tx_buf)) {
                zmk_split_esb_async_tx(state);
            }
            break;
        case APP_ESB_EVT_TX_FAIL:
            LOG_WRN("ESB TX failed %d", event->data_length);
            // ring_buf_get_finish(state->tx_buf, event->data_length);
            if (!ring_buf_is_empty(state->tx_buf)) {
                zmk_split_esb_async_tx(state);
            }
            break;
        case APP_ESB_EVT_RX:
            LOG_DBG("ESB RX received: %d", event->data_length);
            // LOG_HEXDUMP_DBG(event->buf, event->data_length, "received");

            size_t received = ring_buf_put(state->rx_buf, event->buf, event->data_length);
            if (received < event->data_length) {
                LOG_ERR("RX overrun! %d < %d", received, event->data_length);
                break;
            }

            LOG_DBG("RX %d and now buffer is %d", received, ring_buf_size_get(state->rx_buf));
            if (state->process_tx_callback) {
                state->process_tx_callback();
            } else if (state->process_tx_work) {
                k_work_submit(state->process_tx_work);
            }

            break;
        default:
            LOG_ERR("Unknown APP ESB event!");
            break;
    }
}

int zmk_split_esb_get_item(struct ring_buf *rx_buf, uint8_t *env, size_t env_size) {
    size_t buf_len = ring_buf_size_get(rx_buf);
    // while (buf_len > sizeof(struct esb_msg_prefix) + sizeof(struct esb_msg_postfix)) {
    while (buf_len > sizeof(struct esb_msg_prefix)) {
        struct esb_msg_prefix prefix;

        __ASSERT_EVAL(
            (void)ring_buf_peek(rx_buf, (uint8_t *)&prefix, sizeof(prefix)),
            uint32_t peek_read = ring_buf_peek(rx_buf, (uint8_t *)&prefix, sizeof(prefix)),
            peek_read == sizeof(prefix), "Somehow read less than we expect from the RX buffer");

        if (memcmp(&prefix.magic_prefix, &ZMK_SPLIT_ESB_ENVELOPE_MAGIC_PREFIX,
                   sizeof(prefix.magic_prefix)) != 0) {
            uint8_t discarded_byte;
            ring_buf_get(rx_buf, &discarded_byte, 1);

            LOG_WRN("Prefix mismatch, discarding byte %0x", discarded_byte);

            continue;
        }

        size_t payload_to_read = sizeof(prefix) + prefix.payload_size;

        if (payload_to_read > env_size) {
            LOG_WRN("Invalid message with payload %d bigger than expected max %d", payload_to_read,
                    env_size);
            return -EINVAL;
        }

        // if (buf_len < payload_to_read + sizeof(struct esb_msg_postfix)) {
        if (buf_len < payload_to_read) {
            LOG_WRN("Payload chunk shorter than expected %d < %d ", buf_len, payload_to_read);
            return -EAGAIN;
        }

        // Now that prefix matches, read it out so we can read the rest of the payload.
        __ASSERT_EVAL((void)ring_buf_get(rx_buf, env, payload_to_read),
                      uint32_t read = ring_buf_get(rx_buf, env, payload_to_read),
                      read == payload_to_read,
                      "Somehow read less than we expect from the RX buffer");

        // struct esb_msg_postfix postfix;
        // __ASSERT_EVAL((void)ring_buf_get(rx_buf, (uint8_t *)&postfix, sizeof(postfix)),
        //               uint32_t read = ring_buf_get(rx_buf, (uint8_t *)&postfix, sizeof(postfix)),
        //               read == sizeof(postfix),
        //               "Somehow read less of the postfix than we expect from the RX buffer");

        // uint32_t crc = crc32_ieee(env, payload_to_read);
        // if (crc != postfix.crc) {
        //     LOG_WRN("Data corruption in received peripheral event, ignoring %d vs %d", crc,
        //             postfix.crc);
        //     return -EINVAL;
        // }

        return 0;
    }

    return -EAGAIN;
}
