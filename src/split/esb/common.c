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


void zmk_split_esb_async_tx(struct zmk_split_esb_async_state *state) {
    if (k_sem_take(state->tx_sem, K_FOREVER) != 0) {
        LOG_WRN("semaphore already taken");

        return;
    }
    size_t tx_buf_len = ring_buf_size_get(state->tx_buf);
    // LOG_DBG("tx_buf_len %u, CONFIG_ESB_MAX_PAYLOAD_LENGTH %u", 
    //         tx_buf_len, CONFIG_ESB_MAX_PAYLOAD_LENGTH);
    if (!tx_buf_len || tx_buf_len > CONFIG_ESB_MAX_PAYLOAD_LENGTH) {
        k_sem_give(state->tx_sem);

        return;
    }
    // LOG_DBG("tx_buf_len %d", tx_buf_len);

    uint8_t buf[CONFIG_ESB_MAX_PAYLOAD_LENGTH];
    // LOG_DBG("tx_buf_len: %d, claim_len: %d", tx_buf_len, claim_len);
    // LOG_HEXDUMP_DBG(buf, claim_len, "buf");
    uint32_t buf_len = ring_buf_get(state->tx_buf, buf, tx_buf_len);

    k_sem_give(state->tx_sem);

    if (buf_len != tx_buf_len)
    {
        LOG_WRN("buf size different?");

        return;
    }

    app_esb_data_t my_data;
    my_data.data = buf;
    my_data.len = buf_len;
    zmk_split_esb_send(&my_data); // callback > zmk_split_esb_cb()

    // LOG_DBG("ESB TX Buf finish %d", claim_len);
}


void zmk_split_esb_cb(app_esb_event_t *event, struct zmk_split_esb_async_state *state) {
    switch(event->evt_type) {
        case APP_ESB_EVT_TX_SUCCESS:
            // LOG_DBG("ESB TX sent");
            if (!ring_buf_is_empty(state->tx_buf)) {
                zmk_split_esb_async_tx(state);
            }
            break;
        case APP_ESB_EVT_TX_FAIL:
            // LOG_WRN("ESB TX failed");
            if (!ring_buf_is_empty(state->tx_buf)) {
                zmk_split_esb_async_tx(state);
            }
            break;
        case APP_ESB_EVT_RX:
            // LOG_DBG("ESB RX received: %d", event->payload->length);

            // lock it for a safe result from ring_buf_space_get()
            int ret = k_sem_take(state->rx_sem, K_FOREVER);
            if (ret) {
                LOG_WRN("semaphore taken");
                break;
            }

            if (ring_buf_space_get(state->rx_buf) < event->payload->length) {
                LOG_WRN("No room to receive from peripheral (have %d but only space for %d/%d)",
                        event->payload->length, ring_buf_space_get(state->rx_buf), 
                        ring_buf_capacity_get(state->rx_buf));
                k_sem_give(state->rx_sem);
                break;
            }

            size_t received = ring_buf_put(state->rx_buf, event->payload->data, event->payload->length);
            if (received < event->payload->length) {
                LOG_ERR("RX overrun! %d < %d", received, event->payload->length);
                k_sem_give(state->rx_sem);
                break;
            }

            k_sem_give(state->rx_sem);

            // LOG_DBG("RX + %3d and now buffer is %3d", received, ring_buf_size_get(state->rx_buf));
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


int zmk_split_esb_get_item(struct ring_buf *rx_buf, uint8_t *env, struct k_sem *sem, size_t env_size) {
    int ret = k_sem_take(sem, K_FOREVER);
    if (ret) {
        LOG_WRN("sempahore already taken");
        return 0;
    }

    while (ring_buf_size_get(rx_buf) > sizeof(struct esb_msg_prefix) + sizeof(struct esb_msg_postfix)) {
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
            k_sem_give(sem);
            return -EINVAL;
        }

        if (ring_buf_size_get(rx_buf) < payload_to_read + sizeof(struct esb_msg_postfix)) {
            k_sem_give(sem);
            return -EAGAIN;
        }

        // Now that prefix matches, read it out so we can read the rest of the payload.
        __ASSERT_EVAL((void)ring_buf_get(rx_buf, env, payload_to_read),
                      uint32_t read = ring_buf_get(rx_buf, env, payload_to_read),
                      read == payload_to_read,
                      "Somehow read less than we expect from the RX buffer");


        struct esb_msg_postfix postfix;
        __ASSERT_EVAL((void)ring_buf_get(rx_buf, (uint8_t *)&postfix, sizeof(postfix)),
                      uint32_t read = ring_buf_get(rx_buf, (uint8_t *)&postfix, sizeof(postfix)),
                      read == sizeof(postfix),
                      "Somehow read less of the postfix than we expect from the RX buffer");

        // LOG_HEXDUMP_DBG(&postfix, sizeof(postfix), "postfix");

        uint32_t crc = crc32_ieee(env, payload_to_read);

        if (crc != postfix.crc) {
            LOG_WRN("Data corruption in received peripheral event, ignoring %d vs %d", crc,
                    postfix.crc);
            k_sem_give(sem);
            return -EINVAL;
        }

        k_sem_give(sem);
        return 0;
    }

    k_sem_give(sem);
    return -EAGAIN;
}
