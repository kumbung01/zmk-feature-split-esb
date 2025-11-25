/*
 * Copyright (c) 2025 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/types.h>
#include <zephyr/init.h>

#include <zephyr/settings/settings.h>
#include <zephyr/sys/crc.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_SPLIT_ESB_LOG_LEVEL);

#include <zmk/stdlib.h>
#include <zmk/behavior.h>
#include <zmk/sensors.h>
#include <zmk/split/transport/peripheral.h>
#include <zmk/split/transport/types.h>
#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/sensor_event.h>
#include <zmk/pointing/input_split.h>
#include <zmk/hid_indicators_types.h>
#include <zmk/physical_layouts.h>

#include "timeslot.h"
#include "app_esb.h"
#include "common.h"

#define RSSI_REQUEST_INTREVAL (1)
static uint8_t position_state[POSITION_STATE_DATA_LEN] = {
    0,
};
static void rx_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(rx_work, rx_work_handler);
static void tx_work_handler(struct k_work *work);
K_WORK_DEFINE(tx_work, tx_work_handler);
static int peripheral_handler(struct esb_data_envelope *env);
static ssize_t packet_maker_peripheral(struct esb_data_envelope *env, struct payload_buffer *buf);
static int
split_peripheral_esb_report_event(const struct zmk_split_transport_peripheral_event *event);
static void tx_op() { k_sem_give(&tx_sem); }
static void rx_op() { k_work_submit(&rx_work.work); }

static void rssi_request_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(rssi_request_work, rssi_request_work_handler);
static void rssi_request_work_handler(struct k_work *work) {
    if (!is_esb_enabled())
        return;

    // set_tx_flag(RSSI_REQ);
    // LOG_WRN("rssi request by flag");

    k_work_reschedule(&rssi_request_work, K_SECONDS(RSSI_REQUEST_INTREVAL));
}

static void on_enabled();
static void on_disabled();
static void on_active();
static void on_suspend();
static void alarm_callback(const struct device *dev, uint8_t chan_id, uint32_t ticks,
                           void *user_data);

static struct zmk_split_esb_ops peripheral_ops = {
    .event_handler = peripheral_handler,
    .get_data_size_rx = get_payload_data_size_cmd,
    .get_data_size_tx = get_payload_data_size_evt,
    .tx_op = tx_op,
    .rx_op = rx_op,
    .packet_make = packet_maker_peripheral,
    .on_enabled = on_enabled,
    .on_disabled = on_disabled,
    .on_active = on_active,
    .on_suspend = on_suspend,
};

static void rx_work_handler(struct k_work *work) { handle_packet(); }
static void tx_work_handler(struct k_work *work) {
    if (esb_tx_app() != -ENODATA) {
        k_work_submit(&tx_work);
    }
}

static atomic_t is_kb_event_pending = ATOMIC_INIT(0);

static ssize_t packet_maker_peripheral(struct esb_data_envelope *env, struct payload_buffer *buf) {
    ssize_t data_size = 0;

    switch (env->buf.type) {
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_KEY_POSITION_EVENT:
        uint32_t key = irq_lock();
        memcpy(buf->body, position_state, sizeof(position_state));
        data_size = sizeof(position_state);
        atomic_clear(&is_kb_event_pending);
        irq_unlock(key);
        break;
    default:
        data_size = make_packet_default(env, buf);
        break;
    }

    return data_size;
}

static zmk_split_transport_peripheral_status_changed_cb_t transport_status_cb;
static bool is_enabled = false;

static int zmk_split_bt_position_state(uint8_t position, bool is_pressed) {
    WRITE_BIT(position_state[position / 8], position % 8, is_pressed);
    return 0;
}

static int
split_peripheral_esb_report_event(const struct zmk_split_transport_peripheral_event *event) {
    if (event->type == ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_KEY_POSITION_EVENT) {
        zmk_split_bt_position_state(event->data.key_position_event.position,
                                    event->data.key_position_event.pressed);

        if (!atomic_cas(&is_kb_event_pending, 0, 1)) {
            return 0;
        }
    }

    enqueue_event(PERIPHERAL_ID, event);

TX_OP:
    tx_op();

    return 0;
}

static int split_peripheral_esb_set_enabled(bool enabled) {
    is_enabled = enabled;

    return zmk_split_esb_set_enable(enabled);
}

static int
split_peripheral_esb_set_status_callback(zmk_split_transport_peripheral_status_changed_cb_t cb) {
    transport_status_cb = cb;
    return 0;
}

static struct zmk_split_transport_status split_peripheral_esb_get_status(void) {
    return (struct zmk_split_transport_status){
        .available = true,
        .enabled = is_enabled,
        .connections = ZMK_SPLIT_TRANSPORT_CONNECTIONS_STATUS_ALL_CONNECTED,
    };
}

static const struct zmk_split_transport_peripheral_api peripheral_api = {
    .report_event = split_peripheral_esb_report_event,
    .set_enabled = split_peripheral_esb_set_enabled,
    .set_status_callback = split_peripheral_esb_set_status_callback,
    .get_status = split_peripheral_esb_get_status,
};

ZMK_SPLIT_TRANSPORT_PERIPHERAL_REGISTER(esb_peripheral, &peripheral_api,
                                        CONFIG_ZMK_SPLIT_ESB_PRIORITY);

static void notify_transport_status(void) {
    if (transport_status_cb) {
        transport_status_cb(&esb_peripheral, split_peripheral_esb_get_status());
    }
}

static void notify_status_work_cb(struct k_work *_work) { notify_transport_status(); }

static K_WORK_DEFINE(notify_status_work, notify_status_work_cb);

static int zmk_split_esb_peripheral_init(void) {
    esb_ops = &peripheral_ops;
    print_reset_reason();

    int ret = zmk_split_esb_init(APP_ESB_MODE_PTX);
    if (ret < 0) {
        LOG_ERR("zmk_split_esb_init failed (ret %d)", ret);
        return ret;
    }

    tdma_timer_init(alarm_callback);
    tdma_timer_set(TX_PERIOD);

    k_work_submit(&notify_status_work);
    return 0;
}

SYS_INIT(zmk_split_esb_peripheral_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

static int peripheral_handler(struct esb_data_envelope *env) {
    if (env->buf.type == ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_RSSI) {
        power_set_t cmd = check_rssi(env->buf.rssi);
        LOG_WRN("rssi: %d, tx power: %s", env->buf.rssi, TX_POWER_CHAR[cmd]);
        tx_power_change(cmd);

        return 0;
    }

    return zmk_split_transport_peripheral_command_handler(&esb_peripheral, env->command);
}

static atomic_t timer_start = ATOMIC_INIT(0);
static K_SEM_DEFINE(thread_sem, 0, 1);
void tx_thread() {
    while (true) {
        k_sem_take(&thread_sem, K_FOREVER);

        while (is_esb_active()) {
            k_sem_take(&tx_sem, K_FOREVER);
            if (!is_esb_active()) {
                break;
            }

            LOG_DBG("tx thread awake");
            do {
                if (esb_tx_app() < 0) {
                    break;
                }
                k_yield();
            } while (true);
        }

        // LOG_WRN("esb not active, tx thread going to sleep");
    }
}

K_THREAD_DEFINE(tx_thread_id, 1024, tx_thread, NULL, NULL, NULL, -1, 0, 0);

static void on_enabled() {
    // LOG_WRN("on_enabled called");
    k_work_reschedule(&rssi_request_work, K_SECONDS(RSSI_REQUEST_INTREVAL));
    memset(position_state, 0, sizeof(position_state));
}

static void on_disabled() {
    // LOG_WRN("on_disabled called");
    k_work_cancel_delayable(&rssi_request_work);
    atomic_clear(&timer_start);
    tdma_timer_stop();
}

static void on_active() {
    // LOG_WRN("on_active called");
    k_sem_give(&thread_sem);
    if (atomic_cas(&timer_start, 0, 1)) {
        tdma_timer_start();
    }
}

static void on_suspend() {
    // LOG_WRN("on_suspend called");
    k_sem_give(&tx_sem); // guarantee tx thread can exit tx loop
}

static void alarm_callback(const struct device *dev, uint8_t chan_id, uint32_t ticks,
                           void *user_data) {
    esb_start_tx();
}
