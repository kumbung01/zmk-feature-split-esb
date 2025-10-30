/*
 * Copyright (c) 2025 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/types.h>
#include <zephyr/init.h>

#include <zephyr/settings/settings.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/ring_buffer.h>

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

#include "app_esb.h"
#include "common.h"

static const int event_prio[] = {
    [ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_KEY_POSITION_EVENT]  = 0,
    [ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_INPUT_EVENT]         = 1,
    [ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_SENSOR_EVENT]        = 2,
    [ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_BATTERY_EVENT]       = 3
};

static void rx_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(rx_work, rx_work_handler);
static int peripheral_handler(struct esb_data_envelope* env);\
static struct zmk_split_esb_ops peripheral_ops = {
    .event_handler = peripheral_handler,
    .get_data_size_rx = get_payload_data_size_cmd,
    .get_data_size_tx = get_payload_data_size_evt,
    .tx_op = tx_op,
    .rx_op = rx_op,
};

static void tx_op() {
    k_sem_give(&tx_sem);
}

static void rx_op() {
    k_work_schedule(&rx_work);
}


static void rx_work_handler(struct k_work *work) {
    size_t total = 0;
    int64_t start = k_uptime_get();
    int64_t total_delta = 0;
    do {
        size_t evt_count = handle_packet();
        int64_t delta = k_uptime_delta(&start);
        LOG_DBG("rx_work delta: %lld, count: %u", delta, evt_count);
        total_delta += delta;
        total += evt_count;
        if (evt_count == 0) {
            break;
        }
    } while (total < CAN_HANDLE_RX);

    if (get_rx_count() > 0) {
        LOG_DBG("rx_work reschedule");
        k_work_schedule(&rx_work, K_MSEC(TIMEOUT_MS));
    }
    else {
        LOG_DBG("rx_work finish");
        rx_work_finished();
    }

    LOG_WRN("rx_work end. total: %u, delta: %lld", total, total_delta);
}


static zmk_split_transport_peripheral_status_changed_cb_t transport_status_cb;
static bool is_enabled = false;
static int
split_peripheral_esb_report_event(const struct zmk_split_transport_peripheral_event *event) {
    int err = send_event(PERIPHERAL_ID, event);
    
    if (is_esb_active())
        k_work_submit(&tx_work);

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

    int ret = tx_msgq_init(event_prio);
    if (ret) {
        LOG_ERR("tx_msgq_init faied(%d)", ret);
        return ret;
    }

    ret = zmk_split_esb_init(APP_ESB_MODE_PTX);
    if (ret < 0) {
        LOG_ERR("zmk_split_esb_init failed (ret %d)", ret);
        return ret;
    }

    k_work_submit(&notify_status_work);
    return 0;
}

SYS_INIT(zmk_split_esb_peripheral_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

static int peripheral_handler(struct esb_data_envelope* env) {
    return zmk_split_transport_peripheral_command_handler(&esb_peripheral, env->command);
}

void tx_thread() {
    while (true)
    {
        k_sem_take(&tx_sem, K_FOREVER);
        LOG_DBG("tx thread awake");
        esb_tx_app();
    }
}

K_THREAD_DEFINE(tx_thread_id, 1300,
        tx_thread, NULL, NULL, NULL,
        5, 0, 0);