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
static uint8_t position_state[POSITION_STATE_DATA_LEN] = {0, };
static void rx_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(rx_work, rx_work_handler);
static void tx_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(tx_work, tx_work_handler);
static int peripheral_handler(struct esb_data_envelope* env);
static ssize_t packet_maker_peripheral(struct esb_data_envelope *env, struct payload_buffer *buf);
static void tx_op(int timeout_us) {
    // if (!k_work_delayable_is_pending(&tx_work))
    //     k_work_reschedule(&tx_work, timeout);    
    timeout_set(timeout_us);
    k_sem_give(&tx_sem);
}

static void rx_op(int timeout_us) {
    if (!k_work_delayable_is_pending(&rx_work))
        k_work_reschedule(&rx_work, K_USEC(timeout_us));
}
static struct zmk_split_esb_ops peripheral_ops = {
    .event_handler = peripheral_handler,
    .get_data_size_rx = get_payload_data_size_cmd,
    .get_data_size_tx = get_payload_data_size_evt,
    .tx_op = tx_op,
    .rx_op = rx_op,
    .packet_make = packet_maker_peripheral,
};


static void rx_work_handler(struct k_work *work) {
    do {
        if (handle_packet() != 0)
            return;
    } while(true);
}


static void tx_work_handler(struct k_work *work) {
    esb_start_tx();
    do {
        if (esb_tx_app() != 0)
            return;
    } while(true);
}

static ssize_t packet_maker_peripheral(struct esb_data_envelope *env, struct payload_buffer *buf) {
    ssize_t data_size = 0;

    switch (env->buf.type) {
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_KEY_POSITION_EVENT:
        memcpy(buf->body, position_state, sizeof(position_state));
        data_size = sizeof(position_state);
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
        zmk_split_bt_position_state(event->data.key_position_event.position, event->data.key_position_event.pressed);
    }

    int err = enqueue_event(PERIPHERAL_ID, event);
    
    if (is_esb_active())
        tx_op(NO_WAIT);

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

    k_work_submit(&notify_status_work);
    return 0;
}

SYS_INIT(zmk_split_esb_peripheral_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);


static int peripheral_handler(struct esb_data_envelope* env) {
    static const char* str[] = {"OK", "UP", "DOWN"};
    if (env->buf.type == ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_RSSI) {
        power_set_t cmd = check_rssi(env->buf.rssi);
        LOG_WRN("rssi: %d, tx power: %s", env->buf.rssi, str[cmd]);
        tx_power_change(cmd);

        return 0;
    }

    return zmk_split_transport_peripheral_command_handler(&esb_peripheral, env->command);
}

void tx_thread() {
    int64_t before = 0;

    while (true)
    {
        k_sem_take(&tx_sem, K_FOREVER);
        LOG_DBG("tx thread awake");
        do {
            if (esb_tx_app() != 0)
                break;
        } while (true);
    }
}

K_THREAD_DEFINE(tx_thread_id, 640,
        tx_thread, NULL, NULL, NULL,
        5, 0, 0);