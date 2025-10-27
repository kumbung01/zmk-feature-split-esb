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

static int type_to_idx[] = {
    [ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_KEY_POSITION_EVENT]  = 0,
    [ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_INPUT_EVENT]         = 1,
    [ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_SENSOR_EVENT]        = 2,
    [ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_BATTERY_EVENT]       = 3
};

static void process_tx_work_handler(struct k_work *work);
K_WORK_DEFINE(process_tx_work, process_tx_work_handler);

static int peripheral_handler(struct esb_data_envelope* env);

static struct zmk_split_esb_async_state async_state = {
    .peripheral_rx_work = &process_tx_work,
    .handler = peripheral_handler,
    .get_data_size_rx = get_payload_data_size_cmd,
    .get_data_size_tx = get_payload_data_size_evt,
};

void zmk_split_esb_on_ptx_esb_callback(app_esb_event_t *event) {
    zmk_split_esb_cb(event, &async_state);
}

static zmk_split_transport_peripheral_status_changed_cb_t transport_status_cb;
static bool is_enabled = false;
extern struct k_work_q esb_work_q;
static int
split_peripheral_esb_report_event(const struct zmk_split_transport_peripheral_event *event) {
    ssize_t data_size = get_payload_data_size_evt(event->type);
    if (data_size < 0) {
        LOG_WRN("get_payload_data_size_evt failed (err %d)", data_size);
        return -ENOTSUP;
    }

    struct esb_data_envelope *env;
    int ret = tx_alloc(&env);
    if (ret < 0) {
        LOG_WRN("Failed to allocate tx_slab (err %d)", ret);
        return -ENOMEM;
    }

    env->event = *event;
    env->source = PERIPHERAL_ID;
    env->timestamp = k_uptime_get();

    int idx = type_to_idx[event->type];

    ret = k_msgq_put(get_tx_msgq(idx), &env, K_NO_WAIT);
    if (ret < 0) {
        LOG_WRN("k_msgq_put failed (err %d)", ret);
        tx_free(env);
        return ret;
    }

    set_tx_queued(true);
    
    k_sem_give(&tx_sem);

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
    int ret = tx_msgq_init(type_to_idx);
    if (ret) {
        LOG_ERR("tx_msgq_init faied(%d)", ret);
        return ret;
    }

    ret = zmk_split_esb_init(APP_ESB_MODE_PTX, zmk_split_esb_on_ptx_esb_callback, &async_state);
    if (ret < 0) {
        LOG_ERR("zmk_split_esb_init failed (ret %d)", ret);
        return ret;
    }

    service_init();

    k_work_submit_to_queue(&esb_work_q, &notify_status_work);
    return 0;
}

SYS_INIT(zmk_split_esb_peripheral_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);


static void process_tx_work_handler(struct k_work *work) {
    while (true) {
        if (handle_packet(&async_state) == 0) {
            break;
        }
    }
}

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
        0, 0, 0);