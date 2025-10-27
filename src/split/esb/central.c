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
#include <zmk/split/transport/central.h>
#include <zmk/split/transport/types.h>
#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/sensor_event.h>
#include <zmk/pointing/input_split.h>
#include <zmk/hid_indicators_types.h>
#include <zmk/physical_layouts.h>

#include "app_esb.h"
#include "common.h"
#include <esb.h>

#define MPSL_THREAD_PRIO             CONFIG_MPSL_THREAD_COOP_PRIO
#define STACKSIZE                    CONFIG_MAIN_STACK_SIZE


static int type_to_idx[] = {
    [ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_POLL_EVENTS]          = 0,
    [ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_INVOKE_BEHAVIOR]      = 1,
    [ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_SET_PHYSICAL_LAYOUT]  = 2,
    [ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_SET_HID_INDICATORS]   = 3,
};
static int idx_to_type[ARRAY_SIZE(type_to_idx)];
K_MSGQ_DEFINE(msgq0, sizeof(void*), TX_MSGQ_SIZE, 4);
K_MSGQ_DEFINE(msgq1, sizeof(void*), SAFE_DIV(TX_MSGQ_SIZE, 2), 4);
K_MSGQ_DEFINE(msgq2, sizeof(void*), SAFE_DIV(TX_MSGQ_SIZE, 4), 4);
K_MSGQ_DEFINE(msgq3, sizeof(void*), SAFE_DIV(TX_MSGQ_SIZE, 8), 4);
static struct k_msgq* msgqs[] = {&msgq0, &msgq1, &msgq2, &msgq3};

#if IS_ENABLED(CONFIG_ZMK_BATTERY_REPORT_INTERVAL)
#define PERIPHERAL_REPORT_INTERVAL CONFIG_ZMK_BATTERY_REPORT_INTERVAL
#else
#define PERIPHERAL_REPORT_INTERVAL 60
#endif

enum peripheral_slot_state {
    PERIPHERAL_DOWN,
    PERIPHERAL_UP,
};

struct peripheral_slot {
    enum peripheral_slot_state state;
    int64_t last_reported;
};

static struct peripheral_slot peripherals[CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS];

static zmk_split_transport_central_status_changed_cb_t transport_status_cb;
static bool is_enabled = false;

static int central_handler(struct esb_data_envelope *env);

static struct zmk_split_esb_async_state async_state = {
    .handler = central_handler,
    .get_data_size_rx = get_payload_data_size_evt,
    .get_data_size_tx = get_payload_data_size_cmd,
    .central_tx_work = &tx_work,
};

static int split_central_esb_send_command(uint8_t source,
                                          struct zmk_split_transport_central_command cmd) {
    ssize_t data_size = get_payload_data_size_cmd(cmd.type);
    if (data_size < 0) {
        LOG_ERR("get_payload_data_size_cmd failed (err %d)", data_size);
        return -ENOTSUP;
    }

    struct esb_data_envelope *env;
    int ret = tx_alloc(env);
    if (ret < 0) {
        LOG_ERR("k_mem_slab_alloc failed (err %d)", ret);
        return -ENOMEM;
    }

    env->command = cmd;
    env->source = source;
    env->timestamp = k_uptime_get();

    ret = k_msgq_put(msgqs[cmd.type], &env, K_NO_WAIT);
    if (ret < 0) {
        LOG_ERR("k_msgq_put failed (err %d)", ret);
        tx_free(env);
        return ret;
    }

    set_tx_queued(true);

    if (is_esb_active())
        k_work_submit_to_queue(&esb_work_q, &tx_work);

    return 0;
}

void zmk_split_esb_on_prx_esb_callback(app_esb_event_t *event) {
    zmk_split_esb_cb(event, &async_state);
}

static int split_central_esb_get_available_source_ids(uint8_t *sources) {
    int count = 0;
    for (int i = 0; i < ARRAY_SIZE(peripherals); i++) {
        if (peripherals[i].state != PERIPHERAL_UP) {
            continue;
        }

        sources[count++] = i;
    }

    return count;
}


static int split_central_esb_set_enabled(bool enabled) {
    is_enabled = enabled;

    return zmk_split_esb_set_enable(enabled);
}

static int
split_central_esb_set_status_callback(zmk_split_transport_central_status_changed_cb_t cb) {
    transport_status_cb = cb;
    return 0;
}

static struct zmk_split_transport_status split_central_esb_get_status() {
    int64_t now = k_uptime_get();
    size_t peripherals_connected = 0;
    enum zmk_split_transport_connections_status conn;
    for (int i = 0; i < ARRAY_SIZE(peripherals); ++i) {
        if (now - peripherals[i].last_reported >= PERIPHERAL_REPORT_INTERVAL) {
            peripherals[i].state = PERIPHERAL_DOWN;
            continue;
        }

        peripherals_connected++;
    }

    switch(peripherals_connected) {
    case 0:
        conn = ZMK_SPLIT_TRANSPORT_CONNECTIONS_STATUS_DISCONNECTED;
        break;
    case CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS:
        conn = ZMK_SPLIT_TRANSPORT_CONNECTIONS_STATUS_ALL_CONNECTED;
        break;
    default:
        conn = ZMK_SPLIT_TRANSPORT_CONNECTIONS_STATUS_SOME_CONNECTED;
        break;
    }

    return (struct zmk_split_transport_status){
        .available = true,
        .enabled = is_enabled,
        .connections = conn,
    };
}

static const struct zmk_split_transport_central_api central_api = {
    .send_command = split_central_esb_send_command,
    .get_available_source_ids = split_central_esb_get_available_source_ids,
    .set_enabled = split_central_esb_set_enabled,
    .set_status_callback = split_central_esb_set_status_callback,
    .get_status = split_central_esb_get_status,
};

ZMK_SPLIT_TRANSPORT_CENTRAL_REGISTER(esb_central, &central_api, CONFIG_ZMK_SPLIT_ESB_PRIORITY);

static void notify_transport_status(void) {
    if (transport_status_cb) {
        transport_status_cb(&esb_central, split_central_esb_get_status());
    }
}

static void notify_status_work_cb(struct k_work *_work) { notify_transport_status(); }

static K_WORK_DEFINE(notify_status_work, notify_status_work_cb);


static void publish_events_thread() {
    int64_t time = k_uptime_get();

    while (true)
    {
        k_sem_take(&rx_sem, K_FOREVER);
        handle_packet(&async_state);
    }
}

K_THREAD_DEFINE(publish_events_thread_id, STACKSIZE,
        publish_events_thread, NULL, NULL, NULL,
        -1, 0, 0);


static int zmk_split_esb_central_init(void) {
    int ret = tx_msgq_init(msgqs, ARRAY_SIZE(msgqs), type_to_idx, idx_to_type);
    if (ret) {
        LOG_ERR("tx_msgq_init failed(%d)", ret);
        return ret;
    }

    ret = zmk_split_esb_init(APP_ESB_MODE_PRX, zmk_split_esb_on_prx_esb_callback, &async_state);
    if (ret) {
        LOG_ERR("zmk_split_esb_init failed (err %d)", ret);
        return ret;
    }

    service_init();

    k_work_submit_to_queue(&esb_work_q, &notify_status_work);
    return 0;
}

SYS_INIT(zmk_split_esb_central_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);


static int central_handler(struct esb_data_envelope *env) {
    uint8_t source = env->source;
    peripherals[source].state = PERIPHERAL_UP;
    peripherals[source].last_reported = k_uptime_get();
    
    return zmk_split_transport_central_peripheral_event_handler(&esb_central, source, env->event);
}
