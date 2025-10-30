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

#if IS_ENABLED(CONFIG_ZMK_BATTERY_REPORT_INTERVAL)
#define PERIPHERAL_REPORT_INTERVAL CONFIG_ZMK_BATTERY_REPORT_INTERVAL
#else
#define PERIPHERAL_REPORT_INTERVAL 60
#endif

static const int event_prio[] = {
    [ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_POLL_EVENTS]          = 0,
    [ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_INVOKE_BEHAVIOR]      = 1,
    [ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_SET_PHYSICAL_LAYOUT]  = 2,
    [ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_SET_HID_INDICATORS]   = 3,
};

enum peripheral_slot_state {
    PERIPHERAL_DOWN,
    PERIPHERAL_UP,
};

struct peripheral_slot {
    enum peripheral_slot_state state;
    int64_t last_reported;
};

static struct peripheral_slot peripherals[PERIPHERAL_COUNT];
static zmk_split_transport_central_status_changed_cb_t transport_status_cb;
static bool is_enabled = false;

static int central_handler(struct esb_data_envelope *env);
static void rx_work_handler(struct k_work *work);
static void tx_work_handler(struct k_work *work);
K_WORK_DEFINE(rx_work, rx_work_handler);
K_WORK_DEFINE(tx_work, tx_work_handler);

static void tx_op() {
    k_work_submit(&tx_work);
}

static void rx_op() {
    k_work_submit(&rx_work);
}

static struct zmk_split_esb_ops central_ops = {
    .event_handler = central_handler,
    .get_data_size_rx = get_payload_data_size_evt,
    .get_data_size_tx = get_payload_data_size_cmd,
    .tx_op = tx_op,
    .rx_op = rx_op,
};

static void rx_work_handler(struct k_work *work) {
    int64_t deadline = k_uptime_get() + 4;

    do {
        if (handle_packet() == 0) {
            return;
        }
    } while (k_uptime_get() < deadline);

    k_work_submit(&rx_work);
}


static void tx_work_handler(struct k_work *work) {
    size_t total = 0;
    
    do {
        size_t evt_count = esb_tx_app();
        if (evt_count == 0) {
            break;                                   
        }
        total += evt_count;
    } while (total < CAN_HANDLE_TX);

    if (get_tx_count() > 0) {
        k_work_submit(&tx_work);
    }
}

static int split_central_esb_send_command(uint8_t source,
                                          struct zmk_split_transport_central_command cmd) {
    
    int err = send_event(source, &cmd);
    if (err) {
        return err;
    }
    
    if (is_esb_active())
        tx_op();

    return 0;
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


static int zmk_split_esb_central_init(void) {
    esb_ops = &central_ops;

    int ret = tx_msgq_init(event_prio);
    if (ret) {
        LOG_ERR("tx_msgq_init failed(%d)", ret);
        return ret;
    }

    ret = zmk_split_esb_init(APP_ESB_MODE_PRX);
    if (ret) {
        LOG_ERR("zmk_split_esb_init failed (err %d)", ret);
        return ret;
    }

    k_work_submit(&notify_status_work);
    return 0;
}

SYS_INIT(zmk_split_esb_central_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);


static int central_handler(struct esb_data_envelope *env) {
    int source = env->source;
    __ASSERT(0 <= source && source < ARRAY_SIZE(peripherals), "source must within valid range");

    peripherals[source].state = PERIPHERAL_UP;
    peripherals[source].last_reported = k_uptime_get();
    
    return zmk_split_transport_central_peripheral_event_handler(&esb_central, source, env->event);
}
