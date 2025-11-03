/*
 * Copyright (c) 2025 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/types.h>
#include <zephyr/init.h>
#include <zephyr/settings/settings.h>

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


enum peripheral_slot_state {
    PERIPHERAL_DOWN,
    PERIPHERAL_UP,
};

#define TX_CHANGE_SENT 0

struct peripheral_slot {
    enum peripheral_slot_state state;
    int64_t last_reported;
    int rssi;
    uint8_t flag;
};

static struct peripheral_slot peripherals[PERIPHERAL_COUNT];
static zmk_split_transport_central_status_changed_cb_t transport_status_cb;
static bool is_enabled = false;
static int central_handler(struct esb_data_envelope *env);
static void tx_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(tx_work, tx_work_handler);
static void rx_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(rx_work, rx_work_handler);
static void set_power_level_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(set_power_level_work, set_power_level_handler);
K_THREAD_STACK_DEFINE(my_work_q_stack, 2304);
struct k_work_q my_work_q;

static void tx_op(int timeout_us) {
    if (timeout_us == NO_WAIT) {
        timeout_us = 0;
    }

    if (!k_work_delayable_is_pending(&tx_work)) {
        k_work_reschedule_for_queue(&my_work_q, &tx_work, K_USEC(timeout_us));
    }

}

static void rx_op(int timeout_us) {
    k_work_reschedule(&rx_work, K_NO_WAIT);
}

static struct zmk_split_esb_ops central_ops = {
    .event_handler = central_handler,
    .get_data_size_rx = get_payload_data_size_evt,
    .get_data_size_tx = get_payload_data_size_cmd,
    .tx_op = tx_op,
    .rx_op = rx_op,
};


static void tx_work_handler(struct k_work *work) {
    esb_tx_app();
}

static void rx_work_handler(struct k_work *work) {
    handle_packet();
}

static int split_central_esb_send_command(uint8_t source,
                                          struct zmk_split_transport_central_command cmd) {
    
    int err = enqueue_event(source, &cmd);
    if (err) {
        return err;
    }
    
    if (is_esb_active())
        tx_op(0);

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
    print_reset_reason();

    k_work_queue_start(&my_work_q, my_work_q_stack,
                       K_THREAD_STACK_SIZEOF(my_work_q_stack),
                       0, NULL);

    int ret = zmk_split_esb_init(APP_ESB_MODE_PRX);
    if (ret) {
        LOG_ERR("zmk_split_esb_init failed (err %d)", ret);
        return ret;
    }

    k_work_submit(&notify_status_work);
    k_work_reschedule_for_queue(&my_work_q, &set_power_level_work, K_SECONDS(10));
    return 0;
}

SYS_INIT(zmk_split_esb_central_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);


static int central_handler(struct esb_data_envelope *env) {
    int source = env->source;
    __ASSERT(0 <= source && source < ARRAY_SIZE(peripherals), "source must within valid range");

    peripherals[source].state = PERIPHERAL_UP;
    peripherals[source].last_reported = k_uptime_get();
    peripherals[source].rssi = -(env->payload->rssi);

    if (env->buf.type == ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_TX_POWER_CHANGED) {
        LOG_DBG("source (%d) tx power_changed");
        WRITE_BIT(peripherals[source].flag, TX_CHANGE_SENT, 0);
        return 0;
    }
    
    return zmk_split_transport_central_peripheral_event_handler(&esb_central, source, env->event);
}

static void set_power_level_handler(struct k_work *work) {
    for (int source = 0; source < PERIPHERAL_COUNT; ++source) {
        if (peripherals[source].state == PERIPHERAL_DOWN)
            continue;

        power_set_t tx_power = check_rssi(peripherals[source].rssi);

        LOG_DBG("source (%d) rssi (%d) tx_power %d", source, peripherals[source].rssi, tx_power);
        if (tx_power == POWER_OK) 
            continue;

        if (peripherals[source].flag & BIT(TX_CHANGE_SENT)) {
            continue;
        }
        
        struct zmk_split_transport_buffer buf = {.type = ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_SET_TX_POWER,
                                                 .tx_power = tx_power,};
        
        enqueue_event(source, &buf);
        WRITE_BIT(peripherals[source].flag, TX_CHANGE_SENT, 1);
    }

    if (is_esb_active() && get_tx_count() > 0)
        tx_op(NO_WAIT);

    k_work_reschedule_for_queue(&my_work_q, &set_power_level_work, K_SECONDS(10));
}
                                                        
// void rx_thread() {
//     while (true)
//     {
//         k_sem_take(&rx_sem, K_FOREVER);
//         LOG_DBG("rx thread awake");
//         handle_packet();                                                                                                                                                                                        
//     }
// }

// K_THREAD_DEFINE(rx_thread_id, 2304,
//         rx_thread, NULL, NULL, NULL,
//         -1, 0, 0);