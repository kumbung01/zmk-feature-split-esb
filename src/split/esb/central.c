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


enum peripheral_slot_state {
    PERIPHERAL_DOWN,
    PERIPHERAL_UP,
};

// rssi sample count
#define RSSI_SAMPLE_CNT 4
#define PERIPHERAL_REPORT_INTERVAL 10
struct peripheral_slot {
    enum peripheral_slot_state state;
    uint8_t changed_positions[POSITION_STATE_DATA_LEN];
    uint8_t position_state[POSITION_STATE_DATA_LEN];
    int64_t last_reported;
    int rssi_avg;
    uint8_t flag;
};
static struct peripheral_slot peripherals[PERIPHERAL_COUNT];

static void peripheral_init() {
    for (int source = 0; source < PERIPHERAL_COUNT; ++source) {
        peripherals[source].state = PERIPHERAL_DOWN;
        peripherals[source].last_reported = 0;
        peripherals[source].rssi_avg = RSSI_BASELINE;
        peripherals[source].flag = 0;
        memset(peripherals[source].changed_positions, 0, POSITION_STATE_DATA_LEN);
        memset(peripherals[source].position_state, 0, POSITION_STATE_DATA_LEN);
    }
}

static zmk_split_transport_central_status_changed_cb_t transport_status_cb;
static bool is_enabled = false;
static ssize_t packet_maker_central(struct esb_data_envelope *env, struct payload_buffer *buf);
static int central_handler(struct esb_data_envelope *env);
static void tx_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(tx_work, tx_work_handler);
static void rx_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(rx_work, rx_work_handler);
static void set_power_level_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(set_power_level_work, set_power_level_handler);
K_THREAD_STACK_DEFINE(my_work_q_stack, 640);
struct k_work_q my_work_q;

static void tx_op(int timeout_us) {
    timeout_set(timeout_us);
    k_work_reschedule_for_queue(&my_work_q, &tx_work, K_NO_WAIT);
}

static void rx_op(int timeout_us) {
    // k_work_reschedule(&rx_work, K_NO_WAIT);
    k_sem_give(&rx_sem);
}

static struct zmk_split_esb_ops central_ops = {
    .event_handler = central_handler,
    .get_data_size_rx = get_payload_data_size_evt,
    .get_data_size_tx = get_payload_data_size_cmd,
    .tx_op = tx_op,
    .rx_op = rx_op,
    .packet_make = packet_maker_central,
};


static void tx_work_handler(struct k_work *work) {
    LOG_DBG("tx work start");
    esb_tx_app();
}

static void rx_work_handler(struct k_work *work) {
    handle_packet();
}

static ssize_t packet_maker_central(struct esb_data_envelope *env, struct payload_buffer *buf) {
    switch (env->buf.type) {
    default:
        return make_packet_default(env, buf);
        break;
    }

    return 0;
}

static int split_central_esb_send_command(uint8_t source,
                                          struct zmk_split_transport_central_command cmd) {
    
    int err = enqueue_event(source, &cmd);
    if (err) {
        return err;
    }
    
    if (is_esb_active())
        tx_op(NO_WAIT);

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
    peripheral_init();

    k_work_queue_start(&my_work_q, my_work_q_stack,
                       K_THREAD_STACK_SIZEOF(my_work_q_stack),
                       5, NULL);

    int ret = zmk_split_esb_init(APP_ESB_MODE_PRX);
    if (ret) {
        LOG_ERR("zmk_split_esb_init failed (err %d)", ret);
        return ret;
    }

    k_work_submit(&notify_status_work);
    k_work_reschedule_for_queue(&my_work_q, &set_power_level_work, K_SECONDS(PERIPHERAL_REPORT_INTERVAL));
    return 0;
}

SYS_INIT(zmk_split_esb_central_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

static int key_position_handler(struct esb_data_envelope *env) {
    int source = env->source;
    struct peripheral_slot *slot = &peripherals[source];
    uint8_t *data = env->buf.data;
    size_t changed_position_count = 0;
    for (int i = 0; i < POSITION_STATE_DATA_LEN; i++) {
        slot->changed_positions[i] = data[i] ^ slot->position_state[i];
        slot->position_state[i] = data[i];
        changed_position_count += get_bit_count(slot->changed_positions[i]);
    }
    // LOG_HEXDUMP_DBG(slot->position_state, POSITION_STATE_DATA_LEN, "data");

    LOG_DBG("changed position count = %d", changed_position_count);
    for (int i = 0; i < POSITION_STATE_DATA_LEN; i++) {
        for (int j = 0; j < 8; j++) {
            if (slot->changed_positions[i] & BIT(j)) {
                uint32_t position = (i * 8) + j;
                bool pressed = slot->position_state[i] & BIT(j);
                struct zmk_split_transport_peripheral_event evt = {
                    .type = ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_KEY_POSITION_EVENT,
                    .data = {
                        .key_position_event = {
                            .position = position,
                            .pressed = pressed
                        }
                    }
                };
                zmk_split_transport_central_peripheral_event_handler(&esb_central, source, evt);
                k_yield();
                if (--changed_position_count == 0) {
                    return 0;
                }
            }
        }
    }

    return 0;
}

static int central_handler(struct esb_data_envelope *env) {
    int source = env->source;
    __ASSERT(0 <= source && source < ARRAY_SIZE(peripherals), "source must within valid range");

    peripherals[source].state = PERIPHERAL_UP;
    peripherals[source].last_reported = k_uptime_get();
    int rssi = -(env->payload->rssi);
    peripherals[source].rssi_avg = (peripherals[source].rssi_avg * (RSSI_SAMPLE_CNT - 1) + rssi) / RSSI_SAMPLE_CNT; //sliding average
    LOG_DBG("source (%d) rssi-new (%d) rssi-avg (%d)", source, rssi, peripherals[source].rssi_avg);

    switch (env->event.type) {
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_KEY_POSITION_EVENT:
        return key_position_handler(env);
    default:
        return zmk_split_transport_central_peripheral_event_handler(&esb_central, source, env->event);
    }
    
    return 0;
}

static void set_power_level_handler(struct k_work *work) {
    for (int source = 0; source < PERIPHERAL_COUNT; ++source) {
        if (peripherals[source].state == PERIPHERAL_DOWN)
            continue;

        struct zmk_split_transport_buffer buf = {.type = ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_RSSI,
                                                 .rssi = peripherals[source].rssi_avg,};
        
        enqueue_event(source, &buf);
    }

    if (is_esb_active()) 
        tx_op(NO_WAIT);

    split_central_esb_get_status();

    k_work_reschedule_for_queue(&my_work_q, &set_power_level_work, K_SECONDS(PERIPHERAL_REPORT_INTERVAL));
}
                                                        
void rx_thread() {
    while (true)
    {
        k_sem_take(&rx_sem, K_FOREVER);
        LOG_DBG("rx thread awake");
        handle_packet();     
    }
}

K_THREAD_DEFINE(rx_thread_id, 2304,
        rx_thread, NULL, NULL, NULL,
        -1, 0, 0);