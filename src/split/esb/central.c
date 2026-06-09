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
#include <zmk/events/activity_state_changed.h>
#include <zmk/events/sensor_event.h>
#include <zmk/pointing/input_split.h>
#include <zmk/hid_indicators_types.h>
#include <zmk/physical_layouts.h>

#include <app_esb.h>
#include "common.h"
#include <esb.h>

volatile static struct peripheral_slot_context slot[PERIPHERAL_COUNT];

static zmk_split_transport_central_status_changed_cb_t transport_status_cb;
static bool is_enabled = false;
static int central_handler_rb(int source, uint8_t *data, uint8_t size);
static int split_central_esb_send_command(uint8_t source,
                                          struct zmk_split_transport_central_command cmd);
static int handle_key_position_event(int source, uint8_t *data);

static void peripheral_connected(int pipe) {}
static void peripheral_disconnected(int pipe) {
    uint8_t reset_state[POSITION_STATE_DATA_LEN] = {
        0,
    };

    int source = PIPE_TO_SOURCE(pipe);

    handle_key_position_event(source, reset_state);
}

static struct esb_conn_cb conn_cb = {
    .connected = peripheral_connected,
    .disconnected = peripheral_disconnected,
};

K_SEM_DEFINE(rx_sem, 0, 1);
static void rx_op() { k_sem_give(&rx_sem); }

static struct esb_context ctx = {
    .rx_size = get_payload_data_size_evt,
    .tx_size = get_payload_data_size_cmd,
    .rx_handler = central_handler_rb,
};

static void tx_work_handler(struct k_work *work) { return; }

static int split_central_esb_send_command(uint8_t source,
                                          struct zmk_split_transport_central_command cmd) {

    switch (cmd.type) {
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_POLL_EVENTS:
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_INVOKE_BEHAVIOR:
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_SET_PHYSICAL_LAYOUT:
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_SET_HID_INDICATORS:
        send_data(source, cmd.type, &cmd.data);
    default:
        break;
    }

    return 0;
}

static int split_central_esb_get_available_source_ids(uint8_t *sources) {
    int count = 0;
    uint8_t slot_state = esb_get_slot_state();

    for (int i = 0; i < 8; ++i) {
        if (slot_state & BIT(i))
            sources[count++] = i;
    }

    return count;
}

static void peripheral_init(void) {
    for (int i = 0; i < PERIPHERAL_COUNT; ++i) {
        init_slot(&slot[i]);
    }
}

static int split_central_esb_set_enabled(bool enabled) {
    is_enabled = enabled;

    return 0;
}

static int
split_central_esb_set_status_callback(zmk_split_transport_central_status_changed_cb_t cb) {
    transport_status_cb = cb;
    return 0;
}

static struct zmk_split_transport_status split_central_esb_get_status() {
    enum zmk_split_transport_connections_status conn;

    uint8_t slot_state = esb_get_slot_state();
    int peripherals_connected = __builtin_popcount(slot_state);

    switch (peripherals_connected) {
    case 0:
        conn = ZMK_SPLIT_TRANSPORT_CONNECTIONS_STATUS_DISCONNECTED;
        break;
    case PERIPHERAL_COUNT:
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
    esb_ctx = &ctx;
    print_reset_reason();

    peripheral_init();
    esb_conn_cb_register(&conn_cb);

    int ret = zmk_split_esb_init();
    if (ret) {
        LOG_ERR("zmk_split_esb_init failed (err %d)", ret);
        return ret;
    }

    k_work_submit(&notify_status_work);
    // k_work_submit(&test_work.work);

    return 0;
}

SYS_INIT(zmk_split_esb_central_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

static int handle_key_position_event(int source, uint8_t *data) {
    int ret = 0;
    uint8_t *position_state = slot[source].position_state;
#if CONFIG_ZMK_SPLIT_ESB_SEND_WHOLE_KEY
    for (int i = 0; i < POSITION_STATE_DATA_LEN; ++i) {
        volatile uint32_t changed = (uint32_t)(data[i] ^ position_state[i]);
        while (changed) {
            int changed_bit = __builtin_ctz(changed);

            struct zmk_split_transport_peripheral_event evt = {
                .type = ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_KEY_POSITION_EVENT,
                .data.key_position_event.position = (i * 8) + changed_bit,
                .data.key_position_event.pressed = ((data[i] & BIT(changed_bit)) != 0),
            };

            if (zmk_split_transport_central_peripheral_event_handler(&esb_central, source, evt) ==
                -EINVAL) {
                k_work_submit(&notify_status_work);
            }
            changed &= (changed - 1);
        }
        position_state[i] = data[i];
    }
#else
    struct zmk_split_transport_peripheral_event evt = {
        .type = ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_KEY_POSITION_EVENT,
    };
    memcpy(&evt.data, data, get_payload_data_size_evt(ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_KEY_POSITION_EVENT));
    if (!change_position_state(evt.data.key_position_event.position,
                               evt.data.key_position_event.pressed, position_state)) {
        return 0;
    }

    if (zmk_split_transport_central_peripheral_event_handler(&esb_central, source, evt) ==
        -EINVAL) {
        k_work_submit(&notify_status_work);
    }
#endif

    return 0;
}

static int central_handler_rb(int source, uint8_t *data, uint8_t size) {
    uint8_t type = data[0];
    ssize_t data_size = get_payload_data_size_evt(type);
    if (data_size < 0) {
        LOG_WRN("INVALID TYPE(%d)", data_size);
        return 1;
    }

    if (size < data_size + 1) {
        return 0;
    }
    struct zmk_split_transport_peripheral_event evt = {.type = type};
    switch (type) {
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_KEY_POSITION_EVENT:
        handle_key_position_event(source, &data[1]);
        break;
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_SENSOR_EVENT:
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_INPUT_EVENT:
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_BATTERY_EVENT:
        memcpy(&evt.data, &data[1], data_size);
        if (zmk_split_transport_central_peripheral_event_handler(&esb_central, source, evt) ==
            -EINVAL) {
            k_work_submit(&notify_status_work);
        }
        break;
    default:
        break;
    }

    return data_size + 1;
}