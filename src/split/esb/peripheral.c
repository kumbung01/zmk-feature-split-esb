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

#define TX_BUFFER_SIZE                                                                             \
    ((sizeof(struct esb_event_envelope) + sizeof(struct esb_msg_postfix)) *                        \
     CONFIG_ZMK_SPLIT_ESB_EVENT_BUFFER_ITEMS) + 4
#define RX_BUFFER_SIZE                                                                             \
    ((sizeof(struct esb_command_envelope) + sizeof(struct esb_msg_postfix)) *                      \
     CONFIG_ZMK_SPLIT_ESB_CMD_BUFFER_ITEMS) + 4

static const uint8_t peripheral_id = CONFIG_ZMK_SPLIT_ESB_PERIPHERAL_ID;

static void publish_commands_work(struct k_work *work);

extern struct k_msgq esb_work_q;

K_WORK_DEFINE(publish_commands, publish_commands_work);

static void process_tx_cb(void);
K_MSGQ_DEFINE(cmd_msg_queue, sizeof(struct zmk_split_transport_central_command), 3, 4);

static struct zmk_split_esb_async_state async_state = {
    .rx_size_process_trigger = sizeof(struct esb_command_envelope),
    .process_tx_callback = process_tx_cb,
};

void zmk_split_esb_on_ptx_esb_callback(app_esb_event_t *event) {
    zmk_split_esb_cb(event, &async_state);
}

static ssize_t get_payload_data_size(const struct zmk_split_transport_peripheral_event *evt) {
    switch (evt->type) {
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_INPUT_EVENT:
        return sizeof(evt->data.input_event);
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_KEY_POSITION_EVENT:
        return sizeof(evt->data.key_position_event);
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_SENSOR_EVENT:
        return sizeof(evt->data.sensor_event);
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_BATTERY_EVENT:
        return sizeof(evt->data.battery_event);
    default:
        return -ENOTSUP;
    }
}

extern struct k_msgq m_msgq_tx_payloads;
extern struct k_work_q esb_work_q;

static int
split_peripheral_esb_report_event(const struct zmk_split_transport_peripheral_event *event) {
    uint8_t buf[CONFIG_ESB_MAX_PAYLOAD_LENGTH];

    ssize_t data_size = get_payload_data_size(event);
    if (data_size < 0) {
        LOG_WRN("Failed to determine payload data size %d", data_size);
        return data_size;
    }

    // Data + type + source
    size_t payload_size =
        data_size + sizeof(peripheral_id) + sizeof(enum zmk_split_transport_peripheral_event_type);

    struct esb_data_envelope env = { .event = {
                                        .source = peripheral_id,
                                        .event = *event,
                                    }};

    size_t pfx_len = payload_size;
    memcpy(buf, &env, pfx_len);

    app_esb_data_t data;
    data.len = pfx_len;
    data.data = buf;
    zmk_split_esb_send(&data);

    return 0;
}

static zmk_split_transport_peripheral_status_changed_cb_t transport_status_cb;
static bool is_enabled = false;

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
    int ret = zmk_split_esb_init(APP_ESB_MODE_PTX, zmk_split_esb_on_ptx_esb_callback);
    if (ret < 0) {
        LOG_ERR("zmk_split_esb_init failed (ret %d)", ret);
        return ret;
    }
    service_init();

    k_work_submit_to_queue(&esb_work_q, &notify_status_work);
    return 0;
}

SYS_INIT(zmk_split_esb_peripheral_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

static void process_tx_cb(void) {
    while (k_msgq_num_used_get(&rx_msgq) > 0) {
        struct esb_command_envelope env;
        int item_err = k_msgq_get(&rx_msgq, &env, K_NO_WAIT);
        switch (item_err) {
        case 0:
            if (env.payload.cmd.type == ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_POLL_EVENTS) {
                // pull_packet_from_tx_msgq();
            } else {
                int ret = k_msgq_put(&cmd_msg_queue, &env.payload.cmd, K_NO_WAIT);
                if (ret < 0) {
                    LOG_WRN("Failed to queue command for processing (%d)", ret);
                    return;
                }

                k_work_submit_to_queue(&esb_work_q, &publish_commands);
            }
            break;
        case -EAGAIN:
            return;
        default:
            LOG_WRN("Issue fetching an item from the RX buffer: %d", item_err);
            return;
        }
    }
}

static void publish_commands_work(struct k_work *work) {
    struct zmk_split_transport_central_command cmd;

    while (k_msgq_get(&cmd_msg_queue, &cmd, K_NO_WAIT) >= 0) {
        zmk_split_transport_peripheral_command_handler(&esb_peripheral, cmd);
    }
}
