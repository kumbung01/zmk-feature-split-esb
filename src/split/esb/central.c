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

#define STACKSIZE                    CONFIG_MAIN_STACK_SIZE

#define RX_BUFFER_SIZE                                                                             \
    ((sizeof(struct esb_event_envelope) + sizeof(struct esb_msg_postfix)) *                        \
     CONFIG_ZMK_SPLIT_ESB_EVENT_BUFFER_ITEMS) * 2 + 4
#define TX_BUFFER_SIZE                                                                             \
    ((sizeof(struct esb_command_envelope) + sizeof(struct esb_msg_postfix)) *                      \
     CONFIG_ZMK_SPLIT_ESB_CMD_BUFFER_ITEMS) + 4

static void publish_events_work(struct k_work *work);

K_WORK_DEFINE(publish_events, publish_events_work);

extern struct k_msgq rx_msgq;

static struct zmk_split_esb_async_state async_state = {
    .process_tx_work = &publish_events,
    .rx_size_process_trigger = ESB_MSG_EXTRA_SIZE + 1,
};

static ssize_t get_payload_data_size(const struct zmk_split_transport_central_command *cmd) {
    switch (cmd->type) {
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_POLL_EVENTS:
        return 0;
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_INVOKE_BEHAVIOR:
        return sizeof(cmd->data.invoke_behavior);
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_SET_PHYSICAL_LAYOUT:
        return sizeof(cmd->data.set_physical_layout);
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_SET_HID_INDICATORS:
        return sizeof(cmd->data.set_hid_indicators);
    default:
        return -ENOTSUP;
    }
}

static int split_central_esb_send_command(uint8_t source,
                                          struct zmk_split_transport_central_command cmd) {
    uint8_t buf[CONFIG_ESB_MAX_PAYLOAD_LENGTH];

    ssize_t data_size = get_payload_data_size(&cmd);
    if (data_size < 0) {
        LOG_WRN("Failed to determine payload data size %d", data_size);
        return data_size;
    }

    // Data + type + source
    size_t payload_size =
        data_size + sizeof(source) + sizeof(enum zmk_split_transport_central_command_type);

    struct esb_command_envelope env = {.prefix = {
                                            .magic_prefix = ZMK_SPLIT_ESB_ENVELOPE_MAGIC_PREFIX,
                                            .payload_size = payload_size,
                                        },
                                        .payload = {
                                            .source = source,
                                            .cmd = cmd,
                                        }};

    size_t pfx_len = sizeof(env.prefix) + payload_size;
    memcpy(buf, &env, pfx_len);

    struct esb_msg_postfix postfix = {.crc = crc32_ieee((void *)&env, pfx_len)};

    memcpy(buf + pfx_len, &postfix, sizeof(struct esb_msg_postfix));

    app_esb_data_t data;
    data.len = pfx_len + sizeof(struct esb_msg_postfix);
    data.data = buf;
    zmk_split_esb_send(&data);

    return 0;
}

void zmk_split_esb_on_prx_esb_callback(app_esb_event_t *event) {
    zmk_split_esb_cb(event, &async_state);
}

static int split_central_esb_get_available_source_ids(uint8_t *sources) {
    sources[0] = 0;

    return 1;
}

static zmk_split_transport_central_status_changed_cb_t transport_status_cb;
static bool is_enabled;

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
    return (struct zmk_split_transport_status){
        .available = true,
        .enabled = is_enabled,
        .connections = ZMK_SPLIT_TRANSPORT_CONNECTIONS_STATUS_ALL_CONNECTED,
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
    int ret = zmk_split_esb_init(APP_ESB_MODE_PRX, zmk_split_esb_on_prx_esb_callback);
    if (ret) {
        LOG_ERR("zmk_split_esb_init failed (err %d)", ret);
        return ret;
    }
    k_work_submit(&notify_status_work);
    return 0;
}

SYS_INIT(zmk_split_esb_central_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

static void publish_events_work(struct k_work *work) {
    while (k_msgq_num_used_get(&rx_msgq) > 0) {
        struct esb_event_envelope env;
        int item_err = k_msgq_get(&rx_msgq, &env, K_NO_WAIT);
            // zmk_split_esb_get_item(&rx_buf, (uint8_t *)&env, async_state.rx_sem, sizeof(struct esb_event_envelope));
        switch (item_err) {
        case 0:
            zmk_split_transport_central_peripheral_event_handler(&esb_central, env.payload.source,
                                                                 env.payload.event);
            break;
        case -EAGAIN:
            LOG_WRN("k_msgq get fail(%d)", item_err);
            return;
        default:
            LOG_WRN("Issue fetching an item from the RX buffer: %d", item_err);
            return;
        }
    }
}

static void publish_events_thread(void) {
    struct esb_event_envelope env;
    while (true) { 
        int err = k_msgq_get(&rx_msgq, &env, K_NO_WAIT);
        if (err) {
            LOG_WRN("k_msgq get fail(%d)", err);
        }
        else {
            zmk_split_transport_central_peripheral_event_handler(&esb_central, env.payload.source,
                                                                 env.payload.event);
        }
    }
}

K_THREAD_DEFINE(publish_events_thread_id, STACKSIZE,
        publish_events_thread, NULL, NULL, NULL,
        1, 0, 0);
