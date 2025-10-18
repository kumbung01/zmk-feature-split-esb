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

#define MPSL_THREAD_PRIO             CONFIG_MPSL_THREAD_COOP_PRIO
#define STACKSIZE                    CONFIG_MAIN_STACK_SIZE

#define TX_BUFFER_SIZE                                                                             \
    ((sizeof(struct esb_event_envelope) + sizeof(struct esb_msg_postfix)) *                        \
     CONFIG_ZMK_SPLIT_ESB_EVENT_BUFFER_ITEMS) + 4
#define RX_BUFFER_SIZE                                                                             \
    ((sizeof(struct esb_command_envelope) + sizeof(struct esb_msg_postfix)) *                      \
     CONFIG_ZMK_SPLIT_ESB_CMD_BUFFER_ITEMS) + 4

static const uint8_t peripheral_id = CONFIG_ZMK_SPLIT_ESB_PERIPHERAL_ID;

static void process_tx_work_handler(struct k_work *work);
K_WORK_DEFINE(process_tx_work, process_tx_work_handler);

static struct zmk_split_esb_async_state async_state = {
    .process_tx_work = &process_tx_work,
};

void zmk_split_esb_on_ptx_esb_callback(app_esb_event_t *event) {
    zmk_split_esb_cb(event, &async_state);
}


extern struct k_msgq tx_msgq;
extern struct k_msgq rx_msgq;
extern struct k_sem tx_sem;
extern struct k_work_q esb_work_q;

static zmk_split_transport_peripheral_status_changed_cb_t transport_status_cb;
static bool is_enabled = false;

static int
split_peripheral_esb_report_event(const struct zmk_split_transport_peripheral_event *event) {
    struct esb_data_envelope env = { 
                                     .timestamp = k_uptime_get(),
                                     .event = *event
                                    };

    if (k_msgq_put(&tx_msgq, &env, K_MSEC(TIMEOUT_MS)) == 0) {
        k_sem_give(&tx_sem); 
    }

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

static int break_packet(struct esb_payload *payload) {
    int count = payload->data[0]; // first byte = number of events
    uint8_t source = payload->pipe;
    uint32_t nonce = get_u32_le(&payload->data[1]);
    uint8_t *data = &payload->data[5];
    LOG_WRN("RX packet with %d events from source %d", count, source);

    process_payload((char*)&payload->data[5], payload->length - 5, nonce);

    for (int i = 0; i < count; i++) {
        struct zmk_split_transport_central_command cmd = {0};

        cmd.type = (enum zmk_split_transport_central_command_type)data[0];
        data += 1;

        ssize_t data_size = get_payload_data_size_cmd(&cmd);


        if (data_size < 0) {
            LOG_ERR("Invalid data size %zd for command type %d", data_size, cmd.type);
            break;
        }

        memcpy(&cmd.data, data, data_size);
        data += data_size;

        LOG_DBG("RX command type %d from source %d", cmd.type, source);
        zmk_split_transport_peripheral_command_handler(&esb_peripheral, cmd);
    }

    return count;
}

static void process_tx_work_handler(struct k_work *work) {
    struct esb_payload payload;

    while (k_msgq_get(&rx_msgq, &payload, K_NO_WAIT) == 0) {
        break_packet(&payload);
    }
}
