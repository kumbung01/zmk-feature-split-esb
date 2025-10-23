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


extern struct k_work_q esb_work_q;

extern struct k_sem tx_sem;
extern struct k_sem rx_sem;
extern struct k_mem_slab tx_slab;
extern struct k_mem_slab rx_slab;
extern struct k_msgq tx_msgq;
extern struct k_msgq rx_msgq;
extern struct k_work tx_work;
static zmk_split_transport_central_status_changed_cb_t transport_status_cb;
static bool is_enabled = false;

static void process_tx_work_handler(struct k_work *work) {}
K_WORK_DEFINE(process_tx_work, process_tx_work_handler);

static struct zmk_split_esb_async_state async_state = {
    // .process_tx_work = &process_tx_work,
};

static int split_central_esb_send_command(uint8_t source,
                                          struct zmk_split_transport_central_command cmd) {
    struct esb_data_envelope *env;
    
    ssize_t data_size = get_payload_data_size_cmd(cmd.type);
    if (data_size < 0) {
        LOG_ERR("get_payload_data_size_cmd failed (err %d)", data_size);
        return -ENOTSUP;
    }

    int ret = k_mem_slab_alloc(&tx_slab, (void **)&env, K_NO_WAIT);
    if (ret < 0) {
        LOG_ERR("k_mem_slab_alloc failed (err %d)", ret);
        return -ENOMEM;
    }

    env->command = cmd;
    env->source = source;
    env->timestamp = k_uptime_get();

    ret = k_msgq_put(&tx_msgq, &env, K_MSEC(TIMEOUT_MS));
    if (ret < 0) {
        LOG_ERR("k_msgq_put failed (err %d)", ret);
        k_mem_slab_free(&tx_slab, (void *)env);
        return ret;
    }

    LOG_DBG("Queued ptr %p to tx_msgq", env);
    LOG_HEXDUMP_DBG(env, sizeof(*env), "ptr:");

    k_work_submit_to_queue(&esb_work_q, &tx_work);

    return 0;
}

void zmk_split_esb_on_prx_esb_callback(app_esb_event_t *event) {
    zmk_split_esb_cb(event, &async_state);
}

static int split_central_esb_get_available_source_ids(uint8_t *sources) {
    sources[0] = 0;

    return 1;
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
    async_state.central_transport = &esb_central;

    int ret = zmk_split_esb_init(APP_ESB_MODE_PRX, zmk_split_esb_on_prx_esb_callback);
    if (ret) {
        LOG_ERR("zmk_split_esb_init failed (err %d)", ret);
        return ret;
    }

    service_init();

    k_work_submit_to_queue(&esb_work_q, &notify_status_work);
    return 0;
}

SYS_INIT(zmk_split_esb_central_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);


static void publish_events_thread() {
    while (true)
    {
        k_sem_take(&rx_sem, K_FOREVER);
        handle_packet(&async_state);
    }
}

K_THREAD_DEFINE(publish_events_thread_id, STACKSIZE,
        publish_events_thread, NULL, NULL, NULL,
        -1, 0, 0);


