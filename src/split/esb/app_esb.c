/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "app_esb.h"
#include "common.h"
#include "timeslot.h"
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <esb.h>

// for backoff logic
#include <zephyr/kernel.h>

#include <zmk/events/activity_state_changed.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app_esb, CONFIG_ZMK_SPLIT_ESB_LOG_LEVEL);

#define DT_DRV_COMPAT zmk_esb_split
#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
#define MPSL_THREAD_PRIO             CONFIG_MPSL_THREAD_COOP_PRIO

#define HAS_BASE_ADDR_0 (DT_INST_NODE_HAS_PROP(0, base_addr_0))
#define HAS_BASE_ADDR_1 (DT_INST_NODE_HAS_PROP(0, base_addr_1))
#define HAS_ADDR_PREFIX (DT_INST_NODE_HAS_PROP(0, addr_prefix))

#define BASE_ADDR_0_LEN (DT_INST_PROP_LEN(0, base_addr_0))
#define BASE_ADDR_1_LEN (DT_INST_PROP_LEN(0, base_addr_1))
#define ADDR_PREFIX_LEN (DT_INST_PROP_LEN(0, addr_prefix))

#if (!HAS_BASE_ADDR_0 || BASE_ADDR_0_LEN != 4)
#error "zmk,esb-split :: base-addr-0 must include 4 bytes"
#endif

#if (!HAS_BASE_ADDR_1 || BASE_ADDR_1_LEN != 4)
#error "zmk,esb-split :: base-addr-1 must include 4 bytes"
#endif

#if (!HAS_ADDR_PREFIX || ADDR_PREFIX_LEN != 8)
#error "zmk,esb-split :: addr-prefix must include 8 bytes"
#endif

uint8_t esb_base_addr_0[4] = DT_INST_PROP(0, base_addr_0);
uint8_t esb_base_addr_1[4] = DT_INST_PROP(0, base_addr_1);
uint8_t esb_addr_prefix[8] = DT_INST_PROP(0, addr_prefix);

#else
#error "Need to create a node with compatible of 'zmk,esb-split` with `all `address` property set."
#endif

#define RETRANSMIT_DELAY 0

#define TIMEOUT_MS CONFIG_ZMK_SPLIT_ESB_KEYBOARD_EVENT_TIMEOUT_MS

static app_esb_callback_t m_callback;

K_SEM_DEFINE(tx_sem, 1, 1);
K_SEM_DEFINE(rx_sem, 1, 1);

static app_esb_mode_t m_mode;
static bool m_active = false;
static bool m_enabled = false;
static void on_timeslot_start_stop(zmk_split_esb_timeslot_callback_type_t type);
static volatile int tx_fail_count = 0;

static void event_handler(struct esb_evt const *event) {
    app_esb_event_t m_event = {0};
    switch (event->evt_id) {
        case ESB_EVENT_TX_SUCCESS:
            // Forward an event to the application
            m_event.evt_type = APP_ESB_EVT_TX_SUCCESS;
            tx_fail_count = 0;
            k_sem_give(&tx_sem);
            break;
        case ESB_EVENT_TX_FAILED:
            // Forward an event to the application
            m_event.evt_type = APP_ESB_EVT_TX_FAIL;
            LOG_WRN("ESB_EVENT_TX_FAILED");
#if !IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
                if (tx_fail_count > 0) {
                    esb_pop_tx();
                    esb_start_tx();
                    tx_fail_count = 0;
                    k_sem_give(&tx_sem);
                }
                else {
                    tx_fail_count++;
                    esb_start_tx();
                }
#endif
            break;
        case ESB_EVENT_RX_RECEIVED:
            LOG_DBG("RX SUCCESS");
            m_event.evt_type = APP_ESB_EVT_RX;
            struct esb_payload *rx_payload = NULL;
            if (rx_alloc(rx_payload) != 0) {
                LOG_ERR("Failed to allocate rx_slab");
                break;
            }

            if (esb_read_rx_payload(rx_payload) == 0) {
                if (k_msgq_put(&rx_msgq, &rx_payload, K_NO_WAIT) != 0) {
                    LOG_ERR("k_msgq_put failed");
                    rx_free(rx_payload);
                    break;
                }
                k_sem_give(&rx_sem);
            }
            m_callback(&m_event);
            break;
    }
}

static int make_packet(struct k_msgq *msgq, struct esb_payload *payload, uint8_t type) {
    const uint32_t now = k_uptime_get();
    size_t count = 0;
    uint8_t offset = 0;
    struct payload_buffer *buf = payload->data;
    const size_t body_size = sizeof(buf->body);
    const size_t data_size = get_payload_data_size_buf(type, m_mode == APP_ESB_MODE_PRX);
#if !IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    payload->pipe = CONFIG_ZMK_SPLIT_ESB_PERIPHERAL_ID; // use the peripheral_id as the ESB pipe number
#endif
    payload->noack = !CONFIG_ZMK_SPLIT_ESB_PROTO_TX_ACK;

    while (true) {
        if (offset + data_size > body_size) {
            LOG_DBG("packet full (%u + %u > %d)", offset, data_size, body_size);
            break;
        }

        struct esb_data_envelope *env = NULL;
        int err = k_msgq_get(msgq, &env, K_NO_WAIT);
        if (err != 0) {
            LOG_DBG("k_msgq_get failed(%d).", err);
            break;
        }

        LOG_WRN("end of the test, type is (%d)", type);
        tx_free(env);
        break;
        
        uint32_t timestamp = env->timestamp;
        if (now - timestamp > TIMEOUT_MS) {
            LOG_DBG("dropping old event type %d timestamp %d", type, now - timestamp);
            tx_free(env);
            continue;
        }

        LOG_DBG("adding type %u size %u to packet", type, data_size);

        memcpy(&buf->body[offset], env->buf.data, data_size);
        offset += data_size;
        count++;

        tx_free(env);

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
        payload->pipe = env->source; // use the source as the ESB pipe number
        break;
#endif

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ESB_SINGLE_PACKET)
        break;
#endif
    }

    // write header and length
    if (count > 0) {
        // put_u32_le(&buf->header.nonce, nonce);
        buf->header.type = type;
        payload->length = offset + HEADER_SIZE;
    }

    // process_payload((char*)&payload->data[5], payload->length - HEADER_SIZE, nonce);

    return count;
}

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
void tx_work_handler() {
    while (true) {

        int type = -1;
        struct k_msgq *msgq = tx_msgq_ready(&type);
        if (msgq == NULL) {
            break;
        }

        struct esb_payload payload;
        int packet_count = make_packet(&msgq, &payload, type);
        if (packet_count == 0) {
            LOG_DBG("no packet to send");
            break;
        }

        LOG_WRN("TX packet with %d events on pipe %d", packet_count, payload.pipe);
        int ret = esb_write_payload(&payload);
        if (ret != 0) {
            LOG_DBG("esb_write_payload returned %d", ret);
            break;
        }
    }
}

K_WORK_DEFINE(tx_work, tx_work_handler);
#else
void tx_thread() {
    while (true)
    {
        k_sem_take(&tx_sem, K_FOREVER);
        LOG_DBG("tx thread awake");

        if (!is_esb_active()) {
            LOG_DBG("esb not active, retry later");
            continue;
        }

        while (true) {
            if (esb_tx_full()) {
                LOG_DBG("esb tx full, wait for next tx event");
                break;
            }

            int type = -1;
            struct k_msgq *msgq = tx_msgq_ready(&type);
            if (msgq == NULL) {
                break;
            }

            LOG_DBG("dequeueing msgq (%p), type (%d)", msgq, type);

            struct esb_payload payload;
            int packet_count = make_packet(&msgq, &payload, type);
            if (packet_count == 0) {
                LOG_DBG("no packet to send");
                break;
            }

            int ret = esb_write_payload(&payload);
            if (ret != 0) {
                LOG_WRN("esb_write_payload returned %d", ret);
                break;
            }

            ret = esb_start_tx();
            if (ret != -EBUSY) {
                LOG_DBG("esb_start_tx() returned (%d)", ret);
                break; // break if not busy but error
            }
        }
    }
}

K_THREAD_DEFINE(tx_thread_id, 1600,
        tx_thread, NULL, NULL, NULL,
        0, 0, 0);
#endif


static int clocks_start(void) {
    int err;
    int res;
    struct onoff_manager *clk_mgr;
    struct onoff_client clk_cli;

    clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
    if (!clk_mgr) {
        LOG_ERR("Unable to get the Clock manager");
        return -ENXIO;
    }

    sys_notify_init_spinwait(&clk_cli.notify);

    err = onoff_request(clk_mgr, &clk_cli);
    if (err < 0) {
        LOG_ERR("Clock request failed: %d", err);
        return err;
    }

    do {
        err = sys_notify_fetch_result(&clk_cli.notify, &res);
        if (!err && res) {
            LOG_ERR("Clock could not be started: %d", res);
            return res;
        }
    } while (err);

    LOG_DBG("HF clock started");
    
    return 0;
}

static int esb_initialize(app_esb_mode_t mode) {
    int err;
    struct esb_config config = ESB_DEFAULT_CONFIG;

    config.protocol = ESB_PROTOCOL_ESB_DPL;
    config.retransmit_delay = CONFIG_ZMK_SPLIT_ESB_PROTO_TX_RETRANSMIT_DELAY;
    config.retransmit_count = CONFIG_ZMK_SPLIT_ESB_PROTO_TX_RETRANSMIT_COUNT;
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ESB_BITRATE_2MBPS)
    config.bitrate = ESB_BITRATE_2MBPS;
#else
    config.bitrate = ESB_BITRATE_1MBPS;
#endif
    config.event_handler = event_handler;
    config.mode = (mode == APP_ESB_MODE_PTX) ? ESB_MODE_PTX : ESB_MODE_PRX;
    config.tx_mode = ESB_TXMODE_MANUAL_START;
    config.selective_auto_ack = true;

    err = esb_init(&config);

    if (err) {
        return err;
    }

    err = esb_set_base_address_0(esb_base_addr_0);
    if (err) {
        return err;
    }

    err = esb_set_base_address_1(esb_base_addr_1);
    if (err) {
        return err;
    }

    err = esb_set_prefixes(esb_addr_prefix, ARRAY_SIZE(esb_addr_prefix));
    if (err) {
        return err;
    }

    NVIC_SetPriority(RADIO_IRQn, 0);

    if (mode == APP_ESB_MODE_PRX) {
        esb_start_rx();
    }

    return 0;
}

#define ESB_TX_FIFO_REQUE_MAX (CONFIG_ZMK_SPLIT_ESB_PROTO_MSGQ_ITEMS \
                               * CONFIG_ZMK_SPLIT_ESB_PROTO_TX_RETRANSMIT_COUNT)

int zmk_split_esb_init(app_esb_mode_t mode, app_esb_callback_t callback) {
    int ret;
    m_callback = callback;
    m_mode = mode;
    ret = clocks_start();
    if (ret < 0) {
        return ret;
    }
    LOG_INF("Timeslothandler init");
    zmk_split_esb_timeslot_init(on_timeslot_start_stop);
    
    const uint32_t channel = 1;
    LOG_DBG("setting rf channel to %d", channel);
    ret = esb_set_rf_channel(channel);
    if (ret < 0) {
        LOG_ERR("esb_set_rf_channel failed: %d", ret);
    }

    return 0;
}

int zmk_split_esb_set_enable(bool enabled) {
    m_enabled = enabled;
    if (enabled) {
        zmk_split_esb_timeslot_open_session();
        return 0;
    } else {
        zmk_split_esb_timeslot_close_session();
        return 0;
    }
}

static int app_esb_suspend(void) {
    m_active = false;
    if(m_mode == APP_ESB_MODE_PTX) {
        uint32_t irq_key = irq_lock();

        irq_disable(RADIO_IRQn);
        NVIC_DisableIRQ(RADIO_IRQn);

        NRF_RADIO->SHORTS = 0;

        NRF_RADIO->EVENTS_DISABLED = 0;
        NRF_RADIO->TASKS_DISABLE = 1;
        while(NRF_RADIO->EVENTS_DISABLED == 0);

        NRF_TIMER2->TASKS_STOP = 1;
        NRF_RADIO->INTENCLR = 0xFFFFFFFF;
        
        esb_disable();

        NVIC_ClearPendingIRQ(RADIO_IRQn);

        irq_unlock(irq_key);
    }
    else {
        esb_stop_rx();
    }

    // Todo: Figure out how to use the esb_suspend() function 
    // rather than having to disable at the end of every timeslot
    //esb_suspend();
    return 0;
}

static int app_esb_resume(void) {
    int err = esb_initialize(m_mode);
    m_active = true;
    return err;
}

/* Callback function signalling that a timeslot is started or stopped */
static void on_timeslot_start_stop(zmk_split_esb_timeslot_callback_type_t type) {
    switch (type) {
        case APP_TS_STARTED:
            app_esb_resume();
            break;
        case APP_TS_STOPPED:
            app_esb_suspend();
            break;
    }
}

static int on_activity_state(const zmk_event_t *eh) {
    struct zmk_activity_state_changed *state_ev = as_zmk_activity_state_changed(eh);
    if (!state_ev) {
        return 0;
    }

    if (m_mode == APP_ESB_MODE_PTX) {
        if (state_ev->state != ZMK_ACTIVITY_ACTIVE && m_enabled) {
            zmk_split_esb_set_enable(false);
            reset_buffers();
            tx_fail_count = 0;
        }
        else if (state_ev->state == ZMK_ACTIVITY_ACTIVE && !m_enabled) {
            zmk_split_esb_set_enable(true);
#if !IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
            k_sem_give(&tx_sem);
#else
            k_work_submit_to_queue(&esb_work_q, &tx_work);
#endif
        }
    }

    return 0;
}

bool is_esb_active(void) {
    return m_active;
}

ZMK_LISTENER(zmk_split_esb_idle_sleeper, on_activity_state);
ZMK_SUBSCRIPTION(zmk_split_esb_idle_sleeper, zmk_activity_state_changed);
