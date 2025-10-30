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

static void event_handler(struct esb_evt const *event);
static struct esb_config config = {
    .protocol = ESB_PROTOCOL_ESB_DPL,
    .mode = ESB_MODE_PTX,
    .bitrate = IS_ENABLED(CONFIG_ZMK_SPLIT_ESB_BITRATE_2MBPS) ? ESB_BITRATE_2MBPS : ESB_BITRATE_1MBPS,
    .crc = ESB_CRC_16BIT,
    .retransmit_delay = CONFIG_ZMK_SPLIT_ESB_PROTO_TX_RETRANSMIT_DELAY,
    .retransmit_count = CONFIG_ZMK_SPLIT_ESB_PROTO_TX_RETRANSMIT_COUNT,
    .tx_mode = ESB_TXMODE_MANUAL_START,
    .payload_length = CONFIG_ESB_MAX_PAYLOAD_LENGTH,
    .selective_auto_ack = true,
    .use_fast_ramp_up = false,
    .event_handler = event_handler,
    .tx_output_power = -4,
};

K_SEM_DEFINE(tx_sem, 0, 1);

static app_esb_mode_t m_mode;

static atomic_t m_is_active = ATOMIC_INIT(0);
void set_esb_active(bool is_active) {
    atomic_set(&m_is_active, is_active ? 1 : 0);
}

bool is_esb_active(void) {
    return atomic_get(&m_is_active) ? true : false;
}

static atomic_t m_is_rx_working = ATOMIC_INIT(0);
bool try_start_rx_work() {
    return atomic_cas(&m_is_rx_working, 0, 1);
}

void rx_work_finished() {
    atomic_clear(&m_is_rx_working);
}

static void on_timeslot_start_stop(zmk_split_esb_timeslot_callback_type_t type);


static void event_handler(struct esb_evt const *event) {
    struct esb_payload *payload = NULL;
    static volatile int tx_fail_count = 0;

    switch (event->evt_id) {
        case ESB_EVENT_TX_SUCCESS:
            LOG_DBG("TX SUCCESS");
            break;
        case ESB_EVENT_TX_FAILED:
            LOG_WRN("ESB_EVENT_TX_FAILED");            
#if IS_PERIPHERAL
            if (tx_fail_count++ >= 5) {
                tx_fail_count = 0;
                esb_flush_tx();
                k_work_submit(esb_ops->tx_work);
            }
            esb_start_tx();
#endif
            break;
        case ESB_EVENT_RX_RECEIVED:
            LOG_DBG("RX SUCCESS");
            if (rx_alloc(&payload) != 0) {
                LOG_WRN("Failed to allocate rx_slab");
                return;
            }

            if (esb_read_rx_payload(payload) != 0) {
                LOG_WRN("esb_read_payload fail");
                rx_free(payload);
                return;
            }

            if (put_rx_data(payload) != 0) {
                LOG_ERR("k_msgq_put failed");
                rx_free(payload);
                return;
            }
            if (try_start_rx_work()) {
                LOG_DBG("rx_event submit");
                k_work_schedule(esb_ops->rx_work, K_NO_WAIT);
            }
            break;
    }
}


size_t esb_tx_app() {
    struct esb_payload payload;
    
    if (esb_tx_full()) {
        LOG_DBG("esb tx full, wait for next tx event");
        return 0;
    }

    size_t evt_count = make_packet(&payload);
    if (evt_count == 0) {
        LOG_DBG("no packet to send");
        return 0;
    }

    LOG_DBG("sending payload through pipe %d", payload.pipe);

    int ret = esb_write_payload(&payload);
    if (ret != 0) {
        LOG_WRN("esb_write_payload returned %d", ret);
        return 0;
    }

    ret = esb_start_tx();
    if (ret != 0 && ret != -EBUSY) {
        LOG_DBG("esb_start_tx() returned (%d)", ret);
        return 0;
    }

    return evt_count;
}


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



static int
esb_initialize(app_esb_mode_t mode)
{
    int err = esb_init(&config);
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

    err = esb_enable_pipes(ENABLED_PIPES);
    if (err) {
        return err;
    }

    const uint32_t channel = 22;
    LOG_DBG("setting rf channel to %d", channel);
    err = esb_set_rf_channel(channel);
    if (err < 0) {
        LOG_ERR("esb_set_rf_channel failed: %d", err);
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

int zmk_split_esb_init(app_esb_mode_t mode) {
    int ret;
    m_mode = mode;
    config.mode = mode == APP_ESB_MODE_PTX ? ESB_MODE_PTX : ESB_MODE_PRX;
    ret = clocks_start();
    if (ret < 0) {
        return ret;
    }
    LOG_INF("Timeslothandler init");
    zmk_split_esb_timeslot_init(on_timeslot_start_stop);
    
    return 0;
}

static atomic_t m_enabled = ATOMIC_INIT(0);
int zmk_split_esb_set_enable(bool enabled) {
    atomic_set(&m_enabled, enabled ? 1 : 0);
    if (enabled) {
        zmk_split_esb_timeslot_open_session();
        return 0;
    } else {
        zmk_split_esb_timeslot_close_session();
        return 0;
    }
}

bool zmk_split_esb_get_enable() {
    return atomic_get(&m_enabled) ? true : false;
}

static int app_esb_suspend(void) {
    set_esb_active(false);
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
    set_esb_active(true);
    return err;
}

static atomic_t tx_work_submit = ATOMIC_INIT(0);
/* Callback function signalling that a timeslot is started or stopped */
static void on_timeslot_start_stop(zmk_split_esb_timeslot_callback_type_t type) {
    switch (type) {
        case APP_TS_STARTED:
            app_esb_resume();
            if (atomic_cas(&tx_work_submit, 0, 1)) {
                k_work_submit(esb_ops->tx_work);
            }
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

    LOG_DBG("activity state changed: %s", ACTIVE_STATE_CHAR[state_ev->state]);

    if (m_mode == APP_ESB_MODE_PTX) {
        if (state_ev->state != ZMK_ACTIVITY_ACTIVE && zmk_split_esb_get_enable()) {
            zmk_split_esb_set_enable(false);
            atomic_set(&tx_work_submit, 0);
        }
        else if (state_ev->state == ZMK_ACTIVITY_ACTIVE && !zmk_split_esb_get_enable()) {
            zmk_split_esb_set_enable(true);
        }
    }

    return 0;
}



ZMK_LISTENER(zmk_split_esb_idle_sleeper, on_activity_state);
ZMK_SUBSCRIPTION(zmk_split_esb_idle_sleeper, zmk_activity_state_changed);
