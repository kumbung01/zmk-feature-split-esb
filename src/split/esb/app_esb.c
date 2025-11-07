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

static const enum esb_tx_power tx_power[] = {
#if defined(RADIO_TXPOWER_TXPOWER_Pos4dBm)
	/** 4 dBm radio transmit power. */
	ESB_TX_POWER_4DBM,
#endif /* defined(RADIO_TXPOWER_TXPOWER_Pos4dBm) */

#if defined(RADIO_TXPOWER_TXPOWER_Pos3dBm)
	/** 3 dBm radio transmit power. */
	ESB_TX_POWER_3DBM,
#endif /* defined(RADIO_TXPOWER_TXPOWER_Pos3dBm) */

	/** 0 dBm radio transmit power. */
	ESB_TX_POWER_0DBM,
	/** -4 dBm radio transmit power. */
	ESB_TX_POWER_NEG4DBM,
	/** -8 dBm radio transmit power. */
	ESB_TX_POWER_NEG8DBM,
	/** -12 dBm radio transmit power. */
	ESB_TX_POWER_NEG12DBM,
	/** -16 dBm radio transmit power. */
	ESB_TX_POWER_NEG16DBM,
	/** -20 dBm radio transmit power. */
	ESB_TX_POWER_NEG20DBM,
	/** -30 dBm radio transmit power. */
	ESB_TX_POWER_NEG30DBM,
	/** -40 dBm radio transmit power. */
#if defined(RADIO_TXPOWER_TXPOWER_Neg40dBm)
	ESB_TX_POWER_NEG40DBM,
#endif /* defined(RADIO_TXPOWER_TXPOWER_Neg40dBm) */
};

static void event_handler(struct esb_evt const *event);
static struct esb_config config = {
    .protocol = ESB_PROTOCOL_ESB_DPL,
    .mode = ESB_MODE_PTX,
    .bitrate = BITRATE,
    .crc = ESB_CRC_16BIT,
    .retransmit_delay = RETRANSMIT_DELAY,
    .retransmit_count = RETRANSMIT_COUNT,
    .tx_mode = ESB_TXMODE_MANUAL_START,
    .payload_length = CONFIG_ESB_MAX_PAYLOAD_LENGTH,
    .selective_auto_ack = true,
    .use_fast_ramp_up = true,
    .event_handler = event_handler,
    .tx_output_power = TX_POWER_INIT,
};

static size_t tx_power_idx = 0;
int tx_power_change(power_set_t cmd) {
    if (cmd == POWER_OK) {
        return 0;
    }

    size_t new_idx = tx_power_idx;
    switch (cmd) {
    case POWER_UP:
        new_idx--;
        break;
    case POWER_DOWN:
        new_idx++;
        break;
    default:
        return -EINVAL;
    }

    if (new_idx >= ARRAY_SIZE(tx_power)) {
        return -ENOTSUP;
    }

    tx_power_idx = new_idx;
    int8_t new_tx_output_power = (int8_t)tx_power[tx_power_idx];
    config.tx_output_power = new_tx_output_power;

    LOG_WRN("setting tx power to %d", new_tx_output_power);

    return esb_set_tx_power(new_tx_output_power);
}


K_SEM_DEFINE(tx_sem, 0, 1);
K_SEM_DEFINE(rx_sem, 0, RX_FIFO_SIZE);

static app_esb_mode_t m_mode;

static atomic_t m_is_active = ATOMIC_INIT(0);
void set_esb_active(bool is_active) {
    atomic_set(&m_is_active, is_active ? 1 : 0);
}

bool is_esb_active(void) {
    return atomic_get(&m_is_active) ? true : false;
}

static void start_tx_work_handler(struct k_work *work) {
    LOG_DBG("start_tx_work");
    esb_start_tx();
}
K_WORK_DELAYABLE_DEFINE(start_tx_work, start_tx_work_handler);

bool is_tx_delayed(void) {
    return k_work_delayable_is_pending(&start_tx_work);
}

static void on_timeslot_start_stop(zmk_split_esb_timeslot_callback_type_t type);

static void event_handler(struct esb_evt const *event) {
    static int tx_fail_count = 0;

    switch (event->evt_id) {
        case ESB_EVENT_TX_SUCCESS:
            LOG_DBG("TX SUCCESS");
            tx_fail_count = 0;
            esb_ops->tx_op();
            break;
        case ESB_EVENT_TX_FAILED:
            LOG_WRN("ESB_EVENT_TX_FAILED");            
#if IS_PERIPHERAL
            if (tx_fail_count++ >= 3) {
                tx_fail_count = 0;
                esb_pop_tx();
            }
            else if (tx_fail_count == 1) {
                tx_power_change(POWER_UP);
            }
#endif
            k_work_reschedule(&start_tx_work, K_USEC(SLEEP_DELAY));
            esb_ops->tx_op();
            break;
        case ESB_EVENT_RX_RECEIVED:
            LOG_DBG("RX SUCCESS");
            esb_ops->rx_op();
            break;
    }
}


int esb_tx_app() {
    static struct esb_payload payload;
    static bool write_payload_failed = false;
    int ret = 0;

    if (!is_esb_active()) {
        LOG_DBG("esb not active");
        return -EACCES;
    }

    if (esb_tx_full()) {
        LOG_DBG("esb tx full, wait for next tx event");
        ret = -ENOMEM;
        goto START_TX;
    }

    if (!write_payload_failed) {
        ret = make_packet(&payload);
        if (ret != 0) {
            LOG_DBG("no packet to send");
            return -ENODATA;
        }
    }

    LOG_DBG("sending payload through pipe %d", payload.pipe);

    write_payload_failed = esb_write_payload(&payload);
    if (write_payload_failed != 0) {
        LOG_WRN("esb_write_payload returned %d", write_payload_failed);
        return write_payload_failed;
    }

START_TX:
    if (is_tx_delayed()) {
        LOG_DBG("tx_delayed");
        return -EAGAIN;
    }

    LOG_DBG("start tx");
    esb_start_tx();
    return ret;
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


static atomic_t is_esb_initialized = ATOMIC_INIT(0);
static int
esb_initialize(app_esb_mode_t mode)
{
#if ESB_ONLY
    if (!atomic_cas(&is_esb_initialized, 0, 1)) {
        LOG_WRN("skip init");
        goto start;
    }
#endif
    
    LOG_DBG("esb init");
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

    const uint32_t channel = 12;
    LOG_DBG("setting rf channel to %d", channel);
    err = esb_set_rf_channel(channel);
    if (err < 0) {
        LOG_ERR("esb_set_rf_channel failed: %d", err);
        return err;
    }
start:
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

    for (int i = 0; i < ARRAY_SIZE(tx_power); ++i) {
        if ((int8_t)tx_power[i] == TX_POWER_INIT) {
            tx_power_idx = i;
            break;
        }
    }

    LOG_DBG("init power is %d", (int8_t)tx_power[tx_power_idx]);
    
    return 0;
}

static atomic_t m_enabled = ATOMIC_INIT(0);
int zmk_split_esb_set_enable(bool enabled) {
    atomic_set(&m_enabled, enabled ? 1 : 0);
    if (enabled) {
        zmk_split_esb_timeslot_open_session();
        if (esb_ops->works) {
            esb_ops->works();
        }
        return 0;
    } else {
        zmk_split_esb_timeslot_close_session();
        atomic_set(&is_esb_initialized, 0);
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
#if ESB_ONLY
        esb_suspend();
#else
        NRF_RADIO->SHORTS = 0;

        NRF_RADIO->EVENTS_DISABLED = 0;
        NRF_RADIO->TASKS_DISABLE = 1;
        while(NRF_RADIO->EVENTS_DISABLED == 0);

        NRF_TIMER2->TASKS_STOP = 1;
        NRF_RADIO->INTENCLR = 0xFFFFFFFF;

        esb_disable();
#endif
        NVIC_ClearPendingIRQ(RADIO_IRQn);

        irq_unlock(irq_key);
    }
    else {
#if ESB_ONLY
        esb_suspend();
#else
        esb_stop_rx();
#endif
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

static atomic_t oneshot = ATOMIC_INIT(0);
/* Callback function signalling that a timeslot is started or stopped */
static void on_timeslot_start_stop(zmk_split_esb_timeslot_callback_type_t type) {
    switch (type) {
        case APP_TS_STARTED:
            app_esb_resume();
            if (atomic_cas(&oneshot, 0, 1)) {
                esb_ops->tx_op();
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
            atomic_clear(oneshot);
        }
        else if (state_ev->state == ZMK_ACTIVITY_ACTIVE && !zmk_split_esb_get_enable()) {
            zmk_split_esb_set_enable(true);
        }
    }

    return 0;
}



ZMK_LISTENER(zmk_split_esb_idle_sleeper, on_activity_state);
ZMK_SUBSCRIPTION(zmk_split_esb_idle_sleeper, zmk_activity_state_changed);
