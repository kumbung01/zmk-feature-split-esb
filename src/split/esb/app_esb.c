/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "app_esb.h"
#include "common.h"
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <esb.h>
#include <zmk/events/endpoint_changed.h>

// for backoff logic
#include <zephyr/kernel.h>

#include <zmk/events/activity_state_changed.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app_esb, CONFIG_ZMK_SPLIT_ESB_LOG_LEVEL);
#define DT_DRV_COMPAT zmk_esb_split
#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#define IS_CENTRAL IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
#define IS_PERIPHERAL !IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)

#if IS_CENTRAL
#define TX_POWER_INIT (4)
#define ENABLED_PIPES GENMASK(PERIPHERAL_COUNT - 1, 0)
#else
#define TX_POWER_INIT (-4)
#define ENABLED_PIPES BIT(PERIPHERAL_ID)
#endif

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ESB_BITRATE_2MBPS)
#define BITRATE ESB_BITRATE_2MBPS
#else
#define BITRATE ESB_BITRATE_1MBPS
#endif

#if CONFIG_LOG
const char *ACTIVE_STATE_CHAR[] = {"ACTIVE", "IDLE", "SLEEP"};
const char *TX_POWER_CHAR[] = {"OK", "UP", "DOWN"};
#endif

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

struct esb_context *esb_ctx;

static void event_handler(struct esb_evt const *event);
static struct esb_config config = {
    .protocol = ESB_PROTOCOL_ESB_DPL,
    .mode = ESB_MODE_PTX,
    .bitrate = BITRATE,
    .crc = ESB_CRC_16BIT,
    .tx_mode = ESB_TXMODE_AUTO,
    .payload_length = CONFIG_ESB_MAX_PAYLOAD_LENGTH,
    .use_fast_ramp_up = true,
    .event_handler = event_handler,
    .tx_output_power = TX_POWER_INIT,
#if IS_PERIPHERAL
    .pipe = SOURCE_TO_PIPE(CONFIG_ZMK_SPLIT_ESB_PERIPHERAL_ID),
#endif
};

static void event_handler(struct esb_evt const *event) {
    switch (event->evt_id) {
    case ESB_EVENT_TX_SUCCESS:
        LOG_DBG("TX SUCCESS: TRY %u", event->tx.tx_attempts);
        if (esb_ctx->tx_op)
            esb_ctx->tx_op();
        break;
    case ESB_EVENT_TX_FAILED:
        LOG_WRN("ESB_EVENT_TX_FAILED");
        break;
    case ESB_EVENT_RX_RECEIVED:
        LOG_DBG("RX SUCCESS");
        if (esb_ctx->rx_op)
            esb_ctx->rx_op();
        break;
    }
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

static int esb_initialize(void) {
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

    // NVIC_SetPriority(RADIO_IRQn, 0);

    LOG_DBG("esb init done");

    return 0;
}

int zmk_split_esb_init(void) {
    int ret;
    config.mode = IS_PERIPHERAL ? ESB_MODE_PTX : ESB_MODE_PRX;
    ret = clocks_start();
    if (ret < 0) {
        return ret;
    }

    esb_initialize();
#if IS_PERIPHERAL
    esb_tdma_start();
#endif
    return 0;
}

static bool is_valid_source(const uint8_t source) {
    return BIT(SOURCE_TO_PIPE(source)) & ENABLED_PIPES;
}

int send_data(uint8_t source, uint8_t type, void *data) {
    if (!is_valid_source(source)) {
        LOG_WRN("invalid source(%d)", source);

        return -EINVAL;
    }

    ssize_t data_size = esb_ctx->tx_size(type);
    if (data_size < 0) {
        LOG_WRN("invalid type(%d)", type);

        return -EINVAL;
    }

    struct esb_payload payload = {.pipe = SOURCE_TO_PIPE(source)};

    int idx = 0;
#if CONFIG_ESB_TX_RINGBUF
    payload.length = data_size + 2;
    payload.data[idx++] = HEADER;
#else
    payload.length = data_size + 1;
#endif
    payload.data[idx++] = type;
    memcpy(&payload.data[idx], data, data_size);

    int err = esb_write_payload(&payload);
    if (err) {
        LOG_DBG("esb_write_payload error(%d).", err);

        return err;
    }

    return 0;
}

#define RXBUF_SIZE (CONFIG_ESB_MAX_PAYLOAD_LENGTH * 2)
static struct rx_buffer {
    uint8_t length;
    uint8_t data[RXBUF_SIZE];
} rxbuf[PERIPHERAL_COUNT];

static struct rx_buffer *get_buffer(int source) {
#if IS_CENTRAL
    return &rxbuf[SOURCE_TO_PIPE(source)];
#else
    return &rxbuf[0];
#endif
}

static int handle_data_dr(void) {
    struct esb_payload payload;
    int ret = esb_read_rx_payload(&payload);
    if (ret != 0) {
        LOG_DBG("esb_read_rx_payload returned err(%d)", ret);

        return -ENODATA;
    }

    int source = PIPE_TO_SOURCE(payload.pipe);
    if (!is_valid_source(source)) {
        LOG_WRN("invalid source(%d)", source);

        return -EINVAL;
    }

    LOG_HEXDUMP_DBG(payload.data, payload.length, "rx ");

    esb_ctx->rx_handler(source, payload.data, payload.length);
}

static int handle_data_rb(void) {
    struct esb_payload payload;
    int ret = esb_read_rx_payload(&payload);
    if (ret != 0) {
        LOG_DBG("esb_read_rx_payload returned err(%d)", ret);

        return -ENODATA;
    }

    LOG_DBG("pipe %d len %d", payload.pipe, payload.length);

    int source = PIPE_TO_SOURCE(payload.pipe);
    if (!is_valid_source(source)) {
        LOG_WRN("invalid source(%d)", source);

        return -EINVAL;
    }

    struct rx_buffer *buffer = get_buffer(source);

    uint8_t len = buffer->length;
    uint8_t *buf = buffer->data;

    __ASSERT(payload.length + len <= RXBUF_SIZE, "payload + buf(%u) > RXBUF_SIZE(%u)",
             payload.length + len, RXBUF_SIZE);

    memcpy(&buf[len], payload.data, payload.length);
    len += payload.length;

    // LOG_WRN("payload: source %d, len %u, total: %u", source, payload.length, len);

    int used = 0;
    while (used < len) {
        uint8_t header = buf[used];
        if (header != HEADER) {
            used++;
            continue;
        }

        uint8_t consumed = esb_ctx->rx_handler(source, &buf[used + 1], len - used - 1);
        if (consumed == 0) {
            break;
        }

        used += consumed + 1;
        // LOG_WRN("used: %d", used);
    }

    int left = len - used;
    // LOG_WRN("used: %d, left: %d", used, left);

    memmove(buf, &buf[used], left);
    buffer->length = left;

    return 0;
}

int handle_data(void) {
#if CONFIG_ESB_TX_RINGBUF
    return handle_data_rb();
#else
    return handle_data_dr();
#endif
}

static int on_activity_state(const zmk_event_t *eh) {
    struct zmk_activity_state_changed *state_ev = as_zmk_activity_state_changed(eh);
    if (!state_ev) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    LOG_DBG("activity state changed: %s", ACTIVE_STATE_CHAR[state_ev->state]);

    switch (state_ev->state) {
    case ZMK_ACTIVITY_ACTIVE:
        esb_tdma_start();
        break;
    case ZMK_ACTIVITY_IDLE:
#if IS_PERIPHERAL
        esb_tdma_stop();
#endif
        break;
    case ZMK_ACTIVITY_SLEEP:
        esb_tdma_stop();
        break;
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(zmk_split_esb_idle_sleeper, on_activity_state);
ZMK_SUBSCRIPTION(zmk_split_esb_idle_sleeper, zmk_activity_state_changed);

#if IS_CENTRAL
static int my_endpoint_handler(const zmk_event_t *eh) {
    struct zmk_endpoint_changed *ev = as_zmk_endpoint_changed(eh);
    if (ev->endpoint.transport == ZMK_TRANSPORT_USB) {
        esb_tdma_start();
    }
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(my_listener, my_endpoint_handler);
ZMK_SUBSCRIPTION(my_listener, zmk_endpoint_changed);
#endif