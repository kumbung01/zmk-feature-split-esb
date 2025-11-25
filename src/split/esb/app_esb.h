/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef __APP_ESB_H
#define __APP_ESB_H

#include <zephyr/kernel.h>
#include <esb.h>

extern struct k_sem tx_sem;
extern struct k_sem rx_sem;

#define RF_CHANNEL CONFIG_ZMK_SPLIT_ESB_RF_CHANNEL
#define TIMESLOTS_PER_CHANNEL 2
#define TX_PERIOD CONFIG_ZMK_SPLIT_ESB_PROTO_TX_RETRANSMIT_DELAY
#define RX_PERIOD (TIMESLOTS_PER_CHANNEL * TX_PERIOD)

#define RSSI_BASELINE (-60)
typedef enum { POWER_OK, POWER_UP, POWER_DOWN } power_set_t;

typedef enum { NO_WAIT = -1 } timeout_t;

typedef enum { APP_ESB_EVT_TX_SUCCESS, APP_ESB_EVT_TX_FAIL, APP_ESB_EVT_RX } app_esb_event_type_t;

typedef enum { APP_ESB_MODE_PTX, APP_ESB_MODE_PRX } app_esb_mode_t;

/* Radio Tx ramp-up time in microseconds. */
#define TX_RAMP_UP_TIME_US 129

/* Radio Rx fast ramp-up time in microseconds. */
#define TX_FAST_RAMP_UP_TIME_US 40

/* Radio Rx ramp-up time in microseconds. */
#define RX_RAMP_UP_TIME_US 124

typedef struct {
    app_esb_event_type_t evt_type;
} app_esb_event_t;

typedef struct {
    app_esb_mode_t mode;
} app_esb_config_t;

typedef void (*app_esb_callback_t)(app_esb_event_t *event);

struct esb_simple_addr {
    uint8_t base_0[4];
    uint8_t base_1[4];
    uint8_t prefix[8];
};

int zmk_split_esb_init(app_esb_mode_t mode);

int zmk_split_esb_set_enable(bool enabled);
bool zmk_split_esb_get_enable();

int pull_packet_from_tx_msgq(void);
int esb_tx_app();
void set_esb_active(bool is_active);
bool is_esb_active(void);
bool is_esb_initialized(void);
bool is_tx_oneshot_set(void);
void set_esb_enabled(bool enabled);
bool is_esb_enabled();
power_set_t check_rssi(int rssi);
int tx_power_change(power_set_t cmd);
bool is_tx_delayed();
void set_tx_delayed(bool set);
void timeslot_counter();
void change_channel();
#endif
