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

extern const enum esb_tx_power tx_power[];

#define RSSI_BASELINE (-60)
typedef enum{
    POWER_OK,
    POWER_UP,
    POWER_DOWN
} power_set_t;

typedef enum {
    NO_WAIT = -1
} timeout_t;

typedef enum {
    APP_ESB_EVT_TX_SUCCESS,
    APP_ESB_EVT_TX_FAIL,
    APP_ESB_EVT_RX
} app_esb_event_type_t;

typedef enum {
    APP_ESB_MODE_PTX,
    APP_ESB_MODE_PRX
} app_esb_mode_t;

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
ssize_t esb_tx_app();
void set_esb_active(bool is_active);
bool is_esb_active(void);
power_set_t check_rssi(int rssi);
void timeout_set(int timeout_us);
int tx_power_change(power_set_t cmd);
#endif
