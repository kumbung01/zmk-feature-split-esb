/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef __ZMK_SPLIT_ESB_TIMESLOT_H
#define __ZMK_SPLIT_ESB_TIMESLOT_H

typedef enum {
  APP_TS_STARTED,
  APP_TS_STOPPED
} zmk_split_esb_timeslot_callback_type_t;

typedef void (*zmk_split_esb_timeslot_callback_t)(zmk_split_esb_timeslot_callback_type_t type);

void zmk_split_esb_timeslot_init(zmk_split_esb_timeslot_callback_t callback);

void zmk_split_esb_timeslot_open_session(void);

void zmk_split_esb_timeslot_close_session(void);

bool is_in_timeslot(void);

#endif
