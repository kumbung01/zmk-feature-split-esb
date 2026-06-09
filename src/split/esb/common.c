/*
 * Copyright (c) 2025 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include "common.h"

#include <zephyr/sys/crc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
#include <zmk/split/transport/central.h>
#else
#include <zmk/split/transport/peripheral.h>
#endif
// #include <esb.h>
#include <zephyr/drivers/hwinfo.h>
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_SPLIT_ESB_LOG_LEVEL);

bool change_position_state(uint8_t position, bool is_pressed, uint8_t *position_state) {

    bool is_changed = IS_BIT_SET(position_state[position / 8], position % 8) != is_pressed;

    WRITE_BIT(position_state[position / 8], position % 8, is_pressed);
    return is_changed;
}

void init_slot(struct peripheral_slot_context *slot) {
    memset(slot->position_state, 0, sizeof(slot->position_state));
}

ssize_t get_payload_data_size_cmd(int _type) {
    ssize_t size = -1;

    switch (_type) {
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_POLL_EVENTS:
        size = 0;
        break;
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_INVOKE_BEHAVIOR:
        size = sizeof(((struct zmk_split_transport_central_command *)0)->data.invoke_behavior);
        break;
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_SET_PHYSICAL_LAYOUT:
        size = sizeof(((struct zmk_split_transport_central_command *)0)->data.set_physical_layout);
        break;
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_SET_HID_INDICATORS:
        size = sizeof(((struct zmk_split_transport_central_command *)0)->data.set_hid_indicators);
        break;
    default:
        size = -ENOTSUP;
        break;
    }

    // LOG_DBG("cmd type (%d) size (%d)", _type, size);

    return size;
}

ssize_t get_payload_data_size_evt(int _type) {
    ssize_t size = -1;

    switch (_type) {
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_KEY_POSITION_EVENT:
#if CONFIG_ZMK_SPLIT_ESB_SEND_WHOLE_KEY
        size = POSITION_STATE_DATA_LEN;
#else
        size = sizeof(((struct zmk_split_transport_peripheral_event *)0)->data.key_position_event);
#endif
        break;
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_SENSOR_EVENT:
        size = sizeof(((struct zmk_split_transport_peripheral_event *)0)->data.sensor_event);
        break;
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_INPUT_EVENT:
        size = sizeof(((struct zmk_split_transport_peripheral_event *)0)->data.input_event);
        break;
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_BATTERY_EVENT:
        size = sizeof(((struct zmk_split_transport_peripheral_event *)0)->data.battery_event);
        break;
    default:
        size = -ENOTSUP;
        break;
    }

    // LOG_DBG("evt type (%d) size (%d)", _type, size);

    return size;
}

#if CONFIG_THREAD_STACK_INFO && CONFIG_INIT_STACKS
static void check_stack_usage(struct k_thread *thread, const char *name) {
    size_t unused_stack;

    int ret = k_thread_stack_space_get(thread, &unused_stack);
    if (ret == 0) {
        size_t total_size = thread->stack_info.size;
        size_t used_stack = total_size - unused_stack;

        LOG_WRN("\"%s\": Max Used %zu bytes / Total %zu bytes (%u%%)", name, used_stack, total_size,
                (uint32_t)(used_stack * 100 / total_size));

    } else {
        LOG_WRN("Error: Failed to get stack space for %s (ret: %d)", name, ret);
    }
}

static struct k_thread *thread;
static void thread_info_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    k_work_reschedule(dwork, K_SECONDS(1));
    if (thread == NULL) {
        return;
    }
    check_stack_usage(thread, "rx_thread");
}
static K_WORK_DELAYABLE_DEFINE(thread_info_work, thread_info_work_cb);

void schedule_thread_info_check_work(int interval_ms) {
    thread = k_current_get();
    k_work_reschedule(&thread_info_work, K_MSEC(interval_ms));
}
#endif // CONFIG_THREAD_STACK_INFO && CONFIG_INIT_STACKS

void print_reset_reason(void) {
    uint32_t cause = 0;
    int ret = hwinfo_get_reset_cause(&cause);

    if (ret < 0) {
        LOG_ERR("Failed to get reset cause (err %d)", ret);
        return;
    }

    LOG_INF("Reset cause raw value: 0x%08x", cause);

    if (cause & RESET_PIN) {
        LOG_INF("Reset reason: Pin reset");
    }
    if (cause & RESET_SOFTWARE) {
        LOG_INF("Reset reason: Software reset");
    }
    if (cause & RESET_WATCHDOG) {
        LOG_INF("Reset reason: Watchdog");
    }
    if (cause & RESET_BROWNOUT) {
        LOG_INF("Reset reason: Brownout / Power failure");
    }
    if (cause & RESET_POR) {
        LOG_INF("Reset reason: Power-on reset");
    }

    hwinfo_clear_reset_cause();
}
