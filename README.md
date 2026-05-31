# ZMK ESB Split Transport

This is a [ZMK](https://zmk.dev) _Split Transport_ module adding support for [Enhanced ShockBurst (ESB)](https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/protocols/esb/index.html) protocol on Nordic nRF5 Series devices.

This work is based on [zmk-feature-split-esb](https://github.com/badjeff/zmk-feature-split-esb), [zmk,wired-split](https://github.com/zmkfirmware/zmk/tree/main/app/src/split/wired) and [nRF Connect SDK > ESB Examples](https://docs.nordicsemi.com/bundle/ncs-2.6.4/page/nrf/samples/esb.html).

**currently tested only on NRF52840.**
**input event not tested.(e.g. mouse)**

## What it does

This module implements a custom protocol based on Nordic ESB physical layer from [nRF Connect SDK (NCS)](<[https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/index.html](https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/index.html)>), featuring TDMA-based scheduling and FHSS across 39 channels (1~79), instead of the Zephyr BLE stack. The protocol supports two-way data packet communication, packet buffering, packet acknowledgment, and automatic retransmission.

Key features:

- **TDMA-based scheduling** for deterministic radio timeslot management
- **FHSS (Frequency Hopping Spread Spectrum)** across 39 channels (1~79) using slot-based hash for channel selection
- **USB-only dongle** topology; central connects to HID host via USB, peripherals connect via ESB
- Latency: varies by polling rate and peripheral count.
- e.g. 2kHz polling with 2 peripherals: 1ms per slot typical, may increase under poor radio conditions.

## Installation

Include this project in your ZMK west manifest (`config/west.yml`):

```yaml

remotes:
  - name: kumbung01
    url-base: https://github.com/kumbung01
  - name: nrfconnect
    url-base: https://github.com/nrfconnect

projects:
  - name: zmk-split-esb
    remote: kumbung01
    revision: main
  - name: sdk-nrf
    remote: kumbung01
    revision: v3.1-branch+zmk-fixes
    path: nrf
  - name: nrfxlib
    remote: nrfconnect
    revision: v3.1-branch
    repo-path: sdk-nrfxlib
    path: nrfxlib
```

Add the ESB arbitrary address to `{shield}.overlay` for both central and peripherals:

```dts
/ {
    esb_split {
        compatible = "zmk,esb-split";
        // These are arbitrary default addresses.
        // Use different addresses for each set of devices in production.
        base-addr-0 = <0xE7 0xE7 0xE7 0xE7>;
        base-addr-1 = <0xC2 0xC2 0xC2 0xC2>;
        addr-prefix = <0xE7 0xC2 0xC3 0xC4 0xC5 0xC6 0xC7 0xC8>;
    };
};
```

## Configuration

### Core

```conf

# Disable BLE
CONFIG_ZMK_BLE=n

# Disable default split transports
CONFIG_ZMK_SPLIT_BLE=n
CONFIG_ZMK_SPLIT_WIRED=n

# Enable ESB split transport
CONFIG_ZMK_SPLIT_ESB=y

# Peripheral ID (assign a unique integer to each peripheral, must start from 0)
CONFIG_ZMK_SPLIT_ESB_PERIPHERAL_ID=0
```

### ESB Options

| Config                                                 | Default | Description                     |
| ------------------------------------------------------ | ------- | ------------------------------- |
| `CONFIG_ZMK_SPLIT_ESB_BITRATE_2MBPS`                   | `y`     | Use 2Mbps bitrate               |
| `CONFIG_ZMK_SPLIT_ESB_CENTRAL_RX_THREAD_STACK_SIZE`    | `2560`  | Central RX thread stack size    |
| `CONFIG_ZMK_SPLIT_ESB_PERIPHERAL_RX_THREAD_STACK_SIZE` | `1024`  | Peripheral RX thread stack size |

### Battery Reporting

```conf
# Central
CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS=2
CONFIG_ZMK_BATTERY_REPORTING=y
CONFIG_ZMK_SPLIT_BLE_CENTRAL_BATTERY_LEVEL_FETCHING=y

# Peripheral
CONFIG_ZMK_BATTERY_REPORTING=y
```

> **Note:** `CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS` is used even in ESB-only setups to configure the number of peripherals.

## License

- See `SPDX-License-Identifier` in each file header
    - [LicenseRef-Nordic-5-Clause](https://github.com/nrfconnect/sdk-nrf/blob/main/LICENSE) — from [nRF Connect SDK](https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/index.html)
    - [MIT](https://github.com/zmkfirmware/zmk/blob/main/LICENSE) — from [ZMK](https://github.com/zmkfirmware/zmk/)
