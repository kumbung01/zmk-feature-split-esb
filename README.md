# ZMK ESB Split Transport

This is an experimental [ZMK](https://zmk.dev) *Split Transport* module adding support for [Enhanced ShockBurst (ESB)](https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/protocols/esb/index.html) protocol on Nordic nRF5 Series device.

This work is based on [zmk,wired-split](https://github.com/zmkfirmware/zmk/tree/main/app/src/split/wired), [nRF Connect SDK > ESB Examples](https://docs.nordicsemi.com/bundle/ncs-2.6.4/page/nrf/samples/esb.html) and [ncs-esb-ble-mpsl-demo](https://github.com/too1/ncs-esb-ble-mpsl-demo/).


## What it does

This module uses [nRF Connect SDK (NCS)](https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/index.html) ESB implementation as communication protocol between ZMK central and peripherals, instead of Zephyr BLE stack. The protocol implementation is supporting ~~two-way data packet communication (should work, not tested yet),~~ packet buffering, packet acknowledgment, and automatic retransmission, etc. All devices could be communicated with predefined semantic address. 

This module also uses [Multi-Protocol Service Layer (MPSL)](https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/protocols/multiprotocol/index.html) library which provides services for multiprotocol applications that allows the nRF5 radio driver to negotiate for transmission timeslots. As result, ZMK central allows to pair BLE host as conventional HID input device (keyboard & mouse) and act as an ESB transceiver simultaneously. And all ZMK peripherals talk to ZMK central over ESB only with reduced packet overhead.

However, the MCU embedded radio controller in nRF52840 (which i used to develop on) doesn't have enough resource to establish ESB connection and perform BLE advertisting & scanning between central and peripheral at the same time. In short, central doesn't have timeslots to scan peripherals, and peripheral doesn't have timeslots to advertisting itself to central. But, central has configed as ESB Primarily Receiver (PRX) and it has free timeslots to advertise itself to HID host.


## Installation

Include this project on your ZMK's west manifest in `config/west.yml`:
```diff
  [...]
  remotes:
+    - name: badjeff
+      url-base: https://github.com/badjeff
+    - name: nrfconnect
+      url-base: https://github.com/nrfconnect
  projects:
+    - name: zmk-feature-split-esb
+      remote: badjeff
+      revision: main  
+    - name: sdk-nrf
+      remote: nrfconnect
+      revision: v2.6.4 # zephyr v3.5.99
+      path: nrf
+    - name: nrfxlib
+      remote: nrfconnect
+      revision: v2.6.4
+     repo-path: sdk-nrfxlib
+     path: nrfxlib
  [...]
```

Update `{shield}.conf` to enable ESB Split Transport.
```conf
# enable BLE on central (if needed)
CONFIG_ZMK_BLE=y

# disable BLE on peripheral
CONFIG_ZMK_BLE=n

# disable default split transport on central and peripheral
CONFIG_ZMK_SPLIT_BLE=n
CONFIG_ZMK_SPLIT_WIRED=n

# enable split esb transport
CONFIG_ZMK_SPLIT_ESB=y

# assige a source id on peripheral. (no need for central)
# default zero. give an integer (<256) on all peripheral(s)
CONFIG_ZMK_SPLIT_ESB_PERIPHERAL_ID=1

# enable ESB TX send request packet payload with ACK bit
# ESB protocol has built-in retransmit counter (default one), if RX does not response ACK properly.
# disable this iif you are pursuing extreme low latency, not much different in real-life experiment.
CONFIG_ZMK_SPLIT_ESB_PROTO_TX_ACK=y

# extra requeue after ESB retransmition attempts fail (default 3)
# use this wisely on keyboard peripheral for packet delivery ordering in noisy environnment.
CONFIG_ZMK_SPLIT_ESB_EXTRA_RETRY_AFTER_RETRAN=3

# enable Multi-Protocol Service Layer (MPSL)
# set 2, if CONFIG_ZMK_BLE is enabled on central, which uses BLE and ESB simultaneously
# set 1, if CONFIG_ZMK_BLE is disabled on both central and peripherals. (dongle-only mode)
CONFIG_MPSL_TIMESLOT_SESSION_COUNT=2

# Number of message queue size to buffer ESB payload for TX in between multi-protocol service 
# timeslots (CONFIG_MPSL_TIMESLOT_SESSION_COUNT)
CONFIG_ZMK_SPLIT_ESB_PROTO_MSGQ_ITEMS=16

# qeuue size for both peripheral (EVENT) and central (CMD)
CONFIG_ZMK_SPLIT_ESB_EVENT_BUFFER_ITEMS=16
CONFIG_ZMK_SPLIT_ESB_CMD_BUFFER_ITEMS=4

# another config for ESB
CONFIG_CLOCK_CONTROL_NRF_K32SRC_RC=y
CONFIG_SYSTEM_WORKQUEUE_STACK_SIZE=2048
CONFIG_ESB_MAX_PAYLOAD_LENGTH=64
# CONFIG_ZMK_SPLIT_ESB_LOG_LEVEL_DBG=y
```

And, add ESB arbitrary address to `{shield}.overlay` of your central and peripherals.
```c
/{
    esb_split {
        compatible = "zmk,esb-split";
        // These are arbitrary default addresses. In end user products
        // different addresses should be used for each set of devices.
        base-addr-0 = <0xE7 0xE7 0xE7 0xE7>;
        base-addr-1 = <0xC2 0xC2 0xC2 0xC2>;
        addr-prefix = <0xE7 0xC2 0xC3 0xC4 0xC5 0xC6 0xC7 0xC8>;
    };
};
```


## Local Development Setup

Pull nRF Connect SDK repositories via west.
```shell
git clone https://github.com/badjeff/zmk-feature-split-esb.git
cd zmk-feature-split-esb
west init -l config/
west update nrf
west update nrfxlib
```

Build with *ZMK_EXTRA_MODULES*
```shell
git clone https://github.com/zmkfirmware/zmk.git
cd zmk
export NRF_MODULE_DIRS="../zmk-feature-split-esb/nrf"
export NRFXLIB_MODULE_DIRS="../zmk-feature-split-esb/nrfxlib"
ln -s "${NRF_MODULE_DIRS}" nrf
ln -s "${NRFXLIB_MODULE_DIRS}" nrfxlib
export ZMK_ESB_MODULE_DIRS="../zmk-feature-split-esb"
export ZMK_MODULE_DIRS="${ZMK_ESB_MODULE_DIRS};${NRF_MODULE_DIRS};${NRFXLIB_MODULE_DIRS}"
export SHIELD="corne_left"
export BOARD="nice_nano_v2"
export ZMK_CONFIG_DIR="../zmk-config"
west build -d "build/${SHIELD}" -b "${BOARD}" -S zmk-usb-logging -- \
  -DZMK_EXTRA_MODULES="${ZMK_MODULE_DIRS}" \
  -DSHIELD="${SHIELD}" -DZMK_CONFIG="${ZMK_CONFIG_DIR}"
```


## License

- See `SPDX-License-Identifier` in each file heading
  * [LicenseRef-Nordic-5-Clause](https://github.com/nrfconnect/sdk-nrf/blob/main/LICENSE) licensed from [nRF Connect SDK](https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/index.html)
  * [MIT](https://github.com/zmkfirmware/zmk/blob/main/LICENSE) licensed from [ZMK](https://github.com/zmkfirmware/zmk/)
