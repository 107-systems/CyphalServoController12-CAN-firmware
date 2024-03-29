<a href="https://107-systems.org/"><img align="right" src="https://raw.githubusercontent.com/107-systems/.github/main/logo/107-systems.png" width="15%"></a>
:floppy_disk: `CyphalServoController12/CAN-firmware`
====================================================
<a href="https://opencyphal.org/"><img align="right" src="https://raw.githubusercontent.com/107-systems/.github/main/logo/opencyphal.svg" width="25%"></a>
[![General Formatting Checks](https://github.com/107-systems/CyphalServoController12-CAN-firmware/workflows/General%20Formatting%20Checks/badge.svg)](https://github.com/107-systems/CyphalServoController12-CAN-firmware/actions?workflow=General+Formatting+Checks)
[![Spell Check](https://github.com/107-systems/CyphalServoController12-CAN-firmware/workflows/Spell%20Check/badge.svg)](https://github.com/107-systems/CyphalServoController12-CAN-firmware/actions?workflow=Spell+Check)
[![Compile Examples](https://github.com/107-systems/CyphalServoController12-CAN-firmware/workflows/Compile/badge.svg)](https://github.com/107-systems/CyphalServoController12-CAN-firmware/actions?workflow=Compile)

Firmware for the 12-chanel PWM servo controller which is based on the [OpenCyphalServoController12](https://github.com/generationmake/OpenCyphalServoController12) board.

<p align="center">
  <a href="https://github.com/107-systems/l3xz"><img src="https://raw.githubusercontent.com/107-systems/.github/main/logo/l3xz-logo-memento-mori-github.png" width="40%"></a>
</p>

## How-to-build/upload
```bash
arduino-cli compile -b rp2040:rp2040:rpipico -v .
arduino-cli upload -b rp2040:rp2040:rpipico -v . -p /dev/ttyACM0
```
**or**
```bash
arduino-cli compile -b rp2040:rp2040:rpipico -v . --build-property compiler.cpp.extra_flags="-DCYPHAL_NODE_INFO_GIT_VERSION=0x$(git rev-parse --short=16 HEAD)"
```
Adding argument `--build-property compiler.cpp.extra_flags="-DCYPHAL_NODE_INFO_GIT_VERSION=0x$(git rev-parse --short=16 HEAD)"` allows to feed the Git hash of the current software version to [107-Arduino-Cyphal](https://github.com/107-systems/107-Arduino-Cyphal) stack from where it can be retrieved via i.e. [yakut](https://github.com/opencyphal/yakut).

### How-to-`yakut`
[Install](https://github.com/OpenCyphal/yakut) and configure `yakut`:
```bash
. setup_yakut.sh
```
Set servo pulse width via `yakut` (`cyphal.sub.pulse_width.id` = `4001`):
```bash
y pub --period=0.1 4001:uavcan.primitive.array.Natural16.1.0 '[1000, 2000]'
```
Set servo pulse with for all 12 channels via `yakut` (`cyphal.sub.pulse_width.id` = `4001`):
```bash
y pub --period=0.1 4001:uavcan.primitive.array.Natural16.1.0 '[1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000]'
...
y pub --period=0.1 4001:uavcan.primitive.array.Natural16.1.0 '[2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000]'
...
y pub --period=0.1 4001:uavcan.primitive.array.Natural16.1.0 '[1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]'
```
