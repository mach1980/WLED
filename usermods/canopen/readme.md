# CANOpen Usermod

Usermod to allow WLED to receive configuration and events through a CANOpen interface.

CANopen is a high-level communication protocol and device profile specification that is based on the CAN (Controller Area Network) protocol. The protocol was developed for embedded networking applications, such as in-vehicle networks.

## Installation

Add the compile-time option `-D USERMOD_CANOPEN` to your `platformio.ini` (or `platformio_override.ini`) or use `#define USERMOD_CANOPEN` in `my_config.h`.

## Settings
Settings can be controlled via both the usermod setting page and through CANOpen messages.


## Example

## Version
20240814 - Initial release
