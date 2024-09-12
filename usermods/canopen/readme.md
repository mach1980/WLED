# CANOpen Usermod

Usermod to allow WLED to receive configuration and events through a CANOpen interface.

CANopen is a high-level communication protocol and device profile specification that is based on the CAN (Controller Area Network) protocol. The protocol was developed for embedded networking applications, such as in-vehicle networks.

CANOpen® is a registered trademark of CAN in Automation (CiA). This code is in no way affiliated with CiA. No warranty is provided - use at your own risk.

## Installation

Add the compile-time option `-D USERMOD_CANOPEN` to your `platformio.ini` (or `platformio_override.ini`) or use `#define USERMOD_CANOPEN` in `my_config.h`.

## Settings
Settings can be controlled via both the usermod setting page and through CANOpen messages.

## Supported CANOpen commands

  - NMT Control - Reset Node


## Example


## Jitter fix

NeoEsp32RmtChannelN::

    static void tx_end_callback(rmt_channel_t channel, void *arg)
    {
        DEBUG_PRINTLN("rmt_tx_end_callback");
        //esp_intr_enable_source(5);
        esp_intr_enable_source(4);
    }

        void Initialize()
      {
        [...]
        rmt_register_tx_end_callback(tx_end_callback, &transmission_counter);
        [...]


void Update(bool maintainBufferConsistency)
    {

        if (ESP_OK == ESP_ERROR_CHECK_WITHOUT_ABORT(rmt_wait_tx_done(_channel.RmtChannelNumber, 10000 / portTICK_PERIOD_MS)))
        {
            esp_intr_disable_source(4);

## Version
20240814 - Initial release
