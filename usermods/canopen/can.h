#pragma once

#include "wled.h"
#include "driver/gpio.h"
#include "driver/twai.h"


#ifndef CAN_RX
  #define CAN_RX    18
#endif

#ifndef CAN_TX
  #define CAN_TX    19
#endif

#ifndef CAN_RX_BUFFER_SIZE
  #define CAN_RX_BUFFER_SIZE 3
#endif

#ifndef CAN_TX_BUFFER_SIZE
  #define CAN_TX_BUFFER_SIZE 3
#endif

#define CAN_READ_TIMEOUT 0

#define TWAI_FILTER_CONFIG() {.acceptance_code = 0, .acceptance_mask = 0xFFFFFFFF, .single_filter = true}

static bool isTwaiDriverInstalled = false;

bool end() {
    bool ret = false;
    if(isTwaiDriverInstalled) {
        //Stop the TWAI driver
        if (twai_stop() == ESP_OK) {
            DEBUG_PRINTLN("Driver stopped\n");
            ret = true;
        } else {
            DEBUG_PRINTLN("Failed to stop driver\n");
        }

        //Uninstall the TWAI driver
        if (twai_driver_uninstall() == ESP_OK) {
            DEBUG_PRINTLN("Driver uninstalled\n");
            ret &= true;
        } else {
         DEBUG_PRINTLN("Failed to uninstall driver\n");
            ret &= false;
        }
       isTwaiDriverInstalled = !ret;
    } else ret = true;
    return ret;
}

boolean begin(twai_mode_t mode) {
    bool ret = false;
    if (end()) {
        isTwaiDriverInstalled = true;
        twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
        twai_general_config_t g_config = {
            .mode = mode,
            .tx_io = (gpio_num_t)CAN_TX,
            .rx_io = (gpio_num_t)CAN_RX,
            .clkout_io = TWAI_IO_UNUSED,
            .bus_off_io = TWAI_IO_UNUSED,
            .tx_queue_len = CAN_TX_BUFFER_SIZE,
            .rx_queue_len = CAN_RX_BUFFER_SIZE,
            .alerts_enabled = TWAI_ALERT_NONE,
            .clkout_divider = 0,
            .intr_flags = ESP_INTR_FLAG_LEVEL1
        };
        twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();

        // Install TWAI driver
        if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
            DEBUG_PRINTLN("TWAI Driver installed in mode " + String(mode));
        } else {
            DEBUG_PRINTLN("Failed to install TWAI driver");
        }

        // Start TWAI driver
        if (twai_start() == ESP_OK) {
            DEBUG_PRINTLN("TWAI Driver started");
            ret = true;
        } else {
            DEBUG_PRINTLN("Failed to start TWAI driver");
        }

        if (!ret) end();
    }

    return ret;
}

inline bool IRAM_ATTR readFrame(twai_message_t* frame, uint32_t timeout = 1000)
{
    bool ret = false;
    if((frame) && twai_receive(frame, pdMS_TO_TICKS(timeout)) == ESP_OK)
    {
        ret = true;
    }
    return ret;
}

inline bool IRAM_ATTR sendFrame(twai_message_t* frame, uint32_t timeout = 1000)
{
    bool ret = false;
    if ((frame) && twai_transmit(frame, pdMS_TO_TICKS(timeout)) == ESP_OK)
    {
    ret = true;
    }
    return ret;
}