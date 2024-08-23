#pragma once


#ifndef CAN_RX
  #define CAN_RX    18
#endif

#ifndef CAN_TX
  #define CAN_TX    19
#endif

#ifndef LED_GPIO_ERROR
  #define LED_GPIO_ERROR      4
#endif

#ifndef LED_GPIO_STATUS
  #define LED_GPIO_STATUS     15
#endif

#ifndef LED_GPIO_HEARTBEAT
  #define LED_GPIO_HEARTBEAT  2
#endif


/**
 * START --- CANOPEN DEFINES
 */
#define DEFAULT_NODE_ID 0x66 // 102  This nodes ID
#define DEFAULT_CONTROLLER_NODE_ID 0x25 // 37

// Color is @ 0x2100:00
#define MPDO_COLOR_INDEX_MSB 0x2F
#define MPDO_COLOR_INDEX_LSB 0x00
#define MPDO_COLOR_SUBINDEX 0x00

// Fade time is at 0x2101:00
#define MPDO_FADE_TIME_INDEX_MSB 0x2F
#define MPDO_FADE_TIME_INDEX_LSB 0x01
#define MPDO_FADE_TIME_SUBINDEX 0x00

// Type of effect is at 0x2110:00
#define MPDO_EFFECT_INDEX_MSB 0x2F
#define MPDO_EFFECT_INDEX_LSB 0x10
#define MPDO_EFFECT_SUBINDEX 0x00

#define MDPO_NO_OF_LEDS_INDEX_MSB 0x2F
#define MDPO_NO_OF_LEDS_INDEX_LSB 0x10
#define MPDO_NO_OF_LEDS_SUBINDEX 0x01

#define HEARTBEAT_TIMEOUT 2000
#define CAN_READ_TIMEOUT 0

/**
 * END --- CANOPEN DEFINES
 */


#include "driver/gpio.h"
#include "wled.h"
#include "canopen.h"
#include <ESP32-TWAI-CAN.hpp>

class CanopenUsermod : public Usermod
{

private:
  
  // string that are used multiple time (this will save some flash memory)
  static const char _name[];
  static const char _enabled[];
  static const char _node_id[];
  static const char _controller_id[];

  // These config variables have defaults set inside readFromConfig()
  bool enabled = true;
  uint8_t nodeId;
  uint8_t controllerNodeId;

  //Runtime variables.
  bool initDone = false;
  unsigned long lastRefresh = 0;
  CanFrame rxFrame;
  int heartbeatState = LOW;
  int statusState = LOW;
  unsigned long lastHeartbeat = millis();

  /**
   * Configure a GPIO pin as output.
   * 
   * @param pin GPIO pin
   */
  void setupOutputLedPin(int pin)
  {
      gpio_reset_pin((gpio_num_t) pin);
      PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[pin], PIN_FUNC_GPIO);
      gpio_set_direction((gpio_num_t) pin, (gpio_mode_t)GPIO_MODE_DEF_OUTPUT);
      pinMode(pin, OUTPUT);
      gpio_output_set( 1 << pin, 0 << pin, 1 << pin ,0);
      gpio_matrix_out(pin, SIG_GPIO_OUT_IDX, false, false);
      gpio_pad_select_gpio(pin);
      digitalWrite(pin, HIGH);
  }

  boolean isColorMpdo()
  {
      return  rxFrame.data[0] == (controllerNodeId | DAM_PDO_MASK) && \
              rxFrame.data[1] == MPDO_COLOR_INDEX_LSB && \
              rxFrame.data[2] == MPDO_COLOR_INDEX_MSB && \
              rxFrame.data[3] == MPDO_COLOR_SUBINDEX;
  }

  boolean isFadeTimeMpdo()
  {
      return  rxFrame.data[0] == (controllerNodeId | DAM_PDO_MASK) && \
              rxFrame.data[1] == MPDO_FADE_TIME_INDEX_LSB && \
              rxFrame.data[2] == MPDO_FADE_TIME_INDEX_MSB && \
              rxFrame.data[3] == MPDO_FADE_TIME_SUBINDEX;
  }

  boolean isEffectMpdo()
  {
      return  rxFrame.data[0] == (controllerNodeId | DAM_PDO_MASK) && \
              rxFrame.data[1] == MPDO_EFFECT_INDEX_LSB && \
              rxFrame.data[2] == MPDO_EFFECT_INDEX_MSB && \
              rxFrame.data[3] == MPDO_EFFECT_SUBINDEX;
  }

public:
  //Functions called by WLED

  /*
     * setup() is called once at boot. WiFi is not yet connected at this point.
     * You can use it to initialize variables, sensors or similar.
     */
  void setup()
  {
    setupOutputLedPin(LED_GPIO_ERROR);
    setupOutputLedPin(LED_GPIO_STATUS);
    setupOutputLedPin(LED_GPIO_HEARTBEAT);

    delay(1000);
    digitalWrite(LED_GPIO_ERROR, LOW);
    digitalWrite(LED_GPIO_STATUS, LOW);
    digitalWrite(LED_GPIO_HEARTBEAT, LOW);

    // Set pins
    ESP32Can.setPins(CAN_TX, CAN_RX);
    // You can set custom size for the queues - those are default
    ESP32Can.setRxQueueSize(5);
    ESP32Can.setTxQueueSize(5);
    // .setSpeed() and .begin() functions require to use TwaiSpeed enum,
    // but you can easily convert it from numerical value using .convertSpeed()
    ESP32Can.setSpeed(ESP32Can.convertSpeed(250));

    ESP32Can.begin();
    
    initDone = true;
  }

  /*
     * loop() is called continuously. Here you can check for events, read sensors, etc.
     */
  void loop()
  {

  if (millis() < lastHeartbeat + HEARTBEAT_TIMEOUT)
    {
        if (statusState == LOW)
        {
            // Going from state from heartbeat timeout to running
            statusState = HIGH;
            digitalWrite(LED_GPIO_ERROR, LOW);
            digitalWrite(LED_GPIO_STATUS, HIGH);
        }
    }
    else
    {
        if (statusState == HIGH)
        {
            // Going from state running to heartbeat timeout
            statusState = LOW;
            heartbeatState = LOW;
            digitalWrite(LED_GPIO_ERROR, HIGH);
            digitalWrite(LED_GPIO_STATUS, LOW);
            digitalWrite(LED_GPIO_HEARTBEAT, LOW);
        }
    }

    if(ESP32Can.readFrame(rxFrame, CAN_READ_TIMEOUT))
    {
       if(rxFrame.identifier == (R_PDO4 | nodeId))
       {
            if (isColorMpdo())
            {
                // A request for setting current RGB(W) primary color
                byte white = rxFrame.data[7];
                byte red =rxFrame.data[6];
                byte green = rxFrame.data[5];
                byte blue = rxFrame.data[4];

                col[0] = red;
                col[1] = green;
                col[2] = blue;
                col[3] = white;

                colorUpdated(CALL_MODE_BUTTON);
            }
            else if (isFadeTimeMpdo())
            {
                // A request set fade time
                byte lsb = rxFrame.data[4];
                byte msb = rxFrame.data[5];
                transitionDelay = ((uint16_t)(msb) << 8) | (uint16_t)lsb;
                
            }
            else if (isEffectMpdo())
            {
                // A request to set effect
                effectCurrent = rxFrame.data[4];
                stateChanged = true;
                
                colorUpdated(CALL_MODE_BUTTON);
            }
       }
       else if (rxFrame.identifier == HEARTBEAT_COB_ID)
       {
            if (rxFrame.data[0] == NMT_ERROR_CONTROL_STATE_OPERATIONAL)
            {
                if (heartbeatState == LOW)
                {
                    heartbeatState = HIGH;
                }
                else
                {
                    heartbeatState = LOW;
                }
                digitalWrite(LED_GPIO_HEARTBEAT, heartbeatState);
                lastHeartbeat = millis();
            }
       }
    }
  }


    /*
     * addToJsonInfo() can be used to add custom entries to the /json/info part of the JSON API.
     */
    void addToJsonInfo(JsonObject& root)
    {
      // Check if the "u" object exists; if not, create it
      JsonObject user = root["u"];
      if (user.isNull()) {
          user = root.createNestedObject("u");
      }

      // Create or access the "CANOpen" object within "u"
      JsonObject canOpen = user[PSTR(CanopenUsermod::_name)];
      if (canOpen.isNull()) {
          canOpen = user.createNestedObject(FPSTR(CanopenUsermod::_name));
      }

      // Add the "NodeId" entry to the "CANOpen" object
      canOpen[FPSTR(CanopenUsermod::_node_id)] = nodeId;
      // Add the "Controller NodeId" entry to the "CANOpen" object
      canOpen[FPSTR(CanopenUsermod::_controller_id)] = controllerNodeId;
    }


    /*
     * addToConfig() can be used to add custom persistent settings to the cfg.json file in the "um" (usermod) object.
     * It will be called by WLED when settings are actually saved (for example, LED settings are saved)
     * If you want to force saving the current state, use serializeConfig() in your loop().
     */
    void addToConfig(JsonObject& root)
    {
      JsonObject top = root.createNestedObject(FPSTR(_name));
      top[FPSTR(_enabled)] = enabled;
      top[FPSTR(_node_id)] = nodeId;
      top[FPSTR(_controller_id)] = controllerNodeId;
    }


    /*
     * This is called by WLED when settings are loaded (currently this only happens immediately after boot, or after saving on the Usermod Settings page)
     * 
     * readFromConfig() is called BEFORE setup(). This means you can use your persistent values in setup() (e.g. pin assignments, buffer sizes),
     * but also that if you want to write persistent values to a dynamic buffer, you'd need to allocate it here instead of in setup.
     * If you don't know what that is, don't fret. It most likely doesn't affect your use case :)
     * 
     * Return true in case the config values returned from Usermod Settings were complete, or false if you'd like WLED to save your defaults to disk (so any missing values are editable in Usermod Settings)
     * 
     * getJsonValue() returns false if the value is missing, or copies the value into the variable provided and returns true if the value is present
     * The configComplete variable is true only if the "exampleUsermod" object and all values are present.  If any values are missing, WLED will know to call addToConfig() to save them
     * 
     * This function is guaranteed to be called on boot, but could also be called every time settings are updated
     */
    bool readFromConfig(JsonObject& root)
    {
      JsonObject top = root[FPSTR(_name)];

      bool configComplete = !top.isNull();
      configComplete &= getJsonValue(top[FPSTR(_enabled)], enabled, true);
      configComplete &= getJsonValue(top[FPSTR(_node_id)], nodeId, DEFAULT_NODE_ID);
      configComplete &= getJsonValue(top[FPSTR(_controller_id)], controllerNodeId, DEFAULT_CONTROLLER_NODE_ID);

      return configComplete;
    }


  /*
     * getId() allows you to optionally give your V2 usermod an unique ID (please define it in const.h!).
     * This could be used in the future for the system to determine whether your usermod is installed.
     */
  uint16_t getId()
  {
    return USERMOD_ID_CANOPEN;
  }
};

// add more strings here to reduce flash memory usage
const char CanopenUsermod::_name[]    PROGMEM = "CANOpen";
const char CanopenUsermod::_enabled[] PROGMEM = "Enabled";
const char CanopenUsermod::_node_id[] PROGMEM = "NodeId";
const char CanopenUsermod::_controller_id[] PROGMEM = "NodeId of Controller";
