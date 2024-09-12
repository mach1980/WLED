#pragma once

#ifndef LED_GPIO_ERROR
  #define LED_GPIO_ERROR      4
#endif

#ifndef LED_GPIO_STATUS
  #define LED_GPIO_STATUS     15
#endif

#ifndef LED_GPIO_HEARTBEAT
  #define LED_GPIO_HEARTBEAT  2
#endif


#include "wled.h"
#include "driver/gpio.h"
#include "driver/twai.h"
#include "can.h"
#include "canopen.h"
#include "SdoHandler.h"
#include "PdoHandler.h"

typedef struct {
  uint8_t errorLed;
  uint8_t statusLed;
  uint8_t heartbeatLed;

  bool isMainLightsOn;
} canopen_ui_state_t;

class CanopenUsermod : public Usermod
{

private:
  
  // string that are used multiple time (this will save some flash memory)
  static const char _NAME_STR[];
  static const char _ENABLED_STR[];
  static const char _NODE_ID_STR[];
  static const char _CONTROLLER_ID_STR[];

  // These config variables have defaults set inside readFromConfig()
  bool _enabled = true;
  uint8_t _nodeId;
  uint8_t _controllerNodeId;

  //Runtime variables.
  bool initDone = false;

  bool transmitting = false;

  unsigned long lastRefresh = 0;
  
  nmt_handle_t nmt = {
        .state = NMTState::PRE_OPERATIONAL,
        .lastSentHeartbeat = 0,
        .lastReceivedHeartbeat = 0
    };

  canopen_ui_state_t uiState = {LOW, LOW, LOW, false};

  SdoHandler _sdoHandler = {};
  PdoHandler _pdoHandler = {};

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
  }

  /**
   * Check if there are messages waiting for transmission in the TWAI driver.
   *
   * This function retrieves the current status information of the TWAI driver using
   * `twai_get_status_info()` and checks if there are any messages queued for transmission.
   * It returns `true` if there are messages waiting for transmission, and `false` otherwise.
   *
   * @param[in,out]  s  Pointer to a `twai_status_info_t` structure that will be filled 
   *                    with the current status information of the TWAI driver.
   *
   * @return
   *      - `true` if there are messages queued for transmission (`msgs_to_tx > 0`).
   *      - `false` if there are no messages queued for transmission or if `twai_get_status_info()` fails.
   */
  bool hasWaitingTx(twai_status_info_t *s) {
    return (ESP_OK == twai_get_status_info(s)) && (s->msgs_to_tx > 0);
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

    digitalWrite(LED_GPIO_ERROR, HIGH);
    digitalWrite(LED_GPIO_STATUS, HIGH);
    digitalWrite(LED_GPIO_HEARTBEAT, HIGH);

    delay(1000);

    DEBUG_PRINTF("Starting CANOpen node with node id %u\n", _nodeId);
    begin(TWAI_MODE_NORMAL);


    /**
    twai_message_t frame = buildSyncronizationFrame();
    transmitting = true;
    esp_err_t e = twai_transmit(&frame, pdMS_TO_TICKS(1000));
    if (e != ESP_OK)
    {
      DEBUG_PRINTF("Failed to enqueue Synchronization frame! Error code %u \n", e);
    }
     */

    digitalWrite(LED_GPIO_ERROR, LOW);
    digitalWrite(LED_GPIO_STATUS, LOW);
    digitalWrite(LED_GPIO_HEARTBEAT, LOW);

    _sdoHandler.onSetup(_nodeId);
    _pdoHandler.onSetup(_nodeId, _controllerNodeId);

  }

  /*
     * loop() is called continuously. Here you can check for events, read sensors, etc.
     */
  void loop()
  {

    unsigned long currentTime = millis();

    twai_message_t rxFrame;
    twai_status_info_t status = {};

    // The loop will continue running if:
    //   - There are no messages waiting for transmission (msgs_to_tx == 0).
    //   - A message is successfully received from the TWAI driver (twai_receive returns ESP_OK).
    while(!hasWaitingTx(&status) && (twai_receive(&rxFrame, 0) == ESP_OK))
    {
       if(rxFrame.identifier == (R_PDO4 | _nodeId))
       {

       }
       else if (isNmtErrorControlHeartbeat(rxFrame))
       {
          // Recieved a heartbeat from the CANOpen master
          // DEBUG_PRINTLN("Recieved heartbeat");
          nmt.lastReceivedHeartbeat = currentTime;
       }
       else if (isNmtCommandFrame(rxFrame, _nodeId))
       {
          // Recieved a NMT Command from the CANOpen master
          NMTCommand command = nmtCommandFromFrame(rxFrame);
          switch (command)
          {
          case START_NETWORK_NODE:
              DEBUG_PRINTLN("START_NETWORK_NODE");
              nmt.state = NMTState::OPERATIONAL;
              // end();
              // begin(TWAI_MODE_NORMAL);
              break;
          case STOP_NETWORK_NODE:
              DEBUG_PRINTLN("STOP_NETWORK_NODE");
              nmt.state = NMTState::DISCONNECTED;
              // end();
              // begin(TWAI_MODE_LISTEN_ONLY);
              break;
          case GO_TO_PRE_OPERATIONAL:
              DEBUG_PRINTLN("GO_TO_PRE_OPERATIONAL");
              nmt.state = NMTState::PRE_OPERATIONAL;
              // end();
              // begin(TWAI_MODE_NORMAL);
              break;
          case RESET_COMMUNICATION:
              DEBUG_PRINTLN("Received CANOpen NMT command to reset communication");
              nmt.state = NMTState::DISCONNECTED;
               // end();
               // begin(TWAI_MODE_LISTEN_ONLY);
              break;
          case RESET_NODE:
              DEBUG_PRINTLN("Received CANOpen NMT command to reset node. Restarting...");
              // ESP.restart();
              break;
          case UNKNOWN:
              // Fallthrough
          default:
              break;
          }
       }
       else if (isBasicFunctionMainLightsAck(rxFrame))
       {
          if (CAR_LIGHTS_ON == getBasicFunctionMainLigthsAckSubFunction(rxFrame))
          {
            if (uiState.isMainLightsOn == false)
            {
              DEBUG_PRINTLN("Turn On Lights");
              uiState.isMainLightsOn = true;
            }
          } else {
            if (uiState.isMainLightsOn == true)
            {
              DEBUG_PRINTLN("Turn Off Lights");
              uiState.isMainLightsOn = false;
            }
          }
       }
    }

    if (shouldSendHeartbeat(currentTime, nmt, _nodeId) && _enabled && (transmitting == false))
    {
      DEBUG_PRINTF("Sending NMT Error Control frame with state %u \n", nmt.state);
      twai_message_t frame = buildNmtErrorControlFrame(nmt.state, _nodeId);
      transmitting = true;
      esp_err_t e = twai_transmit(&frame, pdMS_TO_TICKS(1000));
      if (e != ESP_OK)
      {
        DEBUG_PRINTF("Failed to send NMT Error Control frame! Error code %u \n", e);
      }
      nmt.lastSentHeartbeat = currentTime;
    }    

    // Map business states to UI
    uiState.statusLed = (nmt.state == NMTState::PRE_OPERATIONAL) || (nmt.state == NMTState::OPERATIONAL);
    if (currentTime <= (nmt.lastReceivedHeartbeat + 100))
    {
      uiState.heartbeatLed = HIGH;
    }
    else
    {
      uiState.heartbeatLed = LOW;
    }
    
    // Update UI
    digitalWrite(LED_GPIO_ERROR, uiState.errorLed);
    digitalWrite(LED_GPIO_STATUS,uiState.statusLed);
    digitalWrite(LED_GPIO_HEARTBEAT, uiState.heartbeatLed);

    
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
      JsonObject canOpen = user[PSTR(_NAME_STR
  )];
      if (canOpen.isNull()) {
          canOpen = user.createNestedObject(FPSTR(_NAME_STR));
      }

      // Add the "NodeId" entry to the "CANOpen" object
      canOpen[FPSTR(_NODE_ID_STR)] = _nodeId;
      // Add the "Controller NodeId" entry to the "CANOpen" object
      canOpen[FPSTR(_CONTROLLER_ID_STR)] = _controllerNodeId;
    }


    /*
     * addToConfig() can be used to add custom persistent settings to the cfg.json file in the "um" (usermod) object.
     * It will be called by WLED when settings are actually saved (for example, LED settings are saved)
     * If you want to force saving the current state, use serializeConfig() in your loop().
     */
    void addToConfig(JsonObject& root)
    {
      JsonObject top = root.createNestedObject(FPSTR(_NAME_STR));
      top[FPSTR(_ENABLED_STR)] = _enabled;
      top[FPSTR(_NODE_ID_STR)] = _nodeId;
      top[FPSTR(_CONTROLLER_ID_STR)] = _controllerNodeId;
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
      JsonObject top = root[FPSTR(_NAME_STR)];

      bool configComplete = !top.isNull();
      configComplete &= getJsonValue(top[FPSTR(_ENABLED_STR)], _enabled, true);
      configComplete &= getJsonValue(top[FPSTR(_NODE_ID_STR)], _nodeId, DEFAULT_NODE_ID);
      configComplete &= getJsonValue(top[FPSTR(_CONTROLLER_ID_STR)], _controllerNodeId, DEFAULT_CONTROLLER_NODE_ID);
     
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
const char CanopenUsermod::_NAME_STR[]    PROGMEM = "CANOpen";
const char CanopenUsermod::_ENABLED_STR[] PROGMEM = "Enabled";
const char CanopenUsermod::_NODE_ID_STR[] PROGMEM = "NodeId";
const char CanopenUsermod::_CONTROLLER_ID_STR[] PROGMEM = "NodeId of Controller";
