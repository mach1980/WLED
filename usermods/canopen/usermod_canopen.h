#pragma once


#ifndef CAN_TX
  #define CAN_TX    19
#endif

#ifndef CAN_RX
  #define CAN_RX    18
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
#define NODE_ID 0x66 // 102  This nodes ID

#define SENDER_NODE_ID 0x25 // 37
#define DAM_PDO_ID (SENDER_NODE_ID | DAM_PDO_MASK)

#define MASTER_CAN_ADDRESS 0x01
#define UNJO_CAN_ADDRESS  0x66

#define WLED_COB_ID (R_PDO4 | NODE_ID)
#define HEARTBEAT_COB_ID (NMT_ERROR_CONTROL | MASTER_CAN_ADDRESS)

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
  
  bool enabled = false;
  bool initDone = false;

  // These config variables have defaults set inside readFromConfig()
  int8_t nodeId;
  int8_t testPins[5];

  // string that are used multiple time (this will save some flash memory)
  static const char _name[];
  static const char _enabled[];

  CanFrame rxFrame;
  int heartbeatState = LOW;
  int statusState = LOW;
  unsigned long lastHeartbeat = millis();

  //Runtime variables.
  unsigned long lastRefresh = 0;
  unsigned int ssNodeId = NODE_ID;

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
      return  rxFrame.data[0] == DAM_PDO_ID && \
              rxFrame.data[1] == MPDO_COLOR_INDEX_LSB && \
              rxFrame.data[2] == MPDO_COLOR_INDEX_MSB && \
              rxFrame.data[3] == MPDO_COLOR_SUBINDEX;
  }

  boolean isFadeTimeMpdo()
  {
      return  rxFrame.data[0] == DAM_PDO_ID && \
              rxFrame.data[1] == MPDO_FADE_TIME_INDEX_LSB && \
              rxFrame.data[2] == MPDO_FADE_TIME_INDEX_MSB && \
              rxFrame.data[3] == MPDO_FADE_TIME_SUBINDEX;
  }

  boolean isEffectMpdo()
  {
      return  rxFrame.data[0] == DAM_PDO_ID && \
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
       if(rxFrame.identifier == WLED_COB_ID)
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
     * Creating an "u" object allows you to add custom key/value pairs to the Info section of the WLED web UI.
     * Below it is shown how this could be used for e.g. a light sensor
     */
    void addToJsonInfo(JsonObject& root)
    {
      // if "u" object does not exist yet wee need to create it
      JsonObject user = root["u"];
      if (user.isNull()) user = root.createNestedObject("u");

      //this code adds "u":{"ExampleUsermod":[20," lux"]} to the info object
      //int reading = 20;
      //JsonArray lightArr = user.createNestedArray(FPSTR(_name))); //name
      //lightArr.add(reading); //value
      //lightArr.add(F(" lux")); //unit

      // if you are implementing a sensor usermod, you may publish sensor data
      //JsonObject sensor = root[F("sensor")];
      //if (sensor.isNull()) sensor = root.createNestedObject(F("sensor"));
      //temp = sensor.createNestedArray(F("light"));
      //temp.add(reading);
      //temp.add(F("lux"));
    }


    /*
     * addToJsonState() can be used to add custom entries to the /json/state part of the JSON API (state object).
     * Values in the state object may be modified by connected clients
     */
    void addToJsonState(JsonObject& root)
    {
      if (!initDone || !enabled) return;  // prevent crash on boot applyPreset()

      JsonObject usermod = root[FPSTR(_name)];
      if (usermod.isNull()) usermod = root.createNestedObject(FPSTR(_name));

      //usermod["user0"] = userVar0;
    }


    /*
     * readFromJsonState() can be used to receive data clients send to the /json/state part of the JSON API (state object).
     * Values in the state object may be modified by connected clients
     */
    void readFromJsonState(JsonObject& root)
    {
      if (!initDone) return;  // prevent crash on boot applyPreset()

      JsonObject usermod = root[FPSTR(_name)];
      if (!usermod.isNull()) {
        // expect JSON usermod data in usermod name object: {"ExampleUsermod:{"user0":10}"}
        userVar0 = usermod["user0"] | userVar0; //if "user0" key exists in JSON, update, else keep old value
      }
      // you can as well check WLED state JSON keys
      //if (root["bri"] == 255) Serial.println(F("Don't burn down your garage!"));
    }

    /*
     * addToConfig() can be used to add custom persistent settings to the cfg.json file in the "um" (usermod) object.
     * It will be called by WLED when settings are actually saved (for example, LED settings are saved)
     * If you want to force saving the current state, use serializeConfig() in your loop().
     * 
     * CAUTION: serializeConfig() will initiate a filesystem write operation.
     * It might cause the LEDs to stutter and will cause flash wear if called too often.
     * Use it sparingly and always in the loop, never in network callbacks!
     * 
     * addToConfig() will make your settings editable through the Usermod Settings page automatically.
     *
     * Usermod Settings Overview:
     * - Numeric values are treated as floats in the browser.
     *   - If the numeric value entered into the browser contains a decimal point, it will be parsed as a C float
     *     before being returned to the Usermod.  The float data type has only 6-7 decimal digits of precision, and
     *     doubles are not supported, numbers will be rounded to the nearest float value when being parsed.
     *     The range accepted by the input field is +/- 1.175494351e-38 to +/- 3.402823466e+38.
     *   - If the numeric value entered into the browser doesn't contain a decimal point, it will be parsed as a
     *     C int32_t (range: -2147483648 to 2147483647) before being returned to the usermod.
     *     Overflows or underflows are truncated to the max/min value for an int32_t, and again truncated to the type
     *     used in the Usermod when reading the value from ArduinoJson.
     * - Pin values can be treated differently from an integer value by using the key name "pin"
     *   - "pin" can contain a single or array of integer values
     *   - On the Usermod Settings page there is simple checking for pin conflicts and warnings for special pins
     *     - Red color indicates a conflict.  Yellow color indicates a pin with a warning (e.g. an input-only pin)
     *   - Tip: use int8_t to store the pin value in the Usermod, so a -1 value (pin not set) can be used
     *
     * See usermod_v2_auto_save.h for an example that saves Flash space by reusing ArduinoJson key name strings
     * 
     * If you need a dedicated settings page with custom layout for your Usermod, that takes a lot more work.  
     * You will have to add the setting to the HTML, xml.cpp and set.cpp manually.
     * See the WLED Soundreactive fork (code and wiki) for reference.  https://github.com/atuline/WLED
     * 
     * I highly recommend checking out the basics of ArduinoJson serialization and deserialization in order to use custom settings!
     */
    void addToConfig(JsonObject& root)
    {
      JsonObject top = root.createNestedObject(FPSTR(_name));
      top[FPSTR(_enabled)] = enabled;

      top["Node ID"] = nodeId;

      JsonArray pinArray = top.createNestedArray("pin");
      pinArray.add(testPins[0]);
      pinArray.add(testPins[1]);
      pinArray.add(testPins[2]);
      pinArray.add(testPins[3]);
      pinArray.add(testPins[4]);
    }


    /*
     * readFromConfig() can be used to read back the custom settings you added with addToConfig().
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
      // default settings values could be set here (or below using the 3-argument getJsonValue()) instead of in the class definition or constructor
      // setting them inside readFromConfig() is slightly more robust, handling the rare but plausible use case of single value being missing after boot (e.g. if the cfg.json was manually edited and a value was removed)

      JsonObject top = root[FPSTR(_name)];

      bool configComplete = !top.isNull();

      configComplete &= getJsonValue(top["enabled"], enabled, true);
      
      // A 3-argument getJsonValue() assigns the 3rd argument as a default value if the Json value is missing
      configComplete &= getJsonValue(top["node_id"], nodeId, NODE_ID);  
      
      // "pin" fields have special handling in settings page (or some_pin as well)
      configComplete &= getJsonValue(top["pin_can_rx"][0], testPins[0], CAN_TX);
      configComplete &= getJsonValue(top["pin_can_tx"][1], testPins[1], CAN_RX);
      configComplete &= getJsonValue(top["pin_error_led"][1], testPins[2], LED_GPIO_ERROR);
      configComplete &= getJsonValue(top["pin_status_led"][1], testPins[3], LED_GPIO_STATUS);
      configComplete &= getJsonValue(top["pin_heartbeat_led"][1], testPins[4], LED_GPIO_HEARTBEAT);

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
const char CanopenUsermod::_enabled[] PROGMEM = "enabled";
