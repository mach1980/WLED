
#ifndef canopen_h
#define canopen_h

#include "driver/twai.h"

/**
 * COB-ID Function Codes
 * 
 * The four most significatn bits of the CAN address defines the type of CanOpen message.
 * The seven least significant bits of the CAN address are the node id.
 */ 
#define NMT_FUNCTION        0b00000000000
#define SYNCHRONIZATION     0b00010000000
#define EMERGENCY           0b00010000000
#define TIMESTAMP           0b00100000000
#define T_PDO1              0b00110000000
#define R_PDO1              0b01000000000
#define T_PDO2              0b01010000000
#define R_PDO2              0b01100000000
#define T_PDO3              0b01110000000
#define R_PDO3              0b10000000000
#define T_PDO4              0b10010000000
#define R_PDO4              0b10100000000
#define T_SDO               0b10110000000
#define R_SDO               0b11000000000
#define NMT_ERROR_CONTROL   0b11100000000

#define COB_FUNCTION_CODE_MASK 0b11110000000 // Mask to isolate bits 10-7
#define COB_NODE_ID_MASK       0b00001111111 // Mask to isolate bits 6-0
// Destination Address Mode bit mask
#define DAM_PDO_MASK 0x80

#define MASTER_CAN_ADDRESS 0x01
#define DEFAULT_NODE_ID 0x66 // 102
#define DEFAULT_CONTROLLER_NODE_ID 0x25 // 37
#define HEARTBEAT_COB_ID (NMT_ERROR_CONTROL | MASTER_CAN_ADDRESS)


/** --------- Start of NMT help functions ---------*/

#define NMT_HEARTBEAT_MS (unsigned long) 500
#define NMT_HEARTBEAT_TIMEOUT_MS (unsigned long) 2000


enum NMTCommand : uint8_t {
    START_NETWORK_NODE = 0x01,
    STOP_NETWORK_NODE = 0x02,
    GO_TO_PRE_OPERATIONAL = 0x80,
    RESET_NODE = 0x81,
    RESET_COMMUNICATION = 0x82,
    UNKNOWN = 0xFF
};

// NMT ERROR CONTROL STATE in rxFrame.data[0] 
enum class NMTState : uint8_t{
    DISCONNECTED = 0x01,
    CONNECTING = 0x02,
    PREPARING = 0x03,
    PREPARED = 0x04,
    OPERATIONAL = 0x05,
    PRE_OPERATIONAL = 0x7F
};

typedef struct {
    NMTState state;
    unsigned long lastSentHeartbeat;
    unsigned long lastReceivedHeartbeat;
} nmt_handle_t;


nmt_handle_t nmtCreate()
{
    return {
        .state = NMTState::CONNECTING,
        .lastSentHeartbeat = 0,
        .lastReceivedHeartbeat = 0
    };
}


/**
 * @return true if the frame is a NMT Error Control frame to given node.
 */
bool isNmtErrorControlFrame(const twai_message_t& m, uint8_t node_id)
{
    return m.identifier == (NMT_ERROR_CONTROL | static_cast<uint32_t>(node_id));
}

/**
 * @return CANOpen NMT Error Control frame
 */
twai_message_t buildNmtErrorControlFrame(NMTState state, uint8_t node_id)
{
    twai_message_t msg = {0};  // Initialize all fields to zero

    msg.identifier = NMT_FUNCTION | (static_cast<uint32_t>(node_id) & COB_NODE_ID_MASK);
    msg.data_length_code = 2;

    // Split the 16-bit EEC into two 8-bit values
    msg.data[0] = static_cast<uint8_t>(state);
    msg.data[1] = static_cast<uint8_t>(node_id);
    return msg;
}

/**
 * @return CANOpen Synchronization frame
 */
twai_message_t buildSyncronizationFrame()
{
    twai_message_t msg = {0};  // Initialize all fields to zero
    msg.identifier = SYNCHRONIZATION;
    msg.data_length_code = 0;
    return msg;
}

/**
 * Build CANOpen Emergency frame.
 *
 *       Data field
 *          2 byte 1 byte      0-5 byte(s)
 *       ┌────┬────┬────┬────┬────┬────┬────┬────┐
 *       │   EEC   │ ER │          MEF           │
 *       └────┴────┴────┴────┴────┴────┴────┴────┘
 *            ▲      ▲              ▲
 *            │      │              │
 *            │      │              └───────────── Manufacturer-specific Error Field
 *            │      └──────────────────────────── Error Register (0x1001)
 *            │
 *            └─────────────────────────────────── Emergency Error Code
 *
 * @return CANOpen Emergency frame
 */
twai_message_t buildEmergencyFrame(uint8_t node_id, uint16_t eec, uint8_t er)
{
    twai_message_t msg = {0};  // Initialize all fields to zero

    msg.identifier = EMERGENCY | (static_cast<uint32_t>(node_id) & COB_NODE_ID_MASK);
    msg.data_length_code = 3;

    // Split the 16-bit EEC into two 8-bit values
    msg.data[0] = static_cast<uint8_t>(eec & 0xFF);         // Lower byte of EEC
    msg.data[1] = static_cast<uint8_t>((eec >> 8) & 0xFF);  // Upper byte of EEC
    msg.data[2] = er;
    return msg;
}


/**
 * @return true if the frame is a NMT Command frame to given node.
 */
bool isNmtCommandFrame(const twai_message_t &m, uint8_t node_id)
{
    return m.identifier == (NMT_FUNCTION | (static_cast<uint32_t>(node_id) & COB_NODE_ID_MASK));
}

NMTCommand nmtCommandFromFrame(const twai_message_t& message)
{
    NMTCommand command = UNKNOWN;
    if (message.data_length_code >= 1) {
        command =  (NMTCommand) message.data[0];
    }
    return command;
}

/**
 * @return true if the frame is a NMT Error Control with heartbeat information from CANOpen master.
 */
bool isNmtErrorControlHeartbeat(const twai_message_t& m)
{
    return m.identifier == (NMT_ERROR_CONTROL | MASTER_CAN_ADDRESS);
}

/**
 * @return true if the CANOpen master has not sent any NMT Control Frames within NMT_HEARTBEAT_TIMEOUT_MS.
 */
bool isCANOpenMasterTimedOut(unsigned long millis, const nmt_handle_t& h) {
    return (millis > (h.lastReceivedHeartbeat + NMT_HEARTBEAT_TIMEOUT_MS));
}

/**
 * @return true if it is time to send a NMT Control Frame with status
 */
bool shouldSendHeartbeat(unsigned long millis, const nmt_handle_t& h, uint8_t node_id)
{
    return (h.state == NMTState::PRE_OPERATIONAL || h.state == NMTState::OPERATIONAL) && (millis > (NMT_HEARTBEAT_MS + h.lastSentHeartbeat));
}
/** --------- End of NMT help functions ---------*/

/** --------- Start of Basic Function help functions ---------
 *
 * Parsing of R_PDO_3 according to CiA 417 part 4.4.1.1 as Basic Function.
 *
 *       Basic Function Data field
 *       1 byte    1 byte    1 byte
 *            1 byte    1 byte      3 bytes
 *       ┌────┬────┬────┬────┬────┬────┬────┬────┐
 *       │ fn │sub │lift│flr │door│     data     │
 *       └────┴────┴────┴────┴────┴────┴────┴────┘
 *          ▲    ▲    ▲    ▲   ▲         ▲
 *          │    │    │    │   │         │
 *          │    │    │    │   │         └─────── Value
 *          │    │    │    │   │
 *          │    │    │    │   └───────────────── Bitmask for doors
 *          │    │    │    │
 *          │    │    │    └───────────────────── Floors
 *          │    │    │
 *          │    │    └────────────────────────── Lift
 *          │    │
 *          │    └─────────────────────────────── Sub-function
 *          │
 *          └──────────────────────────────────── Function
 *
 */


enum BasicFunctionFunction : uint8_t {
    LIFT_LIGHTS = 0x3F,
    HALL_LANTERN_INDICATION = 0x41
};


enum BasicFunctionLiftsLightsSubFunction : uint8_t {
    CAR_LIGHTS_ON = 0x01,
    CAR_LIGHTS_OFF = 0x02
};

/**
 *  @return true if the message is a Basic Function - Main Lights Acknowledgement message.
 */
bool isBasicFunctionMainLightsAck(const twai_message_t& message)
{
    return ((message.identifier & COB_FUNCTION_CODE_MASK) == R_PDO3) && \
            (message.data_length_code >= 2) && \
            (message.data[0] == LIFT_LIGHTS);
}

BasicFunctionLiftsLightsSubFunction getBasicFunctionMainLigthsAckSubFunction(const twai_message_t& message)
{
    return (BasicFunctionLiftsLightsSubFunction) message.data[1];
}


/** --------- End of Basic Function help functions ---------*/
#endif  // canopen_h