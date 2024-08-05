
#ifndef canopen_h
#define canopen_h


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

// NMT ERROR CONTROL STATE in rxFrame.data[0]
#define NMT_ERROR_CONTROL_STATE_DISCONNECTED    0x01
#define NMT_ERROR_CONTROL_STATE_CONNECTING      0x02
#define NMT_ERROR_CONTROL_STATE_PREPARING       0x03
#define NMT_ERROR_CONTROL_STATE_PREPARED        0x04
#define NMT_ERROR_CONTROL_STATE_OPERATIONAL     0x05
#define NMT_ERROR_CONTROL_STATE_PRE_OPERATIONAL 0x7F


// Destination Address Mode bit mask
#define DAM_PDO_MASK 0x80

#endif  // canopen_h