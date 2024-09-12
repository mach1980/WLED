#ifndef SDOHANDLER_H
#define SDOHANDLER_H

#include "wled.h"
#include "driver/twai.h"
#include "can.h"
#include "canopen.h"


enum class SdoState {
    DISCONNECTED,
    IDLE,
    SEGMENTED_UPLOAD,
    SEGMENTED_DOWNLOAD,
};

enum ClientCommandSpecifier : uint8_t {
    CSS_DOWNLOAD_SEGMENT = 0b00000000,
    CSS_INITIATE_DOWNLOAD = 0b00100000,
    CSS_INITIATE_UPLOAD = 0b01000000,
    CCS_UPLOAD_SEGMENT = 0b01100000,
    CCS_ABORT = 0b10000000,
    CCS_BLOCK_UPLOAD = 0b10100000,
    CCS_BLOCK_DOWNLOAD = 0b11000000,
    CCS_UNKNOWN = 0b11100000,
};

typedef struct {
    uint8_t cs = CSS_INITIATE_UPLOAD;
    uint16_t index;
    uint8_t sub;
} sdo_initiate_upload_request_t;


#define CS_OFFSET 0
#define INDEX_OFFSET 1
#define SUB_OFFSET 3
#define CCS_MASK 0b11100000



class SdoHandler
{
private:
    SdoState _sdoState = SdoState::IDLE;

    /**
     * @return true if the message is a SDO Request to this node
     */
    bool isSdoRequest(const twai_message_t& m, uint8_t nodeId)
    {
        // Check that it is a Request SDO to this node
        return m.identifier == (R_SDO | nodeId) && m.data_length_code >= 1;
    }

    ClientCommandSpecifier getCcsFromSdoRequest(const twai_message_t& m)
    {
        ClientCommandSpecifier ret = CCS_UNKNOWN;
        if (m.data_length_code > 0) {
            ret = static_cast<ClientCommandSpecifier>(m.data[0] & CCS_MASK);
        }
        return ret;
    }


    /**
     * CanOpen Request Initiate Upload Service Data Object
     *
     * This is the initial request from the client to read a specific Data Object from a Server.
     *
     *       Initiate Upload Request
     *       1 byte 2 bytes 1 byte    0-4 byte(s)
     *       ┌────┬────┬────┬────┬────┬────┬────┬────┐
     *       │ cs │  index  │sub │      reserved     │
     *       └────┴────┴────┴────┴────┴────┴────┴────┘
     *          ▲      ▲      ▲            ▲
     *          │      │      │            │
     *          │      │      │            └──────── Reserved for further use , always 0
     *          │      │      │
     *          │      │      └───────────────────── Sub-index in the Object Dictionary as u8.
     *          │      │
     *          │      └──────────────────────────── Index in the Object Dictionary as u16.
     *          │
     *          │   ┌────┬────┬────┬────┬────┬────┬────┬────┐
     *          └───│    ccs = 2   │            x           │ Bit register of Client Command
     *              └──7─┴──6─┴─5──┴──4─┴──3─┴──2─┴──1─┴──0─┘
     *                      ▲                   ▲
     *                      │                   │
     *                      │                   └─────────────────── Not used, always 0
     *                      │
     *                      └─────────────────────────────────────── Client Command Specifier
     *                                                               ccs = 2: Initiate upload
     *
     *
     * @return A sdo_initiate_upload_request_t. index and sub is null if not a valid message.
     * 
     * Copyright Drone Bone AB 2024
     *
     * @author: Jonas Estberger (jonas.estberger@gmail.com)
     */
    sdo_initiate_upload_request_t parseInitiateUploadRequest(const twai_message_t& m)
    {
        sdo_initiate_upload_request_t ret = {};
        if (m.data_length_code > 4 && (static_cast<ClientCommandSpecifier>(m.data[0] & CCS_MASK) == CSS_INITIATE_UPLOAD))
        {
            ret.index = (m.data[2] << 8) | m.data[1];   // Big-endian parsing
            ret.sub = m.data[3];
        }
        return ret;
    }




    /**
     * Client requested a WRITE to Server
     */
    SdoState onInitiateDownload(SdoState s, const twai_message_t& m)
    {
        SdoState newState = s;
        switch (s)
        {
            case SdoState::DISCONNECTED:
                break;
            case SdoState::IDLE:
                break;
            case SdoState::SEGMENTED_UPLOAD:
                break;
            case SdoState::SEGMENTED_DOWNLOAD:
                break;
            default:
                break;
        };
        return newState;
    }

    /**
     * Client requested a READ from Server
     */
    SdoState onInitiateUpload(SdoState s, const twai_message_t& m)
    {

        SdoState newState = s;
        switch (s)
        {
            case SdoState::DISCONNECTED:
                break;
            case SdoState::IDLE:
            {
                sdo_initiate_upload_request_t req = parseInitiateUploadRequest(m);
                DEBUG_PRINTF("SdoHandler got a client request to READ %u:%u\n", req.index, req.sub);
                break;
            }
            case SdoState::SEGMENTED_UPLOAD:
                break;
            case SdoState::SEGMENTED_DOWNLOAD:
                break;
            default:
                break;
        };
        return newState;
    }


public:

    void onSetup(uint8_t nodeId)
    {
        _sdoState = SdoState::IDLE;
    }


    void onTeardown()
    {
        _sdoState = SdoState::DISCONNECTED;
    }

    void onMessage(uint8_t nodeId, const twai_message_t& m)
    {
        if (isSdoRequest(m, nodeId))
        {
            SdoState newState = _sdoState;
            switch(getCcsFromSdoRequest(m))
            {
                case CSS_DOWNLOAD_SEGMENT:
                    break;
                case CSS_INITIATE_DOWNLOAD:
                    newState = onInitiateDownload(_sdoState, m);
                    break;
                case CSS_INITIATE_UPLOAD:
                    newState = onInitiateUpload(_sdoState, m);
                    break;
                case CCS_UPLOAD_SEGMENT:
                    break;
                case CCS_ABORT:
                    break;
                case CCS_BLOCK_UPLOAD:
                    break;
                case CCS_BLOCK_DOWNLOAD:
                    break;
                default:
                    break;
            }

            if (_sdoState != newState)
            {
                DEBUG_PRINTF("SdoHandler going from state %u to state %u\n", _sdoState, newState);
                _sdoState = newState;
            }

        }


        return;
    }
};




#endif // SDOHANDLER_H
