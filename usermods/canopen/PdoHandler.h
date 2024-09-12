#ifndef PDOHANDLER_H
#define PDOHANDLER_H

#include "wled.h"
#include "driver/twai.h"
#include "can.h"
#include "canopen.h"


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

class PdoHandler
{
private:

    uint8_t _nodeId;
    uint8_t _controllerNodeId;

    /**
     * @return true if the message is a PDO Request to this node
     */
    bool isPdoRequest(const twai_message_t& m, uint8_t nodeId)
    {
        return m.identifier == (R_PDO4 | nodeId);
    }

    bool isColorMpdo(const twai_message_t& m)
    {
        return  m.data[0] == (_controllerNodeId | DAM_PDO_MASK) && \
                m.data[1] == MPDO_COLOR_INDEX_LSB && \
                m.data[2] == MPDO_COLOR_INDEX_MSB && \
                m.data[3] == MPDO_COLOR_SUBINDEX;
    }

    bool isFadeTimeMpdo(const twai_message_t& m)
    {
        return  m.data[0] == (_controllerNodeId | DAM_PDO_MASK) && \
                m.data[1] == MPDO_FADE_TIME_INDEX_LSB && \
                m.data[2] == MPDO_FADE_TIME_INDEX_MSB && \
                m.data[3] == MPDO_FADE_TIME_SUBINDEX;
    }

    bool isEffectMpdo(const twai_message_t& m)
    {
        return  m.data[0] == (_controllerNodeId | DAM_PDO_MASK) && \
                m.data[1] == MPDO_EFFECT_INDEX_LSB && \
                m.data[2] == MPDO_EFFECT_INDEX_MSB && \
                m.data[3] == MPDO_EFFECT_SUBINDEX;
    }


public:

    void onSetup(uint8_t nodeId, uint8_t controllerNodeId)
    {
        _nodeId = nodeId;
        _controllerNodeId = controllerNodeId;
    }


    void onTeardown()
    {
    }

    void onMessage(uint8_t nodeId, const twai_message_t& m)
    {
        if (isPdoRequest(m, nodeId))
        {
            if (isColorMpdo(m))
            {
                // A request for setting current RGB(W) primary color
                byte white = m.data[7];
                byte red =m.data[6];
                byte green = m.data[5];
                byte blue = m.data[4];

                col[0] = red;
                col[1] = green;
                col[2] = blue;
                col[3] = white;

                colorUpdated(CALL_MODE_BUTTON);
            }
            else if (isFadeTimeMpdo(m))
            {
                // A request set fade time
                byte lsb = m.data[4];
                byte msb = m.data[5];
                transitionDelay = ((uint16_t)(msb) << 8) | (uint16_t)lsb;
                
            }
            else if (isEffectMpdo(m))
            {
                // A request to set effect
                effectCurrent = m.data[4];
                stateChanged = true;
                
                colorUpdated(CALL_MODE_BUTTON);
            }
        }
        return;
    }
};




#endif // PDOHANDLER_H
