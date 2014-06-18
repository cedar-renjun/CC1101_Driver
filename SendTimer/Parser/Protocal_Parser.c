#include <stdint.h>
#include <string.h>
#include "MF_CRC16.h"

#define PROTOCAL_HEADER        0x5A

#define S_IDLE     0
#define S_HEADER   1
#define S_LEN      2
#define S_VERSION  3
#define S_PAYLOAD  4

#define PROTOCAL_FRAME_MAX_CNT   5
#define PROTOCAL_FRAME_MAX_SIZE  200
#define PROTOCAL_VERSION         1

typedef struct
{
    uint8_t   hdr_flag;         
    uint8_t   length;           
    uint8_t   version;          
    uint8_t   crc8;             
}Header;

typedef struct
{
    uint8_t target_addr;
    uint8_t sender_addr;
    uint8_t data[PROTOCAL_FRAME_MAX_CNT];
}Body;

typedef struct
{
    uint16_t  crc16;
}Checksum;

static uint8_t* ScanHeader(uint8_t pData)
{
    static uint8_t buffer[4];
    static uint8_t state = S_IDLE;

    switch(state)
    {
        case S_IDLE:                           // Wait Protocal Header
        {
            if(pData == PROTOCAL_HEADER)
            {
                state     = S_HEADER;
                buffer[0] = pData;
            }

            return (void *)0;
        }

        case S_HEADER:                        // Find Header Flag, Then get length info
        {
            state     = S_LEN;
            buffer[1] = pData;
            return (void *)0;
        }

        case S_LEN:                          // Get Version info
        {
            state     = S_VERSION;
            buffer[2] = pData;
            return (void *)0;
        }

        case S_VERSION:                     // Check Header via CRC8
        {
            state     = S_IDLE;
            buffer[3] = pData;
            if(Verify_CRC8_Check_Sum(buffer, 4))
            {
                return buffer;             // Find it
            }
            else
            {
                return (void *)0;
            }
        }

        default:
        {
            return (void *)0;
        }
    }
}

uint8_t FrameUnpack(uint8_t token, uint8_t* pBuffer)
{
    uint8_t* pHeader;
    static uint8_t PayLoadLen = 0;
    static uint8_t cnt = 0;
    static uint8_t state = S_IDLE;
    static uint8_t __RecvFrameBuffer[PROTOCAL_FRAME_MAX_SIZE];
    
#ifdef DEBUG
    assert(pBuffer != (void*)0 );
#endif

    switch(state)
    {
        case S_IDLE:                                    // Wait Header
        {
            pHeader = ScanHeader(token);
            if((void*)0 != pHeader)
            {
                cnt = 0;
                // Find Header Success!
                __RecvFrameBuffer[cnt++] = pHeader[0];
                __RecvFrameBuffer[cnt++] = pHeader[1];
                __RecvFrameBuffer[cnt++] = pHeader[2];
                __RecvFrameBuffer[cnt++] = pHeader[3];

                PayLoadLen = pHeader[1];

                state = S_PAYLOAD;
            }
            return 0;
        }

        case S_PAYLOAD:                               // Receive Payload data
        {
            __RecvFrameBuffer[cnt++] = token;
            if(cnt == sizeof(Header) + PayLoadLen + sizeof(Checksum))
            {
                state = S_IDLE;
                if(Verify_CRC16_Check_Sum(__RecvFrameBuffer, sizeof(Header) + PayLoadLen + sizeof(Checksum)))
                {
                    // CRC Check OK                
                    memcpy(pBuffer, &__RecvFrameBuffer[sizeof(Header)], PayLoadLen);
                    return PayLoadLen;
                }
                else
                {
                    return 0;
                }
            }
            return 0;
        }

        default:
        {
            return 0;
        }
    }
}

uint8_t FramePack(uint8_t* pDataIn, uint8_t len, uint8_t* pDataOut)
{
    uint32_t retval = 0;
    
#ifdef DEBUG
    assert(pDataIn != (void*)0 && pDataOut != (void*)0 );
#endif

    pDataOut[0] = PROTOCAL_HEADER;
    pDataOut[1] = len;
    pDataOut[2] = PROTOCAL_VERSION;
    Append_CRC8_Check_Sum(pDataOut, sizeof(Header));
    memcpy(&pDataOut[sizeof(Header)], pDataIn, len);
    Append_CRC16_Check_Sum(pDataOut, len + sizeof(Header) + sizeof(Checksum));

    
    return sizeof(Header) + len +  sizeof(Checksum);
}
