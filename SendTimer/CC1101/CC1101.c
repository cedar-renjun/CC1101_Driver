/**************************************************************************************************
 * \file            CC1101.c
 * \description     Provide Some CC1101 Basic APIs
 * \version         V1.0
 * \date            2014-06-10
 * \author          cedar
 * 
 **************************************************************************************************/
 
#include "CC1101.h"

/*******************************************************************************/
#define     WRITE_BURST         0x40             //!< write in burst mode
#define     READ_SINGLE         0x80             //!< read in single mode
#define     READ_BURST          0xC0             //!< read in burst mode
#define     BYTES_IN_RXFIFO     0x7F             //!< volum in RX FIFO
#define     CRC_OK              0x80             //!< CRC-OK bit
#define     RSSI                0x00             //!< RSSI bit
#define     LQI                 0x01             //!< LQI bit
#define     DUMP_DATA           0x00             //!< Dump Data

//! register initialize table struct
typedef struct
{
    uint8_t addr;
    uint8_t value;
}RegType;

//! CC1101 Address Configure
#if CC1101_ADDR_EN == 0
#undef  CC1101_DRV_ADDR
#define CC1101_DRV_ADDR   0x00
#define CC1101_ADDR_BOARD 0x0C                   //!< No Address check
#else
#define CC1101_ADDR_BOARD 0x0E                   //!< Address Check and 0x00 is Broadcast
#endif

//! Set GD0 a
#if CC1101_MODE==GFSK_433M_250K_POLL
#define CC1101_GD2_CFG 0x29                     //!< Chip Ready
#elif CC1101_MODE==GFSK_433_250K_INT
#define CC1101_GD2_CFG 0x07                     //!< Asserts when a packet has been received with CRC OK.
                                                //!< De-asserts when the first byte is read from the RX FIFO.
#endif

//! configure registers value
static const RegType RegTbl[] =
{
//    { CC1101_IOCFG2   , CC1101_GD2_CFG,},        //!< GDO2 Output Pin Configuration
    { CC1101_IOCFG2   , 0X07,},        //!< GDO2 Output Pin Configuration
    { CC1101_IOCFG1   , 0x2E,},                  //!< GDO1 Output Pin Configuration
    { CC1101_IOCFG0   , 0x06,},                  //!< GDO0 Output Pin Configuration
    { CC1101_FIFOTHR  , 0x07,},                  //!< RX FIFO and TX FIFO Thresholds
    { CC1101_SYNC1    , 0xD3,},                  //!< Sync Word, High Byte
    { CC1101_SYNC0    , 0x91,},                  //!< Sync Word, Low Byte
    { CC1101_PKTLEN   , 0xFF,},                  //!< Packet Length
    { CC1101_PKTCTRL1 , CC1101_ADDR_BOARD,},     //!< Packet Automation Control
    { CC1101_PKTCTRL0 , 0x45,},                  //!< Packet Automation Control
    { CC1101_ADDR     , CC1101_DRV_ADDR,},       //!< Device Address
    { CC1101_CHANNR   , 0x00,},                  //!< Channel Number
    { CC1101_FSCTRL1  , 0x0C,},                  //!< Frequency Synthesizer Control
    { CC1101_FSCTRL0  , 0x00,},                  //!< Frequency Synthesizer Control
    { CC1101_FREQ2    , 0x10,},                  //!< Frequency Control Word, High Byte
    { CC1101_FREQ1    , 0xB1,},                  //!< Frequency Control Word, Middle Byte
    { CC1101_FREQ0    , 0x3B,},                  //!< Frequency Control Word, Low Byte
    { CC1101_MDMCFG4  , 0x2D,},                  //!< Modem Configuration
    { CC1101_MDMCFG3  , 0x3B,},                  //!< Modem Configuration
    { CC1101_MDMCFG2  , 0x13,},                  //!< Modem Configuration
    { CC1101_MDMCFG1  , 0x22,},                  //!< Modem Configuration
    { CC1101_MDMCFG0  , 0xF8,},                  //!< Modem Configuration
    { CC1101_DEVIATN  , 0x62,},                  //!< Modem Deviation Setting
    { CC1101_MCSM2    , 0x07,},                  //!< Main Radio Control State Machine Configuration
    { CC1101_MCSM1    , 0x30,},                  //!< Main Radio Control State Machine Configuration
    { CC1101_MCSM0    , 0x18,},                  //!< Main Radio Control State Machine Configuration
    { CC1101_FOCCFG   , 0x1D,},                  //!< Frequency Offset Compensation Configuration
    { CC1101_BSCFG    , 0x1C,},                  //!< Bit Synchronization Configuration
    { CC1101_AGCCTRL2 , 0xC7,},                  //!< AGC Control
    { CC1101_AGCCTRL1 , 0x00,},                  //!< AGC Control
    { CC1101_AGCCTRL0 , 0xB0,},                  //!< AGC Control
    { CC1101_WOREVT1  , 0x87,},                  //!< High Byte Event0 Timeout
    { CC1101_WOREVT0  , 0x6B,},                  //!< Low Byte Event0 Timeout
    { CC1101_WORCTRL  , 0xFB,},                  //!< Wake On Radio Control
    { CC1101_FREND1   , 0xB6,},                  //!< Front End RX Configuration
    { CC1101_FREND0   , 0x10,},                  //!< Front End TX Configuration
    { CC1101_FSCAL3   , 0xEA,},                  //!< Frequency Synthesizer Calibration
    { CC1101_FSCAL2   , 0x2A,},                  //!< Frequency Synthesizer Calibration
    { CC1101_FSCAL1   , 0x00,},                  //!< Frequency Synthesizer Calibration
    { CC1101_FSCAL0   , 0x1F,},                  //!< Frequency Synthesizer Calibration
    { CC1101_RCCTRL1  , 0x41,},                  //!< RC Oscillator Configuration
    { CC1101_RCCTRL0  , 0x00,},                  //!< RC Oscillator Configuration
    { CC1101_FSTEST   , 0x59,},                  //!< Frequency Synthesizer Calibration Control
    { CC1101_PTEST    , 0x7F,},                  //!< Production Test
    { CC1101_AGCTEST  , 0x3F,},                  //!< AGC Test
    { CC1101_TEST2    , 0x88,},                  //!< Various Test Settings
    { CC1101_TEST1    , 0x31,},                  //!< Various Test Settings
    { CC1101_TEST0    , 0x09,},                  //!< Various Test Setting
};

//! register table element count
static uint8_t reg_num = sizeof(RegTbl)/sizeof(RegType);

static void delay(volatile uint32_t tick)
{
    while(tick--);
}

#if 1
//! Power configure registers
static uint8_t PA_CfgTbl[8] = 
{
    0x60,  // PATABLE 0
    0x60,  // PATABLE 1
    0x60,  // PATABLE 2
    0x60,  // PATABLE 3
    0x60,  // PATABLE 4
    0x60,  // PATABLE 5
    0x60,  // PATABLE 6
    0x60,  // PATABLE 7    
};
#else
//! Power configure registers
static uint8_t PA_CfgTbl[8] = 
{
    0xC0,  // PATABLE 0
    0xC0,  // PATABLE 1
    0xC0,  // PATABLE 2
    0xC0,  // PATABLE 3
    0xC0,  // PATABLE 4
    0xC0,  // PATABLE 5
    0xC0,  // PATABLE 6
    0xC0,  // PATABLE 7    
};
#endif

uint8_t CC1101_Init(void)
{
    uint8_t i      = 0;
    uint8_t tmp[8] = {0};

    // HardWare Initialize
    SPI_INIT();
    PIN_CS_INIT();
    PIN_GD0_INIT();
    PIN_GD1_INIT();    
    PIN_GD2_INIT();

    // Delay
    delay(0xFFFF);
    CC1101_StrobeSend(STROBE_SRES);
    delay(0xFFFF);

    // Configure RF-IC register
    // Note: register value can be imported from TI's tool SmartRF
    for(i = 0; i < reg_num; i++)
    {
        CC1101_RegWrite(RegTbl[i].addr, RegTbl[i].value);
        delay(0xFFF);
        tmp[0] = CC1101_RegRead(RegTbl[i].addr);

        if(tmp[0] != RegTbl[i].value)
        {
            // Error
            return (1);
        }
    }

    // Configure RF-IC Power register
    CC1101_RegWriteBurst(CC1101_PATABLE, PA_CfgTbl, sizeof(PA_CfgTbl));
    CC1101_RegReadBurst(CC1101_PATABLE, tmp, sizeof(PA_CfgTbl));
    for (i = 0; i < sizeof(PA_CfgTbl); i++)
    {
        if(PA_CfgTbl[i] != tmp[i])
        {
            // Error
            return (1);
        }
    }
    
    return (0);
}

uint8_t CC1101_RegRead(uint8_t addr)
{

    uint8_t value = 0;

    // 0 <= addr <= 0x2E
    assert(addr <= (uint8_t)0x2E || addr == CC1101_RXFIFO);

    // Select SPI Slave
    PIN_CS_LOW();

    while(PIN_GD1_READ());

    // Send addr then read back data
    SPI_EXCHANGE_DATA(addr | READ_SINGLE);
    value = SPI_EXCHANGE_DATA(DUMP_DATA);

    // Release SPI Slave
    PIN_CS_HIGH();

    return (value);
}

uint8_t CC1101_RegWrite(uint8_t addr, uint8_t value)
{
    // 0 <= addr <= 0x2E
    assert(addr <= (uint8_t)0x2E || addr == CC1101_TXFIFO);

    // Select SPI Slave
    PIN_CS_LOW();

    while(PIN_GD1_READ());

    // Send addr first then data
    SPI_EXCHANGE_DATA(addr);
    SPI_EXCHANGE_DATA(value);    

    // Release SPI Slave
    PIN_CS_HIGH();
    
    return (0);
}

uint8_t CC1101_RegReadBurst(uint8_t addr, uint8_t *pData, uint8_t len)
{
    uint8_t i = 0;

    assert(addr <= (uint8_t)0x2E || addr == CC1101_RXFIFO || addr == CC1101_PATABLE);

    // Select SPI Slave
    PIN_CS_LOW();

    while(PIN_GD1_READ()); 

    // Send addr first then read back data
    SPI_EXCHANGE_DATA(addr | READ_BURST);
    
    for(i = 0; i < len; i++)
    {
        pData[i] = SPI_EXCHANGE_DATA(DUMP_DATA);
    } 

    // Release SPI Slave
    PIN_CS_HIGH();
    
    return (len);
}

uint8_t CC1101_RegWriteBurst(uint8_t addr, uint8_t *pData, uint8_t len)
{
    uint8_t i = 0;

    assert(addr <= (uint8_t)0x2E || addr == CC1101_TXFIFO || addr == CC1101_PATABLE);

    // Select SPI Slave
    PIN_CS_LOW();

    while(PIN_GD1_READ());

    // Send addr first then data
    SPI_EXCHANGE_DATA(addr | WRITE_BURST);
    
    for(i = 0; i < len; i++)
    {
        SPI_EXCHANGE_DATA(pData[i]);
    }    

    // Release SPI Slave
    PIN_CS_HIGH();
    
    return (len);
}

uint8_t CC1101_StatusRead(uint8_t addr)
{
    uint8_t value = 0;

    assert( addr == CC1101_PARTNUM        ||
            addr == CC1101_VERSION        ||
            addr == CC1101_FREQEST        ||
            addr == CC1101_LQI            ||
            addr == CC1101_RSSI           ||
            addr == CC1101_MARCSTATE      ||
            addr == CC1101_WORTIME1       ||
            addr == CC1101_WORTIME0       ||
            addr == CC1101_PKTSTATUS      ||
            addr == CC1101_VCO_VC_DAC     ||
            addr == CC1101_TXBYTES        ||
            addr == CC1101_RXBYTES        ||
            addr == CC1101_RCCTRL1_STATUS ||
            addr == CC1101_RCCTRL0_STATUS );

    // Select SPI Slave
    PIN_CS_LOW();
    
    while(PIN_GD1_READ());

    // Send addr then read back data
    SPI_EXCHANGE_DATA(addr | READ_BURST);
    value = SPI_EXCHANGE_DATA(DUMP_DATA);

    // Release SPI Slave
    PIN_CS_HIGH();

    return value;
}

uint8_t CC1101_StrobeSend(uint8_t cmd)
{
    assert( cmd == STROBE_SRES     ||
            cmd == STROBE_SFSTXON  ||
            cmd == STROBE_SXOFF    ||
            cmd == STROBE_SCAL     ||
            cmd == STROBE_SRX      ||
            cmd == STROBE_STX      ||
            cmd == STROBE_SIDLE    ||
            cmd == STROBE_SWOR     ||
            cmd == STROBE_SPWD     ||
            cmd == STROBE_SFRX     ||
            cmd == STROBE_SFTX     ||
            cmd == STROBE_SWORRST  ||
            cmd == STROBE_SNOP     );

    // Select SPI Slave
    PIN_CS_LOW();

    while(PIN_GD1_READ());
    
    // Send strobe command
    SPI_EXCHANGE_DATA(cmd);
   
    // Release SPI Slave
    PIN_CS_HIGH();
    
    return (0);
}

uint8_t CC1101_PacketSend(uint8_t* pData, uint8_t len)
{
    assert(pData != (void *)0 && len != 0);

    CC1101_RegWriteBurst(CC1101_TXFIFO, pData, len);

    CC1101_StrobeSend(STROBE_STX);

    // Wait for GDO0 to be set -> sync transmitted
    while (!PIN_GD0_READ());

    // Wait for GDO0 to be cleared -> end of packet
    while (PIN_GD0_READ());
    
    return (len);
}

// TODO
uint8_t CC1101_PacketSend_ISR(uint8_t* pData, uint8_t len)
{
    assert(pData != (void *)0 && len != 0);

    CC1101_RegWriteBurst(CC1101_TXFIFO, pData, len);

    CC1101_StrobeSend(STROBE_STX);

    // Wait for GDO0 to be set -> sync transmitted
    //while (!PIN_GD0_READ());

    // Wait for GDO0 to be cleared -> end of packet
    //while (PIN_GD0_READ());
    
    return (len);
}

uint8_t CC1101_PacketRecv(uint8_t* pData, uint8_t len)
{    
    uint8_t packetLength  = 0;
    uint8_t payloadLength = 0;

    assert(pData != (void *)0 && len != 0);

    CC1101_StrobeSend(STROBE_SRX);

    // Wait for GDO0 to be set -> sync received    
    while (!PIN_GD0_READ());

    // Wait for GDO0 to be cleared -> end of packet
    while (PIN_GD0_READ());

    // This status register is safe to read since it will not be updated after
    // the packet has been received (See the CC1100 and 2500 Errata Note)
    packetLength = CC1101_StatusRead(CC1101_RXBYTES) & BYTES_IN_RXFIFO;
    if (packetLength)
    {
        // Read length (in bytes)
        payloadLength = CC1101_RegRead(CC1101_RXFIFO);
        
        if((payloadLength + 3 == packetLength) && (packetLength <= len))
        {
            pData[0] = packetLength;
            CC1101_RegReadBurst(CC1101_RXFIFO, &pData[1], packetLength-1);

            return (packetLength);
        }
        else // Flush RX FIFO
        {
            // Make sure that the radio is in IDLE state before flushing the FIFO
            // (Unless RXOFF_MODE has been changed, the radio should be in IDLE state at this point) 
            CC1101_StrobeSend(STROBE_SIDLE);

            // Flush RX FIFO
            CC1101_StrobeSend(STROBE_SFRX);

            return (0);
        }
    }
    else    // No Data Avaible
    {
        return (0);
    }
}

uint8_t CC1101_PacketRecv_ISR(uint8_t* pData, uint8_t len)
{
    uint8_t packetLength  = 0;
    uint8_t payloadLength = 0;

    assert(pData != (void *)0 && len != 0);

    //CC1101_StrobeSend(STROBE_SRX);

    // Wait for GDO0 to be set -> sync received
    //while (!PIN_GD0_READ());

    // Wait for GDO0 to be cleared -> end of packet
    //while (PIN_GD0_READ());

    // This status register is safe to read since it will not be updated after
    // the packet has been received (See the CC1100 and 2500 Errata Note)
    packetLength = CC1101_StatusRead(CC1101_RXBYTES) & BYTES_IN_RXFIFO;
    if (packetLength)
    {
        // Read length (in bytes)
        payloadLength = CC1101_RegRead(CC1101_RXFIFO);
        
        if((payloadLength + 3 == packetLength) && (packetLength <= len))
        {
            pData[0] = packetLength;
            CC1101_RegReadBurst(CC1101_RXFIFO, &pData[1], packetLength-1);

            return (packetLength);
        }
        else // Flush RX FIFO
        {
            // Make sure that the radio is in IDLE state before flushing the FIFO
            // (Unless RXOFF_MODE has been changed, the radio should be in IDLE state at this point) 
            CC1101_StrobeSend(STROBE_SIDLE);

            // Flush RX FIFO
            CC1101_StrobeSend(STROBE_SFRX);

            return (0);
        }
    }
    else    // No Data Avaible
    {
        return (0);
    }
}