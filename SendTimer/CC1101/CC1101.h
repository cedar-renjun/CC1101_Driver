/**************************************************************************************************
 * 
 * \file        CC1101.h
 * \brief       Provide Some CC1101 Basic APIs
 * \version     V1.0
 * \date        2014-06-10
 * \author      Cedar
 * 
 **************************************************************************************************/

#ifndef __CC1101_H_
#define __CC1101_H_

#ifdef  __cpluscplus
extern "C"
{
#endif

//#include "CC1101_Cfg_backup.h"
#include "CC1101_Cfg.h"

/**************************************************************************************************
 * 
 * @addtogroup CC1101_Regs CC1101 Registers
 * @{
 * 
 **************************************************************************************************/

/**************************************************************************************************
 * 
 * @addtogroup CC1101_Regs_Cfg Configure Registers
 * @{
 * 
 **************************************************************************************************/

#define CC1101_IOCFG2         0x00  //!<  GDO2 Output Pin Configuration
#define CC1101_IOCFG1         0x01  //!<  GDO1 Output Pin Configuration
#define CC1101_IOCFG0         0x02  //!<  GDO0 Output Pin Configuration
#define CC1101_FIFOTHR        0x03  //!<  RX FIFO and TX FIFO Thresholds
#define CC1101_SYNC1          0x04  //!<  Sync Word, High Byte
#define CC1101_SYNC0          0x05  //!<  Sync Word, Low Byte
#define CC1101_PKTLEN         0x06  //!<  Packet Length
#define CC1101_PKTCTRL1       0x07  //!<  Packet Automation Control
#define CC1101_PKTCTRL0       0x08  //!<  Packet Automation Control
#define CC1101_ADDR           0x09  //!<  Device Address
#define CC1101_CHANNR         0x0A  //!<  Channel Number
#define CC1101_FSCTRL1        0x0B  //!<  Frequency Synthesizer Control
#define CC1101_FSCTRL0        0x0C  //!<  Frequency Synthesizer Control
#define CC1101_FREQ2          0x0D  //!<  Frequency Control Word, High Byte
#define CC1101_FREQ1          0x0E  //!<  Frequency Control Word, Middle Byte
#define CC1101_FREQ0          0x0F  //!<  Frequency Control Word, Low Byte
#define CC1101_MDMCFG4        0x10  //!<  Modem Configuration
#define CC1101_MDMCFG3        0x11  //!<  Modem Configuration
#define CC1101_MDMCFG2        0x12  //!<  Modem Configuration
#define CC1101_MDMCFG1        0x13  //!<  Modem Configuration
#define CC1101_MDMCFG0        0x14  //!<  Modem Configuration
#define CC1101_DEVIATN        0x15  //!<  Modem Deviation Setting
#define CC1101_MCSM2          0x16  //!<  Main Radio Control State Machine Configuration
#define CC1101_MCSM1          0x17  //!<  Main Radio Control State Machine Configuration
#define CC1101_MCSM0          0x18  //!<  Main Radio Control State Machine Configuration
#define CC1101_FOCCFG         0x19  //!<  Frequency Offset Compensation Configuration
#define CC1101_BSCFG          0x1A  //!<  Bit Synchronization Configuration
#define CC1101_AGCCTRL2       0x1B  //!<  AGC Control
#define CC1101_AGCCTRL1       0x1C  //!<  AGC Control
#define CC1101_AGCCTRL0       0x1D  //!<  AGC Control
#define CC1101_WOREVT1        0x1E  //!<  High Byte Event0 Timeout
#define CC1101_WOREVT0        0x1F  //!<  Low Byte Event0 Timeout
#define CC1101_WORCTRL        0x20  //!<  Wake On Radio Control
#define CC1101_FREND1         0x21  //!<  Front End RX Configuration
#define CC1101_FREND0         0x22  //!<  Front End TX Configuration
#define CC1101_FSCAL3         0x23  //!<  Frequency Synthesizer Calibration
#define CC1101_FSCAL2         0x24  //!<  Frequency Synthesizer Calibration
#define CC1101_FSCAL1         0x25  //!<  Frequency Synthesizer Calibration
#define CC1101_FSCAL0         0x26  //!<  Frequency Synthesizer Calibration
#define CC1101_RCCTRL1        0x27  //!<  RC Oscillator Configuration
#define CC1101_RCCTRL0        0x28  //!<  RC Oscillator Configuration
#define CC1101_FSTEST         0x29  //!<  Frequency Synthesizer Calibration Control
#define CC1101_PTEST          0x2A  //!<  Production Test
#define CC1101_AGCTEST        0x2B  //!<  AGC Test
#define CC1101_TEST2          0x2C  //!<  Various Test Settings
#define CC1101_TEST1          0x2D  //!<  Various Test Settings
#define CC1101_TEST0          0x2E  //!<  Various Test Settings

/**************************************************************************************************
 * 
 * @}
 * 
 **************************************************************************************************/

/**************************************************************************************************
 * 
 * @addtogroup CC1101_Regs_Status Status Registers
 * @{
 * 
 **************************************************************************************************/

#define CC1101_PARTNUM        0x30  //!<  Chip ID
#define CC1101_VERSION        0x31  //!<  Chip ID
#define CC1101_FREQEST        0x32  //!<  Frequency Offset Estimate from Demodulator
#define CC1101_LQI            0x33  //!<  Demodulator Estimate for Link Quality
#define CC1101_RSSI           0x34  //!<  Received Signal Strength Indication
#define CC1101_MARCSTATE      0x35  //!<  Main Radio Control State Machine State
#define CC1101_WORTIME1       0x36  //!<  High Byte of WOR Time
#define CC1101_WORTIME0       0x37  //!<  Low Byte of WOR Time
#define CC1101_PKTSTATUS      0x38  //!<  Current GDOx Status and Packet Status
#define CC1101_VCO_VC_DAC     0x39  //!<  Current Setting from PLL Calibration Module
#define CC1101_TXBYTES        0x3A  //!<  Underflow and Number of Bytes
#define CC1101_RXBYTES        0x3B  //!<  Overflow and Number of Bytes
#define CC1101_RCCTRL1_STATUS 0x3C  //!<  Last RC Oscillator Calibration Result
#define CC1101_RCCTRL0_STATUS 0x3D  //!<  Last RC Oscillator Calibration Result

/**************************************************************************************************
 * 
 * @}
 * 
 **************************************************************************************************/

/**************************************************************************************************
 * 
 * @addtogroup CC1101_Regs_Strobe Strobe Commands
 * @{
 * 
 **************************************************************************************************/

#define STROBE_SRES           0x30  //!< Reset chip.
#define STROBE_SFSTXON        0x31  //!< Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1).
                                    //!< If in RX (with CCA): Go to a wait state where only the synthesizer is
                                    //!< running (for quick RX / TX turnaround).
#define STROBE_SXOFF          0x32  //!< Turn off crystal oscillator.
#define STROBE_SCAL           0x33  //!< Calibrate frequency synthesizer and turn it off. SCAL can be strobed from
                                    //!< IDLE mode without setting manual calibration mode (MCSM0.FS_AUTOCAL=0)
#define STROBE_SRX            0x34  //!< Enable RX. Perform calibration first if coming from IDLE and MCSM0.FS_AUTOCAL=1.
#define STROBE_STX            0x35  //!< In IDLE state: Enable TX. Perform calibration first if MCSM0.FS_AUTOCAL=1.
                                    //!< If in RX state and CCA is enabled: Only go to TX if channel is clear.
#define STROBE_SIDLE          0x36  //!< Exit RX / TX, turn off frequency synthesizer and exit Wake-On-Radio mode if applicable.
#define STROBE_SWOR           0x38  //!< Start automatic RX polling sequence (Wake-on-Radio) if WORCTRL.RC_PD=0.
#define STROBE_SPWD           0x39  //!< Enter power down mode when CSn goes high.
#define STROBE_SFRX           0x3A  //!< Flush the RX FIFO buffer. Only issue SFRX in IDLE or RXFIFO_OVERFLOW states.
#define STROBE_SFTX           0x3B  //!< Flush the TX FIFO buffer. Only issue SFTX in IDLE or TXFIFO_UNDERFLOW states.
#define STROBE_SWORRST        0x3C  //!< Reset real time clock to Event1 value.
#define STROBE_SNOP           0x3D  //!< No operation. May be used to get access to the chip status byte.

/**************************************************************************************************
 * 
 * @}
 * 
 **************************************************************************************************/

/**************************************************************************************************
 * 
 * @addtogroup CC1101_Regs_Etc Etc
 * @{
 * 
 **************************************************************************************************/

#define CC1101_PATABLE        0x3E  //!< PA power setting.
#define CC1101_TXFIFO         0x3F  //!< TX FIFO
#define CC1101_RXFIFO         0x3F  //!< RX FIFO

/**************************************************************************************************
 * 
 * @}
 * 
 **************************************************************************************************/

/**************************************************************************************************
 *  
 * @}
 * 
 **************************************************************************************************/


/**************************************************************************************************
 * 
 * @addtogroup CC1101_Export_APIs Export APIs
 * @{
 * 
 **************************************************************************************************/

/**
 * @brief   Initialize CC1101 Hardware
 * 
 * @return  0     --> success
 *          NOT 0 --> failure
 *          
 * @note    This Function must be called first when using CC1101 RF-IC
 */
extern uint8_t CC1101_Init(void);

/**
 * @brief   Read CC1101 register in single mode
 * 
 * @param   [in] addr is register address you want to read, which can be one of the following
 *               CC1101_IOCFG2  0x00
 *               CC1101_IOCFG1  0x01
 *               CC1101_IOCFG0  0x02
 *               CC1101_FIFOTHR 0x03
 *               ...
 *               CC1101_TEST0   0x2E
 * 
 * @return  the value of register.           
 *          
 * @note    This API is low-layer function, NOT suggest application developer use this function
 *          directly.
 */
extern uint8_t CC1101_RegRead(uint8_t addr);

/**
 * @brief   Write CC1101 register in single mode
 * 
 * @param   [in] addr is register address you want to write, which can be one of the following
 *               CC1101_IOCFG2  0x00
 *               CC1101_IOCFG1  0x01
 *               CC1101_IOCFG0  0x02
 *               CC1101_FIFOTHR 0x03
 *               ...
 *               CC1101_TEST0   0x2E
 * 
 * @param   [in] value is target register value.
 * 
 * @return  Reserved for future
 *          always return 0 now.
 *
 * @note    This API is low-layer function, NOT suggest application developer use this function
 *          directly.
 */
extern uint8_t CC1101_RegWrite(uint8_t addr, uint8_t value);

/**
 * @brief   Read CC1101 register in burst mode
 * 
 * @param   [in] addr is register address you want to read, which can be one of the following
 *               CC1101_IOCFG2  0x00
 *               CC1101_IOCFG1  0x01
 *               CC1101_IOCFG0  0x02
 *               CC1101_FIFOTHR 0x03
 *               ...
 *               CC1101_TEST0   0x2E
 *               
 *               OR
 *               
 *               CC1101_RXFIFO  0x3F
 *               
 * @param   [out] pData is the data buffer address pointer
 * 
 * @param   [in] the number of data you want to read
 * 
 * @return  the number of data that have been receive successfully
 *          
 * @note    This API is low-layer function, NOT suggest application developer use this function
 *          directly.
 */
extern uint8_t CC1101_RegReadBurst(uint8_t addr, uint8_t *pData, uint8_t len);

/**
 * @brief   Write CC1101 register in burst mode
 * 
 * @param   [in] addr is register address you want to write, which can be one of the following
 *               CC1101_IOCFG2  0x00
 *               CC1101_IOCFG1  0x01
 *               CC1101_IOCFG0  0x02
 *               CC1101_FIFOTHR 0x03
 *               ...
 *               CC1101_TEST0   0x2E
 *               
 *               OR
 *               
 *               CC1101_TXFIFO  0x3F
 *               
 * @param   [out] pData is the data buffer address pointer
 * 
 * @param   [in] the number of data you want to write
 * 
 * @return  the number of data that have been write successfully
 *          
 * @note    This API is low-layer function, NOT suggest application developer use this function
 *          directly.
 */
extern uint8_t CC1101_RegWriteBurst(uint8_t addr, uint8_t *pData, uint8_t len);

/**
 * @brief   Read CC1101 status register.
 * 
 * @param   [in] addr is register address you want to read, which can be one of the following
 *               CC1101_PARTNUM         --> 0x30
 *               CC1101_VERSION         --> 0x31
 *               CC1101_FREQEST         --> 0x32
 *               CC1101_LQI             --> 0x33
 *               CC1101_RSSI            --> 0x34
 *               CC1101_MARCSTATE       --> 0x35
 *               CC1101_WORTIME1        --> 0x36
 *               CC1101_WORTIME0        --> 0x37
 *               CC1101_PKTSTATUS       --> 0x38
 *               CC1101_VCO_VC_DAC      --> 0x39
 *               CC1101_TXBYTES         --> 0x3A
 *               CC1101_RXBYTES         --> 0x3B
 *               CC1101_RCCTRL1_STATUS  --> 0x3C
 *               CC1101_RCCTRL0_STATUS  --> 0x3D
 * 
 * @return  the status register value.
 *
 * @note    This API is low-layer function, NOT suggest application developer use this function
 *          directly.
 */
extern uint8_t CC1101_StatusRead(uint8_t addr);

/**
 * @brief   Release strobe command
 * 
 * @param   [in] cmd is CC1101 support strobe commands, which can be one of the following
 *               
 *               STROBE_SRES
 *               STROBE_SFSTXON
 *               STROBE_SXOFF
 *               STROBE_SCAL
 *               STROBE_SRX
 *               STROBE_STX
 *               STROBE_SIDLE
 *               STROBE_SWOR
 *               STROBE_SPWD
 *               STROBE_SFRX
 *               STROBE_SFTX
 *               STROBE_SWORRST
 *               STROBE_SNOP
 *
 * @return  Reserved for future
 *          always return 0 now.
 */
extern uint8_t CC1101_StrobeSend(uint8_t cmd);

/**
 * @brief   Send Packet data(polling)
 * 
 * @param   [in] pData is the data buffer address pointer
 *
 * @param   [in] len is the data length you want to send
 *               (in bytes)
 *
 * @return  the number of data that have been sent successfully
 * 
 * @note    If you want to send data in ISR, use CC1101_PacketSend_ISR instead
 */
extern uint8_t CC1101_PacketSend(uint8_t* pData, uint8_t len);

/**
 * @brief   Send Packet data (ISR)
 * 
 * @param   [in] pData is the data buffer address pointer
 *
 * @param   [in] len is the data length you want to send
 *               (in bytes)
 *
 * @return  Reserved for future.
 *          Always return len.
 * 
 * @note    This function is designed for ISR
 */
extern uint8_t CC1101_PacketSend_ISR(uint8_t* pData, uint8_t len);

/**
 * @brief   Receive packet data (polling)
 * 
 * @param   [out] pData is the data buffer address pointer
 *
 * @param   [in] len is the data buffer size (in bytes)
 *
 * @return  the number of data that have been receive successfully
 * 
 * @note    If you want receive packet in ISR, use CC1101_PacketRecv_ISR instead
 */
extern uint8_t CC1101_PacketRecv(uint8_t* pData, uint8_t len);

/**
 * @brief   Receive packet data (ISR)
 * 
 * @param   [out] pData is the data buffer address pointer
 *
 * @param   [in] len is the data buffer size (in bytes)
 *
 * @return  the number of data that have been receive successfully
 * 
 * @note    This function can be used in ISR
 */
extern uint8_t CC1101_PacketRecv_ISR(uint8_t* pData, uint8_t len);

/**************************************************************************************************
 *  
 * @}
 * 
 **************************************************************************************************/

#ifdef  __cpluscplus
}
#endif

#endif

/*********************************** FILE END *****************************************************/