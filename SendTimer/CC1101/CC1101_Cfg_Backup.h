
#ifndef __CC1101_CFG_H__
#define __CC1101_CFG_H__

#ifdef  __cpluscplus
extern "C"
{
#endif

/**************************************************************************************************
 *                        Pre-defined MACRO
 *  Notice:
 *         Application User MUST NOT change those macro
 *  
 *************************************************************************************************/

#define GFSK_433_250K_POLL          0
#define GFSK_433_250K_INT           1
#define MCU_STM32                   0
#define MCU_8051                    1

/**************************************************************************************************
 *                        CC1101 App-Layer Configure
 *************************************************************************************************/

#define MCU_TYPE                    MCU_STM32
#define CC1101_MODE                 GFSK_433_250K_POLL
#define CC1101_ADDR_EN              0
#define CC1101_DRV_ADDR             0x00

/**************************************************************************************************
 *                        MCU Low-Layer Configure
 *************************************************************************************************/

/*------------------------------------------- STM32 ----------------------------------------------*/

#if     MCU_TYPE == MCU_STM32

#include "stm32f10x.h"
#include <stdint.h>

//////////////////////////////// Pin Configure //////////////////////////////////////////////////
#define CC1101_SPI_ID               2               //!< STM32 SPI Module ID, can be 1 or 2

#define CC1101_GD0_PORT             GPIOB           //!< Port   : A/B/C...
#define CC1101_GD0_PIN              GPIO_Pin_10     //!< Pin ID : 0/1/2...15

#define CC1101_GD1_PORT             GPIOB           //!< Port   : A/B/C...
#define CC1101_GD1_PIN              GPIO_Pin_14     //!< Pin ID : 0/1/2...15

#define CC1101_GD2_PORT             GPIOB           //!< Port   : A/B/C...
#define CC1101_GD2_PIN              GPIO_Pin_11     //!< Pin ID : 0/1/2...15

#define CC1101_CS_PORT              GPIOB           //!< Port   : A/B/C...
#define CC1101_CS_PIN               GPIO_Pin_12     //!< Pin ID : 0/1/2...15

/////////////////////////////// Function Configure ///////////////////////////////////////////////

extern void    STM32_PIN_GD0_INIT(void);
extern uint8_t STM32_PIN_GD0_READ(void);
extern void    STM32_PIN_GD1_INIT(void);
extern uint8_t STM32_PIN_GD1_READ(void);
extern void    STM32_PIN_GD2_INIT(void);
extern void    STM32_PIN_CS_INIT(void);
extern void    STM32_PIN_CS_HIGH(void);
extern void    STM32_PIN_CS_LOW(void);
extern void    STM32_SPI_INIT(void);
extern uint8_t STM32_SPI_EXCHANGE_DATA(uint8_t data_tx);

#define PIN_GD0_INIT()              STM32_PIN_GD0_INIT()
#define PIN_GD0_READ()              STM32_PIN_GD0_READ()  

#define PIN_GD1_INIT()              STM32_PIN_GD1_INIT()
#define PIN_GD1_READ()              STM32_PIN_GD1_READ()

#define PIN_GD2_INIT()              STM32_PIN_GD2_INIT()
#define PIN_GD2_READ()              

#define PIN_CS_INIT()               STM32_PIN_CS_INIT()
#define PIN_CS_HIGH()               STM32_PIN_CS_HIGH()
#define PIN_CS_LOW()                STM32_PIN_CS_LOW()

#define SPI_INIT()                  STM32_SPI_INIT()
#define SPI_EXCHANGE_DATA(data_rx)  STM32_SPI_EXCHANGE_DATA(data_rx)

#endif // MCU_TYPE == MCU_STM32

#ifdef  __cpluscplus
}
#endif

#endif // __CC1101_CFG_H__