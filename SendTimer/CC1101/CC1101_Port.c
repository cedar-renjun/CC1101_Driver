
#include "CC1101_Cfg.h"

#if     MCU_TYPE == MCU_STM32

static void GPIO_PORT_ENABLE(GPIO_TypeDef* x)
{
    if(GPIOA == x)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
        return;
    }
    else if(GPIOB == x)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
        return;
    }
    else if(GPIOC == x)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); 
        return;
    }
    else if(GPIOD == x)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE); 
        return;
    }
    else if(GPIOE == x)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE); 
        return;
    }
    else if(GPIOF == x)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE); 
        return;
    }
    else if(GPIOG == x)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE); 
        return;
    }
}

#if CC1101_MODE==GFSK_433_250K_INT
static void GPIO_EXT_Cfg(GPIO_TypeDef* port, uint16_t pin)
{
    uint8_t PortSrc = 0;
    uint8_t i       = 0;

    if(GPIOA == port)
    {
        PortSrc = GPIO_PortSourceGPIOA;
    }
    else if(GPIOB == port)
    {
        PortSrc = GPIO_PortSourceGPIOB;
    }
    else if(GPIOC == port)
    {
        PortSrc = GPIO_PortSourceGPIOC;
    }
    else if(GPIOD == port)
    {
        PortSrc = GPIO_PortSourceGPIOD;
    }
    else if(GPIOE == port)
    {
        PortSrc = GPIO_PortSourceGPIOE;
    }
    else if(GPIOF == port)
    {
        PortSrc = GPIO_PortSourceGPIOF;
    }
    else if(GPIOG == port)
    {
        PortSrc = GPIO_PortSourceGPIOG;
    }

    for(i = 0; i < 16; i++)
    {
        if(pin & 1<<i)
        {
            GPIO_EXTILineConfig(PortSrc, i);
            return;
        }
    }
}
#endif

void STM32_PIN_GD0_INIT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;                                      
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_PORT_ENABLE(CC1101_GD0_PORT);
    GPIO_InitStructure.GPIO_Pin  = CC1101_GD0_PIN;                               
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;                             
    GPIO_Init(CC1101_GD0_PORT, &GPIO_InitStructure);                                                                                                                    
}

uint8_t STM32_PIN_GD0_READ(void)
{
    return (uint8_t) GPIO_ReadInputDataBit(CC1101_GD0_PORT, CC1101_GD0_PIN);
}

void STM32_PIN_GD1_INIT(void)
{
    // Nothing is here
}

uint8_t STM32_PIN_GD1_READ(void)
{
    return (uint8_t) GPIO_ReadInputDataBit(CC1101_GD1_PORT, CC1101_GD1_PIN);
}

void STM32_PIN_GD2_INIT(void)
{
    //! Set GD2 as receive interrupt source
#if CC1101_MODE==GFSK_433M_250K_POLL
    
    GPIO_InitTypeDef   GPIO_InitStructure;
    /* Enable GPIOB clock */
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_PORT_ENABLE(CC1101_GD2_PORT);

    /* Configure GD2 pin as input floating */
    GPIO_InitStructure.GPIO_Pin  = CC1101_GD2_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(CC1101_GD2_PORT, &GPIO_InitStructure);

#elif CC1101_MODE==GFSK_433_250K_INT

    EXTI_InitTypeDef   EXTI_InitStructure;
    GPIO_InitTypeDef   GPIO_InitStructure;
    NVIC_InitTypeDef   NVIC_InitStructure;
    /* Enable GPIOB clock */
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_PORT_ENABLE(CC1101_GD2_PORT);

    /* Configure GD2 pin as input floating */
    GPIO_InitStructure.GPIO_Pin  = CC1101_GD2_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(CC1101_GD2_PORT, &GPIO_InitStructure);

    /* Enable AFIO clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    GPIO_EXT_Cfg(CC1101_GD2_PORT, CC1101_GD2_PIN);
    //GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource11);

    /* Configure EXTI0 line */
    EXTI_InitStructure.EXTI_Line    = (uint32_t)CC1101_GD2_PIN;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt; 
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable and set EXTI0 Interrupt to the lowest priority */
    if      (CC1101_GD2_PIN == GPIO_Pin_0)
    {
        NVIC_InitStructure.NVIC_IRQChannel  = EXTI0_IRQn;
    }
    else if (CC1101_GD2_PIN == GPIO_Pin_1)
    {
        NVIC_InitStructure.NVIC_IRQChannel  = EXTI1_IRQn;
    }
    else if (CC1101_GD2_PIN == GPIO_Pin_2)
    {
        NVIC_InitStructure.NVIC_IRQChannel  = EXTI2_IRQn;
    }
    else if (CC1101_GD2_PIN == GPIO_Pin_3)
    {
        NVIC_InitStructure.NVIC_IRQChannel  = EXTI3_IRQn;
    }
    else if (CC1101_GD2_PIN == GPIO_Pin_4)
    {
        NVIC_InitStructure.NVIC_IRQChannel  = EXTI4_IRQn;
    }
    else if (CC1101_GD2_PIN == GPIO_Pin_5 || CC1101_GD2_PIN == GPIO_Pin_6 || 
             CC1101_GD2_PIN == GPIO_Pin_7 || CC1101_GD2_PIN == GPIO_Pin_8 ||
             CC1101_GD2_PIN == GPIO_Pin_9)
    {
        NVIC_InitStructure.NVIC_IRQChannel  = EXTI9_5_IRQn;
    }
    else
    {
        NVIC_InitStructure.NVIC_IRQChannel  = EXTI15_10_IRQn;
    }

    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

#else

    // Add your code here

#endif

}

uint8_t STM32_PIN_GD2_READ(void)
{
    return (uint8_t) GPIO_ReadInputDataBit(CC1101_GD2_PORT, CC1101_GD2_PIN);
}

void STM32_PIN_CS_INIT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_PORT_ENABLE(CC1101_CS_PORT);
    GPIO_InitStructure.GPIO_Pin   = CC1101_CS_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(CC1101_CS_PORT, &GPIO_InitStructure);   
}

void STM32_PIN_CS_HIGH(void)
{
    GPIO_SetBits(CC1101_CS_PORT, CC1101_CS_PIN);
}
    
void STM32_PIN_CS_LOW(void)
{
    GPIO_ResetBits(CC1101_CS_PORT, CC1101_CS_PIN);
}

void STM32_SPI_INIT(void)
{
    SPI_InitTypeDef  SPI_InitStructure;                                         
    GPIO_InitTypeDef GPIO_InitStructure;                                        
#if CC1101_SPI_ID == 2
    /* Configure SPI SCK, MISO, MOSI Pins */                                    
    /* SCK---->PB13                       */                                    
    /* MISO--->PB14                       */                                    
    /* MOSI--->PB15                       */                                    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE); 
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;    
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;                            
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                           
    GPIO_Init(GPIOB, &GPIO_InitStructure);                                      
                                                                                
    /* Configure SPI2 Port */                                                   
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);                        
    SPI_InitStructure.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;  
    SPI_InitStructure.SPI_Mode              = SPI_Mode_Master;                  
    SPI_InitStructure.SPI_DataSize          = SPI_DataSize_8b;                  
    SPI_InitStructure.SPI_CPOL              = SPI_CPOL_High;                    
    SPI_InitStructure.SPI_CPHA              = SPI_CPHA_2Edge;                   
    SPI_InitStructure.SPI_NSS               = SPI_NSS_Soft;                     
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;          
    SPI_InitStructure.SPI_FirstBit          = SPI_FirstBit_MSB;                 
    SPI_InitStructure.SPI_CRCPolynomial     = 7;                                
    SPI_Init(SPI2, &SPI_InitStructure);                                         
                                                                                
    /* Enable SPI2 */                                                           
    SPI_Cmd(SPI2, ENABLE);

#elif CC1101_SPI_ID == 1
    
    /* Configure SPI SCK, MISO, MOSI Pins */                                    
    /* SCK---->PA5                       */                                    
    /* MISO--->PA6                       */                                    
    /* MOSI--->PA7                       */                                    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA, ENABLE); 
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;    
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;                            
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                           
    GPIO_Init(GPIOA, &GPIO_InitStructure);                                      
                                                                                
    /* Configure SPI1 Port */                                                   
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI1, ENABLE);                        
    SPI_InitStructure.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;  
    SPI_InitStructure.SPI_Mode              = SPI_Mode_Master;                  
    SPI_InitStructure.SPI_DataSize          = SPI_DataSize_8b;                  
    SPI_InitStructure.SPI_CPOL              = SPI_CPOL_High;                    
    SPI_InitStructure.SPI_CPHA              = SPI_CPHA_2Edge;                   
    SPI_InitStructure.SPI_NSS               = SPI_NSS_Soft;                     
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;          
    SPI_InitStructure.SPI_FirstBit          = SPI_FirstBit_MSB;                 
    SPI_InitStructure.SPI_CRCPolynomial     = 7;                                
    SPI_Init(SPI1, &SPI_InitStructure);                                         
                                                                                
    /* Enable SPI2 */                                                           
    SPI_Cmd(SPI1, ENABLE);
#endif

}

uint8_t STM32_SPI_EXCHANGE_DATA(uint8_t data_tx)
{
    uint8_t data_rx = 0;
#if CC1101_SPI_ID == 2

    // Send one byte then read back one byte
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(SPI2, data_tx);

    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
    data_rx = (uint8_t) SPI_I2S_ReceiveData(SPI2);

#elif CC1101_SPI_ID == 1

    // Send one byte then read back one byte
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(SPI1, data_tx);

    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
    data_rx = (uint8_t) SPI_I2S_ReceiveData(SPI1);

#endif
    return (data_rx);
}

// ISR Example
// static int index = 0;
// static uint8_t RecvData[30] = {0x00};
// static uint8_t RecvLen[30]  = {0x00};
// void EXTI15_10_IRQHandler(void)
// {
//     if(EXTI_GetITStatus(EXTI_Line11) != RESET)
//     {      
//         EXTI_ClearITPendingBit(EXTI_Line11);
//         RecvLen[index] = CC1101_PacketRecv_ISR(RecvData, sizeof(RecvData));
//         CC1101_StrobeSend(STROBE_SRX);
//     }     
// } 

#endif // MCSU_TYPE == MCU_STM32