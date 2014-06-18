#include "CC1101.h"
#include "FIFO.h"
#include <string.h>


static void LED_Init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    gpio.GPIO_Pin   = GPIO_Pin_2; 
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);
}

static void TIMER3_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    TIM_DeInit(TIM3);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    TIM_TimeBaseStructure.TIM_Period      = 2000;
    TIM_TimeBaseStructure.TIM_Prescaler   = 7200-1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
    TIM_Cmd(TIM3, ENABLE);
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    NVIC_InitStructure.NVIC_IRQChannel            = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd         = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

static void delay(volatile uint32_t tick)
{
    while(tick--);
}

static uint8_t TransData[14] = {0x0D, 0x5A, 0x00, 0xAA, 0xAA, 0x00, 0x00, 0xBB, 0xBB, 
                        0x01, 0x34, 0x00, 0x12, 0x34};

static uint8_t RecvData[20];

static void UART_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
 
    /* Enable GPIO clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    
    /* Enable USARTy Clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    
    /* Configure USARTy Rx as input floating */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    /* Configure USARTy Tx as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    
    /* Configure USARTy */
    USART_Init(USART1, &USART_InitStructure);
 
    
    /* Configure the NVIC Preemption Priority Bits */  
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

    /* Enable the USARTy Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);  
    
    /* Enable the USARTy */
    USART_Cmd(USART1, ENABLE);
    
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    //USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
}

static void UART_SendByte(uint8_t data)
{
    /* Loop until USARTy DR register is empty */ 
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);

    /* Send one byte from USARTy to USARTz */
    USART_SendData(USART1, (uint8_t)data);
}

static void UART_Send(uint8_t* pdata, uint8_t size)
{    
    int i = 0;
    
    for(i = 0; i < size; i++)
    {
        UART_SendByte(pdata[i]);
    }
}

static uint8_t Frame[10];


extern uint8_t FramePack(uint8_t* pDataIn, uint8_t len, uint8_t* pDataOut);


extern volatile uint8_t UART_RecvLen;
extern volatile uint8_t UART_TranLen;

extern uint8_t UART_RecvData[100];
extern uint8_t UART_TranData[100];

static uint8_t UART_buf[50];

typedef struct
{
    uint16_t hp_total;
    uint16_t hp_cur;
}HP_STATUS;


FIFO_t UART_Fifo;

static uint8_t str[] = {"Hello,world\r\n"};

void main(void)
{
    uint8_t retval = 0;
    uint8_t i      = 0;
    
    HP_STATUS car_hp = {1000, 10000};
    
    FIFO_Init(&UART_Fifo, UART_TranData, sizeof(uint8_t), sizeof(UART_TranData));
    
    FIFO_Put(&UART_Fifo, &str[0]);
    FIFO_Put(&UART_Fifo, &str[1]);
    FIFO_Put(&UART_Fifo, &str[2]);
    FIFO_Put(&UART_Fifo, &str[3]);
    FIFO_Put(&UART_Fifo, &str[4]);
    FIFO_Put(&UART_Fifo, &str[5]);
    
    FIFO_Get(&UART_Fifo, &str[6]);
    FIFO_PreRead(&UART_Fifo, 0, &str[7]);
    FIFO_PreRead(&UART_Fifo, 1, &str[7]);
    FIFO_PreRead(&UART_Fifo, 2, &str[7]);
    FIFO_Get(&UART_Fifo, &str[6]);
    
    while(1);
    
    LED_Init();
    retval = CC1101_Init();    
    if(0 != retval)
    {
        while(1); // Error
    }

    //TIMER3_Init();
    UART_Init();

#if 1
    while(1)
    {
        while(!UART_RecvLen);

        if(UART_RecvData[2] == 0x00 && UART_RecvData[3] == 0x10)
        {
            car_hp.hp_cur -= 100;
            if(car_hp.hp_cur < 200)
            {
                car_hp.hp_cur = 1000;
            }
            
            Frame[0] = 0x00;
            Frame[1] = 0x00;
            Frame[2] = 0x00;
            Frame[3] = 0x10;
            memcpy(&Frame[4], &car_hp, sizeof(car_hp));

            UART_TranLen = FramePack(Frame, 8, UART_TranData);
            USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
        }
        
        UART_RecvLen = 0;
    }
#endif
    
#if 1

    //CC1101_StrobeSend(STROBE_SRX);
    //while(1);

    while(1)
    {
        //delay(0x2FFFF);
        GPIOA->ODR ^= GPIO_Pin_2;        
        retval = CC1101_PacketRecv(RecvData, sizeof(RecvData));
        if(retval != 0)
        {
            retval = FramePack(RecvData, retval, UART_buf);
            UART_Send(UART_buf, retval);
        }
    }
#else
    while(1)
    {
        delay(0x5FFFF);
        TransData[1] = 0x5A;
        GPIOA->ODR ^= GPIO_Pin_2;        
        CC1101_PacketSend(TransData, sizeof(TransData));
        
        delay(0x5FFFF);
        TransData[1] = 0xA5;
        GPIOA->ODR ^= GPIO_Pin_2;        
        CC1101_PacketSend(TransData, sizeof(TransData));
    }
#endif
    while(1);        
}