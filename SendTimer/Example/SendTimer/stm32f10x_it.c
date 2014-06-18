/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "CC1101.h"

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{

}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


static uint8_t TransData[14] = {0x0D, 0x00, 0x00, 0xAA, 0xAA, 0x00, 0x00, 0xBB, 0xBB, 
                        0x01, 0x34, 0x00, 0x12, 0x34};

void TIM3_IRQHandler()
{ 
    static int cnt = 0;
    
    if(TIM_GetITStatus(TIM3 , TIM_IT_Update) == SET)
    {
        TIM_ClearITPendingBit(TIM3 , TIM_FLAG_Update);
        GPIOA->ODR ^= GPIO_Pin_2;        
        CC1101_PacketSend(TransData, sizeof(TransData));
    }
}

static int index = 0;
static uint8_t RecvData[30] = {0x00};
static uint8_t RecvLen[30]  = {0x00};


volatile uint8_t UART_RecvLen = 0;
volatile uint8_t UART_TranLen = 0;

uint8_t UART_RecvData[100] = {0x00};
uint8_t UART_TranData[100] = {0x00};

void EXTI15_10_IRQHandler(void)
{    
    if(EXTI_GetITStatus(EXTI_Line11) != RESET)
    {      
        GPIOA->ODR ^= GPIO_Pin_2;
                
        EXTI_ClearITPendingBit(EXTI_Line11);
        
        RecvLen[index++] = CC1101_PacketRecv_ISR(RecvData, sizeof(RecvData));
        CC1101_StrobeSend(STROBE_SRX);
        
        if(index >= 20)
        {
            index  = 0;
        }
    }
}

extern uint8_t FrameUnpack(uint8_t token, uint8_t* pBuffer);

void USART1_IRQHandler(void)
{
    uint8_t Ch = 'O';
    static uint8_t buffer[] = {"hello world!\r\n" };
    static uint8_t index = 0;
    
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        /* Read one byte from the receive data register */
        //RxBuffer1[RxCounter1++] = USART_ReceiveData(USART1);
        Ch = USART_ReceiveData(USART1);
        UART_RecvLen = FrameUnpack(Ch, UART_RecvData);
        /* Dis65able the USARTy Receive interrupt */
        // USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);      
    }

    if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
    {
       if(index < UART_TranLen)
       {
           USART_SendData(USART1, UART_TranData[index++]);
       }
       else
       {
           index = 0;
           UART_TranLen = 0;
           USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
       }
    }
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
