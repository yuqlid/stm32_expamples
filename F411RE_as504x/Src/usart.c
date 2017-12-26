/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */
static USART_Buffer_t* pUSART_Buf;
USART_Buffer_t USARTx_Buf;
/* USER CODE END 0 */

UART_HandleTypeDef huart2;

/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 230400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();
  
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX 
    */
    GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */
    /* Init Ring Buffer */
    pUSART_Buf = &USARTx_Buf;
    USARTx_Buf.RX_Tail = 0;
    USARTx_Buf.RX_Head = 0;
    USARTx_Buf.TX_Tail = 0;
    USARTx_Buf.TX_Head = 0;

    /* Enable UART Receive interrupts */
    __HAL_UART_ENABLE_IT(uartHandle, UART_IT_RXNE);
    //UartInst->CR1 |= USART_CR1_RXNEIE;
  /* USER CODE END USART2_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();
  
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX 
    */
    HAL_GPIO_DeInit(GPIOA, USART_TX_Pin|USART_RX_Pin);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

/**************************************************************************/
/*!
    Check UART TX Buffer Empty.
*/
/**************************************************************************/
bool USART_TXBuffer_FreeSpace(USART_Buffer_t* USART_buf)
{
    /* Make copies to make sure that volatile access is specified. */
    unsigned int tempHead = (USART_buf->TX_Head + 1) & (UART_BUFSIZE-1);
    unsigned int tempTail = USART_buf->TX_Tail;

    /* There are data left in the buffer unless Head and Tail are equal. */
    return (tempHead != tempTail);
}

/**************************************************************************/
/*!
    Put Bytedata with Buffering.
*/
/**************************************************************************/
bool USART_TXBuffer_PutByte(USART_Buffer_t* USART_buf, uint8_t data)
{

    unsigned int tempTX_Head;
    bool TXBuffer_FreeSpace;

    TXBuffer_FreeSpace = USART_TXBuffer_FreeSpace(USART_buf);


    if(TXBuffer_FreeSpace)
    {
        tempTX_Head = USART_buf->TX_Head;

        __disable_irq();
        USART_buf->TX[tempTX_Head]= data;
        /* Advance buffer head. */
        USART_buf->TX_Head = (tempTX_Head + 1) & (UART_BUFSIZE-1);
        __enable_irq();

        /* Enable TXE interrupt. */
        UartInst->CR1 |= USART_CR1_TXEIE;
    }
    return TXBuffer_FreeSpace;
}

/**************************************************************************/
/*!
    Check UART RX Buffer Empty.
*/
/**************************************************************************/
bool USART_RXBufferData_Available(USART_Buffer_t* USART_buf)
{
    /* Make copies to make sure that volatile access is specified. */
    unsigned int tempHead = pUSART_Buf->RX_Head;
    unsigned int tempTail = pUSART_Buf->RX_Tail;

    /* There are data left in the buffer unless Head and Tail are equal. */
    return (tempHead != tempTail);
}

/**************************************************************************/
/*!
    Get Bytedata with Buffering.
*/
/**************************************************************************/
uint8_t USART_RXBuffer_GetByte(USART_Buffer_t* USART_buf)
{
    uint8_t ans;

    __disable_irq();
    ans = (pUSART_Buf->RX[pUSART_Buf->RX_Tail]);

    /* Advance buffer tail. */
    pUSART_Buf->RX_Tail = (pUSART_Buf->RX_Tail + 1) & (UART_BUFSIZE-1);

    __enable_irq();

    return ans;
}

/**************************************************************************/
/*!
    High Level function.
*/
/**************************************************************************/
/* Send 1 character */
inline void putch(uint8_t data)
{
#if defined(UART_INTERRUPT_MODE)
    /* Interrupt Version */
    while(!USART_TXBuffer_FreeSpace(pUSART_Buf));
    USART_TXBuffer_PutByte(pUSART_Buf,data);
#else
    /* Polling version */
    while (!(UartInst->SR & USART_SR_TXE));
    UartInst->DR = data;
#endif
}

/**************************************************************************/
/*!
    High Level function.
*/
/**************************************************************************/
/* Receive 1 character */
uint8_t getch(void)
{
#if defined(UART_INTERRUPT_MODE)
    if (USART_RXBufferData_Available(pUSART_Buf)) return USART_RXBuffer_GetByte(pUSART_Buf);
    else return false;
#else
    /* Polling version */
#if (UART_NOBLOCK_RECV == 1)
    if ((UartInst->SR & USART_SR_RXNE)) return (uint8_t)(UartInst->DR);
    else return false;
#else
    while (!(UartInst->SR & USART_SR_RXNE));
        return (uint8_t)(UartInst->DR);
#endif
#endif
}

/**************************************************************************/
/*!
    High Level function.
*/
/**************************************************************************/
/* Return 1 if key pressed */
uint8_t keypressed(void)
{
#if defined(UART_INTERRUPT_MODE)
    return (USART_RXBufferData_Available(pUSART_Buf));
#else
    return (UartInst->SR & USART_SR_RXNE);
#endif
}

/**************************************************************************/
/*!
    High Level function.
*/
/**************************************************************************/
/* Send a string */
void cputs(char *s)
{
    while (*s)
    putch(*s++);
}

/**************************************************************************/
/*!
    High Level function.
*/
/**************************************************************************/
/* Receive a string, with rudimentary line editing */
void cgets(char *s, int bufsize)
{
    char *p;
    int c;

    memset(s, 0, bufsize);

    p = s;

    for (p = s; p < s + bufsize-1;)
    {
        /* 20090521Nemui */
        do{
            c = getch();
        }while(c == false);
        /* 20090521Nemui */
        switch (c)
        {
            case '\r' :
            case '\n' :
                putch('\r');
                putch('\n');
                *p = '\n';
            return;

            case '\b' :
                if (p > s)
                {
                  *p-- = 0;
                  putch('\b');
                  putch(' ');
                  putch('\b');
                }
            break;

            default :
                putch(c);
                *p++ = c;
            break;
        }
    }
    return;
}

void UART_Callback(void){

    uint32_t IntStat = UartInst->SR;

    if(IntStat & USART_SR_RXNE)
    {
        /* Advance buffer head. */
        unsigned int tempRX_Head = ((&USARTx_Buf)->RX_Head + 1) & (UART_BUFSIZE-1);

        /* Check for overflow. */
        unsigned int tempRX_Tail = (&USARTx_Buf)->RX_Tail;
        uint8_t data =  UartInst->DR;

        if (tempRX_Head == tempRX_Tail) {
            /* Overflow MAX size Situation */
            /* Disable the UART Receive interrupt */
            UartInst->CR1 &= ~(USART_CR1_RXNEIE);
        }else{
            (&USARTx_Buf)->RX[(&USARTx_Buf)->RX_Head] = data;
            (&USARTx_Buf)->RX_Head = tempRX_Head;
        }
    }

    if(IntStat & USART_SR_TXE)
    {
        /* Check if all data is transmitted. */
        unsigned int tempTX_Tail = (&USARTx_Buf)->TX_Tail;
        if ((&USARTx_Buf)->TX_Head == tempTX_Tail){
            /* Overflow MAX size Situation */
            /* Disable the UART Transmit interrupt */
            UartInst->CR1 &= ~(USART_CR1_TXEIE);
        }else{
            /* Start transmitting. */
            uint8_t data = (&USARTx_Buf)->TX[(&USARTx_Buf)->TX_Tail];
            UartInst->DR = data;

            /* Advance buffer tail. */
            (&USARTx_Buf)->TX_Tail = ((&USARTx_Buf)->TX_Tail + 1) & (UART_BUFSIZE-1);
        }
    }
}
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
