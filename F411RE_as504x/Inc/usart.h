/**
  ******************************************************************************
  * File Name          : USART.h
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __usart_H
#define __usart_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"

/* USER CODE BEGIN Includes */
 /* General Inclusion */
 #include <stdio.h>
 #include <stdlib.h>
 #include <string.h>
 #include <stdbool.h>
 #include <inttypes.h>
/* USER CODE END Includes */

extern UART_HandleTypeDef huart2;

/* USER CODE BEGIN Private defines */

#define UartInst    huart2.Instance
/* USART Definition */
#define UART_BUFSIZE            128     /* Buffer size MUST Takes power of 2(64,128,256,512...) */
#define UART_INTERRUPT_MODE     /* If u want polling mode, uncomment this */
#define UART_NOBLOCK_RECV       1           /* Set 1 to non-blocking receive on polling mode */

/* General Definition */
#define countof(a)              (sizeof(a) / sizeof(*(a)))

/* USER CODE END Private defines */

extern void _Error_Handler(char *, int);

void MX_USART2_UART_Init(void);

/* USER CODE BEGIN Prototypes */
/* Funcion Prototypes */
extern void putch(uint8_t c);
extern uint8_t getch(void);
extern uint8_t keypressed(void);
extern void cputs(char *s);
extern void cgets(char *s, int bufsize);

extern void UART_Callback(void);

/* Structs of UART(This is Based on AVRX uC Sample!!!) */
/* @brief USART transmit and receive ring buffer. */
typedef struct USART_Buffer
{
    /* @brief Receive buffer. */
    volatile uint8_t RX[UART_BUFSIZE];
    /* @brief Transmit buffer. */
    volatile uint8_t TX[UART_BUFSIZE];
    /* @brief Receive buffer head. */
    volatile unsigned int RX_Head;
    /* @brief Receive buffer tail. */
    volatile unsigned int RX_Tail;
    /* @brief Transmit buffer head. */
    volatile unsigned int TX_Head;
    /* @brief Transmit buffer tail. */
    volatile unsigned int TX_Tail;
} USART_Buffer_t;

extern USART_Buffer_t USARTx_Buf;
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
