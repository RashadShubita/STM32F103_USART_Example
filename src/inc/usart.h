/*
 * usart.h
 *
 *  Created on: Dec 12, 2018
 *      Author: shubi
 */

#ifndef INC_USART_H_
#define INC_USART_H_

#include "stm32f10x.h"
#include "gpio.h"
#include <math.h>

#define Baud_Rate       115200
#define With_DMA        1
#define Without_DMA     0
#define Yes             1
#define No              0

/**
 * @brief   USART initialization function
 * @note    F_CK Input clock to the peripheral(PCLK1[APB1] for USART2, 3, 4, 5 or PCLK2[APB2] for USART1) & always  over-sampling by 16
 * @param   USARTx:       where x=1 ..3
 *          Is_With_DMA:  With_DMA
 *                        Without_DMA
 *          F_CK:         Input clock to the peripheral in Hz
 * @retval  None
 */
void USART_Init(USART_TypeDef *USARTx, uint8_t Is_With_DMA, uint32_t F_CK );

/**
 * @brief   Enable USART transmitter and receiver
 * @note
 * @param   USARTx ,where x=1 ..3
 * @retval  None
 */
void USART_Enable(USART_TypeDef *USARTx);

/**
 * @brief   Char transmit
 * @note
 * @param   USARTX, char
 * @retval  None
 */
void USART_Send_Char(USART_TypeDef *USARTx, uint16_t Value);

/**
 * @brief   String transmit
 * @note
 * @param   USARTX, str
 * @retval  None
 */
void USART_Send_String(USART_TypeDef *USARTx,const char *str);

/**
 * @brief   IRQ callback function
 * @note
 * @param   None
 * @retval  None
 */
void USART1_IRQ_Callback(void);

/**
 * @brief   USART1 transmit and receive data
 * @note
 * @param   None
 * @retval  None
 */
void USART1_Process(void);

#endif /* INC_USART_H_ */
