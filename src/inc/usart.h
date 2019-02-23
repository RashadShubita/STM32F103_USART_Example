/*******************************************************************************
 * @file    usart.h
 * @author  Rashad Shubita
 * @email   shubitarashad@gmail.com
 * @date    23.02.2019
 *
 * @brief   USART configuration header file
 * @note
 *
@verbatim
Copyright (C) 2019, Rashad Shubita

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or any later version.

This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program.  If not, see <http://www.gnu.org/licenses/>.
@endverbatim
*******************************************************************************/

#ifndef INC_USART_H_
#define INC_USART_H_

#include "stm32f10x.h"
#include "gpio.h"
#include <math.h>

#define Yes             1
#define No              0

/**
 * @brief   USART1 GPIO initialization function
 * @note    PA9 -> USART1_TX, PA10 -> USART1_RX
 * @param   None
 * @retval  None
 */
void USART1_GPIO_Init(void);

/**
 * @brief   USART BRR value calculation
 * @note    F_CK Input clock to the peripheral(PCLK1[APB1] for USART2, 3, 4, 5 or PCLK2[APB2] for USART1) & always  over-sampling by 16
 * @param   Baud_Rate:    Desired Baud Rate value
 *          F_CK:         Input clock to the peripheral in Hz
 * @retval  Value of BRR
 */
uint16_t Cal_USART_BRR_Val(uint32_t Baud_Rate, uint32_t F_CK);

/**
 * @brief   USART initialization function
 * @note    None
 * @param   BRR_Val:     Can be calculated using Cal_USART_BRR_Val function
 * @retval  None
 */
void USART1_Init(uint16_t BRR_Val);

/**
 * @brief   Enable USART transmitter and receiver
 * @note
 * @param   USARTx ,where x=1 ..3
 * @retval  None
 */
void USART_Enable(USART_TypeDef *USARTx);

/**
 * @brief   String transmit
 * @note
 * @param   str, size
 * @retval  None
 */
void strTransmit(const char * str, uint8_t size);

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
