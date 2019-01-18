/*
 * usart.c
 *
 *  Created on: Dec 12, 2018
 *      Author: Rasahd Shubita
 */

#include "usart.h"

/* Private typedef -----------------------------------------------------------*/
/**
 * @brief   USART1 states definition
 */
typedef enum
{
  USART1_IDLE,
  USART1_WAIT_FOR_RESPONCE,
  USART1_ASK_FOR_NAME,
  USART1_WAIT_FOR_NAME,
  USART1_WAIT_FOR_COMMAND,
} USART1_StateType;

/**
 * @brief   USART1 IRQ status definition
 */
typedef enum
{
  USART1_NO_IRQ,
  USART1_CHAR_RECEIVED,
  USART1_PARITY_ERROR,
} USART1_IRQStatusType;

/**
 * @brief   Return type
 */
typedef enum
{
  STR_NOT_EQUAL,
  STR_EQUAL
} strCmpReturnType;

/* Private define ------------------------------------------------------------*/
/**
 * @brief   Maximum USART reception buffer length
 */
#define MAX_BUFFER_LENGTH                     ((uint32_t) 200u)

/* Private variables constants -----------------------------------------------*/
/**
 * @brief   USART1 messages to be transmitted
 */
static const char hello_world[]        = "Hello World!";
static const char ask_for_name[]       = "What is your name?";
static const char hi[]                 = "Hi,";
static const char ask_for_command[]    = "Please, send command";
static const char ask_for_command_ex[] = "Action[turn_on / turn_off] Led[green_led / red_led]";
static const char turn_on_green_led[]  = "turn_on green_led";
static const char turn_on_red_led[]    = "turn_on red_led";
static const char turn_off_green_led[] = "turn_off green_led";
static const char turn_off_red_led[]   = "turn_off red_led";
static const char done[]               = "Done";
static const char wrong_command[]      = "Wrong Command";
static const char parity_error[]       = "Parity Error";

/* Private variables ---------------------------------------------------------*/
/**
 * @brief   USART1 current state
 */
static USART1_StateType currentState = USART1_IDLE;

/**
 * @brief   USART1 current IRQ status
 */
static USART1_IRQStatusType currentIRQStatus = USART1_NO_IRQ;

/**
 * @brief   USART1 last char recieved
 */
static char RxChar = 0;

/**
 * @brief   USART1 message buffer
 */
static char RxBuffer[MAX_BUFFER_LENGTH + 1];

/**
 * @brief   USART1 message length
 */
static uint8_t RxMessageLength = 0;


/**
 * @brief   Compare two strings
 * @note    take the size of the predefined string
 * @param   str1, str2, size
 * @retval  strCmpReturnType
 */
static strCmpReturnType strCmp(const char * str1, const char * str2,
    const uint8_t size)
{
  /* Compare status */
  strCmpReturnType cmpStatus = STR_EQUAL;

  /* Check null pointers */
  if((NULL != str1) && (NULL != str2))
  {
    /* Start comparing */
    for (int idx = 0; idx < size; idx++)
    {
      /* When not equal set the return status */
      if(str1[idx] != str2[idx])
      {
        cmpStatus = STR_NOT_EQUAL;
      }
      else
      {
        /* Do nothing */
      }
    }
  }
  else
  {
    /* Null pointers, do nothing */
  }
  return cmpStatus;
}

/**
 * @brief   IRQ callback function
 * @note
 * @param   None
 * @retval  None
 */
void USART1_IRQ_Callback(void)
{
  /* Check if parity error detected */
  if((USART1->SR & USART_SR_PE) == USART_SR_PE)
  {
    while((USART1->SR & USART_SR_RXNE) != USART_SR_RXNE)
    {
      /* Wait for RXNE flag to be set */
    }

    /* Read data register to clear parity error */
    USART1->DR;

    /* Set parity error */
    currentIRQStatus = USART1_PARITY_ERROR;
  }
  else
  {
    /* No parity error */
  }

  /* Check USART receiver */
  if((USART1->SR & USART_SR_RXNE) == USART_SR_RXNE)
  {
    /* Read character */
    RxChar = USART1->DR;

    /* Set IRQ status */
    currentIRQStatus = USART1_CHAR_RECEIVED;
  }
  else
  {
    /* No new data received */
  }
}

/**
 * @brief   String receive
 * @note
 * @param   None
 * @retval  None
 */
static void strReceive(void)
{
  /* Local string buffer */
  static char RxLocalBuffer[MAX_BUFFER_LENGTH];

  /* Current reception index */
  static int RxIndex = 0;

  /* Check for end-of-line condition */
  if (RxChar == '\0')
  {
    /* Check if string data was received before */
    if (RxIndex != 0)
    {
      /* Copy string data from local buffer */
      for(int idx = 0; idx < RxIndex; idx++)
      {
        RxBuffer[idx] = RxLocalBuffer[idx];
      }

      /* Add terminating NULL at the end */
      RxBuffer[RxIndex] = 0;

      /* Set message length */
      RxMessageLength = RxIndex + 1;

      /* Reset current index */
      RxIndex = 0;
    }
    else
    {
      /* String buffer already empty */
    }
  }
  else
  {
    /* Char data was received, Check for buffer overflow */
    if (MAX_BUFFER_LENGTH == RxIndex)
    {
      /* Reset the reception buffer */
      RxIndex = 0;
    }
    else
    {
      /* Do nothing, No overflow */
    }

    /* Copy char data into string buffer */
    RxLocalBuffer[RxIndex] = RxChar;

    /* Increment current index for the next char reception */
    RxIndex++;
  }
}


/**
 * @brief   String transmit
 * @note
 * @param   USARTX, str
 * @retval  None
 */
void USART_Send_String(USART_TypeDef *USARTx,const char *str)
{
 /* Check null pointers */
 if(NULL != str)
  {
   while(*str != 0)
	{
	  USART_Send_Char(USARTx, *str++);
	}
   USART_Send_Char(USARTx, *str);
  }
 else
 {
   /* Null pointers, do nothing */
 }
}

/**
 * @brief   Char transmit
 * @note
 * @param   USARTX, char
 * @retval  None
 */
void USART_Send_Char(USART_TypeDef *USARTx, uint16_t Value)
{
    /* Check USART status register */
    while(!(USARTx->SR & USART_SR_TXE))
    {
      /* Wait for transmission buffer empty flag */
    }
	USARTx ->DR = Value;
}

/**
 * @brief   USART initialization function
 * @note    F_CK Input clock to the peripheral(PCLK1[APB1] for USART2, 3, 4, 5 or PCLK2[APB2] for USART1) & always  over-sampling by 16
 * @param   USARTx:       where x=1 ..3
 *          Is_With_DMA:  With_DMA
 *                        Without_DMA
 *          F_CK:         Input clock to the peripheral in Hz
 * @retval  None
 */
void USART_Init(USART_TypeDef *USARTx, uint8_t Is_With_DMA, uint32_t F_CK )
{

 double USARTDIV=0;
 uint8_t Fraction;

/* USART GPIO configuration -------------------------------------------------------*/
 if(USARTx == USART1)
 {
  /* Configuration GPIOA TX & RX based on Reference manual Table 24 & Table 54	*/
	 GPIO_USART1_Init();

 }

 else if(USARTx == USART3)
 {
  /* Configuration GPIOB TX & RX based on Reference manual Table 24 & Table 52	*/
	 GPIO_USART2_Init();
 }


/* USART configuration -------------------------------------------------------*/
   /*Enable USART1 clock */
   if(USARTx == USART1)
     RCC ->APB2ENR   |=  RCC_APB2ENR_USART1EN;
   /* Enable USART3 clock */
   else if(USARTx == USART3)
    RCC ->APB1ENR   |=  RCC_APB1ENR_USART3EN;

   /* select 1 Start bit, 9 Data bits, n Stop bit  */
   USARTx ->CR1    |= USART_CR1_M;

   /* STOP bits, 00: 1 Stop bit */
   USARTx->CR2    &= ~USART_CR2_STOP;

   /* Select odd parity */
   USARTx->CR1 |= USART_CR1_PS;

   /* Enable parity control */
   USART1->CR1 |= USART_CR1_PCE;

   /* Set baud rate = 115200 Bps
    * USARTDIV = Fck / (16 * baud_rate)
    *          = 72000000 / (16 * 115200) = 39.0625
    *
    * DIV_Fraction = 16 * 0.0625 = 1 = 0x1
    * DIV_Mantissa = 39 = 0x27
    *
    * BRR          = 0x271 */
  USARTDIV    = ( F_CK/(Baud_Rate*16.0) );
  Fraction = round( (USARTDIV - ((uint16_t)USARTDIV) )* 16 ) ;
  if(Fraction > 15)
	 {
	    Fraction=0;
	    USARTDIV++;
	 }
  USARTx ->BRR = ( ( ((uint16_t)USARTDIV) << 4 ) + Fraction) ;

	if(Is_With_DMA == Yes)
  		/* DMA mode enabled for reception */
  		USARTx->CR3  |= USART_CR3_DMAR;

	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");
	__ASM("NOP");

}

/**
 * @brief   Enable USART transmitter and receiver
 * @note
 * @param   USARTx ,where x=1 ..3
 * @retval  None
 */
void USART_Enable(USART_TypeDef *USARTx)
{
  /* Enable USART1 */
  USARTx->CR1 |= USART_CR1_UE;

  /* Enable transmitter */
  USARTx->CR1 |= USART_CR1_TE;

  /* Enable receiver */
  USARTx->CR1 |= USART_CR1_RE;

  /* Enable reception buffer not empty flag interrupt */
  USARTx->CR1 |= USART_CR1_RXNEIE;

  /* Enable parity error interrupt */
  USARTx->CR1 |= USART_CR1_PEIE;

}


/**
 * @brief   USART1 transmit and receive data
 * @note
 * @param   None
 * @retval  None
 */
void USART1_Process(void)
{
  /* Check error status */
  switch (currentIRQStatus)
  {
    case USART1_PARITY_ERROR:
      /* Transmit parity error */
    	USART_Send_String(USART1, parity_error);

      /* Reset USART1 state */
      currentState = USART1_IDLE;

      /* Reset IRQ status */
      currentIRQStatus = USART1_NO_IRQ;
      break;

    case USART1_CHAR_RECEIVED:
      /* Receive string data */
      strReceive();

      /* Reset IRQ status */
      currentIRQStatus = USART1_NO_IRQ;
      break;

    case USART1_NO_IRQ:
      break;

    default:
      break;
  }

  /* Check current USART state */
  switch (currentState)
  {
    case USART1_IDLE:
      /* Transmit data */
    	USART_Send_String(USART1, hello_world);

      /* Go to next state */
      currentState = USART1_WAIT_FOR_RESPONCE;
      break;

    case USART1_WAIT_FOR_RESPONCE:
      /* Check if new message received */
      if(0 != RxMessageLength)
      {
        /* Reset message length */
        RxMessageLength = 0;

        /* Go to next state */
        currentState = USART1_ASK_FOR_NAME;
      }
      else
      {
        /* Nothing received yet */
      }
      break;

    case USART1_ASK_FOR_NAME:
      /* Transmit data */
    	USART_Send_String(USART1, ask_for_name);

      /* Go to next state */
      currentState = USART1_WAIT_FOR_NAME;
      break;

    case USART1_WAIT_FOR_NAME:
      /* Check if new message received */
      if(0 != RxMessageLength)
      {
        /* Transmit data */
    	  USART_Send_String(USART1, hi);
    	  USART_Send_String(USART1, RxBuffer);
    	  USART_Send_String(USART1, ask_for_command);
    	  USART_Send_String(USART1, ask_for_command_ex);

        /* Reset message length */
        RxMessageLength = 0;

        /* Go to next state */
        currentState = USART1_WAIT_FOR_COMMAND;
      }
      else
      {
        /* Nothing received yet */
      }
      break;

    case USART1_WAIT_FOR_COMMAND:
      /* Check if new message received */
      if(0 != RxMessageLength)
      {
        /* Reset message length */
        RxMessageLength = 0;

        /* String compare results */
        strCmpReturnType isMatch_01 = STR_NOT_EQUAL;
        strCmpReturnType isMatch_02 = STR_NOT_EQUAL;
        strCmpReturnType isMatch_03 = STR_NOT_EQUAL;
        strCmpReturnType isMatch_04 = STR_NOT_EQUAL;

        /* Compare with turn on green led command */
        isMatch_01 =  strCmp(turn_on_green_led, RxBuffer,
            sizeof(turn_on_green_led));

        /* Check return status */
        if(STR_EQUAL == isMatch_01)
        {
          /* Turn on green led */
          //GPIO_TurnON_LED(EVAL_GREEN_LED);

          /* Transmit data */
          USART_Send_String(USART1, done);
        }
        else
        {
          /* Compare with turn on red led command */
          isMatch_02 =  strCmp(turn_on_red_led, RxBuffer,
              sizeof(turn_on_red_led));
        }

        /* Check return status */
        if(STR_EQUAL == isMatch_02)
        {
          /* Turn on red led */
          //GPIO_TurnON_LED(EVAL_RED_LED);

          /* Transmit data */
          USART_Send_String(USART1, done);
        }
        else if(STR_NOT_EQUAL == isMatch_01)
        {
          /* Compare with turn off green led command */
          isMatch_03 =  strCmp(turn_off_green_led, RxBuffer,
              sizeof(turn_off_green_led));
        }
        else
        {
          /* Do nothing */
        }

        /* Check return status */
        if(STR_EQUAL == isMatch_03)
        {
          /* Turn off green led */
          //GPIO_TurnOFF_LED(EVAL_GREEN_LED);

          /* Transmit data */
          USART_Send_String(USART1, done);
        }
        else if((STR_NOT_EQUAL == isMatch_02)
            && (STR_NOT_EQUAL == isMatch_01))
        {
          /* Compare with turn off red led command */
          isMatch_04 =  strCmp(turn_off_red_led, RxBuffer,
              sizeof(turn_off_red_led));
        }
        else
        {
          /* Do nothing */
        }

        /* Check return status */
        if(STR_EQUAL == isMatch_04)
        {
          /* Turn off red led */
         // GPIO_TurnOFF_LED(EVAL_RED_LED);

          /* Transmit data */
        	USART_Send_String(USART1, done);
        }
        else if((STR_NOT_EQUAL == isMatch_03)
            && (STR_NOT_EQUAL == isMatch_02)
            && (STR_NOT_EQUAL == isMatch_01))
        {
          /* Transmit data */
        	USART_Send_String(USART1, wrong_command);
        }
        else
        {
          /* Do nothing */
        }
      }
      else
      {
        /* Nothing received yet */
      }
      break;

    default:
      break;
  }
}
