/**
  ******************************************************************************
  * @file    hal/uart.c
  * @author  Dominik Muth          
  * @brief   Configuration and Usage of the serial UART communication
	* @date    2020-07-04
  ******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/

#include "uart.h"
#include "stm32f0xx_hal.h"

/* Defines -------------------------------------------------------------------*/

/* Macros --------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/
static char* ptxBuffer;
static uint32_t txBSize = 0u;
static uint32_t txIdx = 0u;
static uint32_t xBusy = 0u;

/* Functions -----------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
void
	InitUart
		(void)
{
  /* Enable Uart Clock */
  __HAL_RCC_USART1_CLK_ENABLE();

  /* --> Pin-Configuration in main.c <-- */

  /* Configure Baud-Rate */
  /* 	f_CK = 32MHz, Oversampling = 16, Baudrate = 9600 ==> BRR = 32000000/9600 = 3333 */
  USART1->BRR = 3333u;

  /* CR1: Odd Parity, Parity Enabled, 8 Data Bits (9 Bits - 1 Parity = 8 Bit), Oversampling 16 */
  USART1->CR1 = (USART_CR1_M | USART_CR1_PS | USART_CR1_PCE );

  /* CR2: 1 stop bit */
  USART1->CR2 = 0u;

  /* CR1: Enable Uart, Receiver Enable, Transmitter Enable */
  USART1->CR1 |= (USART_CR1_UE | USART_CR1_RE | USART_CR1_TE);

  NVIC_EnableIRQ(USART1_IRQn);
  NVIC_SetPriority(USART1_IRQn, 2);
}

/*----------------------------------------------------------------------------*/
void
	TransmitUart
		(char text[], uint32_t size)
{
	if (xBusy == 0u)  // No new transmission, if there is still one in progress
	{
		xBusy = 1u;
		ptxBuffer = &text[0];
		txBSize = size;

		/* CR1: Enable Interrupt for empty Transmit register */
		USART1->CR1 |= USART_CR1_TXEIE;
	}
}

/*----------------------------------------------------------------------------*/
void
	USART1_IRQHandler
		(void)
{
	/* Transmit-Interrupt? */
	if ( (USART1->ISR & USART_ISR_TXE) > 0u)
	{
		/* do transmit stuff */
		if (txIdx < txBSize)
		{
			USART1->TDR = ptxBuffer[txIdx];
			txIdx++;
		}
		else
		{
		  CLEAR_BIT(USART1->CR1, USART_CR1_TXEIE);  // Stop interrupt, if transmission is complete
		  txIdx = 0u;
		  xBusy = 0u;
		}
	}
	else
	{
		/* do receive stuff */
	}
}
/* END ************************************************************************/
