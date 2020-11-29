/**
  ******************************************************************************
  * @file    term.c
  * @author  Dominik Muth          
  * @brief   Configuration and Usage of the terminal Program         
	* @date    2020-08-27
  ******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/

#include "term.h"
#include "uart.h"
#include "main.h"
#include "ctrl_vars.h"

/* Defines -------------------------------------------------------------------*/


#define  U16_HIGH_BYTE  0x0000FF00
#define  U16_LOW_BYTE   0x000000FF

/* Macros --------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

extern uint32_t state;					/* State of the System */
extern uint32_t adcVal;         /* ADC Value */
extern uint32_t batVolt;        /* Battery Voltage */

intern_var_t sInternVar[] = {
    { .nr = 0u, .value = &state },      /* State of the System */
    { .nr = 1u, .value = &adcVal },     /* ADC-Value */
    { .nr = 2u, .value = &batVolt },    /* Battery Voltage */
};

char termTx[512];                           /* Terminal Characters to transfer  */

static char os_name_ver[16] = OS_NAME_VER;  /* OS-Name and Version */

/* Buffer used for reception */
uint8_t aRxBuffer[255];

/* Functions -----------------------------------------------------------------*/

void
	InitTerm
		(void)
{
}

/******************************************************************************/

void
	CycleTerm
		(void)
{
	static unsigned char cHB = 48;
	static uint32_t tmpValue = 0u;

//	/*** Start receiving data, if not started yet ***/
// 	if (HAL_UART_GetState(&hUart1) != HAL_UART_STATE_BUSY_RX)
//	{
//  	if(HAL_UART_Receive_IT(&hUart1, (uint8_t *)aRxBuffer, 4u) != HAL_OK)
//  	{
//   	 	/* Transfer error in reception process */
//    	Error_Handler();
//  	}
//	}

	/*** Send new values ***/

	/* Byte 0: Start of Transmission */
	termTx[0] = 0xFF; 

	/* Byte 1: Heartbeat */
	termTx[1] = cHB++;
	if (cHB > 57u) { cHB = 48u; }

	/* Bytes 2-18: OS-Name and Version */
	for (int i=2; i<18; i++)
	{
		termTx[i] = os_name_ver[i-2];
	}

	/* Bytes 19-XX: Variables in 2-Byte Blocks */
	int i = 0;
	while (i<2*NUM_CV)
	{
		tmpValue = (*(sInternVar[i/2].value)) * sCtrlVar[i/2].tcMax / sCtrlVar[i/2].iMax;     /* scale value to readable units in terminal */
		termTx[18 + i] = (char) ((tmpValue & U16_HIGH_BYTE) >> 8u);
		i++;
		termTx[18 + i] = (char)  (tmpValue & U16_LOW_BYTE );
		i++;
	}

	/* send terminal data */
	TransmitUart(termTx, (18u + 2u*NUM_CV));
}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @retval None
  */
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
//{
//	unsigned short uReceived;
//	unsigned char varNr;
//	if (aRxBuffer[0] == 0xFF) /* Start of Receive */
//	{
//		varNr = aRxBuffer[1];
//		uReceived = (unsigned short) ((aRxBuffer[2] << 8u) | aRxBuffer[3]);
//		if (uReceived <= sCtrlVar[varNr].iMax && uReceived >= sCtrlVar[varNr].iMin)
//		{
//			*(sInternVar[varNr].value) = (uReceived*sInternVar[varNr].iMax)/sInternVar[varNr].tcMax; 
//		}
//	}
//}

/* END ************************************************************************/
