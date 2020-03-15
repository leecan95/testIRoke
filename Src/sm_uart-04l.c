/*
 * sm_uart-04l.c
 *
 *  Created on: Mar 6, 2020
 *      Author: VHT
 */

#include "sm_uart-04l.h"
#include "stm32f4xx_hal.h"

UART_HandleTypeDef *huart_sm_uart_04l;
static uint8_t rxBuffer[32];

amphenol amphenol1;


void SM_UART_04L_Init(UART_HandleTypeDef * huart_handler) {
	huart_sm_uart_04l = huart_handler;
}


void startToRevSM04L(void) {
	HAL_UART_Receive_DMA(huart_sm_uart_04l, rxBuffer, sizeof(rxBuffer));

}


__weak void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */
  if ( rxBuffer[0] == 'B' && rxBuffer[1] == 'M') {

	  amphenol1.PM1_Standard = rxBuffer[4]*256 + rxBuffer[5];
	  amphenol1.PM2p5_Standard = rxBuffer[6]*256 + rxBuffer[7];
	  amphenol1.PM10_Standard = rxBuffer[8]*256 + rxBuffer[9];

	  amphenol1.PM1_Environment = rxBuffer[10]*256 + rxBuffer[11];
	  amphenol1.PM2p5_Environment = rxBuffer[12]*256 + rxBuffer[13];
	  amphenol1.PM10_Environment = rxBuffer[14]*256 + rxBuffer[15];

	  printf("PM1_Standard = %d\n",amphenol1.PM1_Standard);
	  printf("PM2p5_Standard = %d\n",amphenol1.PM2p5_Standard);
	  printf("PM10_Standard = %d\n",amphenol1.PM10_Standard);

	  printf("PM1_Environment = %d\n",amphenol1.PM1_Environment);
	  printf("PM2p5_Environment = %d\n",amphenol1.PM2p5_Environment);
	  printf("PM10_Environment = %d\n",amphenol1.PM10_Environment);
  }

  for (int i = 0; i<32 ; i++) {
	  rxBuffer[i] = 0;
  }

}

