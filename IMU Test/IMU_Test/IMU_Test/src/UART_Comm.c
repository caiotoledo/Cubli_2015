/*
 * UART_Comm.c
 *
 * Created: 19/09/2015 16:25:59
 *  Author: Caio
 */ 

#include <asf.h>
#include "UART_Comm.h"

uint8_t keys[100];
uint8_t uc_char = 0;
static uint8_t cont_char = 0;
volatile uint8_t uc_flag = 0;

/**
 * \brief Configure the console UART.
 */
void configure_console(void){
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CHAR_LENGHT,
		.stopbits = STOPBITS,
		.paritytype = PARITY,
	};

	/* Configure console UART. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(UART0, &uart_serial_options);
	
	NVIC_DisableIRQ(UART0_IRQn);
	NVIC_ClearPendingIRQ(UART0_IRQn);
	NVIC_SetPriority(UART0_IRQn, 1);
	NVIC_EnableIRQ(UART0_IRQn);
	
	uart_enable_interrupt(UART0, UART_IER_RXRDY);
}

void clear_keys(void){
	for (unsigned int i = 0; i < sizeof(keys); i++){
		keys[i] = 0;
	}
}

void UART0_Handler (void){	
	uint32_t ul_status;
	ul_status = uart_get_status(UART0);
	
	if (ul_status & UART_SR_RXRDY){
		uart_read(UART0, &uc_char);
		if (cont_char == 0){
			clear_keys();
		}
		//Fim do comando, character "Enter"
		if (uc_char == 13){
			cont_char = 0;
			uc_flag = 1;
		} 
		//Implementando Backspace:
		else if ( uc_char == 8 ){
			keys[--cont_char] = 0;
		}
		else {
			keys[cont_char++] = uc_char;
		}
	}
}

void shift_left (uint8_t *items, uint8_t size){
	for (unsigned int k = 0; k < size; k++){
		items[k] = items[k+1];
	}
}